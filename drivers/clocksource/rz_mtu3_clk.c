// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G2L MTU3 Timer Support
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 *
 * Based on sh_mtu2.c
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/sh_timer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/mfd/rz-mtu3.h>

struct rz_mtu3_clk_device;

enum mtu3_functions {
	MTU3_CLOCKSOURCE,	/* Assign clocksource function */
	MTU3_CLOCKEVENT,	/* Assign clockevent function */
};

struct rz_mtu3_clk_channel_priv {
	struct rz_mtu3_clk_device *mtu;
	unsigned int index;
	struct clock_event_device ced;
	struct clocksource cs;
	u64 total_cycles;
	raw_spinlock_t lock;
	unsigned long flags;
	bool cs_enabled;
	enum mtu3_functions function;
	struct rz_mtu3_channel *chan;
};

struct rz_mtu3_clk_device {
	struct platform_device *pdev;
	void __iomem *mapbase;
	struct clk *clk;
	struct reset_control *rstc;
	unsigned long rate;
	raw_spinlock_t lock; /* Protect the shared registers */
	struct rz_mtu3_clk_channel_priv *channels;
	unsigned int num_channels;
	bool has_clockevent;
	bool has_clocksource;
};

/* private flags */
#define FLAG_CLOCKEVENT		(1 << 0)
#define FLAG_CLOCKSOURCE	(1 << 1)
#define FLAG_REPROGRAM		(1 << 2)
#define FLAG_SKIPEVENT		(1 << 3)
#define FLAG_IRQCONTEXT		(1 << 4)

/* Macros for setting registers */
#define TIER_TGIEA		(1 << 0)


static int rz_mtu3_clk_enable(struct rz_mtu3_clk_channel_priv *ch)
{
	unsigned long periodic;
	unsigned long rate;
	int ret;

	/* enable clock */
	ret = clk_enable(ch->mtu->clk);
	if (ret) {
		dev_err(&ch->mtu->pdev->dev, "ch%u: cannot enable clock\n",
			ch->index);
		return ret;
	}

	/* make sure channel is disabled */
	rz_mtu3_disable(ch->chan);

	rate = clk_get_rate(ch->mtu->clk) / 64;
	periodic = (rate + HZ / 2) / HZ;

	/*
	 * "Periodic Counter Operation"
	 * Clear on TGRA compare match, divide clock by 64.
	 */
	if (ch->function == MTU3_CLOCKSOURCE) {
		rz_mtu3_8bit_ch_write(ch->chan, RZ_MTU3_TCR,
				      FIELD_PREP(RZ_MTU3_TCR_TPCS, 3));
		rz_mtu3_8bit_ch_write(ch->chan, RZ_MTU3_TIER, 0);
	} else if (ch->function == MTU3_CLOCKEVENT) {
		rz_mtu3_8bit_ch_write(ch->chan, RZ_MTU3_TCR,
				      RZ_MTU3_TCR_CCLR_TGRA |
				      FIELD_PREP(RZ_MTU3_TCR_TPCS, 3));
		rz_mtu3_8bit_ch_write(ch->chan, RZ_MTU3_TIOR,
				      RZ_MTU3_TIOR_OC_IOB_L_COMP_MATCH |
				      RZ_MTU3_TIOR_OC_IOA_L_COMP_MATCH);
		rz_mtu3_16bit_ch_write(ch->chan, RZ_MTU3_TGRA, periodic);
		rz_mtu3_8bit_ch_write(ch->chan, RZ_MTU3_TMDR1, RZ_MTU3_TMDR1_MD_NORMAL);
		rz_mtu3_8bit_ch_write(ch->chan, RZ_MTU3_TIER, TIER_TGIEA);
	}

	/* enable channel */
	rz_mtu3_enable(ch->chan);
	return 0;
}

static void rz_mtu3_clk_disable(struct rz_mtu3_clk_channel_priv *ch)
{
	/* disable channel */
	rz_mtu3_disable(ch->chan);
	/* stop clock */
	clk_disable(ch->mtu->clk);
	dev_pm_syscore_device(&ch->mtu->pdev->dev, false);
	pm_runtime_put(&ch->mtu->pdev->dev);
}

static int rz_mtu3_clk_start(struct rz_mtu3_clk_channel_priv *ch, unsigned long flag)
{
	int ret = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&ch->lock, flags);

	if (!(ch->flags & (FLAG_CLOCKEVENT | FLAG_CLOCKSOURCE)))
		ret = rz_mtu3_clk_enable(ch);

	if (ret)
		goto out;
	ch->flags |= flag;

	/* setup timeout if no clockevent */
out:
	raw_spin_unlock_irqrestore(&ch->lock, flags);
	return ret;
}

static irqreturn_t rz_mtu3_clk_interrupt(int irq, void *dev_id)
{
	struct rz_mtu3_clk_channel_priv *ch = dev_id;

	/* acknowledge interrupt */
	/* notify clockevent layer */
	if (ch->flags & FLAG_CLOCKEVENT)
		ch->ced.event_handler(&ch->ced);
	return IRQ_HANDLED;
}

static void rz_mtu3_clk_stop(struct rz_mtu3_clk_channel_priv *ch, unsigned long flag)
{
	unsigned long flags;
	unsigned long f;

	raw_spin_lock_irqsave(&ch->lock, flags);
	f = ch->flags & (FLAG_CLOCKEVENT | FLAG_CLOCKSOURCE);
	ch->flags &= ~flag;

	if (f && !(ch->flags & (FLAG_CLOCKEVENT | FLAG_CLOCKSOURCE)))
		rz_mtu3_clk_disable(ch);

	/* adjust the timeout to maximum if only clocksource left */

	raw_spin_unlock_irqrestore(&ch->lock, flags);
}

static struct rz_mtu3_clk_channel_priv *ced_to_rz_mtu3_clk(
				struct clock_event_device *ced)
{
	return container_of(ced, struct rz_mtu3_clk_channel_priv, ced);
}

static int rz_mtu3_clk_clock_event_shutdown(struct clock_event_device *ced)
{
	struct rz_mtu3_clk_channel_priv *ch = ced_to_rz_mtu3_clk(ced);

	if (clockevent_state_periodic(ced))
		rz_mtu3_clk_disable(ch);

	return 0;
}

static int rz_mtu3_clk_clock_event_set_periodic(struct clock_event_device *ced)
{
	struct rz_mtu3_clk_channel_priv *ch = ced_to_rz_mtu3_clk(ced);

	if (clockevent_state_periodic(ced))
		rz_mtu3_clk_disable(ch);

	dev_info(&ch->mtu->pdev->dev, "ch%u: used for periodic clock events\n",
		ch->index);
	rz_mtu3_clk_enable(ch);

	return 0;
}

static void rz_mtu3_clk_clock_event_suspend(struct clock_event_device *ced)
{
	pm_genpd_syscore_poweroff(&ced_to_rz_mtu3_clk(ced)->mtu->pdev->dev);
}

static void rz_mtu3_clk_clock_event_resume(struct clock_event_device *ced)
{
	pm_genpd_syscore_poweron(&ced_to_rz_mtu3_clk(ced)->mtu->pdev->dev);
}

static struct rz_mtu3_clk_channel_priv *cs_to_sh_mtu(struct clocksource *cs)
{
	return container_of(cs, struct rz_mtu3_clk_channel_priv, cs);
}

static u32 rz_mtu3_clk_get_counter(struct rz_mtu3_clk_channel_priv *ch)
{
	u32 v2;

	if (ch->chan->channel_number == 8)
		v2 = rz_mtu3_32bit_ch_read(ch->chan, RZ_MTU3_TCNT);
	else
		v2 = rz_mtu3_16bit_ch_read(ch->chan, RZ_MTU3_TCNT);
	return v2;
}

static u64 rz_mtu3_clk_clocksource_read(struct clocksource *cs)
{
	struct rz_mtu3_clk_channel_priv *ch = cs_to_sh_mtu(cs);
	unsigned long flags;
	u64 value;
	u32 raw;

	raw_spin_lock_irqsave(&ch->lock, flags);
	value = ch->total_cycles;
	raw = rz_mtu3_clk_get_counter(ch);
	raw_spin_unlock_irqrestore(&ch->lock, flags);

	return value + raw;
}

static int rz_mtu3_clk_clocksource_enable(struct clocksource *cs)
{
	int ret;
	struct rz_mtu3_clk_channel_priv *ch = cs_to_sh_mtu(cs);

	WARN_ON(ch->cs_enabled);
	ch->total_cycles = 0;
	ret = rz_mtu3_clk_start(ch, FLAG_CLOCKSOURCE);
	if (!ret)
		ch->cs_enabled = true;

	return ret;
}

static void rz_mtu3_clk_clocksource_disable(struct clocksource *cs)
{
	struct rz_mtu3_clk_channel_priv *ch = cs_to_sh_mtu(cs);

	WARN_ON(!ch->cs_enabled);
	rz_mtu3_clk_stop(ch, FLAG_CLOCKSOURCE);
	ch->cs_enabled = false;
}

static void rz_mtu3_clk_clocksource_suspend(struct clocksource *cs)
{
	struct rz_mtu3_clk_channel_priv *ch = cs_to_sh_mtu(cs);

	if (!ch->cs_enabled)
		return;
	rz_mtu3_clk_stop(ch, FLAG_CLOCKSOURCE);
	pm_genpd_syscore_poweroff(&ch->mtu->pdev->dev);
}

static void rz_mtu3_clk_clocksource_resume(struct clocksource *cs)
{
	struct rz_mtu3_clk_channel_priv *ch = cs_to_sh_mtu(cs);

	if (!ch->cs_enabled)
		return;
	pm_genpd_syscore_poweron(&ch->mtu->pdev->dev);
	rz_mtu3_clk_start(ch, FLAG_CLOCKSOURCE);
}

static void rz_mtu3_clk_register_clockevent(struct rz_mtu3_clk_channel_priv *ch,
			const char *name)
{
	struct clock_event_device *ced = &ch->ced;

	ced->name = name;
	ced->features = CLOCK_EVT_FEAT_PERIODIC;
	ced->rating = 200;
	ced->cpumask = cpu_possible_mask;
	ced->set_state_shutdown = rz_mtu3_clk_clock_event_shutdown;
	ced->set_state_periodic = rz_mtu3_clk_clock_event_set_periodic;
	ced->suspend = rz_mtu3_clk_clock_event_suspend;
	ced->resume = rz_mtu3_clk_clock_event_resume;
	dev_info(&ch->mtu->pdev->dev, "ch%u: used for clock events\n",
		ch->index);
	clockevents_register_device(ced);
}

static int rz_mtu3_clk_register_clocksource(struct rz_mtu3_clk_channel_priv *ch,
			const char *name)
{
	struct clocksource *cs = &ch->cs;

	cs->name = name;
	cs->rating = 126;
	cs->read = rz_mtu3_clk_clocksource_read;
	cs->enable = rz_mtu3_clk_clocksource_enable;
	cs->disable = rz_mtu3_clk_clocksource_disable;
	cs->suspend = rz_mtu3_clk_clocksource_suspend;
	cs->resume = rz_mtu3_clk_clocksource_resume;
	cs->mask = 0xffff;
	cs->flags = CLOCK_SOURCE_IS_CONTINUOUS;
	dev_info(&ch->mtu->pdev->dev, "ch%u: used as clock source\n",
		ch->index);
	clocksource_register_hz(cs, ch->mtu->rate);
	return 0;
}

static int rz_mtu3_clk_register(struct rz_mtu3_clk_channel_priv *ch,
			       const char *name)
{
	if (ch->function == MTU3_CLOCKEVENT)
		rz_mtu3_clk_register_clockevent(ch, name);
	else if (ch->function == MTU3_CLOCKSOURCE)
		rz_mtu3_clk_register_clocksource(ch, name);
	return 0;
}

static int rz_mtu3_clk_setup_channel(struct rz_mtu3_clk_channel_priv *ch,
				    unsigned int index,
				    struct rz_mtu3_clk_device *mtu, struct rz_mtu3 *ddata)
{
	char name[6];
	int irq;
	int ret;

	ch->mtu = mtu;

	sprintf(name, "tgia%u", index);
	irq = platform_get_irq_byname(ddata->pdev, name);
	if (irq < 0) {
		/* Skip channels with no declared interrupt. */
		return 0;
	}

	ret = request_irq(irq, rz_mtu3_clk_interrupt,
			  IRQF_TIMER | IRQF_IRQPOLL | IRQF_NOBALANCING,
			  dev_name(&ch->mtu->pdev->dev), ch->chan);
	if (ret) {
		dev_err(&ch->mtu->pdev->dev, "ch%u: failed to request irq %d\n",
			index, irq);
		return ret;
	}

	ch->index = index;
	ddata->channels[index].channel_number = index;

	switch (index) {
	case RZ_MTU3_CHAN_0:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_0];
		break;
	case RZ_MTU3_CHAN_1:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_1];
		break;
	case RZ_MTU3_CHAN_2:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_2];
		break;
	case RZ_MTU3_CHAN_3:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_3];
		break;
	case RZ_MTU3_CHAN_4:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_4];
		break;
	case RZ_MTU3_CHAN_5:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_5];
		break;
	case RZ_MTU3_CHAN_6:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_6];
		break;
	case RZ_MTU3_CHAN_7:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_7];
		break;
	case RZ_MTU3_CHAN_8:
		ch->chan = &ddata->channels[RZ_MTU3_CHAN_8];
		break;
	default:
		break;
	}
	ch->chan->dev = &ch->mtu->pdev->dev;
	return rz_mtu3_clk_register(ch, dev_name(&mtu->pdev->dev));
}

static int rz_mtu3_clk_setup(struct rz_mtu3_clk_device *mtu,
			struct platform_device *pdev, struct rz_mtu3 *ddata)
{
	int ret, i;

	mtu->pdev = pdev;
	raw_spin_lock_init(&mtu->lock);

	/* Get hold of clock. */
	mtu->clk = ddata->clk;
	if (IS_ERR(mtu->clk)) {
		dev_err(&mtu->pdev->dev, "cannot get clock\n");
		return PTR_ERR(mtu->clk);
	}

	ret = clk_prepare(mtu->clk);
	if (ret < 0)
		goto err_clk_put;

	ret = clk_enable(mtu->clk);
	if (ret < 0)
		goto err_clk_unprepare;

	mtu->rate = clk_get_rate(mtu->clk) / 64;
	clk_disable(mtu->clk);
	/* Allocate and setup the channels. */
	mtu->has_clockevent = true;
	mtu->has_clocksource = false;
	mtu->num_channels = RZ_MTU_NUM_CHANNELS;

	mtu->channels = kcalloc(mtu->num_channels, sizeof(*mtu->channels),
		GFP_KERNEL);
	if (mtu->channels == NULL) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	for (i = 8; i < mtu->num_channels; i++) {
		mtu->channels[i].index = i;
		if (!(mtu->has_clocksource)) {
			mtu->channels[i].function = MTU3_CLOCKSOURCE;
			ret = rz_mtu3_clk_setup_channel(
				&mtu->channels[i], i, mtu, ddata);
			if (ret < 0)
				goto err_unmap;
			mtu->has_clocksource = true;
		} else if (!(mtu->has_clockevent)) {
			mtu->channels[i].function = MTU3_CLOCKEVENT;
			ret = rz_mtu3_clk_setup_channel(
				&mtu->channels[i], i, mtu, ddata);
			if (ret < 0)
				goto err_unmap;
			mtu->has_clockevent = true;
		}
	}

	clk_disable(mtu->clk);

	platform_set_drvdata(pdev, mtu);

	return 0;

err_unmap:
	kfree(mtu->channels);
	iounmap(mtu->mapbase);
err_clk_unprepare:
	clk_unprepare(mtu->clk);
err_clk_put:
	clk_put(mtu->clk);
	return ret;
}

static int rz_mtu3_clk_probe(struct platform_device *pdev)
{
	struct rz_mtu3 *ddata = dev_get_drvdata(pdev->dev.parent);
	struct rz_mtu3_clk_device *mtu = platform_get_drvdata(pdev);
	int ret;

	mtu = kzalloc(sizeof(*mtu), GFP_KERNEL);
	if (mtu == NULL)
		return -ENOMEM;

	ret = rz_mtu3_clk_setup(mtu, pdev, ddata);
	if (ret) {
		kfree(mtu);
		return ret;
	}

	return 0;
}

static struct platform_driver rz_mtu3_clk_device_driver = {
	.probe		= rz_mtu3_clk_probe,
	.driver		= {
		.name	= "rz-mtu3-clk",
	},
};
module_platform_driver(rz_mtu3_clk_device_driver);
MODULE_DESCRIPTION("RZ/G2L MTU3 Timer Driver");
MODULE_LICENSE("GPL v2");
