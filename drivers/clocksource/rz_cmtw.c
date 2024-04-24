// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ Compare Match Timer W (CMTW) Support
 *
 * Copyright (C) 2024 Renesas Electronics Corporation.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/reset.h>

/* Timer Start Register */
#define CMWSTR			0x00
#define CMWSTR_STR_START	1
#define CMWSTR_STR_STOP		0

/* Timer Control Register */
#define CMWCR			0x04
#define CMWCR_CKS8		(0 << 0)
#define CMWCR_CKS32		(1 << 0)
#define CMWCR_CKS128		(2 << 0)
#define CMWCR_CKS512		(3 << 0)
#define CMWCR_CMWIE		(1 << 3)
#define CMWCR_CMS32		(0 << 9)
#define CMWCR_CMS16		(1 << 9)
#define CMWCR_CCLR_COR		(0 << 13)

/* Timer I/O Control Register */
#define CMWIOR			0x08

/* Timer Counter */
#define CMWCNT			0x10

/* Compare Match Constant Register */
#define CMWCOR			0x14

struct rz_cmtw_device;

struct rz_cmtw_info {
	unsigned int num_channels;
	unsigned int channel_offset;
};

struct rz_cmtw_channel {
	struct rz_cmtw_device *cmtw;
	unsigned int index;

	void __iomem *base;
	int irq;

	struct clk *clk;
	unsigned long rate;
	struct reset_control *rst;
	unsigned long periodic;
	struct clock_event_device ced;
	struct clocksource cs;
	bool cs_enabled;
	unsigned int enable_count;
};

struct rz_cmtw_device {
	struct platform_device *pdev;

	const struct rz_cmtw_info *info;

	void __iomem *mapbase;

	raw_spinlock_t lock;	/* Protect the shared start/stop register */

	struct rz_cmtw_channel *channels;
	unsigned int num_channels;

	bool has_clockevent;
	bool has_clocksource;
};

static struct rz_cmtw_channel *cs_to_rz_cmtw(struct clocksource *cs)
{
	return container_of(cs, struct rz_cmtw_channel, cs);
}

static struct rz_cmtw_channel *ced_to_rz_cmtw(struct clock_event_device *ced)
{
	return container_of(ced, struct rz_cmtw_channel, ced);
}

static void rz_cmtw_start_stop_ch(struct rz_cmtw_channel *ch, bool start)
{
	unsigned long flags, value;

	/* start stop register shared by multiple timer channels */
	raw_spin_lock_irqsave(&ch->cmtw->lock, flags);
	if (start)
		value = CMWSTR_STR_START;
	else
		value = CMWSTR_STR_STOP;

	writew(value, ch->base + CMWSTR);
	raw_spin_unlock_irqrestore(&ch->cmtw->lock, flags);
}

static int __rz_cmtw_enable(struct rz_cmtw_channel *ch)
{
	int ret;

	/* Release reset state */
	ret = reset_control_deassert(ch->rst);
	if (ret) {
		dev_err(&ch->cmtw->pdev->dev, "ch%u: cannot release reset\n",
			ch->index);
		return ret;
	}

	/* Enable clock */
	ret = clk_prepare_enable(ch->clk);
	if (ret) {
		dev_err(&ch->cmtw->pdev->dev, "ch%u: cannot enable clock\n",
			ch->index);
		return ret;
	}

	/* make sure channel is disabled */
	rz_cmtw_start_stop_ch(ch, 0);

	/* maximum timeout */
	writel(0xFFFFFFFF, ch->base + CMWCOR);
	writel(0, ch->base + CMWCNT);

	/*
	 * Configure channel:
	 * - Parent clock / 8
	 * - 32 bits counter
	 * - Counter cleared by CMWCOR
	 * - IRQ off
	 */
	writew(CMWCR_CKS8 | CMWCR_CMS32 | CMWCR_CCLR_COR, ch->base + CMWCR);

	/* enable channel */
	rz_cmtw_start_stop_ch(ch, 1);

	return 0;
}

static int rz_cmtw_enable(struct rz_cmtw_channel *ch)
{
	if (ch->enable_count++ > 0)
		return 0;

	return __rz_cmtw_enable(ch);
}

static void __rz_cmtw_disable(struct rz_cmtw_channel *ch)
{
	/* disable channel */
	rz_cmtw_start_stop_ch(ch, 0);

	/*
	 * Configure channel:
	 * - Parent clock / 8
	 * - 32 bits counter
	 * - Counter cleared by CMWCOR
	 * - IRQ off
	 */
	writew(CMWCR_CKS8 | CMWCR_CMS32 | CMWCR_CCLR_COR, ch->base + CMWCR);

	/* stop clock */
	clk_disable_unprepare(ch->clk);

	/* put module to reset state */
	reset_control_assert(ch->rst);
}

static void rz_cmtw_disable(struct rz_cmtw_channel *ch)
{
	if (WARN_ON(ch->enable_count == 0))
		return;

	if (--ch->enable_count > 0)
		return;

	__rz_cmtw_disable(ch);
}

static u64 rz_cmtw_clocksource_read(struct clocksource *cs)
{
	struct rz_cmtw_channel *ch = cs_to_rz_cmtw(cs);

	return (u64) readl(ch->base + CMWCNT);
}

static int rz_cmtw_clocksource_enable(struct clocksource *cs)
{
	struct rz_cmtw_channel *ch = cs_to_rz_cmtw(cs);
	int ret;

	if (WARN_ON(ch->cs_enabled))
		return 0;

	ret = rz_cmtw_enable(ch);
	if (!ret)
		ch->cs_enabled = true;

	return ret;
}

static void rz_cmtw_clocksource_disable(struct clocksource *cs)
{
	struct rz_cmtw_channel *ch = cs_to_rz_cmtw(cs);

	if (WARN_ON(!ch->cs_enabled))
		return;

	rz_cmtw_disable(ch);
	ch->cs_enabled = false;
}

static void rz_cmtw_set_next(struct rz_cmtw_channel *ch, unsigned long delta,
			    int periodic)
{
	/* stop timer */
	rz_cmtw_start_stop_ch(ch, 0);

	/*
	 * Configure channel:
	 * - Parent clock / 8
	 * - 32 bits counter
	 * - Counter cleared by CMWCOR
	 * - Enable IRQ
	 */
	writew(CMWCR_CKS8 | CMWCR_CMS32 | CMWCR_CCLR_COR | CMWCR_CMWIE,
						ch->base + CMWCR);

	if (periodic) {
		writel(delta, ch->base + CMWCOR);
		delta = 0;
		writel(0, ch->base + CMWCNT);
	} else {
		writel(0xFFFFFFFF, ch->base + CMWCOR);
		delta = delta ^ 0xFFFFFFFF;
		writel(delta, ch->base + CMWCNT);
	}

	/* start timer */
	rz_cmtw_start_stop_ch(ch, 1);
}

static irqreturn_t rz_cmtw_interrupt(int irq, void *dev_id)
{
	struct rz_cmtw_channel *ch = dev_id;

	if (clockevent_state_oneshot(&ch->ced))
		rz_cmtw_disable(ch);

	/* notify clockevent layer */
	ch->ced.event_handler(&ch->ced);

	return IRQ_HANDLED;
}

static void rz_cmtw_clock_event_start(struct rz_cmtw_channel *ch, int periodic)
{
	rz_cmtw_enable(ch);

	if (periodic) {
		ch->periodic = ((ch->rate + HZ / 2) / HZ) - 1;
		rz_cmtw_set_next(ch, ch->periodic, 1);
	}
}

static int rz_cmtw_clock_event_shutdown(struct clock_event_device *ced)
{
	struct rz_cmtw_channel *ch = ced_to_rz_cmtw(ced);

	if (clockevent_state_oneshot(ced) || clockevent_state_periodic(ced))
		rz_cmtw_disable(ch);
	return 0;
}

static int rz_cmtw_clock_event_set_state(struct clock_event_device *ced,
					int periodic)
{
	struct rz_cmtw_channel *ch = ced_to_rz_cmtw(ced);

	/* deal with old setting first */
	if (clockevent_state_oneshot(ced) || clockevent_state_periodic(ced))
		rz_cmtw_disable(ch);

	dev_info(&ch->cmtw->pdev->dev, "ch%u: used for %s clock events\n",
		 ch->index, periodic ? "periodic" : "oneshot");
	rz_cmtw_clock_event_start(ch, periodic);
	return 0;
}


static int rz_cmtw_clock_event_set_oneshot(struct clock_event_device *ced)
{
	return rz_cmtw_clock_event_set_state(ced, 0);
}

static int rz_cmtw_clock_event_set_periodic(struct clock_event_device *ced)
{
	return rz_cmtw_clock_event_set_state(ced, 1);
}

static int rz_cmtw_clock_event_next(unsigned long delta,
				   struct clock_event_device *ced)
{
	struct rz_cmtw_channel *ch = ced_to_rz_cmtw(ced);

	WARN_ON(!clockevent_state_oneshot(ced));

	/* program new delta value */
	rz_cmtw_set_next(ch, delta, 0);
	return 0;
}


static int rz_cmtw_register_clockevent(struct rz_cmtw_channel *ch,
				       const char *name)
{
	struct clock_event_device *ced = &ch->ced;
	int irq;
	int ret;

	ch->irq = platform_get_irq(ch->cmtw->pdev, ch->index);
	if (ch->irq < 0)
		return irq;

	ret = request_irq(ch->irq, rz_cmtw_interrupt,
			  IRQF_TIMER | IRQF_IRQPOLL | IRQF_NOBALANCING,
			  dev_name(&ch->cmtw->pdev->dev), ch);
	if (ret) {
		dev_err(&ch->cmtw->pdev->dev, "ch%u: failed to request irq %d\n",
			ch->index, irq);
		return ret;
	}

	ced->name = name;
	ced->features = CLOCK_EVT_FEAT_PERIODIC;
	ced->features |= CLOCK_EVT_FEAT_ONESHOT;
	ced->rating = 100;
	ced->cpumask = cpu_possible_mask;
	ced->set_next_event = rz_cmtw_clock_event_next;
	ced->set_state_shutdown = rz_cmtw_clock_event_shutdown;
	ced->set_state_oneshot = rz_cmtw_clock_event_set_oneshot;
	ced->set_state_periodic = rz_cmtw_clock_event_set_periodic;
	dev_info(&ch->cmtw->pdev->dev, "ch%u: used for clock events\n",
		ch->index);
	clockevents_register_device(ced);

	return 0;
};

static int rz_cmtw_register_clocksource(struct rz_cmtw_channel *ch,
				       const char *name)
{
	struct clocksource *cs = &ch->cs;

	cs->name = name;
	cs->rating = 100;
	cs->read = rz_cmtw_clocksource_read;
	cs->enable = rz_cmtw_clocksource_enable;
	cs->disable = rz_cmtw_clocksource_disable;
	cs->mask = CLOCKSOURCE_MASK(32);
	cs->flags = CLOCK_SOURCE_IS_CONTINUOUS;

	dev_info(&ch->cmtw->pdev->dev, "ch%u: used as clock source\n",
		 ch->index);

	clocksource_register_hz(cs, ch->rate);

	return 0;
}

static int rz_cmtw_register(struct rz_cmtw_channel *ch, const char *name,
			   bool clockevent, bool clocksource)
{
	if (clockevent) {
		ch->cmtw->has_clockevent = true;
		return rz_cmtw_register_clockevent(ch, name);
	} else if (clocksource) {
		ch->cmtw->has_clocksource = true;
		return rz_cmtw_register_clocksource(ch, name);
	}

	return 0;
}

static int rz_cmtw_channel_setup(struct rz_cmtw_channel *ch, unsigned int index,
				 bool clockevent, bool clocksource,
				 struct rz_cmtw_device *cmtw)
{
	/* Skip unused channels. */
	if (!clockevent && !clocksource)
		return 0;

	ch->cmtw = cmtw;
	ch->index = index;
	ch->base = cmtw->mapbase + ch->index * cmtw->info->channel_offset;
	ch->cs_enabled = false;
	ch->enable_count = 0;

	return rz_cmtw_register(ch, dev_name(&cmtw->pdev->dev),
					clockevent, clocksource);
}

static int rz_cmtw_setup(struct rz_cmtw_device *cmtw,
			 struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	unsigned int i;
	int ret;

	cmtw->pdev = pdev;
	cmtw->info = of_device_get_match_data(dev);
	if (!cmtw->info) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENXIO;
	}

	raw_spin_lock_init(&cmtw->lock);

	cmtw->num_channels = cmtw->info->num_channels;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to get I/O memory\n");
		return -ENODEV;
	}

	cmtw->mapbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(cmtw->mapbase))
		return PTR_ERR(cmtw->mapbase);

	/* Allocate and setup the channels. */
	cmtw->channels = kcalloc(cmtw->num_channels, sizeof(*cmtw->channels),
				 GFP_KERNEL);
	if (!cmtw->channels) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	/* Get clock and resets for each channels */
	for (i = 0; i < cmtw->num_channels; i++) {
		char clk_name[5];

		sprintf(clk_name, "fck%u", i);
		cmtw->channels[i].clk = devm_clk_get(dev, clk_name);
		if (IS_ERR(cmtw->channels[i].clk)) {
			dev_err(dev, "cannot get clock %s\n", clk_name);
			ret = PTR_ERR(cmtw->channels[i].clk);
			goto err_unmap;
		};

		/* Determine clock rate. */
		cmtw->channels[i].rate = clk_get_rate(cmtw->channels[i].clk) / 8;

		cmtw->channels[i].rst =
			devm_reset_control_get_exclusive_by_index(dev, i);
		if (IS_ERR(cmtw->channels[i].rst)) {
			dev_err(dev, "cannot get reset\n");
			ret = PTR_ERR(cmtw->channels[i].rst);
			goto err_unmap;
		}
	};

	/*
	 * Define below clock usage:
	 * - 1st channel: clock source
	 * - 2nd channel: clock event
	 */
	for (i = 0; i < cmtw->num_channels; ++i) {
		ret = rz_cmtw_channel_setup(&cmtw->channels[i], i,
					   i == 1, i == 0, cmtw);
		if (ret < 0)
			goto err_unmap;
	}

	platform_set_drvdata(pdev, cmtw);

	return 0;

err_unmap:
	kfree(cmtw->channels);
	iounmap(cmtw->mapbase);

	return ret;
}

static int rz_cmtw_probe(struct platform_device *pdev)
{
	struct rz_cmtw_device *cmtw = platform_get_drvdata(pdev);
	int ret;

	cmtw = kzalloc(sizeof(*cmtw), GFP_KERNEL);
	if (cmtw == NULL)
		return -ENOMEM;

	ret = rz_cmtw_setup(cmtw, pdev);
	if (ret) {
		kfree(cmtw);
		return ret;
	}

	return 0;
}

static int rz_cmtw_remove(struct platform_device *pdev)
{
	return -EBUSY; /* cannot unregister clockevent and clocksource */
}

static const struct rz_cmtw_info rzv2h_cmtw_info = {
	.num_channels = 4,
	.channel_offset = 0x400,
};

static const struct of_device_id rz_cmtw_of_table[] __maybe_unused = {
	{ .compatible = "renesas,rzv2h-cmtw", .data = &rzv2h_cmtw_info },
	{ .compatible = "renesas,rzg3e-cmtw", .data = &rzv2h_cmtw_info },
	{ }
};
MODULE_DEVICE_TABLE(of, rz_cmtw_of_table);

static struct platform_driver rz_cmtw_device_driver = {
	.probe		= rz_cmtw_probe,
	.remove		= rz_cmtw_remove,
	.driver		= {
		.name	= "rz_cmtw",
		.of_match_table = of_match_ptr(rz_cmtw_of_table),
	},
};

static int __init rz_cmtw_init(void)
{
	return platform_driver_register(&rz_cmtw_device_driver);
}

static void __exit rz_cmtw_exit(void)
{
	platform_driver_unregister(&rz_cmtw_device_driver);
}

subsys_initcall(rz_cmtw_init);
module_exit(rz_cmtw_exit);

MODULE_DESCRIPTION("Renesas RZ CMTW Timer Driver");
MODULE_LICENSE("GPL v2");
