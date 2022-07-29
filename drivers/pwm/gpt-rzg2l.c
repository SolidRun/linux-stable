// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G2L GPT Timer driver
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#define GTPR_MAX_VALUE	0xFFFFFFFF
#define GTSTR		0x0004
#define GTSTP		0x0008
#define GTCLR		0x000C
#define GTSSR		0x0010
#define GTPSR		0x0014
#define GTCSR		0x0018
#define GTUPSR		0x001C
#define GTDNSR		0x0020
#define GTICASR		0x0024
#define GTICBSR		0x0028
#define GTCR		0x002C
#define GTUDDTYC	0x0030
#define GTIOR		0x0034
#define GTINTAD		0x0038
#define GTST		0x003C
#define GTBER		0x0040
#define GTCNT		0x0048
#define GTPR		0x0064
#define GTCCRA		0x004C
#define GTCCRB		0x0050
#define GTCCRC		0x0054
#define GTCCRD		0x005C
#define GTCCRE		0x0058
#define GTCCRF		0x0060

#define SAW_WAVE		(0x00<<16)
#define SAW_WAVE_ONE_SHOT	(0x01<<16)
#define TRIANGLE_WAVE_MODE1	(0x04<<16)
#define TRIANGLE_WAVE_MODE2	(0x05<<16)
#define TRIANGLE_WAVE_MODE3	(0x06<<16)
#define GTCR_MODE_MASK		(0x7<<16)
#define GTCR_PRESCALE_MASK	(0x7<<24)
#define GTIOB_OUTPUT_HIGH_END_LOW_COMPARE	(0x119<<16)
/* GTIOR.GTIOB = 11001 */
/* GTIOR.OBE = 1 */
#define GTIOA_OUTPUT_HIGH_END_LOW_COMPARE	0x119
#define GTCR_CST	0x00000001
#define UP_COUNTING	3
#define P0_1024		(0x05<<24)
#define INPUT_CAP_GTIOB_BOTH_EDGE	0x0000F000
#define INPUT_CAP_GTIOB_RISING_EDGE	0x00003000
#define INPUT_CAP_GTIOB_FALLING_EDGE	0x0000C000
#define INPUT_CAP_GTIOA_BOTH_EDGE	0x00000F00
#define INPUT_CAP_GTIOA_RISING_EDGE	0x00000300
#define INPUT_CAP_GTIOA_FALLING_EDGE	0x00000C00
#define NOISE_FILT_BEN	(1<<29)
#define NOISE_FILT_B_P0_64	(0x11<<30)
#define NOISE_FILT_AEN	(1<<13)
#define NOISE_FILT_A_P0_64	(0x11<<14)
#define GTINTB	(1<<1)
#define GTINTA	(1<<0)
#define TCFB	(1<<1)
#define TCFA	(1<<0)
#define TCFPO	(1<<6)
#define GTINTPROV	(0x01<<6)
#define CCRSWT	(1<<22)
#define GTCCRB_BUFFER_SINGLE	(0x01<<18)
#define GTCCRB_BUFFER_DOUBLE	(1<<19)
#define GTCCRA_BUFFER_SINGLE	(0x01<<16)
#define GTCCRA_BUFFER_DOUBLE	(1<<17)

struct rzg2l_gpt_chip {
	struct	pwm_chip chip;
	struct	clk *clk;
	int	clk_enable;
	void	__iomem *mmio_base;
	spinlock_t lock;
	struct reset_control *rstc;
	wait_queue_head_t wait;
	struct mutex mutex;
	u64 snapshot[3];
	unsigned int index;
	unsigned int overflow_count, buffer_mode_count;
	unsigned long buffer[3];
	unsigned int period_ns;
	const char *channel;
};

static inline struct rzg2l_gpt_chip *to_rzg2l_gpt_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct rzg2l_gpt_chip, chip);
}

static void rzg2l_gpt_write(struct rzg2l_gpt_chip *pc, u32 data,
				unsigned int offset)
{
	iowrite32(data, pc->mmio_base + offset);
}

static void rzg2l_gpt_write_mask(struct rzg2l_gpt_chip *pc, u32 data, u32 mask,
				unsigned int offset)
{
	u32 tmp = 0;

	tmp = ioread32(pc->mmio_base + offset);
	tmp &= ~mask;
	iowrite32(tmp|data, pc->mmio_base + offset);
}

static u32 rzg2l_gpt_read(struct rzg2l_gpt_chip *pc, unsigned int offset)
{
	return ioread32(pc->mmio_base + offset);
}

static void rzg2l_timer_count_start(struct rzg2l_gpt_chip *pc)
{
	uint32_t tmp = 0;

	/* Timer count start */
	/* GTCR.CST = 1 */
	tmp = rzg2l_gpt_read(pc, GTCR);
	tmp |= GTCR_CST;
	rzg2l_gpt_write(pc, tmp, GTCR);
}

static void rzg2l_timer_count_stop(struct rzg2l_gpt_chip *pc)
{
	uint32_t tmp = 0;

	/* Timer count stop */
	/* GTCR.CST = 0 */
	tmp = rzg2l_gpt_read(pc, GTCR);
	tmp &= ~GTCR_CST;
	rzg2l_gpt_write(pc, tmp, GTCR);
}

static int rzg2l_calculate_prescale(struct rzg2l_gpt_chip *pc,
						int period_ns)
{
	unsigned long long c, clk_rate;
	unsigned long period_cycles;
	int prescale;

	clk_rate = clk_get_rate(pc->clk);
	c = clk_rate * period_ns;
	do_div(c, 1000000000);

	period_cycles = c;
	if (period_cycles < 1)
		period_cycles = 1;
	/* prescale 1,4,16,64 */
	/* 1 Dividing */
	if ((period_cycles / GTPR_MAX_VALUE) == 0) {
		prescale = 0;
	/* 4 Dividing */
	} else if ((period_cycles / (GTPR_MAX_VALUE * 4)) == 0) {
		prescale = 1;
	/* 16 Dividing */
	} else if ((period_cycles / (GTPR_MAX_VALUE * 16)) == 0) {
		prescale = 2;
	/* 64 Dividing */
	} else if ((period_cycles / (GTPR_MAX_VALUE * 64)) == 0) {
		prescale = 3;
	/* 256 Dividing */
	} else if ((period_cycles / (GTPR_MAX_VALUE * 256)) == 0) {
		prescale = 4;
	/* 1024 Dividing */
	} else if ((period_cycles / (GTPR_MAX_VALUE * 1024)) == 0) {
		prescale = 5;
	} else {
		dev_err(pc->chip.dev, "gpt-rzg2l prescale over!!\n");
		return -1;
	}

	return prescale;
}

static unsigned long
rzg2l_time_to_tick_number(struct rzg2l_gpt_chip *pc, int time_ns,
				unsigned long prescale)
{
	unsigned long long c, clk_rate;
	unsigned long period_cycles;

	clk_rate = clk_get_rate(pc->clk);
	c = clk_rate * time_ns;
	do_div(c, 1000000000);

	period_cycles = c;
	if (period_cycles < 1)
		period_cycles = 1;

	switch (prescale) {
	case 0:
		period_cycles /= 1;
		break;
	case 1:
		period_cycles /= 4;
		break;
	case 2:
		period_cycles /= 16;
		break;
	case 3:
		period_cycles /= 64;
		break;
	case 4:
		period_cycles /= 256;
		break;
	case 5:
		period_cycles /= 1024;
		break;
	}

	return period_cycles;
}

static unsigned long long
rzg2l_tick_number_to_time(struct rzg2l_gpt_chip *pc, u32 tick_number,
			unsigned long prescale)
{
	unsigned long long c, clk_rate, time_ns;

	clk_rate = clk_get_rate(pc->clk);
	c = (unsigned long long)tick_number * 1000000000;
	do_div(c, clk_rate);
	time_ns = c;

	switch (prescale) {
	case 0:
		time_ns *= 1;
		break;
	case 1:
		time_ns *= 4;
		break;
	case 2:
		time_ns *= 16;
		break;
	case 3:
		time_ns *= 64;
		break;
	case 4:
		time_ns *= 256;
		break;
	case 5:
		time_ns *= 1024;
		break;
	}

	return time_ns;
}

static int rzg2l_gpt_config(struct pwm_chip *chip, struct pwm_device *pwm,
				int duty_ns, int period_ns)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);
	unsigned long pv, dc;
	int rc, prescale;

	if ((duty_ns < 0) || (period_ns < 0)) {
		dev_err(chip->dev, "Set time negative\n");
		return -EINVAL;
	}

	prescale = rzg2l_calculate_prescale(pc, period_ns);
	if (prescale < 0)
		return -EINVAL;

	pv = rzg2l_time_to_tick_number(pc, period_ns, prescale);
	dc = rzg2l_time_to_tick_number(pc, duty_ns, prescale);

	if (duty_ns == period_ns)
		dc = pv;

	pc->period_ns = period_ns;
	/* NOTE: the clock to GPT has to be enabled first
	 * before writing to the registers
	 */
	rc = clk_prepare_enable(pc->clk);
	if (rc < 0) {
		dev_err(chip->dev, "Unavailable  clock\n");
		return rc;
	}

	/* GPT setting saw-wave up-counting */
	/* Set operation GPT mode and select count clock */
	rzg2l_gpt_write_mask(pc, SAW_WAVE, GTCR_MODE_MASK, GTCR);
	rzg2l_gpt_write_mask(pc, (prescale<<24), GTCR_PRESCALE_MASK, GTCR);
	/* Set counting mode */
	rzg2l_gpt_write(pc, UP_COUNTING, GTUDDTYC); //up-counting
	/* Set period */
	rzg2l_gpt_write(pc, pv, GTPR);

	if (!strcmp(pc->channel, "channel_B")) {
		/* Set duty */
		rzg2l_gpt_write(pc, dc, GTCCRB);
		/* Enable pin output */
		rzg2l_gpt_write(pc, GTIOB_OUTPUT_HIGH_END_LOW_COMPARE, GTIOR);
	} else {
		/* Set duty */
		rzg2l_gpt_write(pc, dc, GTCCRA);
		/* Enable pin output */
		rzg2l_gpt_write(pc, GTIOA_OUTPUT_HIGH_END_LOW_COMPARE, GTIOR);
	}

	/* Set initial value for counter */
	rzg2l_gpt_write(pc, 0, GTCNT); // reset counter value
	/* Set no buffer operation */
	rzg2l_gpt_write(pc, 0, GTBER);

	clk_disable_unprepare(pc->clk);

	return 0;
}

static int rzg2l_gpt_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);
	int rc = 0;

	if (!pc->clk_enable) {
		rc = clk_prepare_enable(pc->clk);
		if (!rc)
			pc->clk_enable++;
		rzg2l_timer_count_start(pc);
	}

	return rc;
}

static void rzg2l_gpt_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);

	if (pc->clk_enable) {
		rzg2l_timer_count_stop(pc);
		clk_disable_unprepare(pc->clk);
		pc->clk_enable--;
	}
}

static int
rzg2l_gpt_capture(struct pwm_chip *chip, struct pwm_device *pwm,
		struct pwm_capture *result, unsigned long timeout)
{
	int ret, rc;
	unsigned long flags;
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);
	unsigned int effective_ticks;
	u64 high, low;

	rc = clk_prepare_enable(pc->clk);
	if (rc < 0) {
		dev_err(chip->dev, "Unavailable  clock\n");
		return rc;
	}

	spin_lock_irqsave(&pc->lock, flags);

	result->period = result->duty_cycle = 0;
	pc->index = 0;
	pc->snapshot[0] = 0;
	pc->snapshot[1] = 0;
	pc->snapshot[2] = 0;
	pc->overflow_count = 0;

	/* Prepare capture measurement */
	/* Set operating mode GTCR.MD[2:0] and count clock GTCR.TPCS[2:0]*/
	//Using lowest frequency P0/1024 to avoid overflow
	rzg2l_gpt_write(pc, SAW_WAVE|P0_1024, GTCR);
	/* Set count direction with GTUDDTYC[1:0]*/
	rzg2l_gpt_write(pc, UP_COUNTING, GTUDDTYC); //up-counting
	/* Set cycle in GTPR */
	rzg2l_gpt_write(pc, GTPR_MAX_VALUE, GTPR); //Maximum overflow value
	/* Set initial value in GTCNT */
	rzg2l_gpt_write(pc, 0, GTCNT);
	/* Set input pin as capture mode */
	if (!strcmp(pc->channel, "channel_B")) {
		//Using noise filter with P0/64 clock
		rzg2l_gpt_write(pc, NOISE_FILT_BEN|NOISE_FILT_B_P0_64, GTIOR);
		/* Select input capture source in GTICASR and GTICBSR */
		rzg2l_gpt_write(pc, INPUT_CAP_GTIOB_RISING_EDGE, GTICBSR);
		/* Enable input capture and overflow interrupt*/
		rzg2l_gpt_write(pc, GTINTB|GTINTPROV, GTINTAD);
	} else {
		//Using noise filter with P0/64 clock
		rzg2l_gpt_write(pc, NOISE_FILT_AEN|NOISE_FILT_A_P0_64, GTIOR);
		/* Select input capture source in GTICASR and GTICBSR */
		rzg2l_gpt_write(pc, INPUT_CAP_GTIOA_RISING_EDGE, GTICASR);
		/* Enable input capture and overflow interrupt*/
		rzg2l_gpt_write(pc, GTINTA|GTINTPROV, GTINTAD);
	}
	/* Start count operation set GTCR.CST to 1 to start count operation*/
	rzg2l_timer_count_start(pc);

	spin_unlock_irqrestore(&pc->lock, flags);

	ret = wait_event_interruptible(pc->wait, pc->index > 1);
	if (ret == -ERESTARTSYS)
		goto out;

	spin_lock_irqsave(&pc->lock, flags);

	switch (pc->index) {
	case 0:
	case 1:
		result->period = 0;
		result->duty_cycle = 0;
		break;
	case 2:
		high = pc->snapshot[1] - pc->snapshot[0];
		low = pc->snapshot[2] - pc->snapshot[1];
		effective_ticks = clk_get_rate(pc->clk)/1024;
		result->period = (high + low) * MSEC_PER_SEC;
		result->period /= effective_ticks;
		result->duty_cycle = high * MSEC_PER_SEC;
		result->duty_cycle /= effective_ticks;
		break;
	default:
		dev_err(chip->dev, "internal error\n");
		break;
	}

out:
	if (!strcmp(pc->channel, "channel_B")) {
		/* Disable capture operation */
		rzg2l_gpt_write(pc, 0, GTICBSR);
	} else {
		/* Disable capture operation */
		rzg2l_gpt_write(pc, 0, GTICASR);
	}
	/* Disable interrupt */
	rzg2l_gpt_write(pc, 0, GTINTAD);

	spin_unlock_irqrestore(&pc->lock, flags);

	clk_disable_unprepare(pc->clk);

	return ret;
}

static const struct pwm_ops rzg2l_gpt_ops = {
	.capture = rzg2l_gpt_capture,
	.config = rzg2l_gpt_config,
	.enable = rzg2l_gpt_enable,
	.disable = rzg2l_gpt_disable,
	.owner = THIS_MODULE,
};

static irqreturn_t gpt_gtciv_interrupt(int irq, void *data)
{
	struct rzg2l_gpt_chip *pc = data;
	int ret = IRQ_NONE;
	uint32_t irq_flags;
	unsigned long flags;
	uint32_t tmp;

	spin_lock_irqsave(&pc->lock, flags);

	irq_flags = rzg2l_gpt_read(pc, GTST);
	if (irq_flags & TCFPO) {
		pc->overflow_count++;
		pc->buffer_mode_count--;

		if (!strcmp(pc->channel, "channel_B")) {
			tmp = rzg2l_gpt_read(pc, GTBER);
			if (tmp & GTCCRB_BUFFER_SINGLE) {
				rzg2l_gpt_write(pc,
					pc->buffer[pc->buffer_mode_count],
					GTCCRE);
				if (pc->buffer_mode_count == 0)
					pc->buffer_mode_count = 2;
			}

			tmp = rzg2l_gpt_read(pc, GTBER);
			if (tmp & GTCCRB_BUFFER_DOUBLE) {
				rzg2l_gpt_write(pc,
					pc->buffer[pc->buffer_mode_count],
					GTCCRF);
				if (pc->buffer_mode_count == 0)
					pc->buffer_mode_count = 3;
			}
		} else {
			tmp = rzg2l_gpt_read(pc, GTBER);
			if (tmp & GTCCRA_BUFFER_SINGLE) {
				rzg2l_gpt_write(pc,
					pc->buffer[pc->buffer_mode_count],
					GTCCRC);
				if (pc->buffer_mode_count == 0)
					pc->buffer_mode_count = 2;
			}

			tmp = rzg2l_gpt_read(pc, GTBER);
			if (tmp & GTCCRA_BUFFER_DOUBLE) {
				rzg2l_gpt_write(pc,
					pc->buffer[pc->buffer_mode_count],
					GTCCRD);
				if (pc->buffer_mode_count == 0)
					pc->buffer_mode_count = 3;
			}
		}

		irq_flags &= ~TCFPO;
		ret = IRQ_HANDLED;
	}

	/* Disable overflow interrupt flags */
	rzg2l_gpt_write(pc, irq_flags, GTST);

	spin_unlock_irqrestore(&pc->lock, flags);

	return ret;
}

static irqreturn_t gpt_gtcia_interrupt(int irq, void *data)
{
	struct rzg2l_gpt_chip *pc = data;
	int ret = IRQ_NONE;
	uint32_t irq_flags;
	unsigned long flags;
	uint32_t tmp = 0;

	spin_lock_irqsave(&pc->lock, flags);

	irq_flags = rzg2l_gpt_read(pc, GTST);
	if (irq_flags & TCFA) {
		pc->snapshot[pc->index] = rzg2l_gpt_read(pc, GTCCRA) +
			(pc->overflow_count) * GTPR_MAX_VALUE;
		switch (pc->index) {
		case 0:
		case 1:
			tmp = rzg2l_gpt_read(pc, GTICASR);
			if (tmp & INPUT_CAP_GTIOA_RISING_EDGE)
				rzg2l_gpt_write(pc,
				INPUT_CAP_GTIOA_FALLING_EDGE, GTICASR);
			if (tmp & INPUT_CAP_GTIOA_FALLING_EDGE)
				rzg2l_gpt_write(pc, INPUT_CAP_GTIOA_RISING_EDGE,
						GTICASR);
			pc->index++;
			break;
		case 2:
			/* Disable capture operation */
			rzg2l_gpt_write(pc, 0, GTICASR);
			wake_up(&pc->wait);
			break;
		default:
			dev_err(pc->chip.dev, "Internal error\n");
		}

		irq_flags &= ~TCFA;
		ret = IRQ_HANDLED;
	}

	/* Disable input capture interrupt flags */
	rzg2l_gpt_write(pc, irq_flags, GTST);

	spin_unlock_irqrestore(&pc->lock, flags);

	return ret;
}

static irqreturn_t gpt_gtcib_interrupt(int irq, void *data)
{
	struct rzg2l_gpt_chip *pc = data;
	int ret = IRQ_NONE;
	uint32_t irq_flags;
	unsigned long flags;
	uint32_t tmp = 0;

	spin_lock_irqsave(&pc->lock, flags);

	irq_flags = rzg2l_gpt_read(pc, GTST);
	if (irq_flags & TCFB) {
		pc->snapshot[pc->index] = rzg2l_gpt_read(pc, GTCCRB) +
			(pc->overflow_count) * GTPR_MAX_VALUE;
		switch (pc->index) {
		case 0:
		case 1:
			tmp = rzg2l_gpt_read(pc, GTICBSR);
			if (tmp & INPUT_CAP_GTIOB_RISING_EDGE)
				rzg2l_gpt_write(pc,
					INPUT_CAP_GTIOB_FALLING_EDGE, GTICBSR);
			if (tmp & INPUT_CAP_GTIOB_FALLING_EDGE)
				rzg2l_gpt_write(pc, INPUT_CAP_GTIOB_RISING_EDGE,
						GTICBSR);
			pc->index++;
			break;
		case 2:
			/* Disable capture operation */
			rzg2l_gpt_write(pc, 0, GTICBSR);
			wake_up(&pc->wait);
			break;
		default:
			dev_err(pc->chip.dev, "Internal error\n");
		}

		irq_flags &= ~TCFB;
		ret = IRQ_HANDLED;
	}

	/* Disable input capture interrupt flags */
	rzg2l_gpt_write(pc, irq_flags, GTST);

	spin_unlock_irqrestore(&pc->lock, flags);

	return ret;
}

static ssize_t buff0_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned int val;
	int ret;
	unsigned long prescale;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (pc->period_ns < val)
		return -EINVAL;

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->buffer[0] = rzg2l_time_to_tick_number(pc, val, prescale);
	if (pc->buffer[0] == 0) {
		ret = -EINVAL;
		goto out;
	}

	ret = clk_prepare_enable(pc->clk);
	if (ret < 0) {
		dev_err(pc->chip.dev, "Unavailable  clock\n");
		goto out;
	}

	/*Set compare match value in GTCCRA in GTCCRB*/
	if (!strcmp(pc->channel, "channel_B"))
		rzg2l_gpt_write(pc, pc->buffer[0], GTCCRB);
	else
		rzg2l_gpt_write(pc, pc->buffer[0], GTCCRA);

	clk_disable_unprepare(pc->clk);

out:
	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buff0_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->buffer[0], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buff1_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned int val;
	int ret;
	unsigned long prescale;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (pc->period_ns < val)
		return -EINVAL;

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->buffer[1] = rzg2l_time_to_tick_number(pc, val, prescale);
	if (pc->buffer[1] == 0) {
		ret = -EINVAL;
		goto out;
	}

	pc->buffer_mode_count = 2;

	ret = clk_prepare_enable(pc->clk);
	if (ret < 0) {
		dev_err(pc->chip.dev, "Unavailable  clock\n");
		goto out;
	}

	if (!strcmp(pc->channel, "channel_B")) {
		/*Set buffer operation with CCRA CCRB in GTBER*/
		rzg2l_gpt_write(pc, GTCCRB_BUFFER_SINGLE, GTBER);
		/* Set buffer value for B in GTCCRE(single), GTCCRF(double)*/
		rzg2l_gpt_write(pc, pc->buffer[1], GTCCRE);
	} else {
		/*Set buffer operation with CCRA CCRB in GTBER*/
		rzg2l_gpt_write(pc, GTCCRA_BUFFER_SINGLE, GTBER);
		/* Set buffer value for A in GTCCRC(single), GTCCRD(double)*/
		rzg2l_gpt_write(pc, pc->buffer[1], GTCCRC);
	}

	/* Enable overflow interrupt*/
	rzg2l_gpt_write(pc, GTINTPROV, GTINTAD);

	clk_disable_unprepare(pc->clk);

out:
	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buff1_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->buffer[1], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buff2_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned int val;
	int ret;
	unsigned long prescale;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (pc->period_ns < val)
		return -EINVAL;

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->buffer[2] = rzg2l_time_to_tick_number(pc, val, prescale);
	if (pc->buffer[2] == 0) {
		ret = -EINVAL;
		goto out;
	}

	pc->buffer_mode_count = 3;

	ret = clk_prepare_enable(pc->clk);
	if (ret < 0) {
		dev_err(pc->chip.dev, "Unavailable  clock\n");
		goto out;
	}

	if (!strcmp(pc->channel, "channel_B")) {
		/*Set buffer operation with CCRA CCRB in GTBER*/
		rzg2l_gpt_write(pc, GTCCRB_BUFFER_DOUBLE, GTBER);
		/* Set buffer value for B in GTCCRE(single), GTCCRF(double)*/
		rzg2l_gpt_write(pc, pc->buffer[2], GTCCRF);
	} else {
		/*Set buffer operation with CCRA CCRB in GTBER*/
		rzg2l_gpt_write(pc, GTCCRA_BUFFER_DOUBLE, GTBER);
		/* Set buffer value for B in GTCCRC(single), GTCCRD(double)*/
		rzg2l_gpt_write(pc, pc->buffer[2], GTCCRD);
	}

	/* Enable overflow interrupt*/
	rzg2l_gpt_write(pc, GTINTPROV, GTINTAD);

	clk_disable_unprepare(pc->clk);

out:
	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buff2_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->buffer[2], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static DEVICE_ATTR_RW(buff0);
static DEVICE_ATTR_RW(buff1);
static DEVICE_ATTR_RW(buff2);

static struct attribute *buffer_attrs[] = {
	&dev_attr_buff0.attr,
	&dev_attr_buff1.attr,
	&dev_attr_buff2.attr,
	NULL,
};

static const struct attribute_group buffer_attr_group = {
	.attrs = buffer_attrs,
};

static int rzg2l_gpt_probe(struct platform_device *pdev)
{
	struct rzg2l_gpt_chip *rzg2l_gpt;
	struct resource *res;
	int ret;
	int irq = 0;

	rzg2l_gpt = devm_kzalloc(&pdev->dev, sizeof(*rzg2l_gpt), GFP_KERNEL);
	if (rzg2l_gpt == NULL)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No memory resource defined\n");
		return -ENODEV;
	}

	rzg2l_gpt->mmio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rzg2l_gpt->mmio_base))
		return PTR_ERR(rzg2l_gpt->mmio_base);

	rzg2l_gpt->rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rzg2l_gpt->rstc)) {
		dev_err(&pdev->dev, "failed to get cpg reset\n");
		return PTR_ERR(rzg2l_gpt->rstc);
	}

	reset_control_deassert(rzg2l_gpt->rstc);

	irq = platform_get_irq_byname(pdev, "gtcib");
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to obtain IRQ\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, gpt_gtcib_interrupt, 0,
				dev_name(&pdev->dev), rzg2l_gpt);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "gtcia");
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to obtain IRQ\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, gpt_gtcia_interrupt, 0,
				dev_name(&pdev->dev), rzg2l_gpt);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "gtciv");
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to obtain IRQ\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, gpt_gtciv_interrupt, 0,
				dev_name(&pdev->dev), rzg2l_gpt);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "channel",
					&rzg2l_gpt->channel);
	if (ret < 0) {
		dev_err(&pdev->dev, "Not define GPT channel\n");
		return ret;
	}

	if (strcmp(rzg2l_gpt->channel, "channel_A")
	    && strcmp(rzg2l_gpt->channel, "channel_B")) {
		dev_err(&pdev->dev, "Failed to get GPT channel\n");
		return -ENODEV;
	}

	rzg2l_gpt->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(rzg2l_gpt->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(rzg2l_gpt->clk);
	}

	rzg2l_gpt->clk_enable = 0;
	rzg2l_gpt->chip.dev = &pdev->dev;
	rzg2l_gpt->chip.ops = &rzg2l_gpt_ops;
	rzg2l_gpt->chip.base = -1;
	rzg2l_gpt->chip.npwm = 1;

	spin_lock_init(&rzg2l_gpt->lock);
	mutex_init(&rzg2l_gpt->mutex);
	init_waitqueue_head(&rzg2l_gpt->wait);

	ret = pwmchip_add(&rzg2l_gpt->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register GPT chip: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&rzg2l_gpt->chip.dev->kobj,
				&buffer_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create sysfs: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "RZ/G2L GPT Driver probed\n");
	platform_set_drvdata(pdev, rzg2l_gpt);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int rzg2l_gpt_remove(struct platform_device *pdev)
{
	struct rzg2l_gpt_chip *rzg2l_gpt = platform_get_drvdata(pdev);

	sysfs_remove_group(&rzg2l_gpt->chip.dev->kobj, &buffer_attr_group);
	pm_runtime_disable(&pdev->dev);

	return pwmchip_remove(&rzg2l_gpt->chip);
}

static const struct of_device_id rzg2l_gpt_of_table[] = {
	{ .compatible = "renesas,gpt-r9a07g044", },
	{ },
};

MODULE_DEVICE_TABLE(of, rzg2l_gpt_of_table);

static struct platform_driver rzg2l_gpt_driver = {
	.probe = rzg2l_gpt_probe,
	.remove = rzg2l_gpt_remove,
	.driver = {
		.name = "gpt-rzg2l",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rzg2l_gpt_of_table),
	},
};

module_platform_driver(rzg2l_gpt_driver);

MODULE_DESCRIPTION("Renesas RZG2L GPT Driver");
MODULE_LICENSE("GPL v2");
