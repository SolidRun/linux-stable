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
#define GTIOR_CHANNEL_B_OUTPUT_MASK	(0x01FF<<16)
#define GTIOB_OUTPUT_HIGH_END_LOW_COMPARE	(0x119<<16)
/* GTIOR.GTIOB = 11001 */
/* GTIOR.OBE = 1 */
#define GTIOR_CHANNEL_A_OUTPUT_MASK	0x01FF
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
#define GTINTAD_GTINTPR_MASK	(3<<6)
#define CCRSWT	(1<<22)
#define GTCCRB_BUFFER_SINGLE	(0x01<<18)
#define GTCCRB_BUFFER_DOUBLE	(1<<19)
#define GTCCRA_BUFFER_SINGLE	(0x01<<16)
#define GTCCRA_BUFFER_DOUBLE	(1<<17)
#define GTCCRA_BUFFER_MASK	(3<<16)
#define GTCCRB_BUFFER_MASK	(3<<18)

enum {
	CHANNEL_A,
	CHANNEL_B,
	BOTH_AB,
	NR_CHANNEL,
};

struct reg {
	u32 value, mask, offset;
};

struct set_params {
	struct reg phase;
	struct reg duty;
	struct reg noise_filter;
	struct reg input_source;
	struct reg input_flag;
};

/* Parameters which we must set manually via sysfs or not use
 * is 0 in default
 */
static const struct set_params channel_set[NR_CHANNEL] = {
	/* Setting for channel A mode */
	[CHANNEL_A] = {
		.phase = {
			GTIOA_OUTPUT_HIGH_END_LOW_COMPARE,
			GTIOR_CHANNEL_A_OUTPUT_MASK,
			GTIOR,
		},
		.duty = {
			0,
			0,
			GTCCRA,
		},
		.noise_filter = {
			NOISE_FILT_AEN|NOISE_FILT_A_P0_64,
			0,
			GTIOR,
		},
		.input_source = {
			INPUT_CAP_GTIOA_RISING_EDGE,
			0,
			GTICASR,
		},
		.input_flag = {
			GTINTA,
			0,
			GTINTAD,
		},
	},

	/* Setting for channel B mode */
	[CHANNEL_B] = {
		.phase = {
			GTIOB_OUTPUT_HIGH_END_LOW_COMPARE,
			GTIOR_CHANNEL_B_OUTPUT_MASK,
			GTIOR,
		},
		.duty = {
			0,
			0,
			GTCCRB,
		},
		.noise_filter = {
			NOISE_FILT_BEN|NOISE_FILT_B_P0_64,
			0,
			GTIOR,
		},
		.input_source = {
			INPUT_CAP_GTIOB_RISING_EDGE,
			0,
			GTICBSR,
		},
		.input_flag = {
			GTINTB,
			0,
			GTINTAD,
		},
	},

	/* Setting for both AB mode */
	[BOTH_AB] = {
		.phase = {
			GTIOA_OUTPUT_HIGH_END_LOW_COMPARE
			|GTIOB_OUTPUT_HIGH_END_LOW_COMPARE,
			GTIOR_CHANNEL_A_OUTPUT_MASK
			|GTIOR_CHANNEL_B_OUTPUT_MASK,
			GTIOR,
		},
		.duty = {
			0,
			0,
			0,
		},
		.noise_filter = {
			0,
			0,
			0,
		},
		.input_source = {
			0,
			0,
			0,
		},
		.input_flag = {
			0,
			0,
			0,
		},
	},
};

struct rzg2l_gpt_chip {
	struct	pwm_chip chip;
	struct	clk *clk;
	void	__iomem *mmio_base;
	spinlock_t lock;
	struct reset_control *rstc;
	wait_queue_head_t wait;
	struct mutex mutex;
	u64 snapshot[3];
	unsigned int index;
	unsigned int overflow_count, buffer_mode_count_A, buffer_mode_count_B;
	unsigned long bufferA[3];
	unsigned long bufferB[3];
	unsigned int period_ns;
	int channel;
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

static int rzg2l_gpt_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	return pm_runtime_get_sync(chip->dev);
}

static void rzg2l_gpt_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pm_runtime_put(chip->dev);
}

static int rzg2l_gpt_config(struct pwm_chip *chip, struct pwm_device *pwm,
				int duty_ns, int period_ns)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);
	unsigned long pv, dc;
	int prescale;

	if ((duty_ns < 0) || (period_ns < 0)) {
		dev_err(chip->dev, "Set time negative\n");
		return -EINVAL;
	}

	if ((duty_ns > 0) && (pc->channel == BOTH_AB)) {
		dev_err(chip->dev, "In both channel A and B mode please set duty cycle via buff\n");
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

	/* GPT setting saw-wave up-counting */
	/* Set operation GPT mode and select count clock */
	rzg2l_gpt_write_mask(pc, SAW_WAVE, GTCR_MODE_MASK, GTCR);
	rzg2l_gpt_write_mask(pc, (prescale<<24), GTCR_PRESCALE_MASK, GTCR);
	/* Set counting mode */
	rzg2l_gpt_write(pc, UP_COUNTING, GTUDDTYC); //up-counting
	/* Set period */
	rzg2l_gpt_write(pc, pv, GTPR);

	/* Enable pin output */
	rzg2l_gpt_write_mask(pc, channel_set[pc->channel].phase.value,
				channel_set[pc->channel].phase.mask, GTIOR);

	/* Set duty cycle */
	rzg2l_gpt_write(pc, dc, channel_set[pc->channel].duty.offset);

	/* Set initial value for counter */
	rzg2l_gpt_write(pc, 0, GTCNT); // reset counter value
	/* Set no buffer operation */
	rzg2l_gpt_write(pc, 0, GTBER);

	return 0;
}

static int rzg2l_gpt_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);
	int rc = 0;

	/* Start count */
	rzg2l_gpt_write_mask(pc, 1, GTCR_CST, GTCR);

	return rc;
}

static void rzg2l_gpt_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);

	/* Stop count */
	rzg2l_gpt_write_mask(pc, 0, GTCR_CST, GTCR);
}

static int
rzg2l_gpt_capture(struct pwm_chip *chip, struct pwm_device *pwm,
		struct pwm_capture *result, unsigned long timeout)
{
	int ret;
	unsigned long flags;
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);
	unsigned int effective_ticks;
	u64 high, low;

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
	if (pc->channel == BOTH_AB) {
		dev_err(chip->dev, "Input capture function not available on both A B channel mode\n");
		goto out;
	} else {
		/* Using noise filter with P0/64 clock */
		rzg2l_gpt_write(pc, channel_set[pc->channel].noise_filter.value,
				GTIOR);
		/* Select input capture source in GTICASR and GTICBSR */
		rzg2l_gpt_write(pc, channel_set[pc->channel].input_source.value,
				channel_set[pc->channel].input_source.offset);
		/* Enable input capture and overflow interrupt*/
		rzg2l_gpt_write(pc, channel_set[pc->channel].input_flag.value
					|GTINTPROV, GTINTAD);
	}

	/* Start count operation set GTCR.CST to 1 to start count operation*/
	rzg2l_gpt_write_mask(pc, 1, GTCR_CST, GTCR);

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
	/* Disable capture operation */
	rzg2l_gpt_write(pc, 0, channel_set[pc->channel].input_source.offset);

	/* Disable interrupt */
	rzg2l_gpt_write(pc, 0, GTINTAD);

	/* Stop count */
	rzg2l_gpt_write_mask(pc, 0, GTCR_CST, GTCR);

	spin_unlock_irqrestore(&pc->lock, flags);

	return ret;
}

static const struct pwm_ops rzg2l_gpt_ops = {
	.request = rzg2l_gpt_request,
	.free = rzg2l_gpt_free,
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
		pc->buffer_mode_count_A--;
		pc->buffer_mode_count_B--;

		tmp = rzg2l_gpt_read(pc, GTBER);

		if (tmp & GTCCRB_BUFFER_SINGLE) {
			rzg2l_gpt_write(pc,
				pc->bufferB[pc->buffer_mode_count_B],
				GTCCRE);
			if (pc->buffer_mode_count_B == 0)
				pc->buffer_mode_count_B = 2;
		}

		if (tmp & GTCCRB_BUFFER_DOUBLE) {
			rzg2l_gpt_write(pc,
				pc->bufferB[pc->buffer_mode_count_B],
				GTCCRF);
			if (pc->buffer_mode_count_B == 0)
				pc->buffer_mode_count_B = 3;
		}

		if (tmp & GTCCRA_BUFFER_SINGLE) {
			rzg2l_gpt_write(pc,
				pc->bufferA[pc->buffer_mode_count_A],
				GTCCRC);
			if (pc->buffer_mode_count_A == 0)
				pc->buffer_mode_count_A = 2;
		}

		if (tmp & GTCCRA_BUFFER_DOUBLE) {
			rzg2l_gpt_write(pc,
				pc->bufferA[pc->buffer_mode_count_A],
				GTCCRD);
			if (pc->buffer_mode_count_A == 0)
				pc->buffer_mode_count_A = 3;
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

static ssize_t buffA0_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;
	unsigned long prescale;

	if (pc->channel == CHANNEL_B) {
		dev_err(pc->chip.dev, "Can not set channel A in channel B mode\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val <= 0) {
		dev_err(pc->chip.dev, "Duty cycle must greater than 0\n");
		return -EINVAL;
	}

	if (pc->period_ns < val) {
		dev_err(pc->chip.dev, "Duty cycle greater than period\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->bufferA[0] = rzg2l_time_to_tick_number(pc, val, prescale);

	/*Set compare match value in GTCCRA*/
	rzg2l_gpt_write(pc, pc->bufferA[0], GTCCRA);
	/* Set no buffer operation */
	rzg2l_gpt_write_mask(pc, 0, GTCCRA_BUFFER_MASK, GTBER);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffA0_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferA[0], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buffB0_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;
	unsigned long prescale;

	if (pc->channel == CHANNEL_A) {
		dev_err(pc->chip.dev, "Can not set channel B in channel A mode\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val <= 0) {
		dev_err(pc->chip.dev, "Duty cycle must greater than 0\n");
		return -EINVAL;
	}

	if (pc->period_ns < val) {
		dev_err(pc->chip.dev, "Duty cycle greater than period\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->bufferB[0] = rzg2l_time_to_tick_number(pc, val, prescale);

	/*Set compare match value in GTCCRB*/
	rzg2l_gpt_write(pc, pc->bufferB[0], GTCCRB);
	/* Set no buffer operation */
	rzg2l_gpt_write_mask(pc, 0, GTCCRB_BUFFER_MASK, GTBER);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffB0_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferB[0], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buffA1_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;
	unsigned long prescale;

	if (pc->channel == CHANNEL_B) {
		dev_err(pc->chip.dev, "Can not set channel A in channel B mode\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val <= 0) {
		dev_err(pc->chip.dev, "Duty cycle must greater than 0\n");
		return -EINVAL;
	}

	if (pc->period_ns < val) {
		dev_err(pc->chip.dev, "Duty cycle greater than period\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->bufferA[1] = rzg2l_time_to_tick_number(pc, val, prescale);

	pc->buffer_mode_count_A = 2;

	/*Set buffer operation with CCRA in GTBER*/
	rzg2l_gpt_write_mask(pc, GTCCRA_BUFFER_SINGLE,
				GTCCRA_BUFFER_MASK, GTBER);

	/* Set buffer value for A in GTCCRC(single), GTCCRD(double)*/
	rzg2l_gpt_write(pc, pc->bufferA[1], GTCCRC);

	/* Enable overflow interrupt*/
	rzg2l_gpt_write_mask(pc, GTINTPROV, GTINTAD_GTINTPR_MASK, GTINTAD);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffA1_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferA[1], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buffB1_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;
	unsigned long prescale;

	if (pc->channel == CHANNEL_A) {
		dev_err(pc->chip.dev, "Can not set channel B in channel A mode\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val <= 0) {
		dev_err(pc->chip.dev, "Duty cycle must greater than 0\n");
		return -EINVAL;
	}

	if (pc->period_ns < val) {
		dev_err(pc->chip.dev, "Duty cycle greater than period\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->bufferB[1] = rzg2l_time_to_tick_number(pc, val, prescale);

	pc->buffer_mode_count_B = 2;

	/*Set buffer operation with CCRB in GTBER*/
	rzg2l_gpt_write_mask(pc, GTCCRB_BUFFER_SINGLE,
				GTCCRB_BUFFER_MASK, GTBER);

	/* Set buffer value for B in GTCCRE(single), GTCCRF(double)*/
	rzg2l_gpt_write(pc, pc->bufferB[1], GTCCRE);

	/* Enable overflow interrupt*/
	rzg2l_gpt_write_mask(pc, GTINTPROV, GTINTAD_GTINTPR_MASK, GTINTAD);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffB1_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferB[1], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buffA2_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;
	unsigned long prescale;

	if (pc->channel == CHANNEL_B) {
		dev_err(pc->chip.dev, "Can not set channel A in channel B mode\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val <= 0) {
		dev_err(pc->chip.dev, "Duty cycle must greater than 0\n");
		return -EINVAL;
	}

	if (pc->period_ns < val) {
		dev_err(pc->chip.dev, "Duty cycle greater than period\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->bufferA[2] = rzg2l_time_to_tick_number(pc, val, prescale);

	pc->buffer_mode_count_A = 3;

	/*Set buffer operation with CCRA in GTBER*/
	rzg2l_gpt_write_mask(pc, GTCCRA_BUFFER_DOUBLE,
				GTCCRA_BUFFER_MASK, GTBER);

	/* Set buffer value for A in GTCCRC(single), GTCCRD(double)*/
	rzg2l_gpt_write(pc, pc->bufferA[2], GTCCRD);

	/* Enable overflow interrupt*/
	rzg2l_gpt_write_mask(pc, GTINTPROV, GTINTAD_GTINTPR_MASK, GTINTAD);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffA2_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferA[2], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buffB2_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;
	unsigned long prescale;

	if (pc->channel == CHANNEL_A) {
		dev_err(pc->chip.dev, "Can not set channel B in channel A mode\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val <= 0) {
		dev_err(pc->chip.dev, "Duty cycle must greater than 0\n");
		return -EINVAL;
	}

	if (pc->period_ns < val) {
		dev_err(pc->chip.dev, "Duty cycle greater than period\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	pc->bufferB[2] = rzg2l_time_to_tick_number(pc, val, prescale);

	pc->buffer_mode_count_B = 3;

	/*Set buffer operation with CCRB in GTBER*/
	rzg2l_gpt_write_mask(pc, GTCCRB_BUFFER_DOUBLE,
				GTCCRB_BUFFER_MASK, GTBER);

	/* Set buffer value for B in GTCCRE(single), GTCCRF(double)*/
	rzg2l_gpt_write(pc, pc->bufferB[2], GTCCRF);

	/* Enable overflow interrupt*/
	rzg2l_gpt_write_mask(pc, GTINTPROV, GTINTAD_GTINTPR_MASK, GTINTAD);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffB2_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferB[2], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static DEVICE_ATTR_RW(buffA0);
static DEVICE_ATTR_RW(buffA1);
static DEVICE_ATTR_RW(buffA2);
static DEVICE_ATTR_RW(buffB0);
static DEVICE_ATTR_RW(buffB1);
static DEVICE_ATTR_RW(buffB2);

static struct attribute *buffer_attrs[] = {
	&dev_attr_buffA0.attr,
	&dev_attr_buffA1.attr,
	&dev_attr_buffA2.attr,
	&dev_attr_buffB0.attr,
	&dev_attr_buffB1.attr,
	&dev_attr_buffB2.attr,
	NULL,
};

static const struct attribute_group buffer_attr_group = {
	.attrs = buffer_attrs,
};

static int rzg2l_gpt_probe(struct platform_device *pdev)
{
	struct rzg2l_gpt_chip *rzg2l_gpt;
	struct resource *res;
	int ret, irq = 0;
	const char *read_string;

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
					&read_string);
	if (ret < 0) {
		dev_err(&pdev->dev, "Not define GPT channel\n");
		return ret;
	}

	if (!strcmp(read_string, "channel_A")) {
		rzg2l_gpt->channel = CHANNEL_A;
	} else if (!strcmp(read_string, "channel_B")) {
		rzg2l_gpt->channel = CHANNEL_B;
	} else if (!strcmp(read_string, "both_AB")) {
		rzg2l_gpt->channel = BOTH_AB;
	} else {
		dev_err(&pdev->dev, "Failed to get GPT channel\n");
		return -ENODEV;
	}

	rzg2l_gpt->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(rzg2l_gpt->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(rzg2l_gpt->clk);
	}

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

static struct pwm_device *rzg2l_gpt_dev_to_pwm_dev(struct device *dev)
{
	struct rzg2l_gpt_chip *rzg2l_gpt = dev_get_drvdata(dev);
	struct pwm_chip *chip = &rzg2l_gpt->chip;

	return &chip->pwms[0];
}

static int rzg2l_gpt_suspend(struct device *dev)
{
	struct pwm_device *pwm = rzg2l_gpt_dev_to_pwm_dev(dev);

	if (!test_bit(PWMF_REQUESTED, &pwm->flags))
		return 0;

	pm_runtime_put(dev);

	return 0;
}

static int rzg2l_gpt_resume(struct device *dev)
{
	struct pwm_device *pwm = rzg2l_gpt_dev_to_pwm_dev(dev);

	if (!test_bit(PWMF_REQUESTED, &pwm->flags))
		return 0;

	pm_runtime_get_sync(dev);

	rzg2l_gpt_config(pwm->chip, pwm, pwm->state.duty_cycle,
			pwm->state.period);
	if (pwm_is_enabled(pwm))
		rzg2l_gpt_enable(pwm->chip, pwm);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rzg2l_gpt_pm_ops, rzg2l_gpt_suspend, rzg2l_gpt_resume);

static struct platform_driver rzg2l_gpt_driver = {
	.probe = rzg2l_gpt_probe,
	.remove = rzg2l_gpt_remove,
	.driver = {
		.name = "gpt-rzg2l",
		.owner = THIS_MODULE,
		.pm	= &rzg2l_gpt_pm_ops,
		.of_match_table = of_match_ptr(rzg2l_gpt_of_table),
	},
};

module_platform_driver(rzg2l_gpt_driver);

MODULE_DESCRIPTION("Renesas RZG2L GPT Driver");
MODULE_LICENSE("GPL v2");
