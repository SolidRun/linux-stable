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
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/bitfield.h>
#include <linux/iio/iio.h>

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
#define GTDTCR		0x0088
#define GTDVU		0x008C
#define GTDVD		0x0090

#define SAW_WAVE		(0x00<<16)
#define SAW_WAVE_ONE_SHOT	(0x01<<16)
#define TRIANGLE_WAVE_MODE1	(0x04<<16)
#define TRIANGLE_WAVE_MODE2	(0x05<<16)
#define TRIANGLE_WAVE_MODE3	(0x06<<16)
#define GTCR_MODE_MASK		(0x7<<16)
#define GTCR_PRESCALE_MASK	(0x7<<24)
#define GTIOR_CHANNEL_B_OUTPUT_MASK	(0x01FF<<16)
#define GTIOB_OUTPUT_HIGH_END_TOGGLE_COMPARE	(0x11B<<16)
#define GTIOB_OUTPUT_LOW_END_TOGGLE_COMPARE	(0x147<<16)
/* GTIOR.GTIOB = 11001 */
/* GTIOR.OBE = 1 */
#define GTIOR_CHANNEL_A_OUTPUT_MASK	0x01FF
#define GTIOA_OUTPUT_HIGH_END_TOGGLE_COMPARE	0x11B
#define GTIOA_OUTPUT_LOW_END_TOGGLE_COMPARE	0x147
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
#define NOISE_FILT_B_P0_64	(0x3 << 30)
#define NOISE_FILT_AEN	(1<<13)
#define NOISE_FILT_A_P0_64	(0x3 << 14)
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
#define GTIOR_CHANNEL_B_OUTPUT_DISABLE_MASK	(3<<25)
#define GTIOB_OUTPUT_DISABLE	(1 << 25)
#define GTIOR_CHANNEL_A_OUTPUT_DISABLE_MASK	(3<<9)
#define GTIOA_OUTPUT_DISABLE	(1 << 9)
#define GTINTAD_OUTPUT_DISABLE_GRP_MASK		(3 << 24)
#define GRPA	(0 << 24)
#define GRPB	(1 << 24)
#define GRPC	(2 << 24)
#define GRPD	(3 << 24)
#define GTINTAD_OUTPUT_DISABLE_POEG_MASK	(7 << 28)
#define OUTPUT_DISABLE_SAME_LEVEL_HIGH		(1 << 29)
#define OUTPUT_DISABLE_SAME_LEVEL_LOW		(1 << 30)
#define OUTPUT_DISABLE_DEADTIME_ERROR		(1 << 28)
#define GTDTCR_DEADTIME_MODE			(0x01)
#define GTBER_BUFFER_DEADTIME			(1 << 22)

#define POEGG		0x0000

#define PIDE		(1 << 4)
#define EN_NFEN		(1 << 29)
#define SSF		(1 << 3)
#define PIDF		(1 << 0)
#define ST		(1 << 16)
#define IOCE		(1 << 5)
#define IOCF		(1 << 1)

#define NR_GPT_OPERATION	6

static const char *const gpt_operation_enum[] = {
	"normal_output",
	"single_buffer_output",
	"double_buffer_output",
	"deadtime_output",
	"input_capture",
	"counting_input",
};

enum {
	NORMAL_OUTPUT,
	SINGLE_BUFFER_OUTPUT,
	DOUBLE_BUFFER_OUTPUT,
	DEADTIME_OUTPUT,
	INPUT_CAPTURE,
	COUNTING_INPUT,
};

enum {
	CHANNEL_A,
	CHANNEL_B,
	BOTH_AB,
	NR_CHANNEL,
};

struct reg {
	u32 value, mask, offset;
};

struct polarity_reg {
	u32 polar[2];
	u32 mask, offset;
};

struct set_params {
	struct polarity_reg phase;
	struct reg duty;
	struct reg noise_filter;
	struct reg input_source;
	struct reg input_flag;
	struct reg output_disable;
};

/* Parameters which we must set manually via sysfs or not use
 * is 0 in default
 */
static const struct set_params channel_set[NR_CHANNEL] = {
	/* Setting for channel A mode */
	[CHANNEL_A] = {
		.phase = {
			.polar = {
				GTIOA_OUTPUT_HIGH_END_TOGGLE_COMPARE,
				GTIOA_OUTPUT_LOW_END_TOGGLE_COMPARE,
			},
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
		.output_disable = {
			GTIOA_OUTPUT_DISABLE,
			GTIOR_CHANNEL_A_OUTPUT_DISABLE_MASK,
			GTIOR,
		},
	},

	/* Setting for channel B mode */
	[CHANNEL_B] = {
		.phase = {
			.polar = {
				GTIOB_OUTPUT_HIGH_END_TOGGLE_COMPARE,
				GTIOB_OUTPUT_LOW_END_TOGGLE_COMPARE,
			},
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
		.output_disable = {
			GTIOB_OUTPUT_DISABLE,
			GTIOR_CHANNEL_B_OUTPUT_DISABLE_MASK,
			GTIOR,
		},
	},

	/* Setting for both AB mode */
	[BOTH_AB] = {
		.phase = {
			.polar = {
				GTIOA_OUTPUT_HIGH_END_TOGGLE_COMPARE
				|GTIOB_OUTPUT_HIGH_END_TOGGLE_COMPARE,
				GTIOA_OUTPUT_LOW_END_TOGGLE_COMPARE
				|GTIOB_OUTPUT_LOW_END_TOGGLE_COMPARE,
			},
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
		.output_disable = {
			GTIOA_OUTPUT_DISABLE
			|GTIOB_OUTPUT_DISABLE,
			GTIOR_CHANNEL_A_OUTPUT_DISABLE_MASK
			|GTIOR_CHANNEL_B_OUTPUT_DISABLE_MASK,
			GTIOR,
		},
	},
};

static const char *rzg2l_gpt_POEGs[5] = {
	"NOT_USE",
};

enum {
	NOT_USE,
};

struct POEG_params {
	u32 poeg;
	struct platform_device *poeg_dev;
};

static struct POEG_params POEG_mode_set[5] = {
	[NOT_USE] = {
		.poeg = 0,
		.poeg_dev = NULL,
	},
};

static struct POEG_params POEG_mode_set_A = {
	.poeg = GRPA,
};

static struct POEG_params POEG_mode_set_B = {
	.poeg = GRPB,
};

static struct POEG_params POEG_mode_set_C = {
	.poeg = GRPC,
};

static struct POEG_params POEG_mode_set_D = {
	.poeg = GRPD,
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
	enum pwm_polarity channel_polar[NR_CHANNEL];
	unsigned int period_ns;
	int channel;
	unsigned int gpt_operation;
	unsigned long deadtime_first, deadtime_second;
	u32 poeg;
	bool enable_clock;
	u32 counter_mode, reset_counter;
	u32 pulse_number;
	u32 GTIOR_val;
	u32 GTINTAD_val;
};

#if IS_BUILTIN(CONFIG_POEG_RZG2L)
extern void rzg2l_poeg_clear_bit_export(struct platform_device *poeg_dev,
				u32 data, unsigned int offset);
#endif

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

static u32 rzg2l_gpt_read_mask(struct rzg2l_gpt_chip *pc, u32 mask,
				unsigned int offset)
{
	u32 tmp = 0;

	tmp = ioread32(pc->mmio_base + offset);
	tmp &= mask;

	return tmp;
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

static void rzg2l_reset_period_and_duty(struct rzg2l_gpt_chip *pc)
{
	rzg2l_gpt_write(pc, 0, GTCR);
	rzg2l_gpt_write(pc, 0, GTST);
	rzg2l_gpt_write(pc, 0, GTUDDTYC);
	rzg2l_gpt_write(pc, 0, GTIOR);
	rzg2l_gpt_write(pc, 0, GTINTAD);
	rzg2l_gpt_write(pc, 0, GTBER);
	rzg2l_gpt_write(pc, 0, GTDTCR);
	rzg2l_gpt_write(pc, 0, GTCNT);
	rzg2l_gpt_write(pc, 0, GTICASR);
	rzg2l_gpt_write(pc, 0, GTICBSR);
	rzg2l_gpt_write(pc, 0, GTCCRA);
	rzg2l_gpt_write(pc, 0, GTCCRB);
	rzg2l_gpt_write(pc, 0, GTCCRC);
	rzg2l_gpt_write(pc, 0, GTCCRD);
	rzg2l_gpt_write(pc, 0, GTCCRE);
	rzg2l_gpt_write(pc, 0, GTCCRF);
	rzg2l_gpt_write(pc, 0, GTPR);
	rzg2l_gpt_write(pc, 0, GTDVU);
	rzg2l_gpt_write(pc, 0, GTDVD);
	rzg2l_gpt_write(pc, 0, GTSSR);
	rzg2l_gpt_write(pc, 0, GTPSR);
	rzg2l_gpt_write(pc, 0, GTCSR);
	rzg2l_gpt_write(pc, 0, GTUPSR);
	rzg2l_gpt_write(pc, 0, GTDNSR);
	pc->deadtime_first = 0;
	pc->deadtime_second = 0;
	pc->bufferA[0] = 0;
	pc->bufferA[1] = 0;
	pc->bufferA[2] = 0;
	pc->bufferB[0] = 0;
	pc->bufferB[1] = 0;
	pc->bufferB[2] = 0;
	pc->counter_mode = 0;
	pc->reset_counter = 0;
}

static int rzg2l_gpt_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);

	pc->enable_clock = 1;

	reset_control_deassert(pc->rstc);

	return pm_runtime_get_sync(chip->dev);
}

static void rzg2l_gpt_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);

	pc->enable_clock = 0;

	pm_runtime_put(chip->dev);

	reset_control_assert(pc->rstc);
}

static int rzg2l_gpt_config(struct pwm_chip *chip, struct pwm_device *pwm,
				int duty_ns, int period_ns)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);
	unsigned long pv, dc;
	int prescale;

	if ((pc->gpt_operation == INPUT_CAPTURE) || (pc->gpt_operation == COUNTING_INPUT)) {
		dev_err(chip->dev, "This operation not use this config\n");
		return -EINVAL;
	}

	if ((duty_ns < 0) || (period_ns < 0)) {
		dev_err(chip->dev, "Set time negative\n");
		return -EINVAL;
	}

	if ((duty_ns > 0) && (pc->channel == BOTH_AB)) {
		dev_err(chip->dev, "In both channel A and B output please set duty cycle via buff\n");
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
	if (pc->gpt_operation != DEADTIME_OUTPUT)
		rzg2l_gpt_write_mask(pc, SAW_WAVE, GTCR_MODE_MASK, GTCR);

	/* Set prescale for GPT */
	rzg2l_gpt_write_mask(pc, (prescale<<24), GTCR_PRESCALE_MASK, GTCR);
	/* Set counting mode */
	rzg2l_gpt_write(pc, UP_COUNTING, GTUDDTYC); //up-counting
	/* Set period */
	rzg2l_gpt_write(pc, pv, GTPR);

	/* Enable pin output */
	rzg2l_gpt_write_mask(pc,
	channel_set[pc->channel].phase.polar[pc->channel_polar[pc->channel]],
	channel_set[pc->channel].phase.mask, GTIOR);

	/* Enable overflow interrupt*/
	if ((!pc->poeg) && (pc->gpt_operation == NORMAL_OUTPUT)) {
		rzg2l_gpt_write_mask(pc, 0, GTINTAD_GTINTPR_MASK, GTINTAD);
	} else {
		rzg2l_gpt_write_mask(pc,
				GTINTPROV, GTINTAD_GTINTPR_MASK, GTINTAD);
	}

	/* Set duty cycle */
	rzg2l_gpt_write(pc, dc, channel_set[pc->channel].duty.offset);

	/* Set initial value for counter */
	rzg2l_gpt_write(pc, 0, GTCNT); // reset counter value

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

	if (pc->gpt_operation != INPUT_CAPTURE) {
		dev_err(chip->dev, "Only input capture operation use this config\n");
		return -EINVAL;
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
	if (pc->channel == BOTH_AB) {
		dev_err(chip->dev, "Input capture function not available on both A B channel output\n");
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

static int rzg2l_gpt_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
	enum pwm_polarity polarity)
{
	struct rzg2l_gpt_chip *pc = to_rzg2l_gpt_chip(chip);

	if (pc->channel != BOTH_AB) {
		pc->channel_polar[pc->channel] = polarity;

		rzg2l_gpt_write_mask(pc,
	channel_set[pc->channel].phase.polar[pc->channel_polar[pc->channel]],
		channel_set[pc->channel].phase.mask, GTIOR);
	} else {
		dev_err(chip->dev, "Set polarity via polarityA or polarityB\n");
		return -EINVAL;
	}

	return 0;
}

static const struct pwm_ops rzg2l_gpt_ops = {
	.request = rzg2l_gpt_request,
	.free = rzg2l_gpt_free,
	.capture = rzg2l_gpt_capture,
	.config = rzg2l_gpt_config,
	.set_polarity = rzg2l_gpt_set_polarity,
	.enable = rzg2l_gpt_enable,
	.disable = rzg2l_gpt_disable,
	.owner = THIS_MODULE,
};

static int rzg2l_gpt_cnt_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val < 0 || val > 1)
			return -EINVAL;

		rzg2l_gpt_write_mask(pc, val, GTCR_CST, GTCR);

		return 0;
	case IIO_CHAN_INFO_RAW:
		if (val < 0)
			return -EINVAL;

		if (rzg2l_gpt_read_mask(pc, GTCR_CST, GTCR))
			return -EBUSY;

		rzg2l_gpt_write(pc, val, GTCNT);

		return 0;
	default:
		return -EINVAL;
	}
}

static int rzg2l_gpt_cnt_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);
	u32 dat;

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		dat = rzg2l_gpt_read(pc, GTCNT);
		*val = dat;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		dat = rzg2l_gpt_read_mask(pc, GTCR_CST, GTCR);
		*val = dat;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info rzg2l_gpt_cnt_iio_info = {
	.read_raw = rzg2l_gpt_cnt_read_raw,
	.write_raw = rzg2l_gpt_cnt_write_raw,
};

static const char *const rzg2l_gpt_counter_modes[] = {
	"mode-1",
	"mode-2A",
	"mode-2B",
	"mode-2C",
	"mode-3A",
	"mode-3B",
	"mode-3C",
	"mode-4",
	"mode-5A",
	"mode-5B",
};

enum {
	MODE_1,
	MODE_2A,
	MODE_2B,
	MODE_2C,
	MODE_3A,
	MODE_3B,
	MODE_3C,
	MODE_4,
	MODE_5A,
	MODE_5B,
	NR_MODE,
};

struct reg_full {
	u32 value, offset;
};

struct counter_mode_params {
	struct reg_full up_count;
	struct reg_full down_count;
};

static const struct counter_mode_params mode_set[NR_MODE] = {
	[MODE_1] = {
		.up_count	= { 0x00006900, GTUPSR },
		.down_count	= { 0x00009600, GTDNSR },
	},
	[MODE_2A] = {
		.up_count	= { 0x00000800, GTUPSR },
		.down_count	= { 0x00000400, GTDNSR },
	},
	[MODE_2B] = {
		.up_count	= { 0x00000200, GTUPSR },
		.down_count	= { 0x00000100, GTDNSR },
	},
	[MODE_2C] = {
		.up_count	= { 0x00000A00, GTUPSR },
		.down_count	= { 0x00000500, GTDNSR },
	},
	[MODE_3A] = {
		.up_count	= { 0x00000800, GTUPSR },
		.down_count	= { 0x00008000, GTDNSR },
	},
	[MODE_3B] = {
		.up_count	= { 0x00000200, GTUPSR },
		.down_count	= { 0x00002000, GTDNSR },
	},
	[MODE_3C] = {
		.up_count	= { 0x00000A00, GTUPSR },
		.down_count	= { 0x0000A000, GTDNSR },
	},
	[MODE_4] = {
		.up_count	= { 0x00006000, GTUPSR },
		.down_count	= { 0x00009000, GTDNSR },
	},
	[MODE_5A] = {
		.up_count	= { 0x00000C00, GTUPSR },
		.down_count	= { 0x00000000, GTDNSR },
	},
	[MODE_5B] = {
		.up_count	= { 0x0000C000, GTUPSR },
		.down_count	= { 0x00000000, GTDNSR },
	},
};

static int rzg2l_gpt_get_counter_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	return pc->counter_mode;
}

static int rzg2l_gpt_set_counter_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   unsigned int type)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	if (rzg2l_gpt_read_mask(pc, GTCR_CST, GTCR))
		return -EBUSY;

	pc->counter_mode = type;

	rzg2l_gpt_write(pc, mode_set[type].up_count.value, GTUPSR);
	rzg2l_gpt_write(pc, mode_set[type].down_count.value, GTDNSR);
	/* Reset counter when set mode */
	rzg2l_gpt_write(pc, 0, GTCNT);

	return 0;
}

static const struct iio_enum rzg2l_gpt_counter_mode_en = {
	.items = rzg2l_gpt_counter_modes,
	.num_items = ARRAY_SIZE(rzg2l_gpt_counter_modes),
	.get = rzg2l_gpt_get_counter_mode,
	.set = rzg2l_gpt_set_counter_mode,
};


static const char *rzg2l_gpt_reset_counters[5] = {
	"NOT_USE",
};

struct reset_counter_params {
	struct reg_full gtssr;
	struct reg_full gtpsr;
	struct reg_full gtcsr;
};

static struct reset_counter_params reset_counter_mode_set[5] = {
	[NOT_USE] = {
		.gtssr  = { 0x00000000, GTSSR },
		.gtpsr  = { 0x00000000, GTPSR },
		.gtcsr  = { 0x00000000, GTCSR },
	},
};

static struct reset_counter_params reset_counter_mode_set_A = {
	.gtssr  = { 0x00000002, GTSSR },
	.gtpsr  = { 0x00000001, GTPSR },
	.gtcsr  = { 0x00000001, GTCSR },
};

static struct reset_counter_params reset_counter_mode_set_B = {
	.gtssr  = { 0x00000008, GTSSR },
	.gtpsr  = { 0x00000004, GTPSR },
	.gtcsr  = { 0x00000004, GTCSR },
};

static struct reset_counter_params reset_counter_mode_set_C = {
	.gtssr  = { 0x00000020, GTSSR },
	.gtpsr  = { 0x00000010, GTPSR },
	.gtcsr  = { 0x00000010, GTCSR },
};

static struct reset_counter_params reset_counter_mode_set_D = {
	.gtssr  = { 0x00000080, GTSSR },
	.gtpsr  = { 0x00000040, GTPSR },
	.gtcsr  = { 0x00000040, GTCSR },
};

static int rzg2l_gpt_get_reset_counter(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	return pc->reset_counter;
}

static int rzg2l_gpt_set_reset_counter(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int type)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	pc->reset_counter = type;

	rzg2l_gpt_write(pc, reset_counter_mode_set[type].gtssr.value, GTSSR);
	rzg2l_gpt_write(pc, reset_counter_mode_set[type].gtpsr.value, GTPSR);
	rzg2l_gpt_write(pc, reset_counter_mode_set[type].gtcsr.value, GTCSR);

	return 0;
}

static const struct iio_enum rzg2l_gpt_reset_counter_en = {
	.items = rzg2l_gpt_reset_counters,
	.num_items = ARRAY_SIZE(rzg2l_gpt_reset_counters),
	.get = rzg2l_gpt_get_reset_counter,
	.set = rzg2l_gpt_set_reset_counter,
};

static ssize_t rzg2l_gpt_cnt_get_counter_preset(struct iio_dev *indio_dev,
						uintptr_t private,
						const struct iio_chan_spec *chan,
						char *buf)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);
	u32 tmp = 0;

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	tmp = rzg2l_gpt_read(pc, GTPR);

	return snprintf(buf, PAGE_SIZE, "%u\n", tmp);
}

static ssize_t rzg2l_gpt_cnt_set_counter_preset(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  const char *buf, size_t len)
{
	struct rzg2l_gpt_chip *pc = iio_priv(indio_dev);
	int ret, tmp = 0;

	if (pc->gpt_operation != COUNTING_INPUT) {
		dev_err(pc->chip.dev, "Must in counting input operation to use this config\n");
		return -EINVAL;
	}

	if (rzg2l_gpt_read_mask(pc, GTCR_CST, GTCR))
		return -EBUSY;

	ret = kstrtoint(buf, 0, &tmp);
	if (ret)
		return ret;

	if ((tmp > (BIT(31)-1)) || (tmp < 0))
		return -EINVAL;

	rzg2l_gpt_write(pc, tmp, GTPR);
	/* Reset counter when set preset */
	rzg2l_gpt_write(pc, 0, GTCNT);

	return len;
}

static const struct iio_chan_spec_ext_info rzg2l_gpt_cnt_ext_info[] = {
	{
		.name = "counter_preset",
		.shared = IIO_SEPARATE,
		.read = rzg2l_gpt_cnt_get_counter_preset,
		.write = rzg2l_gpt_cnt_set_counter_preset,
	},
	IIO_ENUM("counter_mode", IIO_SEPARATE,
		 &rzg2l_gpt_counter_mode_en),
	IIO_ENUM_AVAILABLE("counter_mode", &rzg2l_gpt_counter_mode_en),
	IIO_ENUM("reset_counter", IIO_SEPARATE,
		&rzg2l_gpt_reset_counter_en),
	IIO_ENUM_AVAILABLE("reset_counter", &rzg2l_gpt_reset_counter_en),
	{}
};

static const struct iio_chan_spec rzg2l_gpt_cnt_channels = {
	.type = IIO_COUNT,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_ENABLE),
	.ext_info = rzg2l_gpt_cnt_ext_info,
	.indexed = 1,
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

#if IS_BUILTIN(CONFIG_POEG_RZG2L)
		if (pc->poeg) {
			/*Clear input edge flag*/
			rzg2l_poeg_clear_bit_export(POEG_mode_set[pc->poeg].poeg_dev, PIDF, POEGG);
			/*Clear GPT disable flag*/
			rzg2l_poeg_clear_bit_export(POEG_mode_set[pc->poeg].poeg_dev, IOCF, POEGG);

		}
#endif

		if (pc->gpt_operation == DEADTIME_OUTPUT) {
			rzg2l_gpt_write(pc, pc->bufferA[1], GTCCRC);
			rzg2l_gpt_write(pc, pc->bufferA[2], GTCCRD);
		}

		if (pc->pulse_number) {
			pc->pulse_number--;
			if (!pc->pulse_number) {
				/* Stop count */
				rzg2l_gpt_write_mask(pc, 0, GTCR_CST, GTCR);
				pc->chip.pwms[0].state.enabled = 0;
				if ((!pc->poeg) &&
					(pc->gpt_operation == NORMAL_OUTPUT))
					rzg2l_gpt_write_mask(pc, 0,
						GTINTAD_GTINTPR_MASK, GTINTAD);
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

static ssize_t buffA0_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;
	unsigned long prescale;

	if (pc->channel == CHANNEL_B) {
		dev_err(pc->chip.dev, "Can not set channel A in channel B output\n");
		return -EINVAL;
	}

	if ((pc->gpt_operation != NORMAL_OUTPUT) &&
		(pc->gpt_operation != SINGLE_BUFFER_OUTPUT) &&
		(pc->gpt_operation != DOUBLE_BUFFER_OUTPUT)) {
		dev_err(pc->chip.dev, "This operation not use this config\n");
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

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffA0_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale, read_data;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	read_data = rzg2l_gpt_read(pc, GTCCRA);

	if (pc->gpt_operation == DEADTIME_OUTPUT)
		time_ns = rzg2l_tick_number_to_time(pc, read_data, prescale);
	else
		time_ns = rzg2l_tick_number_to_time(pc,
						pc->bufferA[0], prescale);

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
		dev_err(pc->chip.dev, "Can not set channel B in channel A output\n");
		return -EINVAL;
	}

	if ((pc->gpt_operation != NORMAL_OUTPUT) &&
		(pc->gpt_operation != SINGLE_BUFFER_OUTPUT) &&
		(pc->gpt_operation != DOUBLE_BUFFER_OUTPUT)) {
		dev_err(pc->chip.dev, "This operation not use this config\n");
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

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t buffB0_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale, read_data;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	read_data = rzg2l_gpt_read(pc, GTCCRB);

	if (pc->gpt_operation == DEADTIME_OUTPUT)
		time_ns = rzg2l_tick_number_to_time(pc, read_data, prescale);
	else
		time_ns = rzg2l_tick_number_to_time(pc,
						pc->bufferB[0], prescale);

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
		dev_err(pc->chip.dev, "Can not set channel A in channel B output\n");
		return -EINVAL;
	}

	if ((pc->gpt_operation != DEADTIME_OUTPUT) &&
		(pc->gpt_operation != SINGLE_BUFFER_OUTPUT) &&
		(pc->gpt_operation != DOUBLE_BUFFER_OUTPUT)) {
		dev_err(pc->chip.dev, "This operation not use this config\n");
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

	/* Set buffer value for A in GTCCRC(single), GTCCRD(double)*/
	/* In deadtime mode GTCCRC is first compare */
	rzg2l_gpt_write(pc, pc->bufferA[1], GTCCRC);

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
		dev_err(pc->chip.dev, "Can not set channel B in channel A output\n");
		return -EINVAL;
	}

	if ((pc->gpt_operation != SINGLE_BUFFER_OUTPUT) &&
		(pc->gpt_operation != DOUBLE_BUFFER_OUTPUT)) {
		dev_err(pc->chip.dev, "This operation not use this config\n");
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

	/* Set buffer value for B in GTCCRE(single), GTCCRF(double)*/
	rzg2l_gpt_write(pc, pc->bufferB[1], GTCCRE);

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

	if (pc->gpt_operation == DEADTIME_OUTPUT) {
		dev_err(pc->chip.dev, "In deadtime operation, read B via B0\n");
		return -EINVAL;
	}

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferB[1], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t buffA2_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret, time_A1;
	unsigned long prescale;

	if (pc->channel == CHANNEL_B) {
		dev_err(pc->chip.dev, "Can not set channel A in channel B output\n");
		return -EINVAL;
	}

	if ((pc->gpt_operation != DEADTIME_OUTPUT) &&
		(pc->gpt_operation != DOUBLE_BUFFER_OUTPUT)) {
		dev_err(pc->chip.dev, "This operation not use this config\n");
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

	if (pc->gpt_operation == DEADTIME_OUTPUT) {
		time_A1 = rzg2l_tick_number_to_time(pc,
						pc->bufferA[1], prescale);
		if (time_A1 > val) {
			dev_err(pc->chip.dev, "In deadtime, A2 must greater than A1\n");
			goto out;
		}
	}

	pc->bufferA[2] = rzg2l_time_to_tick_number(pc, val, prescale);

	/* Set buffer value for A in GTCCRC(single), GTCCRD(double)*/
	rzg2l_gpt_write(pc, pc->bufferA[2], GTCCRD);

out:
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
		dev_err(pc->chip.dev, "Can not set channel B in channel A output\n");
		return -EINVAL;
	}

	if (pc->gpt_operation != DOUBLE_BUFFER_OUTPUT) {
		dev_err(pc->chip.dev, "This operation not use this config\n");
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

	/* Set buffer value for B in GTCCRE(single), GTCCRF(double)*/
	rzg2l_gpt_write(pc, pc->bufferB[2], GTCCRF);

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

	if (pc->gpt_operation == DEADTIME_OUTPUT) {
		dev_err(pc->chip.dev, "In deadtime operation, read B via B0\n");
		return -EINVAL;
	}

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	time_ns = rzg2l_tick_number_to_time(pc, pc->bufferB[2], prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t polarityA_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int ret;

	if (pc->channel != BOTH_AB) {
		dev_err(pc->chip.dev, "Only use in both AB output\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	if (sysfs_streq(buf, "normal")) {
		pc->channel_polar[CHANNEL_A] = PWM_POLARITY_NORMAL;
	} else if (sysfs_streq(buf, "inversed")) {
		pc->channel_polar[CHANNEL_A] = PWM_POLARITY_INVERSED;
	} else {
		dev_err(pc->chip.dev, "Invalid value\n");
		ret = -EINVAL;
	}

	rzg2l_gpt_write_mask(pc,
	channel_set[CHANNEL_A].phase.polar[pc->channel_polar[CHANNEL_A]],
	channel_set[CHANNEL_A].phase.mask, GTIOR);

	mutex_unlock(&pc->mutex);

	return count;
}

static ssize_t polarityA_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	const char *polarity = "unknown";

	switch (pc->channel_polar[CHANNEL_A]) {
	case PWM_POLARITY_NORMAL:
		polarity = "normal";
		break;
	case PWM_POLARITY_INVERSED:
		polarity = "inversed";
		break;
	}

	return sprintf(buf, "%s\n", polarity);
}

static ssize_t polarityB_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int ret;

	if (pc->channel != BOTH_AB) {
		dev_err(pc->chip.dev, "Only use in both AB output\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	if (sysfs_streq(buf, "normal")) {
		pc->channel_polar[CHANNEL_B] = PWM_POLARITY_NORMAL;
	} else if (sysfs_streq(buf, "inversed")) {
		pc->channel_polar[CHANNEL_B] = PWM_POLARITY_INVERSED;
	} else {
		dev_err(pc->chip.dev, "Invalid value\n");
		ret = -EINVAL;
	}

	rzg2l_gpt_write_mask(pc,
	channel_set[CHANNEL_B].phase.polar[pc->channel_polar[CHANNEL_B]],
	channel_set[CHANNEL_B].phase.mask, GTIOR);

	mutex_unlock(&pc->mutex);

	return count;
}

static ssize_t polarityB_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	const char *polarity = "unknown";

	switch (pc->channel_polar[CHANNEL_B]) {
	case PWM_POLARITY_NORMAL:
		polarity = "normal";
		break;
	case PWM_POLARITY_INVERSED:
		polarity = "inversed";
		break;
	}

	return sprintf(buf, "%s\n", polarity);
}

static ssize_t deadtime_first_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val;
	int ret;
	unsigned long prescale;

	if (pc->gpt_operation != DEADTIME_OUTPUT) {
		dev_err(pc->chip.dev, "Must in deadtime operation to set\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0) {
		dev_err(pc->chip.dev, "Deadtime first must greater than 0\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	if (!val)
		pc->deadtime_first = 0;
	else
		pc->deadtime_first = rzg2l_time_to_tick_number(pc,
							val, prescale);

	/* Set buffer value for deadtime first half in GTDVU*/
	rzg2l_gpt_write(pc, pc->deadtime_first, GTDVU);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t deadtime_first_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;

	if (!pc->deadtime_first)
		time_ns = 0;
	else
		time_ns = rzg2l_tick_number_to_time(pc,
					pc->deadtime_first, prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t deadtime_second_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val;
	int ret;
	unsigned long prescale;

	if (pc->gpt_operation != DEADTIME_OUTPUT) {
		dev_err(pc->chip.dev, "Must in deadtime operation to set\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0) {
		dev_err(pc->chip.dev, "Deadtime second must greater than 0\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;
	if (!val)
		pc->deadtime_second = 0;
	else
		pc->deadtime_second = rzg2l_time_to_tick_number(pc,
							val, prescale);

	/* Set buffer value for deadtime second half in GTDVD*/
	rzg2l_gpt_write(pc, pc->deadtime_second, GTDVD);

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t deadtime_second_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	unsigned long prescale;
	unsigned long long time_ns;

	prescale = rzg2l_gpt_read(pc, GTCR) >> 24;

	if (pc->gpt_operation != DEADTIME_OUTPUT)
		time_ns = 0;
	else
		time_ns = rzg2l_tick_number_to_time(pc,
				pc->deadtime_second, prescale);

	return sprintf(buf, "%llu\n", time_ns);
}

static ssize_t gpt_operation_available_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int i;
	size_t len = 0;

	for (i = 0; i < NR_GPT_OPERATION; ++i)
		len += sysfs_emit_at(buf, len, "%s ", gpt_operation_enum[i]);

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}

static void gpt_set_operation_mode(struct rzg2l_gpt_chip *pc)
{
	switch (pc->gpt_operation) {
	case NORMAL_OUTPUT:
		/* Set no buffer operation */
		rzg2l_gpt_write_mask(pc, 0, GTCCRA_BUFFER_MASK, GTBER);
		rzg2l_gpt_write_mask(pc, 0, GTCCRB_BUFFER_MASK, GTBER);
		break;
	case SINGLE_BUFFER_OUTPUT:
		pc->buffer_mode_count_A = 2;
		pc->buffer_mode_count_B = 2;
		/*Set buffer operation with CCRA in GTBER*/
		rzg2l_gpt_write_mask(pc, GTCCRA_BUFFER_SINGLE,
					GTCCRA_BUFFER_MASK, GTBER);
		/*Set buffer operation with CCRB in GTBER*/
		rzg2l_gpt_write_mask(pc, GTCCRB_BUFFER_SINGLE,
					GTCCRB_BUFFER_MASK, GTBER);
		break;
	case DOUBLE_BUFFER_OUTPUT:
		pc->buffer_mode_count_A = 3;
		pc->buffer_mode_count_B = 3;
		/*Set buffer operation with CCRA in GTBER*/
		rzg2l_gpt_write_mask(pc, GTCCRA_BUFFER_DOUBLE,
					GTCCRA_BUFFER_MASK, GTBER);
		/*Set buffer operation with CCRB in GTBER*/
		rzg2l_gpt_write_mask(pc, GTCCRB_BUFFER_DOUBLE,
					GTCCRB_BUFFER_MASK, GTBER);
		break;
	case DEADTIME_OUTPUT:
		/* GPT setting saw-wave one shot up-counting */
		rzg2l_gpt_write_mask(pc, SAW_WAVE_ONE_SHOT, GTCR_MODE_MASK, GTCR);
		/* Set buffer deadtime */
		rzg2l_gpt_write(pc, GTBER_BUFFER_DEADTIME, GTBER);
		/* Enable deadtime mode */
		rzg2l_gpt_write(pc, GTDTCR_DEADTIME_MODE, GTDTCR);
		break;
	case INPUT_CAPTURE:
		break;
	case COUNTING_INPUT:
		/* Saw wave mode */
		rzg2l_gpt_write_mask(pc, SAW_WAVE, GTCR_MODE_MASK, GTCR);
		/* Maximum frequency*/
		rzg2l_gpt_write_mask(pc, 0, GTCR_PRESCALE_MASK, GTCR);
		/* Default period */
		rzg2l_gpt_write(pc, GTPR_MAX_VALUE, GTPR);
		/* Set initial value for counter */
		rzg2l_gpt_write(pc, 0, GTCNT); // reset counter value
		/* Using noise filter with P0/64 clock */
		rzg2l_gpt_write(pc, channel_set[pc->channel].noise_filter.value,
					GTIOR);
		/* Default counting mode */
		rzg2l_gpt_write(pc, mode_set[MODE_1].up_count.value, GTUPSR);
		rzg2l_gpt_write(pc, mode_set[MODE_1].down_count.value, GTDNSR);
		/* Default preset */
		rzg2l_gpt_write(pc, BIT(31)-1, GTPR);
		break;
	}
}

static ssize_t gpt_operation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int ret;

	if (!pc->enable_clock) {
		dev_err(pc->chip.dev, "PWM clock is not enabled. Please export pwm module first.\n");
		return -EINVAL;
	}

	ret = sysfs_match_string(gpt_operation_enum, buf);
	if (ret < 0)
		return ret;

	if (pc->channel != BOTH_AB)
		if ((ret == COUNTING_INPUT) || (ret == DEADTIME_OUTPUT)) {
			dev_err(pc->chip.dev, "This operation must set in bothAB output\n");
			return -EINVAL;
		}

	mutex_lock(&pc->mutex);

	/* Reset registers to prevent conflict setting between modes */
	rzg2l_reset_period_and_duty(pc);
	pc->chip.pwms[0].state.period = 0;
	pc->chip.pwms[0].state.duty_cycle = 0;
	pc->chip.pwms[0].state.enabled = 0;
	pc->chip.pwms[0].state.polarity = 0;

	pc->gpt_operation = ret;

	gpt_set_operation_mode(pc);

	mutex_unlock(&pc->mutex);

	return count;
}

static ssize_t gpt_operation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%s\n", gpt_operation_enum[pc->gpt_operation]);
}

static ssize_t POEG_available_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int i;
	size_t len = 0;

	for (i = 0; i < 5; ++i)
		len += sysfs_emit_at(buf, len, "%s ", rzg2l_gpt_POEGs[i]);

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}

static ssize_t POEG_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int ret;

	ret = sysfs_match_string(rzg2l_gpt_POEGs, buf);
	if (ret < 0)
		return ret;

	mutex_lock(&pc->mutex);

	pc->poeg = ret;

	if (pc->poeg) {
		/* Set output disable group */
		rzg2l_gpt_write_mask(pc, POEG_mode_set[pc->poeg].poeg,
				GTINTAD_OUTPUT_DISABLE_GRP_MASK, GTINTAD);
		/* Set output disable source */
		if (pc->gpt_operation == DEADTIME_OUTPUT)
			rzg2l_gpt_write_mask(pc,
					OUTPUT_DISABLE_DEADTIME_ERROR,
					GTINTAD_OUTPUT_DISABLE_POEG_MASK, GTINTAD);
		else
			rzg2l_gpt_write_mask(pc,
					OUTPUT_DISABLE_SAME_LEVEL_HIGH|
					OUTPUT_DISABLE_SAME_LEVEL_LOW,
					GTINTAD_OUTPUT_DISABLE_POEG_MASK, GTINTAD);
		/* Enable pin output disable */
		rzg2l_gpt_write_mask(pc,
				channel_set[pc->channel].output_disable.value,
				channel_set[pc->channel].output_disable.mask,
				GTIOR);
		/* Enable overflow interrupt*/
		rzg2l_gpt_write_mask(pc,
				GTINTPROV, GTINTAD_GTINTPR_MASK, GTINTAD);
	} else {
		/* Set output disable group */
		rzg2l_gpt_write_mask(pc, 0, GTINTAD_OUTPUT_DISABLE_GRP_MASK, GTINTAD);
		/* Set output disable source */
		rzg2l_gpt_write_mask(pc, 0, GTINTAD_OUTPUT_DISABLE_POEG_MASK, GTINTAD);
		/* Disable pin output disable */
		rzg2l_gpt_write_mask(pc, 0, channel_set[pc->channel].output_disable.mask, GTIOR);
		/* Disable interrupt if in normal mode */
		if (pc->gpt_operation == NORMAL_OUTPUT)
			rzg2l_gpt_write_mask(pc, 0, GTINTAD_GTINTPR_MASK, GTINTAD);
	}

	mutex_unlock(&pc->mutex);

	return count;
}

static ssize_t POEG_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%s\n", rzg2l_gpt_POEGs[pc->poeg]);
}

static ssize_t pulse_number_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);
	int val, ret;

	if ((pc->gpt_operation != NORMAL_OUTPUT) &&
		(pc->gpt_operation != SINGLE_BUFFER_OUTPUT) &&
		(pc->gpt_operation != DOUBLE_BUFFER_OUTPUT) &&
		(pc->gpt_operation != DEADTIME_OUTPUT)) {
		dev_err(pc->chip.dev, "This operation not use this config\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0) {
		dev_err(pc->chip.dev, "Pulse number must greater or equal 0\n");
		return -EINVAL;
	}

	mutex_lock(&pc->mutex);

	/* Enable interrupt */
	rzg2l_gpt_write_mask(pc, GTINTPROV, GTINTAD_GTINTPR_MASK, GTINTAD);

	pc->pulse_number = val;

	mutex_unlock(&pc->mutex);

	return ret ? : count;
}

static ssize_t pulse_number_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_gpt_chip *pc = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", pc->pulse_number);
}

static DEVICE_ATTR_RW(buffA0);
static DEVICE_ATTR_RW(buffA1);
static DEVICE_ATTR_RW(buffA2);
static DEVICE_ATTR_RW(buffB0);
static DEVICE_ATTR_RW(buffB1);
static DEVICE_ATTR_RW(buffB2);
static DEVICE_ATTR_RW(polarityA);
static DEVICE_ATTR_RW(polarityB);
static DEVICE_ATTR_RW(deadtime_first);
static DEVICE_ATTR_RW(deadtime_second);
static DEVICE_ATTR_RO(gpt_operation_available);
static DEVICE_ATTR_RW(gpt_operation);
static DEVICE_ATTR_RO(POEG_available);
static DEVICE_ATTR_RW(POEG);
static DEVICE_ATTR_RW(pulse_number);

static struct attribute *buffer_attrs[] = {
	&dev_attr_buffA0.attr,
	&dev_attr_buffA1.attr,
	&dev_attr_buffA2.attr,
	&dev_attr_buffB0.attr,
	&dev_attr_buffB1.attr,
	&dev_attr_buffB2.attr,
	&dev_attr_polarityA.attr,
	&dev_attr_polarityB.attr,
	&dev_attr_deadtime_first.attr,
	&dev_attr_deadtime_second.attr,
	&dev_attr_gpt_operation_available.attr,
	&dev_attr_gpt_operation.attr,
	&dev_attr_POEG_available.attr,
	&dev_attr_POEG.attr,
	&dev_attr_pulse_number.attr,
	NULL,
};

static const struct attribute_group buffer_attr_group = {
	.attrs = buffer_attrs,
};

static int rzg2l_gpt_probe(struct platform_device *pdev)
{
	struct rzg2l_gpt_chip *rzg2l_gpt;
	struct resource *res;
	struct device_node *poeg_np;
	struct platform_device *poeg_dev_np;
	struct iio_dev *indio_dev;
	int ret, irq = 0, i, j;
	const char *read_string;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*rzg2l_gpt));
	if (!indio_dev)
		return -ENOMEM;

	rzg2l_gpt = iio_priv(indio_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No memory resource defined\n");
		return -ENODEV;
	}

	rzg2l_gpt->mmio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rzg2l_gpt->mmio_base))
		return PTR_ERR(rzg2l_gpt->mmio_base);

	for (i = 0, j = 0; i < 4; i++) {
		poeg_np = of_parse_phandle(pdev->dev.of_node, "poeg", i);
		if (poeg_np != NULL) {
			poeg_dev_np = of_find_device_by_node(poeg_np);
			if (poeg_dev_np) {
				j++;
				if (strcmp(poeg_np->name, "poega") == 0) {
					rzg2l_gpt_POEGs[j] = "POEGA";
					POEG_mode_set_A.poeg_dev = poeg_dev_np;
					POEG_mode_set[j] = POEG_mode_set_A;
					rzg2l_gpt_reset_counters[j] = "GTETRGA";
					reset_counter_mode_set[j] = reset_counter_mode_set_A;
					dev_info(&pdev->dev, "Can use GTETRGA as POEG, reset_counter\n");
				} else if (strcmp(poeg_np->name, "poegb") == 0) {
					rzg2l_gpt_POEGs[j] = "POEGB";
					POEG_mode_set_B.poeg_dev = poeg_dev_np;
					POEG_mode_set[j] = POEG_mode_set_B;
					rzg2l_gpt_reset_counters[j] = "GTETRGB";
					reset_counter_mode_set[j] = reset_counter_mode_set_B;
					dev_info(&pdev->dev, "Can use GTETRGB as POEG, reset_counter\n");
				} else if (strcmp(poeg_np->name, "poegc") == 0) {
					rzg2l_gpt_POEGs[j] = "POEGC";
					POEG_mode_set_C.poeg_dev = poeg_dev_np;
					POEG_mode_set[j] = POEG_mode_set_C;
					rzg2l_gpt_reset_counters[j] = "GTETRGC";
					reset_counter_mode_set[j] = reset_counter_mode_set_C;
					dev_info(&pdev->dev, "Can use GTETRGC as POEG, reset_counter\n");
				} else if (strcmp(poeg_np->name, "poegd") == 0) {
					rzg2l_gpt_POEGs[j] = "POEGD";
					POEG_mode_set_D.poeg_dev = poeg_dev_np;
					POEG_mode_set[j] = POEG_mode_set_D;
					rzg2l_gpt_reset_counters[j] = "GTETRGD";
					reset_counter_mode_set[j] = reset_counter_mode_set_D;
					dev_info(&pdev->dev, "Can use GTETRGD as POEG, reset_counter\n");
				}
			}
		}
	}

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &rzg2l_gpt_cnt_iio_info;
	indio_dev->channels = &rzg2l_gpt_cnt_channels;
	indio_dev->num_channels = 1;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create counter device: %d\n", ret);
		return ret;
	}

	rzg2l_gpt->rstc = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(rzg2l_gpt->rstc)) {
		dev_err(&pdev->dev, "failed to get cpg reset\n");
		return PTR_ERR(rzg2l_gpt->rstc);
	}

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

	rzg2l_gpt->gpt_operation = NORMAL_OUTPUT;

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
	struct rzg2l_gpt_chip *rzg2l_gpt = dev_get_drvdata(dev);

	if (!test_bit(PWMF_REQUESTED, &pwm->flags))
		return 0;

	rzg2l_gpt->GTIOR_val = rzg2l_gpt_read(rzg2l_gpt, GTIOR);
	rzg2l_gpt->GTINTAD_val = rzg2l_gpt_read(rzg2l_gpt, GTINTAD);

	pm_runtime_put(dev);

	reset_control_assert(rzg2l_gpt->rstc);

	return 0;
}

static int rzg2l_gpt_resume(struct device *dev)
{
	struct pwm_device *pwm = rzg2l_gpt_dev_to_pwm_dev(dev);
	struct rzg2l_gpt_chip *rzg2l_gpt = dev_get_drvdata(dev);

	if (!test_bit(PWMF_REQUESTED, &pwm->flags))
		return 0;

	reset_control_deassert(rzg2l_gpt->rstc);

	pm_runtime_get_sync(dev);

	rzg2l_gpt_config(pwm->chip, pwm, pwm->state.duty_cycle,
			pwm->state.period);
	if (pwm_is_enabled(pwm))
		rzg2l_gpt_enable(pwm->chip, pwm);

	/* Restore buffer value. */
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->bufferA[0], GTCCRA);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->bufferB[0], GTCCRB);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->bufferA[1], GTCCRC);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->bufferB[1], GTCCRE);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->bufferA[2], GTCCRD);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->bufferB[2], GTCCRF);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->deadtime_first, GTDVU);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->deadtime_second, GTDVD);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->GTIOR_val, GTIOR);
	rzg2l_gpt_write(rzg2l_gpt, rzg2l_gpt->GTINTAD_val, GTINTAD);

	/* Restore gpt mode setting */
	gpt_set_operation_mode(rzg2l_gpt);

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
