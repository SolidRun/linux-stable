// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/G2L MTU3a PWM Timer driver
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 *
 * Hardware manual for this IP can be found here
 * https://www.renesas.com/eu/en/document/mah/rzg2l-group-rzg2lc-group-users-manual-hardware-0?language=en
 *
 * Limitations:
 * - When PWM is disabled, the output is driven to Hi-Z.
 * - While the hardware supports both polarities, the driver (for now)
 *   only handles normal polarity.
 * - HW uses one counter and two match components to configure duty_cycle
 *   and period.
 * - Multi-Function Timer Pulse Unit (a.k.a MTU) has 7 HW channels for PWM
 *   operations. (The channels are MTU{0..4, 6, 7}.)
 * - MTU{1, 2} channels have a single IO, whereas all other HW channels have
 *   2 IOs.
 * - Each IO is modelled as an independent PWM channel.
 * - rz_mtu3_channel_io_map table is used to map the PWM channel to the
 *   corresponding HW channel as there are difference in number of IOs
 *   between HW channels.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/limits.h>
#include <linux/mfd/rz-mtu3.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/time.h>

#define RZ_MTU3_MAX_COMP_PWM_CHANNEL	4
#define RZ_MTU3_MAX_PWM_MODE1_CHANNELS	12
#define RZ_MTU3_MAX_PWM_CHANNELS	(RZ_MTU3_MAX_COMP_PWM_CHANNEL + RZ_MTU3_MAX_PWM_MODE1_CHANNELS)
#define RZ_MTU3_MAX_PWM_MODE1_HW_CHANNELS 7
#define RZ_MTU3_MAX_COMP_PWM_HW_CHANNELS  4
#define RZ_MTU3_MAX_HW_CHANNELS		  (RZ_MTU3_MAX_COMP_PWM_HW_CHANNELS + RZ_MTU3_MAX_PWM_MODE1_HW_CHANNELS)

enum mtu3_functions {
	MTU3_PWM_MODE_1,        /* Assign pwm mode 1 function */
	MTU3_PWM_COMPLEMENTARY, /* Assign complementary pwm */
};

enum mtu3_comp_outputs {
	MTU3_PWM_COMP_OUTPUT_BD_AC, /* MTIOCB/D and MTIOCA/C */
	MTU3_PWM_COMP_OUTPUT_BD_BD, /* MTIOCB/D and MTIOCB/D */
};

/**
 * struct rz_mtu3_channel_io_map - MTU3 pwm channel map
 *
 * @base_pwm_number: First PWM of a channel
 * @num_channel_ios: number of IOs on the HW channel.
 */
struct rz_mtu3_channel_io_map {
	u8 base_pwm_number;
	u8 num_channel_ios;
};

/**
 * struct rz_mtu3_pwm_channel - MTU3 pwm channel data
 *
 * @mtu: MTU3 channel data
 * @mtu_comp_pwm: MTU3 channel data of complementary PWM mode
 * @map: MTU3 pwm channel map
 * @function: select between pwm mode 1 and complementary pwm
 * @deadtime: support deadtime for complementary pwm mode
 */
struct rz_mtu3_pwm_channel {
	struct rz_mtu3_channel *mtu;
	struct rz_mtu3_channel *mtu_comp_pwm;
	const struct rz_mtu3_channel_io_map *map;
	enum mtu3_functions function;
	enum mtu3_comp_outputs comp_pwm_outputs;
	u32 deadtime_ns;
};

/**
 * struct rz_mtu3_pwm_chip - MTU3 pwm private data
 *
 * @chip: MTU3 pwm chip data
 * @clk: MTU3 module clock
 * @lock: Lock to prevent concurrent access for usage count
 * @rate: MTU3 clock rate
 * @user_count: MTU3 usage count
 * @enable_count: MTU3 enable count
 * @prescale: MTU3 prescale
 * @channel_data: MTU3 pwm channel data
 */

struct rz_mtu3_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
	struct mutex lock;
	unsigned long rate;
	u32 user_count[RZ_MTU3_MAX_HW_CHANNELS];
	u32 enable_count[RZ_MTU3_MAX_HW_CHANNELS];
	u8 prescale[RZ_MTU3_MAX_HW_CHANNELS];
	struct rz_mtu3_pwm_channel channel_data[RZ_MTU3_MAX_HW_CHANNELS];
};

/*
 * The MTU channels are {0..4, 6, 7} and the number of IO on MTU1
 * and MTU2 channel is 1 compared to 2 on others.
 *
 * Complementay pwm mode mtu {3,6} has 1 channel and mtu{4, 7} have 2 channels IO
 */
static const struct rz_mtu3_channel_io_map channel_map[] = {
	{ 0, 2 }, { 2, 1 }, { 3, 1 }, { 4, 2 }, { 6, 2 }, { 8, 2 }, { 10, 2 },
	{ 12, 1 }, { 13, 1 }, { 14, 1 }, { 15, 1 }
};

static inline struct rz_mtu3_pwm_chip *to_rz_mtu3_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct rz_mtu3_pwm_chip, chip);
}

static void rz_mtu3_pwm_read_tgr_registers(struct rz_mtu3_pwm_channel *priv,
					   u16 reg_pv_offset, u16 *pv_val,
					   u16 reg_dc_offset, u16 *dc_val)
{
	*pv_val = rz_mtu3_16bit_ch_read(priv->mtu, reg_pv_offset);
	*dc_val = rz_mtu3_16bit_ch_read(priv->mtu, reg_dc_offset);
}

static void rz_mtu3_pwm_write_tgr_registers(struct rz_mtu3_pwm_channel *priv,
					    u16 reg_pv_offset, u16 pv_val,
					    u16 reg_dc_offset, u16 dc_val)
{
	rz_mtu3_16bit_ch_write(priv->mtu, reg_pv_offset, pv_val);
	rz_mtu3_16bit_ch_write(priv->mtu, reg_dc_offset, dc_val);
}

static u8 rz_mtu3_pwm_calculate_prescale(struct rz_mtu3_pwm_chip *rz_mtu3,
					 u64 period_cycles)
{
	u32 prescaled_period_cycles;
	u8 prescale;

	/*
	 * Supported prescale values are 1, 4, 16 and 64.
	 * TODO: Support prescale values 2, 8, 32, 256 and 1024.
	 */
	prescaled_period_cycles = period_cycles >> 16;
	if (prescaled_period_cycles >= 16)
		prescale = 3;
	else
		prescale = (fls(prescaled_period_cycles) + 1) / 2;

	return prescale;
}

static struct rz_mtu3_pwm_channel *
rz_mtu3_get_channel(struct rz_mtu3_pwm_chip *rz_mtu3_pwm, u32 hwpwm)
{
	struct rz_mtu3_pwm_channel *priv = rz_mtu3_pwm->channel_data;
	unsigned int ch;

	for (ch = 0; ch < RZ_MTU3_MAX_HW_CHANNELS; ch++, priv++) {
		if (priv->map->base_pwm_number + priv->map->num_channel_ios > hwpwm)
			break;
	}

	return priv;
}

static bool rz_mtu3_pwm_is_ch_enabled(struct rz_mtu3_pwm_chip *rz_mtu3_pwm,
				      u32 hwpwm)
{
	struct rz_mtu3_pwm_channel *priv;
	bool is_channel_en;
	u8 val;

	priv = rz_mtu3_get_channel(rz_mtu3_pwm, hwpwm);
	is_channel_en = rz_mtu3_is_enabled(priv->mtu);
	if (!is_channel_en)
		return false;

	if (priv->map->base_pwm_number == hwpwm)
		val = rz_mtu3_8bit_ch_read(priv->mtu, RZ_MTU3_TIORH);
	else
		val = rz_mtu3_8bit_ch_read(priv->mtu, RZ_MTU3_TIORL);

	return val & RZ_MTU3_TIOR_IOA;
}

static int rz_mtu3_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = to_rz_mtu3_pwm_chip(chip);
	struct rz_mtu3_pwm_channel *priv;
	bool is_mtu3_channel_available;
	u32 ch;

	priv = rz_mtu3_get_channel(rz_mtu3_pwm, pwm->hwpwm);
	ch = priv - rz_mtu3_pwm->channel_data;

	mutex_lock(&rz_mtu3_pwm->lock);
	/*
	 * Each channel must be requested only once, so if the channel
	 * serves two PWMs and the other is already requested, skip over
	 * rz_mtu3_request_channel()
	 */
	if (!rz_mtu3_pwm->user_count[ch]) {
		is_mtu3_channel_available = rz_mtu3_request_channel(priv->mtu);
		if (!is_mtu3_channel_available) {
			mutex_unlock(&rz_mtu3_pwm->lock);
			return -EBUSY;
		}
	}

	rz_mtu3_pwm->user_count[ch]++;
	mutex_unlock(&rz_mtu3_pwm->lock);

	return 0;
}

static void rz_mtu3_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = to_rz_mtu3_pwm_chip(chip);
	struct rz_mtu3_pwm_channel *priv;
	u32 ch;

	priv = rz_mtu3_get_channel(rz_mtu3_pwm, pwm->hwpwm);
	ch = priv - rz_mtu3_pwm->channel_data;

	mutex_lock(&rz_mtu3_pwm->lock);
	rz_mtu3_pwm->user_count[ch]--;
	if (!rz_mtu3_pwm->user_count[ch])
		rz_mtu3_release_channel(priv->mtu);

	mutex_unlock(&rz_mtu3_pwm->lock);
}

static int rz_mtu3_pwm_enable(struct rz_mtu3_pwm_chip *rz_mtu3_pwm,
			      struct pwm_device *pwm)
{
	struct rz_mtu3_pwm_channel *priv;
	u32 ch;
	u8 val, val_comp;
	int rc;

	rc = pm_runtime_resume_and_get(rz_mtu3_pwm->chip.dev);
	if (rc)
		return rc;

	priv = rz_mtu3_get_channel(rz_mtu3_pwm, pwm->hwpwm);
	ch = priv - rz_mtu3_pwm->channel_data;

	if (rz_mtu3_pwm->channel_data[ch].function == MTU3_PWM_MODE_1) {
		val = RZ_MTU3_TIOR_OC_IOB_TOGGLE;

		rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TMDR1,
						RZ_MTU3_TMDR1_MD_PWMMODE1);
		if (priv->map->base_pwm_number == pwm->hwpwm)
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TIORH,
				(rz_mtu3_8bit_ch_read(priv->mtu, RZ_MTU3_TIORH)
							& RZ_MTU3_TIOR_IOA) | val);
		else
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TIORL,
				(rz_mtu3_8bit_ch_read(priv->mtu, RZ_MTU3_TIORL)
							& RZ_MTU3_TIOR_IOA) | val);
	} else if (rz_mtu3_pwm->channel_data[ch].function
						    == MTU3_PWM_COMPLEMENTARY) {
		rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TMDR1,
						RZ_MTU3_TMDR1_MD_COMP_PWMMODE3);
		rz_mtu3_8bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TMDR1,
						RZ_MTU3_TMDR1_MD_COMP_PWMMODE3);
		rz_mtu3_16bit_ch_write(priv->mtu, RZ_MTU3_TCNT, 0);
		rz_mtu3_16bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TCNT, 0);

		/* select enable waveform output */
		if (priv->mtu->channel_number == 3) {
			if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_AC) {
				val_comp = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERA)
						  | (RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47A_EN
						  | RZ_MTU3_TOER_MTIOC47C_EN);
				rz_mtu3_shared_reg_write(priv->mtu,
						      RZ_MTU3_TOERA, val_comp);
			} else if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_BD) {
				val_comp = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERA)
						  | (RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47B_EN
						  | RZ_MTU3_TOER_MTIOC47D_EN);
				rz_mtu3_shared_reg_write(priv->mtu,
						      RZ_MTU3_TOERA, val_comp);
			}
		} else if (priv->mtu->channel_number == 6) {
			if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_AC) {
				val_comp = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERB)
						  | (RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47A_EN
						  | RZ_MTU3_TOER_MTIOC47C_EN);
				rz_mtu3_shared_reg_write(priv->mtu,
						      RZ_MTU3_TOERB, val_comp);
			} else if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_BD) {
				val_comp = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERB)
						  | (RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47B_EN
						  | RZ_MTU3_TOER_MTIOC47D_EN);

				rz_mtu3_shared_reg_write(priv->mtu,
						      RZ_MTU3_TOERB, val_comp);
			}
		}

		/* It is important to start 2 channels of complementary pwm simultaneously */
		if (priv->mtu->channel_number == 3) {
			val = rz_mtu3_shared_reg_read(priv->mtu,
				      RZ_MTU3_TSTRA) | RZ_MTU3_TSTR_PWM_COMP_EN;
			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TSTRA, val);
		} else {
			val = rz_mtu3_shared_reg_read(priv->mtu,
				      RZ_MTU3_TSTRB) | RZ_MTU3_TSTR_PWM_COMP_EN;
			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TSTRB, val);
		}
	}

	mutex_lock(&rz_mtu3_pwm->lock);
	if (!rz_mtu3_pwm->enable_count[ch])
		rz_mtu3_enable(priv->mtu);

	rz_mtu3_pwm->enable_count[ch]++;
	mutex_unlock(&rz_mtu3_pwm->lock);

	return 0;
}

static void rz_mtu3_pwm_disable(struct rz_mtu3_pwm_chip *rz_mtu3_pwm,
				struct pwm_device *pwm)
{
	struct rz_mtu3_pwm_channel *priv;
	u32 ch;
	u8 val, val_tstr;

	priv = rz_mtu3_get_channel(rz_mtu3_pwm, pwm->hwpwm);
	ch = priv - rz_mtu3_pwm->channel_data;

	if (rz_mtu3_pwm->channel_data[ch].function == MTU3_PWM_MODE_1) {
		/* Disable output pins of MTU3 channel */
		if (priv->map->base_pwm_number == pwm->hwpwm)
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TIORH,
					       RZ_MTU3_TIOR_OC_RETAIN);
		else
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TIORL,
					       RZ_MTU3_TIOR_OC_RETAIN);
	} else if (rz_mtu3_pwm->channel_data[ch].function ==
						      MTU3_PWM_COMPLEMENTARY) {
		/* select disable waveform output */
		if (priv->mtu->channel_number == 3) {
			if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_AC) {
				val = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERA)
						& ~(RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47A_EN
						  | RZ_MTU3_TOER_MTIOC47C_EN);
				rz_mtu3_shared_reg_write(priv->mtu,
							  RZ_MTU3_TOERA, val);
			} else if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_BD) {
				val = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERA)
						& ~(RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47B_EN
						  | RZ_MTU3_TOER_MTIOC47D_EN);
				rz_mtu3_shared_reg_write(priv->mtu,
							  RZ_MTU3_TOERA, val);
			}
		} else if (priv->mtu->channel_number == 6) {
			if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_AC) {
				val = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERB)
						& ~(RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47A_EN
						  | RZ_MTU3_TOER_MTIOC47C_EN);
				rz_mtu3_shared_reg_write(priv->mtu,
							  RZ_MTU3_TOERB, val);
			} else if (rz_mtu3_pwm->channel_data[ch].comp_pwm_outputs
						== MTU3_PWM_COMP_OUTPUT_BD_BD) {
				val = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TOERB)
						& ~(RZ_MTU3_TOER_MTIOC36B_EN
						  | RZ_MTU3_TOER_MTIOC36D_EN
						  | RZ_MTU3_TOER_MTIOC47B_EN
						  | RZ_MTU3_TOER_MTIOC47D_EN);

				rz_mtu3_shared_reg_write(priv->mtu,
							  RZ_MTU3_TOERB, val);
			}
		}

		rz_mtu3_8bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TMDR1,
						      RZ_MTU3_TMDR1_MD_NORMAL);
		/* It is important to stop 2 channels of complementary pwm simultaneously */
		if (priv->mtu->channel_number == 3) {
			 val_tstr = rz_mtu3_shared_reg_read(priv->mtu,
				    RZ_MTU3_TSTRA) & ~RZ_MTU3_TSTR_PWM_COMP_EN;
			 rz_mtu3_shared_reg_write(priv->mtu,
						      RZ_MTU3_TSTRA, val_tstr);
		} else {
			val_tstr = rz_mtu3_shared_reg_read(priv->mtu,
				    RZ_MTU3_TSTRB) & ~RZ_MTU3_TSTR_PWM_COMP_EN;
			rz_mtu3_shared_reg_write(priv->mtu,
						      RZ_MTU3_TSTRB, val_tstr);
		}
	}

	mutex_lock(&rz_mtu3_pwm->lock);
	rz_mtu3_pwm->enable_count[ch]--;
	if (!rz_mtu3_pwm->enable_count[ch])
		rz_mtu3_disable(priv->mtu);

	mutex_unlock(&rz_mtu3_pwm->lock);

	pm_runtime_put_sync(rz_mtu3_pwm->chip.dev);
}

static void rz_mtu3_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = to_rz_mtu3_pwm_chip(chip);
	int rc;

	rc = pm_runtime_resume_and_get(chip->dev);
	if (rc) {
		WARN_ON(1);
		return;
	}

	state->enabled = rz_mtu3_pwm_is_ch_enabled(rz_mtu3_pwm, pwm->hwpwm);
	if (state->enabled) {
		struct rz_mtu3_pwm_channel *priv;
		u8 prescale, val;
		u16 dc, pv;
		u64 tmp;

		priv = rz_mtu3_get_channel(rz_mtu3_pwm, pwm->hwpwm);
		if (priv->map->base_pwm_number == pwm->hwpwm)
			rz_mtu3_pwm_read_tgr_registers(priv, RZ_MTU3_TGRA, &pv,
						       RZ_MTU3_TGRB, &dc);
		else
			rz_mtu3_pwm_read_tgr_registers(priv, RZ_MTU3_TGRC, &pv,
						       RZ_MTU3_TGRD, &dc);

		val = rz_mtu3_8bit_ch_read(priv->mtu, RZ_MTU3_TCR);
		prescale = FIELD_GET(RZ_MTU3_TCR_TPCS, val);

		/* With prescale <= 7 and pv <= 0xffff this doesn't overflow. */
		tmp = NSEC_PER_SEC * (u64)pv << (2 * prescale);
		state->period = DIV_ROUND_UP_ULL(tmp, rz_mtu3_pwm->rate);
		tmp = NSEC_PER_SEC * (u64)dc << (2 * prescale);
		state->duty_cycle = DIV_ROUND_UP_ULL(tmp, rz_mtu3_pwm->rate);

		if (state->duty_cycle > state->period)
			state->duty_cycle = state->period;
	}

	state->polarity = PWM_POLARITY_NORMAL;
	pm_runtime_put(chip->dev);
}

static u16 rz_mtu3_pwm_calculate_pv_or_dc(u64 period_or_duty_cycle, u8 prescale)
{
	return min(period_or_duty_cycle >> (2 * prescale), (u64)U16_MAX);
}

static int rz_mtu3_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      const struct pwm_state *state)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = to_rz_mtu3_pwm_chip(chip);
	struct rz_mtu3_pwm_channel *priv;
	u64 period_cycles;
	u64 duty_cycles;
	u8 prescale;
	u16 pv, dc, deadtime;
	u8 val, val_polarity, val_tder;
	u32 ch;

	priv = rz_mtu3_get_channel(rz_mtu3_pwm, pwm->hwpwm);
	ch = priv - rz_mtu3_pwm->channel_data;

	if (rz_mtu3_pwm->channel_data[ch].function == MTU3_PWM_MODE_1)
		period_cycles = mul_u64_u32_div(state->period, rz_mtu3_pwm->rate,
					NSEC_PER_SEC);
	else if (rz_mtu3_pwm->channel_data[ch].function ==
					       MTU3_PWM_COMPLEMENTARY)
		period_cycles = mul_u64_u32_div(state->period / 2,
					        rz_mtu3_pwm->rate, NSEC_PER_SEC);

	prescale = rz_mtu3_pwm_calculate_prescale(rz_mtu3_pwm, period_cycles);

	/*
	 * Prescalar is shared by multiple channels, so prescale can
	 * NOT be modified when there are multiple channels in use with
	 * different settings. Modify prescalar if other PWM is off or handle
	 * it, if current prescale value is less than the one we want to set.
	 */
	if (rz_mtu3_pwm->enable_count[ch] > 1) {
		if (rz_mtu3_pwm->prescale[ch] > prescale)
			return -EBUSY;

		prescale = rz_mtu3_pwm->prescale[ch];
	}

	pv = rz_mtu3_pwm_calculate_pv_or_dc(period_cycles, prescale);

	if (rz_mtu3_pwm->channel_data[ch].function == MTU3_PWM_MODE_1)
		duty_cycles = mul_u64_u32_div(state->duty_cycle,
					rz_mtu3_pwm->rate, NSEC_PER_SEC);
	else if (rz_mtu3_pwm->channel_data[ch].function ==
						MTU3_PWM_COMPLEMENTARY) {
		duty_cycles = mul_u64_u32_div(state->duty_cycle / 2,
					rz_mtu3_pwm->rate, NSEC_PER_SEC);

		if ((!rz_mtu3_pwm->channel_data[ch].deadtime_ns))
			deadtime = 1;
		else
			deadtime = mul_u64_u32_div(rz_mtu3_pwm->
					channel_data[ch].deadtime_ns,
					      rz_mtu3_pwm->rate, NSEC_PER_SEC);
		deadtime = rz_mtu3_pwm_calculate_pv_or_dc(deadtime, prescale);

	}
	dc = rz_mtu3_pwm_calculate_pv_or_dc(duty_cycles, prescale);

	/*
	 * If the PWM channel is disabled, make sure to turn on the clock
	 * before writing the register.
	 */
	if (!pwm->state.enabled) {
		int rc;

		rc = pm_runtime_resume_and_get(chip->dev);
		if (rc)
			return rc;
	}

	val = RZ_MTU3_TCR_CKEG_RISING | prescale;

	/* Counter must be stopped while updating TCR register */
	if (rz_mtu3_pwm->prescale[ch] != prescale && rz_mtu3_pwm->enable_count[ch])
		rz_mtu3_disable(priv->mtu);

	if (rz_mtu3_pwm->channel_data[ch].function == MTU3_PWM_MODE_1) {
		if (priv->map->base_pwm_number == pwm->hwpwm) {
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TCR,
					      RZ_MTU3_TCR_CCLR_TGRA | val);
			rz_mtu3_pwm_write_tgr_registers(priv, RZ_MTU3_TGRA, pv,
						RZ_MTU3_TGRB, dc);
		} else {
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TCR,
						RZ_MTU3_TCR_CCLR_TGRC | val);
			rz_mtu3_pwm_write_tgr_registers(priv, RZ_MTU3_TGRC, pv,
						RZ_MTU3_TGRD, dc);
		}
	} else if (rz_mtu3_pwm->channel_data[ch].function ==
						      MTU3_PWM_COMPLEMENTARY) {
		rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TCR, val);
		rz_mtu3_8bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TCR, val);

		rz_mtu3_16bit_ch_write(priv->mtu, RZ_MTU3_TGRC, pv + deadtime);
		rz_mtu3_16bit_ch_write(priv->mtu, RZ_MTU3_TGRA, pv + deadtime);
		rz_mtu3_16bit_ch_write(priv->mtu, RZ_MTU3_TGRB, dc);
		rz_mtu3_16bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TGRA, dc);
		rz_mtu3_16bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TGRB, dc);
		rz_mtu3_16bit_ch_write(priv->mtu, RZ_MTU3_TGRD, dc);
		rz_mtu3_16bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TGRC, dc);
		rz_mtu3_16bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TGRD, dc);
		rz_mtu3_16bit_ch_write(priv->mtu, RZ_MTU3_TCNT, deadtime);
		rz_mtu3_16bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TCNT, 0);
	}

	if (rz_mtu3_pwm->prescale[ch] != prescale) {
		/*
		 * Prescalar is shared by multiple channels, we cache the
		 * prescalar value from first enabled channel and use the same
		 * value for both channels.
		 */
		rz_mtu3_pwm->prescale[ch] = prescale;

		if (rz_mtu3_pwm->enable_count[ch])
			rz_mtu3_enable(priv->mtu);
	}

	if (rz_mtu3_pwm->channel_data[ch].function == MTU3_PWM_MODE_1) {
		if (state->polarity == PWM_POLARITY_NORMAL)
			val_polarity = RZ_MTU3_TIOR_OC_IOA_H_COMP_MATCH;
		else
			val_polarity = RZ_MTU3_TIOR_OC_IOA_L_COMP_MATCH;

		if (priv->map->base_pwm_number == pwm->hwpwm)
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TIORH,
				(rz_mtu3_8bit_ch_read(priv->mtu, RZ_MTU3_TIORH)
						& RZ_MTU3_TIOR_IOB) | val_polarity);
		else
			rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TIORL,
				(rz_mtu3_8bit_ch_read(priv->mtu, RZ_MTU3_TIORL)
						& RZ_MTU3_TIOR_IOB) | val_polarity);
	} else if (rz_mtu3_pwm->channel_data[ch].function ==
						      MTU3_PWM_COMPLEMENTARY) {
		rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TMDR1,
						      RZ_MTU3_TMDR1_MD_NORMAL);
		rz_mtu3_8bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TMDR1,
						      RZ_MTU3_TMDR1_MD_NORMAL);

		if (priv->mtu->channel_number == 3) {
			val_tder = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TDERA);
			if (!(rz_mtu3_pwm->channel_data[ch].deadtime_ns)
								   && val_tder)
				rz_mtu3_shared_reg_write(priv->mtu,
				   RZ_MTU3_TDERA, val_tder & !RZ_MTU3_TDER_EN);
			else if (!!(rz_mtu3_pwm->channel_data[ch].deadtime_ns)
								   && !val_tder)
				rz_mtu3_shared_reg_write(priv->mtu,
				    RZ_MTU3_TDERA, val_tder | RZ_MTU3_TDER_EN);

			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TCDRA, pv);
			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TCBRA, pv);
			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TDDRA,
								     deadtime);
		} else if (priv->mtu->channel_number == 6) {
			val_tder = rz_mtu3_shared_reg_read(priv->mtu,
								RZ_MTU3_TDERB);
			if (!(rz_mtu3_pwm->channel_data[ch].deadtime_ns)
								   && val_tder)
				rz_mtu3_shared_reg_write(priv->mtu,
				   RZ_MTU3_TDERB, val_tder & !RZ_MTU3_TDER_EN);
			else if (!!(rz_mtu3_pwm->channel_data[ch].deadtime_ns)
								  && !val_tder)
				rz_mtu3_shared_reg_write(priv->mtu,
				    RZ_MTU3_TDERB, val_tder | RZ_MTU3_TDER_EN);

			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TCDRB, pv);
			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TCBRB, pv);
			rz_mtu3_shared_reg_write(priv->mtu, RZ_MTU3_TDDRB,
								     deadtime);
		}

		rz_mtu3_8bit_ch_write(priv->mtu, RZ_MTU3_TMDR1,
					       RZ_MTU3_TMDR1_MD_COMP_PWMMODE3);
		rz_mtu3_8bit_ch_write(priv->mtu_comp_pwm, RZ_MTU3_TMDR1,
					       RZ_MTU3_TMDR1_MD_COMP_PWMMODE3);
	}

	/* If the PWM is not enabled, turn the clock off again to save power. */
	if (!pwm->state.enabled)
		pm_runtime_put(chip->dev);

	return 0;
}

static int rz_mtu3_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = to_rz_mtu3_pwm_chip(chip);
	bool enabled = pwm->state.enabled;
	int ret;

	if (!state->enabled) {
		if (enabled)
			rz_mtu3_pwm_disable(rz_mtu3_pwm, pwm);

		return 0;
	}

	mutex_lock(&rz_mtu3_pwm->lock);
	ret = rz_mtu3_pwm_config(chip, pwm, state);
	mutex_unlock(&rz_mtu3_pwm->lock);
	if (ret)
		return ret;

	if (!enabled)
		ret = rz_mtu3_pwm_enable(rz_mtu3_pwm, pwm);

	return ret;
}

static const struct pwm_ops rz_mtu3_pwm_ops = {
	.request = rz_mtu3_pwm_request,
	.free = rz_mtu3_pwm_free,
	.get_state = rz_mtu3_pwm_get_state,
	.apply = rz_mtu3_pwm_apply,
	.owner = THIS_MODULE,
};

static ssize_t mtu34_pwm_deadtime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rz_mtu3_pwm_chip *mtu3 = platform_get_drvdata(pdev);
	unsigned int deadtime_ns;
	int ret;

	ret = kstrtouint(buf, 0, &deadtime_ns);
	if (ret)
		return ret;

	mtu3->channel_data[7].deadtime_ns = deadtime_ns;
	mtu3->channel_data[8].deadtime_ns = deadtime_ns;

	return ret ? : size;
}

static ssize_t mtu34_pwm_deadtime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rz_mtu3_pwm_chip *mtu3 = platform_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%u ns\n",
				mtu3->channel_data[7].deadtime_ns);
}

static DEVICE_ATTR_RW(mtu34_pwm_deadtime);

static ssize_t mtu67_pwm_deadtime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev =  to_platform_device(dev);
	struct rz_mtu3_pwm_chip *mtu3 = platform_get_drvdata(pdev);
	unsigned int deadtime_ns;
	int ret;

	ret = kstrtouint(buf, 0, &deadtime_ns);
	if (ret)
		return ret;

	mtu3->channel_data[9].deadtime_ns = deadtime_ns;
	mtu3->channel_data[10].deadtime_ns = deadtime_ns;

	return ret ? : size;
}

static ssize_t mtu67_pwm_deadtime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =  to_platform_device(dev);
	struct rz_mtu3_pwm_chip *mtu3 = platform_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%u ns\n",
				mtu3->channel_data[9].deadtime_ns);
}

static DEVICE_ATTR_RW(mtu67_pwm_deadtime);


#ifdef CONFIG_PM_SLEEP
static int rz_mtu3_pwm_pm_runtime_suspend(struct device *dev)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = dev_get_drvdata(dev);

	clk_disable_unprepare(rz_mtu3_pwm->clk);

	return 0;
}

static int rz_mtu3_pwm_pm_runtime_resume(struct device *dev)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = dev_get_drvdata(dev);

	return clk_prepare_enable(rz_mtu3_pwm->clk);
}
#endif

static const struct dev_pm_ops rz_mtu3_pwm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rz_mtu3_pwm_pm_runtime_suspend,
				rz_mtu3_pwm_pm_runtime_resume)
};

static void rz_mtu3_pwm_pm_disable(void *data)
{
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm = data;

	clk_rate_exclusive_put(rz_mtu3_pwm->clk);
	pm_runtime_disable(rz_mtu3_pwm->chip.dev);
	pm_runtime_set_suspended(rz_mtu3_pwm->chip.dev);
}

static int rz_mtu3_pwm_probe(struct platform_device *pdev)
{
	struct rz_mtu3 *parent_ddata = dev_get_drvdata(pdev->dev.parent);
	struct rz_mtu3_pwm_chip *rz_mtu3_pwm;
	struct device *dev = &pdev->dev;
	unsigned int i, j = 0;
	int ret;

	rz_mtu3_pwm = devm_kzalloc(&pdev->dev, sizeof(*rz_mtu3_pwm), GFP_KERNEL);
	if (!rz_mtu3_pwm)
		return -ENOMEM;

	rz_mtu3_pwm->clk = parent_ddata->clk;

	for (i = 0; i < RZ_MTU_NUM_CHANNELS; i++) {
		if (i == RZ_MTU3_CHAN_5 || i == RZ_MTU3_CHAN_8)
			continue;

		rz_mtu3_pwm->channel_data[j].mtu = &parent_ddata->channels[i];
		rz_mtu3_pwm->channel_data[j].mtu->dev = dev;
		rz_mtu3_pwm->channel_data[j].map = &channel_map[j];
		rz_mtu3_pwm->channel_data[j].function = MTU3_PWM_MODE_1;
		j++;
	}

	for (i = 0; i < 2; i++) {
		rz_mtu3_pwm->channel_data[j].mtu = &parent_ddata->channels[3];
		rz_mtu3_pwm->channel_data[j].mtu_comp_pwm = &parent_ddata->channels[4];
		rz_mtu3_pwm->channel_data[j].mtu->dev = dev;
		rz_mtu3_pwm->channel_data[j].mtu_comp_pwm->dev = dev;
		rz_mtu3_pwm->channel_data[j].map = &channel_map[j];
		rz_mtu3_pwm->channel_data[j].function = MTU3_PWM_COMPLEMENTARY;
		rz_mtu3_pwm->channel_data[j].comp_pwm_outputs = (j % 2 == 0)
						? MTU3_PWM_COMP_OUTPUT_BD_BD
						: MTU3_PWM_COMP_OUTPUT_BD_AC;
		j++;
	}

	for (i = 0; i < 2; i++) {
		rz_mtu3_pwm->channel_data[j].mtu = &parent_ddata->channels[6];
		rz_mtu3_pwm->channel_data[j].mtu_comp_pwm = &parent_ddata->channels[7];
		rz_mtu3_pwm->channel_data[j].mtu->dev = dev;
		rz_mtu3_pwm->channel_data[j].mtu_comp_pwm->dev = dev;
		rz_mtu3_pwm->channel_data[j].map = &channel_map[j];
		rz_mtu3_pwm->channel_data[j].function = MTU3_PWM_COMPLEMENTARY;
		rz_mtu3_pwm->channel_data[j].comp_pwm_outputs = (j % 2 == 0)
						? MTU3_PWM_COMP_OUTPUT_BD_BD
						: MTU3_PWM_COMP_OUTPUT_BD_AC;
		j++;
	}

	ret = device_create_file(dev, &dev_attr_mtu34_pwm_deadtime);
	if (ret)
		dev_err(dev, "failed to create api deadtime34 on sysfs\n");

	ret = device_create_file(dev, &dev_attr_mtu67_pwm_deadtime);
	if (ret)
		dev_err(dev, "failed to create api deadtime67 on sysfs\n");

	mutex_init(&rz_mtu3_pwm->lock);
	platform_set_drvdata(pdev, rz_mtu3_pwm);
	ret = clk_prepare_enable(rz_mtu3_pwm->clk);
	if (ret)
		return dev_err_probe(dev, ret, "Clock enable failed\n");

	clk_rate_exclusive_get(rz_mtu3_pwm->clk);

	rz_mtu3_pwm->rate = clk_get_rate(rz_mtu3_pwm->clk);
	/*
	 * Refuse clk rates > 1 GHz to prevent overflow later for computing
	 * period and duty cycle.
	 */
	if (rz_mtu3_pwm->rate > NSEC_PER_SEC) {
		ret = -EINVAL;
		clk_rate_exclusive_put(rz_mtu3_pwm->clk);
		goto disable_clock;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	rz_mtu3_pwm->chip.dev = &pdev->dev;
	ret = devm_add_action_or_reset(&pdev->dev, rz_mtu3_pwm_pm_disable,
				       rz_mtu3_pwm);
	if (ret < 0)
		return ret;

	rz_mtu3_pwm->chip.ops = &rz_mtu3_pwm_ops;
	rz_mtu3_pwm->chip.npwm = RZ_MTU3_MAX_PWM_CHANNELS;
	rz_mtu3_pwm->chip.base = -1;
	ret = devm_pwmchip_add(&pdev->dev, &rz_mtu3_pwm->chip);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to add PWM chip\n");

	pm_runtime_idle(&pdev->dev);

	return 0;

disable_clock:
	clk_disable_unprepare(rz_mtu3_pwm->clk);
	return ret;
}

static struct platform_driver rz_mtu3_pwm_driver = {
	.driver = {
		.name = "pwm-rz-mtu3",
		.pm = &rz_mtu3_pwm_pm_ops,
	},
	.probe = rz_mtu3_pwm_probe,
};
module_platform_driver(rz_mtu3_pwm_driver);

MODULE_AUTHOR("Biju Das <biju.das.jz@bp.renesas.com>");
MODULE_ALIAS("platform:pwm-rz-mtu3");
MODULE_DESCRIPTION("Renesas RZ/G2L MTU3a PWM Timer Driver");
MODULE_LICENSE("GPL");
