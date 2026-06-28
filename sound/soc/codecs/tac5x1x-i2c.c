// SPDX-License-Identifier: GPL-2.0
//
// tac5x1x.c
//
// Copyright (C) 2022 - 2025 Texas Instruments Incorporated
//
// Author: Kevin Lu <kevin-lu@ti.com>
// Author: Kokila Karuppusamy <kokila.karuppusamy@ti.com>
// Author: Niranjan H Y <niranjan.hy@ti.com>
//

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <sound/tlv.h>
#include <linux/i2c.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/version.h>

#include "tac5x1x.h"

struct tac5x1x_setup_gpio {
	s32 gpio_func[3];
	s32 gpio_drive[3];
	s32 pdm_input_pins[2];
	s32 gpi1_func;
	s32 gpa_gpio;
};

struct tac5x1x_input_diag_config {
	s32 in_ch_en;
	s32 out_ch_en;
	s32 incl_se_inm;
	s32 incl_ac_coup;
};

struct tac5x1x_irqinfo {
	s32 irq_gpio;
	s32 irq;
	bool irq_enable;
	u32 *latch_regs;
	u32 *latch_data;
};

struct tac5x1x_priv {
	struct snd_soc_component *component;
	struct regmap *regmap;
	struct device *dev;
	enum tac5x1x_type codec_type;
	s32 vref_vg;
	s32 micbias_en;
	s32 micbias_vg;
	s32 uad_en;
	s32 vad_en;
	s32 uag_en;
	s32 micbias_threshold[2];
	s32 gpa_threshold[2];
	s32 adc_impedance[2];
	s32 out2x_vcom_cfg;
	u32 ch_enabled;
	struct mutex ch_lock;
	struct tac5x1x_setup_gpio *gpio_setup;
	struct tac5x1x_irqinfo irqinfo;
	struct tac5x1x_input_diag_config input_diag_config;
	struct delayed_work powerup_work;
};

static int tac5x1x_num_regulators = 2;
static struct regulator_bulk_data tac5x1x_regulators[] = {
	{.supply = "iovdd"},
	{.supply = "avdd"},
};

struct mask_to_txt {
	u8 mask;
	const char *const name;
};

struct interrupt_info {
	u32 reg;
	u32 count;
	const struct mask_to_txt *mask_str_map;
};

#define TAC5X1X_EVENT(bit, evt_txt) (\
	(struct mask_to_txt) { \
		.mask = BIT((bit)), \
		.name = evt_txt \
	})

static const struct mask_to_txt int_chx_latch[] = {
	TAC5X1X_EVENT(7, "Input Channel1 fault"),
	TAC5X1X_EVENT(6, "Input Channel2 fault"),
	TAC5X1X_EVENT(5, "Output Channel1 fault"),
	TAC5X1X_EVENT(4, "Output Channel2 fault"),
	TAC5X1X_EVENT(3, "Short to VBAT_IN"),
};

static const struct mask_to_txt in_ch1_latch[] = {
	TAC5X1X_EVENT(7, "IN_CH1 open Input"),
	TAC5X1X_EVENT(6, "IN_CH1 Input shorted"),
	TAC5X1X_EVENT(5, "IN_CH1 INP shorted to GND"),
	TAC5X1X_EVENT(4, "IN_CH1 INM shorted to GND"),
	TAC5X1X_EVENT(3, "IN_CH1 INP shorted to MICBIAS"),
	TAC5X1X_EVENT(2, "IN_CH1 INM shorted to MICBIAS"),
	TAC5X1X_EVENT(1, "IN_CH1 INP shorted to VBAT_IN"),
	TAC5X1X_EVENT(0, "IN_CH1 INM shorted to VBAT_IN"),
};

static const struct mask_to_txt in_ch2_latch[] = {
	TAC5X1X_EVENT(7, "IN_CH2 open Input"),
	TAC5X1X_EVENT(6, "IN_CH2 Input shorted"),
	TAC5X1X_EVENT(5, "IN_CH2 INP shorted to GND"),
	TAC5X1X_EVENT(4, "IN_CH2 INM shorted to GND"),
	TAC5X1X_EVENT(3, "IN_CH2 INP shorted to MICBIAS"),
	TAC5X1X_EVENT(2, "IN_CH2 INM shorted to MICBIAS"),
	TAC5X1X_EVENT(1, "IN_CH2 INP shorted to VBAT_IN"),
	TAC5X1X_EVENT(0, "IN_CH2 INM shorted to VBAT_IN"),
};

static const struct mask_to_txt out_ch1_latch[] = {
	TAC5X1X_EVENT(7, "OUT_CH1 OUT1P Short circuit Fault"),
	TAC5X1X_EVENT(6, "OUT_CH1 OUT1M Short circuit Fault"),
	TAC5X1X_EVENT(5, "OUT_CH1 DRVRP Virtual Ground Fault"),
	TAC5X1X_EVENT(4, "OUT_CH1 DRVRM Virtual ground Fault"),
	/* masks */
	TAC5X1X_EVENT(3, "OUT_CH1 ADC CH1 Mask"),
	TAC5X1X_EVENT(2, "OUT_CH1 ADC CH2 MASK"),
};

static const struct mask_to_txt out_ch2_latch[] = {
	TAC5X1X_EVENT(7, "OUT_CH2 OUT2P Short circuit Fault"),
	TAC5X1X_EVENT(6, "OUT_CH2 OUT2M Short circuit Fault"),
	TAC5X1X_EVENT(5, "OUT_CH2 DRVRP Virtual Ground Fault"),
	TAC5X1X_EVENT(4, "OUT_CH2 DRVRM Virtual ground Fault"),
	/* mask */
	TAC5X1X_EVENT(1, "AREG SC Fault Mask"),
	TAC5X1X_EVENT(0, "AREG SC Fault"),
};

static const struct mask_to_txt int_latch1[] = {
	TAC5X1X_EVENT(7, "CH1 INP Over Voltage"),
	TAC5X1X_EVENT(6, "CH1 INM Over Voltage"),
	TAC5X1X_EVENT(5, "CH2 INP over Voltage"),
	TAC5X1X_EVENT(4, "CH2 INM Over Voltage"),
	TAC5X1X_EVENT(3, "Headset Insert Detection"),
	TAC5X1X_EVENT(2, "Headset Remove Detection"),
	TAC5X1X_EVENT(1, "Headset Hook"),
	TAC5X1X_EVENT(0, "MIPS Overload"),
};

static const struct mask_to_txt int_latch2[] = {
	TAC5X1X_EVENT(7, "GPA Up threashold Fault"),
	TAC5X1X_EVENT(6, "GPA low threashold Fault"),
	TAC5X1X_EVENT(5, "VAD Power up detect"),
	TAC5X1X_EVENT(4, "VAD power down detect"),
	TAC5X1X_EVENT(3, "Micbias short circuit"),
	TAC5X1X_EVENT(2, "Micbias high current fault"),
	TAC5X1X_EVENT(1, "Micbias low current fault"),
	TAC5X1X_EVENT(0, "Micbias Over voltage fault"),
};

static const struct mask_to_txt int_latch_0[] = {
	TAC5X1X_EVENT(7, "Clock Error"),
	TAC5X1X_EVENT(6, "PLL Lock"),
	TAC5X1X_EVENT(5, "Boost Over Temperature"),
	TAC5X1X_EVENT(4, "Boost Over Current"),
	TAC5X1X_EVENT(3, "Boost MO"),
};

#define LTCH_TO_MASK_STR_MAP(latch_reg, str_map, map_size) (\
	(struct interrupt_info){ \
		.reg = (latch_reg), \
		.count = (map_size), \
		.mask_str_map = (str_map), \
	})

static const struct interrupt_info intr_info_list[] = {
	LTCH_TO_MASK_STR_MAP(TAC5X1X_CHX_LTCH, int_chx_latch,
			     ARRAY_SIZE(int_chx_latch)),
	LTCH_TO_MASK_STR_MAP(TAC5X1X_IN_CH1_LTCH, in_ch1_latch,
			     ARRAY_SIZE(in_ch1_latch)),
	LTCH_TO_MASK_STR_MAP(TAC5X1X_IN_CH2_LTCH, in_ch2_latch,
			     ARRAY_SIZE(in_ch2_latch)),
	LTCH_TO_MASK_STR_MAP(TAC5X1X_OUT_CH1_LTCH, out_ch1_latch,
			     ARRAY_SIZE(out_ch1_latch)),
	LTCH_TO_MASK_STR_MAP(TAC5X1X_OUT_CH2_LTCH, out_ch2_latch,
			     ARRAY_SIZE(out_ch2_latch)),
	LTCH_TO_MASK_STR_MAP(TAC5X1X_INT_LTCH1, int_latch1,
			     ARRAY_SIZE(int_latch1)),
	LTCH_TO_MASK_STR_MAP(TAC5X1X_INT_LTCH2, int_latch2,
			     ARRAY_SIZE(int_latch2)),
	/* This should be the last entry */
	LTCH_TO_MASK_STR_MAP(TAC5X1X_INT_LTCH0, int_latch_0,
			     ARRAY_SIZE(int_latch_0)),
};

static const struct regmap_range_cfg tac5x1x_ranges[] = {
	{
		.range_min = 0,
		.range_max = 0xfe * 128,
		.selector_reg = TAC_PAGE_SELECT,
		.selector_mask = GENMASK(7, 0),
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static bool tac5x1x_volatile_regs(struct device *dev, unsigned int reg)
{
	bool is_volatile;

	switch (reg) {
	case TAC5X1X_RESET:
	case TAC5X1X_INT_LTCH0 ... TAC5X1X_INT_LTCH2:
	case TAC5X1X_TP_START ... TAC5X1X_TP_END:
		is_volatile = true;
		break;

	default:
		is_volatile = false;
		break;
	}

	return is_volatile;
}

static bool tac5x1x_precious_regs(struct device *dev, unsigned int reg)
{
	bool is_precious;

	switch (reg) {
	/* self clearing register latches */
	case TAC5X1X_INT_LTCH0 ... TAC5X1X_INT_LTCH2:
		is_precious = true;
		break;
	default:
		is_precious = false;
		break;
	}

	return is_precious;
}

static const struct regmap_config tac5x1x_regmap = {
	.max_register = 0xfe * 128,
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.ranges = tac5x1x_ranges,
	.volatile_reg = tac5x1x_volatile_regs,
	.precious_reg = tac5x1x_precious_regs,
	.num_ranges = ARRAY_SIZE(tac5x1x_ranges),
};


#define IS_DAC_CH(ch)	((ch) & 0x0f)
#define IS_ADC_CH(ch)	((ch) & 0xf0)

#define print_regs() \
	do { \
		int val1, ret1, val2, ret2; \
		ret1 = regmap_read(tac5x1x->regmap, TAC5X1X_PWR_CFG, &val1); \
		ret2 = regmap_read(tac5x1x->regmap, TAC5X1X_CH_EN, &val2); \
		dev_dbg(tac5x1x->dev, "%s v1=%x, v2=%x, errs=%d, %d", \
			 __func__, val1, val2, ret1, ret2); \
	} while (0);

static void post_powerup_work(struct work_struct *work)
{
	u8 mask;
	u8 pwr_cfg;
	struct tac5x1x_priv *tac5x1x =
		container_of(work, struct tac5x1x_priv, powerup_work.work);
	struct snd_soc_component *component = tac5x1x->component;

	msleep(100);

	mutex_lock(&tac5x1x->ch_lock);
	mask = pwr_cfg = TAC5X1X_PWR_CFG_ADC_PDZ |
		TAC5X1X_PWR_CFG_MICBIAS | TAC5X1X_PWR_CFG_DAC_PDZ;
	if (IS_DAC_CH(tac5x1x->ch_enabled) == 0)
		pwr_cfg &= ~TAC5X1X_PWR_CFG_DAC_PDZ;

	if (IS_ADC_CH(tac5x1x->ch_enabled) == 0) {
		pwr_cfg &= ~TAC5X1X_PWR_CFG_ADC_PDZ;
		pwr_cfg &= ~TAC5X1X_PWR_CFG_MICBIAS;
	}

	snd_soc_component_write(component, TAC5X1X_CH_EN, tac5x1x->ch_enabled);
	snd_soc_component_update_bits(component, TAC5X1X_PWR_CFG, mask, pwr_cfg);
	print_regs();
	mutex_unlock(&tac5x1x->ch_lock);
}

static int tac5x1x_enable_channel_unlocked(struct snd_soc_component *comp,
					 bool is_adc, s32 right)
{
	s32 ret;
	u8 mask_dev;
	u8 mask;
	struct tac5x1x_priv *tac5x1x;

	tac5x1x = snd_soc_component_get_drvdata(comp);
	if (right) {
		mask_dev = TAC5X1X_CH_EN_ADC_CH2 | TAC5X1X_CH_EN_DAC_CH2;
		mask = is_adc ? TAC5X1X_CH_EN_ADC_CH2 : TAC5X1X_CH_EN_DAC_CH2;
	} else {
		mask_dev = TAC5X1X_CH_EN_ADC_CH1 | TAC5X1X_CH_EN_DAC_CH1;
		mask = is_adc ? TAC5X1X_CH_EN_ADC_CH1 : TAC5X1X_CH_EN_DAC_CH1;
	}
	tac5x1x->ch_enabled |= mask;
	ret = snd_soc_component_update_bits(comp, TAC5X1X_CH_EN,
					    mask_dev, mask_dev);

	return ret;
}

static int tac5x1x_disable_channel_unlocked(struct snd_soc_component *comp,
					    bool is_adc, s32 right)
{
	u8 mask;
	s32 ret;
	struct tac5x1x_priv *tac5x1x;

	tac5x1x = snd_soc_component_get_drvdata(comp);
	mask = is_adc ?
		(right ? TAC5X1X_CH_EN_ADC_CH2 : TAC5X1X_CH_EN_ADC_CH1) :
		(right ? TAC5X1X_CH_EN_DAC_CH2 : TAC5X1X_CH_EN_DAC_CH1);
	tac5x1x->ch_enabled &= ~mask;
	ret = snd_soc_component_update_bits(comp, TAC5X1X_CH_EN, mask, 0);

	return ret;
}

/*
 * When ADC and DAC are enabled with time delay between them
 * the one which is started latter doesn't work because of HW bug.
 * So DAC and ADC events with dealyed work is added to follow a
 * particular powerup sequence.
*/
static int tac5x1x_dac_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	u8 pwr_cfg;
	int right = w->shift;
	int ret = 0;
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct tac5x1x_priv *tac5x1x = snd_soc_component_get_drvdata(component);

	mutex_lock(&tac5x1x->ch_lock);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = tac5x1x_enable_channel_unlocked(component, false, right);
		if (ret < 0) {
			dev_err(component->dev,
				"Failed to update enable DAC channels\n");
			break;
		}

		pwr_cfg = TAC5X1X_PWR_CFG_DAC_PDZ |
			TAC5X1X_PWR_CFG_ADC_PDZ |
			TAC5X1X_PWR_CFG_MICBIAS;
		ret = snd_soc_component_write(component,
						TAC5X1X_PWR_CFG,
						pwr_cfg);
		if (ret) {
			dev_err(component->dev,
				"Failed to power up DAC\n");
			tac5x1x_disable_channel_unlocked(component,
							false, right);
			break;
		}
		print_regs();
		schedule_delayed_work(&tac5x1x->powerup_work,
				      msecs_to_jiffies(0));
		break;

	case SND_SOC_DAPM_PRE_PMD:
		tac5x1x_disable_channel_unlocked(component, false, right);
		if (IS_DAC_CH(tac5x1x->ch_enabled) == 0)
			snd_soc_component_update_bits(component, TAC5X1X_PWR_CFG,
						      TAC5X1X_PWR_CFG_DAC_PDZ, 0);
		print_regs();
		break;
	}
	mutex_unlock(&tac5x1x->ch_lock);

	return ret;
}

static int tac5x1x_adc_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	u8 pwr_cfg;
	u32 right = w->shift;
	s32 ret = 0;
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct tac5x1x_priv *tac5x1x = snd_soc_component_get_drvdata(c);

	mutex_lock(&tac5x1x->ch_lock);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = tac5x1x_enable_channel_unlocked(c, true, right);
		if (ret < 0) {
			dev_err(c->dev,
				"Failed to update enable ADC channels\n");
			break;
		}

		pwr_cfg = TAC5X1X_PWR_CFG_ADC_PDZ |
			TAC5X1X_PWR_CFG_MICBIAS | 
			TAC5X1X_PWR_CFG_DAC_PDZ;
		ret = snd_soc_component_write(c, TAC5X1X_PWR_CFG,
						pwr_cfg);
		if (ret) {
			dev_err(c->dev,
				"Failed to power up ADC\n");
			tac5x1x_disable_channel_unlocked(c, true, right);
		}

		print_regs();
		schedule_delayed_work(&tac5x1x->powerup_work,
				      msecs_to_jiffies(0));
		break;

	case SND_SOC_DAPM_PRE_PMD:
		tac5x1x_disable_channel_unlocked(c, true, right);
		if (IS_ADC_CH(tac5x1x->ch_enabled) == 0)
			snd_soc_component_update_bits(c, TAC5X1X_PWR_CFG,
						      TAC5X1X_PWR_CFG_ADC_PDZ |
						      TAC5X1X_PWR_CFG_MICBIAS,
						      0);
		print_regs()
		break;
	}
	mutex_unlock(&tac5x1x->ch_lock);

	return ret;
}

static int tac5x1x_pdm_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	u32 right = w->shift;
	s32 ret = 0;
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct tac5x1x_priv *tac5x1x = snd_soc_component_get_drvdata(c);

	mutex_lock(&tac5x1x->ch_lock);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = tac5x1x_enable_channel_unlocked(c, true, right);
		if (ret < 0) {
			dev_err(c->dev,
				"Failed to update enable ADC channels\n");
			break;
		}

		ret = snd_soc_component_write(c, TAC5X1X_PWR_CFG, TAC5X1X_PWR_CFG_ADC_PDZ);
		if (ret) {
			dev_err(c->dev,
				"Failed to power up ADC\n");
			tac5x1x_disable_channel_unlocked(c, true, right);
		}

		print_regs();
		schedule_delayed_work(&tac5x1x->powerup_work,
				      msecs_to_jiffies(0));
		break;

	case SND_SOC_DAPM_PRE_PMD:
		tac5x1x_disable_channel_unlocked(c, true, right);
		if (IS_ADC_CH(tac5x1x->ch_enabled) == 0)
			snd_soc_component_update_bits(c, TAC5X1X_PWR_CFG,
							 TAC5X1X_PWR_CFG_ADC_PDZ, 0);
		print_regs()
		break;
	}
	mutex_unlock(&tac5x1x->ch_lock);

	return ret;
}

static void process_one_interrupt(struct tac5x1x_priv *tac5x1x, s32 index,
				  s32 value)
{
	u32 map_count, i;
	const struct mask_to_txt *map_items;

	map_count = intr_info_list[index].count;
	map_items = intr_info_list[index].mask_str_map;

	for (i = 0; i < map_count; i++) {
		if (value & map_items[i].mask)
			dev_dbg(tac5x1x->dev, "Interrupt %s detected\n",
				map_items[i].name);
	}
}

static int tac5x1x_read_latch_registers(struct tac5x1x_priv *tac5x1x)
{
	s32 ret, i, latch_count;

	ret = 0;
	latch_count = ARRAY_SIZE(intr_info_list);

	for (i = 0; i < latch_count; i++) {
		ret = regmap_read(tac5x1x->regmap,
				  tac5x1x->irqinfo.latch_regs[i],
				  &tac5x1x->irqinfo.latch_data[i]);
		if (ret)
			break;
	}

	return ret;
}

static irqreturn_t irq_thread_func(s32 irq, void *dev_id)
{
	u8 latch_set = 0;
	u32 latch_count;
	s32 i, ret;
	struct tac5x1x_priv *tac5x1x = (struct tac5x1x_priv *)dev_id;

	latch_count = ARRAY_SIZE(intr_info_list);

	ret = tac5x1x_read_latch_registers(tac5x1x);
	if (ret) {
		dev_err(tac5x1x->dev,
			"interrupt: latch register read failed");
		return IRQ_NONE;
	}

	for (i = 0; i < latch_count; i++) {
		dev_dbg(tac5x1x->dev, "reg=0x%0x, val=0x%02x",
			tac5x1x->irqinfo.latch_regs[i],
			tac5x1x->irqinfo.latch_data[i]);
		latch_set |= tac5x1x->irqinfo.latch_data[i] & 0xff;
	}

	if (!latch_set)
		return IRQ_NONE;

	for (i = 0; i < latch_count; i++) {
		if (!tac5x1x->irqinfo.latch_data[i])
			continue;
		process_one_interrupt(tac5x1x, i,
				      tac5x1x->irqinfo.latch_data[i]);
		tac5x1x->irqinfo.latch_data[i] = 0;
	}

	return IRQ_HANDLED;
}

static s32 tac5x1x_register_interrupt(struct tac5x1x_priv *tac5x1x)
{
	struct device_node *np = tac5x1x->dev->of_node;
	s32 ret, latch_count, i;
	u32 *latch_regs;
	u32 *latch_data;

	latch_count = ARRAY_SIZE(intr_info_list);
	tac5x1x->irqinfo.irq = of_irq_get(np, 0);
	if (tac5x1x->irqinfo.irq < 0)
		return tac5x1x->irqinfo.irq;

	latch_regs = devm_kzalloc(tac5x1x->dev, latch_count * sizeof(u32),
				  GFP_KERNEL);
	latch_data = devm_kzalloc(tac5x1x->dev, latch_count * sizeof(u32),
				  GFP_KERNEL);
	if (!latch_data || !latch_regs)
		return -ENOMEM;

	for (i = 0; i < latch_count; i++)
		latch_regs[i] = intr_info_list[i].reg;

	tac5x1x->irqinfo.latch_regs = latch_regs;
	tac5x1x->irqinfo.latch_data = latch_data;

	ret = devm_request_threaded_irq(tac5x1x->dev, tac5x1x->irqinfo.irq,
					NULL, irq_thread_func,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"TAC-IRQ", tac5x1x);
	if (ret)
		dev_err(tac5x1x->dev, "request irq failed, irq=%d ret %d\n",
			tac5x1x->irqinfo.irq, ret);

	return ret;
}

/* Record */
/* ADC Analog/PDM Selection */
static const char *const tac5x1x_input_source_text[] = {"Analog", "PDM"};

static SOC_ENUM_SINGLE_DECL(tac5x1x_in1_source_enum, TAC5X1X_INTF4, 7,
		tac5x1x_input_source_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_in2_source_enum, TAC5X1X_INTF4, 6,
		tac5x1x_input_source_text);

static const struct snd_kcontrol_new tac5x1x_dapm_in1_source_control[] = {
	SOC_DAPM_ENUM("CH1 Source MUX", tac5x1x_in1_source_enum),
};

static const struct snd_kcontrol_new tac5x1x_dapm_in2_source_control[] = {
	SOC_DAPM_ENUM("CH2 Source MUX", tac5x1x_in2_source_enum),
};

static const char *const tad5x1x_input_source_text[] = {"Disable", "PDM"};
static SOC_ENUM_SINGLE_DECL(tad5x1x_in1_source_enum, TAC5X1X_INTF4, 7,
		tad5x1x_input_source_text);
static SOC_ENUM_SINGLE_DECL(tad5x1x_in2_source_enum, TAC5X1X_INTF4, 6,
		tad5x1x_input_source_text);

static const struct snd_kcontrol_new tad5x1x_dapm_in1_source_control[] = {
	SOC_DAPM_ENUM("CH1 Source MUX", tad5x1x_in1_source_enum),
};

static const struct snd_kcontrol_new tad5x1x_dapm_in2_source_control[] = {
	SOC_DAPM_ENUM("CH2 Source MUX", tad5x1x_in2_source_enum),
};

/* ADC Analog source Selection */
static const char *const tac5x1x_input_analog_sel_text[] = {
	"Differential",
	"Single-ended",
	"Single-ended mux INxP",
	"Single-ended mux INxM",
};

static const char *const tac5x1x_input_analog2_sel_text[] = {
	"Differential",
	"Single-ended",
};

static SOC_ENUM_SINGLE_DECL(tac5x1x_adc1_config_enum, TAC5X1X_ADCCH1C0, 6,
		tac5x1x_input_analog_sel_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_adc2_config_enum, TAC5X1X_ADCCH2C0, 6,
		tac5x1x_input_analog2_sel_text);

static const struct snd_kcontrol_new tac5x1x_dapm_adc1_config_control[] = {
	SOC_DAPM_ENUM("ADC1 Analog MUX", tac5x1x_adc1_config_enum),
};

static const struct snd_kcontrol_new tac5x1x_dapm_adc2_config_control[] = {
	SOC_DAPM_ENUM("ADC2 Analog MUX", tac5x1x_adc2_config_enum),
};

/*
 * ADC full-scale selection
 * 2/10-VRMS is for TAX52xx/TAX51xx devices
 * 4/5-VRMS is for TAX54xx/TAX53xx devices
 */
static const char *const tac5x1x_adc_fscale_text[] = {"2/10-VRMS",
	"4/5-VRMS"};

static SOC_ENUM_SINGLE_DECL(tac5x1x_adc1_fscale_enum, TAC5X1X_ADCCH1C0, 1,
		tac5x1x_adc_fscale_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_adc2_fscale_enum, TAC5X1X_ADCCH2C0, 1,
		tac5x1x_adc_fscale_text);

static const struct snd_kcontrol_new tac5x1x_dapm_adc1_fscale_control[] = {
	SOC_DAPM_ENUM("ADC1 FSCALE MUX", tac5x1x_adc1_fscale_enum),
};

static const struct snd_kcontrol_new tac5x1x_dapm_adc2_fscale_control[] = {
	SOC_DAPM_ENUM("ADC2 FSCALE MUX", tac5x1x_adc2_fscale_enum),
};

static const char *const pdmclk_text[] = {
	"2.8224 MHz or 3.072 MHz", "1.4112 MHz or 1.536 MHz",
	"705.6 kHz or 768 kHz", "5.6448 MHz or 6.144 MHz"};

static SOC_ENUM_SINGLE_DECL(pdmclk_select_enum, TAC5X1X_CNTCLK0, 6,
		pdmclk_text);

/* Digital Volume control. From -80 to 47 dB in 0.5 dB steps */
static DECLARE_TLV_DB_SCALE(record_dig_vol_tlv, -8000, 50, 0);

/* Gain Calibration control. From -0.8db to 0.7db dB in 0.1 dB steps */
static DECLARE_TLV_DB_MINMAX(record_gain_cali_tlv, -80, 70);

/* Analog Level control. From -12 to 24 dB in 6 dB steps */
static DECLARE_TLV_DB_SCALE(playback_analog_level_tlv, -1200, 600, 0);

/* Digital Volume control. From -100 to 27 dB in 0.5 dB steps */
static DECLARE_TLV_DB_SCALE(dac_dig_vol_tlv, -10000, 50, 0); // mute ?

/* Gain Calibration control. From -0.8db to 0.7db dB in 0.1 dB steps */
static DECLARE_TLV_DB_MINMAX(playback_gain_cali_tlv, -80, 70);

/* Output Source Selection */
static const char *const tac5x1x_output_source_text[] = {
	"Disabled",
	"DAC Input",
	"Analog Bypass",
	"DAC + Analog Bypass Mix",
	"DAC -> OUTxP, INxP -> OUTxM",
	"INxM -> OUTxP, DAC -> OUTxM",
};

static SOC_ENUM_SINGLE_DECL(tac5x1x_out1_source_enum, TAC5X1X_OUT1CFG0, 5,
		tac5x1x_output_source_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_out2_source_enum, TAC5X1X_OUT2CFG0, 5,
		tac5x1x_output_source_text);

static const struct snd_kcontrol_new tac5x1x_dapm_out1_source_control[] = {
	SOC_DAPM_ENUM("OUT1X MUX", tac5x1x_out1_source_enum),
};

static const struct snd_kcontrol_new tac5x1x_dapm_out2_source_control[] = {
	SOC_DAPM_ENUM("OUT2X MUX", tac5x1x_out2_source_enum),
};

/* Output Config Selection */
static const char *const tac5x1x_output_config_text[] = {
	"Differential",
	"Stereo Single-ended",
	"Mono Single-ended at OUTxP only",
	"Mono Single-ended at OUTxM only",
	"Pseudo differential with OUTxM as VCOM",
	"Pseudo differential with OUTxM as external sensing",
	"Pseudo differential with OUTxP as VCOM",
};

static const char *const tac5x1x_output2_config_text[] = {
	"Differential",
	"Stereo Single-ended",
	"Mono Single-ended at OUTxP only",
	"Mono Single-ended at OUTxM only",
	"Pseudo differential with OUTxM as VCOM",
	"Pseudo differential with OUTxP as VCOM",
};

static const s32 tac5x1x_output2_config_values[] = {
	0, 1, 2, 3, 4, 6
};

static SOC_ENUM_SINGLE_DECL(tac5x1x_out1_config_enum, TAC5X1X_OUT1CFG0, 2,
			tac5x1x_output_config_text);
static SOC_VALUE_ENUM_SINGLE_DECL(tac5x1x_out2_config_enum,
				  TAC5X1X_OUT2CFG0, 2, 0x7,
				  tac5x1x_output2_config_text,
				  tac5x1x_output2_config_values);

static const struct snd_kcontrol_new tac5x1x_dapm_out1_config_control[] = {
	SOC_DAPM_ENUM("OUT1X Config MUX", tac5x1x_out1_config_enum),
};

static const struct snd_kcontrol_new tac5x1x_dapm_out2_config_control[] = {
	SOC_DAPM_ENUM("OUT2X Config MUX", tac5x1x_out2_config_enum),
};

static const char *const tac5x1x_wideband_text[] = {
	"Audio BW 24-kHz",
	"Wide BW 96-kHz",
};

static SOC_ENUM_SINGLE_DECL(tac5x1x_adc1_wideband_enum, TAC5X1X_ADCCH1C0, 0,
		tac5x1x_wideband_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_adc2_wideband_enum, TAC5X1X_ADCCH2C0, 0,
		tac5x1x_wideband_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_dac1_wideband_enum, TAC5X1X_OUT1CFG1, 0,
		tac5x1x_wideband_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_dac2_wideband_enum, TAC5X1X_OUT2CFG1, 0,
		tac5x1x_wideband_text);

static const char *const tac5x1x_tolerance_text[] = {
	"AC Coupled with 100mVpp",
	"AC/DC Coupled with 1Vpp",
	"AC/DC Coupled with Rail-to-rail",
};

static SOC_ENUM_SINGLE_DECL(tac5x1x_adc1_tolerance_enum, TAC5X1X_ADCCH1C0, 2,
		tac5x1x_tolerance_text);
static SOC_ENUM_SINGLE_DECL(tac5x1x_adc2_tolerance_enum, TAC5X1X_ADCCH2C0, 2,
		tac5x1x_tolerance_text);

/* Output Drive Selection */
static const char *const tac5x1x_output_driver_text[] = {
	"Line-out",
	"Headphone",
	"4 ohm",
	"FD Receiver/Debug",
};

static SOC_ENUM_SINGLE_DECL(out1p_driver_enum, TAC5X1X_OUT1CFG1, 6,
		tac5x1x_output_driver_text);

static SOC_ENUM_SINGLE_DECL(out2p_driver_enum, TAC5X1X_OUT2CFG1, 6,
		tac5x1x_output_driver_text);

static const struct snd_kcontrol_new tac5x1x_dapm_out1_driver_control[] = {
	SOC_DAPM_ENUM("OUT1 driver MUX", out1p_driver_enum),
};

static const struct snd_kcontrol_new tac5x1x_dapm_out2_driver_control[] = {
	SOC_DAPM_ENUM("OUT2 driver MUX", out2p_driver_enum),
};

/* Decimation Filter Selection */
static const char *const decimation_filter_text[] = {
	"Linear Phase", "Low Latency", "Ultra-low Latency"};

static SOC_ENUM_SINGLE_DECL(decimation_filter_record_enum, TAC5X1X_DSP0, 6,
			decimation_filter_text);
static SOC_ENUM_SINGLE_DECL(decimation_filter_playback_enum, TAC5X1X_DSP1, 6,
			    decimation_filter_text);

static const struct snd_kcontrol_new tx_ch1_asi_switch =
	SOC_DAPM_SINGLE("Capture Switch", TAC5X1X_PASITXCH1, 5, 1, 0);
static const struct snd_kcontrol_new tx_ch2_asi_switch =
	SOC_DAPM_SINGLE("Capture Switch", TAC5X1X_PASITXCH2, 5, 1, 0);
static const struct snd_kcontrol_new tx_ch3_asi_switch =
	SOC_DAPM_SINGLE("Capture Switch", TAC5X1X_PASITXCH3, 5, 1, 0);
static const struct snd_kcontrol_new tx_ch4_asi_switch =
	SOC_DAPM_SINGLE("Capture Switch", TAC5X1X_PASITXCH4, 5, 1, 0);

static const struct snd_kcontrol_new rx_ch1_asi_switch =
	SOC_DAPM_SINGLE("Switch", TAC5X1X_PASIRXCH1, 5, 1, 0);
static const struct snd_kcontrol_new rx_ch2_asi_switch =
	SOC_DAPM_SINGLE("Switch", TAC5X1X_PASIRXCH2, 5, 1, 0);

static const char *const rx_ch5_asi_cfg_text[] = {
		"Disable",
		"DAC channel data",
		"ADC channel output loopback",
};

static const char *const rx_ch6_asi_cfg_text[] = {
		"Disable",
		"DAC channel data",
		"ADC channel output loopback",
		"Channel Input to ICLA device",
};

static const char *const tx_ch5_asi_cfg_text[] = {
		"Tristate",
		"Input Channel Loopback data",
		"Echo reference Channel data",
};

static const char *const tx_ch7_asi_cfg_text[] = {
		"Tristate",
		"Vbat_Wlby2,Temp_Wlby2",
		"echo_ref_ch1,echo_ref_ch2",
};

static const char *const tx_ch8_asi_cfg_text[] = {
		"Tristate",
		"ICLA data",
};

static const char *const diag_cfg_text[] = {
	"0mv", "30mv", "60mv", "90mv",
	"120mv", "150mv", "180mv", "210mv",
	"240mv", "270mv", "300mv", "330mv",
	"360mv", "390mv", "420mv", "450mv",
};

static const char *const diag_cfg_gnd_text[] = {
	"0mv", "60mv", "120mv", "180mv",
	"240mv", "300mv", "360mv", "420mv",
	"480mv", "540mv", "600mv", "660mv",
	"720mv", "780mv", "840mv", "900mv",
};

static SOC_ENUM_SINGLE_DECL(tx_ch5_asi_cfg_enum, TAC5X1X_PASITXCH5, 5,
		tx_ch5_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(tx_ch6_asi_cfg_enum, TAC5X1X_PASITXCH6, 5,
		tx_ch5_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(tx_ch7_asi_cfg_enum, TAC5X1X_PASITXCH7, 5,
		tx_ch7_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(tx_ch8_asi_cfg_enum, TAC5X1X_PASITXCH8, 5,
		tx_ch8_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(rx_ch5_asi_cfg_enum, TAC5X1X_PASIRXCH5, 5,
		rx_ch5_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(rx_ch6_asi_cfg_enum, TAC5X1X_PASIRXCH6, 5,
		rx_ch6_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(rx_ch7_asi_cfg_enum, TAC5X1X_PASIRXCH7, 5,
		rx_ch6_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(rx_ch8_asi_cfg_enum, TAC5X1X_PASIRXCH8, 5,
		rx_ch6_asi_cfg_text);
static SOC_ENUM_SINGLE_DECL(diag_cfg1_sht_term_enum, TAC5X1X_DIAG_CFG1, 4,
		diag_cfg_text);
static SOC_ENUM_SINGLE_DECL(diag_cfg1_vbat_in_enum, TAC5X1X_DIAG_CFG1, 0,
		diag_cfg_text);
static SOC_ENUM_SINGLE_DECL(diag_cfg2_sht_gnd_enum, TAC5X1X_DIAG_CFG2, 4,
		diag_cfg_gnd_text);
static SOC_ENUM_SINGLE_DECL(diag_cfg2_micbias, TAC5X1X_DIAG_CFG2, 0,
		diag_cfg_text);

static const struct snd_kcontrol_new taa5x1x_controls[] = {
	SOC_ENUM("Record Decimation Filter",
		 decimation_filter_record_enum),
	SOC_ENUM("ADC1 Audio BW", tac5x1x_adc1_wideband_enum),

	SOC_SINGLE_TLV("ADC1 Digital Capture Volume", TAC5X1X_ADCCH1C2,
		       0, 0xff, 0, record_dig_vol_tlv),

	SOC_SINGLE_TLV("ADC1 Fine Capture Volume", TAC5X1X_ADCCH1C3,
		       0, 0xff, 0, record_gain_cali_tlv),

	SOC_SINGLE_RANGE("ADC1 Phase Capture Volume", TAC5X1X_ADCCH1C4,
			 2, 0, 63, 0),

	SOC_ENUM("ASI_TX_CH5_CFG", tx_ch5_asi_cfg_enum),
	SOC_ENUM("ASI_TX_CH6_CFG", tx_ch6_asi_cfg_enum),
	SOC_ENUM("ASI_TX_CH7_CFG", tx_ch7_asi_cfg_enum),
	SOC_ENUM("ASI_TX_CH8_CFG", tx_ch8_asi_cfg_enum),
	//SOC_SINGLE("IN_CH1_EN Capture Switch", TAC5X1X_CH_EN, 7, 1, 0),
	//SOC_SINGLE("IN_CH2_EN Capture Switch", TAC5X1X_CH_EN, 6, 1, 0),
	//SOC_SINGLE("IN_CH3_EN Capture Switch", TAC5X1X_CH_EN, 5, 1, 0),
	//SOC_SINGLE("IN_CH4_EN Capture Switch", TAC5X1X_CH_EN, 4, 1, 0),
};

static const struct snd_kcontrol_new tad5x1x_controls[] = {
	SOC_ENUM("Playback Decimation Filter",
		 decimation_filter_playback_enum),
	SOC_ENUM("DAC1 Audio BW", tac5x1x_dac1_wideband_enum),
	SOC_SINGLE_TLV("OUT1P Analog Level Playback Volume", TAC5X1X_OUT1CFG1,
		       3, 6, 1, playback_analog_level_tlv),
	SOC_SINGLE_TLV("OUT1M Analog Level Playback Volume", TAC5X1X_OUT1CFG2,
		       3, 6, 1, playback_analog_level_tlv),
	SOC_SINGLE_TLV("DAC1 CHA Digital Playback Volume", TAC5X1X_DACCH1A0,
		       0, 0xff, 0, dac_dig_vol_tlv),
	SOC_SINGLE_TLV("DAC1 CHB Digital Playback Volume", TAC5X1X_DACCH1B0,
		       0, 0xff, 0, dac_dig_vol_tlv),
	SOC_SINGLE_TLV("DAC1 CHA Gain Calibration Playback Volume",
		       TAC5X1X_DACCH1A1, 4, 0xf, 0,
		       playback_gain_cali_tlv),
	SOC_SINGLE_TLV("DAC1 CHB Gain Calibration Playback Volume",
		       TAC5X1X_DACCH1B1, 4, 0xf, 0,
		       playback_gain_cali_tlv),

	SOC_SINGLE("ASI_RX_CH3_EN Playback Switch",
		   TAC5X1X_PASIRXCH3, 5, 1, 0),
	SOC_SINGLE("ASI_RX_CH4_EN Playback Switch",
		   TAC5X1X_PASIRXCH4, 5, 1, 0),
	SOC_ENUM("ASI_RX_CH5_EN Playback", rx_ch5_asi_cfg_enum),
	SOC_ENUM("ASI_RX_CH6_EN Playback", rx_ch6_asi_cfg_enum),
	SOC_ENUM("ASI_RX_CH7_EN Playback", rx_ch7_asi_cfg_enum),
	SOC_ENUM("ASI_RX_CH8_EN Playback", rx_ch8_asi_cfg_enum),
	//SOC_SINGLE("OUT_CH1_EN Playback Switch", TAC5X1X_CH_EN, 3, 1, 0),
	//SOC_SINGLE("OUT_CH2_EN Playback Switch", TAC5X1X_CH_EN, 2, 1, 0),
	//SOC_SINGLE("OUT_CH3_EN Playback Switch", TAC5X1X_CH_EN, 1, 1, 0),
	//SOC_SINGLE("OUT_CH4_EN Playback Switch", TAC5X1X_CH_EN, 0, 1, 0),
};

static const struct snd_kcontrol_new tac5x11_controls[] = {
	SOC_ENUM("ADC1 Common-mode Tolerance", tac5x1x_adc1_tolerance_enum),
};

static const struct snd_kcontrol_new tad5x12_controls[] = {
	SOC_SINGLE_TLV("OUT2P Analog Level Playback Volume", TAC5X1X_OUT2CFG1,
		       3, 6, 1, playback_analog_level_tlv),
	SOC_SINGLE_TLV("OUT2M Analog Level Playback Volume", TAC5X1X_OUT2CFG2,
		       3, 6, 1, playback_analog_level_tlv),
	SOC_SINGLE_TLV("DAC2 CHA Digital Playback Volume", TAC5X1X_DACCH2A0,
		       0, 0xff, 0, dac_dig_vol_tlv),
	SOC_SINGLE_TLV("DAC2 CHB Digital Playback Volume", TAC5X1X_DACCH2B0,
		       0, 0xff, 0, dac_dig_vol_tlv),
	SOC_SINGLE_TLV("DAC2 CHA Gain Calibration Playback Volume",
		       TAC5X1X_DACCH2A1, 4, 0xf, 0,
			playback_gain_cali_tlv),
	SOC_SINGLE_TLV("DAC2 CHB Gain Calibration Playback Volume",
		       TAC5X1X_DACCH2B1, 4, 0xf, 0,
			playback_gain_cali_tlv),
	SOC_ENUM("DAC2 Audio BW", tac5x1x_dac2_wideband_enum),
};

static const struct snd_kcontrol_new taa5x12_controls[] = {
	SOC_ENUM("ADC2 Audio BW", tac5x1x_adc2_wideband_enum),

	SOC_SINGLE_TLV("ADC2 Digital Capture Volume", TAC5X1X_ADCCH2C2,
		       0, 0xff, 0, record_dig_vol_tlv),

	SOC_SINGLE_TLV("ADC2 Fine Capture Volume", TAC5X1X_ADCCH2C3,
		       0, 0xff, 0, record_gain_cali_tlv),

	SOC_SINGLE_RANGE("ADC2 Phase Capture Volume", TAC5X1X_ADCCH2C4,
			 2, 0, 63, 0),
};

static const struct snd_kcontrol_new tolerance_ctrls[] = {
	SOC_ENUM("ADC1 Common-mode Tolerance", tac5x1x_adc1_tolerance_enum),
	SOC_ENUM("ADC2 Common-mode Tolerance", tac5x1x_adc2_tolerance_enum),
};

static const struct snd_kcontrol_new tac5x1x_pdm_controls[] = {
	SOC_ENUM("PDM Clk Divider", pdmclk_select_enum),

	SOC_SINGLE_TLV("PDM1 Digital Capture Volume", TAC5X1X_ADCCH1C2,
		       0, 0xff, 0, record_dig_vol_tlv),
	SOC_SINGLE_TLV("PDM2 Digital Capture Volume", TAC5X1X_ADCCH2C2,
		       0, 0xff, 0, record_dig_vol_tlv),
	SOC_SINGLE_TLV("PDM1 Fine Capture Volume", TAC5X1X_ADCCH1C3,
		       0, 0xff, 0, record_gain_cali_tlv),
	SOC_SINGLE_TLV("PDM2 Fine Capture Volume", TAC5X1X_ADCCH2C3,
		       0, 0xff, 0, record_gain_cali_tlv),
	SOC_SINGLE_RANGE("PDM1 Phase Capture Volume", TAC5X1X_ADCCH1C4,
			 2, 0, 63, 0),
	SOC_SINGLE_RANGE("PDM2 Phase Capture Volume", TAC5X1X_ADCCH2C4,
			 2, 0, 63, 0),
	SOC_SINGLE_TLV("PDM3 Digital Capture Volume", TAC5X1X_ADCCH3C2,
		       0, 0xff, 0, record_dig_vol_tlv),
	SOC_SINGLE_TLV("PDM4 Digital Capture Volume", TAC5X1X_ADCCH4C2,
		       0, 0xff, 0, record_dig_vol_tlv),
	SOC_SINGLE_TLV("PDM3 Fine Capture Volume", TAC5X1X_ADCCH3C3,
		       0, 0xff, 0, record_gain_cali_tlv),
	SOC_SINGLE_TLV("PDM4 Fine Capture Volume", TAC5X1X_ADCCH4C3,
		       0, 0xff, 0, record_gain_cali_tlv),
	SOC_SINGLE_RANGE("PDM3 Phase Capture Volume", TAC5X1X_ADCCH3C4,
			 2, 0, 63, 0),
	SOC_SINGLE_RANGE("PDM4 Phase Capture Volume", TAC5X1X_ADCCH4C4,
			 2, 0, 63, 0),
};

static const struct snd_kcontrol_new taa_ip_controls[] = {
	SOC_ENUM("DIAG_SHT_TERM", diag_cfg1_sht_term_enum),
	SOC_ENUM("DIAG_SHT_VBAT_IN", diag_cfg1_vbat_in_enum),
	SOC_ENUM("DIAG_SHT_GND", diag_cfg2_sht_gnd_enum),
	SOC_ENUM("DIAG_SHT_MICBIAS", diag_cfg2_micbias),
};

static const struct snd_soc_dapm_widget taa5x1x_dapm_widgets[] = {
	/* ADC1 */
	SND_SOC_DAPM_INPUT("AIN1"),
	SND_SOC_DAPM_MUX("ADC1 Full-Scale", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_adc1_fscale_control),
	SND_SOC_DAPM_MUX("ADC1 Config", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_adc1_config_control),
	SND_SOC_DAPM_ADC_E("CH1_ADC_EN", "CH1 Capture", SND_SOC_NOPM, 0, 0,
		tac5x1x_adc_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SWITCH("ASI_TX_CH1_EN", SND_SOC_NOPM, 0, 0,
			    &tx_ch1_asi_switch),
	SND_SOC_DAPM_MICBIAS("Mic Bias", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SWITCH("ASI_TX_CH2_EN", SND_SOC_NOPM, 0, 0,
			    &tx_ch2_asi_switch),
};

static const struct snd_soc_dapm_widget tad5xx_dapm_widgets[] = {
	/* pdm capture */
	SND_SOC_DAPM_SWITCH("ASI_TX_CH1_EN", SND_SOC_NOPM, 0, 0,
			    &tx_ch1_asi_switch),
	SND_SOC_DAPM_SWITCH("ASI_TX_CH2_EN", SND_SOC_NOPM, 0, 0,
			    &tx_ch2_asi_switch),
};

static const struct snd_soc_dapm_widget tad5x1x_dapm_widgets[] = {
	/* DAC1 */
	SND_SOC_DAPM_AIF_IN("ASI IN1", "ASI Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT1"),
	SND_SOC_DAPM_MUX("OUT1x Source", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_out1_source_control),
	SND_SOC_DAPM_MUX("OUT1x Config", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_out1_config_control),
	SND_SOC_DAPM_MUX("OUT1x Driver", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_out1_driver_control),
	SND_SOC_DAPM_DAC_E("Left DAC Enable", "Playback", SND_SOC_NOPM, 0, 0,
		tac5x1x_dac_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SWITCH("ASI_RX_CH1_EN", SND_SOC_NOPM, 0, 0,
			    &rx_ch1_asi_switch),
};

static const struct snd_soc_dapm_widget taa5x12_dapm_widgets[] = {
	/* ADC2 */
	SND_SOC_DAPM_INPUT("AIN2"),
	SND_SOC_DAPM_MUX("ADC2 Full-Scale", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_adc2_fscale_control),
	SND_SOC_DAPM_MUX("ADC2 Config", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_adc2_config_control),
	SND_SOC_DAPM_ADC_E("CH2_ADC_EN", "CH2 Capture", SND_SOC_NOPM, 1, 0,
		tac5x1x_adc_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

};

static const struct snd_soc_dapm_widget tad5x12_dapm_widgets[] = {
	/* DAC2 */
	SND_SOC_DAPM_OUTPUT("OUT2"),

	SND_SOC_DAPM_AIF_IN("ASI IN2", "ASI Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MUX("OUT2x Source", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_out2_source_control),
	SND_SOC_DAPM_MUX("OUT2x Config", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_out2_config_control),
	SND_SOC_DAPM_MUX("OUT2x Driver", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_out2_driver_control),
	SND_SOC_DAPM_DAC_E("Right DAC Enable", "Playback", SND_SOC_NOPM, 1, 0,
			   tac5x1x_dac_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SWITCH("ASI_RX_CH2_EN", SND_SOC_NOPM, 0, 0,
			    &rx_ch2_asi_switch),
};

static const struct snd_soc_dapm_widget tac5x1x_pdm_widgets[] = {
	/* PDM */
	SND_SOC_DAPM_INPUT("DIN1"),
	SND_SOC_DAPM_INPUT("DIN2"),
	SND_SOC_DAPM_INPUT("DIN3"),
	SND_SOC_DAPM_INPUT("DIN4"),

	SND_SOC_DAPM_ADC_E("CH1_PDM_EN", "PDM CH1 Capture", SND_SOC_NOPM, 0, 0,
			   tac5x1x_pdm_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_ADC_E("CH2_PDM_EN", "PDM CH2 Capture", SND_SOC_NOPM, 1, 0,
			   tac5x1x_pdm_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_ADC("CH3_PDM_EN", "PDM CH3 Capture", TAC5X1X_CH_EN, 5, 0),
	SND_SOC_DAPM_ADC("CH4_PDM_EN", "PDM CH4 Capture", TAC5X1X_CH_EN, 4, 0),

	SND_SOC_DAPM_SWITCH("ASI_TX_CH3_EN", SND_SOC_NOPM, 0, 0,
			    &tx_ch3_asi_switch),
	SND_SOC_DAPM_SWITCH("ASI_TX_CH4_EN", SND_SOC_NOPM, 0, 0,
			    &tx_ch4_asi_switch),
};

static const struct snd_soc_dapm_widget tac5x1x_common_widgets[] = {
	SND_SOC_DAPM_MUX("IN1 Source Mux", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_in1_source_control),
	SND_SOC_DAPM_MUX("IN2 Source Mux", SND_SOC_NOPM, 0, 0,
			 tac5x1x_dapm_in2_source_control),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "ASI Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_widget tad_common_widgets[] = {
	SND_SOC_DAPM_MUX("IN1 Source Mux", SND_SOC_NOPM, 0, 0,
			 tad5x1x_dapm_in1_source_control),
	SND_SOC_DAPM_MUX("IN2 Source Mux", SND_SOC_NOPM, 0, 0,
			 tad5x1x_dapm_in2_source_control),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "ASI Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route taa5x1x_dapm_routes[] = {
	/* ADC channel1 */
	{"IN1 Source Mux", "Analog", "AIN1"},
	{"IN2 Source Mux", "Analog", "IN1 Source Mux"},
	{"ADC1 Config", "Differential", "ASI_TX_CH1_EN"},
	{"ADC1 Config", "Single-ended", "ASI_TX_CH1_EN"},
	{"ADC1 Config", "Single-ended mux INxP",
		"ASI_TX_CH1_EN"},
	{"ADC1 Config", "Single-ended mux INxM",
		"ASI_TX_CH1_EN"},

	{"ADC1 Full-Scale", "2/10-VRMS", "ADC1 Config"},
	{"ADC1 Full-Scale", "4/5-VRMS", "ADC1 Config"},
	{"Mic Bias", NULL, "ADC1 Full-Scale"},

};

static const struct snd_soc_dapm_route tad5x1x_dapm_routes[] = {
	/* Left Output */
	{"ASI_RX_CH1_EN", "Switch", "ASI IN1"},

	{"OUT1x Source", "DAC Input", "ASI_RX_CH1_EN"},
	{"OUT1x Source", "DAC + Analog Bypass Mix", "ASI_RX_CH1_EN"},
	{"OUT1x Source", "DAC -> OUTxP, INxP -> OUTxM", "ASI_RX_CH1_EN"},
	{"OUT1x Source", "INxM -> OUTxP, DAC -> OUTxM", "ASI_RX_CH1_EN"},

	{"OUT1x Config", "Differential", "OUT1x Source"},
	// {"OUT1x Config", "Stereo Single-ended", "OUT1x Source"},
	{"OUT1x Config", "Mono Single-ended at OUTxP only", "OUT1x Source"},
	{"OUT1x Config", "Mono Single-ended at OUTxM only", "OUT1x Source"},
	{"OUT1x Config", "Pseudo differential with OUTxM as VCOM",
		"OUT1x Source"},
	{"OUT1x Config", "Pseudo differential with OUTxM as external sensing",
		"OUT1x Source"},
	{"OUT1x Config", "Pseudo differential with OUTxP as VCOM",
		"OUT1x Source"},

	{"OUT1x Driver", "Line-out", "OUT1x Config"},
	{"OUT1x Driver", "Headphone", "OUT1x Config"},

	{"Left DAC Enable", NULL, "OUT1x Driver"},
	{"OUT1", NULL, "Left DAC Enable"},
};

static const struct snd_soc_dapm_route taa5x12_dapm_routes[] = {
	/* ADC channel2 */
	{"CH2_ADC_EN", NULL, "AIN2"},
	{"ASI_TX_CH2_EN", "Capture Switch", "CH2_ADC_EN"},
	{"ADC2 Config", "Differential", "ASI_TX_CH2_EN"},
	{"ADC2 Config", "Single-ended", "ASI_TX_CH2_EN"},
	{"ADC2 Full-Scale", "2/10-VRMS",
		"ADC2 Config"},
	{"ADC2 Full-Scale", "4/5-VRMS",
		"ADC2 Config"},

	{"Mic Bias", NULL, "ADC2 Full-Scale"},
};

static const struct snd_soc_dapm_route tad5x12_dapm_routes[] = {
	/* Right Output */
	{"ASI_RX_CH2_EN", "Switch", "ASI IN2"},

	{"OUT2x Source", "DAC + Analog Bypass Mix", "ASI_RX_CH1_EN"},
	{"OUT2x Source", "DAC -> OUTxP, INxP -> OUTxM", "ASI_RX_CH1_EN"},
	{"OUT2x Source", "INxM -> OUTxP, DAC -> OUTxM", "ASI_RX_CH1_EN"},

	{"OUT2x Config", "Differential", "OUT2x Source"},
	// {"OUT2x Config", "Stereo Single-ended", "OUT2x Source"},
	{"OUT2x Config", "Mono Single-ended at OUTxP only", "OUT2x Source"},
	{"OUT2x Config", "Mono Single-ended at OUTxM only", "OUT2x Source"},
	{"OUT2x Config", "Pseudo differential with OUTxM as VCOM",
		"OUT2x Source"},
	{"OUT2x Config", "Pseudo differential with OUTxP as VCOM",
		"OUT2x Source"},
	{"OUT2x Driver", "Line-out", "OUT2x Config"},
	{"OUT2x Driver", "Headphone", "OUT2x Config"},
	{"Right DAC Enable", NULL, "OUT2x Driver"},
	{"OUT2", NULL, "Right DAC Enable"},
};

static const struct snd_soc_dapm_route tac5x1x_pdm_routes[] = {
	/* PDM channel1 & Channel2 */
	{"IN1 Source Mux", "PDM", "DIN1"},
	{"IN2 Source Mux", "PDM", "DIN2"},

	{"ASI_TX_CH1_EN", "Capture Switch",
	"IN1 Source Mux"},
	{"ASI_TX_CH2_EN", "Capture Switch",
	"IN2 Source Mux"},

	{"CH1_PDM_EN", NULL, "ASI_TX_CH1_EN"},
	{"CH2_PDM_EN", NULL, "ASI_TX_CH2_EN"},

	{"AIF OUT", NULL, "CH1_PDM_EN"},
	{"AIF OUT", NULL, "CH2_PDM_EN"},

	/* PDM channel3 & Channel4 */
	{"ASI_TX_CH3_EN", "Capture Switch", "DIN3"},
	{"ASI_TX_CH4_EN", "Capture Switch", "DIN4"},

	{"CH3_PDM_EN", NULL, "ASI_TX_CH3_EN"},
	{"CH4_PDM_EN", NULL, "ASI_TX_CH4_EN"},

	{"AIF OUT", NULL, "CH3_PDM_EN"},
	{"AIF OUT", NULL, "CH4_PDM_EN"},
};

static const struct snd_soc_dapm_route tac_common_routes[] = {
	{"AIF OUT", NULL, "Mic Bias"},
};

static s32 tac5x1x_pwr_ctrl(struct snd_soc_component *component,
			    bool power_state)
{
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);
	s32 active_ctrl, ret;
	s32 pwr_ctrl = 0;

	if (power_state) {
		active_ctrl = TAC5X1X_VREF_SLEEP_ACTIVE_MASK;
		snd_soc_component_update_bits(component, TAC5X1X_VREFCFG,
					      TAC5X1X_VREFCFG_MICBIAS_VAL_MASK,
					      tac5x1x->micbias_vg << 2);
		snd_soc_component_update_bits(component, TAC5X1X_VREFCFG,
					      TAC5X1X_VREFCFG_VREF_FSCALE_MASK,
					      tac5x1x->vref_vg);

		if (tac5x1x->uad_en)
			pwr_ctrl |= TAC5X1X_PWR_CFG_UAD_EN;
		if (tac5x1x->vad_en)
			pwr_ctrl |= TAC5X1X_PWR_CFG_VAD_EN;
		if (tac5x1x->uag_en)
			pwr_ctrl |= TAC5X1X_PWR_CFG_UAG_EN;
	} else {
		active_ctrl = 0x0;
	}

	ret = snd_soc_component_update_bits(component, TAC5X1X_VREF,
					    TAC5X1X_VREF_SLEEP_EXIT_VREF_EN |
					    TAC5X1X_VREF_SLEEP_ACTIVE_MASK,
					    active_ctrl);
	if (ret < 0) {
		dev_err(tac5x1x->dev,
			"%s, device active or sleep failed!, ret %d\n",
			__func__, ret);
		return ret;
	}

	ret = snd_soc_component_update_bits(component, TAC5X1X_PWR_CFG,
					    TAC5X1X_PWR_CFG_UAD_EN |
					    TAC5X1X_PWR_CFG_UAG_EN |
					    TAC5X1X_PWR_CFG_VAD_EN, pwr_ctrl);
	if (ret < 0)
		dev_err(tac5x1x->dev,
			"%s, Power control set failed!, ret %d\n",
			__func__, ret);
	return ret;
}

static s32 tac5x1x_set_dai_fmt(struct snd_soc_dai *codec_dai, u32 fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	s32 iface_reg_1 = 0;
	s32 iface_reg_2 = 0;
	s32 iface_reg_3 = 0;

	int right_slot = 1;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		iface_reg_1 |= TAC5X1X_PASI_MODE_MASK;
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	default:
		dev_err(component->dev,
			"%s: invalid DAI master/slave interface\n",
			__func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface_reg_2 |= TAC5X1X_PASI_FMT_I2S;
		right_slot = 16;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg_2 |= TAC5X1X_PASI_FMT_TDM;
		iface_reg_3 |= BIT(0); /* add offset 1 */
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface_reg_2 |= TAC5X1X_PASI_FMT_TDM;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg_2 |= TAC5X1X_PASI_FMT_LJ;
		right_slot = 16;
		break;
	default:
		dev_err(component->dev,
			"%s: invalid DAI interface format\n", __func__);
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, TAC5X1X_CNTCLK2,
				      TAC5X1X_PASI_MODE_MASK, iface_reg_1);
	snd_soc_component_update_bits(component, TAC5X1X_PASI0,
				      TAC5X1X_PASI_FMT_MASK, iface_reg_2);
	snd_soc_component_update_bits(component, TAC5X1X_PASITX1,
				      TAC5X1X_PASITX_OFFSET_MASK, iface_reg_3);
	snd_soc_component_update_bits(component, TAC5X1X_PASIRX0,
				      TAC5X1X_PASIRX_OFFSET_MASK, iface_reg_3);
	snd_soc_component_update_bits(component, TAC5X1X_PASIRXCH2,
				      TAC5X1X_PASIRX_OFFSET_MASK, right_slot);
	snd_soc_component_update_bits(component, TAC5X1X_PASITXCH2,
				      TAC5X1X_PASITX_OFFSET_MASK, right_slot);

	return 0;
}

static s32 tac5x1x_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);
	s32 sample_rate, word_length = 0;

	switch (params_rate(params)) {
	case 24000:
		sample_rate = 25;
		break;
	case 32000:
		sample_rate = 23;
		break;
	case 44100:
	case 48000:
		sample_rate = 20;
		break;
	case 64000:
		sample_rate = 18;
		break;
	case 96000:
		sample_rate = 15;
		break;
	case 192000:
		sample_rate = 10;
		break;
	default:
		/* Auto detect sample rate */
		sample_rate = 0;
		break;
	}

	switch (params_physical_width(params)) {
	case 16:
		word_length |= TAC5X1X_WORD_LEN_16BITS;
		break;
	case 20:
		word_length |= TAC5X1X_WORD_LEN_20BITS;
		break;
	case 24:
		word_length |= TAC5X1X_WORD_LEN_24BITS;
		break;
	case 32:
		word_length |= TAC5X1X_WORD_LEN_32BITS;
		break;
	default:
		dev_err(tac5x1x->dev, "%s, set word length failed\n",
			__func__);
		return -EINVAL;
	}

	/* program PASI sample rate so the DAC interpolator clocks correctly */
	snd_soc_component_update_bits(component, TAC5X1X_CLK0,
				      TAC5X1X_PASI_SAMP_RATE_MASK,
				      sample_rate << 2);
	snd_soc_component_update_bits(component, TAC5X1X_PASI0,
				      TAC5X1X_PASI_DATALEN_MASK, word_length);

	/*
	 * The DAC channel and its analog power are enabled by tac5x1x_dac_event
	 * (SND_SOC_DAPM_DAC_E "Playback") once the playback stream powers up the
	 * DAC widget, which also powers OUT1 so a DAPM-routed speaker amplifier
	 * can follow the stream. No in-band channel poke is needed here.
	 */
	tac5x1x_pwr_ctrl(component, true);
	return 0;
}

static s32 tac5x1x_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	s32 ret = 0;
	struct tac5x1x_priv *tac5x1x =
			snd_soc_component_get_drvdata(component);

	switch (level) {
	case SND_SOC_BIAS_ON:
		ret = tac5x1x_pwr_ctrl(component, true);
		if (ret < 0)
			dev_err(tac5x1x->dev,
				"%s, power up failed!\n", __func__);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		ret = tac5x1x_pwr_ctrl(component, false);
		if (ret < 0)
			dev_err(tac5x1x->dev,
				"%s, power down failed!\n", __func__);
		break;
	}

	return ret;
}

static int tac5x1x_set_sysclk(struct snd_soc_dai *codec_dai, int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct tac5x1x_priv *tac5x1x = snd_soc_component_get_drvdata(component);
	s32 clk_sel = TAC5X1X_INTF0_CCLK_SEL_NONE;
	s32 clk_src = TAC5X1X_CLK2_SRC_PASI_BCLK;
	int ret;

	dev_dbg(component->dev, "%s: clk_id=%d freq=%u dir=%d\n",
		__func__, clk_id, freq, dir);

	if (dir == SND_SOC_CLOCK_IN) {
		if (tac5x1x->gpio_setup->gpio_func[0] == TAC5X1X_GPIO_CCLK_IN) {
			clk_sel = TAC5X1X_INTF0_CCLK_SEL_GPIO1;
			clk_src = TAC5X1X_CLK2_SRC_PASI_CCLK;
			dev_info(component->dev, "cclk from gpio1\n");
		} else if (tac5x1x->gpio_setup->gpio_func[1] == TAC5X1X_GPIO_CCLK_IN) {
			clk_sel = TAC5X1X_INTF0_CCLK_SEL_GPIO2;
			clk_src = TAC5X1X_CLK2_SRC_PASI_CCLK;
			dev_info(component->dev, "cclk from gpio2\n");
		} else if (tac5x1x->gpio_setup->gpi1_func == TAC5X1X_GPI_CCLK_IN) {
			clk_sel = TAC5X1X_INTF0_CCLK_SEL_GPI1;
			clk_src = TAC5X1X_CLK2_SRC_PASI_CCLK;
			dev_info(component->dev, "cclk from gpi1\n");
		} else {
			dev_info(component->dev, "recover cclk from bclk\n");
		}
	} else {
		dev_err(component->dev, "sysclk output not implemented\n");
		return -EINVAL;
	}

	/* select cclk input pin */
	ret = snd_soc_component_update_bits(component, TAC5X1X_INTF0, TAC5X1X_INTF0_CCLK_SEL_MASK, clk_sel);
	if (ret < 0)
		return ret;

	/* select pll clock source */
	ret = snd_soc_component_update_bits(component, TAC5X1X_CLK2, TAC5X1X_CLK2_SRC_SEL_MASK, clk_src);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct snd_soc_dai_ops tac5x1x_ops = {
	.hw_params = tac5x1x_hw_params,
	.set_fmt = tac5x1x_set_dai_fmt,
	.no_capture_mute = 1,
	.set_sysclk = tac5x1x_set_sysclk,
};

static struct snd_soc_dai_driver tac5x1x_dai = {
	.name = "tac5x1x-hifi",
	.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TAC5X1X_RATES,
			.formats = TAC5X1X_FORMATS,},
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TAC5X1X_RATES,
			.formats = TAC5X1X_FORMATS,
			},
	.ops = &tac5x1x_ops,
	.symmetric_rate = 1,
};

static struct snd_soc_dai_driver taa5x1x_dai = {
	.name = "taa5x1x-hifi",
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TAC5X1X_RATES,
			.formats = TAC5X1X_FORMATS,
			},
	.ops = &tac5x1x_ops,
	.symmetric_rate = 1,
};

static struct snd_soc_dai_driver tad5x1x_dai = {
	.name = "tad5x1x-hifi",
	.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TAC5X1X_RATES,
			.formats = TAC5X1X_FORMATS,
			},
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TAC5X1X_RATES,
			.formats = TAC5X1X_FORMATS,
			},
	.ops = &tac5x1x_ops,
	.symmetric_rate = 1,
};

static s32 find_first_s32_in_s32arr(const s32 *arr, s32 val, size_t size)
{
	int i;
	for (i = 0; i < size; i++) {
		if (arr[i] == val) {
			return i;
		}
	}

	return -1;
}

static void tac5x1x_setup_gpios(struct snd_soc_component *component)
{
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);
	s32 *gpio_drive = tac5x1x->gpio_setup->gpio_drive;
	s32 *gpio_func = tac5x1x->gpio_setup->gpio_func;
	s32 *pdm_input_pins = tac5x1x->gpio_setup->pdm_input_pins;

	/* setup GPIO functions */
	/* GPIO1 */
	if (gpio_func[0] <= TAC5X1X_GPIO_DAISY_OUT) {
		snd_soc_component_update_bits(component, TAC5X1X_GPIO1,
					      TAC5X1X_GPIOX_CFG_MASK,
					      gpio_func[0] << 4);
		snd_soc_component_update_bits(component, TAC5X1X_GPIO1,
					      TAC5X1X_GPIOX_DRV_MASK,
					      gpio_drive[0]);
	}
	/* GPIO2 */
	if (gpio_func[1] <= TAC5X1X_GPIO_DAISY_OUT) {
		snd_soc_component_update_bits(component, TAC5X1X_GPIO2,
					      TAC5X1X_GPIOX_CFG_MASK,
					      gpio_func[1] << 4);
		snd_soc_component_update_bits(component, TAC5X1X_GPIO2,
					      TAC5X1X_GPIOX_DRV_MASK,
					      gpio_drive[1]);

	}
	/* GPO1 */
	if (gpio_func[2] <= TAC5X1X_GPIO_DAISY_OUT) {
		snd_soc_component_update_bits(component, TAC5X1X_GPO1,
					      TAC5X1X_GPIOX_CFG_MASK,
					      gpio_func[2] << 4);
		snd_soc_component_update_bits(component, TAC5X1X_GPO1,
					      TAC5X1X_GPIOX_DRV_MASK,
					      gpio_drive[2]);
	}

	/* GPI1 */
	if (tac5x1x->gpio_setup->gpi1_func != TAC5X1X_GPI_DISABLE)
		snd_soc_component_update_bits(component, TAC5X1X_GPI1,
					      TAC5X1X_GPI1_CFG_MASK,
					      TAC5X1X_GPI1_CFG_MASK);
	/*GPA GPIO*/
	if (tac5x1x->gpio_setup->gpa_gpio)
		snd_soc_component_update_bits(component, TAC5X1X_INTF5,
					      TAC5X1X_GPA_CFG_MASK,
					      TAC5X1X_GPA_CFG_MASK);

	/* PDM ch1 & ch2 Datain */
	if (pdm_input_pins[0])
		snd_soc_component_update_bits(component, TAC5X1X_INTF4, 0x0c,
					      pdm_input_pins[0] << 2);
	/* PDM ch3 & ch4 Datain */
	if (pdm_input_pins[1])
		snd_soc_component_update_bits(component, TAC5X1X_INTF4, 0x03,
					      pdm_input_pins[1]);
}

static const struct reg_sequence tac5x1x_init_reg_seq[] = {
	/* ADC Channels input coupling configuration */
	REG_SEQ0(TAC5X1X_ADCCH1C0, 0x04),
	REG_SEQ0(TAC5X1X_ADCCH2C0, 0x04),
	/* Disable inputs and outputs */
	REG_SEQ0(TAC5X1X_CH_EN, 0x00),
	REG_SEQ0(TAC5X1X_PASITXCH1, 0x00),
	REG_SEQ0(TAC5X1X_PASITXCH2, 0x01),
	REG_SEQ0(TAC5X1X_PASIRXCH1, 0x00),
	REG_SEQ0(TAC5X1X_PASIRXCH2, 0x01),
	/* misc writes */
	REG_SEQ0(TAC5X1X_TP_DREG, 0xD),
	REG_SEQ0(TAC5X1X_TP_AREG, 0x08),
	REG_SEQ0(TAC5X1X_TP_DREG, 0x0),
	REG_SEQ0(TAC5X1X_DYN_PUPD, 0xa0),
	/* clear latch irrespctive of live status */
	REG_SEQ0(TAC5X1X_INT, 0x11),
};

static s32 tac5x1x_init(struct tac5x1x_priv *tac5x1x)
{
	return regmap_multi_reg_write(tac5x1x->regmap, tac5x1x_init_reg_seq,
				      ARRAY_SIZE(tac5x1x_init_reg_seq));
}

static s32 tac5x1x_reset(struct tac5x1x_priv *tac5x1x)
{
	s32 ret;

	ret = regmap_write(tac5x1x->regmap, TAC5X1X_RESET, 1);
	if (ret < 0)
		return ret;
	/* Wait >= 10 ms after entering sleep mode. */
	usleep_range(10000, 100000);
	regcache_mark_dirty(tac5x1x->regmap);

	return ret;
}

static s32 tac5x1x_add_controls(struct snd_soc_component *component)
{
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);
	s32 *gpio_func = tac5x1x->gpio_setup->gpio_func;
	s32 ret;

	switch (tac5x1x->codec_type) {
	case TAA5212:
		fallthrough;
	case TAA5412:
		ret =
		 snd_soc_add_component_controls(component, tolerance_ctrls,
						ARRAY_SIZE(tolerance_ctrls));
		if (ret)
			return ret;
		ret =
		 snd_soc_add_component_controls(component,
						taa_ip_controls,
						ARRAY_SIZE(taa_ip_controls));
		if (ret)
			return ret;
		break;
	/* For Mono */
	case TAC5111:
	case TAC5211:
		ret =
		 snd_soc_add_component_controls(component,
						tac5x11_controls,
						ARRAY_SIZE(tac5x11_controls));
		if (ret)
			return ret;
	fallthrough;
	case TAC5311:
	case TAC5411:
		ret =
		 snd_soc_add_component_controls(component, tad5x1x_controls,
						ARRAY_SIZE(tad5x1x_controls));
		if (ret)
			return ret;
		break;
	/* For Stereo */
	case TAC5112:
	case TAC5212:
		fallthrough;
	case TAC5312:
	case TAC5412:
		ret =
		 snd_soc_add_component_controls(component, tolerance_ctrls,
						ARRAY_SIZE(tolerance_ctrls));
		if (ret)
			return ret;
		ret =
		 snd_soc_add_component_controls(component, tad5x1x_controls,
						ARRAY_SIZE(tad5x1x_controls));
		if (ret)
			return ret;

		ret =
		 snd_soc_add_component_controls(component, taa5x12_controls,
						ARRAY_SIZE(taa5x12_controls));
		if (ret)
			return ret;

		ret =
		 snd_soc_add_component_controls(component, tad5x12_controls,
						ARRAY_SIZE(tad5x12_controls));
		if (ret)
			return ret;
		break;
	case TAD5212:
	case TAD5112:
		ret = snd_soc_add_component_controls(component, tad5x12_controls,
						     ARRAY_SIZE(tad5x12_controls));
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	ret = find_first_s32_in_s32arr(gpio_func, TAC5X1X_GPIO_PDMCLK,
					ARRAY_SIZE(tac5x1x->gpio_setup->gpio_func));
	/* If enabled PDM GPIO*/
	if (ret >= 0) {
		ret = snd_soc_add_component_controls(component, tac5x1x_pdm_controls,
						     ARRAY_SIZE(tac5x1x_pdm_controls));
		if (ret)
			return ret;
	}

	return 0;
}

static s32 tac5x1x_add_ip_diag_controls(struct snd_soc_component *component)
{
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);
	s32 ret = 0;

	switch (tac5x1x->codec_type) {
	case TAA5212:
		break;
	case TAA5412:
	case TAC5311:
	case TAC5312:
	case TAC5411:
	case TAC5412:
		if (tac5x1x->input_diag_config.in_ch_en) {
			ret =
			 snd_soc_add_component_controls(component,
							taa_ip_controls,
							ARRAY_SIZE(taa_ip_controls));
			if (ret)
				return ret;
			snd_soc_component_update_bits(component,
						      TAC5X1X_DIAG_CFG0,
						      TAC5X1X_IN_CH_DIAG_EN_MASK,
						      TAC5X1X_IN_CH_DIAG_EN_MASK);
		}
		if (tac5x1x->input_diag_config.out_ch_en) {
			snd_soc_component_update_bits(component, TAC5X1X_DIAG_CFG0,
						      TAC5X1X_OUT1P_DIAG_EN_MASK,
						      TAC5X1X_OUT1P_DIAG_EN_MASK);
		}
		if (tac5x1x->input_diag_config.incl_se_inm) {
			snd_soc_component_update_bits(component, TAC5X1X_DIAG_CFG0,
						      TAC5X1X_INCL_SE_INM_MASK,
						      TAC5X1X_INCL_SE_INM_MASK);
		}
		if (tac5x1x->input_diag_config.incl_ac_coup) {
			snd_soc_component_update_bits(component, TAC5X1X_DIAG_CFG0,
						      TAC5X1X_INCL_AC_COUP_MASK,
						      TAC5X1X_INCL_AC_COUP_MASK);
		}
		snd_soc_component_update_bits(component, TAC5X1X_DIAG_CFG7,
					      0xff,
					      tac5x1x->micbias_threshold[0]);
		snd_soc_component_update_bits(component, TAC5X1X_DIAG_CFG6,
					      0xff,
					      tac5x1x->micbias_threshold[1]);
		fallthrough;
	case TAC5111:
		ret = tac5x1x_register_interrupt(tac5x1x);
		if (ret)
			return ret;
		fallthrough;
	case TAC5112:
	case TAC5211:
	case TAC5212:
	case TAD5112:
	case TAD5212:
		snd_soc_component_update_bits(component, TAC5X1X_DIAG_CFG9,
					      0xff, tac5x1x->gpa_threshold[0]);
		snd_soc_component_update_bits(component, TAC5X1X_DIAG_CFG8,
					      0xff, tac5x1x->gpa_threshold[1]);
		break;
	default:
		break;
	}

	return ret;
}

static s32 tac5x1x_add_widgets(struct snd_soc_component *component)
{
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_get_dapm(component);
	s32 *gpio_func = tac5x1x->gpio_setup->gpio_func;
	s32 ret;

	switch (tac5x1x->codec_type) {
	case TAC5111:
	case TAC5211:
	case TAC5311:
	case TAC5411:
		ret =
		 snd_soc_dapm_new_controls(dapm, tad5x1x_dapm_widgets,
					   ARRAY_SIZE(tad5x1x_dapm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(dapm, tad5x1x_dapm_routes,
					      ARRAY_SIZE(tad5x1x_dapm_routes));
		if (ret)
			return ret;
		break;
	case TAC5112:
	case TAC5212:
	case TAC5312:
	case TAC5412:
		ret =
		 snd_soc_dapm_new_controls(dapm, tad5x1x_dapm_widgets,
					   ARRAY_SIZE(tad5x1x_dapm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(dapm, tad5x1x_dapm_routes,
					      ARRAY_SIZE(tad5x1x_dapm_routes));
		if (ret)
			return ret;
		ret =
		 snd_soc_dapm_new_controls(dapm, tad5x12_dapm_widgets,
					   ARRAY_SIZE(tad5x12_dapm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(dapm, tad5x12_dapm_routes,
					      ARRAY_SIZE(tad5x12_dapm_routes));
		if (ret)
			return ret;
		fallthrough;
	case TAA5212:
	case TAA5412:
		ret =
		 snd_soc_dapm_new_controls(dapm, taa5x12_dapm_widgets,
					   ARRAY_SIZE(taa5x12_dapm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(dapm, taa5x12_dapm_routes,
					      ARRAY_SIZE(taa5x12_dapm_routes));
		if (ret)
			return ret;
		break;
	case TAD5212:
	case TAD5112:
		ret =
		 snd_soc_dapm_new_controls(dapm, tad5xx_dapm_widgets,
					   ARRAY_SIZE(tad5xx_dapm_widgets));
		if (ret)
			return ret;

		ret =
		 snd_soc_dapm_new_controls(dapm, tad5x12_dapm_widgets,
					   ARRAY_SIZE(tad5x12_dapm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(dapm, tad5x12_dapm_routes,
					      ARRAY_SIZE(tad5x12_dapm_routes));
		if (ret)
			return ret;

		break;
	default:
		break;
	}

	switch (tac5x1x->codec_type) {
	case TAD5212:
	case TAD5112:
		ret =
		 snd_soc_dapm_new_controls(dapm, tad_common_widgets,
					   ARRAY_SIZE(tad_common_widgets));
		if (ret)
			return ret;
		break;
	default:
		ret =
		 snd_soc_dapm_new_controls(dapm, tac5x1x_common_widgets,
					   ARRAY_SIZE(tac5x1x_common_widgets));
		if (ret)
			return ret;

		ret = snd_soc_dapm_add_routes(dapm, tac_common_routes,
					      ARRAY_SIZE(tac_common_routes));
		if (ret)
			return ret;
		break;
	}

	ret = find_first_s32_in_s32arr(gpio_func, TAC5X1X_GPIO_PDMCLK,
					ARRAY_SIZE(tac5x1x->gpio_setup->gpio_func));
	/* If enabled PDM GPIO*/
	if (ret >= 0) {
		ret =
		 snd_soc_dapm_new_controls(dapm, tac5x1x_pdm_widgets,
					   ARRAY_SIZE(tac5x1x_pdm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(dapm, tac5x1x_pdm_routes,
					      ARRAY_SIZE(tac5x1x_pdm_routes));
		if (ret)
			return ret;
	}
	return 0;
}

static int tac5x1x_setup_adc_impedance(struct device *dev,
				       struct tac5x1x_priv *tac5x1x)
{
	if (tac5x1x->adc_impedance[0] != -1)
		snd_soc_component_update_bits(tac5x1x->component,
					      TAC5X1X_ADCCH1C0,
					      TAC5X1X_ADCCH1C0_IMPEDANCE_MASK,
					      tac5x1x->adc_impedance[0] << 4);

	if (tac5x1x->adc_impedance[1] != -1)
		snd_soc_component_update_bits(tac5x1x->component,
					      TAC5X1X_ADCCH2C0,
					      TAC5X1X_ADCCH2C0_IMPEDANCE_MASK,
					      tac5x1x->adc_impedance[1] << 4);

	return 0;
}

static s32 tac5x1x_component_probe(struct snd_soc_component *component)
{
	s32 ret;
	struct tac5x1x_priv *tac5x1x =
				snd_soc_component_get_drvdata(component);

	tac5x1x->component = component;
	ret = tac5x1x_add_controls(component);
	if (ret < 0) {
		dev_err(tac5x1x->dev,
			"%s, add control failed\n", __func__);
		return ret;
	}

	ret = tac5x1x_add_widgets(component);
	if (ret < 0) {
		dev_err(tac5x1x->dev,
			"%s, device widget addition failed\n", __func__);
		return ret;
	}

	tac5x1x_setup_adc_impedance(tac5x1x->dev, tac5x1x);

	/* update if vcom property is found */
	if (tac5x1x->out2x_vcom_cfg != -1)
		snd_soc_component_update_bits(component, TAC5X1X_OUT2CFG0,
					      TAC5X1X_OUT2CFG0_VCOM_MASK,
					      tac5x1x->out2x_vcom_cfg);

	if (tac5x1x->gpio_setup)
		tac5x1x_setup_gpios(component);

	ret = tac5x1x_add_ip_diag_controls(component);
	if (ret < 0) {
		dev_err(tac5x1x->dev,
			"%s add diag control failed\n", __func__);
		return ret;
	}
	return ret;
}

static void tac5x1x_disable_regulators(struct tac5x1x_priv *tac5x1x)
{
	regulator_bulk_disable(tac5x1x_num_regulators, tac5x1x_regulators);
}

#ifdef CONFIG_PM
static s32 tac5x1x_soc_suspend(struct snd_soc_component *component)
{
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);

	regcache_cache_only(tac5x1x->regmap, true);
	regcache_mark_dirty(tac5x1x->regmap);

	tac5x1x_disable_regulators(tac5x1x);

	return 0;
}

static s32 tac5x1x_soc_resume(struct snd_soc_component *component)
{
	struct tac5x1x_priv *tac5x1x =
		snd_soc_component_get_drvdata(component);
	s32 ret;

	ret = regulator_bulk_enable(tac5x1x_num_regulators,
				    tac5x1x_regulators);
	if (ret) {
		dev_err(tac5x1x->dev, "Failed to enable regulators\n");
		return ret;
	}

	regcache_cache_only(tac5x1x->regmap, false);
	snd_soc_component_cache_sync(component);

	return ret;
}
#else
#define tac5x1x_soc_suspend	NULL
#define tac5x1x_soc_resume	NULL
#endif /* CONFIG_PM */

static const struct snd_soc_component_driver component_tac5x1x = {
	.probe			= tac5x1x_component_probe,
	.set_bias_level		= tac5x1x_set_bias_level,
	.suspend		= tac5x1x_soc_suspend,
	.resume			= tac5x1x_soc_resume,
	.controls		= taa5x1x_controls,
	.num_controls		= ARRAY_SIZE(taa5x1x_controls),
	.dapm_widgets		= taa5x1x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(taa5x1x_dapm_widgets),
	.dapm_routes		= taa5x1x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(taa5x1x_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct snd_soc_component_driver component_taa5x1x = {
	.probe			= tac5x1x_component_probe,
	.set_bias_level		= tac5x1x_set_bias_level,
	.suspend		= tac5x1x_soc_suspend,
	.resume			= tac5x1x_soc_resume,
	.controls		= taa5x1x_controls,
	.num_controls		= ARRAY_SIZE(taa5x1x_controls),
	.dapm_widgets		= taa5x1x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(taa5x1x_dapm_widgets),
	.dapm_routes		= taa5x1x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(taa5x1x_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct snd_soc_component_driver component_tad5x1x = {
	.probe			= tac5x1x_component_probe,
	.set_bias_level		= tac5x1x_set_bias_level,
	.suspend		= tac5x1x_soc_suspend,
	.resume			= tac5x1x_soc_resume,
	.controls		= tad5x1x_controls,
	.num_controls		= ARRAY_SIZE(tad5x1x_controls),
	.dapm_widgets		= tad5x1x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tad5x1x_dapm_widgets),
	.dapm_routes		= tad5x1x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tad5x1x_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static s32 tac5x1x_parse_dt(struct tac5x1x_priv *tac5x1x,
			    struct device_node *np)
{
	struct tac5x1x_input_diag_config input_config = {0};
	struct tac5x1x_setup_gpio *tac5x1x_setup;
	s32 micbias_value = TAC5X1X_MICBIAS_VREF;
	s32 vref_value = TAC5X1X_VERF_2_5V;
	s32 ret;

	tac5x1x_setup = devm_kzalloc(tac5x1x->dev, sizeof(*tac5x1x_setup),
				     GFP_KERNEL);
	if (!tac5x1x_setup)
		return -ENOMEM;

	ret = fwnode_property_read_u32(tac5x1x->dev->fwnode, "ti,vref",
				       &vref_value);
	if (ret) {
		dev_err(tac5x1x->dev, "Fail to get verf E:%d\n", ret);
		goto out;
	}
	ret = fwnode_property_read_u32(tac5x1x->dev->fwnode,
				       "ti,micbias-vg", &micbias_value);
	if (ret) {
		dev_err(tac5x1x->dev, "Fail to get micbias-vg E:%d\n", ret);
		goto out;
	}

	if (micbias_value == TAC5X1X_MICBIAS_AVDD) {
		tac5x1x->micbias_vg = micbias_value;
		tac5x1x->vref_vg = TAC5X1X_VERF_2_75V;
		tac5x1x->micbias_en = true;
	} else {
		switch (vref_value) {
		case TAC5X1X_VERF_2_75V:
		case TAC5X1X_VERF_2_5V:
			switch (micbias_value) {
			case TAC5X1X_MICBIAS_VREF:
			case TAC5X1X_MICBIAS_0_5VREF:
				tac5x1x->micbias_vg = micbias_value;
				break;
			default:
				dev_err(tac5x1x->dev,
					"Bad tac5x1x-micbias-vg value %d\n",
					micbias_value);
				tac5x1x->micbias_vg = TAC5X1X_MICBIAS_AVDD;
				break;
			}
			tac5x1x->vref_vg = vref_value;
			tac5x1x->micbias_en = true;
			break;
		case TAC5X1X_VERF_1_375V:
			if (micbias_value == TAC5X1X_MICBIAS_VREF) {
				tac5x1x->micbias_vg = micbias_value;
			} else {
				dev_err(tac5x1x->dev,
					"Bad tac5x1x-micbias-vg value %d\n",
					micbias_value);
				tac5x1x->micbias_vg = TAC5X1X_MICBIAS_AVDD;
			}
			tac5x1x->micbias_en = true;
			tac5x1x->vref_vg = vref_value;
			break;
		default:
			dev_err(tac5x1x->dev,
				"Bad tac5x1x-vref-vg value %d\n", vref_value);
			tac5x1x->vref_vg = TAC5X1X_VERF_2_5V;
			tac5x1x->micbias_vg = TAC5X1X_MICBIAS_AVDD;
			tac5x1x->micbias_en = true;
			break;
		}
	}

	if (fwnode_property_read_u32(tac5x1x->dev->fwnode, "ti,gpi1-func",
				     &tac5x1x_setup->gpi1_func))
		dev_dbg(tac5x1x->dev, "Fail to get gpi1-func value\n");

	if (fwnode_property_read_u32(tac5x1x->dev->fwnode, "ti,gpa-gpio",
				     &tac5x1x_setup->gpa_gpio))
		dev_dbg(tac5x1x->dev, "Fail to get gpa-gpio value\n");

	if (fwnode_property_read_u32_array(tac5x1x->dev->fwnode,
					   "ti,gpios-func",
					   tac5x1x_setup->gpio_func, 3))
		dev_dbg(tac5x1x->dev, "Fail to get gpios-func value\n");

	fwnode_property_read_u32_array(tac5x1x->dev->fwnode,
				       "ti,pdm-input-pins",
				       tac5x1x_setup->pdm_input_pins, 2);

	if (fwnode_property_read_u32_array(tac5x1x->dev->fwnode,
					   "ti,gpios-drive",
					   tac5x1x_setup->gpio_drive, 3))
		dev_dbg(tac5x1x->dev, "Fail to get gpios-drive value\n");

	tac5x1x->gpa_threshold[0] = TAC5X1X_GPA_LOW_THRESHOLD;
	tac5x1x->gpa_threshold[1] = TAC5X1X_GPA_HIGH_THRESHOLD;
	if (fwnode_property_read_u32_array(tac5x1x->dev->fwnode,
					   "ti,gpa-threshold",
					   tac5x1x->gpa_threshold, 2))
		dev_dbg(tac5x1x->dev, "Fail to get ti,gpa-threshold value\n");

	tac5x1x->gpio_setup = tac5x1x_setup;
	tac5x1x->adc_impedance[0] = -1;
	tac5x1x->adc_impedance[1] = -1;
	tac5x1x->out2x_vcom_cfg = -1;

	fwnode_property_read_u32(tac5x1x->dev->fwnode, "ti,out2x-vcom-cfg",
				 &tac5x1x->out2x_vcom_cfg);

	switch (tac5x1x->codec_type) {
	case TAA5212:
	case TAC5212:
		if (fwnode_property_read_u32(tac5x1x->dev->fwnode,
					     "ti,adc2-impedance",
					     &tac5x1x->adc_impedance[1]))
			dev_dbg(tac5x1x->dev,
				"Fail to get ti,adc2-impedance value\n");
		fallthrough;
	case TAC5211:
	case TAC5111:
		if (fwnode_property_read_u32(tac5x1x->dev->fwnode,
					     "ti,adc1-impedance",
					     &tac5x1x->adc_impedance[0]))
			dev_dbg(tac5x1x->dev,
				"Fail to get ti,adc1-impedance value\n");

		fallthrough;
	case TAC5112:
	case TAD5112:
	case TAD5212:
		break;
	case TAA5412:
	case TAC5411:
	case TAC5412:
	case TAC5311:
	case TAC5312:
		tac5x1x->input_diag_config.in_ch_en = 0;
		if (fwnode_property_read_u32(tac5x1x->dev->fwnode,
					     "ti,in-ch-en",
					     &input_config.in_ch_en))
			dev_dbg(tac5x1x->dev,
				"Fail to get ti,in-ch-en value\n");
		tac5x1x->input_diag_config.out_ch_en = 0;
		if (fwnode_property_read_u32(tac5x1x->dev->fwnode,
					     "ti,out-ch-en",
					     &input_config.out_ch_en))
			dev_dbg(tac5x1x->dev,
				"Fail to get ti,out-ch-en value\n");
		tac5x1x->input_diag_config.incl_se_inm = 0;
		if (fwnode_property_read_u32(tac5x1x->dev->fwnode,
					     "ti,incl-se-inm",
					     &input_config.incl_se_inm))
			dev_dbg(tac5x1x->dev,
				"Fail to get ti,incl-se-inm value\n");
		tac5x1x->input_diag_config.incl_ac_coup = 0;
		if (fwnode_property_read_u32(tac5x1x->dev->fwnode,
					     "ti,incl-ac-coup",
					     &input_config.incl_ac_coup))
			dev_dbg(tac5x1x->dev,
				"Fail to get ti,incl-ac-coup value\n");
		tac5x1x->input_diag_config = input_config;

		tac5x1x->micbias_threshold[0] = TAC5X1X_MICBIAS_LOW_THRESHOLD;
		tac5x1x->micbias_threshold[1] = TAC5X1X_MICBIAS_HIGH_THRESHOLD;
		if (fwnode_property_read_u32_array(tac5x1x->dev->fwnode,
						   "ti,micbias-threshold",
						   tac5x1x->micbias_threshold,
						   2))
			dev_dbg(tac5x1x->dev,
				"Fail to get ti,micbias-threshold value\n");
		break;
	}
out:
	return ret;
}

static s32 tac5x1x_setup_regulators(struct device *dev,
				    struct tac5x1x_priv *tac5x1x)
{
	int ret;

	ret = devm_regulator_bulk_get(dev, tac5x1x_num_regulators,
				      tac5x1x_regulators);
	if (ret) {
		dev_err(dev, "Failed to get regulators\n");
		return ret;
	}

	ret = regulator_bulk_enable(tac5x1x_num_regulators,
				    tac5x1x_regulators);
	if (ret) {
		dev_err(dev, "Failed to enable regulators\n");
		regulator_bulk_disable(tac5x1x_num_regulators,
				       tac5x1x_regulators);
		return ret;
	}

	return 0;
}

static s32 tac5x1x_probe(struct device *dev, struct regmap *regmap,
			 enum tac5x1x_type type)
{
	struct device_node *np = dev->of_node;
	struct tac5x1x_priv *tac5x1x;
	s32 ret;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	tac5x1x = devm_kzalloc(dev, sizeof(struct tac5x1x_priv),
			       GFP_KERNEL);
	if (!tac5x1x)
		return -ENOMEM;

	tac5x1x->dev = dev;
	tac5x1x->codec_type = type;
	tac5x1x->regmap = regmap;
	mutex_init(&tac5x1x->ch_lock);
	INIT_DELAYED_WORK(&tac5x1x->powerup_work, post_powerup_work);
	dev_set_drvdata(dev, tac5x1x);

	ret = tac5x1x_reset(tac5x1x);
	if (ret) {
		dev_err(dev, "Failed to reset device\n");
		return ret;
	}
	tac5x1x_init(tac5x1x);

	ret = tac5x1x_setup_regulators(dev, tac5x1x);
	if (ret) {
		dev_err(dev, "Failed to setup regulators\n");
		return ret;
	}

	if (np) {
		ret = tac5x1x_parse_dt(tac5x1x, np);
		if (ret) {
			dev_err(dev, "Failed to parse DT node\n");
			goto err_disable_regulators;
		}
	} else {
		dev_err(dev, "%s: Fail to get device node\n", __func__);
		goto err_disable_regulators;
	}

	switch (tac5x1x->codec_type) {
	case TAA5212:
	case TAA5412:
		ret = devm_snd_soc_register_component(dev, &component_taa5x1x,
						      &taa5x1x_dai, 1);
		if (ret) {
			dev_err(dev, "Failed to register component\n");
			goto err_disable_regulators;
		}
		break;
	case TAC5111:
	case TAC5112:
	case TAC5211:
	case TAC5212:
	case TAC5311:
	case TAC5312:
	case TAC5411:
	case TAC5412:
		ret = devm_snd_soc_register_component(dev, &component_tac5x1x,
						      &tac5x1x_dai, 1);
		if (ret) {
			dev_err(dev, "Failed to register component\n");
			goto err_disable_regulators;
		}
		break;
	case TAD5112:
	case TAD5212:
		ret = devm_snd_soc_register_component(dev, &component_tad5x1x,
						      &tad5x1x_dai, 1);
		if (ret) {
			dev_err(dev, "Failed to register component\n");
			goto err_disable_regulators;
		}
		break;
	}
	return 0;

err_disable_regulators:
	tac5x1x_disable_regulators(tac5x1x);

	return ret;
}

static s32 tac5x1x_remove(struct device *dev)
{
	struct tac5x1x_priv *tac5x1x = dev_get_drvdata(dev);

	tac5x1x_disable_regulators(tac5x1x);
	return 0;
}

static const struct of_device_id tac5x1x_of_match[] = {
	{ .compatible = "ti,taa5212", .data = (void *)TAA5212 },
	{ .compatible = "ti,taa5412", .data = (void *)TAA5412 },
	{ .compatible = "ti,tac5111", .data = (void *)TAC5111 },
	{ .compatible = "ti,tac5112", .data = (void *)TAC5112 },
	{ .compatible = "ti,tac5211", .data = (void *)TAC5211 },
	{ .compatible = "ti,tac5212", .data = (void *)TAC5212 },
	{ .compatible = "ti,tac5311", .data = (void *)TAC5311 },
	{ .compatible = "ti,tac5312", .data = (void *)TAC5312 },
	{ .compatible = "ti,tac5411", .data = (void *)TAC5411 },
	{ .compatible = "ti,tac5412", .data = (void *)TAC5412 },
	{ .compatible = "ti,tad5112", .data = (void *)TAD5112 },
	{ .compatible = "ti,tad5212", .data = (void *)TAD5212 },
	{}
};
MODULE_DEVICE_TABLE(of, tac5x1x_of_match);

static const struct i2c_device_id tac5x1x_id[] = {
	{"taa5212", TAA5212},
	{"taa5412", TAA5412},
	{"tac5111", TAC5111},
	{"tac5112", TAC5112},
	{"tac5211", TAC5211},
	{"tac5212", TAC5212},
	{"tac5311", TAC5311},
	{"tac5312", TAC5312},
	{"tac5411", TAC5411},
	{"tac5412", TAC5412},
	{"tad5112", TAD5112},
	{"tad5212", TAD5212},
	{}
};
MODULE_DEVICE_TABLE(i2c, tac5x1x_id);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
static int tac5x1x_i2c_probe(struct i2c_client *i2c)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(i2c);
#else
static int tac5x1x_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
#endif
	int ret;
	enum tac5x1x_type type;
	struct regmap *regmap;
	const struct regmap_config *config = &tac5x1x_regmap;

	regmap = devm_regmap_init_i2c(i2c, config);
	type = (uintptr_t)id->driver_data;

	ret = tac5x1x_probe(&i2c->dev, regmap, type);
	if (ret)
		dev_err(&i2c->dev, "probe failed");

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void tac5x1x_i2c_remove(struct i2c_client *client)
#else
static int tac5x1x_i2c_remove(struct i2c_client *client)
#endif
{
	tac5x1x_remove(&client->dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
	return 0;
#endif
}

static struct i2c_driver tac5x1x_i2c_driver = {
	.driver = {
		.name = "tac5x1x-codec",
		.of_match_table = of_match_ptr(tac5x1x_of_match),
	},
	.probe = tac5x1x_i2c_probe,
	.remove = tac5x1x_i2c_remove,
	.id_table = tac5x1x_id,
};
module_i2c_driver(tac5x1x_i2c_driver);

MODULE_DESCRIPTION("ASoC tac5x1x codec driver");
MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
