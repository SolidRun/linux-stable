// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Renesas Versaclock 3
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define NUM_CONFIG_REGISTERS	37

/* VC3 Input mux settings */
#define VC3_X1		BIT(0)
#define VC3_CLKINB	BIT(1)

#define VC3_GENERAL_CTR		0x0
#define VC3_DIV1_SRC_SEL	BIT(3)
#define VC3_PLL3_REFIN_SEL	BIT(2)

#define VC3_PLL3_M_DIVIDER	0x3
#define VC3_PLL3_M_DIV1		BIT(7)
#define VC3_PLL3_M_DIV2		BIT(6)
#define VC3_PLL3_M_DIV(n)	((n) & 0x3f)

#define VC3_PLL3_N_DIV_LSB	0x04
#define VC3_PLL3_LOOP_FILTER_N_DIVIDER	0x05

#define VC3_PLL3_CHARGE_PUMP_CTRL	0x6
#define VC3_DIV3_SRC_SEL		BIT(7)

#define VC3_PLL1_CTRL_OUTDIV5	0x7
#define VC3_PLL1_MDIV_DOUBLER	BIT(7)

#define VC3_PLL1_M_DIVIDER	0x8
#define VC3_PLL1_M_DIV1		BIT(7)
#define VC3_PLL1_M_DIV2		BIT(6)
#define VC3_PLL1_M_DIV(n)	((n) & 0x3f)

#define VC3_PLL1_VCO_N_DIVIDER	0x9
#define VC3_PLL1_LOOP_FILTER_N_DIVIDER	0x0A

#define VC3_DIV1_DIV2_CTRL	0xF

#define VC3_PLL2_FB_INT_DIV_MSB	0x10
#define VC3_PLL2_FB_INT_DIV_LSB	0x11
#define VC3_PLL2_FB_FRC_DIV_MSB	0x12
#define VC3_PLL2_FB_FRC_DIV_LSB	0x13

#define VC3_PLL2_M_DIVIDER	0x1A
#define VC3_PLL2_MDIV_DOUBLER	BIT(7)
#define VC3_PLL2_M_DIV1		BIT(6)
#define VC3_PLL2_M_DIV2		BIT(5)
#define VC3_PLL2_M_DIV(n)	((n) & 0x1f)

#define VC3_DIV3_DIV4_CTRL	0x1B

#define VC3_PLL_OP_CTRL		0x1C
#define VC3_PLL2_REFIN_SEL	BIT(6)

#define VC3_OUTPUT_CTR		0x1D
#define VC3_DIFF1_OUT_EN	BIT(5)
#define VC3_DIFF2_OUT_EN	BIT(4)
#define VC3_DIV4_SRC_SEL	BIT(3)

#define VC3_OE_DFC_CTRL		0x1E
#define VC3_SE1_EN		BIT(7)
#define VC3_SE3_EN		BIT(4)

#define VC3_SE2_CTRL_REG0	0x1F
#define VC3_SE2_FREERUN_32K	BIT(7)
#define VC3_SE2_CLK_SEL		BIT(6)

#define VC3_SE2_CTRL_REG1	0x20
#define VC3_SE2_EN		BIT(7)

#define VC3_SE3_CTRL_REG	0x21
#define VC3_SE3_FREERUN_32K	BIT(7)
#define VC3_SE3_CLK_SEL		BIT(6)

#define VC3_DIFF1_CTRL_REG	0x22
#define VC3_DIFF1_CLK_SEL	BIT(7)

#define VC3_DIFF2_CTRL_REG	0x23
#define VC3_DIFF2_CLK_SEL	BIT(7)

#define VC3_SE1_DIV4_CTRL	0x24
#define VC3_SE1_FREERUN_32K	BIT(4)
#define VC3_SE1_CLK_SEL		BIT(3)
#define VC3_REF_EN		BIT(2)

#define VC3_PLL1_VCO_MIN	300000000UL
#define VC3_PLL1_VCO_MAX	600000000UL

#define VC3_PLL2_VCO_MIN	400000000UL
#define VC3_PLL2_VCO_MAX	1200000000UL

#define VC3_PLL3_VCO_MIN	300000000UL
#define VC3_PLL3_VCO_MAX	800000000UL

/* Maximum number of clk_out supported by this driver */
#define VC3_MAX_CLK_OUT_NUM	6

/* Maximum number of clk mux  supported by this driver */
#define VC3_MAX_CLK_MUX_NUM	5

/* Maximum number of Dividers supported by this driver */
#define VC3_MAX_DIV_NUM		5

/* Maximum number of divider mux  supported by this driver */
#define VC3_MAX_DIV_MUX_NUM	3

/* Maximum number of pll's supported by this driver */
#define VC3_MAX_PLL_NUM		3

/* Maximum number of pfd's supported by this driver */
#define VC3_MAX_PFD_NUM		3

/* Maximum number of pfd mux supported by this driver */
#define VC3_MAX_PFD_MUX_NUM	2

#define VC3_2_POWER_16		(256 * 256)

#define VC3_DIV_MASK(width)	((1 << (width)) - 1)

struct vc3_driver_data;

struct vc3_hw_data {
	struct clk_hw hw;
	struct vc3_driver_data *vc3;
	unsigned int num;
	u32 div_int;
	u32 div_frc;
	u8 offset;
	u8 reg;
};

struct vc3_hw_div {
	struct clk_hw hw;
	struct vc3_driver_data *vc3;
	const struct clk_div_table *table;
	unsigned int num;
	u32 reg;
	u8 shift;
	u8 width;
	u8 flags;
};

struct vc3_driver_data {
	struct i2c_client *client;
	struct regmap *regmap;

	struct clk *pin_x1;
	struct clk *pin_clkinb;
	struct vc3_hw_div clk_div[VC3_MAX_DIV_NUM];
	struct vc3_hw_data clk_pfd_mux[VC3_MAX_PFD_MUX_NUM];
	struct vc3_hw_data clk_pfd[VC3_MAX_PFD_NUM];
	struct vc3_hw_data clk_pll[VC3_MAX_PLL_NUM];
	struct vc3_hw_data clk_div_mux[VC3_MAX_DIV_MUX_NUM];
	struct vc3_hw_data clk_mux[VC3_MAX_CLK_MUX_NUM];
	struct vc3_hw_data clk_out[VC3_MAX_CLK_OUT_NUM];
	unsigned char clk_in;

	bool is_32k_used;
};

static const char * const vc3_pfd_mux_names[] = {
	"pfd2_mux", "pfd3_mux",
};

static const char * const vc3_pfd_names[] = {
	"pfd1", "pfd2", "pfd3",
};

static const char * const vc3_pll_names[] = {
	"pll1", "pll2", "pll3",
};

static const char * const vc3_div_mux_names[] = {
	"div1_mux", "div3_mux", "div4_mux",
};

static const char * const vc3_div_names[] = {
	"div1", "div2", "div3", "div4", "div5",
};

static const char * const vc3_clk_mux_names[] = {
	"diff2_mux", "diff1_mux", "se3_mux", "se2_mux", "se1_mux",
};

static const char * const vc3_clk_out_names[] = {
	"diff2", "diff1", "se3", "se2", "se1", "ref",
};

static unsigned char vc3_pfd_mux_get_parent(struct clk_hw *hw)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	unsigned char ret = 0;
	u32 src;

	regmap_read(vc3->regmap, hwdata->reg, &src);
	if (src & hwdata->offset)
		ret = 1;
	return ret;
}

static int vc3_pfd_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	unsigned int val = index ? hwdata->offset : 0;

	regmap_update_bits(vc3->regmap, hwdata->reg, hwdata->offset, val);
	return 0;
}

static const struct clk_ops vc3_pfd_mux_ops = {
	.set_parent	= vc3_pfd_mux_set_parent,
	.get_parent	= vc3_pfd_mux_get_parent,
};

static unsigned long vc3_pfd_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	unsigned int prediv, premul;
	unsigned long rate = 0;

	if (hwdata->num == 0) {
		regmap_read(vc3->regmap, VC3_PLL1_M_DIVIDER, &prediv);

		/* The bypass_prediv is set, PLL fed from Ref_in directly. */
		if (prediv & VC3_PLL1_M_DIV1) {
			/* check doubler is set or not */
			regmap_read(vc3->regmap, VC3_PLL1_CTRL_OUTDIV5, &premul);
			if (premul & VC3_PLL1_MDIV_DOUBLER)
				parent_rate *= 2;
			return parent_rate;
		}

		if (prediv & VC3_PLL1_M_DIV2)
			rate = parent_rate / 2;
		else
			rate = parent_rate / VC3_PLL1_M_DIV(prediv);
	} else if (hwdata->num == 1) {
		regmap_read(vc3->regmap, VC3_PLL2_M_DIVIDER, &prediv);

		/* The bypass_prediv is set, PLL fed from Ref_in directly. */
		if (prediv & VC3_PLL2_M_DIV1) {
			/* check doubler is set or not */
			if (premul & VC3_PLL2_MDIV_DOUBLER)
				parent_rate *= 2;
			return parent_rate;
		}

		if (prediv & VC3_PLL2_M_DIV2)
			rate = parent_rate / 2;
		else
			rate = parent_rate / VC3_PLL2_M_DIV(prediv);
	} else {
		regmap_read(vc3->regmap, VC3_PLL3_M_DIVIDER, &prediv);

		/* The bypass_prediv is set, PLL fed from Ref_in directly. */
		if (prediv & VC3_PLL3_M_DIV1)
			return parent_rate;

		if (prediv & VC3_PLL3_M_DIV2)
			rate = parent_rate / 2;
		else
			rate = parent_rate / VC3_PLL3_M_DIV(prediv);
	}

	return rate;
}

static long vc3_pfd_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *parent_rate)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	unsigned long idiv;

	/* PLL cannot operate with input clock above 50 MHz. */
	if (rate > 50000000)
		return -EINVAL;

	/* CLKIN within range of PLL input, feed directly to PLL. */
	if (*parent_rate <= 50000000)
		return *parent_rate;

	idiv = DIV_ROUND_UP(*parent_rate, rate);
	if (hwdata->num == 0 || hwdata->num == 2) {
		if (idiv > 63)
			return -EINVAL;
	} else {
		if (idiv > 31)
			return -EINVAL;
	}

	return *parent_rate / idiv;
}

static int vc3_pfd_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	unsigned long idiv;
	u8 div;

	/* CLKIN within range of PLL input, feed directly to PLL. */
	if (parent_rate <= 50000000) {
		if (hwdata->num == 0) {
			regmap_update_bits(vc3->regmap, VC3_PLL1_M_DIVIDER,
					   VC3_PLL1_M_DIV1, VC3_PLL1_M_DIV1);
			regmap_update_bits(vc3->regmap, VC3_PLL1_M_DIVIDER,
					   VC3_PLL1_M_DIV2, 0);
		} else if (hwdata->num == 1) {
			regmap_update_bits(vc3->regmap, VC3_PLL2_M_DIVIDER,
					   VC3_PLL2_M_DIV1, VC3_PLL2_M_DIV1);
			regmap_update_bits(vc3->regmap, VC3_PLL2_M_DIVIDER,
					   VC3_PLL2_M_DIV2, 0);
		} else {
			regmap_update_bits(vc3->regmap, VC3_PLL3_M_DIVIDER,
					   VC3_PLL3_M_DIV1, VC3_PLL3_M_DIV1);
			regmap_update_bits(vc3->regmap, VC3_PLL3_M_DIVIDER,
					   VC3_PLL3_M_DIV2, 0);
		}
		return 0;
	}

	idiv = DIV_ROUND_UP(parent_rate, rate);

	/* We have dedicated div-2 predivider. */
	if (idiv == 2) {
		if (hwdata->num == 0) {
			regmap_update_bits(vc3->regmap, VC3_PLL1_M_DIVIDER,
					   VC3_PLL1_M_DIV2, VC3_PLL1_M_DIV2);
			regmap_update_bits(vc3->regmap, VC3_PLL1_M_DIVIDER,
					   VC3_PLL1_M_DIV1, 0);
		} else if (hwdata->num == 1) {
			regmap_update_bits(vc3->regmap, VC3_PLL2_M_DIVIDER,
					   VC3_PLL2_M_DIV2, VC3_PLL2_M_DIV2);
			regmap_update_bits(vc3->regmap, VC3_PLL2_M_DIVIDER,
					   VC3_PLL2_M_DIV1, 0);
		} else {
			regmap_update_bits(vc3->regmap, VC3_PLL3_M_DIVIDER,
					   VC3_PLL3_M_DIV2, VC3_PLL3_M_DIV2);
			regmap_update_bits(vc3->regmap, VC3_PLL3_M_DIVIDER,
					   VC3_PLL3_M_DIV1, 0);
		}
	} else {
		if (hwdata->num == 0) {
			div = VC3_PLL1_M_DIV(idiv);
			regmap_write(vc3->regmap, VC3_PLL1_M_DIVIDER, div);
		} else if (hwdata->num == 1) {
			div = VC3_PLL2_M_DIV(idiv);
			regmap_write(vc3->regmap, VC3_PLL2_M_DIVIDER, div);
		} else {
			div = VC3_PLL3_M_DIV(idiv);
			regmap_write(vc3->regmap, VC3_PLL3_M_DIVIDER, div);
		}
	}

	return 0;
}

static const struct clk_ops vc3_pfd_ops = {
	.recalc_rate	= vc3_pfd_recalc_rate,
	.round_rate	= vc3_pfd_round_rate,
	.set_rate	= vc3_pfd_set_rate,
};

static unsigned long vc3_pll_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	u32 div_int, div_frc, val;
	unsigned long rate;

	if (hwdata->num == 0) {
		regmap_read(vc3->regmap, VC3_PLL1_LOOP_FILTER_N_DIVIDER, &val);
		div_int = (val & 0x7) << 8;
		regmap_read(vc3->regmap, VC3_PLL1_VCO_N_DIVIDER, &val);
		div_int |= val;
		rate = parent_rate * div_int;
	} else if (hwdata->num == 1) {
		regmap_read(vc3->regmap, VC3_PLL2_FB_INT_DIV_MSB, &val);
		div_int = (val & 0x7) << 8;
		regmap_read(vc3->regmap, VC3_PLL2_FB_INT_DIV_LSB, &val);
		div_int |= val;

		regmap_read(vc3->regmap, VC3_PLL2_FB_FRC_DIV_MSB, &val);
		div_frc = val << 8;
		regmap_read(vc3->regmap, VC3_PLL2_FB_FRC_DIV_LSB, &val);
		div_frc |= val;
		rate = (parent_rate * (div_int * VC3_2_POWER_16 + div_frc) / VC3_2_POWER_16);
	} else {
		regmap_read(vc3->regmap, VC3_PLL3_LOOP_FILTER_N_DIVIDER, &val);
		div_int = (val & 0x7) << 8;
		regmap_read(vc3->regmap, VC3_PLL3_N_DIV_LSB, &val);
		div_int |= val;
		rate = parent_rate * div_int;
	}

	return rate;
}

static long vc3_pll_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *parent_rate)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	u32 div_int;
	u64 div_frc;

	if (hwdata->num == 0) {
		if (rate < VC3_PLL1_VCO_MIN)
			rate = VC3_PLL1_VCO_MIN;
		if (rate > VC3_PLL1_VCO_MAX)
			rate = VC3_PLL1_VCO_MAX;

		div_int = rate / *parent_rate;
		hwdata->div_int = div_int;
		hwdata->div_frc = 0;
		rate = *parent_rate * div_int;
	} else if (hwdata->num == 1) {
		if (rate < VC3_PLL2_VCO_MIN)
			rate = VC3_PLL2_VCO_MIN;
		if (rate > VC3_PLL2_VCO_MAX)
			rate = VC3_PLL2_VCO_MAX;

		/* Determine integer part, which is 11 bit wide */
		div_int = rate / *parent_rate;
		if (div_int > 0x7ff)
			rate = *parent_rate * 0x7ff;

		/* Determine best fractional part, which is 16 bit wide */
		div_frc = rate % *parent_rate;
		div_frc *= BIT(16) - 1;
		do_div(div_frc, *parent_rate);

		hwdata->div_int = div_int;
		hwdata->div_frc = (u32)div_frc;
		rate = (*parent_rate * (div_int * VC3_2_POWER_16 + div_frc) / VC3_2_POWER_16);
	} else {
		if (rate < VC3_PLL3_VCO_MIN)
			rate = VC3_PLL3_VCO_MIN;
		if (rate > VC3_PLL3_VCO_MAX)
			rate = VC3_PLL3_VCO_MAX;

		/* Determine integer part, which is 10 bit wide */
		div_int = rate / *parent_rate;
		hwdata->div_int = div_int;
		hwdata->div_frc = (u32)0;
		rate = *parent_rate * div_int;
	}

	return rate;
}

static int vc3_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	u32 val;

	if (hwdata->num == 0) {
		regmap_read(vc3->regmap, VC3_PLL1_LOOP_FILTER_N_DIVIDER, &val);
		val = (val & 0xf8) | ((hwdata->div_int >> 8) & 0x7);
		regmap_write(vc3->regmap, VC3_PLL1_LOOP_FILTER_N_DIVIDER, val);
		regmap_write(vc3->regmap, VC3_PLL1_VCO_N_DIVIDER, hwdata->div_int & 0xff);
	} else if (hwdata->num == 1) {
		regmap_read(vc3->regmap, VC3_PLL2_FB_INT_DIV_MSB, &val);
		val = (val & 0xf8) | ((hwdata->div_int >> 8) & 0x7);
		regmap_write(vc3->regmap, VC3_PLL2_FB_INT_DIV_MSB, val);
		regmap_write(vc3->regmap, VC3_PLL2_FB_INT_DIV_LSB, hwdata->div_int & 0xff);

		regmap_write(vc3->regmap, VC3_PLL2_FB_FRC_DIV_MSB, hwdata->div_frc >> 8);
		regmap_write(vc3->regmap, VC3_PLL2_FB_FRC_DIV_LSB, hwdata->div_frc & 0xff);
	} else {
		regmap_read(vc3->regmap, VC3_PLL3_LOOP_FILTER_N_DIVIDER, &val);
		val = (val & 0xf8) | ((hwdata->div_int >> 8) & 0x7);
		regmap_write(vc3->regmap, VC3_PLL3_LOOP_FILTER_N_DIVIDER, val);
		regmap_write(vc3->regmap, VC3_PLL3_N_DIV_LSB, hwdata->div_int & 0xff);
	}

	return 0;
}

static const struct clk_ops vc3_pll_ops = {
	.recalc_rate = vc3_pll_recalc_rate,
	.round_rate = vc3_pll_round_rate,
	.set_rate = vc3_pll_set_rate,
};

static unsigned char vc3_div_mux_get_parent(struct clk_hw *hw)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	unsigned char ret = 0;
	u32 src;

	regmap_read(vc3->regmap, hwdata->reg, &src);
	if (src & hwdata->offset)
		ret = 1;
	return ret;
}

static int vc3_div_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	unsigned int val = index ? hwdata->offset : 0;

	regmap_update_bits(vc3->regmap, hwdata->reg, hwdata->offset, val);
	return 0;
}

static const struct clk_ops vc3_div_mux_ops = {
	.set_parent	= vc3_div_mux_set_parent,
	.get_parent	= vc3_div_mux_get_parent,
};

static const struct clk_div_table div1_divs[] = {
	{ .val = 0, .div = 1, },
	{ .val = 1, .div = 4, },
	{ .val = 2, .div = 5, },
	{ .val = 3, .div = 6, },
	{ .val = 4, .div = 2, },
	{ .val = 5, .div = 8, },
	{ .val = 6, .div = 10, },
	{ .val = 7, .div = 12, },
	{ .val = 8, .div = 4, },
	{ .val = 9, .div = 16, },
	{ .val = 10, .div = 20, },
	{ .val = 11, .div = 24, },
	{ .val = 12, .div = 8, },
	{ .val = 13, .div = 32, },
	{ .val = 14, .div = 40, },
	{ .val = 15, .div = 48, },
	{}
};

static const struct clk_div_table div245_divs[] = {
	{ .val = 0, .div = 1, },
	{ .val = 1, .div = 3, },
	{ .val = 2, .div = 5, },
	{ .val = 3, .div = 10, },
	{ .val = 4, .div = 2, },
	{ .val = 5, .div = 6, },
	{ .val = 6, .div = 10, },
	{ .val = 7, .div = 20, },
	{ .val = 8, .div = 4, },
	{ .val = 9, .div = 12, },
	{ .val = 10, .div = 20, },
	{ .val = 11, .div = 40, },
	{ .val = 12, .div = 5, },
	{ .val = 13, .div = 15, },
	{ .val = 14, .div = 25, },
	{ .val = 15, .div = 50, },
	{}
};

static const struct clk_div_table div3_divs[] = {
	{ .val = 0, .div = 1, },
	{ .val = 1, .div = 3, },
	{ .val = 2, .div = 5, },
	{ .val = 3, .div = 10, },
	{ .val = 4, .div = 2, },
	{ .val = 5, .div = 6, },
	{ .val = 6, .div = 10, },
	{ .val = 7, .div = 20, },
	{ .val = 8, .div = 4, },
	{ .val = 9, .div = 12, },
	{ .val = 10, .div = 20, },
	{ .val = 11, .div = 40, },
	{ .val = 12, .div = 8, },
	{ .val = 13, .div = 24, },
	{ .val = 14, .div = 40, },
	{ .val = 15, .div = 80, },
	{}
};

static unsigned int vc3_get_div(const struct clk_div_table *table,
				unsigned int val, unsigned long flag)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->val == val)
			return clkt->div;
	return 0;
}

static unsigned long vc3_div_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct vc3_hw_div *hwdiv = container_of(hw, struct vc3_hw_div, hw);
	struct vc3_driver_data *vc3 = hwdiv->vc3;
	unsigned int val;

	regmap_read(vc3->regmap, hwdiv->reg, &val);
	val >>= hwdiv->shift;
	val &= VC3_DIV_MASK(hwdiv->width);
	return divider_recalc_rate(hw, parent_rate, val, hwdiv->table,
				   hwdiv->flags, hwdiv->width);
}

static long vc3_div_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *parent_rate)
{
	struct vc3_hw_div *hwdiv = container_of(hw, struct vc3_hw_div, hw);
	struct vc3_driver_data *vc3 = hwdiv->vc3;
	unsigned int bestdiv;

	/* if read only, just return current value */
	if (hwdiv->flags & CLK_DIVIDER_READ_ONLY) {
		regmap_read(vc3->regmap, hwdiv->reg, &bestdiv);
		bestdiv >>= hwdiv->shift;
		bestdiv &= VC3_DIV_MASK(hwdiv->width);
		bestdiv = vc3_get_div(hwdiv->table, bestdiv, hwdiv->flags);
		return DIV_ROUND_UP(*parent_rate, bestdiv);
	}

	return divider_round_rate(hw, rate, parent_rate, hwdiv->table,
				  hwdiv->width, hwdiv->flags);
}

static int vc3_div_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct vc3_hw_div *hwdiv = container_of(hw, struct vc3_hw_div, hw);
	struct vc3_driver_data *vc3 = hwdiv->vc3;
	unsigned int value;

	value = divider_get_val(rate, parent_rate, hwdiv->table,
				hwdiv->width, hwdiv->flags);
	regmap_update_bits(vc3->regmap, hwdiv->reg,
			   VC3_DIV_MASK(hwdiv->width) << hwdiv->shift,
			   value << hwdiv->shift);
	return 0;
}

static const struct clk_ops vc3_div_ops = {
	.recalc_rate = vc3_div_recalc_rate,
	.round_rate = vc3_div_round_rate,
	.set_rate = vc3_div_set_rate,
};

static int vc3_clk_mux_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	int ret = 0;
	int frc = 0;

	ret = clk_mux_determine_rate_flags(hw, req, CLK_SET_RATE_PARENT);
	if (ret) {
		if ((req->rate % req->best_parent_rate) != 0) {
			frc = DIV_ROUND_CLOSEST_ULL(req->best_parent_rate, req->rate);
			req->rate *= frc;
			return clk_mux_determine_rate_flags(hw, req, CLK_SET_RATE_PARENT);
		}
		ret = 0;
	}

	return ret;
}

static unsigned char vc3_clk_mux_get_parent(struct clk_hw *hw)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	u32 val;
	u8 index = 0;

	regmap_read(vc3->regmap, hwdata->reg, &val);
	if (val & hwdata->offset)
		index = 1;

	if (vc3->is_32k_used) {
		u8 offset = 0;

		if (hwdata->num == 2)
			offset = VC3_SE3_FREERUN_32K;
		else if (hwdata->num == 3)
			offset = VC3_SE2_FREERUN_32K;
		else if (hwdata->num == 4)
			offset = VC3_SE1_FREERUN_32K;

		if (offset && (!(val & offset)))
			index = 2;
	}

	return index;
}

static int vc3_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	unsigned int val = index ? hwdata->offset : 0;
	u8 offset = hwdata->offset;

	if (vc3->is_32k_used) {
		if (index == 2) {
			val = 0;
			if (hwdata->num == 2)
				offset = VC3_SE3_FREERUN_32K;
			else if (hwdata->num == 3)
				offset = VC3_SE2_FREERUN_32K;
			else if (hwdata->num == 4)
				offset = VC3_SE1_FREERUN_32K;
		}
	}

	regmap_update_bits(vc3->regmap, hwdata->reg, offset, val);
	return 0;
}

static const struct clk_ops vc3_clk_mux_ops = {
	.determine_rate = vc3_clk_mux_determine_rate,
	.set_parent	= vc3_clk_mux_set_parent,
	.get_parent	= vc3_clk_mux_get_parent,
};

static int vc3_gate_enable(struct clk_hw *hw)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;

	regmap_update_bits(vc3->regmap, hwdata->reg, hwdata->offset, hwdata->offset);
	return 0;
}

static void vc3_gate_disable(struct clk_hw *hw)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;

	regmap_update_bits(vc3->regmap, hwdata->reg, hwdata->offset, 0);
}

static int vc3_gate_is_enabled(struct clk_hw *hw)
{
	struct vc3_hw_data *hwdata = container_of(hw, struct vc3_hw_data, hw);
	struct vc3_driver_data *vc3 = hwdata->vc3;
	int ret = 0;
	u32 val;

	regmap_read(vc3->regmap, hwdata->reg, &val);
	if (val & hwdata->offset)
		ret = 1;

	return ret;
}

static const struct clk_ops vc3_gate_ops = {
	.enable = vc3_gate_enable,
	.disable = vc3_gate_disable,
	.is_enabled = vc3_gate_is_enabled,
};

static bool vc3_regmap_is_writeable(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config vc3_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x24,
	.writeable_reg = vc3_regmap_is_writeable,
};

static struct clk_hw *vc3_of_clk_get(struct of_phandle_args *clkspec,
				     void *data)
{
	struct vc3_driver_data *vc3 = data;
	unsigned int idx = clkspec->args[0];

	return &vc3->clk_out[idx].hw;
}

static void vc3_divider_type_parse_dt(struct device *dev,
				      struct vc3_driver_data *vc3)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	const __be32 *p;
	u32 i = 0;
	u32 val;

	of_property_for_each_u32(np, "clock-divider-read-only", prop, p, val) {
		if (i >= VC3_MAX_DIV_NUM)
			break;
		if (val == 1)
			vc3->clk_div[i].flags = CLK_DIVIDER_READ_ONLY;

		i++;
	}
}

static void vc3_clk_flags_parse_dt(struct device *dev, u32 *crt_clks)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	const __be32 *p;
	u32 i = 0;
	u32 val;

	of_property_for_each_u32(np, "clock-flags", prop, p, val) {
		if (i >= VC3_MAX_CLK_OUT_NUM)
			break;
		*crt_clks++ = val;
		i++;
	}
}

static void vc3_mux_type_parse_dt(struct device *dev,
				  struct vc3_driver_data *vc3)
{
	struct device_node *np = dev->of_node;

	if (of_property_read_bool(np, "32kHz-free-running"))
		vc3->is_32k_used = true;
	else
		vc3->is_32k_used = false;

}

static int vc3_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct vc3_driver_data *vc3;
	const char *parent_names[2];
	const char *pll_parent_names[3];
	struct clk_init_data init;
	u8 settings[NUM_CONFIG_REGISTERS];
	int ret, i;
	u32 crit_clks[6] = {};
	struct device *dev = &client->dev;

	vc3 = devm_kzalloc(&client->dev, sizeof(*vc3), GFP_KERNEL);
	if (!vc3)
		return -ENOMEM;

	i2c_set_clientdata(client, vc3);
	vc3->client = client;

	vc3->pin_x1 = devm_clk_get(&client->dev, "x1");
	if (PTR_ERR(vc3->pin_x1) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	vc3->pin_clkinb = devm_clk_get(&client->dev, "clkinb");
	if (PTR_ERR(vc3->pin_clkinb) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	vc3->regmap = devm_regmap_init_i2c(client, &vc3_regmap_config);
	if (IS_ERR(vc3->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(vc3->regmap);
	}

	ret = of_property_read_u8_array(dev->of_node, "renesas,settings",
					settings, ARRAY_SIZE(settings));
	if (!ret) {
		/*
		* A raw settings array was specified in the DT. Write the
		* settings to the device immediately.
		*/
		for  (i = 0; i < NUM_CONFIG_REGISTERS; i++) {
			ret = regmap_write(vc3->regmap, i, settings[i]);
			if (ret) {
				dev_err(dev, "error writing to chip (%i)\n", ret);
				return ret;
			}
		}
	} else if (ret == -EOVERFLOW) {
		dev_err(&client->dev, "EOVERFLOW reg settings. ARRAY_SIZE: %zu",
			ARRAY_SIZE(settings));
		return ret;
	}

	/* Register clock ref */
	memset(&init, 0, sizeof(init));

	if (!IS_ERR(vc3->pin_x1)) {
		vc3->clk_in = VC3_X1;
		parent_names[init.num_parents++] = __clk_get_name(vc3->pin_x1);
	} else if (!IS_ERR(vc3->pin_clkinb)) {
		vc3->clk_in = VC3_CLKINB;
		parent_names[init.num_parents++] = __clk_get_name(vc3->pin_clkinb);
	}

	if (!init.num_parents) {
		dev_err(&client->dev, "no input clock specified!\n");
		return -EINVAL;
	}

	/* Register pfd mux's. */
	parent_names[1] = vc3_div_names[1];
	for (i = 0; i < VC3_MAX_PFD_MUX_NUM; i++) {
		vc3->clk_pfd_mux[i].num = i;
		vc3->clk_pfd_mux[i].vc3 = vc3;
		init.name = vc3_pfd_mux_names[i];
		init.ops = &vc3_pfd_mux_ops;
		init.flags = CLK_SET_RATE_PARENT;
		init.num_parents = 2;
		init.parent_names = parent_names;
		vc3->clk_pfd_mux[i].hw.init = &init;
		if (i == 0) {
			vc3->clk_pfd_mux[i].reg = VC3_PLL_OP_CTRL;
			vc3->clk_pfd_mux[i].offset = VC3_PLL2_REFIN_SEL;
		} else {
			vc3->clk_pfd_mux[i].reg = VC3_GENERAL_CTR;
			vc3->clk_pfd_mux[i].offset = VC3_PLL3_REFIN_SEL;
		}
		ret = devm_clk_hw_register(&client->dev, &vc3->clk_pfd_mux[i].hw);
		if (ret) {
			dev_err(&client->dev, "%s registration failed\n",
				vc3_pfd_mux_names[i]);
			return ret;
		}
	}

	/* Register pfd's */
	for (i = 0; i < VC3_MAX_PFD_NUM; i++) {
		vc3->clk_pfd[i].num = i;
		vc3->clk_pfd[i].vc3 = vc3;
		init.name = vc3_pfd_names[i];
		init.ops = &vc3_pfd_ops;
		init.flags = CLK_SET_RATE_PARENT;
		init.num_parents = 1;
		if (i == 0)
			pll_parent_names[0] = parent_names[0];
		else
			pll_parent_names[0] = vc3_pfd_mux_names[i - 1];

		init.parent_names = pll_parent_names;

		vc3->clk_pfd[i].hw.init = &init;
		ret = devm_clk_hw_register(&client->dev, &vc3->clk_pfd[i].hw);
		if (ret) {
			dev_err(&client->dev, "%s registration failed\n",
				vc3_pfd_names[i]);
			return ret;
		}
	}

	/* Register Pll's */
	for (i = 0; i < VC3_MAX_PLL_NUM; i++) {
		vc3->clk_pll[i].num = i;
		vc3->clk_pll[i].vc3 = vc3;
		pll_parent_names[0] = vc3_pfd_names[i];
		init.name = vc3_pll_names[i];
		init.ops = &vc3_pll_ops;
		init.flags = CLK_SET_RATE_PARENT;
		init.num_parents = 1;
		init.parent_names = pll_parent_names;

		vc3->clk_pll[i].hw.init = &init;
		ret = devm_clk_hw_register(&client->dev, &vc3->clk_pll[i].hw);
		if (ret) {
			dev_err(&client->dev, "%s registration failed\n",
				vc3_pll_names[i]);
			return ret;
		}
	}

	/* Register Div mux's */
	for (i = 0; i < VC3_MAX_DIV_MUX_NUM; i++) {
		vc3->clk_div_mux[i].num = i;
		vc3->clk_div_mux[i].vc3 = vc3;
		init.name = vc3_div_mux_names[i];
		init.ops = &vc3_div_mux_ops;
		init.flags = CLK_SET_RATE_PARENT;
		init.num_parents = 2;
		if (i == 0) {
			vc3->clk_div_mux[i].reg = VC3_GENERAL_CTR;
			vc3->clk_div_mux[i].offset = VC3_DIV1_SRC_SEL;
			pll_parent_names[0] = vc3_pll_names[0];
			pll_parent_names[1] = parent_names[0];
			init.parent_names = pll_parent_names;
		} else if (i == 1) {
			vc3->clk_div_mux[i].reg = VC3_PLL3_CHARGE_PUMP_CTRL;
			vc3->clk_div_mux[i].offset = VC3_DIV3_SRC_SEL;
			pll_parent_names[0] = vc3_pll_names[1];
			pll_parent_names[1] = vc3_pll_names[2];
			init.parent_names = pll_parent_names;
		} else {
			vc3->clk_div_mux[i].reg = VC3_OUTPUT_CTR;
			vc3->clk_div_mux[i].offset = VC3_DIV4_SRC_SEL;
			pll_parent_names[0] = vc3_pll_names[1];
			pll_parent_names[1] = parent_names[0];
			init.parent_names = pll_parent_names;
		}

		vc3->clk_div_mux[i].hw.init = &init;
		ret = devm_clk_hw_register(&client->dev, &vc3->clk_div_mux[i].hw);
		if (ret) {
			dev_err(&client->dev, "%s registration failed\n",
				vc3_div_mux_names[i]);
			return ret;
		}
	}

	vc3_divider_type_parse_dt(&client->dev, vc3);
	/* Register Dividers */
	for (i = 0; i < VC3_MAX_DIV_NUM; i++) {
		vc3->clk_div[i].num = i;
		vc3->clk_div[i].vc3 = vc3;
		init.name = vc3_div_names[i];
		init.ops = &vc3_div_ops;
		init.flags = CLK_SET_RATE_PARENT;
		init.num_parents = 1;

		switch (i) {
		case 0:
			pll_parent_names[0] = vc3_div_mux_names[0];
			vc3->clk_div[i].table = div1_divs;
			vc3->clk_div[i].reg = VC3_DIV1_DIV2_CTRL;
			vc3->clk_div[i].shift = 4;
			vc3->clk_div[i].width = 4;
			init.parent_names = pll_parent_names;
			break;
		case 1:
			pll_parent_names[0] = vc3_pll_names[0];
			vc3->clk_div[i].table = div245_divs;
			vc3->clk_div[i].reg = VC3_DIV1_DIV2_CTRL;
			vc3->clk_div[i].shift = 0;
			vc3->clk_div[i].width = 4;
			init.parent_names = pll_parent_names;
			break;
		case 2:
			pll_parent_names[0] = vc3_div_mux_names[1];
			vc3->clk_div[i].table = div3_divs;
			vc3->clk_div[i].reg = VC3_DIV3_DIV4_CTRL;
			vc3->clk_div[i].shift = 4;
			vc3->clk_div[i].width = 4;
			init.parent_names = pll_parent_names;
		break;
		case 3:
			pll_parent_names[0] = vc3_div_mux_names[2];
			vc3->clk_div[i].table = div245_divs;
			vc3->clk_div[i].reg = VC3_DIV3_DIV4_CTRL;
			vc3->clk_div[i].shift = 0;
			vc3->clk_div[i].width = 4;
			init.parent_names = pll_parent_names;
		break;
		case 4:
			pll_parent_names[0] = vc3_pll_names[2];
			vc3->clk_div[i].table = div245_divs;
			vc3->clk_div[i].reg = VC3_PLL1_CTRL_OUTDIV5;
			vc3->clk_div[i].shift = 0;
			vc3->clk_div[i].width = 4;
			init.parent_names = pll_parent_names;
			break;
		}

		vc3->clk_div[i].hw.init = &init;
		ret = devm_clk_hw_register(&client->dev, &vc3->clk_div[i].hw);
		if (ret) {
			dev_err(&client->dev, "%s divider registration failed\n",
				vc3_div_names[i]);
			return ret;
		}
	}

	clk_hw_register_fixed_rate(NULL, "32k", NULL, 0, 32768);
	vc3_mux_type_parse_dt(&client->dev, vc3);
	/* Clk muxes */
	for (i = 0; i < VC3_MAX_CLK_MUX_NUM; i++) {
		vc3->clk_mux[i].num = i;
		vc3->clk_mux[i].vc3 = vc3;
		init.name = vc3_clk_mux_names[i];
		init.ops = &vc3_clk_mux_ops;
		init.flags = CLK_SET_RATE_PARENT;
		switch (i) {
		case 0:
			vc3->clk_mux[i].reg = VC3_DIFF2_CTRL_REG;
			vc3->clk_mux[i].offset = VC3_DIFF2_CLK_SEL;
			pll_parent_names[0] = vc3_div_names[0];
			pll_parent_names[1] = vc3_div_names[2];
			init.num_parents = 2;
			init.parent_names = pll_parent_names;
			break;
		case 1:
			vc3->clk_mux[i].reg = VC3_DIFF1_CTRL_REG;
			vc3->clk_mux[i].offset = VC3_DIFF1_CLK_SEL;
			pll_parent_names[0] = vc3_div_names[0];
			pll_parent_names[1] = vc3_div_names[2];
			init.num_parents = 2;
			init.parent_names = pll_parent_names;
			break;
		case 2:
			vc3->clk_mux[i].reg = VC3_SE3_CTRL_REG;
			vc3->clk_mux[i].offset = VC3_SE3_CLK_SEL;
			pll_parent_names[0] = vc3_div_names[1];
			pll_parent_names[1] = vc3_div_names[3];
			pll_parent_names[2] = "32k";
			init.num_parents = (vc3->is_32k_used) ? 3 : 2;
			init.parent_names = pll_parent_names;
			break;
		case 3:
			vc3->clk_mux[i].reg = VC3_SE2_CTRL_REG0;
			vc3->clk_mux[i].offset = VC3_SE2_CLK_SEL;
			pll_parent_names[0] = vc3_div_names[4];
			pll_parent_names[1] = vc3_div_names[3];
			pll_parent_names[2] = "32k";
			init.num_parents = (vc3->is_32k_used) ? 3 : 2;
			init.parent_names = pll_parent_names;
			break;
		case 4:
			vc3->clk_mux[i].reg = VC3_SE1_DIV4_CTRL;
			vc3->clk_mux[i].offset = VC3_SE1_CLK_SEL;
			pll_parent_names[0] = vc3_div_names[4];
			pll_parent_names[1] = vc3_div_names[3];
			pll_parent_names[2] = "32k";
			init.num_parents = (vc3->is_32k_used) ? 3 : 2;
			init.parent_names = pll_parent_names;
			break;
		}

		vc3->clk_mux[i].hw.init = &init;
		ret = devm_clk_hw_register(&client->dev, &vc3->clk_mux[i].hw);
		if (ret) {
			dev_err(&client->dev, "%s clk mux registration failed\n",
				vc3_clk_mux_names[i]);
			return ret;
		}
	}

	/* Clk outputs */
	vc3_clk_flags_parse_dt(&client->dev, crit_clks);
	for (i = 0; i < VC3_MAX_CLK_OUT_NUM; i++) {
		vc3->clk_out[i].num = i;
		vc3->clk_out[i].vc3 = vc3;
		switch (i) {
		case 0:
			vc3->clk_out[i].reg = VC3_OUTPUT_CTR;
			vc3->clk_out[i].offset = VC3_DIFF2_OUT_EN;
			pll_parent_names[0] = vc3_clk_mux_names[i];
			init.name = vc3_clk_out_names[i];
			init.num_parents = 1;
			init.flags = crit_clks[i];
			init.parent_names = pll_parent_names;
			break;
		case 1:
			vc3->clk_out[i].reg = VC3_OUTPUT_CTR;
			vc3->clk_out[i].offset = VC3_DIFF1_OUT_EN;
			pll_parent_names[0] = vc3_clk_mux_names[i];
			init.name = vc3_clk_out_names[i];
			init.flags = crit_clks[i];
			init.parent_names = pll_parent_names;
			break;
		case 2:
			vc3->clk_out[i].reg = VC3_OE_DFC_CTRL;
			vc3->clk_out[i].offset = VC3_SE3_EN;
			pll_parent_names[0] = vc3_clk_mux_names[i];
			init.name = vc3_clk_out_names[i];
			init.flags = crit_clks[i];
			init.parent_names = pll_parent_names;
			break;
		case 3:
			vc3->clk_out[i].reg = VC3_OE_DFC_CTRL;
			vc3->clk_out[i].offset = VC3_SE2_EN;
			pll_parent_names[0] = vc3_clk_mux_names[i];
			init.name = vc3_clk_out_names[i];
			init.flags = crit_clks[i];
			init.parent_names = pll_parent_names;
			break;
		case 4:
			vc3->clk_out[i].reg = VC3_OE_DFC_CTRL;
			vc3->clk_out[i].offset = VC3_SE1_EN;
			pll_parent_names[0] = vc3_clk_mux_names[i];
			init.name = vc3_clk_out_names[i];
			init.flags = crit_clks[i];
			init.parent_names = pll_parent_names;
			break;
		case 5:
			vc3->clk_out[i].reg = VC3_OE_DFC_CTRL;
			vc3->clk_out[i].offset = VC3_REF_EN;
			init.name = vc3_clk_out_names[i];
			init.flags = crit_clks[i];
			init.parent_names = parent_names;
			break;
		}
		vc3->clk_out[i].hw.init = &init;
		ret = devm_clk_hw_register(&client->dev, &vc3->clk_out[i].hw);
		if (ret) {
			dev_err(&client->dev, "%s clk out registration failed\n",
				vc3_clk_out_names[i]);
			return ret;
		}
	}

	ret = of_clk_add_hw_provider(client->dev.of_node, vc3_of_clk_get, vc3);
	if (ret) {
		dev_err(&client->dev, "unable to add clk provider\n");
		return ret;
	}

	return 0;
}

static int vc3_remove(struct i2c_client *client)
{
	of_clk_del_provider(client->dev.of_node);
	return 0;
}

static const struct of_device_id dev_ids[] = {
	{ .compatible = "renesas,5p35023"},
	{}
};
MODULE_DEVICE_TABLE(of, dev_ids);

static struct i2c_driver vc3_driver = {
	.driver = {
		.name = "vc3",
		.of_match_table = of_match_ptr(dev_ids),
	},
	.probe		= vc3_probe,
	.remove		= vc3_remove,
};
module_i2c_driver(vc3_driver);

MODULE_AUTHOR("Biju Das <biju.das.jz@bp.renesas.com>");
MODULE_DESCRIPTION("Renesas VersaClock 3 driver");
MODULE_LICENSE("GPL");
