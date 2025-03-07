// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/V2H(P) Clock Pulse Generator
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 * Based on rzg2l-cpg.c
 *
 * Copyright (C) 2015 Glider bvba
 * Copyright (C) 2013 Ideas On Board SPRL
 * Copyright (C) 2015 Renesas Electronics Corp.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>
#include <linux/reset-controller.h>
#include <linux/units.h>

#include <dt-bindings/clock/renesas-cpg-mssr.h>

#include "rzv2h-cpg.h"

#ifdef DEBUG
#define WARN_DEBUG(x)		WARN_ON(x)
#else
#define WARN_DEBUG(x)		do { } while (0)
#endif

#define GET_CLK_ON_OFFSET(x)	(0x600 + ((x) * 4))
#define GET_CLK_MON_OFFSET(x)	(0x800 + ((x) * 4))
#define GET_RST_OFFSET(x)	(0x900 + ((x) * 4))
#define GET_RST_MON_OFFSET(x)	(0xA00 + ((x) * 4))

#define PLL_RESETB		BIT(0)
#define PLL_SSC_EN		BIT(2)
#define PLL_RESETB_WEN		BIT(16)
#define PLL_SSC_EN_WEN		BIT(18)

#define PLL_MON_RESETB		BIT(0)
#define PLL_MON_LOCK		BIT(4)

#define PLL_FVCO_MIN		(1600000000)
#define PLL_FVCO_MAX		(3200000000)
#define PLL_DIV_P_MIN		(1)
#define PLL_DIV_P_MAX		(4)
#define PLL_DIV_M_MIN		(64)
#define PLL_DIV_M_MAX		(533)
#define PLL_DIV_S_MIN		(0)
#define PLL_DIV_S_MAX		(6)

#define KDIV(val)		((s16)FIELD_GET(GENMASK(31, 16), (val)))
#define MDIV(val)		FIELD_GET(GENMASK(15, 6), (val))
#define PDIV(val)		FIELD_GET(GENMASK(5, 0), (val))
#define SDIV(val)		FIELD_GET(GENMASK(2, 0), (val))

#define DDIV_DIVCTL_WEN(shift)		BIT((shift) + 16)

#define GET_MOD_CLK_ID(base, index, bit)		\
			((base) + ((((index) * (16))) + (bit)))

#define CPG_CLKSTATUS0		(0x700)

#define MAX_VCLK_FREQ		(187500000)
#define MIN_VCLK_FREQ		(5440000)
#define LIMIT_VCLK_FREQ		(25000000)

/**
 * struct rzv2h_cpg_priv - Clock Pulse Generator Private Data
 *
 * @dev: CPG device
 * @base: CPG register block base address
 * @rmw_lock: protects register accesses
 * @clks: Array containing all Core and Module Clocks
 * @num_core_clks: Number of Core Clocks in clks[]
 * @num_mod_clks: Number of Module Clocks in clks[]
 * @resets: Array of resets
 * @num_resets: Number of Module Resets in info->resets[]
 * @last_dt_core_clk: ID of the last Core Clock exported to DT
 * @rcdev: Reset controller entity
 */
struct rzv2h_cpg_priv {
	struct device *dev;
	void __iomem *base;
	spinlock_t rmw_lock;

	struct clk **clks;
	unsigned int num_core_clks;
	unsigned int num_mod_clks;
	struct rzv2h_reset *resets;
	unsigned int num_resets;
	unsigned int last_dt_core_clk;

	struct reset_controller_dev rcdev;
};

#define rcdev_to_priv(x)	container_of(x, struct rzv2h_cpg_priv, rcdev)

struct pll_clk {
	struct rzv2h_cpg_priv *priv;
	void __iomem *base;
	struct clk_hw hw;
	unsigned int conf;
	unsigned int type;
	unsigned long min;
	unsigned long max;
};

#define to_pll(_hw)	container_of(_hw, struct pll_clk, hw)

/**
 * struct mod_clock - Module clock
 *
 * @priv: CPG private data
 * @hw: handle between common and hardware-specific interfaces
 * @on_index: register offset
 * @on_bit: ON/MON bit
 * @mon_index: monitor register offset
 * @mon_bit: montor bit
 */
struct mod_clock {
	struct rzv2h_cpg_priv *priv;
	struct clk_hw hw;
	u8 on_index;
	u8 on_bit;
	s8 mon_index;
	u8 mon_bit;
};

#define to_mod_clock(_hw) container_of(_hw, struct mod_clock, hw)

/**
 * struct sddiv_clk - Static/Dynamic DIV clock
 *
 * @priv: CPG private data
 * @div: divider clk
 * @mon: monitor bit in CPG_CLKSTATUS0 register
 */
struct sddiv_clk {
	struct rzv2h_cpg_priv *priv;
	struct clk_divider div;
	s8 mon;
};

#define to_sddiv_clock(_div) container_of(_div, struct sddiv_clk, div)

static unsigned long rzv2h_cpg_pll_clk_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	struct pll_clk *pll_clk = to_pll(hw);
	struct rzv2h_cpg_priv *priv = pll_clk->priv;
	unsigned int clk1, clk2;
	u64 rate;

	if (!PLL_CLK_ACCESS(pll_clk->conf))
		return 0;

	clk1 = readl(priv->base + PLL_CLK1_OFFSET(pll_clk->conf));
	clk2 = readl(priv->base + PLL_CLK2_OFFSET(pll_clk->conf));

	rate = mul_u64_u32_shr(parent_rate, (MDIV(clk1) << 16) + KDIV(clk1),
			       16 + SDIV(clk2));

	return DIV_ROUND_CLOSEST_ULL(rate, PDIV(clk1));
}
struct rzv2h_pll_div_hw_data {
	struct clk_hw hw;
	u32 conf;
	u32 div;
	u32 mult;
	struct rzv2h_cpg_priv *priv;
};

#define to_rzv2h_pll_div_hw_data(_hw) \
			container_of(_hw, struct rzv2h_pll_div_hw_data, hw)

static unsigned long rzv2h_cpg_pll_div_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	struct rzv2h_pll_div_hw_data *pll_div = to_rzv2h_pll_div_hw_data(hw);
	unsigned long long int rate;

	rate = (unsigned long long int) parent_rate * pll_div->mult;
	do_div(rate, pll_div->div);

	return (unsigned long)rate;
}

static int rzv2h_cpg_pll_div_determine_rate(struct clk_hw *hw,
					    struct clk_rate_request *req)
{
	struct rzv2h_pll_div_hw_data *pll_div = to_rzv2h_pll_div_hw_data(hw);

	req->best_parent_rate = (req->rate / pll_div->mult) * pll_div->div;

	return 0;
};

static int rzv2h_cpg_pll_div_set_rate(struct clk_hw *hw,
				      unsigned long rate,
				      unsigned long parent_rate)
{
	/*
	 * We must report success but we can do so unconditionally because
	 * clk_factor_round_rate returns values that ensure this call is a
	 * nop.
	 */
	return 0;
};

static const struct clk_ops rzv2h_cpg_pll_div_ops = {
	.recalc_rate = rzv2h_cpg_pll_div_recalc_rate,
	.determine_rate = rzv2h_cpg_pll_div_determine_rate,
	.set_rate = rzv2h_cpg_pll_div_set_rate,
};

static struct clk * __init
rzv2h_cpg_pll_div_clk_register(const struct cpg_core_clk *core,
			       struct clk **clks,
			       struct rzv2h_cpg_priv *priv)
{
	struct rzv2h_pll_div_hw_data *clk_hw_data;
	const struct clk *parent;
	const char *parent_name;
	struct clk_init_data init;
	struct clk_hw *clk_hw;
	int ret;

	parent = clks[core->parent & 0xffff];
	if (IS_ERR(parent))
		return ERR_CAST(parent);

	clk_hw_data = devm_kzalloc(priv->dev, sizeof(*clk_hw_data), GFP_KERNEL);
	if (!clk_hw_data)
		return ERR_PTR(-ENOMEM);

	clk_hw_data->priv = priv;
	clk_hw_data->div = core->div;
	clk_hw_data->mult = core->mult;

	parent_name = __clk_get_name(parent);
	init.name = core->name;
	init.ops = &rzv2h_cpg_pll_div_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clk_hw = &clk_hw_data->hw;
	clk_hw->init = &init;

	ret = devm_clk_hw_register(priv->dev, clk_hw);
	if (ret)
		return ERR_PTR(ret);

	return clk_hw->clk;
}


static const struct clk_ops rzv2h_cpg_pll_ops = {
	.recalc_rate = rzv2h_cpg_pll_clk_recalc_rate,
};

static int rzv2h_cpg_plldsi_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	return 0;
}

static int rzv2h_cpg_plldsi_set_rate(struct clk_hw *hw,
				     unsigned long rate,
				     unsigned long parent_rate)
{
	struct pll_clk *pll_clk = to_pll(hw);
	struct rzv2h_cpg_priv *priv = pll_clk->priv;
	u32 pll_m, pll_p, pll_s, val;
	int pll_k;
	u32 conf = pll_clk->conf;
	unsigned long fvco, osc;
	int ret;

	if (rate > (pll_clk->max * MEGA))
		rate = pll_clk->max;
	else if (rate < (pll_clk->min * MEGA))
		rate = pll_clk->min;

	osc = EXTAL_FREQ_IN_MEGA_HZ * MEGA;
	for (pll_s = PLL_DIV_S_MIN; pll_s <= PLL_DIV_S_MAX; pll_s++) {
		/* Check available range of FVCO */
		fvco = rate << (1 * pll_s);
		if ((fvco > PLL_FVCO_MAX) || (fvco < PLL_FVCO_MIN))
			continue;

		for (pll_p = PLL_DIV_P_MIN; pll_p <= PLL_DIV_P_MAX; pll_p++) {
			pll_m = ((u64) (fvco * pll_p) / osc);
			pll_k = ((u64)(fvco * pll_p) % osc);

			/* Check available range of DIV_K */
			if (pll_k >= (osc / 2)) {
				pll_m++;
				pll_k = pll_k - osc;
			}

			/* Check available range of DIV_M */
			if ((pll_m < PLL_DIV_M_MIN) ||
			    (pll_m > PLL_DIV_M_MAX))
				continue;

			pll_k = DIV_S64_ROUND_CLOSEST(((s64)pll_k << 16), osc);

			goto found;
		}
	}

	dev_err(priv->dev, "failed to set %s to rate %lu\n",
		clk_hw_get_name(hw), rate);
	return -EINVAL;

found:
	dev_dbg(priv->dev,
		"rate: %ld pll_k: %hd, pll_m: %d, pll_p: %d, pll_s: %d\n",
		rate, pll_k, pll_m, pll_p, pll_s);

	/* Put PLL into standby mode and wait until unlocked */
	writel(PLL_RESETB_WEN, priv->base + PLL_STBY_OFFSET(conf));
	ret = readl_poll_timeout(priv->base + PLL_MON_OFFSET(conf),
				 val, !(val & PLL_MON_LOCK),
				 100, 250000);
	if (ret) {
		dev_err(priv->dev, "failed to put PLLDSI to stanby mode");
		return ret;
	}

	/* Output clock setting 1 */
	writel(((s16)pll_k << 16) | (pll_m << 6) | (pll_p),
	       priv->base + PLL_CLK1_OFFSET(conf));

	/* Output clock setting 2 */
	val = readl(priv->base + PLL_CLK2_OFFSET(conf));
	writel((val & ~GENMASK(2, 0)) | pll_s,
		priv->base + PLL_CLK2_OFFSET(conf));

	/* Put PLL to normal mode and disable SSC */
	writel(PLL_RESETB | PLL_RESETB_WEN | PLL_SSC_EN_WEN,
	       priv->base + PLL_STBY_OFFSET(conf));

	/* PLL normal mode transition, output clock stability check */
	ret = readl_poll_timeout(priv->base + PLL_MON_OFFSET(conf),
				 val, (val & PLL_MON_LOCK),
				 100, 250000);
	if (ret) {
		dev_err(priv->dev, "failed to put PLLDSI to normal mode");
		return ret;
	}

	return 0;
};

static const struct clk_ops rzv2h_cpg_plldsi_ops = {
	.recalc_rate = rzv2h_cpg_pll_clk_recalc_rate,
	.determine_rate = rzv2h_cpg_plldsi_determine_rate,
	.set_rate = rzv2h_cpg_plldsi_set_rate,
};

static struct clk * __init
rzv2h_cpg_pll_clk_register(const struct cpg_core_clk *core,
			   struct rzv2h_cpg_priv *priv,
			   const struct clk_ops *ops)
{
	void __iomem *base = priv->base;
	struct device *dev = priv->dev;
	struct clk_init_data init;
	const struct clk *parent;
	const char *parent_name;
	struct pll_clk *pll_clk;
	int ret, val;

	/* Put PLL in normal mode if it is in standby mode*/
	val = readl(priv->base + PLL_MON_OFFSET(core->cfg.conf));
	if (val != (PLL_MON_LOCK | PLL_MON_RESETB)) {
		/* Put PLL to normal mode */
		writel(PLL_STBY_RESETB | PLL_STBY_RESETB_WEN,
					priv->base + PLL_STBY_OFFSET(core->cfg.conf));

		/* PLL normal mode transition, output clock stability check */
		ret = readl_poll_timeout(priv->base + PLL_MON_OFFSET(core->cfg.conf),
						val, (val & (PLL_MON_LOCK | PLL_MON_RESETB)),
						100, 250000);

		if (ret) {
			dev_err(priv->dev, "failed to put %s PLL clock normal mode", core->name);
			return ERR_PTR(ret);
		}
	}

	parent = priv->clks[core->parent];
	if (IS_ERR(parent))
		return ERR_CAST(parent);

	pll_clk = devm_kzalloc(dev, sizeof(*pll_clk), GFP_KERNEL);
	if (!pll_clk)
		return ERR_PTR(-ENOMEM);

	parent_name = __clk_get_name(parent);
	init.name = core->name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll_clk->hw.init = &init;
	pll_clk->conf = core->cfg.pll.conf;
	pll_clk->min = core->cfg.pll.min;
	pll_clk->max = core->cfg.pll.max;
	pll_clk->base = base;
	pll_clk->priv = priv;
	pll_clk->type = core->type;

	if (pll_clk->max < pll_clk->min)
		return ERR_PTR(-EINVAL);

	ret = devm_clk_hw_register(dev, &pll_clk->hw);
	if (ret)
		return ERR_PTR(ret);

	return pll_clk->hw.clk;
}

static unsigned long rzv2h_sddiv_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	unsigned int val;

	val = readl(divider->reg) >> divider->shift;
	val &= clk_div_mask(divider->width);

	return divider_recalc_rate(hw, parent_rate, val, divider->table,
				   divider->flags, divider->width);
}

static long rzv2h_sddiv_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *prate)
{
	struct clk_divider *divider = to_clk_divider(hw);

	return divider_round_rate(hw, rate, prate, divider->table,
				  divider->width, divider->flags);
}

static int rzv2h_sddiv_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	struct clk_divider *divider = to_clk_divider(hw);

	return divider_determine_rate(hw, req, divider->table, divider->width,
				      divider->flags);
}

static inline int rzv2h_cpg_wait_ddiv_clk_update_done(void __iomem *base, u8 mon)
{
	u32 bitmask = BIT(mon);
	u32 val;

	return readl_poll_timeout_atomic(base + CPG_CLKSTATUS0, val,
					!(val & bitmask), 10, 200);
}

static int rzv2h_sddiv_set_rate(struct clk_hw *hw, unsigned long rate,
			        unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	struct sddiv_clk *sddiv = to_sddiv_clock(divider);
	struct rzv2h_cpg_priv *priv = sddiv->priv;
	unsigned long flags = 0;
	int value;
	u32 val;
	int ret;

	value = divider_get_val(rate, parent_rate, divider->table,
				divider->width, divider->flags);
	if (value < 0)
		return value;


	spin_lock_irqsave(divider->lock, flags);

	/* Only ddiv support monitor register */
	if (sddiv->mon >= 0) {
		ret = rzv2h_cpg_wait_ddiv_clk_update_done(priv->base,
							  sddiv->mon);
		if (ret)
			goto ddiv_timeout;
	}

	val = readl(divider->reg) | DDIV_DIVCTL_WEN(divider->shift);
	val &= ~(clk_div_mask(divider->width) << divider->shift);
	val |= (u32)value << divider->shift;
	writel(val, divider->reg);

	/* Only ddiv support monitor register */
	if (sddiv->mon >= 0) {
		ret = rzv2h_cpg_wait_ddiv_clk_update_done(priv->base,
							  sddiv->mon);
		if (ret)
			goto ddiv_timeout;
	}

	spin_unlock_irqrestore(divider->lock, flags);

	return 0;

ddiv_timeout:
	spin_unlock_irqrestore(divider->lock, flags);
	return ret;
}

static const struct clk_ops rzv2h_sddiv_clk_divider_ops = {
	.recalc_rate = rzv2h_sddiv_recalc_rate,
	.round_rate = rzv2h_sddiv_round_rate,
	.determine_rate = rzv2h_sddiv_determine_rate,
	.set_rate = rzv2h_sddiv_set_rate,
};

static int rzv2h_plldsi_sdiv_determine_rate(struct clk_hw *hw,
					    struct clk_rate_request *req)
{
	if (req->rate > MAX_VCLK_FREQ)
		req->rate = MAX_VCLK_FREQ;
	else if (req->rate < MIN_VCLK_FREQ)
		req->rate = MIN_VCLK_FREQ;

	if (req->rate < LIMIT_VCLK_FREQ)
		req->best_parent_rate = req->rate * 6;
	else
		req->best_parent_rate = req->rate * 2;

	return 0;
};

static const struct clk_ops rzv2h_plldsi_sdiv_ops = {
	.recalc_rate = rzv2h_sddiv_recalc_rate,
	.determine_rate = rzv2h_plldsi_sdiv_determine_rate,
	.set_rate = rzv2h_sddiv_set_rate,
};

static struct clk * __init
rzv2h_cpg_sddiv_clk_register(const struct cpg_core_clk *core,
			     struct rzv2h_cpg_priv *priv,
			     const struct clk_ops *ops)
{
	struct sddiv cfg_sddiv = core->cfg.sddiv;
	struct clk_init_data init = {};
	struct device *dev = priv->dev;
	u8 shift = cfg_sddiv.shift;
	u8 width = cfg_sddiv.width;
	const struct clk *parent;
	const char *parent_name;
	struct clk_divider *div;
	struct sddiv_clk *sddiv;
	int ret;

	parent = priv->clks[core->parent];
	if (IS_ERR(parent))
		return ERR_CAST(parent);

	parent_name = __clk_get_name(parent);

	if ((shift + width) > 16)
		return ERR_PTR(-EINVAL);

	sddiv = devm_kzalloc(priv->dev, sizeof(*sddiv), GFP_KERNEL);
	if (!sddiv)
		return ERR_PTR(-ENOMEM);

	init.name = core->name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	if (core->type == CLK_TYPE_PLLDSI_SDIV)
		init.flags = CLK_SET_RATE_PARENT;

	sddiv->priv = priv;
	sddiv->mon = cfg_sddiv.monbit;
	div = &sddiv->div;
	div->reg = priv->base + cfg_sddiv.offset;
	div->shift = shift;
	div->width = width;
	div->flags = core->flag;
	div->lock = &priv->rmw_lock;
	div->hw.init = &init;
	div->table = core->dtable;

	ret = devm_clk_hw_register(dev, &div->hw);
	if (ret)
		return ERR_PTR(ret);

	return div->hw.clk;
}

static struct clk * __init
rzv2h_cpg_mux_clk_register(const struct cpg_core_clk *core,
			   void __iomem *base,
			   struct rzv2h_cpg_priv *priv)
{
	const struct clk_hw *clk_hw;
	struct mux cfg_mux = core->cfg.mux;
	u32 offset = cfg_mux.offset;
	u8 shift = cfg_mux.shift;
	u8 width = cfg_mux.width;

	clk_hw = devm_clk_hw_register_mux(priv->dev, core->name,
					  cfg_mux.parent_names,
					  cfg_mux.num_parents,
					  core->flag,
					  base + offset, shift, width,
					  cfg_mux.mux_flags, &priv->rmw_lock);
	if (IS_ERR(clk_hw))
		return ERR_CAST(clk_hw);

	return clk_hw->clk;
}

static struct clk
*rzv2h_cpg_clk_src_twocell_get(struct of_phandle_args *clkspec,
			       void *data)
{
	unsigned int clkidx = clkspec->args[1];
	struct rzv2h_cpg_priv *priv = data;
	struct device *dev = priv->dev;
	const char *type;
	struct clk *clk;

	switch (clkspec->args[0]) {
	case CPG_CORE:
		type = "core";
		if (clkidx > priv->last_dt_core_clk) {
			dev_err(dev, "Invalid %s clock index %u\n", type, clkidx);
			return ERR_PTR(-EINVAL);
		}
		clk = priv->clks[clkidx];
		break;

	case CPG_MOD:
		type = "module";
		if (clkidx >= priv->num_mod_clks) {
			dev_err(dev, "Invalid %s clock index %u\n", type, clkidx);
			return ERR_PTR(-EINVAL);
		}
		clk = priv->clks[priv->num_core_clks + clkidx];
		break;

	default:
		dev_err(dev, "Invalid CPG clock type %u\n", clkspec->args[0]);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR(clk))
		dev_err(dev, "Cannot get %s clock %u: %ld", type, clkidx,
			PTR_ERR(clk));
	else
		dev_dbg(dev, "clock (%u, %u) is %pC at %lu Hz\n",
			clkspec->args[0], clkspec->args[1], clk,
			clk_get_rate(clk));
	return clk;
}

static void __init
rzv2h_cpg_register_core_clk(const struct cpg_core_clk *core,
			    struct rzv2h_cpg_priv *priv)
{
	struct clk *clk = ERR_PTR(-EOPNOTSUPP), *parent;
	unsigned int id = core->id, div = core->div;
	struct device *dev = priv->dev;
	const char *parent_name;
	struct clk_hw *clk_hw;
	const struct clk_ops *ops;

	WARN_DEBUG(id >= priv->num_core_clks);
	WARN_DEBUG(PTR_ERR(priv->clks[id]) != -ENOENT);

	switch (core->type) {
	case CLK_TYPE_IN:
		clk = of_clk_get_by_name(priv->dev->of_node, core->name);
		break;
	case CLK_TYPE_FF:
		WARN_DEBUG(core->parent >= priv->num_core_clks);
		parent = priv->clks[core->parent];
		if (IS_ERR(parent)) {
			clk = parent;
			goto fail;
		}

		parent_name = __clk_get_name(parent);
		clk_hw = devm_clk_hw_register_fixed_factor(dev, core->name,
							   parent_name, CLK_SET_RATE_PARENT,
							   core->mult, div);
		if (IS_ERR(clk_hw))
			clk = ERR_CAST(clk_hw);
		else
			clk = clk_hw->clk;
		break;
	case CLK_TYPE_PLL:
	case CLK_TYPE_PLLDSI:
		ops = (core->type == CLK_TYPE_PLLDSI) ?
		       &rzv2h_cpg_plldsi_ops : &rzv2h_cpg_pll_ops;
		clk = rzv2h_cpg_pll_clk_register(core, priv, ops);
		break;
	case CLK_TYPE_SDIV:
	case CLK_TYPE_DDIV:
	case CLK_TYPE_PLLDSI_SDIV:
		ops = (core->type == CLK_TYPE_PLLDSI_SDIV) ?
		       &rzv2h_plldsi_sdiv_ops :
		       &rzv2h_sddiv_clk_divider_ops;
		clk = rzv2h_cpg_sddiv_clk_register(core, priv, ops);
		break;
	case CLK_TYPE_MUX:
		clk = rzv2h_cpg_mux_clk_register(core, priv->base, priv);
		break;
	case CLK_TYPE_PLL_DIV:
		clk = rzv2h_cpg_pll_div_clk_register(core, priv->clks, priv);
		break;
	default:
		goto fail;
	}

	if (IS_ERR_OR_NULL(clk))
		goto fail;

	dev_dbg(dev, "Core clock %pC at %lu Hz\n", clk, clk_get_rate(clk));
	priv->clks[id] = clk;
	return;

fail:
	dev_err(dev, "Failed to register core clock %s: %ld\n",
		core->name, PTR_ERR(clk));
}

static int rzv2h_mod_clock_endisable(struct clk_hw *hw, bool enable)
{
	struct mod_clock *clock = to_mod_clock(hw);
	unsigned int reg = GET_CLK_ON_OFFSET(clock->on_index);
	struct rzv2h_cpg_priv *priv = clock->priv;
	u32 bitmask = BIT(clock->on_bit);
	struct device *dev = priv->dev;
	u32 value;
	int error;

	dev_dbg(dev, "CLK_ON 0x%x/%pC %s\n", reg, hw->clk,
		enable ? "ON" : "OFF");

	value = bitmask << 16;
	if (enable)
		value |= bitmask;

	writel(value, priv->base + reg);

	if (!enable || clock->mon_index < 0)
		return 0;

	reg = GET_CLK_MON_OFFSET(clock->mon_index);
	bitmask = BIT(clock->mon_bit);
	error = readl_poll_timeout_atomic(priv->base + reg, value,
					  value & bitmask, 0, 10);
	if (error)
		dev_err(dev, "Failed to enable CLK_ON %p\n",
			priv->base + reg);

	return error;
}

static int rzv2h_mod_clock_enable(struct clk_hw *hw)
{
	return rzv2h_mod_clock_endisable(hw, true);
}

static void rzv2h_mod_clock_disable(struct clk_hw *hw)
{
	rzv2h_mod_clock_endisable(hw, false);
}

static int rzv2h_mod_clock_is_enabled(struct clk_hw *hw)
{
	struct mod_clock *clock = to_mod_clock(hw);
	struct rzv2h_cpg_priv *priv = clock->priv;
	u32 bitmask;
	u32 offset;

	if (clock->mon_index >= 0) {
		offset = GET_CLK_MON_OFFSET(clock->mon_index);
		bitmask = BIT(clock->mon_bit);
	} else {
		offset = GET_CLK_ON_OFFSET(clock->on_index);
		bitmask = BIT(clock->on_bit);
	}

	return readl(priv->base + offset) & bitmask;
}

static const struct clk_ops rzv2h_mod_clock_ops = {
	.enable = rzv2h_mod_clock_enable,
	.disable = rzv2h_mod_clock_disable,
	.is_enabled = rzv2h_mod_clock_is_enabled,
};

static void __init
rzv2h_cpg_register_mod_clk(const struct rzv2h_mod_clk *mod,
			   struct rzv2h_cpg_priv *priv)
{
	struct mod_clock *clock = NULL;
	struct device *dev = priv->dev;
	struct clk_init_data init;
	struct clk *parent, *clk;
	const char *parent_name;
	unsigned int id;
	int ret;

	id = GET_MOD_CLK_ID(priv->num_core_clks, mod->on_index, mod->on_bit);
	WARN_DEBUG(id >= priv->num_core_clks + priv->num_mod_clks);
	WARN_DEBUG(mod->parent >= priv->num_core_clks + priv->num_mod_clks);
	WARN_DEBUG(PTR_ERR(priv->clks[id]) != -ENOENT);

	parent = priv->clks[mod->parent];
	if (IS_ERR(parent)) {
		clk = parent;
		goto fail;
	}

	clock = devm_kzalloc(dev, sizeof(*clock), GFP_KERNEL);
	if (!clock) {
		clk = ERR_PTR(-ENOMEM);
		goto fail;
	}

	init.name = mod->name;
	init.ops = &rzv2h_mod_clock_ops;
	init.flags = CLK_SET_RATE_PARENT;
	if (mod->critical)
		init.flags |= CLK_IS_CRITICAL;

	parent_name = __clk_get_name(parent);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clock->on_index = mod->on_index;
	clock->on_bit = mod->on_bit;
	clock->mon_index = mod->mon_index;
	clock->mon_bit = mod->mon_bit;
	clock->priv = priv;
	clock->hw.init = &init;

	ret = devm_clk_hw_register(dev, &clock->hw);
	if (ret) {
		clk = ERR_PTR(ret);
		goto fail;
	}

	priv->clks[id] = clock->hw.clk;

	return;

fail:
	dev_err(dev, "Failed to register module clock %s: %ld\n",
		mod->name, PTR_ERR(clk));
}

static int rzv2h_cpg_assert(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct rzv2h_cpg_priv *priv = rcdev_to_priv(rcdev);
	unsigned int reg = GET_RST_OFFSET(priv->resets[id].reset_index);
	u32 mask = BIT(priv->resets[id].reset_bit);
	u8 monbit = priv->resets[id].mon_bit;
	u32 value = mask << 16;

	dev_dbg(rcdev->dev, "assert id:%ld offset:0x%x\n", id, reg);

	writel(value, priv->base + reg);

	reg = GET_RST_MON_OFFSET(priv->resets[id].mon_index);
	mask = BIT(monbit);

	return readl_poll_timeout_atomic(priv->base + reg, value,
					 value & mask, 10, 200);
}

static int rzv2h_cpg_deassert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct rzv2h_cpg_priv *priv = rcdev_to_priv(rcdev);
	unsigned int reg = GET_RST_OFFSET(priv->resets[id].reset_index);
	u32 mask = BIT(priv->resets[id].reset_bit);
	u8 monbit = priv->resets[id].mon_bit;
	u32 value = (mask << 16) | mask;

	dev_dbg(rcdev->dev, "deassert id:%ld offset:0x%x\n", id, reg);

	writel(value, priv->base + reg);

	reg = GET_RST_MON_OFFSET(priv->resets[id].mon_index);
	mask = BIT(monbit);

	return readl_poll_timeout_atomic(priv->base + reg, value,
					 !(value & mask), 10, 200);
}

static int rzv2h_cpg_reset(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	int ret;

	ret = rzv2h_cpg_assert(rcdev, id);
	if (ret)
		return ret;

	return rzv2h_cpg_deassert(rcdev, id);
}

static int rzv2h_cpg_status(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct rzv2h_cpg_priv *priv = rcdev_to_priv(rcdev);
	unsigned int reg = GET_RST_MON_OFFSET(priv->resets[id].mon_index);
	u8 monbit = priv->resets[id].mon_bit;

	return !!(readl(priv->base + reg) & BIT(monbit));
}

static const struct reset_control_ops rzv2h_cpg_reset_ops = {
	.reset = rzv2h_cpg_reset,
	.assert = rzv2h_cpg_assert,
	.deassert = rzv2h_cpg_deassert,
	.status = rzv2h_cpg_status,
};

static int rzv2h_cpg_reset_xlate(struct reset_controller_dev *rcdev,
				 const struct of_phandle_args *reset_spec)
{
	struct rzv2h_cpg_priv *priv = rcdev_to_priv(rcdev);
	unsigned int id = reset_spec->args[0];
	u8 rst_index = id / 16;
	u8 rst_bit = id % 16;
	unsigned int i;

	for (i = 0; i < rcdev->nr_resets; i++) {
		if (rst_index == priv->resets[i].reset_index &&
		    rst_bit == priv->resets[i].reset_bit)
			return i;
	}

	return -EINVAL;
}

static int rzv2h_cpg_reset_controller_register(struct rzv2h_cpg_priv *priv)
{
	priv->rcdev.ops = &rzv2h_cpg_reset_ops;
	priv->rcdev.of_node = priv->dev->of_node;
	priv->rcdev.dev = priv->dev;
	priv->rcdev.of_reset_n_cells = 1;
	priv->rcdev.of_xlate = rzv2h_cpg_reset_xlate;
	priv->rcdev.nr_resets = priv->num_resets;

	return devm_reset_controller_register(priv->dev, &priv->rcdev);
}

/**
 * struct rzv2h_cpg_pd - RZ/V2H power domain data structure
 * @priv: pointer to CPG private data structure
 * @genpd: generic PM domain
 */
struct rzv2h_cpg_pd {
	struct rzv2h_cpg_priv *priv;
	struct generic_pm_domain genpd;
};

static int rzv2h_cpg_attach_dev(struct generic_pm_domain *domain, struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct of_phandle_args clkspec;
	bool once = true;
	struct clk *clk;
	int error;
	int i = 0;

	while (!of_parse_phandle_with_args(np, "clocks", "#clock-cells", i,
					   &clkspec)) {
		if (once) {
			once = false;
			error = pm_clk_create(dev);
			if (error) {
				of_node_put(clkspec.np);
				goto err;
			}
		}
		clk = of_clk_get_from_provider(&clkspec);
		of_node_put(clkspec.np);
		if (IS_ERR(clk)) {
			error = PTR_ERR(clk);
			goto fail_destroy;
		}

		error = pm_clk_add_clk(dev, clk);
		if (error) {
			dev_err(dev, "pm_clk_add_clk failed %d\n",
				error);
			goto fail_put;
		}
		i++;
	}

	return 0;

fail_put:
	clk_put(clk);

fail_destroy:
	pm_clk_destroy(dev);
err:
	return error;
}

static void rzv2h_cpg_detach_dev(struct generic_pm_domain *unused, struct device *dev)
{
	if (!pm_clk_no_clocks(dev))
		pm_clk_destroy(dev);
}

static void rzv2h_cpg_genpd_remove_simple(void *data)
{
	pm_genpd_remove(data);
}

static int __init rzv2h_cpg_add_pm_domains(struct rzv2h_cpg_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct rzv2h_cpg_pd *pd;
	int ret;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->genpd.name = np->name;
	pd->priv = priv;
	pd->genpd.flags |= GENPD_FLAG_ALWAYS_ON | GENPD_FLAG_PM_CLK | GENPD_FLAG_ACTIVE_WAKEUP;
	pd->genpd.attach_dev = rzv2h_cpg_attach_dev;
	pd->genpd.detach_dev = rzv2h_cpg_detach_dev;
	ret = pm_genpd_init(&pd->genpd, &pm_domain_always_on_gov, false);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, rzv2h_cpg_genpd_remove_simple, &pd->genpd);
	if (ret)
		return ret;

	return of_genpd_add_provider_simple(np, &pd->genpd);
}

static void rzv2h_cpg_del_clk_provider(void *data)
{
	of_clk_del_provider(data);
}

static int __init rzv2h_cpg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct rzv2h_cpg_info *info;
	struct rzv2h_cpg_priv *priv;
	unsigned int nclks, i;
	struct clk **clks;
	int error;

	info = of_device_get_match_data(dev);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->rmw_lock);

	priv->dev = dev;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	nclks = info->num_total_core_clks + info->num_hw_mod_clks;
	clks = devm_kmalloc_array(dev, nclks, sizeof(*clks), GFP_KERNEL);
	if (!clks)
		return -ENOMEM;

	priv->resets = devm_kmemdup(dev, info->resets, sizeof(*info->resets) *
				    info->num_resets, GFP_KERNEL);
	if (!priv->resets)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->clks = clks;
	priv->num_core_clks = info->num_total_core_clks;
	priv->num_mod_clks = info->num_hw_mod_clks;
	priv->last_dt_core_clk = info->last_dt_core_clk;
	priv->num_resets = info->num_resets;

	for (i = 0; i < nclks; i++)
		clks[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < info->num_core_clks; i++)
		rzv2h_cpg_register_core_clk(&info->core_clks[i], priv);

	for (i = 0; i < info->num_mod_clks; i++)
		rzv2h_cpg_register_mod_clk(&info->mod_clks[i], priv);

	error = of_clk_add_provider(np, rzv2h_cpg_clk_src_twocell_get, priv);
	if (error)
		return error;

	error = devm_add_action_or_reset(dev, rzv2h_cpg_del_clk_provider, np);
	if (error)
		return error;

	error = rzv2h_cpg_add_pm_domains(priv);
	if (error)
		return error;

	error = rzv2h_cpg_reset_controller_register(priv);
	if (error)
		return error;

	return 0;
}

static const struct of_device_id rzv2h_cpg_match[] = {
#ifdef CONFIG_CLK_R9A09G057
	{
		.compatible = "renesas,r9a09g057-cpg",
		.data = &r9a09g057_cpg_info,
	},
#endif
#ifdef CONFIG_CLK_R9A09G047
	{
		.compatible = "renesas,r9a09g047-cpg",
		.data = &r9a09g047_cpg_info,
	},
#endif
	{ /* sentinel */ }
};

static struct platform_driver rzv2h_cpg_driver = {
	.driver		= {
		.name	= "rzv2h-cpg",
		.of_match_table = rzv2h_cpg_match,
	},
};

static int __init rzv2h_cpg_init(void)
{
	return platform_driver_probe(&rzv2h_cpg_driver, rzv2h_cpg_probe);
}

subsys_initcall(rzv2h_cpg_init);

MODULE_DESCRIPTION("Renesas RZ/V2H CPG Driver");
