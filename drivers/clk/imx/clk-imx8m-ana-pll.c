/*
 * Copyright 2018 SolidRun Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>

#include "clk.h"

#define IMX8MQ_ANA_PLLOUT_REG                   0x74
#define IMX8MQ_ANA_PLLOUT_CKE                   BIT(4)
#define IMX8MQ_ANA_PLLOUT_SEL_MASK              0xF
#define IMX8MQ_ANA_PLLOUT_SEL_SYSPLL1           0xB
#define IMX8MQ_ANA_PLLOUT_DIV_REG               0x7C
#define IMX8MQ_ANA_PLLOUT_SYSPLL1_DIV           0x7

struct clk_ana_imx8mq {
	struct clk_hw	hw;
	void __iomem	*base;
	int		prepared;
        unsigned int    mult;
        unsigned int    div;
};

#define to_clk_ana_imx8mq(_hw) container_of(_hw, struct clk_ana_imx8mq, hw)

static int clk_ana_imx8mq_is_prepared(struct clk_hw *hw)
{
	struct clk_ana_imx8mq *ana = to_clk_ana_imx8mq(hw);

	return ana->prepared;
}

static int clk_ana_imx8mq_prepare(struct clk_hw *hw)
{
	struct clk_ana_imx8mq *ana = to_clk_ana_imx8mq(hw);
	u32 val;

	if (ana->prepared > 0) {
		ana->prepared++;
		return 0;
	}

	val = readl_relaxed(ana->base + IMX8MQ_ANA_PLLOUT_REG);
	val &= ~(IMX8MQ_ANA_PLLOUT_SEL_MASK);
	val |= IMX8MQ_ANA_PLLOUT_SEL_SYSPLL1;
	writel_relaxed(val, ana->base + IMX8MQ_ANA_PLLOUT_REG);

	val = readl_relaxed(ana->base + IMX8MQ_ANA_PLLOUT_DIV_REG);
	val |= IMX8MQ_ANA_PLLOUT_SYSPLL1_DIV;	
	writel_relaxed(val, ana->base + IMX8MQ_ANA_PLLOUT_DIV_REG);

	ana->prepared++;

	return 0;
}

static void clk_ana_imx8mq_unprepare(struct clk_hw *hw)
{
	struct clk_ana_imx8mq *ana = to_clk_ana_imx8mq(hw);

	ana->prepared--;
}

static unsigned long clk_ana_imx8mq_recalc_rate(struct clk_hw *hw,
                unsigned long parent_rate)
{
        struct clk_ana_imx8mq *ana = to_clk_ana_imx8mq(hw);
        unsigned long long int rate;

        rate = (unsigned long long int)parent_rate * ana->mult;
        do_div(rate, ana->div);
        return (unsigned long)rate;
}

static long clk_ana_imx8mq_round_rate(struct clk_hw *hw, unsigned long rate,
                                unsigned long *prate)
{
        struct clk_ana_imx8mq *ana = to_clk_ana_imx8mq(hw);

        return (*prate / ana->div) * ana->mult;
}

static int clk_ana_imx8mq_set_rate(struct clk_hw *hw, unsigned long rate,
                                unsigned long parent_rate)
{
        return 0;
}

static const struct clk_ops clk_ana_imx8mq_ops = {
	.prepare	= clk_ana_imx8mq_prepare,
	.unprepare	= clk_ana_imx8mq_unprepare,
	.is_prepared	= clk_ana_imx8mq_is_prepared,
	.round_rate	= clk_ana_imx8mq_round_rate,
        .set_rate	= clk_ana_imx8mq_set_rate,
        .recalc_rate	= clk_ana_imx8mq_recalc_rate,
};

struct clk *imx_clk_ana_imx8mq_100_fixed(const char *name, const char *parent_name, void __iomem *base)
{
	struct clk_ana_imx8mq *ana;
	struct clk *clk;
	struct clk_init_data init;

	ana = kzalloc(sizeof(*ana), GFP_KERNEL);
	if (!ana)
		return ERR_PTR(-ENOMEM);

	ana->base = base;
	ana->mult = 1;
	ana->div = 8;
	init.name = name;
	init.ops = &clk_ana_imx8mq_ops;
	init.flags = CLK_IS_BASIC;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	ana->hw.init = &init;

	clk = clk_register(NULL, &ana->hw);
	if (IS_ERR(clk))
		kfree(ana);

	return clk;
}
