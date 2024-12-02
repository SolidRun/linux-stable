// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G3E LVDS Core Driver
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "rzg3e_lvds.h"
#include "rzg3e_lvds_regs.h"

struct drm_bridge;

#define MAX_LVDS_CHANNELS	2

struct rzg3e_lvds {
	struct device *dev;

	struct reset_control *rstc;
	struct clk *pclk;
	void __iomem *mmio;

	struct clk *dclk[MAX_LVDS_CHANNELS];
};

void rzg3e_lvds_write(struct device *dev, u32 reg, u32 data)
{
	struct rzg3e_lvds *lvds = dev_get_drvdata(dev);

	iowrite32(data, lvds->mmio + reg);
}

int rzg3e_lvds_reset_deassert_pclk_enable(struct device *dev)
{
	struct rzg3e_lvds *lvds = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(lvds->rstc);
	if (ret < 0) {
		reset_control_assert(lvds->rstc);
		dev_err(lvds->dev, "deassert error");
		return ret;
	}

	return clk_prepare_enable(lvds->pclk);
}

void rzg3e_lvds_reset_assert_pclk_disable(struct device *dev)
{
	struct rzg3e_lvds *lvds = dev_get_drvdata(dev);

	clk_disable_unprepare(lvds->pclk);
	reset_control_assert(lvds->rstc);
}

void rzg3e_lvds_set_lvds_ch_dot_clk(struct device *dev, u32 id, struct clk *clk)
{
	struct rzg3e_lvds *lvds = dev_get_drvdata(dev);

	lvds->dclk[id] = clk;
}

u32 rzg3e_lvds_single_link_mode(struct device *dev, u32 id)
{
	struct rzg3e_lvds *lvds = dev_get_drvdata(dev);
	u32 mode;

	if (lvds->dclk[0] && lvds->dclk[1]) {
		if (lvds->dclk[0] == lvds->dclk[1])
			mode = LVDS_SINGLE_LINK_MULTI;
		else
			mode = LVDS_SINGLE_LINK_DUAL;
	} else {
		if (id == 0 && lvds->dclk[0])
			mode = LVDS_SINGLE_LINK_CH0;
		else
			mode = LVDS_SINGLE_LINK_CH1;
	}

	return mode;
}

bool rzg3e_lvds_dual_link(struct drm_bridge *bridge)
{
	return rzg3e_lvds_link_dual_link(bridge);
}
EXPORT_SYMBOL_GPL(rzg3e_lvds_dual_link);

bool rzg3e_lvds_is_connected(struct drm_bridge *bridge)
{
	return rzg3e_lvds_link_is_connected(bridge);
}
EXPORT_SYMBOL_GPL(rzg3e_lvds_is_connected);

/* -----------------------------------------------------------------------------
 * Probe
 */
static int rzg3e_lvds_probe(struct platform_device *pdev)
{
	struct rzg3e_lvds *lvds;

	lvds = devm_kzalloc(&pdev->dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	platform_set_drvdata(pdev, lvds);

	lvds->dev = &pdev->dev;

	lvds->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lvds->mmio))
		return PTR_ERR(lvds->mmio);

	lvds->pclk = devm_clk_get(lvds->dev, NULL);
	if (IS_ERR(lvds->pclk))
		return PTR_ERR(lvds->pclk);

	lvds->rstc = devm_reset_control_get_shared(lvds->dev, NULL);
	if (IS_ERR(lvds->rstc))
		return dev_err_probe(lvds->dev, PTR_ERR(lvds->rstc),
				     "failed to get rst\n");

	/* Spawn child devices for the LVDS ports */
	devm_of_platform_populate(lvds->dev);

	return 0;
}

static const struct of_device_id rzg3e_lvds_of_table[] = {
	{ .compatible = "renesas,rzg3e-lvds" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, rzg3e_lvds_of_table);

static struct platform_driver rzg3e_lvds_platform_driver = {
	.probe		= rzg3e_lvds_probe,
	.driver		= {
		.name	= "rzg3e-lvds",
		.of_match_table = rzg3e_lvds_of_table,
	},
};

module_platform_driver(rzg3e_lvds_platform_driver);

MODULE_AUTHOR("Biju Das <biju.das.jz@bp.renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/G3E LVDS Core Driver");
MODULE_LICENSE("GPL");
