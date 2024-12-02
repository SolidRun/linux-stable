// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G3E LVDS Encoder
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "rzg3e_lvds_regs.h"

struct rzg3e_lvds;

#define MAX_LVDS_CHAN_NUM 2

enum rzg3e_lvds_mode {
	RZG3E_LVDS_MODE_JEIDA = 0,
	RZG3E_LVDS_MODE_JEIDA_MIRROR = 1,
	RZG3E_LVDS_MODE_MODE2 = 2,
	RZG3E_LVDS_MODE_MODE2_MIRROR = 3,
	RZG3E_LVDS_MODE_VESA = 4,
	RZG3E_LVDS_MODE_VESA_MIRROR = 5,
	RZG3E_LVDS_MODE_MODE6 = 6,
	RZG3E_LVDS_MODE_MODE6_MIRROR = 7,
};

enum rzg3e_lvds_link_type {
	RZG3E_LVDS_SINGLE_LINK = 0,
	RZG3E_LVDS_DUAL_LINK_EVEN_ODD_PIXELS = 1,
	RZG3E_LVDS_DUAL_LINK_ODD_EVEN_PIXELS = 2,
};

struct rzg3e_lvds {
	struct device *dev;

	struct drm_bridge bridge;

	struct drm_bridge *next_bridge;
	struct drm_panel *panel;

	void __iomem *mmio;
	struct {
		struct clk *phyclk;		/* Phy clock */
		struct clk *dotclk;		/* DU dot clocks */
	} clocks;

	struct drm_bridge *companion;
	enum rzg3e_lvds_link_type link_type;
	u32 ch, mode;
};

#define bridge_to_rzg3e_lvds(b) \
	container_of(b, struct rzg3e_lvds, bridge)

static u32 rzg3e_lvds_link_read(struct rzg3e_lvds *lvds, u32 reg)
{
	return ioread32(lvds->mmio + reg);
}

static void rzg3e_lvds_link_write(struct rzg3e_lvds *lvds, u32 reg, u32 data)
{
	iowrite32(data, lvds->mmio + reg);
}

static void rzg3e_lvds_link_rmw(struct rzg3e_lvds *lvds, u32 offs, u32 msk, u32 val)
{
	u32 reg_val;

	reg_val = rzg3e_lvds_link_read(lvds, offs);

	reg_val &= ~msk;
	reg_val |= (val & msk);

	rzg3e_lvds_link_write(lvds, offs, reg_val);
}

/* -----------------------------------------------------------------------------
 * Bridge
 */
static void __rzg3e_lvds_link_atomic_enable(struct drm_bridge *bridge,
					    struct drm_atomic_state *state,
					    struct drm_crtc *crtc,
					    struct drm_connector *connector)
{
	struct rzg3e_lvds *lvds = bridge_to_rzg3e_lvds(bridge);
	u32 reg_msk, reg_val;
	u32 skw_adj = 0, out_mode = RZG3E_LVDS_MODE_VESA;
	int ret;

	/* Enable the companion LVDS encoder in dual-link mode. */
	if (lvds->link_type != RZG3E_LVDS_SINGLE_LINK && lvds->companion)
		__rzg3e_lvds_link_atomic_enable(lvds->companion, state, crtc,
						connector);

	ret = clk_prepare_enable(lvds->clocks.phyclk);
	if (ret < 0) {
		dev_err(lvds->dev, "phyclk error");
		return;
	}

	ret = clk_prepare_enable(lvds->clocks.dotclk);
	if (ret < 0) {
		dev_err(lvds->dev, "dotclk error");
		return;
	}

	reg_msk = LVDS_CTL_FMT_SEL;
	reg_val = LVDS_CTL_SET(FMT_SEL, 0, out_mode);
	rzg3e_lvds_link_rmw(lvds, LVDS_CTL_OFFSET, reg_msk, reg_val);
	rzg3e_lvds_link_rmw(lvds, LVDS_PHY_OFFSET, LVDS_PHY_CH_IO_EN, 0x1F);
	reg_msk = LVDS_PHY_CH_SKW_ADJ;
	reg_val = LVDS_PHY_SET(CH_SKW_ADJ, 0, skw_adj);
	rzg3e_lvds_link_rmw(lvds, LVDS_PHY_OFFSET, reg_msk, reg_val);

	/* Wait 200us (Analog stable period is 100 us. this time count adds margin.) */
	usleep_range(200, 250);
}

static void rzg3e_lvds_link_atomic_enable(struct drm_bridge *bridge,
					  struct drm_bridge_state *old_bridge_state)
{
	struct drm_atomic_state *state = old_bridge_state->base.state;
	struct rzg3e_lvds *lvds = bridge_to_rzg3e_lvds(bridge);
	struct drm_connector *connector;
	struct drm_crtc *crtc;
	int ret;

	ret = rzg3e_lvds_reset_deassert_pclk_enable(lvds->dev->parent);
	if (ret < 0) {
		dev_err(lvds->dev, "deassert error");
		return;
	}

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	crtc = drm_atomic_get_new_connector_state(state, connector)->crtc;

	rzg3e_lvds_write(lvds->dev->parent, LVDS_CMN, LVDS_UNUSED);

	if (lvds->link_type == RZG3E_LVDS_SINGLE_LINK)
		lvds->mode = rzg3e_lvds_single_link_mode(lvds->dev->parent, lvds->ch);

	rzg3e_lvds_write(lvds->dev->parent, LVDS_CMN, lvds->mode);

	__rzg3e_lvds_link_atomic_enable(bridge, state, crtc, connector);

	rzg3e_lvds_write(lvds->dev->parent, LVDS_CMN, lvds->mode | LVDS_CMN_PHY_RESET_N);
}

static void __rzg3e_lvds_link_atomic_disable(struct drm_bridge *bridge,
					     struct drm_bridge_state *old_bridge_state)
{
	struct rzg3e_lvds *lvds = bridge_to_rzg3e_lvds(bridge);

	/* Disable the companion LVDS encoder in dual-link mode. */
	if (lvds->link_type != RZG3E_LVDS_SINGLE_LINK && lvds->companion)
		__rzg3e_lvds_link_atomic_disable(lvds->companion, old_bridge_state);

	clk_disable_unprepare(lvds->clocks.phyclk);
	clk_disable_unprepare(lvds->clocks.dotclk);
}

static void rzg3e_lvds_link_atomic_disable(struct drm_bridge *bridge,
					   struct drm_bridge_state *old_bridge_state)
{
	struct rzg3e_lvds *lvds = bridge_to_rzg3e_lvds(bridge);

	__rzg3e_lvds_link_atomic_disable(bridge, old_bridge_state);
	rzg3e_lvds_reset_assert_pclk_disable(lvds->dev->parent);
}

static bool rzg3e_lvds_link_mode_fixup(struct drm_bridge *bridge,
				       const struct drm_display_mode *mode,
				       struct drm_display_mode *adjusted_mode)
{
	adjusted_mode->clock = clamp(adjusted_mode->clock, 5400, 187500);

	return true;
}

static int rzg3e_lvds_link_attach(struct drm_bridge *bridge,
				  enum drm_bridge_attach_flags flags)
{
	struct rzg3e_lvds *lvds = bridge_to_rzg3e_lvds(bridge);

	if (!lvds->next_bridge)
		return 0;

	return drm_bridge_attach(bridge->encoder, lvds->next_bridge, bridge,
				 flags);
}

static const struct drm_bridge_funcs rzg3e_lvds_link_bridge_ops = {
	.attach = rzg3e_lvds_link_attach,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_enable = rzg3e_lvds_link_atomic_enable,
	.atomic_disable = rzg3e_lvds_link_atomic_disable,
	.mode_fixup = rzg3e_lvds_link_mode_fixup,
};

bool rzg3e_lvds_link_dual_link(struct drm_bridge *bridge)
{
	struct rzg3e_lvds *lvds = bridge_to_rzg3e_lvds(bridge);

	return lvds->link_type != RZG3E_LVDS_SINGLE_LINK;
}

bool rzg3e_lvds_link_is_connected(struct drm_bridge *bridge)
{
	struct rzg3e_lvds *lvds = bridge_to_rzg3e_lvds(bridge);

	return !!lvds->next_bridge;
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int rzg3e_lvds_link_parse_dt_companion(struct rzg3e_lvds *lvds)
{
	const struct of_device_id *match;
	struct device_node *companion;
	struct device_node *port0, *port1;
	struct rzg3e_lvds *companion_lvds;
	struct device *dev = lvds->dev;
	int dual_link;
	int ret = 0;

	/* Locate the companion LVDS encoder for dual-link operation, if any. */
	companion = of_parse_phandle(dev->of_node, "renesas,companion", 0);
	if (!companion)
		return 0;

	/*
	 * Sanity check: the companion encoder must have the same compatible
	 * string.
	 */
	match = of_match_device(dev->driver->of_match_table, dev);
	if (!of_device_is_compatible(companion, match->compatible)) {
		dev_err(dev, "Companion LVDS encoder is invalid\n");
		ret = -ENXIO;
		goto done;
	}

	/*
	 * We need to work out if the sink is expecting us to function in
	 * dual-link mode. We do this by looking at the DT port nodes we are
	 * connected to, if they are marked as expecting even pixels and
	 * odd pixels than we need to enable vertical stripe output.
	 */
	port0 = of_graph_get_port_by_id(dev->of_node, 1);
	port1 = of_graph_get_port_by_id(companion, 1);
	dual_link = drm_of_lvds_get_dual_link_pixel_order(port0, port1);
	of_node_put(port0);
	of_node_put(port1);

	switch (dual_link) {
	case DRM_LVDS_DUAL_LINK_ODD_EVEN_PIXELS:
		lvds->link_type = RZG3E_LVDS_DUAL_LINK_ODD_EVEN_PIXELS;
		break;
	case DRM_LVDS_DUAL_LINK_EVEN_ODD_PIXELS:
		lvds->link_type = RZG3E_LVDS_DUAL_LINK_EVEN_ODD_PIXELS;
		break;
	default:
		/*
		 * Early dual-link bridge specific implementations populate the
		 * timings field of drm_bridge. If the flag is set, we assume
		 * that we are expected to generate even pixels from the primary
		 * encoder, and odd pixels from the companion encoder.
		 */
		if (lvds->next_bridge->timings &&
		    lvds->next_bridge->timings->dual_link)
			lvds->link_type = RZG3E_LVDS_DUAL_LINK_EVEN_ODD_PIXELS;
	}

	lvds->mode = LVDS_DUAL_LINK;
	lvds->companion = of_drm_find_bridge(companion);
	if (!lvds->companion) {
		ret = -EPROBE_DEFER;
		goto done;
	}

	dev_dbg(dev,
		"Dual-link configuration detected (companion encoder %pOF)\n",
		companion);

	if (lvds->link_type == RZG3E_LVDS_DUAL_LINK_ODD_EVEN_PIXELS)
		dev_dbg(dev, "Data swapping required\n");

	/*
	 * FIXME: We should not be messing with the companion encoder private
	 * data from the primary encoder, we should rather let the companion
	 * encoder work things out on its own. However, the companion encoder
	 * doesn't hold a reference to the primary encoder, and
	 * drm_of_lvds_get_dual_link_pixel_order needs to be given references
	 * to the output ports of both encoders, therefore leave it like this
	 * for the time being.
	 */
	companion_lvds = bridge_to_rzg3e_lvds(lvds->companion);
	companion_lvds->link_type = lvds->link_type;
	companion_lvds->mode = LVDS_DUAL_LINK;

done:
	of_node_put(companion);

	return ret;
}

static int rzg3e_lvds_link_parse_dt(struct rzg3e_lvds *lvds)
{
	int ret;

	ret = drm_of_find_panel_or_bridge(lvds->dev->of_node, 1, 0,
					  &lvds->panel, &lvds->next_bridge);
	if (ret)
		goto done;

	if (lvds->panel) {
		lvds->next_bridge = devm_drm_panel_bridge_add(lvds->dev,
							      lvds->panel);
		if (IS_ERR_OR_NULL(lvds->next_bridge)) {
			ret = -EINVAL;
			goto done;
		}
	}

	ret = rzg3e_lvds_link_parse_dt_companion(lvds);

done:
	return ret;
}

static int rzg3e_lvds_link_get_clocks(struct rzg3e_lvds *lvds)
{
	lvds->clocks.phyclk = devm_clk_get(lvds->dev, "phyclk");
	if (IS_ERR(lvds->clocks.phyclk))
		return PTR_ERR(lvds->clocks.phyclk);

	lvds->clocks.dotclk = devm_clk_get(lvds->dev, "dotclk");
	if (IS_ERR(lvds->clocks.dotclk))
		return PTR_ERR(lvds->clocks.dotclk);

	return 0;
}

static int rzg3e_lvds_link_probe(struct platform_device *pdev)
{
	struct rzg3e_lvds *lvds;
	int ret;
	u32 ch;

	lvds = devm_kzalloc(&pdev->dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	platform_set_drvdata(pdev, lvds);

	lvds->dev = &pdev->dev;

	lvds->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lvds->mmio))
		return PTR_ERR(lvds->mmio);

	ret = rzg3e_lvds_link_get_clocks(lvds);
	if (ret < 0)
		return ret;

	ret = rzg3e_lvds_link_parse_dt(lvds);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(pdev->dev.of_node, "renesas,id", &ch);
	if (ret || ch > MAX_LVDS_CHAN_NUM - 1)
		return dev_err_probe(lvds->dev, -EINVAL, "invalid channel: %u\n", ch);

	lvds->ch = ch;
	rzg3e_lvds_set_lvds_ch_dot_clk(lvds->dev->parent, ch, lvds->clocks.dotclk);

	lvds->bridge.funcs = &rzg3e_lvds_link_bridge_ops;
	lvds->bridge.of_node = pdev->dev.of_node;
	drm_bridge_add(&lvds->bridge);

	return 0;
}

static int rzg3e_lvds_link_remove(struct platform_device *pdev)
{
	struct rzg3e_lvds *lvds = platform_get_drvdata(pdev);

	drm_bridge_remove(&lvds->bridge);

	return 0;
}

static const struct of_device_id rzg3e_lvds_link_of_table[] = {
	{ .compatible = "renesas,rzg3e-lvds-link" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, rzg3e_lvds_link_of_table);

static struct platform_driver rzg3e_lvds_link_platform_driver = {
	.probe		= rzg3e_lvds_link_probe,
	.remove		= rzg3e_lvds_link_remove,
	.driver		= {
		.name	= "rzg3e-lvds-link",
		.of_match_table = rzg3e_lvds_link_of_table,
	},
};

module_platform_driver(rzg3e_lvds_link_platform_driver);

MODULE_AUTHOR("Biju Das <biju.das.jz@bp.renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/G3E LVDS Encoder Driver");
MODULE_LICENSE("GPL");
