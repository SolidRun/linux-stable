// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX6 drm driver - MIPI DSI Host Controller
 *
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc.
 * Copyright (C) 2019-2020 Collabora, Ltd.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"

#define DSI_PWR_UP			0x04
#define RESET				0
#define POWERUP				BIT(0)

#define DSI_PHY_IF_CTRL			0x5c
#define PHY_IF_CTRL_RESET		0x0

#define DSI_PHY_TST_CTRL0		0x64
#define PHY_TESTCLK			BIT(1)
#define PHY_UNTESTCLK			0
#define PHY_TESTCLR			BIT(0)
#define PHY_UNTESTCLR			0

#define DSI_PHY_TST_CTRL1		0x68
#define PHY_TESTEN			BIT(16)
#define PHY_UNTESTEN			0
#define PHY_TESTDOUT(n)			(((n) & 0xff) << 8)
#define PHY_TESTDIN(n)			(((n) & 0xff) << 0)

struct imx_mipi_dsi {
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct device *dev;
	struct regmap *mux_sel;
	struct dw_mipi_dsi *mipi_dsi;
	struct clk *pllref_clk;

	void __iomem *base;
	unsigned int lane_mbps;
};

struct dphy_pll_testdin_map {
	unsigned int max_mbps;
	u8 testdin;
};

/* The table is based on 27MHz DPHY pll reference clock. */
static const struct dphy_pll_testdin_map dptdin_map[] = {
	{160, 0x04}, {180, 0x24}, {200, 0x44}, {210, 0x06},
	{240, 0x26}, {250, 0x46}, {270, 0x08}, {300, 0x28},
	{330, 0x48}, {360, 0x2a}, {400, 0x4a}, {450, 0x0c},
	{500, 0x2c}, {550, 0x0e}, {600, 0x2e}, {650, 0x10},
	{700, 0x30}, {750, 0x12}, {800, 0x32}, {850, 0x14},
	{900, 0x34}, {950, 0x54}, {1000, 0x74}
};

static int
imx_mipi_dsi_bridge_atomic_check(struct drm_bridge *bridge,
				 struct drm_bridge_state *bridge_state,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);

	/* The following values are taken from dw_hdmi_imx_atomic_check */
	imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	imx_crtc_state->di_hsync_pin = 2;
	imx_crtc_state->di_vsync_pin = 3;

	return 0;
}

static void imx_mipi_dsi_bridge_atomic_enable(struct drm_bridge *bridge,
					      struct drm_bridge_state *old_st)
{
	struct imx_mipi_dsi *dsi = bridge->driver_private;
	int mux = drm_of_encoder_active_port_id(dsi->dev->of_node,
						&dsi->encoder);
	/* Set IOMUX DSI output reg to correct IPU 1 or 0 and DI 1 or 0 ports */
	regmap_update_bits(dsi->mux_sel, IOMUXC_GPR3,
			   IMX6Q_GPR3_MIPI_MUX_CTL_MASK,
			   mux << IMX6Q_GPR3_MIPI_MUX_CTL_SHIFT);
}

static int imx_mipi_dsi_bridge_attach(struct drm_bridge *bridge,
				      enum drm_bridge_attach_flags flags)
{
	struct imx_mipi_dsi *dsi = bridge->driver_private;
	struct drm_bridge *dw_bridge = of_drm_find_bridge(dsi->dev->of_node);

	return drm_bridge_attach(bridge->encoder, dw_bridge,
				 &dsi->bridge, flags);
}

static const struct drm_bridge_funcs imx_dsi_bridge_funcs = {
	.atomic_enable = imx_mipi_dsi_bridge_atomic_enable,
	.atomic_check = imx_mipi_dsi_bridge_atomic_check,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.attach	= imx_mipi_dsi_bridge_attach,
};

static enum drm_mode_status
dw_mipi_dsi_imx_mode_valid(void *priv_data,
			   const struct drm_display_mode *mode,
			   unsigned long mode_flags, u32 lanes, u32 format)
{
	/*
	 * The VID_PKT_SIZE field in the DSI_VID_PKT_CFG
	 * register is 11-bit.
	 */
	if (mode->hdisplay > 0x7ff)
		return MODE_BAD_HVALUE;

	/*
	 * The V_ACTIVE_LINES field in the DSI_VTIMING_CFG
	 * register is 11-bit.
	 */
	if (mode->vdisplay > 0x7ff)
		return MODE_BAD_VVALUE;

	return MODE_OK;
}

static unsigned int max_mbps_to_testdin(unsigned int max_mbps)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dptdin_map); i++)
		if (dptdin_map[i].max_mbps == max_mbps)
			return dptdin_map[i].testdin;

	return -EINVAL;
}

static inline void dsi_write(struct imx_mipi_dsi *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + reg);
}

static int imx_mipi_dsi_phy_init(void *priv_data)
{
	struct imx_mipi_dsi *dsi = priv_data;
	int testdin;

	testdin = max_mbps_to_testdin(dsi->lane_mbps);
	if (testdin < 0) {
		DRM_DEV_ERROR(dsi->dev,
			      "failed to get testdin for %dmbps lane clock\n",
			      dsi->lane_mbps);
		return testdin;
	}

	dsi_write(dsi, DSI_PHY_IF_CTRL, PHY_IF_CTRL_RESET);
	dsi_write(dsi, DSI_PWR_UP, POWERUP);

	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK | PHY_UNTESTCLR);
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_TESTEN | PHY_TESTDOUT(0) |
		  PHY_TESTDIN(0x44));
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK | PHY_UNTESTCLR);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK | PHY_UNTESTCLR);
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_UNTESTEN | PHY_TESTDOUT(0) |
		  PHY_TESTDIN(testdin));
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK | PHY_UNTESTCLR);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK | PHY_UNTESTCLR);

	return 0;
}

static int imx_mipi_dsi_get_lane_mbps(void *priv_data,
				      const struct drm_display_mode *mode,
				      unsigned long mode_flags, u32 lanes,
				      u32 format, unsigned int *lane_mbps)
{
	struct imx_mipi_dsi *dsi = priv_data;
	unsigned int i, target_mbps, mpclk;
	int bpp;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get bpp for format %d: %d\n",
			      format, bpp);
		return bpp;
	}

	mpclk = DIV_ROUND_UP(mode->clock, MSEC_PER_SEC);
	if (mpclk) {
		/* take 1/0.7 blanking overhead into consideration */
		target_mbps = (mpclk * (bpp / lanes) * 10) / 7;
	} else {
		DRM_DEV_DEBUG(dsi->dev, "Using default 1Gbps DPHY pll clock\n");
		target_mbps = 1000;
	}

	DRM_DEV_DEBUG(dsi->dev, "Target lane mbps is %u Mbps\n", target_mbps);

	for (i = 0; i < ARRAY_SIZE(dptdin_map); i++) {
		if (target_mbps < dptdin_map[i].max_mbps) {
			*lane_mbps = dptdin_map[i].max_mbps;
			dsi->lane_mbps = *lane_mbps;
			DRM_DEV_DEBUG(dsi->dev, "Real lane mbps is %u Mbps\n",
				      *lane_mbps);
			return 0;
		}
	}

	DRM_DEV_ERROR(dsi->dev, "Out of range DPHY clk frequency for %uMbps\n",
		      target_mbps);

	return -EINVAL;
}

static int
dw_mipi_dsi_phy_get_timing(void *priv_data, unsigned int lane_mbps,
			   struct dw_mipi_dsi_dphy_timing *timing)
{
	timing->clk_hs2lp = 0x40;
	timing->clk_lp2hs = 0x40;
	timing->data_hs2lp = 0x40;
	timing->data_lp2hs = 0x40;

	return 0;
}

static const struct dw_mipi_dsi_phy_ops dw_mipi_dsi_imx6_phy_ops = {
	.init = imx_mipi_dsi_phy_init,
	.get_lane_mbps = imx_mipi_dsi_get_lane_mbps,
	.get_timing = dw_mipi_dsi_phy_get_timing,
};

static struct dw_mipi_dsi_plat_data imx6_mipi_dsi_drv_data = {
	.max_data_lanes = 2,
	.mode_valid = dw_mipi_dsi_imx_mode_valid,
	.phy_ops = &dw_mipi_dsi_imx6_phy_ops,
};

static const struct of_device_id imx_dsi_dt_ids[] = {
	{
		.compatible = "fsl,imx6-mipi-dsi",
		.data = &imx6_mipi_dsi_drv_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_dsi_dt_ids);

static int imx_mipi_dsi_bind(struct device *dev, struct device *master,
			     void *data)
{
	struct imx_mipi_dsi *dsi = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	ret = imx_drm_encoder_parse_of(drm, &dsi->encoder, dsi->dev->of_node);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to parse encoder of node\n");
		return ret;
	}

	ret = drm_simple_encoder_init(drm, &dsi->encoder,
				      DRM_MODE_ENCODER_NONE);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to create drm encoder\n");
		return ret;
	}

	ret = drm_bridge_attach(&dsi->encoder, &dsi->bridge, NULL, 0);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to attach IMX6 bridge\n");
		drm_encoder_cleanup(&dsi->encoder);
		return ret;
	}

	return 0;
}

static void imx_mipi_dsi_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct imx_mipi_dsi *dsi = dev_get_drvdata(dev);

	drm_encoder_cleanup(&dsi->encoder);
}

static const struct component_ops imx_mipi_dsi_ops = {
	.bind	= imx_mipi_dsi_bind,
	.unbind	= imx_mipi_dsi_unbind,
};

static int imx_mipi_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id = of_match_device(imx_dsi_dt_ids, dev);
	struct dw_mipi_dsi_plat_data *pdata = (struct dw_mipi_dsi_plat_data *) of_id->data;
	struct imx_mipi_dsi *dsi;
	struct resource *res;
	unsigned long pllref_rate;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->base))
		return PTR_ERR(dsi->base);

	dsi->pllref_clk = devm_clk_get(dev, "ref");
	if (IS_ERR(dsi->pllref_clk))
		return PTR_ERR(dsi->pllref_clk);

	ret = clk_prepare_enable(dsi->pllref_clk);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to enable pllref_clk: %d\n", ret);
		return ret;
	}

	pllref_rate = clk_get_rate(dsi->pllref_clk);
	if (pllref_rate != 27000000) {
		DRM_DEV_ERROR(dev, "Only 27Mhz pllref_clk is currently supported\n");
		ret = -EINVAL;
		goto err_clkdisable;
	}

	dsi->mux_sel = syscon_regmap_lookup_by_phandle(dev->of_node, "fsl,gpr");
	if (IS_ERR(dsi->mux_sel)) {
		ret = PTR_ERR(dsi->mux_sel);
		DRM_DEV_ERROR(dev, "Failed to get GPR regmap: %d\n", ret);
		goto err_clkdisable;
	}

	dsi->dev = dev;
	dev_set_drvdata(dev, dsi);

	imx6_mipi_dsi_drv_data.base = dsi->base;
	imx6_mipi_dsi_drv_data.priv_data = dsi;

	dsi->mipi_dsi = dw_mipi_dsi_probe(pdev, pdata);
	if (IS_ERR(dsi->mipi_dsi)) {
		ret = PTR_ERR(dsi->mipi_dsi);
		DRM_DEV_ERROR(dev, "Failed to probe DW DSI host: %d\n", ret);
		goto err_clkdisable;
	}

	dsi->bridge.driver_private = dsi;
	dsi->bridge.funcs = &imx_dsi_bridge_funcs;
	dsi->bridge.type = DRM_MODE_CONNECTOR_DSI;
	dsi->bridge.encoder = &dsi->encoder;

	drm_bridge_add(&dsi->bridge);

	return component_add(&pdev->dev, &imx_mipi_dsi_ops);

err_clkdisable:
	clk_disable_unprepare(dsi->pllref_clk);
	return ret;
}

static int imx_mipi_dsi_remove(struct platform_device *pdev)
{
	struct imx_mipi_dsi *dsi = platform_get_drvdata(pdev);

	clk_disable_unprepare(dsi->pllref_clk);
	drm_bridge_remove(&dsi->bridge);
	component_del(&pdev->dev, &imx_mipi_dsi_ops);

	return 0;
}

static struct platform_driver imx_mipi_dsi_driver = {
	.probe		= imx_mipi_dsi_probe,
	.remove		= imx_mipi_dsi_remove,
	.driver		= {
		.of_match_table = imx_dsi_dt_ids,
		.name	= "dw-mipi-dsi-imx6",
	},
};
module_platform_driver(imx_mipi_dsi_driver);

MODULE_DESCRIPTION("i.MX6 MIPI DSI host controller driver");
MODULE_AUTHOR("Liu Ying <victor.liu@nxp.com>");
MODULE_AUTHOR("Adrian Ratiu <adrian.ratiu@collabora.com>");
MODULE_LICENSE("GPL");
