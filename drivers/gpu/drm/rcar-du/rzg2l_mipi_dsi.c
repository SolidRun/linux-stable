// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G2L MIPI DSI Encoder Driver
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <asm-generic/delay.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_panel.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <video/mipi_display.h>

#include "rzg2l_mipi_dsi_regs.h"
#include "rzg2l_mipi_dsi.h"

#define RZ_G2L_MIPI_DSI_MAX_DATA_LANES	4

struct rzg2l_mipi_dsi {
	struct device *dev;
	void __iomem *link_mmio;
	void __iomem *phy_mmio;

	struct mipi_dsi_host host;

	struct drm_bridge bridge;

	struct drm_bridge *next_bridge;
	struct drm_connector connector;
	struct drm_panel *panel;

	struct drm_display_mode display_mode;

	struct {
		struct clk *pllclk;
		struct clk *sysclk;
		struct clk *aclk;
		struct clk *pclk;
		struct clk *vclk;
		struct clk *lpclk;
	} clocks;

	struct {
		struct reset_control *cmn_rstb;
		struct reset_control *areset_n;
		struct reset_control *preset_n;
	} rstc;

	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
	unsigned long mode_flags;

	unsigned long hsfreq;

	bool hsclkmode;	/* 0 for non-continuous and 1 for continuous clock mode */
};

#define bridge_to_rzg2l_mipi_dsi(b) \
	container_of(b, struct rzg2l_mipi_dsi, bridge)

#define connector_to_rzg2l_mipi_dsi(c) \
	container_of(c, struct rzg2l_mipi_dsi, connector)

#define host_to_rzg2l_mipi_dsi(c) \
	container_of(c, struct rzg2l_mipi_dsi, host)

static void rzg2l_mipi_dsi_write(void __iomem *mem, u32 reg, u32 data)
{
	iowrite32(data, mem + reg);
}

static u32 rzg2l_mipi_dsi_read(void __iomem *mem, u32 reg)
{
	return ioread32(mem + reg);
}

static void rzg2l_mipi_dsi_clr(void __iomem *mem, u32 reg, u32 clr)
{
	rzg2l_mipi_dsi_write(mem, reg, rzg2l_mipi_dsi_read(mem, reg) & ~clr);
}

static int rzg2l_mipi_dsi_find_panel_or_bridge(struct rzg2l_mipi_dsi *mipi_dsi);

/* -----------------------------------------------------------------------------
 * Hardware Setup
 */

static int rzg2l_mipi_dsi_startup(struct rzg2l_mipi_dsi *mipi_dsi)
{
	struct drm_display_mode *mode = &mipi_dsi->display_mode;
	u32 txsetr;
	u32 clstptsetr;
	u32 lptrnstsetr;
	u8 max_num_lanes;
	u32 clkkpt;
	u32 clkbfht;
	u32 clkstpt;
	u32 golpbkt;
	unsigned int bpp;
	struct {
		u32 tclk_miss;
		u32 t_init;
		u32 tclk_prepare;
		u32 tclk_settle;
		u32 tclk_trail;
		u32 tclk_post;
		u32 tclk_pre;
		u32 tclk_zero;
		u32 tlpx;
		u32 ths_prepare;
		u32 ths_settle;
		u32 ths_exit;
		u32 ths_trail;
		u32 ths_zero;
	} timings;
	u32 dphyctrl0;
	u32 dphytim0;
	u32 dphytim1;
	u32 dphytim2;
	u32 dphytim3;

	/* Relationship between hsclk and vclk must follow:
	 * vclk * bpp = hsclk * 8 * lanes
	 * where vclk: video clock (Hz)
	 *       bpp: video pixel bit depth
	 *       hsclk: DSI HS Byte clock frequency (Hz)
	 *       lanes: number of data lanes
	 *
	 * hsclk(bit) = hsclk(byte) * 8
	 */

	bpp = mipi_dsi_pixel_format_to_bpp(mipi_dsi->format);
	mipi_dsi->hsfreq = (mode->clock * bpp * 8) / (8 * mipi_dsi->lanes);

	/* Initializing DPHY before accessing LINK */

	/* All DSI global operation timings are set with recommended setting */
	if (mipi_dsi->hsfreq > 250000) {
		timings.tclk_miss = 1;
		timings.t_init = 79801;
		timings.tclk_prepare = 8;
		timings.tclk_settle = 9;
		timings.tclk_trail = 7;
		timings.tclk_post = 35;
		timings.tclk_pre = 4;
		timings.tclk_zero = 33;
		timings.tlpx = 6;
		timings.ths_prepare = 9;
		timings.ths_settle = 9;
		timings.ths_exit = 13;
		timings.ths_trail = 9;
		timings.ths_zero = 16;
	} else {
		timings.tclk_miss = 1;
		timings.t_init = 79801;
		timings.tclk_prepare = 8;
		timings.tclk_settle = 9;
		timings.tclk_trail = 10;
		timings.tclk_post = 94;
		timings.tclk_pre = 13;
		timings.tclk_zero = 33;
		timings.tlpx = 6;
		timings.ths_prepare = 12;
		timings.ths_settle = 9;
		timings.ths_exit = 13;
		timings.ths_trail = 17;
		timings.ths_zero = 23;
	}

	dphytim0 = DSIDPHYTIM0_TCLK_MISS(timings.tclk_miss) |
		   DSIDPHYTIM0_T_INIT(timings.t_init);
	dphytim1 = DSIDPHYTIM1_THS_PREPARE(timings.ths_prepare) |
		   DSIDPHYTIM1_TCLK_PREPARE(timings.tclk_prepare) |
		   DSIDPHYTIM1_THS_SETTLE(timings.ths_settle) |
		   DSIDPHYTIM1_TCLK_SETTLE(timings.tclk_settle);
	dphytim2 = DSIDPHYTIM2_TCLK_TRAIL(timings.tclk_trail) |
		   DSIDPHYTIM2_TCLK_POST(timings.tclk_post) |
		   DSIDPHYTIM2_TCLK_PRE(timings.tclk_pre) |
		   DSIDPHYTIM2_TCLK_ZERO(timings.tclk_zero);
	dphytim3 = DSIDPHYTIM3_TLPX(timings.tlpx) |
		   DSIDPHYTIM3_THS_EXIT(timings.ths_exit) |
		   DSIDPHYTIM3_THS_TRAIL(timings.ths_trail) |
		   DSIDPHYTIM3_THS_ZERO(timings.ths_zero);

	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYTIM0, dphytim0);
	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYTIM1, dphytim1);
	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYTIM2, dphytim2);
	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYTIM3, dphytim3);

	/* Trimming signals is set with normal mode default code */
	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYTRIM0, 0x5A8BBBBB);
	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYTRIM0,
						 DSIDPHYCTRL1_TRIM_REGSEL);

	dphyctrl0 = DSIDPHYCTRL0_CAL_EN_HSRX_OFS | DSIDPHYCTRL0_CMN_MASTER_EN |
		    DSIDPHYCTRL0_RE_VDD_DETVCCQLV18 | DSIDPHYCTRL0_EN_BGR;
	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYCTRL0, dphyctrl0);
	udelay(20);
	dphyctrl0 |= DSIDPHYCTRL0_EN_LDO1200;
	rzg2l_mipi_dsi_write(mipi_dsi->phy_mmio, DSIDPHYCTRL0, dphyctrl0);
	udelay(10);

	/* Check number of lanes capability */
	max_num_lanes = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, TXSETR) >> 16;
	max_num_lanes = (max_num_lanes & 0x3) + 1;
	if (max_num_lanes < mipi_dsi->lanes) {
		dev_err(mipi_dsi->dev, "DPHY can not support %d lanes\n",
					mipi_dsi->lanes);
		return -EINVAL;
	}

	/* Enable Data lanes and Clock lanes */
	txsetr = TXSETR_DLEN | TXSETR_NUMLANEUSE(mipi_dsi->lanes - 1) | TXSETR_CLEN;
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, TXSETR, txsetr);

	/*
	 * Global timings characteristic depends on high speed Clock Frequency
	 * Currently MIPI DSI-IF just supports maximum FHD@60 with:
	 * - videoclock = 148.5 (MHz)
	 * - bpp: maximum 24bpp
	 * - data lanes: maximum 4 lanes
	 * Therefore maximum hsclk will be 891 Mbps.
	 */

	/* TODO
	 * Clock lane Stop time parameter will be hard set in 3 ranges:
	 * 1st range: 0 < hsclk <= 250 Mbps
	 * 2nd range: 250 < hsclk <= 445.5 Mbps
	 * 3rd range: hsclk > 445.5 Mbps
	 * With 445.5 Mbps is hsclk at HD@60 or FHD@30 with 24bpp
	 * and 4 data lanes.
	 */

	if (mipi_dsi->hsfreq > 445500) {
		clkkpt = 12;
		clkbfht = 15;
		clkstpt = 48;
		golpbkt = 75;
	} else if (mipi_dsi->hsfreq > 250000) {
		clkkpt = 7;
		clkbfht = 8;
		clkstpt = 27;
		golpbkt = 40;
	} else {
		clkkpt = 8;
		clkbfht = 6;
		clkstpt = 24;
		golpbkt = 29;
	}

	clstptsetr = CLSTPTSETR_CLKKPT(clkkpt) | CLSTPTSETR_CLKBFHT(clkbfht) |
		     CLSTPTSETR_CLKSTPT(clkstpt);
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, CLSTPTSETR, clstptsetr);

	lptrnstsetr = LPTRNSTSETR_GOLPBKT(golpbkt);
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, LPTRNSTSETR, lptrnstsetr);

	return 0;
}

static void rzg2l_mipi_dsi_set_display_timing(struct rzg2l_mipi_dsi *mipi_dsi)
{
	struct drm_display_mode *mode = &mipi_dsi->display_mode;
	u32 vich1ppsetr;
	u32 vich1vssetr;
	u32 vich1vpsetr;
	u32 vich1hssetr;
	u32 vich1hpsetr;
	int dsi_format;
	u32 delay[2];

	/* Configuration for Pixel Packet */
	dsi_format = mipi_dsi_pixel_format_to_bpp(mipi_dsi->format);
	switch (dsi_format) {
	case 24:
		vich1ppsetr = VICH1PPSETR_DT_RGB24;
		break;
	case 18:
		vich1ppsetr = VICH1PPSETR_DT_RGB18;
		break;
	case 16:
		vich1ppsetr = VICH1PPSETR_DT_RGB16;
		break;
	default:
		dev_warn(mipi_dsi->dev, "unsupported format");
		return;
	}

	if (mipi_dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		vich1ppsetr |= VICH1PPSETR_TXESYNC_PULSE;

	if (mipi_dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		vich1ppsetr &= ~VICH1PPSETR_TXESYNC_PULSE;

	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1PPSETR, vich1ppsetr);

	/* Configuration for Video Parameters */
	vich1vssetr = VICH1VSSETR_VACTIVE(mode->vdisplay) |
		      VICH1VSSETR_VSA(mode->vsync_end - mode->vsync_start);
	vich1vssetr |= (mode->flags & DRM_MODE_FLAG_PVSYNC) ?
			VICH1VSSETR_VSPOL_HIGH : VICH1VSSETR_VSPOL_LOW;

	vich1vpsetr = VICH1VPSETR_VFP(mode->vsync_start - mode->vdisplay) |
		      VICH1VPSETR_VBP(mode->vtotal - mode->vsync_end);

	vich1hssetr = VICH1HSSETR_HACTIVE(mode->hdisplay) |
		      VICH1HSSETR_HSA(mode->hsync_end - mode->hsync_start);
	vich1hssetr |= (mode->flags & DRM_MODE_FLAG_PHSYNC) ?
			VICH1HSSETR_HSPOL_HIGH : VICH1HSSETR_HSPOL_LOW;

	vich1hpsetr = VICH1HPSETR_HFP(mode->hsync_start - mode->hdisplay) |
		      VICH1HPSETR_HBP(mode->htotal - mode->hsync_end);

	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1VSSETR, vich1vssetr);
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1VPSETR, vich1vpsetr);
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1HSSETR, vich1hssetr);
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1HPSETR, vich1hpsetr);

	if (mipi_dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		mipi_dsi->hsclkmode = 0;
	else
		mipi_dsi->hsclkmode = 1;

	/* Configuration for Delay Value */
	/* TODO
	 * In HW manual, delay depends on many parameters with complex formula.
	 * Therefore, hard set delay value based on 2 ranges of video clock.
	 * 74.25MHz is videoclock of HD@60p or FHD@30p
	 */

	if (mode->clock > 74250) {
		delay[0] = 231;
		delay[1] = 216;
	} else {
		delay[0] = 220;
		delay[1] = 212;
	}

	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1SET1R,
			     VICH1SET1R_DLY(delay[mipi_dsi->hsclkmode]));
}

static int rzg2l_mipi_dsi_start_hs_clock(struct rzg2l_mipi_dsi *mipi_dsi)
{
	unsigned int timeout;
	u32 status;
	u32 hsclksetr;

	/* In HW manual, we need to check stability of clock lane before
	 * starting HS clock.
	 * But it does not write how to check.
	 * Therefore, we skip this check in current source and update if
	 * any new information.
	 */

	/* Start HS clock */
	hsclksetr = HSCLKSETR_HSCLKRUN_HS |
		 (mipi_dsi->hsclkmode ? HSCLKSETR_HSCLKMODE_CONT :
					HSCLKSETR_HSCLKMODE_NON_CONT);
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, HSCLKSETR, hsclksetr);

	if (mipi_dsi->hsclkmode) {
		for (timeout = 10; timeout > 0; --timeout) {
			status = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, PLSR);
			if (status & PLSR_CLLP2HS)
				break;

			usleep_range(1000, 2000);
		}

		if (!timeout) {
			dev_err(mipi_dsi->dev, "failed to start HS clock\n");
			return -ETIMEDOUT;
		}
	}

	dev_dbg(mipi_dsi->dev, "Start High Speed Clock with %s clock mode",
		mipi_dsi->hsclkmode ? "continuous" : "non-continuous");

	return 0;
}

static int rzg2l_mipi_dsi_stop_hs_clock(struct rzg2l_mipi_dsi *mipi_dsi)
{
	unsigned int timeout;
	u32 status;

	/* Stop HS clock */
	rzg2l_mipi_dsi_clr(mipi_dsi->link_mmio, HSCLKSETR,
						HSCLKSETR_HSCLKRUN_HS);

	if (mipi_dsi->hsclkmode) {
		for (timeout = 10; timeout > 0; --timeout) {
			status = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, PLSR);
			if (status & PLSR_CLHS2LP)
				break;

			usleep_range(1000, 2000);
		}

		if (!timeout) {
			dev_err(mipi_dsi->dev, "failed to stop HS clock\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}
static int rzg2l_mipi_dsi_start_video(struct rzg2l_mipi_dsi *mipi_dsi)
{
	unsigned int timeout;
	u32 status;
	u32 vich1set0r;

	/* Configuration for Blanking sequence and start video input*/
	vich1set0r = VICH1SET0R_HFPNOLP | VICH1SET0R_HBPNOLP |
		     VICH1SET0R_HSANOLP | VICH1SET0R_VSTART;
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1SET0R, vich1set0r);

	for (timeout = 10; timeout > 0; --timeout) {
		status = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, VICH1SR);
		if (status & VICH1SR_VIRDY)
			break;

		usleep_range(1000, 2000);
	}

	if (!timeout) {
		dev_err(mipi_dsi->dev, "Failed to start video signal input\n");
		return -ETIMEDOUT;
	}

	dev_dbg(mipi_dsi->dev, "Start video transferring");

	return 0;
}

static int rzg2l_mipi_dsi_stop_video(struct rzg2l_mipi_dsi *mipi_dsi)
{
	unsigned int timeout;
	u32 status;

	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, VICH1SET0R, VICH1SET0R_VSTPAFT);

	for (timeout = 10; timeout > 0; --timeout) {
		status = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, VICH1SR);
		if ((status & VICH1SR_STOP) && (!(status & VICH1SR_RUNNING)))
			break;

		usleep_range(1000, 2000);
	}

	if (!timeout)
		goto err;

	for (timeout = 10; timeout > 0; --timeout) {
		status = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, LINKSR);
		if (!(status & LINKSR_HSBUSY))
			break;

		usleep_range(1000, 2000);
	}

	if (!timeout)
		goto err;

	return 0;
err:
	dev_err(mipi_dsi->dev, "Failed to stop video signal input\n");
	return -ETIMEDOUT;
}

/* -----------------------------------------------------------------------------
 * Connector & Panel
 */

static int rzg2l_mipi_dsi_connector_get_modes(struct drm_connector *connector)
{
	struct rzg2l_mipi_dsi *mipi_dsi =
					connector_to_rzg2l_mipi_dsi(connector);

	return drm_panel_get_modes(mipi_dsi->panel, connector);
}

static int
rzg2l_mipi_dsi_connector_atomic_check(struct drm_connector *connector,
				      struct drm_atomic_state *state)
{
	struct rzg2l_mipi_dsi *mipi_dsi =
					connector_to_rzg2l_mipi_dsi(connector);
	const struct drm_display_mode *panel_mode;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state->crtc)
		return 0;

	if (list_empty(&connector->modes)) {
		dev_dbg(mipi_dsi->dev, "connector: empty modes list\n");
		return -EINVAL;
	}

	panel_mode = list_first_entry(&connector->modes,
				      struct drm_display_mode, head);

	/* We're not allowed to modify the resolution. */
	crtc_state = drm_atomic_get_crtc_state(state, conn_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	if (crtc_state->mode.hdisplay != panel_mode->hdisplay ||
	    crtc_state->mode.vdisplay != panel_mode->vdisplay)
		return -EINVAL;

	/* The flat panel mode is fixed, just copy it to the adjusted mode. */
	drm_mode_copy(&crtc_state->adjusted_mode, panel_mode);

	return 0;
}

static const struct
drm_connector_helper_funcs rzg2l_mipi_dsi_conn_helper_funcs = {
	.get_modes = rzg2l_mipi_dsi_connector_get_modes,
	.atomic_check = rzg2l_mipi_dsi_connector_atomic_check,
};

static const struct drm_connector_funcs rzg2l_mipi_dsi_conn_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* -----------------------------------------------------------------------------
 * Bridge
 */
static int rzg2l_mipi_dsi_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct rzg2l_mipi_dsi *mipi_dsi = bridge_to_rzg2l_mipi_dsi(bridge);
	struct drm_connector *connector = &mipi_dsi->connector;
	struct drm_encoder *encoder = bridge->encoder;
	int ret;

	ret = rzg2l_mipi_dsi_find_panel_or_bridge(mipi_dsi);
	if (ret < 0)
		return ret;

	/* If we have a next bridge just attach it. */
	if (mipi_dsi->next_bridge)
		return drm_bridge_attach(bridge->encoder,
					 mipi_dsi->next_bridge, bridge, flags);

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR) {
		DRM_ERROR("Fix bridge driver to make connector optional!");
		return -EINVAL;
	}

	/* Otherwise if we have a panel, create a connector. */
	if (!mipi_dsi->panel)
		return 0;

	ret = drm_connector_init(bridge->dev, connector,
				 &rzg2l_mipi_dsi_conn_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret < 0)
		return ret;

	drm_connector_helper_add(connector, &rzg2l_mipi_dsi_conn_helper_funcs);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret < 0)
		return ret;

	return 0;
}

static void rzg2l_mipi_dsi_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adjusted_mode)
{
	struct rzg2l_mipi_dsi *mipi_dsi = bridge_to_rzg2l_mipi_dsi(bridge);

	mipi_dsi->display_mode = *adjusted_mode;
}

static void rzg2l_mipi_dsi_enable(struct drm_bridge *bridge)
{
	struct rzg2l_mipi_dsi *mipi_dsi = bridge_to_rzg2l_mipi_dsi(bridge);
	int ret;

	ret = rzg2l_mipi_dsi_startup(mipi_dsi);
	if (ret < 0)
		return;

	rzg2l_mipi_dsi_set_display_timing(mipi_dsi);

	if (mipi_dsi->panel) {
		drm_panel_prepare(mipi_dsi->panel);
		drm_panel_enable(mipi_dsi->panel);
	}

	ret = rzg2l_mipi_dsi_start_hs_clock(mipi_dsi);
	if (ret < 0)
		return;

	ret = rzg2l_mipi_dsi_start_video(mipi_dsi);
	if (ret < 0)
		return;
}

static void rzg2l_mipi_dsi_disable(struct drm_bridge *bridge)
{
	struct rzg2l_mipi_dsi *mipi_dsi = bridge_to_rzg2l_mipi_dsi(bridge);
	int ret;

	ret = rzg2l_mipi_dsi_stop_video(mipi_dsi);
	if (ret < 0)
		return;

	if (mipi_dsi->panel) {
		drm_panel_disable(mipi_dsi->panel);
		drm_panel_unprepare(mipi_dsi->panel);
	}

	ret = rzg2l_mipi_dsi_stop_hs_clock(mipi_dsi);
	if (ret < 0)
		return;
}

static enum drm_mode_status
rzg2l_mipi_dsi_bridge_mode_valid(struct drm_bridge *bridge,
				 const struct drm_display_info *info,
				 const struct drm_display_mode *mode)
{
	if (mode->clock > 148500)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void rzg2l_mipi_dsi_detach(struct drm_bridge *bridge)
{
}

static const struct drm_bridge_funcs rzg2l_mipi_dsi_bridge_ops = {
	.attach = rzg2l_mipi_dsi_attach,
	.detach = rzg2l_mipi_dsi_detach,
	.mode_set = rzg2l_mipi_dsi_mode_set,
	.enable = rzg2l_mipi_dsi_enable,
	.disable = rzg2l_mipi_dsi_disable,
	.mode_valid = rzg2l_mipi_dsi_bridge_mode_valid,
};

/* -----------------------------------------------------------------------------
 * Clock Setting
 */


int rzg2l_mipi_dsi_clk_enable(struct drm_bridge *bridge)
{
	struct rzg2l_mipi_dsi *mipi_dsi = bridge_to_rzg2l_mipi_dsi(bridge);
	int ret;

	reset_control_deassert(mipi_dsi->rstc.cmn_rstb);
	reset_control_deassert(mipi_dsi->rstc.areset_n);
	reset_control_deassert(mipi_dsi->rstc.preset_n);

	ret = clk_prepare_enable(mipi_dsi->clocks.pllclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(mipi_dsi->clocks.sysclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(mipi_dsi->clocks.aclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(mipi_dsi->clocks.pclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(mipi_dsi->clocks.vclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(mipi_dsi->clocks.lpclk);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(rzg2l_mipi_dsi_clk_enable);

void rzg2l_mipi_dsi_clk_disable(struct drm_bridge *bridge)
{
	struct rzg2l_mipi_dsi *mipi_dsi = bridge_to_rzg2l_mipi_dsi(bridge);

	clk_disable_unprepare(mipi_dsi->clocks.pllclk);
	clk_disable_unprepare(mipi_dsi->clocks.sysclk);
	clk_disable_unprepare(mipi_dsi->clocks.aclk);
	clk_disable_unprepare(mipi_dsi->clocks.pclk);
	clk_disable_unprepare(mipi_dsi->clocks.vclk);
	clk_disable_unprepare(mipi_dsi->clocks.lpclk);

	reset_control_assert(mipi_dsi->rstc.cmn_rstb);
	reset_control_assert(mipi_dsi->rstc.areset_n);
	reset_control_assert(mipi_dsi->rstc.preset_n);
}
EXPORT_SYMBOL_GPL(rzg2l_mipi_dsi_clk_disable);

/* -----------------------------------------------------------------------------
 * Host setting
 */

static int rzg2l_mipi_dsi_host_attach(struct mipi_dsi_host *host,
				      struct mipi_dsi_device *device)
{
	struct rzg2l_mipi_dsi *mipi_dsi = host_to_rzg2l_mipi_dsi(host);

	if (device->lanes > RZ_G2L_MIPI_DSI_MAX_DATA_LANES)
		return -EINVAL;

	mipi_dsi->lanes = device->lanes;
	mipi_dsi->format = device->format;
	mipi_dsi->mode_flags = device->mode_flags;

	return 0;
}

static int rzg2l_mipi_dsi_host_detach(struct mipi_dsi_host *host,
				      struct mipi_dsi_device *device)
{
	return 0;
}

/* Based on MIPI Alliance Specification for DSI */
static const char * const error_report[16] = {
	"SoT Error",
	"SoT Sync Error",
	"EoT Sync Error",
	"Escape Mode Entry Command Error",
	"Low-Power Transmit Sync Error",
	"Peripheral Timeout Error",
	"False Control Error",
	"Contention Detected",
	"ECC Error, single-bit",
	"ECC Error, multi-bit",
	"Checksum Error",
	"DSI Data Type Not Recognized",
	"DSI VC ID Invalid",
	"Invalid Transmission Length",
	"Reserved",
	"DSI Protocol Violation",
};

static ssize_t rzg2l_mipi_dsi_host_transfer(struct mipi_dsi_host *host,
					    const struct mipi_dsi_msg *msg)
{
	struct rzg2l_mipi_dsi *mipi_dsi = host_to_rzg2l_mipi_dsi(host);
	struct mipi_dsi_packet packet;
	bool is_long = mipi_dsi_packet_format_is_long(msg->type);
	bool is_need_bta = false;
	ssize_t err = 0;
	u32 sqch0dsc00ar, sqch0dsc00br, status;
	unsigned int timeout;
	unsigned int count;
	unsigned int i, j;
	u32 tx_data, rx_data;

	err = mipi_dsi_create_packet(&packet, msg);
	if (err < 0)
		return err;

	/* Temporarily support maximum 16 bytes payload and RX data */
	if ((packet.payload_length > 16) || (msg->rx_len > 16))
		return -ENOSPC;

	if ((msg->flags & MIPI_DSI_MSG_REQ_ACK) ||
	   ((msg->type & MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM ||
	     msg->type & MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM ||
	     msg->type & MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM ||
	     msg->type & MIPI_DSI_DCS_READ) &&
	    (msg->rx_buf && msg->rx_len > 0)))
		is_need_bta = true;
	else
		sqch0dsc00ar = SQCH0DSC00AR_BTA_NO_BTA;

	/* Terminate Operation after this descriptor finished */
	sqch0dsc00ar |= SQCH0DSC00AR_NXACT_TERM;

	/* High speed transmission */
	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		sqch0dsc00ar |= SQCH0DSC00AR_SPD_LOW;
	else
		sqch0dsc00ar |= SQCH0DSC00AR_SPD_HIGH;

	/* Write TX Packet Header */
	sqch0dsc00ar |= (SQCH0DSC00AR_VC_DT(packet.header[0]) |
			 SQCH0DSC00AR_DATA0(packet.header[1]) |
			 SQCH0DSC00AR_DATA1(packet.header[2]));

	/* Sending non-read packets */
	if (is_long) {
		/* Count the amount of TXPPDxR will be used */
		count = ((packet.payload_length - 1) / 4) + 1;
		/* Long packet transmission */
		sqch0dsc00ar |= SQCH0DSC00AR_FMT_LONG;
		/* Write TX Packet Payload Data */
		for (i = 0; i < count; i++) {
			tx_data = 0;
			for (j = 0; j < 4; j++) {
				if (packet.payload_length == (4 * i + j))
					break;

				tx_data |= (packet.payload[4 * i + j] << (8 * j));
			}

			rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, TXPPDxR(i),
					     tx_data);
		}
	} else {
		/* Short packet transmission */
		sqch0dsc00ar |= SQCH0DSC00AR_FMT_SHORT;
	}

	if (is_need_bta) {
		if (msg->flags & MIPI_DSI_MSG_REQ_ACK)
			sqch0dsc00ar |= SQCH0DSC00AR_BTA_NON_READ_BTA;
		else
			sqch0dsc00ar |= SQCH0DSC00AR_BTA_READ_BTA;

	}

	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, SQCH0DSC00AR, sqch0dsc00ar);

	/* Packet Payload Data register is used to data select */
	sqch0dsc00br = SQCH0DSC00BR_DTSEL_PAYLOAD_SIZE;
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, SQCH0DSC00BR, sqch0dsc00br);

	/* Indicate rx result save slot number 0 */
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, SQCH0DSC00CR,
						  SQCH0DSC00CR_FINACT);
	/* Start Sequence 0 Operation */
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, SQCH0SET0R,
			rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, SQCH0SET0R) |
			SQCH0SET0R_START);

	for (timeout = 10; timeout > 0; --timeout) {
		status = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, SQCH0SR);
		if (status & SQCH0SR_ADESFIN) {
			/* Clear the status bit */
			rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, SQCH0SCR,
					     SQCH0SCR_ADESFIN);

			break;
		}

		usleep_range(1000, 2000);
	}

	if (!timeout) {
		err = -ETIMEDOUT;
		goto stop_sequence;

	}

	if (is_need_bta) {
		u8 *msg_rx = msg->rx_buf;
		size_t size = 0;
		u32 datatype, errors;

		status = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, RXRSS0R);
		if (status & RXRSS0R_RXSUC) {
			datatype = (status & RXRSS0R_DT) >> 16;

			switch (datatype & 0x3f) {
			case 0:
				dev_dbg(mipi_dsi->dev, "ACK\n");
				break;
			case MIPI_DSI_RX_END_OF_TRANSMISSION:
				dev_dbg(mipi_dsi->dev, "EoTp\n");
				break;
			case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
				errors = status & RXRSS0R_DATA;

				dev_dbg(mipi_dsi->dev,
					"Acknowledge and error report: %04x\n",
					errors);

				for (i = 0; i < ARRAY_SIZE(error_report); i++)
					if (errors & BIT(i))
						dev_dbg(mipi_dsi->dev,
							"  %2u: %s\n",
							i, error_report[i]);
				break;
			case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
				msg_rx[0] = (status & RXRSS0R_DATA0);
				break;
			case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
				msg_rx[0] = (status & RXRSS0R_DATA0);
				msg_rx[1] = (status & RXRSS0R_DATA1) >> 8;
				break;
			case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
			case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
				size = (status & (RXRSS0R_DATA0 | RXRSS0R_DATA1));
				count = ((size - 1) / 4) + 1;
				/* Read RX Packet Payload Data */
				for (i = 0; i < count; i++) {
					rx_data = rzg2l_mipi_dsi_read(mipi_dsi->link_mmio,
								      RXPPDxR(i));
					for (j = 0; j < 4; j++) {
						if (size == (4 * i + j))
							break;
						msg_rx[4 * i + j] = (rx_data >> (8 * j)) & 0xff;
					}
				}
				break;
			default:
				dev_err(mipi_dsi->dev,
					"unhandled response type: %02x\n",
					datatype & 0x3f);

				err = -EPROTO;
				goto stop_sequence;
			}
		} else {
			err = -EPROTO;
			goto stop_sequence;
		}
	}

	err = 1;

stop_sequence:
	/* Stop Sequence 0 Operation */
	rzg2l_mipi_dsi_write(mipi_dsi->link_mmio, SQCH0SET0R,
			rzg2l_mipi_dsi_read(mipi_dsi->link_mmio, SQCH0SET0R) &
			(~SQCH0SET0R_START));

	return err;
}

static const struct mipi_dsi_host_ops rzg2l_mipi_dsi_host_ops = {
	.attach = rzg2l_mipi_dsi_host_attach,
	.detach = rzg2l_mipi_dsi_host_detach,
	.transfer = rzg2l_mipi_dsi_host_transfer,
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */
static int rzg2l_mipi_dsi_find_panel_or_bridge(struct rzg2l_mipi_dsi *mipi_dsi)
{
	struct device_node *local_output = NULL;
	struct device_node *remote_input = NULL;
	struct device_node *remote = NULL;
	struct device_node *node;
	bool is_bridge = false;
	int ret = 0;

	local_output = of_graph_get_endpoint_by_regs(mipi_dsi->dev->of_node,
						     1, 0);
	if (!local_output) {
		dev_dbg(mipi_dsi->dev, "unconnected port@1\n");
		ret = -ENODEV;
		goto done;
	}

	/*
	 * Locate the connected entity and
	 * infer its type from the number of endpoints.
	 */
	remote = of_graph_get_remote_port_parent(local_output);
	if (!remote) {
		dev_dbg(mipi_dsi->dev, "unconnected endpoint %pOF\n",
		local_output);
		ret = -ENODEV;
		goto done;
	}

	if (!of_device_is_available(remote)) {
		dev_dbg(mipi_dsi->dev, "connected entity %pOF is disabled\n",
		remote);
		ret = -ENODEV;
		goto done;
	}

	remote_input = of_graph_get_remote_endpoint(local_output);

	for_each_endpoint_of_node(remote, node) {
		if (node != remote_input) {
			/*
			 * The endpoint which is not input node must be bridge
			 */
			is_bridge = true;
			of_node_put(node);
			break;
		}
	}

	if (is_bridge) {
		mipi_dsi->next_bridge = of_drm_find_bridge(remote);
		if (!mipi_dsi->next_bridge) {
			ret = -EPROBE_DEFER;
			goto done;
		}
	} else {
		mipi_dsi->panel = of_drm_find_panel(remote);
		if (IS_ERR(mipi_dsi->panel)) {
			ret = PTR_ERR(mipi_dsi->panel);
			goto done;
		}
	}

done:
	of_node_put(local_output);
	of_node_put(remote_input);
	of_node_put(remote);

	return ret;
}

static struct clk *rzg2l_mipi_dsi_get_clock(struct rzg2l_mipi_dsi *mipi_dsi,
					    const char *name)
{
	struct clk *clk;

	clk = devm_clk_get(mipi_dsi->dev, name);
	if (IS_ERR(clk))
		dev_err(mipi_dsi->dev, "failed to get %s clock\n", name);

	return clk;
}

static int rzg2l_mipi_dsi_get_clk(struct rzg2l_mipi_dsi *mipi_dsi)
{
	mipi_dsi->clocks.pllclk = rzg2l_mipi_dsi_get_clock(mipi_dsi, "pllclk");
	if (IS_ERR(mipi_dsi->clocks.pllclk))
		return PTR_ERR(mipi_dsi->clocks.pllclk);

	mipi_dsi->clocks.sysclk = rzg2l_mipi_dsi_get_clock(mipi_dsi, "sysclk");
	if (IS_ERR(mipi_dsi->clocks.sysclk))
		return PTR_ERR(mipi_dsi->clocks.sysclk);

	mipi_dsi->clocks.aclk = rzg2l_mipi_dsi_get_clock(mipi_dsi, "aclk");
	if (IS_ERR(mipi_dsi->clocks.aclk))
		return PTR_ERR(mipi_dsi->clocks.aclk);

	mipi_dsi->clocks.pclk = rzg2l_mipi_dsi_get_clock(mipi_dsi, "pclk");
	if (IS_ERR(mipi_dsi->clocks.pclk))
		return PTR_ERR(mipi_dsi->clocks.pclk);

	mipi_dsi->clocks.vclk = rzg2l_mipi_dsi_get_clock(mipi_dsi, "vclk");
	if (IS_ERR(mipi_dsi->clocks.vclk))
		return PTR_ERR(mipi_dsi->clocks.vclk);

	mipi_dsi->clocks.lpclk = rzg2l_mipi_dsi_get_clock(mipi_dsi, "lpclk");
	if (IS_ERR(mipi_dsi->clocks.lpclk))
		return PTR_ERR(mipi_dsi->clocks.lpclk);

	return 0;
}

static struct reset_control *rzg2l_mipi_dsi_get_rstc(struct rzg2l_mipi_dsi *mipi_dsi,
					    const char *name)
{
	struct reset_control *rstc;

	rstc = devm_reset_control_get(mipi_dsi->dev, name);
	if (IS_ERR(rstc))
		dev_err(mipi_dsi->dev, "failed to get %s reset\n", name);

	return rstc;
}

static int rzg2l_mipi_dsi_get_rst(struct rzg2l_mipi_dsi *mipi_dsi)
{
	mipi_dsi->rstc.cmn_rstb = rzg2l_mipi_dsi_get_rstc(mipi_dsi, "cmn_rstb");
	if (IS_ERR(mipi_dsi->rstc.cmn_rstb))
		return PTR_ERR(mipi_dsi->rstc.cmn_rstb);

	mipi_dsi->rstc.areset_n = rzg2l_mipi_dsi_get_rstc(mipi_dsi, "areset_n");
	if (IS_ERR(mipi_dsi->rstc.areset_n))
		return PTR_ERR(mipi_dsi->rstc.areset_n);

	mipi_dsi->rstc.preset_n = rzg2l_mipi_dsi_get_rstc(mipi_dsi, "preset_n");
	if (IS_ERR(mipi_dsi->rstc.preset_n))
		return PTR_ERR(mipi_dsi->rstc.preset_n);

	return 0;
}

static int rzg2l_mipi_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzg2l_mipi_dsi *mipi_dsi;
	struct resource *mem;
	int ret;

	mipi_dsi = devm_kzalloc(&pdev->dev, sizeof(*mipi_dsi), GFP_KERNEL);
	if (mipi_dsi == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, mipi_dsi);
	mipi_dsi->dev = dev;

	/* Init bridge */
	mipi_dsi->bridge.driver_private = mipi_dsi;
	mipi_dsi->bridge.funcs = &rzg2l_mipi_dsi_bridge_ops;
	mipi_dsi->bridge.of_node = pdev->dev.of_node;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mipi_dsi->link_mmio = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(mipi_dsi->link_mmio))
		return PTR_ERR(mipi_dsi->link_mmio);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mipi_dsi->phy_mmio = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(mipi_dsi->phy_mmio))
		return PTR_ERR(mipi_dsi->phy_mmio);

	ret = rzg2l_mipi_dsi_get_clk(mipi_dsi);
	if (ret < 0)
		return ret;

	ret = rzg2l_mipi_dsi_get_rst(mipi_dsi);
	if (ret < 0)
		return ret;

	/* Init host device */
	mipi_dsi->host.dev = dev;
	mipi_dsi->host.ops = &rzg2l_mipi_dsi_host_ops;
	ret = mipi_dsi_host_register(&mipi_dsi->host);
	if (ret < 0)
		return ret;

	drm_bridge_add(&mipi_dsi->bridge);

	return 0;
};

static int rzg2l_mipi_dsi_remove(struct platform_device *pdev)
{
	struct rzg2l_mipi_dsi *mipi_dsi = platform_get_drvdata(pdev);

	drm_bridge_remove(&mipi_dsi->bridge);

	mipi_dsi_host_unregister(&mipi_dsi->host);

	return 0;
}

static const struct of_device_id rzg2l_mipi_dsi_of_table[] = {
	{ .compatible = "renesas,r9a07g044-mipi-dsi" },
	{ },
};

static struct platform_driver rzg2l_mipi_dsi_platform_driver = {
	.probe	= rzg2l_mipi_dsi_probe,
	.remove	= rzg2l_mipi_dsi_remove,
	.driver	= {
		.name = "rzg2l-mipi-dsi",
		.of_match_table = rzg2l_mipi_dsi_of_table,
	},
};

module_platform_driver(rzg2l_mipi_dsi_platform_driver);

MODULE_AUTHOR("Hien Huynh <hien.huynh.px@renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/G2L MIPI DSI Encoder Driver");
MODULE_LICENSE("GPL");
