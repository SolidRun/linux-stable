// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G2L MIPI DSI Encoder Driver
 *
 * Copyright (C) 2022 Renesas Electronics Corporation
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/units.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include "rzg2l_mipi_dsi_regs.h"
#include "../../../../clk/renesas/rzg2l-cpg.h"

int dsi_div_ab;

#define RZG2L_MIPI_DSI_MAX_INPUT		2

#define RZV2H_MIPI_DPHY_OSC_CLK_IN_MEGA		(24)
#define RZV2H_MIPI_DPHY_FVCO_MIN_IN_MEGA	(1050)
#define RZV2H_MIPI_DPHY_FVCO_MAX_IN_MEGA	(2100)
#define RZV2H_MIPI_DPHY_FOUT_MIN_IN_MEGA	(80)
#define RZV2H_MIPI_DPHY_FOUT_MAX_IN_MEGA	(1500)
#define RZV2H_MIPI_DPHY_PLL_M_MIN		(64)
#define RZV2H_MIPI_DPHY_PLL_M_MAX		(1023)
#define RZV2H_MIPI_DPHY_PLL_P_MIN		(1)
#define RZV2H_MIPI_DPHY_PLL_P_MAX		(4)
#define RZV2H_MIPI_DPHY_PLL_S_MIN		(0)
#define RZV2H_MIPI_DPHY_PLL_S_MAX		(5)

struct rzg2l_mipi_dsi;

enum mipi_dsi_dphy_type {
	MIPI_DSI_DPHY_RZG2L,
	MIPI_DSI_DPHY_RZV2H,
};

struct rzg2l_mipi_dsi_hw_info {
	enum mipi_dsi_dphy_type type;
	bool has_dphy_rstc;
	int (*dphy_init)(struct rzg2l_mipi_dsi *dsi, unsigned long hsfreq);
	void (*dphy_exit)(struct rzg2l_mipi_dsi *dsi);
	u32 phy_reg_offset;
	u32 link_reg_offset;
	u8 vclk_input;
	unsigned long max_dclk;
	unsigned long min_dclk;
};

struct rzg2l_mipi_dsi {
	struct device *dev;
	void __iomem *mmio;

	struct reset_control *rstc;
	struct reset_control *arstc;
	struct reset_control *prstc;

	struct mipi_dsi_host host;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;

	const struct rzg2l_mipi_dsi_hw_info *info;

	struct clk *vclk[RZG2L_MIPI_DSI_MAX_INPUT];
	struct clk *lpclk;

	enum mipi_dsi_pixel_format format;
	unsigned int num_data_lanes;
	unsigned int lanes;
	unsigned long mode_flags;
	unsigned long hsfreq;
	u8 vclk_input;
};

static inline struct rzg2l_mipi_dsi *
bridge_to_rzg2l_mipi_dsi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct rzg2l_mipi_dsi, bridge);
}

static inline struct rzg2l_mipi_dsi *
host_to_rzg2l_mipi_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct rzg2l_mipi_dsi, host);
}

struct rzg2l_mipi_dsi_timings {
	unsigned long hsfreq_max;
	u32 t_init;
	u32 tclk_prepare;
	u32 ths_prepare;
	u32 tclk_zero;
	u32 tclk_pre;
	u32 tclk_post;
	u32 tclk_trail;
	u32 ths_zero;
	u32 ths_trail;
	u32 ths_exit;
	u32 tlpx;
};

static const struct rzg2l_mipi_dsi_timings rzg2l_mipi_dsi_global_timings[] = {
	{
		.hsfreq_max = 80000,
		.t_init = 79801,
		.tclk_prepare = 8,
		.ths_prepare = 13,
		.tclk_zero = 33,
		.tclk_pre = 24,
		.tclk_post = 94,
		.tclk_trail = 10,
		.ths_zero = 23,
		.ths_trail = 17,
		.ths_exit = 13,
		.tlpx = 6,
	},
	{
		.hsfreq_max = 125000,
		.t_init = 79801,
		.tclk_prepare = 8,
		.ths_prepare = 12,
		.tclk_zero = 33,
		.tclk_pre = 15,
		.tclk_post = 94,
		.tclk_trail = 10,
		.ths_zero = 23,
		.ths_trail = 17,
		.ths_exit = 13,
		.tlpx = 6,
	},
	{
		.hsfreq_max = 250000,
		.t_init = 79801,
		.tclk_prepare = 8,
		.ths_prepare = 12,
		.tclk_zero = 33,
		.tclk_pre = 13,
		.tclk_post = 94,
		.tclk_trail = 10,
		.ths_zero = 23,
		.ths_trail = 16,
		.ths_exit = 13,
		.tlpx = 6,
	},
	{
		.hsfreq_max = 360000,
		.t_init = 79801,
		.tclk_prepare = 8,
		.ths_prepare = 10,
		.tclk_zero = 33,
		.tclk_pre = 4,
		.tclk_post = 35,
		.tclk_trail = 7,
		.ths_zero = 16,
		.ths_trail = 9,
		.ths_exit = 13,
		.tlpx = 6,
	},
	{
		.hsfreq_max = 720000,
		.t_init = 79801,
		.tclk_prepare = 8,
		.ths_prepare = 9,
		.tclk_zero = 33,
		.tclk_pre = 4,
		.tclk_post = 35,
		.tclk_trail = 7,
		.ths_zero = 16,
		.ths_trail = 9,
		.ths_exit = 13,
		.tlpx = 6,
	},
	{
		.hsfreq_max = 1500000,
		.t_init = 79801,
		.tclk_prepare = 8,
		.ths_prepare = 9,
		.tclk_zero = 33,
		.tclk_pre = 4,
		.tclk_post = 35,
		.tclk_trail = 7,
		.ths_zero = 16,
		.ths_trail = 9,
		.ths_exit = 13,
		.tlpx = 6,
	},
};

struct rzv2h_mipi_dsi_timings {
	unsigned long hsfreq;
	u32 value;
};

static const struct rzv2h_mipi_dsi_timings TCLKPRPRCTL[] = {
	{       0,  0 },
	{  150000,  1 },
	{  260000,  2 },
	{  370000,  3 },
	{  470000,  4 },
	{  580000,  5 },
	{  690000,  6 },
	{  790000,  7 },
	{  900000,  8 },
	{ 1010000,  9 },
	{ 1110000, 10 },
	{ 1220000, 11 },
	{ 1330000, 12 },
	{ 1430000, 13 },
};

static const struct rzv2h_mipi_dsi_timings TCLKZEROCTL[] = {
	{       0,  2 },
	{   90000,  3 },
	{  110000,  4 },
	{  130000,  5 },
	{  150000,  6 },
	{  180000,  7 },
	{  210000,  8 },
	{  230000,  9 },
	{  240000, 10 },
	{  250000, 11 },
	{  270000, 12 },
	{  290000, 13 },
	{  310000, 14 },
	{  340000, 15 },
	{  360000, 16 },
	{  380000, 17 },
	{  410000, 18 },
	{  430000, 19 },
	{  450000, 20 },
	{  470000, 21 },
	{  500000, 22 },
	{  520000, 23 },
	{  540000, 24 },
	{  570000, 25 },
	{  590000, 26 },
	{  610000, 27 },
	{  630000, 28 },
	{  660000, 29 },
	{  680000, 30 },
	{  700000, 31 },
	{  730000, 32 },
	{  750000, 33 },
	{  770000, 34 },
	{  790000, 35 },
	{  820000, 36 },
	{  840000, 37 },
	{  860000, 38 },
	{  890000, 39 },
	{  910000, 40 },
	{  930000, 41 },
	{  950000, 42 },
	{  980000, 43 },
	{ 1000000, 44 },
	{ 1020000, 45 },
	{ 1050000, 46 },
	{ 1070000, 47 },
	{ 1090000, 48 },
	{ 1110000, 49 },
	{ 1140000, 50 },
	{ 1160000, 51 },
	{ 1180000, 52 },
	{ 1210000, 53 },
	{ 1230000, 54 },
	{ 1250000, 55 },
	{ 1270000, 56 },
	{ 1300000, 57 },
	{ 1320000, 58 },
	{ 1340000, 59 },
	{ 1370000, 60 },
	{ 1390000, 61 },
	{ 1410000, 62 },
	{ 1430000, 63 },
	{ 1460000, 64 },
	{ 1480000, 65 },
};

static const struct rzv2h_mipi_dsi_timings TCLKPOSTCTL[] = {
	{       0,  6 },
	{   80000,  7 },
	{  210000,  8 },
	{  340000,  9 },
	{  480000, 10 },
	{  610000, 11 },
	{  740000, 12 },
	{  880000, 13 },
	{ 1010000, 14 },
	{ 1140000, 15 },
	{ 1280000, 16 },
	{ 1410000, 17 },
};

static const struct rzv2h_mipi_dsi_timings TCLKTRAILCTL[] = {
	{       0,  1 },
	{  140000,  2 },
	{  250000,  3 },
	{  370000,  4 },
	{  480000,  5 },
	{  590000,  6 },
	{  710000,  7 },
	{  820000,  8 },
	{  940000,  9 },
	{ 1050000, 10 },
	{ 1170000, 11 },
	{ 1280000, 12 },
	{ 1390000, 13 },
};

static const struct rzv2h_mipi_dsi_timings THSPRPRCTL[] = {
	{       0,  0 },
	{  110000,  1 },
	{  190000,  2 },
	{  290000,  3 },
	{  400000,  4 },
	{  500000,  5 },
	{  610000,  6 },
	{  720000,  7 },
	{  820000,  8 },
	{  930000,  9 },
	{ 1030000, 10 },
	{ 1140000, 11 },
	{ 1250000, 12 },
	{ 1350000, 13 },
	{ 1460000, 14 },
};

static const struct rzv2h_mipi_dsi_timings THSZEROCTL[] = {
	{       0,  0 },
	{  180000,  1 },
	{  240000,  2 },
	{  290000,  3 },
	{  350000,  4 },
	{  400000,  5 },
	{  460000,  6 },
	{  510000,  7 },
	{  570000,  8 },
	{  620000,  9 },
	{  680000, 10 },
	{  730000, 11 },
	{  790000, 12 },
	{  840000, 13 },
	{  900000, 14 },
	{  950000, 15 },
	{ 1010000, 16 },
	{ 1060000, 17 },
	{ 1120000, 18 },
	{ 1170000, 19 },
	{ 1230000, 20 },
	{ 1280000, 21 },
	{ 1340000, 22 },
	{ 1390000, 23 },
	{ 1450000, 24 },
};

static const struct rzv2h_mipi_dsi_timings THSTRAILCTL[] = {
	{       0,  3 },
	{  100000,  4 },
	{  210000,  5 },
	{  320000,  6 },
	{  420000,  7 },
	{  530000,  8 },
	{  640000,  9 },
	{  750000, 10 },
	{  850000, 11 },
	{  960000, 12 },
	{ 1070000, 13 },
	{ 1180000, 14 },
	{ 1280000, 15 },
	{ 1390000, 16 },
};

static const struct rzv2h_mipi_dsi_timings TLPXCTL[] = {
	{       0,  0 },
	{  130000,  1 },
	{  260000,  2 },
	{  390000,  3 },
	{  530000,  4 },
	{  660000,  5 },
	{  790000,  6 },
	{  930000,  7 },
	{ 1060000,  8 },
	{ 1190000,  9 },
	{ 1330000, 10 },
	{ 1460000, 11 },
};

static const struct rzv2h_mipi_dsi_timings THSEXITCTL[] = {
	{       0,  1 },
	{  150000,  2 },
	{  230000,  3 },
	{  310000,  4 },
	{  390000,  5 },
	{  470000,  6 },
	{  550000,  7 },
	{  630000,  8 },
	{  710000,  9 },
	{  790000, 10 },
	{  870000, 11 },
	{  950000, 12 },
	{ 1030000, 13 },
	{ 1110000, 14 },
};

static const struct rzv2h_mipi_dsi_timings ULPSEXIT[] = {
	{    10000,   0 },
	{   100000,   3 },
	{  1000000,  25 },
	{  2000000,  50 },
	{  3000000,  75 },
	{  4000000, 100 },
	{  5000000, 125 },
	{  6000000, 150 },
	{  7000000, 175 },
	{  8000000, 200 },
	{  9000000, 225 },
	{ 10000000, 250 },
	{ 11000000, 275 },
	{ 12000000, 300 },
	{ 13000000, 325 },
	{ 14000000, 350 },
	{ 15000000, 375 },
	{ 16000000, 400 },
	{ 17000000, 425 },
	{ 18000000, 450 },
	{ 19000000, 475 },
	{ 20000000, 500 },
};

static int dphy_find_timings_val(struct rzg2l_mipi_dsi *mipi_dsi,
				 const struct rzv2h_mipi_dsi_timings timings[],
				 int size, unsigned long freq)
{
	int i;

	for (i = 1; i < size; i++) {
		if (freq <= (timings[i].hsfreq * 1000)) {
			i = (mipi_dsi->hsfreq < (timings[i].hsfreq * 1000)) ?
			    (i - 1) : i;
			break;
		}
	}

	return timings[i].value;
};

static void rzg2l_mipi_dsi_phy_write(struct rzg2l_mipi_dsi *dsi, u32 reg, u32 data)
{
	iowrite32(data, dsi->mmio + dsi->info->phy_reg_offset + reg);
}

static void rzg2l_mipi_dsi_link_write(struct rzg2l_mipi_dsi *dsi, u32 reg, u32 data)
{
	iowrite32(data, dsi->mmio + dsi->info->link_reg_offset + reg);
}

static u32 rzg2l_mipi_dsi_phy_read(struct rzg2l_mipi_dsi *dsi, u32 reg)
{
	return ioread32(dsi->mmio + dsi->info->phy_reg_offset + reg);
}

static u32 rzg2l_mipi_dsi_link_read(struct rzg2l_mipi_dsi *dsi, u32 reg)
{
	return ioread32(dsi->mmio + dsi->info->link_reg_offset + reg);
}

/* -----------------------------------------------------------------------------
 * Hardware Setup
 */

static int rzg2l_mipi_dsi_dphy_init(struct rzg2l_mipi_dsi *dsi,
				    unsigned long hsfreq)
{
	const struct rzg2l_mipi_dsi_timings *dphy_timings;
	unsigned int i;
	u32 dphyctrl0;
	u32 dphytim0;
	u32 dphytim1;
	u32 dphytim2;
	u32 dphytim3;
	int ret;

	/* All DSI global operation timings are set with recommended setting */
	for (i = 0; i < ARRAY_SIZE(rzg2l_mipi_dsi_global_timings); ++i) {
		dphy_timings = &rzg2l_mipi_dsi_global_timings[i];
		if (hsfreq <= (dphy_timings->hsfreq_max * 1000))
			break;
	}

	/* Initializing DPHY before accessing LINK */
	dphyctrl0 = DSIDPHYCTRL0_CAL_EN_HSRX_OFS | DSIDPHYCTRL0_CMN_MASTER_EN |
		    DSIDPHYCTRL0_RE_VDD_DETVCCQLV18 | DSIDPHYCTRL0_EN_BGR;

	rzg2l_mipi_dsi_phy_write(dsi, DSIDPHYCTRL0, dphyctrl0);
	usleep_range(20, 30);

	dphyctrl0 |= DSIDPHYCTRL0_EN_LDO1200;
	rzg2l_mipi_dsi_phy_write(dsi, DSIDPHYCTRL0, dphyctrl0);
	usleep_range(10, 20);

	dphytim0 = DSIDPHYTIM0_TCLK_MISS(0) |
		   DSIDPHYTIM0_T_INIT(dphy_timings->t_init);
	dphytim1 = DSIDPHYTIM1_THS_PREPARE(dphy_timings->ths_prepare) |
		   DSIDPHYTIM1_TCLK_PREPARE(dphy_timings->tclk_prepare) |
		   DSIDPHYTIM1_THS_SETTLE(0) |
		   DSIDPHYTIM1_TCLK_SETTLE(0);
	dphytim2 = DSIDPHYTIM2_TCLK_TRAIL(dphy_timings->tclk_trail) |
		   DSIDPHYTIM2_TCLK_POST(dphy_timings->tclk_post) |
		   DSIDPHYTIM2_TCLK_PRE(dphy_timings->tclk_pre) |
		   DSIDPHYTIM2_TCLK_ZERO(dphy_timings->tclk_zero);
	dphytim3 = DSIDPHYTIM3_TLPX(dphy_timings->tlpx) |
		   DSIDPHYTIM3_THS_EXIT(dphy_timings->ths_exit) |
		   DSIDPHYTIM3_THS_TRAIL(dphy_timings->ths_trail) |
		   DSIDPHYTIM3_THS_ZERO(dphy_timings->ths_zero);

	rzg2l_mipi_dsi_phy_write(dsi, DSIDPHYTIM0, dphytim0);
	rzg2l_mipi_dsi_phy_write(dsi, DSIDPHYTIM1, dphytim1);
	rzg2l_mipi_dsi_phy_write(dsi, DSIDPHYTIM2, dphytim2);
	rzg2l_mipi_dsi_phy_write(dsi, DSIDPHYTIM3, dphytim3);

	ret = reset_control_deassert(dsi->rstc);
	if (ret < 0)
		return ret;

	udelay(1);

	return 0;
}

static void rzg2l_mipi_dsi_dphy_exit(struct rzg2l_mipi_dsi *dsi)
{
	u32 dphyctrl0;

	dphyctrl0 = rzg2l_mipi_dsi_phy_read(dsi, DSIDPHYCTRL0);

	dphyctrl0 &= ~(DSIDPHYCTRL0_EN_LDO1200 | DSIDPHYCTRL0_EN_BGR);
	rzg2l_mipi_dsi_phy_write(dsi, DSIDPHYCTRL0, dphyctrl0);

	reset_control_assert(dsi->rstc);
}

static int rzv2h_mipi_dsi_dphy_init(struct rzg2l_mipi_dsi *dsi,
				    unsigned long hsfreq)
{
	struct rzg2l_mipi_dsi_timings dphy_timings;
	u32 ulpsexit;
	u32 phytclksetr, phythssetr, phytlpxsetr, phycr;
	unsigned long lpclk_rate = clk_get_rate(dsi->lpclk);
	int pll_k;
	int pll_s, pll_m, pll_p;
	unsigned long fout, fvco, osc;

	dsi->hsfreq = hsfreq;

	dphy_timings.tclk_trail = dphy_find_timings_val(dsi,
					TCLKTRAILCTL,
					ARRAY_SIZE(TCLKTRAILCTL),
					hsfreq);
	dphy_timings.tclk_post = dphy_find_timings_val(dsi,
					TCLKPOSTCTL,
					ARRAY_SIZE(TCLKPOSTCTL),
					hsfreq);
	dphy_timings.tclk_zero = dphy_find_timings_val(dsi,
					TCLKZEROCTL,
					ARRAY_SIZE(TCLKZEROCTL),
					hsfreq);
	dphy_timings.tclk_prepare = dphy_find_timings_val(dsi,
					TCLKPRPRCTL,
					ARRAY_SIZE(TCLKPRPRCTL),
					hsfreq);
	dphy_timings.ths_exit = dphy_find_timings_val(dsi,
					THSEXITCTL,
					ARRAY_SIZE(THSEXITCTL),
					hsfreq);
	dphy_timings.ths_trail = dphy_find_timings_val(dsi,
					THSTRAILCTL,
					ARRAY_SIZE(THSTRAILCTL),
					hsfreq);
	dphy_timings.ths_zero =  dphy_find_timings_val(dsi,
					THSZEROCTL,
					ARRAY_SIZE(THSZEROCTL),
					hsfreq);
	dphy_timings.ths_prepare = dphy_find_timings_val(dsi,
					THSPRPRCTL,
					ARRAY_SIZE(THSPRPRCTL),
					hsfreq);
	dphy_timings.tlpx = dphy_find_timings_val(dsi,
					TLPXCTL,
					ARRAY_SIZE(TLPXCTL),
					hsfreq);
	ulpsexit = dphy_find_timings_val(dsi,
					 ULPSEXIT,
					 ARRAY_SIZE(ULPSEXIT),
					 lpclk_rate);

	phytclksetr = PHYTCLKSETR_TCLKTRAILCTL(dphy_timings.tclk_trail) |
		      PHYTCLKSETR_TCLKPOSTCTL(dphy_timings.tclk_post) |
		      PHYTCLKSETR_TCLKZEROCTL(dphy_timings.tclk_zero) |
		      PHYTCLKSETR_TCLKPRPRCTL(dphy_timings.tclk_prepare);
	phythssetr = PHYTHSSETR_THSEXITCTL(dphy_timings.ths_exit) |
		     PHYTHSSETR_THSTRAILCTL(dphy_timings.ths_trail) |
		     PHYTHSSETR_THSZEROCTL(dphy_timings.ths_zero) |
		     PHYTHSSETR_THSPRPRCTL(dphy_timings.ths_prepare);
	phytlpxsetr = rzg2l_mipi_dsi_phy_read(dsi, PHYTLPXSETR) & ~GENMASK(7, 0);
	phytlpxsetr |= PHYTLPXSETR_TLPXCTL(dphy_timings.tlpx);
	phycr = rzg2l_mipi_dsi_phy_read(dsi, PHYCR) & ~GENMASK(9, 0);
	phycr |= PHYCR_ULPSEXIT(ulpsexit);

	/* Setting all D-PHY Timings Registers */
	rzg2l_mipi_dsi_phy_write(dsi, PHYTCLKSETR, phytclksetr);
	rzg2l_mipi_dsi_phy_write(dsi, PHYTHSSETR, phythssetr);
	rzg2l_mipi_dsi_phy_write(dsi, PHYTLPXSETR, phytlpxsetr);
	rzg2l_mipi_dsi_phy_write(dsi, PHYCR, phycr);

	/* Setting all PLL Registers */
	osc = RZV2H_MIPI_DPHY_OSC_CLK_IN_MEGA * MEGA;
	for (pll_p = RZV2H_MIPI_DPHY_PLL_P_MAX;
	     pll_p >= RZV2H_MIPI_DPHY_PLL_P_MIN; pll_p--) {
		for (pll_s = RZV2H_MIPI_DPHY_PLL_S_MAX;
		     pll_s >= RZV2H_MIPI_DPHY_PLL_S_MIN; pll_s--) {
			/* Check FOUT condition */
			fout = hsfreq;
			if ((fout > (RZV2H_MIPI_DPHY_FOUT_MAX_IN_MEGA * MEGA)) ||
			    (fout < (RZV2H_MIPI_DPHY_FOUT_MIN_IN_MEGA * MEGA)))
				continue;

			/* Check FVCO condition */
			fvco = fout * (1 << pll_s);
			if ((fvco > (RZV2H_MIPI_DPHY_FVCO_MAX_IN_MEGA * MEGA)) ||
			    (fvco < (RZV2H_MIPI_DPHY_FVCO_MIN_IN_MEGA * MEGA)))
				continue;

			pll_m = (fvco * pll_p) / osc;
			pll_k = (fvco * pll_p) % osc;

			/* Check available range of PLL_K */
			if (pll_k >= (osc / 2)) {
				pll_m++;
				pll_k = pll_k - osc;
			}

			/* Check available range of PLL_M */
			if ((pll_m < RZV2H_MIPI_DPHY_PLL_M_MIN) ||
			    (pll_m > RZV2H_MIPI_DPHY_PLL_M_MAX))
				continue;

			pll_k = DIV_S64_ROUND_CLOSEST(((s64)pll_k << 16), osc);

			goto found;
		}
	}

	dev_err(dsi->dev,
		"Not found pll setting for %lu (Hz)\n", hsfreq);
	return -EINVAL;

found:
	dev_dbg(dsi->dev,
		"hsfreq:%lu pll_k: %hd, pll_m: %d, pll_p: %d, pll_s: %d\n",
		hsfreq, pll_k, pll_m, pll_p, pll_s);

	rzg2l_mipi_dsi_phy_write(dsi, PLLCLKSET0R,
				 PLLCLKSET0R_PLL_S(pll_s) |
				 PLLCLKSET0R_PLL_P(pll_p) |
				 PLLCLKSET0R_PLL_M(pll_m));
	rzg2l_mipi_dsi_phy_write(dsi, PLLCLKSET1R, PLLCLKSET1R_PLL_K(pll_k));
	udelay(20);

	rzg2l_mipi_dsi_phy_write(dsi, PLLENR, PLLENR_PLLEN);
	udelay(500);

	return 0;
}

static void rzv2h_mipi_dsi_dphy_exit(struct rzg2l_mipi_dsi *dsi)
{
	rzg2l_mipi_dsi_phy_write(dsi, PLLENR, 0);
}

static int rzg2l_mipi_dsi_startup(struct rzg2l_mipi_dsi *dsi,
				  const struct drm_display_mode *mode)
{
	unsigned long hsfreq, vclk_rate;
	unsigned int bpp;
	u32 txsetr;
	u32 clstptsetr;
	u32 lptrnstsetr;
	u32 clkkpt;
	u32 clkbfht;
	u32 clkstpt;
	u32 golpbkt;
	int ret;

	clk_set_rate(dsi->vclk[dsi->vclk_input], mode->clock * 1000);
	/* To get the precise vclk for MIPI DPHY PLL calculation */
	vclk_rate = clk_get_rate(dsi->vclk[dsi->vclk_input]);

	/*
	 * Relationship between hsclk and vclk must follow
	 * vclk * bpp = hsclk * 8 * lanes
	 * where vclk: video clock (Hz)
	 *       bpp: video pixel bit depth
	 *       hsclk: DSI HS Byte clock frequency (Hz)
	 *       lanes: number of data lanes
	 *
	 * hsclk(bit) = hsclk(byte) * 8
	 */
	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	hsfreq = DIV_ROUND_CLOSEST_ULL(vclk_rate * bpp * 8, 8 * dsi->lanes);

	ret = pm_runtime_resume_and_get(dsi->dev);
	if (ret < 0)
		return ret;

	clk_set_rate(dsi->vclk[dsi->vclk_input], mode->clock * 1000);

	ret = dsi->info->dphy_init(dsi, hsfreq);
	if (ret < 0)
		goto err_phy;

	/* Set VCLK input clock for MIPI DSI */
	if (dsi->info->vclk_input > 1)
		rzg2l_mipi_dsi_link_write(dsi, GPO0R, dsi->vclk_input);

	/* Enable Data lanes and Clock lanes */
	txsetr = TXSETR_DLEN | TXSETR_NUMLANEUSE(dsi->lanes - 1) | TXSETR_CLEN;
	rzg2l_mipi_dsi_link_write(dsi, TXSETR, txsetr);

	if (dsi->info->type == MIPI_DSI_DPHY_RZV2H) {
		udelay(250);
		rzg2l_mipi_dsi_phy_write(dsi, PHYRSTR, PHYRSTR_PHYMRSTN);
	}

	/*
	 * Global timings characteristic depends on high speed Clock Frequency
	 * Currently MIPI DSI-IF just supports maximum FHD@60 with:
	 * - videoclock = 148.5 (MHz)
	 * - bpp: maximum 24bpp
	 * - data lanes: maximum 4 lanes
	 * Therefore maximum hsclk will be 891 Mbps.
	 */
	if (hsfreq > 445500000) {
		clkkpt = 12;
		clkbfht = 15;
		clkstpt = 48;
		golpbkt = 75;
	} else if (hsfreq > 250000000) {
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
	rzg2l_mipi_dsi_link_write(dsi, CLSTPTSETR, clstptsetr);

	lptrnstsetr = LPTRNSTSETR_GOLPBKT(golpbkt);
	rzg2l_mipi_dsi_link_write(dsi, LPTRNSTSETR, lptrnstsetr);

	return 0;

err_phy:
	dsi->info->dphy_exit(dsi);
	pm_runtime_put(dsi->dev);

	return ret;
}

static void rzg2l_mipi_dsi_stop(struct rzg2l_mipi_dsi *dsi)
{
	dsi->info->dphy_exit(dsi);
	pm_runtime_put(dsi->dev);
}

static void rzg2l_mipi_dsi_set_display_timing(struct rzg2l_mipi_dsi *dsi,
					      const struct drm_display_mode *mode)
{
	u32 vich1ppsetr;
	u32 vich1vssetr;
	u32 vich1vpsetr;
	u32 vich1hssetr;
	u32 vich1hpsetr;
	int dsi_format;
	u32 delay[2];
	u8 index;

	/* Configuration for Pixel Packet */
	dsi_format = mipi_dsi_pixel_format_to_bpp(dsi->format);
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
	}

	if ((dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) &&
	    !(dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST))
		vich1ppsetr |= VICH1PPSETR_TXESYNC_PULSE;

	rzg2l_mipi_dsi_link_write(dsi, VICH1PPSETR, vich1ppsetr);

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

	rzg2l_mipi_dsi_link_write(dsi, VICH1VSSETR, vich1vssetr);
	rzg2l_mipi_dsi_link_write(dsi, VICH1VPSETR, vich1vpsetr);
	rzg2l_mipi_dsi_link_write(dsi, VICH1HSSETR, vich1hssetr);
	rzg2l_mipi_dsi_link_write(dsi, VICH1HPSETR, vich1hpsetr);

	/*
	 * Configuration for Delay Value
	 * Delay value based on 2 ranges of video clock.
	 * 74.25MHz is videoclock of HD@60p or FHD@30p
	 */
	if (mode->clock > 74250) {
		delay[0] = 231;
		delay[1] = 216;
	} else {
		delay[0] = 220;
		delay[1] = 212;
	}

	if (dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		index = 0;
	else
		index = 1;

	rzg2l_mipi_dsi_link_write(dsi, VICH1SET1R,
				  VICH1SET1R_DLY(delay[index]));
}

static int rzg2l_mipi_dsi_start_hs_clock(struct rzg2l_mipi_dsi *dsi)
{
	bool is_clk_cont;
	u32 hsclksetr;
	u32 status;
	int ret;

	is_clk_cont = !(dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS);

	/* Start HS clock */
	hsclksetr = HSCLKSETR_HSCLKRUN_HS | (is_clk_cont ?
					     HSCLKSETR_HSCLKMODE_CONT :
					     HSCLKSETR_HSCLKMODE_NON_CONT);
	rzg2l_mipi_dsi_link_write(dsi, HSCLKSETR, hsclksetr);

	if (is_clk_cont) {
		ret = read_poll_timeout(rzg2l_mipi_dsi_link_read, status,
					status & PLSR_CLLP2HS,
					2000, 20000, false, dsi, PLSR);
		if (ret < 0) {
			dev_err(dsi->dev, "failed to start HS clock\n");
			return ret;
		}
	}

	dev_dbg(dsi->dev, "Start High Speed Clock with %s clock mode",
		is_clk_cont ? "continuous" : "non-continuous");

	return 0;
}

static int rzg2l_mipi_dsi_stop_hs_clock(struct rzg2l_mipi_dsi *dsi)
{
	bool is_clk_cont;
	u32 status;
	int ret;

	is_clk_cont = !(dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS);

	/* Stop HS clock */
	rzg2l_mipi_dsi_link_write(dsi, HSCLKSETR,
				  is_clk_cont ? HSCLKSETR_HSCLKMODE_CONT :
				  HSCLKSETR_HSCLKMODE_NON_CONT);

	if (is_clk_cont) {
		ret = read_poll_timeout(rzg2l_mipi_dsi_link_read, status,
					status & PLSR_CLHS2LP,
					2000, 20000, false, dsi, PLSR);
		if (ret < 0) {
			dev_err(dsi->dev, "failed to stop HS clock\n");
			return ret;
		}
	}

	return 0;
}

static int rzg2l_mipi_dsi_start_video(struct rzg2l_mipi_dsi *dsi)
{
	u32 vich1set0r;
	u32 status;
	int ret;

	/* Configuration for Blanking sequence and start video input*/
	vich1set0r = VICH1SET0R_HFPNOLP | VICH1SET0R_HBPNOLP |
		     VICH1SET0R_HSANOLP | VICH1SET0R_VSTART;
	rzg2l_mipi_dsi_link_write(dsi, VICH1SET0R, vich1set0r);

	ret = read_poll_timeout(rzg2l_mipi_dsi_link_read, status,
				status & VICH1SR_VIRDY,
				2000, 20000, false, dsi, VICH1SR);
	if (ret < 0)
		dev_err(dsi->dev, "Failed to start video signal input\n");

	return ret;
}

static int rzg2l_mipi_dsi_stop_video(struct rzg2l_mipi_dsi *dsi)
{
	u32 status;
	int ret;

	rzg2l_mipi_dsi_link_write(dsi, VICH1SET0R, VICH1SET0R_VSTPAFT);
	ret = read_poll_timeout(rzg2l_mipi_dsi_link_read, status,
				(status & VICH1SR_STOP) && (!(status & VICH1SR_RUNNING)),
				2000, 20000, false, dsi, VICH1SR);
	if (ret < 0)
		goto err;

	ret = read_poll_timeout(rzg2l_mipi_dsi_link_read, status,
				!(status & LINKSR_HSBUSY),
				2000, 20000, false, dsi, LINKSR);
	if (ret < 0)
		goto err;

	return 0;

err:
	dev_err(dsi->dev, "Failed to stop video signal input\n");
	return ret;
}

/* -----------------------------------------------------------------------------
 * Bridge
 */

static int rzg2l_mipi_dsi_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct rzg2l_mipi_dsi *dsi = bridge_to_rzg2l_mipi_dsi(bridge);

	return drm_bridge_attach(bridge->encoder, dsi->next_bridge, bridge,
				 flags);
}

static void rzg2l_mipi_dsi_atomic_enable(struct drm_bridge *bridge,
					 struct drm_bridge_state *old_bridge_state)
{
	struct drm_atomic_state *state = old_bridge_state->base.state;
	struct rzg2l_mipi_dsi *dsi = bridge_to_rzg2l_mipi_dsi(bridge);
	const struct drm_display_mode *mode;
	struct drm_connector *connector;
	struct drm_crtc *crtc;
	int ret;

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	crtc = drm_atomic_get_new_connector_state(state, connector)->crtc;
	mode = &drm_atomic_get_new_crtc_state(state, crtc)->adjusted_mode;

	dsi->vclk_input = crtc->index;

	ret = rzg2l_mipi_dsi_startup(dsi, mode);
	if (ret < 0)
		return;

	rzg2l_mipi_dsi_set_display_timing(dsi, mode);

	ret = rzg2l_mipi_dsi_start_hs_clock(dsi);
	if (ret < 0)
		goto err_stop;

	ret = rzg2l_mipi_dsi_start_video(dsi);
	if (ret < 0)
		goto err_stop_clock;

	return;

err_stop_clock:
	rzg2l_mipi_dsi_stop_hs_clock(dsi);
err_stop:
	rzg2l_mipi_dsi_stop(dsi);
}

static void rzg2l_mipi_dsi_atomic_disable(struct drm_bridge *bridge,
					  struct drm_bridge_state *old_bridge_state)
{
	struct rzg2l_mipi_dsi *dsi = bridge_to_rzg2l_mipi_dsi(bridge);


	rzg2l_mipi_dsi_stop_video(dsi);
	rzg2l_mipi_dsi_stop_hs_clock(dsi);
	rzg2l_mipi_dsi_stop(dsi);
}

static enum drm_mode_status
rzg2l_mipi_dsi_bridge_mode_valid(struct drm_bridge *bridge,
				 const struct drm_display_info *info,
				 const struct drm_display_mode *mode)
{
	struct rzg2l_mipi_dsi *dsi = bridge_to_rzg2l_mipi_dsi(bridge);

	if ((dsi->info->max_dclk) && (mode->clock > dsi->info->max_dclk))
		return MODE_CLOCK_HIGH;

	if ((dsi->info->min_dclk) && (mode->clock < dsi->info->min_dclk))
		return MODE_CLOCK_LOW;

	return MODE_OK;
}

static const struct drm_bridge_funcs rzg2l_mipi_dsi_bridge_ops = {
	.attach = rzg2l_mipi_dsi_attach,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_enable = rzg2l_mipi_dsi_atomic_enable,
	.atomic_disable = rzg2l_mipi_dsi_atomic_disable,
	.mode_valid = rzg2l_mipi_dsi_bridge_mode_valid,
};

/* -----------------------------------------------------------------------------
 * Host setting
 */

static int rzg2l_mipi_dsi_host_attach(struct mipi_dsi_host *host,
				      struct mipi_dsi_device *device)
{
	struct rzg2l_mipi_dsi *dsi = host_to_rzg2l_mipi_dsi(host);
	int ret;

	if (device->lanes > dsi->num_data_lanes) {
		dev_err(dsi->dev,
			"Number of lines of device (%u) exceeds host (%u)\n",
			device->lanes, dsi->num_data_lanes);
		return -EINVAL;
	}

	switch (mipi_dsi_pixel_format_to_bpp(device->format)) {
	case 24:
	case 18:
	case 16:
		if ((dsi->info->type == MIPI_DSI_DPHY_RZG2L) &&
		    (mipi_dsi_pixel_format_to_bpp(device->format) == 16))
			dev_err(dsi->dev, "Unsupported format 0x%04x\n",
				device->format);
		break;
	default:
		dev_err(dsi->dev, "Unsupported format 0x%04x\n", device->format);
		return -EINVAL;
	}

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;

	/* Calculate the Final Division ratio setting for the MIPI clock */
	dsi_div_ab = mipi_dsi_pixel_format_to_bpp(dsi->format) / dsi->lanes;

	dsi->next_bridge = devm_drm_of_get_bridge(dsi->dev, dsi->dev->of_node,
						  1, 0);
	if (IS_ERR(dsi->next_bridge)) {
		ret = PTR_ERR(dsi->next_bridge);
		dev_err(dsi->dev, "failed to get next bridge: %d\n", ret);
		return ret;
	}

	drm_bridge_add(&dsi->bridge);

	return 0;
}

static int rzg2l_mipi_dsi_host_detach(struct mipi_dsi_host *host,
				      struct mipi_dsi_device *device)
{
	struct rzg2l_mipi_dsi *dsi = host_to_rzg2l_mipi_dsi(host);

	drm_bridge_remove(&dsi->bridge);

	return 0;
}

static const struct mipi_dsi_host_ops rzg2l_mipi_dsi_host_ops = {
	.attach = rzg2l_mipi_dsi_host_attach,
	.detach = rzg2l_mipi_dsi_host_detach,
};

/* -----------------------------------------------------------------------------
 * Power Management
 */

static int __maybe_unused rzg2l_mipi_pm_runtime_suspend(struct device *dev)
{
	struct rzg2l_mipi_dsi *dsi = dev_get_drvdata(dev);

	reset_control_assert(dsi->prstc);
	reset_control_assert(dsi->arstc);

	return 0;
}

static int __maybe_unused rzg2l_mipi_pm_runtime_resume(struct device *dev)
{
	struct rzg2l_mipi_dsi *dsi = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(dsi->arstc);
	if (ret < 0)
		return ret;

	ret = reset_control_deassert(dsi->prstc);
	if (ret < 0)
		reset_control_assert(dsi->arstc);

	return ret;
}

static const struct dev_pm_ops rzg2l_mipi_pm_ops = {
	SET_RUNTIME_PM_OPS(rzg2l_mipi_pm_runtime_suspend, rzg2l_mipi_pm_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int rzg2l_mipi_dsi_probe(struct platform_device *pdev)
{
	unsigned int num_data_lanes;
	struct rzg2l_mipi_dsi *dsi;
	u32 txsetr;
	int ret, i;
	char clk_name[9];

	dsi = devm_kzalloc(&pdev->dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	platform_set_drvdata(pdev, dsi);
	dsi->dev = &pdev->dev;

	dsi->info = of_device_get_match_data(&pdev->dev);
	if (!dsi->info)
		return dev_err_probe(dsi->dev, ret,
				     "missing data info\n");

	ret = drm_of_get_data_lanes_count_ep(dsi->dev->of_node, 1, 0, 1, 4);
	if (ret < 0)
		return dev_err_probe(dsi->dev, ret,
				     "missing or invalid data-lanes property\n");

	num_data_lanes = ret;

	dsi->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi->mmio))
		return PTR_ERR(dsi->mmio);

	for (i = 0; i < dsi->info->vclk_input; i++) {
		sprintf(clk_name, "vclk%u", i + 1);
		dsi->vclk[i] = devm_clk_get(dsi->dev, clk_name);
		if (IS_ERR(dsi->vclk[i]))
			return PTR_ERR(dsi->vclk[i]);
	}

	dsi->lpclk = devm_clk_get(dsi->dev, "lpclk");
	if (IS_ERR(dsi->lpclk))
		return PTR_ERR(dsi->lpclk);

	if (dsi->info->has_dphy_rstc) {
		dsi->rstc = devm_reset_control_get_exclusive(dsi->dev, "rst");
		if (IS_ERR(dsi->rstc))
			return dev_err_probe(dsi->dev, PTR_ERR(dsi->rstc),
					     "failed to get rst\n");
	}

	dsi->arstc = devm_reset_control_get_exclusive(dsi->dev, "arst");
	if (IS_ERR(dsi->arstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(dsi->arstc),
				     "failed to get arst\n");

	dsi->prstc = devm_reset_control_get_exclusive(dsi->dev, "prst");
	if (IS_ERR(dsi->prstc))
		return dev_err_probe(dsi->dev, PTR_ERR(dsi->prstc),
				     "failed to get prst\n");

	platform_set_drvdata(pdev, dsi);

	pm_runtime_enable(dsi->dev);

	ret = pm_runtime_resume_and_get(dsi->dev);
	if (ret < 0)
		goto err_pm_disable;

	if ((!dsi->info->dphy_init) || (!dsi->info->dphy_exit)) {
		dev_err(dsi->dev, "must have both DPHY Init and Exit funcs\n");
		goto err_pm_disable;
	}

	/*
	 * TXSETR register can be read only after DPHY init. But during probe
	 * mode->clock and format are not available. So initialize DPHY with
	 * timing parameters for 80Mbps.
	 */
	ret = dsi->info->dphy_init(dsi, 80000000);
	if (ret < 0)
		goto err_phy;

	txsetr = rzg2l_mipi_dsi_link_read(dsi, TXSETR);
	dsi->num_data_lanes = min(((txsetr >> 16) & 3) + 1, num_data_lanes);
	dsi->info->dphy_exit(dsi);
	pm_runtime_put(dsi->dev);

	/* Initialize the DRM bridge. */
	dsi->bridge.funcs = &rzg2l_mipi_dsi_bridge_ops;
	dsi->bridge.of_node = dsi->dev->of_node;

	/* Init host device */
	dsi->host.dev = dsi->dev;
	dsi->host.ops = &rzg2l_mipi_dsi_host_ops;
	ret = mipi_dsi_host_register(&dsi->host);
	if (ret < 0)
		goto err_pm_disable;

	return 0;

err_phy:
	dsi->info->dphy_exit(dsi);
	pm_runtime_put(dsi->dev);
err_pm_disable:
	pm_runtime_disable(dsi->dev);
	return ret;
}

static int rzg2l_mipi_dsi_remove(struct platform_device *pdev)
{
	struct rzg2l_mipi_dsi *dsi = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsi->host);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct rzg2l_mipi_dsi_hw_info rzg2l_mipi_dsi_info = {
	.type = MIPI_DSI_DPHY_RZG2L,
	.has_dphy_rstc = 1,
	.dphy_init = rzg2l_mipi_dsi_dphy_init,
	.dphy_exit = rzg2l_mipi_dsi_dphy_exit,
	.phy_reg_offset = 0,
	.link_reg_offset = 0x10000,
	.vclk_input = 1,
	.max_dclk = 148500,
	.min_dclk = 5803,
};

static const struct rzg2l_mipi_dsi_hw_info rzv2h_mipi_dsi_info = {
	.type = MIPI_DSI_DPHY_RZV2H,
	.has_dphy_rstc = 0,
	.dphy_init = rzv2h_mipi_dsi_dphy_init,
	.dphy_exit = rzv2h_mipi_dsi_dphy_exit,
	.phy_reg_offset = 0x10000,
	.link_reg_offset = 0,
	.vclk_input = 1,
	.max_dclk = 187500,
	.min_dclk = 5440,
};

static const struct rzg2l_mipi_dsi_hw_info rzg3e_mipi_dsi_info = {
	.type = MIPI_DSI_DPHY_RZV2H,
	.has_dphy_rstc = 0,
	.dphy_init = rzv2h_mipi_dsi_dphy_init,
	.dphy_exit = rzv2h_mipi_dsi_dphy_exit,
	.phy_reg_offset = 0x10000,
	.link_reg_offset = 0,
	.vclk_input = 2,
	.max_dclk = 187500,
	.min_dclk = 5440,
};

static const struct of_device_id rzg2l_mipi_dsi_of_table[] = {
	{ .compatible = "renesas,rzg2l-mipi-dsi",
	  .data = &rzg2l_mipi_dsi_info, },
	{ .compatible = "renesas,rzv2h-mipi-dsi",
	  .data = &rzv2h_mipi_dsi_info, },
	{ .compatible = "renesas,rzg3e-mipi-dsi",
	  .data = &rzg3e_mipi_dsi_info, },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, rzg2l_mipi_dsi_of_table);

static struct platform_driver rzg2l_mipi_dsi_platform_driver = {
	.probe	= rzg2l_mipi_dsi_probe,
	.remove	= rzg2l_mipi_dsi_remove,
	.driver	= {
		.name = "rzg2l-mipi-dsi",
		.pm = &rzg2l_mipi_pm_ops,
		.of_match_table = rzg2l_mipi_dsi_of_table,
	},
};

module_platform_driver(rzg2l_mipi_dsi_platform_driver);

MODULE_AUTHOR("Biju Das <biju.das.jz@bp.renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/G2L MIPI DSI Encoder Driver");
MODULE_LICENSE("GPL");
