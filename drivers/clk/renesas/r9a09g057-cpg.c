// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/V2H(P) CPG driver
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 */

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <dt-bindings/clock/renesas,r9a09g057-cpg.h>

#include "rzv2h-cpg.h"

enum clk_ids {
	/* Core Clock Outputs exported to DT */
	LAST_DT_CORE_CLK = R9A09G057_IOTOP_0_SHCLK,

	/* External Input Clocks */
	CLK_AUDIO_EXTAL,
	CLK_RTXIN,
	CLK_QEXTAL,
	AUDIO_CLKA,
	AUDIO_CLKB,
	AUDIO_CLKC,
	CLK_ET0_TXC_TX_CLK,
	CLK_ET0_RXC_RX_CLK,
	CLK_ET1_TXC_TX_CLK,
	CLK_ET1_RXC_RX_CLK,

	/* PLL Clocks */
	CLK_PLLCM33,
	CLK_PLLCLN,
	CLK_PLLDTY,
	CLK_PLLCA55,

	/* Internal Core Clocks */
	CLK_PLLCM33_DIV2,
	CLK_PLLCM33_DIV3,
	CLK_PLLCM33_DIV4,
	CLK_PLLCM33_DIV5,
	CLK_PLLCM33_DIV16,
	CLK_PLLCM33_DIV32,
	CLK_PLLCM33_ADC_PCLK,
	CLK_PLLCM33_ADC_ADCLK,
	CLK_PLLCM33_ADC_PCLK_DIV2,
	CLK_PLLCM33_DIV4_DDIV2,
	CLK_SMUX2_XSPI_CLK0,
	CLK_SMUX2_XSPI_CLK1,
	CLK_PLLCM33_XSPI,
	CLK_PLLCM33_XSPI_DIV2,
	CLK_PLLCLN_DIV2,
	CLK_PLLCLN_DIV4,
	CLK_PLLCLN_DIV8,
	CLK_PLLCLN_DIV16,
	CLK_PLLCLN_DIV20,
	CLK_PLLCLN_DIV32,
	CLK_PLLCLN_DIV64,
	CLK_PLLCLN_DIV256,
	CLK_PLLCLN_DIV1024,
	CLK_PLLDTY_DIV4,
	CLK_PLLDTY_DIV8,
	CLK_PLLDTY_DIV16,
	CLK_PLLDTY_ACPU,
	CLK_PLLDTY_ACPU_DIV2,
	CLK_PLLDTY_ACPU_DIV4,
	CLK_PLLDTY_RCPU,
	CLK_PLLDTY_RCPU_DIV4,
	CLK_PLLVDO,
	CLK_PLLVDO_CRU0,
	CLK_PLLVDO_CRU1,
	CLK_PLLVDO_CRU2,
	CLK_PLLVDO_CRU3,
	CLK_PLLVDO_ISU,
	CLK_PLLDSI,
	CLK_PLLDSI_SDIV2,
	CLK_PLLETH,
	CLK_PLLETH_DIV4,
	CLK_PLLETH_DIV4_DIV2,
	CLK_PLLETH_GBE0,
	CLK_PLLETH_GBE1,
	CLK_PLLETH_LPCLK,
	CLK_SMUX2_GBE0_TXCLK,
	CLK_SMUX2_GBE0_RXCLK,
	CLK_SMUX2_GBE1_TXCLK,
	CLK_SMUX2_GBE1_RXCLK,
	CLK_PLLGPU,
	CLK_PLLDRP,
	CLK_PLLGPU_GEAR,

	/* Module Clocks */
	MOD_CLK_BASE,
};

static const struct clk_div_table dtable_1_8[] = {
	{0, 1},
	{1, 2},
	{2, 4},
	{3, 8},
	{0, 0},
};

static const struct clk_div_table dtable_2_4[] = {
	{0, 2},
	{1, 4},
	{0, 0},
};

static const struct clk_div_table dtable_2_16[] = {
	{0, 2},
	{1, 4},
	{2, 8},
	{3, 16},
	{0, 0},
};

static const struct clk_div_table dtable_2_32[] = {
	{ 0,  2},
	{ 1,  4},
	{ 2,  6},
	{ 3,  8},
	{ 4, 10},
	{ 5, 12},
	{ 6, 14},
	{ 7, 16},
	{ 8, 18},
	{ 9, 20},
	{10, 22},
	{11, 24},
	{12, 26},
	{13, 28},
	{14, 30},
	{15, 32},
	{ 0,  0},
};

static const struct clk_div_table dtable_2_64[] = {
	{0, 2},
	{1, 4},
	{2, 8},
	{3, 16},
	{4, 64},
	{0, 0},
};

static const struct clk_div_table dtable_2_100[] = {
	{0, 2},
	{1, 10},
	{2, 100},
	{0, 0},
};

static const struct clk_div_table dtable_8_10[] = {
	{0, 8},
	{1, 10},
	{0, 0},
};

static const struct clk_div_table dtable_16_128[] = {
	{0,  16},
	{1,  32},
	{2,  64},
	{3, 128},
	{0,   0},
};

/* Mux clock tables */
static const char * const smux2_xspi_clk0[] = { ".pllcm33_div3", ".pllcm33_div4" };
static const char * const smux2_xspi_clk1[] = { ".smux2_xspi_clk0", ".pllcm33_div5" };
static const char * const smux2_gbe0_txclk[] = { ".plleth_gbe0", "et0_txc_tx_clk" };
static const char * const smux2_gbe0_rxclk[] = { ".plleth_gbe0", "et0_rxc_rx_clk" };
static const char * const smux2_gbe1_txclk[] = { ".plleth_gbe1", "et1_txc_tx_clk" };
static const char * const smux2_gbe1_rxclk[] = { ".plleth_gbe1", "et1_rxc_rx_clk" };

static const struct cpg_core_clk r9a09g057_core_clks[] __initconst = {
	/* External Clock Inputs */
	DEF_INPUT("audio_extal", CLK_AUDIO_EXTAL),
	DEF_INPUT("rtxin", CLK_RTXIN),
	DEF_INPUT("qextal", CLK_QEXTAL),
	DEF_INPUT("audio_clka", AUDIO_CLKA),
	DEF_INPUT("audio_clkb", AUDIO_CLKB),
	DEF_INPUT("audio_clkc", AUDIO_CLKC),
	DEF_INPUT("et0_txc_tx_clk", CLK_ET0_TXC_TX_CLK),
	DEF_INPUT("et0_rxc_rx_clk", CLK_ET0_RXC_RX_CLK),
	DEF_INPUT("et1_txc_tx_clk", CLK_ET1_TXC_TX_CLK),
	DEF_INPUT("et1_rxc_rx_clk", CLK_ET1_RXC_RX_CLK),

	/* PLL Clocks */
	DEF_FIXED(".pllcm33", CLK_PLLCM33, CLK_QEXTAL, 200, 3),
	DEF_FIXED(".pllcln", CLK_PLLCLN, CLK_QEXTAL, 200, 3),
	DEF_FIXED(".plldty", CLK_PLLDTY, CLK_QEXTAL, 200, 3),
	DEF_FIXED(".pllvdo", CLK_PLLVDO, CLK_QEXTAL, 210, 4),
	DEF_FIXED(".plleth", CLK_PLLETH, CLK_QEXTAL, 250, 6),
	DEF_PLL(".pllca55", CLK_PLLCA55, CLK_QEXTAL, PLL_PACK(PLL_CONF(0x64), 1100, 1800)),
	DEF_PLLDSI(".plldsi", CLK_PLLDSI, CLK_QEXTAL, PLL_PACK(PLL_CONF(0xC4), 25, 375)),
	DEF_PLL(".pllgpu", CLK_PLLGPU, CLK_QEXTAL, PLL_PACK(PLL_CONF(0x124), 1000, 1260)),
	DEF_PLL(".plldrp", CLK_PLLDRP, CLK_QEXTAL, PLL_PACK(PLL_CONF(0x144), 1260, 1260)),

	/* Internal Core Clocks */
	DEF_FIXED(".pllcm33_div2", CLK_PLLCM33_DIV2, CLK_PLLCM33, 1, 2),
	DEF_FIXED(".pllcm33_div3", CLK_PLLCM33_DIV3, CLK_PLLCM33, 1, 3),
	DEF_FIXED(".pllcm33_div4", CLK_PLLCM33_DIV4, CLK_PLLCM33, 1, 4),
	DEF_FIXED(".pllcm33_div5", CLK_PLLCM33_DIV5, CLK_PLLCM33, 1, 5),
	DEF_FIXED(".pllcm33_div16", CLK_PLLCM33_DIV16, CLK_PLLCM33, 1, 16),
	DEF_FIXED(".pllcm33_div32", CLK_PLLCM33_DIV32, CLK_PLLCM33, 1, 32),
	DEF_SDIV(".pllcm33_adc_pclk", CLK_PLLCM33_ADC_PCLK, CLK_PLLCM33_DIV2,
		 CSDIVx_DIVCTLy(1, 0, 1), dtable_8_10),
	DEF_FIXED(".pllcm33_adc_pclk_div2", CLK_PLLCM33_ADC_PCLK_DIV2, CLK_PLLCM33_ADC_PCLK, 1, 2),
	DEF_SDIV(".pllcm33_adc_adclk", CLK_PLLCM33_ADC_ADCLK, CLK_PLLCM33_ADC_PCLK,
		 CSDIVx_DIVCTLy(1, 1, 2), dtable_2_16),
	DEF_DDIV(".pllcm33_div4_ddiv2", CLK_PLLCM33_DIV4_DDIV2, CLK_PLLCM33_DIV4,
		 CDDIVx_DIVCTLy(0, 1, 3), dtable_2_64),
	DEF_MUX(".smux2_xspi_clk0", CLK_SMUX2_XSPI_CLK0, SSELx_SELCTLy(1, 2), smux2_xspi_clk0),
	DEF_MUX(".smux2_xspi_clk1", CLK_SMUX2_XSPI_CLK1, SSELx_SELCTLy(1, 3), smux2_xspi_clk1),
	DEF_SDIV(".pllcm33_xspi", CLK_PLLCM33_XSPI, CLK_SMUX2_XSPI_CLK1,
		 CSDIVx_DIVCTLy(0, 3, 2), dtable_2_16),
	DEF_FIXED(".pllcm33_xspi_div2", CLK_PLLCM33_XSPI_DIV2, CLK_PLLCM33_XSPI, 1, 2),

	DEF_FIXED(".pllcln_div2", CLK_PLLCLN_DIV2, CLK_PLLCLN, 1, 2),
	DEF_FIXED(".pllcln_div4", CLK_PLLCLN_DIV4, CLK_PLLCLN, 1, 4),
	DEF_FIXED(".pllcln_div8", CLK_PLLCLN_DIV8, CLK_PLLCLN, 1, 8),
	DEF_FIXED(".pllcln_div16", CLK_PLLCLN_DIV16, CLK_PLLCLN, 1, 16),
	DEF_FIXED(".pllcln_div20", CLK_PLLCLN_DIV20, CLK_PLLCLN, 1, 20),
	DEF_FIXED(".pllcln_div32", CLK_PLLCLN_DIV32, CLK_PLLCLN, 1, 32),
	DEF_FIXED(".pllcln_div64", CLK_PLLCLN_DIV64, CLK_PLLCLN, 1, 64),
	DEF_FIXED(".pllcln_div256", CLK_PLLCLN_DIV256, CLK_PLLCLN, 1, 256),
	DEF_FIXED(".pllcln_div1024", CLK_PLLCLN_DIV1024, CLK_PLLCLN, 1, 1024),


	DEF_FIXED(".plldty_div4", CLK_PLLDTY_DIV4, CLK_PLLDTY, 1, 4),
	DEF_FIXED(".plldty_div8", CLK_PLLDTY_DIV8, CLK_PLLDTY, 1, 8),
	DEF_FIXED(".plldty_div16", CLK_PLLDTY_DIV16, CLK_PLLDTY, 1, 16),
	DEF_DDIV(".plldty_acpu", CLK_PLLDTY_ACPU, CLK_PLLDTY,
		 CDDIVx_DIVCTLy(0, 2, 3), dtable_2_64),
	DEF_FIXED(".plldty_acpu_div2", CLK_PLLDTY_ACPU_DIV2, CLK_PLLDTY_ACPU, 1, 2),
	DEF_FIXED(".plldty_acpu_div4", CLK_PLLDTY_ACPU_DIV4, CLK_PLLDTY_ACPU, 1, 4),
	DEF_DDIV(".plldty_rcpu", CLK_PLLDTY_RCPU, CLK_PLLDTY,
		 CDDIVx_DIVCTLy(3, 2, 3), dtable_2_64),
	DEF_FIXED(".plldty_rcpu_div4", CLK_PLLDTY_RCPU_DIV4, CLK_PLLDTY_RCPU, 1, 4),
	DEF_DDIV(".pllvdo_cru0", CLK_PLLVDO_CRU0, CLK_PLLVDO,
		 CDDIVx_DIVCTLy(3, 3, 1), dtable_2_4),
	DEF_DDIV(".pllvdo_cru1", CLK_PLLVDO_CRU1, CLK_PLLVDO,
		 CDDIVx_DIVCTLy(4, 0, 1), dtable_2_4),
	DEF_DDIV(".pllvdo_cru2", CLK_PLLVDO_CRU2, CLK_PLLVDO,
		 CDDIVx_DIVCTLy(4, 1, 1), dtable_2_4),
	DEF_DDIV(".pllvdo_cru3", CLK_PLLVDO_CRU3, CLK_PLLVDO,
		 CDDIVx_DIVCTLy(4, 2, 1), dtable_2_4),
	DEF_DDIV(".pllvdo_isu", CLK_PLLVDO_ISU, CLK_PLLVDO,
		 CDDIVx_DIVCTLy(3, 0, 3), dtable_2_64),
	DEF_FIXED(".plleth_div4", CLK_PLLETH_DIV4, CLK_PLLETH, 1, 4),
	DEF_FIXED(".plleth_div4_div2", CLK_PLLETH_DIV4_DIV2, CLK_PLLETH_DIV4, 1, 2),
	DEF_SDIV(".plleth_gbe0", CLK_PLLETH_GBE0, CLK_PLLETH_DIV4,
		 CSDIVx_DIVCTLy(0, 0, 2), dtable_2_100),
	DEF_SDIV(".plleth_gbe1", CLK_PLLETH_GBE1, CLK_PLLETH_DIV4,
		 CSDIVx_DIVCTLy(0, 1, 2), dtable_2_100),
	DEF_MUX(".smux2_gbe0_txclk", CLK_SMUX2_GBE0_TXCLK, SSELx_SELCTLy(0, 2), smux2_gbe0_txclk),
	DEF_MUX(".smux2_gbe0_rxclk", CLK_SMUX2_GBE0_RXCLK, SSELx_SELCTLy(0, 3), smux2_gbe0_rxclk),
	DEF_MUX(".smux2_gbe1_txclk", CLK_SMUX2_GBE1_TXCLK, SSELx_SELCTLy(1, 0), smux2_gbe1_txclk),
	DEF_MUX(".smux2_gbe1_rxclk", CLK_SMUX2_GBE1_RXCLK, SSELx_SELCTLy(1, 1), smux2_gbe1_rxclk),
	DEF_SDIV(".plleth_lpclk", CLK_PLLETH_LPCLK, CLK_PLLETH_DIV4,
		 CSDIVx_DIVCTLy(0, 2, 2), dtable_16_128),
	DEF_PLLDSI_SDIV(".plldsi_sdiv2", CLK_PLLDSI_SDIV2, CLK_PLLDSI,
		 CSDIVx_DIVCTLy(1, 2, 4), dtable_2_32),
	DEF_DDIV(".pllgpu_gear", CLK_PLLGPU_GEAR, CLK_PLLGPU,
		 CDDIVx_DIVCTLy(3, 1, 3), dtable_2_64),

	/* Core Clocks */
	DEF_FIXED("sys_0_pclk", R9A09G057_SYS_0_PCLK, CLK_QEXTAL, 1, 1),
	DEF_FIXED("iotop_0_shclk", R9A09G057_IOTOP_0_SHCLK, CLK_PLLCM33_DIV16, 1, 1),
	DEF_DDIV(".ca55_0_coreclk0", R9A09G057_CA55_0_CORE_CLK0,
		 CLK_PLLCA55, CDDIVx_DIVCTLy(1, 0, 2), dtable_1_8),
	DEF_DDIV(".ca55_0_coreclk1", R9A09G057_CA55_0_CORE_CLK1,
		 CLK_PLLCA55, CDDIVx_DIVCTLy(1, 1, 2), dtable_1_8),
	DEF_DDIV(".ca55_0_coreclk2", R9A09G057_CA55_0_CORE_CLK2,
		 CLK_PLLCA55, CDDIVx_DIVCTLy(1, 2, 2), dtable_1_8),
	DEF_DDIV(".ca55_0_coreclk3", R9A09G057_CA55_0_CORE_CLK3,
		 CLK_PLLCA55, CDDIVx_DIVCTLy(1, 3, 2), dtable_1_8),
};

static const struct rzv2h_mod_clk r9a09g057_mod_clks[] __initconst = {
	DEF_MOD_CRITICAL("mcpu_dmac0_aclk",	CLK_PLLCM33_DIV4_DDIV2, 0, 0, 0, 0),
	DEF_MOD_CRITICAL("acpu_dmac0_aclk",	CLK_PLLDTY_ACPU_DIV2, 0, 1, 0, 1),
	DEF_MOD_CRITICAL("acpu_dmac1_aclk",	CLK_PLLDTY_ACPU_DIV2, 0, 2, 0, 2),
	DEF_MOD_CRITICAL("rcpu_dmac0_aclk",	CLK_PLLDTY_RCPU_DIV4, 0, 3, 0, 3),
	DEF_MOD_CRITICAL("rcpu_dmac1_aclk",	CLK_PLLDTY_RCPU_DIV4, 0, 4, 0, 4),
	DEF_MOD_CRITICAL("icu",			CLK_PLLCM33_DIV16, 0, 5, 0, 5),
	DEF_MOD_CRITICAL("gic",			CLK_PLLDTY_ACPU_DIV4, 1, 3, 0, 19),
	DEF_MOD("gpt0_pclk_sfr",		CLK_PLLCLN_DIV8, 3, 1, 1, 17),
	DEF_MOD("gpt1_pclk_sfr",		CLK_PLLCLN_DIV8, 3, 2, 1, 18),
	DEF_MOD("mcpu_cmtw0_clkm",		CLK_PLLCM33_DIV32, 3, 11, 1, 27),
	DEF_MOD("mcpu_cmtw1_clkm",		CLK_PLLCM33_DIV32, 3, 12, 1, 28),
	DEF_MOD("mcpu_cmtw2_clkm",		CLK_PLLCM33_DIV32, 3, 13, 1, 29),
	DEF_MOD("mcpu_cmtw3_clkm",		CLK_PLLCM33_DIV32, 3, 14, 1, 30),
	DEF_MOD("rcpu_cmtw0_clkm",		CLK_PLLCLN_DIV32, 3, 15, 1, 31),
	DEF_MOD("rcpu_cmtw1_clkm",		CLK_PLLCLN_DIV32, 4, 0, 2, 0),
	DEF_MOD("rcpu_cmtw2_clkm",		CLK_PLLCLN_DIV32, 4, 1, 2, 1),
	DEF_MOD("rcpu_cmtw3_clkm",		CLK_PLLCLN_DIV32, 4, 2, 2, 2),
	DEF_MOD("gtm_0_pclk",			CLK_PLLCM33_DIV16, 4, 3, 2, 3),
	DEF_MOD("gtm_1_pclk",			CLK_PLLCM33_DIV16, 4, 4, 2, 4),
	DEF_MOD("gtm_2_pclk",			CLK_PLLCLN_DIV16, 4, 5, 2, 5),
	DEF_MOD("gtm_3_pclk",			CLK_PLLCLN_DIV16, 4, 6, 2, 6),
	DEF_MOD("gtm_4_pclk",			CLK_PLLCLN_DIV16, 4, 7, 2, 7),
	DEF_MOD("gtm_5_pclk",			CLK_PLLCLN_DIV16, 4, 8, 2, 8),
	DEF_MOD("gtm_6_pclk",			CLK_PLLCLN_DIV16, 4, 9, 2, 9),
	DEF_MOD("gtm_7_pclk",			CLK_PLLCLN_DIV16, 4, 10, 2, 10),
	DEF_MOD("wdt_0_clkp",			CLK_PLLCM33_DIV16, 4, 11, 2, 11),
	DEF_MOD("wdt_0_clk_loco",		CLK_QEXTAL, 4, 12, 2, 12),
	DEF_MOD("wdt_1_clkp",			CLK_PLLCLN_DIV16, 4, 13, 2, 13),
	DEF_MOD("wdt_1_clk_loco",		CLK_QEXTAL, 4, 14, 2, 14),
	DEF_MOD("wdt_2_clkp",			CLK_PLLCLN_DIV16, 4, 15, 2, 15),
	DEF_MOD("wdt_2_clk_loco",		CLK_QEXTAL, 5, 0, 2, 16),
	DEF_MOD("wdt_3_clkp",			CLK_PLLCLN_DIV16, 5, 1, 2, 17),
	DEF_MOD("wdt_3_clk_loco",		CLK_QEXTAL, 5, 2, 2, 18),
	DEF_MOD("rtc_clk_rtc",			CLK_PLLCM33_DIV16, 5, 3, 2, 19),
	DEF_MOD("rspi0_pclk",			CLK_PLLCLN_DIV8, 5, 4, 2, 20),
	DEF_MOD("rspi0_pclk_sfr",		CLK_PLLCLN_DIV8, 5, 5, 2, 21),
	DEF_MOD("rspi0_tclk",			CLK_PLLCLN_DIV8, 5, 6, 2, 22),
	DEF_MOD("rspi1_pclk",			CLK_PLLCLN_DIV8, 5, 7, 2, 23),
	DEF_MOD("rspi1_pclk_sfr",		CLK_PLLCLN_DIV8, 5, 8, 2, 24),
	DEF_MOD("rspi1_tclk",			CLK_PLLCLN_DIV8, 5, 9, 2, 25),
	DEF_MOD("rspi2_pclk",			CLK_PLLCLN_DIV8, 5, 10, 2, 26),
	DEF_MOD("rspi2_pclk_sfr",		CLK_PLLCLN_DIV8, 5, 11, 2, 27),
	DEF_MOD("rspi2_tclk",			CLK_PLLCLN_DIV8, 5, 12, 2, 28),
	DEF_MOD("rsci0_pclk",			CLK_PLLCM33_DIV16, 5, 13, 2, 29),
	DEF_MOD("rsci0_tclk",			CLK_PLLCM33_DIV16, 5, 14, 2, 30),
	DEF_MOD("rsci0_ps_ps3_n",		CLK_PLLCLN_DIV1024, 5, 15, 2, 31),
	DEF_MOD("rsci0_ps_ps2_n",		CLK_PLLCLN_DIV256, 6, 0, 3, 0),
	DEF_MOD("rsci0_ps_ps1_n",		CLK_PLLCLN_DIV64, 6, 1, 3, 1),
	DEF_MOD("rsci1_pclk",			CLK_PLLCM33_DIV16, 6, 2, 3, 2),
	DEF_MOD("rsci1_tclk",			CLK_PLLCM33_DIV16, 6, 3, 3, 3),
	DEF_MOD("rsci1_ps_ps3_n",		CLK_PLLCLN_DIV1024, 6, 4, 3, 4),
	DEF_MOD("rsci1_ps_ps2_n",		CLK_PLLCLN_DIV256, 6, 5, 3, 5),
	DEF_MOD("rsci1_ps_ps1_n",		CLK_PLLCLN_DIV64, 6, 6, 3, 6),
	DEF_MOD("rsci2_pclk",			CLK_PLLCM33_DIV16, 6, 7, 3, 7),
	DEF_MOD("rsci2_tclk",			CLK_PLLCM33_DIV16, 6, 8, 3, 8),
	DEF_MOD("rsci2_ps_ps3_n",		CLK_PLLCLN_DIV1024, 6, 9, 3, 9),
	DEF_MOD("rsci2_ps_ps2_n",		CLK_PLLCLN_DIV256, 6, 10, 3, 10),
	DEF_MOD("rsci2_ps_ps1_n",		CLK_PLLCLN_DIV64, 6, 11, 3, 11),
	DEF_MOD("rsci3_pclk",			CLK_PLLCM33_DIV16, 6, 12, 3, 12),
	DEF_MOD("rsci3_tclk",			CLK_PLLCM33_DIV16, 6, 13, 3, 13),
	DEF_MOD("rsci3_ps_ps3_n",		CLK_PLLCLN_DIV1024, 6, 14, 3, 14),
	DEF_MOD("rsci3_ps_ps2_n",		CLK_PLLCLN_DIV256, 6, 15, 3, 15),
	DEF_MOD("rsci3_ps_ps1_n",		CLK_PLLCLN_DIV64, 7, 0, 3, 16),
	DEF_MOD("rsci4_pclk",			CLK_PLLCM33_DIV16, 7, 1, 3, 17),
	DEF_MOD("rsci4_tclk",			CLK_PLLCM33_DIV16, 7, 2, 3, 18),
	DEF_MOD("rsci4_ps_ps3_n",		CLK_PLLCLN_DIV1024, 7, 3, 3, 19),
	DEF_MOD("rsci4_ps_ps2_n",		CLK_PLLCLN_DIV256, 7, 4, 3, 20),
	DEF_MOD("rsci4_ps_ps1_n",		CLK_PLLCLN_DIV64, 7, 5, 3, 21),
	DEF_MOD("rsci5_pclk",			CLK_PLLCM33_DIV16, 7, 6, 3, 22),
	DEF_MOD("rsci5_tclk",			CLK_PLLCM33_DIV16, 7, 7, 3, 23),
	DEF_MOD("rsci5_ps_ps3_n",		CLK_PLLCLN_DIV1024, 7, 8, 3, 24),
	DEF_MOD("rsci5_ps_ps2_n",		CLK_PLLCLN_DIV256, 7, 9, 3, 25),
	DEF_MOD("rsci5_ps_ps1_n",		CLK_PLLCLN_DIV64, 7, 10, 3, 26),
	DEF_MOD("rsci6_pclk",			CLK_PLLCM33_DIV16, 7, 11, 3, 27),
	DEF_MOD("rsci6_tclk",			CLK_PLLCM33_DIV16, 7, 12, 3, 28),
	DEF_MOD("rsci6_ps_ps3_n",		CLK_PLLCLN_DIV1024, 7, 13, 3, 29),
	DEF_MOD("rsci6_ps_ps2_n",		CLK_PLLCLN_DIV256, 7, 14, 3, 30),
	DEF_MOD("rsci6_ps_ps1_n",		CLK_PLLCLN_DIV64, 7, 15, 3, 31),
	DEF_MOD("rsci7_pclk",			CLK_PLLCM33_DIV16, 8, 0, 4, 0),
	DEF_MOD("rsci7_tclk",			CLK_PLLCM33_DIV16, 8, 1, 4, 1),
	DEF_MOD("rsci7_ps_ps3_n",		CLK_PLLCLN_DIV1024, 8, 2, 4, 2),
	DEF_MOD("rsci7_ps_ps2_n",		CLK_PLLCLN_DIV256, 8, 3, 4, 3),
	DEF_MOD("rsci7_ps_ps1_n",		CLK_PLLCLN_DIV64, 8, 4, 4, 4),
	DEF_MOD("rsci8_pclk",			CLK_PLLCM33_DIV16, 8, 5, 4, 5),
	DEF_MOD("rsci8_tclk",			CLK_PLLCM33_DIV16, 8, 6, 4, 6),
	DEF_MOD("rsci8_ps_ps3_n",		CLK_PLLCLN_DIV1024, 8, 7, 4, 7),
	DEF_MOD("rsci8_ps_ps2_n",		CLK_PLLCLN_DIV256, 8, 8, 4, 8),
	DEF_MOD("rsci8_ps_ps1_n",		CLK_PLLCLN_DIV64, 8, 9, 4, 9),
	DEF_MOD("rsci9_pclk",			CLK_PLLCM33_DIV16, 8, 10, 4, 10),
	DEF_MOD("rsci9_tclk",			CLK_PLLCM33_DIV16, 8, 11, 4, 11),
	DEF_MOD("rsci9_ps_ps3_n",		CLK_PLLCLN_DIV1024, 8, 12, 4, 12),
	DEF_MOD("rsci9_ps_ps2_n",		CLK_PLLCLN_DIV256, 8, 13, 4, 13),
	DEF_MOD("rsci9_ps_ps1_n",		CLK_PLLCLN_DIV64, 8, 14, 4, 14),
	DEF_MOD("scif_0_clk_pck",		CLK_PLLCM33_DIV16, 8, 15, 4, 15),
	DEF_MOD("i3c_pclkrw",			CLK_PLLCLN_DIV16, 9, 0, 4, 16),
	DEF_MOD("i3c_pclk",			CLK_PLLCLN_DIV16, 9, 1, 4, 17),
	DEF_MOD("i3c_tclk",			CLK_PLLCLN_DIV8, 9, 2, 4, 18),
	DEF_MOD("riic_8_ckm",			CLK_PLLCM33_DIV16, 9, 3, 4, 19),
	DEF_MOD("riic_0_ckm",			CLK_PLLCLN_DIV16, 9, 4, 4, 20),
	DEF_MOD("riic_1_ckm",			CLK_PLLCLN_DIV16, 9, 5, 4, 21),
	DEF_MOD("riic_2_ckm",			CLK_PLLCLN_DIV16, 9, 6, 4, 22),
	DEF_MOD("riic_3_ckm",			CLK_PLLCLN_DIV16, 9, 7, 4, 23),
	DEF_MOD("riic_4_ckm",			CLK_PLLCLN_DIV16, 9, 8, 4, 24),
	DEF_MOD("riic_5_ckm",			CLK_PLLCLN_DIV16, 9, 9, 4, 25),
	DEF_MOD("riic_6_ckm",			CLK_PLLCLN_DIV16, 9, 10, 4, 26),
	DEF_MOD("riic_7_ckm",			CLK_PLLCLN_DIV16, 9, 11, 4, 27),
	DEF_MOD("canfd_pclk",			CLK_PLLCLN_DIV16, 9, 12, 4, 28),
	DEF_MOD("canfd_clk_ram",		CLK_PLLCLN_DIV8, 9, 13, 4, 29),
	DEF_MOD("canfd_clkc",			CLK_PLLCLN_DIV20, 9, 14, 4, 30),
	DEF_MOD("spi_hclk",			CLK_PLLCM33_DIV4_DDIV2, 9, 15, 4, 31),
	DEF_MOD("spi_aclk",			CLK_PLLCM33_DIV4_DDIV2, 10, 0, 5, 0),
	DEF_MOD("spi_clk_spi",			CLK_PLLCM33_XSPI_DIV2, 10, 1, 5, 1),
	DEF_MOD("spi_clk_spix2",		CLK_PLLCM33_XSPI, 10, 2, 5, 2),
	DEF_MOD("sdhi_0_imclk",			CLK_PLLCLN_DIV8, 10, 3, 5, 3),
	DEF_MOD("sdhi_0_imclk2",		CLK_PLLCLN_DIV8, 10, 4, 5, 4),
	DEF_MOD("sdhi_0_clk_hs",		CLK_PLLCLN_DIV2, 10, 5, 5, 5),
	DEF_MOD("sdhi_0_aclk",			CLK_PLLDTY_ACPU_DIV4, 10, 6, 5, 6),
	DEF_MOD("sdhi_1_imclk",			CLK_PLLCLN_DIV8, 10, 7, 5, 7),
	DEF_MOD("sdhi_1_imclk2",		CLK_PLLCLN_DIV8, 10, 8, 5, 8),
	DEF_MOD("sdhi_1_clk_hs",		CLK_PLLCLN_DIV2, 10, 9, 5, 9),
	DEF_MOD("sdhi_1_aclk",			CLK_PLLDTY_ACPU_DIV4, 10, 10, 5, 10),
	DEF_MOD("sdhi_2_imclk",			CLK_PLLCLN_DIV8, 10, 11, 5, 11),
	DEF_MOD("sdhi_2_imclk2",		CLK_PLLCLN_DIV8, 10, 12, 5, 12),
	DEF_MOD("sdhi_2_clk_hs",		CLK_PLLCLN_DIV2, 10, 13, 5, 13),
	DEF_MOD("sdhi_2_aclk",			CLK_PLLDTY_ACPU_DIV4, 10, 14, 5, 14),
	DEF_MOD("usb30_aclk",			CLK_PLLDTY_DIV8, 10, 15, 5, 15),
	DEF_MOD("usb30_pclk_usbtst",		CLK_PLLDTY_ACPU_DIV4, 11, 0, 5, 16),
	DEF_MOD("usb31_aclk",			CLK_PLLDTY_DIV8, 11, 1, 5, 17),
	DEF_MOD("usb31_pclk_usbtst",		CLK_PLLDTY_ACPU_DIV4, 11, 2, 5, 18),
	DEF_MOD("usb2_u2h0_hclk",		CLK_PLLDTY_DIV8, 11, 3, 5, 19),
	DEF_MOD("usb2_u2h1_hclk",		CLK_PLLDTY_DIV8, 11, 4, 5, 20),
	DEF_MOD("usb2_u2p_exr_cpuclk",		CLK_PLLDTY_ACPU_DIV4, 11, 5, 5, 21),
	DEF_MOD("usb2_pclk_usbtst0",		CLK_PLLDTY_ACPU_DIV4, 11, 6, 5, 22),
	DEF_MOD("usb2_pclk_usbtst1",		CLK_PLLDTY_ACPU_DIV4, 11, 7, 5, 23),
	DEF_MOD("gbeth0_clk_tx_i",              CLK_SMUX2_GBE0_TXCLK, 11, 8, 5, 24),
	DEF_MOD("gbeth0_clk_rx_i",              CLK_SMUX2_GBE0_RXCLK, 11, 9, 5, 25),
	DEF_MOD("gbeth0_clk_tx_180_i",          CLK_SMUX2_GBE0_TXCLK, 11, 10, 5, 26),
	DEF_MOD("gbeth0_clk_rx_180_i",          CLK_SMUX2_GBE0_RXCLK, 11, 11, 5, 27),
	DEF_MOD("gbeth0_aclk_csr_i",            CLK_PLLDTY_DIV8, 11, 12, 5, 28),
	DEF_MOD("gbeth0_aclk_i",                CLK_PLLDTY_DIV8, 11, 13, 5, 29),
	DEF_MOD("gbeth1_clk_tx_i",              CLK_SMUX2_GBE1_TXCLK, 11, 14, 5, 30),
	DEF_MOD("gbeth1_clk_rx_i",              CLK_SMUX2_GBE1_RXCLK, 11, 15, 5, 31),
	DEF_MOD("gbeth1_clk_tx_180_i",          CLK_SMUX2_GBE1_TXCLK, 12, 0, 6, 0),
	DEF_MOD("gbeth1_clk_rx_180_i",          CLK_SMUX2_GBE1_RXCLK, 12, 1, 6, 1),
	DEF_MOD("gbeth1_aclk_csr_i",            CLK_PLLDTY_DIV8, 12, 2, 5, 2),
	DEF_MOD("gbeth1_aclk_i",                CLK_PLLDTY_DIV8, 12, 3, 6, 3),
	DEF_MOD("pcie_aclk",			CLK_PLLDTY_ACPU_DIV2, 12, 4, 6, 4),
	DEF_MOD("pcie_clk_pmu",			CLK_PLLDTY_ACPU_DIV2, 12, 5, 6, 5),
	DEF_MOD("cru0_aclk",			CLK_PLLDTY_ACPU_DIV2, 13, 2, 6, 18),
	DEF_MOD("cru0_vclk",			CLK_PLLVDO_CRU0, 13, 3, 6, 19),
	DEF_MOD("cru0_pclk",			CLK_PLLDTY_DIV16, 13, 4, 6, 20),
	DEF_MOD("cru1_aclk",			CLK_PLLDTY_ACPU_DIV2, 13, 5, 6, 21),
	DEF_MOD("cru1_vclk",			CLK_PLLVDO_CRU1, 13, 6, 6, 22),
	DEF_MOD("cru1_pclk",			CLK_PLLDTY_DIV16, 13, 7, 6, 23),
	DEF_MOD("cru2_aclk",			CLK_PLLDTY_ACPU_DIV2, 13, 8, 6, 24),
	DEF_MOD("cru2_vclk",			CLK_PLLVDO_CRU2, 13, 9, 6, 25),
	DEF_MOD("cru2_pclk",			CLK_PLLDTY_DIV16, 13, 10, 6, 26),
	DEF_MOD("cru3_aclk",			CLK_PLLDTY_ACPU_DIV2, 13, 11, 6, 27),
	DEF_MOD("cru3_vclk",			CLK_PLLVDO_CRU3, 13, 12, 6, 28),
	DEF_MOD("cru3_pclk",			CLK_PLLDTY_DIV16, 13, 13, 6, 29),
	DEF_MOD("cru0_m_xi",			CLK_QEXTAL, 13, 14, 6, 30),
	DEF_MOD("cru1_m_xi",			CLK_QEXTAL, 13, 15, 6, 31),
	DEF_MOD("cru2_m_xi",			CLK_QEXTAL, 14, 0, 7, 0),
	DEF_MOD("cru3_m_xi",			CLK_QEXTAL, 14, 1, 7, 1),
	DEF_MOD("isu_aclk",			CLK_PLLVDO_ISU, 14, 6, 7, 6),
	DEF_MOD("isu_pclk",			CLK_PLLDTY_DIV16, 14, 7, 7, 7),
	DEF_MOD("dsi_pclk",			CLK_PLLDTY_DIV16, 14, 8, 7, 8),
	DEF_MOD("dsi_aclk",			CLK_PLLDTY_ACPU_DIV2, 14, 9, 7, 9),
	DEF_MOD("dsi_vclk",			CLK_PLLDSI_SDIV2, 14, 10, 7, 10),
	DEF_MOD("dsi_lpclk",			CLK_PLLETH_LPCLK, 14, 11, 7, 11),
	DEF_MOD("dsi_pllclk",			CLK_QEXTAL, 14, 12, 7, 12),
	DEF_MOD("lcdc_clka",			CLK_PLLDTY_ACPU_DIV2, 14, 13, 7, 13),
	DEF_MOD("lcdc_clkp",			CLK_PLLDTY_DIV16, 14, 14, 7, 14),
	DEF_MOD("lcdc_clkd",			CLK_PLLDSI_SDIV2, 14, 15, 7, 15),
	DEF_MOD("gpu_clk",			CLK_PLLGPU_GEAR, 15, 0, 7, 16),
	DEF_MOD("gpu_axi_clk",			CLK_PLLDTY_ACPU_DIV2, 15, 1, 7, 17),
	DEF_MOD("gpu_ace_clk",			CLK_PLLDTY_ACPU_DIV2, 15, 2, 7, 18),
	DEF_MOD("vcd_aclk",			CLK_PLLDTY_DIV4, 15, 3, 7, 19),
	DEF_MOD("vcd_pclk",			CLK_PLLDTY_DIV8, 15, 4, 7, 20),
	DEF_MOD("ssif_clk",			CLK_PLLCLN_DIV8, 15, 5, 7, 21),
	DEF_MOD("scu_clk",			CLK_PLLCLN_DIV8, 15, 6, 7, 22),
	DEF_MOD("scu_clkx2",			CLK_PLLCLN_DIV4, 15, 7, 7, 23),
	DEF_MOD("dmacpp_clk",			CLK_PLLCLN_DIV8, 15, 8, 7, 24),
	DEF_MOD("adg_clks1",			CLK_PLLCLN_DIV8, 15, 9, 7, 25),
	DEF_MOD("adg_clk_195m",			CLK_PLLCLN_DIV8, 15, 10, 7, 26),
	DEF_MOD("adg_audio_clka",		AUDIO_CLKA, 15, 11, 7, 27),
	DEF_MOD("adg_audio_clkb",		AUDIO_CLKB, 15, 12, 7, 28),
	DEF_MOD("adg_audio_clkc",		AUDIO_CLKC, 15, 13, 7, 29),
	DEF_MOD("spdif0_clkp",			CLK_PLLDTY_DIV16, 15, 14, 7, 30),
	DEF_MOD("spdif1_clkp",			CLK_PLLDTY_DIV16, 15, 15, 7, 31),
	DEF_MOD("spdif2_clkp",			CLK_PLLDTY_DIV16, 16, 0, 8, 0),
	DEF_MOD("adc_pclk",			CLK_PLLCM33_ADC_PCLK_DIV2, 16, 7, 8, 7),
	DEF_MOD("adc_adclk",			CLK_PLLCM33_ADC_ADCLK, 16, 8, 8, 8),
	DEF_MOD("tsu0_pclk",			CLK_QEXTAL, 16, 9, 8, 9),
	DEF_MOD("tsu1_pclk",			CLK_QEXTAL, 16, 10, 8, 10),
	DEF_MOD("ssi9_clk",			CLK_PLLCLN_DIV8, 24, 10, -1, -1),
	DEF_MOD("ssi8_clk",			CLK_PLLCLN_DIV8, 24,  9, -1, -1),
	DEF_MOD("ssi7_clk",			CLK_PLLCLN_DIV8, 24,  8, -1, -1),
	DEF_MOD("ssi6_clk",			CLK_PLLCLN_DIV8, 24,  7, -1, -1),
	DEF_MOD("ssi5_clk",			CLK_PLLCLN_DIV8, 24,  6, -1, -1),
	DEF_MOD("ssi4_clk",			CLK_PLLCLN_DIV8, 24,  5, -1, -1),
	DEF_MOD("ssi3_clk",			CLK_PLLCLN_DIV8, 24,  4, -1, -1),
	DEF_MOD("ssi2_clk",			CLK_PLLCLN_DIV8, 24,  3, -1, -1),
	DEF_MOD("ssi1_clk",			CLK_PLLCLN_DIV8, 24,  2, -1, -1),
	DEF_MOD("ssi0_clk",			CLK_PLLCLN_DIV8, 24,  1, -1, -1),
	DEF_MOD("ssif_supply_clk",		CLK_PLLCLN_DIV8, 24,  0, -1, -1),
	DEF_MOD("src9_clk",			CLK_PLLCLN_DIV8, 23, 13, -1, -1),
	DEF_MOD("src8_clk",			CLK_PLLCLN_DIV8, 23, 12, -1, -1),
	DEF_MOD("src7_clk",			CLK_PLLCLN_DIV8, 23, 11, -1, -1),
	DEF_MOD("src6_clk",			CLK_PLLCLN_DIV8, 23, 10, -1, -1),
	DEF_MOD("src5_clk",			CLK_PLLCLN_DIV8, 23,  9, -1, -1),
	DEF_MOD("src4_clk",			CLK_PLLCLN_DIV8, 23,  8, -1, -1),
	DEF_MOD("src3_clk",			CLK_PLLCLN_DIV8, 23,  7, -1, -1),
	DEF_MOD("src2_clk",			CLK_PLLCLN_DIV8, 23,  6, -1, -1),
	DEF_MOD("src1_clk",			CLK_PLLCLN_DIV8, 23,  5, -1, -1),
	DEF_MOD("src0_clk",			CLK_PLLCLN_DIV8, 23,  4, -1, -1),
	DEF_MOD("dvc0_clk",			CLK_PLLCLN_DIV8, 23,  0, -1, -1),
	DEF_MOD("dvc1_clk",			CLK_PLLCLN_DIV8, 23,  1, -1, -1),
	DEF_MOD("ctu0_mix0_clk",		CLK_PLLCLN_DIV8, 23,  2, -1, -1),
	DEF_MOD("ctu1_mix1_clk",		CLK_PLLCLN_DIV8, 23,  3, -1, -1),
	DEF_MOD("scu_supply_clk",		CLK_PLLCLN_DIV8, 23, 14, -1, -1),
	DEF_MOD("adg_ssi0_clk",			CLK_PLLCLN_DIV8, 22, 0, -1, -1),
	DEF_MOD("adg_ssi1_clk",			CLK_PLLCLN_DIV8, 22, 1, -1, -1),
	DEF_MOD("adg_ssi2_clk",			CLK_PLLCLN_DIV8, 22, 2, -1, -1),
	DEF_MOD("adg_ssi3_clk",			CLK_PLLCLN_DIV8, 22, 3, -1, -1),
	DEF_MOD("adg_ssi4_clk",			CLK_PLLCLN_DIV8, 22, 4, -1, -1),
	DEF_MOD("adg_ssi5_clk",			CLK_PLLCLN_DIV8, 22, 5, -1, -1),
	DEF_MOD("adg_ssi6_clk",			CLK_PLLCLN_DIV8, 22, 6, -1, -1),
	DEF_MOD("adg_ssi7_clk",			CLK_PLLCLN_DIV8, 22, 7, -1, -1),
	DEF_MOD("adg_ssi8_clk",			CLK_PLLCLN_DIV8, 22, 8, -1, -1),
	DEF_MOD("adg_ssi9_clk",			CLK_PLLCLN_DIV8, 22, 9, -1, -1),
	DEF_MOD("adg_spdif0_clk",		CLK_PLLCLN_DIV8, 22, 10, -1, -1),
	DEF_MOD("adg_spdif1_clk",		CLK_PLLCLN_DIV8, 22, 11, -1, -1),
	DEF_MOD("adg_spdif2_clk",		CLK_PLLCLN_DIV8, 22, 12, -1, -1),
};

static const struct rzv2h_reset r9a09g057_resets[] __initconst = {
	DEF_RST(3, 1, 1, 2),		/* MCPU_DMAC_0_ARESETN */
	DEF_RST(3, 2, 1, 3),		/* ACPU_DMAC_0_ARESETN */
	DEF_RST(3, 3, 1, 4),		/* ACPU_DMAC_1_ARESETN */
	DEF_RST(3, 4, 1, 5),		/* RCPU_DMAC_0_ARESETN */
	DEF_RST(3, 5, 1, 6),		/* RCPU_DMAC_1_ARESETN */
	DEF_RST(3, 6, 1, 7),		/* ICU_PRESETN	*/
	DEF_RST(3, 8, 1, 9),		/* GIC_GICRESET_N */
	DEF_RST(3, 9, 1, 10),		/* GIC_DBG_GICRESET_N */
	DEF_RST(5, 9, 2, 10),		/* GPT0_RST_P_REG */
	DEF_RST(5, 10, 2, 11),		/* GPT0_RST_S_REG */
	DEF_RST(5, 11, 2, 12),		/* GPT1_RST_P_REG */
	DEF_RST(5, 12, 2, 13),		/* GPT1_RST_S_REG */
	DEF_RST(6, 5, 2, 22),		/* MCPU_CMTW0_RST_M */
	DEF_RST(6, 6, 2, 23),		/* MCPU_CMTW1_RST_M */
	DEF_RST(6, 7, 2, 24),		/* MCPU_CMTW2_RST_M */
	DEF_RST(6, 8, 2, 25),		/* MCPU_CMTW3_RST_M */
	DEF_RST(6, 9, 2, 26),		/* RCPU_CMTW0_RST_M */
	DEF_RST(6, 10, 2, 27),		/* RCPU_CMTW1_RST_M */
	DEF_RST(6, 11, 2, 28),		/* RCPU_CMTW2_RST_M */
	DEF_RST(6, 12, 2, 29),		/* RCPU_CMTW3_RST_M */
	DEF_RST(6, 13, 2, 30),		/* GTM_0_PRESETZ */
	DEF_RST(6, 14, 2, 31),		/* GTM_1_PRESETZ */
	DEF_RST(6, 15, 3, 0),		/* GTM_2_PRESETZ */
	DEF_RST(7, 0, 3, 1),		/* GTM_3_PRESETZ */
	DEF_RST(7, 1, 3, 2),		/* GTM_4_PRESETZ */
	DEF_RST(7, 2, 3, 3),		/* GTM_5_PRESETZ */
	DEF_RST(7, 3, 3, 4),		/* GTM_6_PRESETZ */
	DEF_RST(7, 4, 3, 5),		/* GTM_7_PRESETZ */
	DEF_RST(7, 5, 3, 6),		/* WDT_0_RESET */
	DEF_RST(7, 6, 3, 7),		/* WDT_1_RESET */
	DEF_RST(7, 7, 3, 8),		/* WDT_2_RESET */
	DEF_RST(7, 8, 3, 9),		/* WDT_3_RESET */
	DEF_RST(7, 9, 3, 10),		/* RTC_RST_RTC */
	DEF_RST(7, 10, 3, 11),		/* RTC_RST_RTC_V */
	DEF_RST(7, 11, 3, 12),		/* RSPI0_PRESETN */
	DEF_RST(7, 12, 3, 13),		/* RSPI0_TRESETN */
	DEF_RST(7, 13, 3, 14),		/* RSPI1_PRESETN */
	DEF_RST(7, 14, 3, 15),		/* RSPI1_TRESETN */
	DEF_RST(7, 15, 3, 16),		/* RSPI2_PRESETN */
	DEF_RST(8, 0, 3, 17),		/* RSPI2_TRESETN */
	DEF_RST(8, 1, 3, 18),		/* RSCI0_PRESETN */
	DEF_RST(8, 2, 3, 19),		/* RSCI0_TRESETN */
	DEF_RST(8, 3, 3, 20),		/* RSCI1_PRESETN */
	DEF_RST(8, 4, 3, 21),		/* RSCI1_TRESETN */
	DEF_RST(8, 5, 3, 22),		/* RSCI2_PRESETN */
	DEF_RST(8, 6, 3, 23),		/* RSCI2_TRESETN */
	DEF_RST(8, 7, 3, 24),		/* RSCI3_PRESETN */
	DEF_RST(8, 8, 3, 25),		/* RSCI3_TRESETN */
	DEF_RST(8, 9, 3, 26),		/* RSCI4_PRESETN */
	DEF_RST(8, 10, 3, 27),		/* RSCI4_TRESETN */
	DEF_RST(8, 11, 3, 28),		/* RSCI5_PRESETN */
	DEF_RST(8, 12, 3, 29),		/* RSCI5_TRESETN */
	DEF_RST(8, 13, 3, 30),		/* RSCI6_PRESETN */
	DEF_RST(8, 14, 3, 31),		/* RSCI6_TRESETN */
	DEF_RST(8, 15, 4, 0),		/* RSCI7_PRESETN */
	DEF_RST(9, 0, 4, 1),		/* RSCI7_TRESETN */
	DEF_RST(9, 1, 4, 2),		/* RSCI8_PRESETN */
	DEF_RST(9, 2, 4, 3),		/* RSCI8_TRESETN */
	DEF_RST(9, 3, 4, 4),		/* RSCI9_PRESETN */
	DEF_RST(9, 4, 4, 5),		/* RSCI9_TRESETN */
	DEF_RST(9, 5, 4, 6),		/* SCIF_0_RST_SYSTEM_N */
	DEF_RST(9, 6, 4, 7),		/* I3C_PRESETN */
	DEF_RST(9, 7, 4, 8),		/* I3C_TRESETN */
	DEF_RST(9, 8, 4, 9),		/* RIIC_0_MRST */
	DEF_RST(9, 9, 4, 10),		/* RIIC_1_MRST */
	DEF_RST(9, 10, 4, 11),		/* RIIC_2_MRST */
	DEF_RST(9, 11, 4, 12),		/* RIIC_3_MRST */
	DEF_RST(9, 12, 4, 13),		/* RIIC_4_MRST */
	DEF_RST(9, 13, 4, 14),		/* RIIC_5_MRST */
	DEF_RST(9, 14, 4, 15),		/* RIIC_6_MRST */
	DEF_RST(9, 15, 4, 16),		/* RIIC_7_MRST */
	DEF_RST(10, 0, 4, 17),		/* RIIC_8_MRST */
	DEF_RST(10, 1, 4, 18),		/* CANFD_RSTP_N */
	DEF_RST(10, 2, 4, 19),		/* CANFD_RSTC_N */
	DEF_RST(10, 3, 4, 20),		/* SPI_HRESETN */
	DEF_RST(10, 4, 4, 21),		/* SPI_ARESETN */
	DEF_RST(10, 7, 4, 24),		/* SDHI_0_IXRST */
	DEF_RST(10, 8, 4, 25),		/* SDHI_1_IXRST */
	DEF_RST(10, 9, 4, 26),		/* SDHI_2_IXRST */
	DEF_RST(10, 10, 4, 27),		/* USB30_ARESETN */
	DEF_RST(10, 11, 4, 28),         /* USB31_ARESETN */
	DEF_RST(10, 12, 4, 29),		/* USB2_U2H0_HRESETN */
	DEF_RST(10, 13, 4, 30),		/* USB2_U2H1_HRESETN */
	DEF_RST(10, 14, 4, 31),		/* USB2_U2P_EXL_SYSRST */
	DEF_RST(10, 15, 5, 0),		/* USB2_PRESETN */
	DEF_RST(11, 0, 5, 1),		/* GBETH0_ARESETN_I */
	DEF_RST(11, 1, 5, 2),		/* GBETH1_ARESETN_I */
	DEF_RST(11, 2, 5, 3),		/* PCIE_ARESETN	*/
	DEF_RST(12, 5, 5, 22),		/* CRU0_PRESETN	*/
	DEF_RST(12, 6, 5, 23),		/* CRU0_ARESETN	*/
	DEF_RST(12, 7, 5, 24),		/* CRU0_S_RESETN */
	DEF_RST(12, 8, 5, 25),		/* CRU1_PRESETN */
	DEF_RST(12, 9, 5, 26),		/* CRU1_ARESETN	*/
	DEF_RST(12, 10, 5, 27),		/* CRU1_S_RESETN */
	DEF_RST(12, 11, 5, 28),		/* CRU2_PRESETN */
	DEF_RST(12, 12, 5, 29),		/* CRU2_ARESETN */
	DEF_RST(12, 13, 5, 30),		/* CRU2_S_RESETN */
	DEF_RST(12, 14, 5, 31),		/* CRU3_PRESETN */
	DEF_RST(12, 15, 6, 0),		/* CRU3_ARESETN */
	DEF_RST(13, 0, 6, 1),		/* CRU3_S_RESETN */
	DEF_RST(13, 5, 6, 6),		/* ISU_ARESETN */
	DEF_RST(13, 6, 6, 7),		/* ISU_PRESETN */
	DEF_RST(13, 7, 6, 8),		/* DSI_PRESETN */
	DEF_RST(13, 8, 6, 9),		/* DSI_ARESETN */
	DEF_RST(13, 12, 6, 13),		/* LCDC_RESET_N */
	DEF_RST(13, 13, 6, 14),		/* GPU_RESETN */
	DEF_RST(13, 14, 6, 15),		/* GPU_AXI_RESETN */
	DEF_RST(13, 15, 6, 16),		/* GPU_ACE_RESETN */
	DEF_RST(14, 0, 6, 17),		/* VCD_RESETN */
	DEF_RST(14, 1, 6, 18),		/* SSIF_0_ASYNC_RESET_SSI */
	DEF_RST(14, 2, 6, 19),		/* SSIF_0_SYNC_RESET_SSI0 */
	DEF_RST(14, 3, 6, 20),		/* SSIF_0_SYNC_RESET_SSI1 */
	DEF_RST(14, 4, 6, 21),		/* SSIF_0_SYNC_RESET_SSI2 */
	DEF_RST(14, 5, 6, 22),		/* SSIF_0_SYNC_RESET_SSI3 */
	DEF_RST(14, 6, 6, 23),		/* SSIF_0_SYNC_RESET_SSI4 */
	DEF_RST(14, 7, 6, 24),		/* SSIF_0_SYNC_RESET_SSI5 */
	DEF_RST(14, 8, 6, 25),		/* SSIF_0_SYNC_RESET_SSI6 */
	DEF_RST(14, 9, 6, 26),		/* SSIF_0_SYNC_RESET_SSI7 */
	DEF_RST(14, 10, 6, 27),		/* SSIF_0_SYNC_RESET_SSI8 */
	DEF_RST(14, 11, 6, 28),		/* SSIF_0_SYNC_RESET_SSI9 */
	DEF_RST(14, 12, 6, 29),		/* SCU_RESET_SRU */
	DEF_RST(14, 13, 6, 30),		/* DMACPP_ARST */
	DEF_RST(14, 14, 6, 31),		/* ADG_RST_RESET_ADG */
	DEF_RST(14, 15, 7, 0),		/* SPDIF_0_RST */
	DEF_RST(15, 0, 7, 1),		/* SPDIF1_RST */
	DEF_RST(15, 1, 7, 2),		/* SPDIF2_RST */
	DEF_RST(15, 6, 7, 7),		/* ADC_ADRST_N */
	DEF_RST(15, 7, 7, 8),		/* TSU0_PRESETN */
	DEF_RST(15, 8, 7, 9),		/* TSU1_PRESETN */
};

const struct rzv2h_cpg_info r9a09g057_cpg_info __initconst = {
	/* Core Clocks */
	.core_clks = r9a09g057_core_clks,
	.num_core_clks = ARRAY_SIZE(r9a09g057_core_clks),
	.last_dt_core_clk = LAST_DT_CORE_CLK,
	.num_total_core_clks = MOD_CLK_BASE,

	/* Module Clocks */
	.mod_clks = r9a09g057_mod_clks,
	.num_mod_clks = ARRAY_SIZE(r9a09g057_mod_clks),
	.num_hw_mod_clks = 25 * 16,

	/* Resets */
	.resets = r9a09g057_resets,
	.num_resets = ARRAY_SIZE(r9a09g057_resets),
};
