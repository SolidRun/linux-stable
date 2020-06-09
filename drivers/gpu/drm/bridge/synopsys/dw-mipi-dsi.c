// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 * Copyright (C) STMicroelectronics SA 2017
 *
 * Modified by Philippe Cornu <philippe.cornu@st.com>
 * This generic Synopsys DesignWare MIPI DSI host driver is based on the
 * Rockchip version from rockchip/dw-mipi-dsi.c with phy & bridge APIs.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <video/mipi_display.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>

#define HWVER_131			0x31333100	/* IP version 1.31 */
#define HWVER_130			0x31333000	/* IP version 1.30 */
#define HWVER_101			0x31303000	/* IP version 1.01 */

#define DSI_VERSION			0x00
#define VERSION				GENMASK(31, 8)

#define DSI_PWR_UP			0x04
#define RESET				0
#define POWERUP				BIT(0)

#define DSI_CLKMGR_CFG			0x08
#define TO_CLK_DIVISION(div)		(((div) & 0xff) << 8)
#define TX_ESC_CLK_DIVISION(div)	((div) & 0xff)

#define DSI_DPI_VCID			0x0c
#define DPI_VCID(vcid)			((vcid) & 0x3)

#define DSI_DPI_COLOR_CODING		0x10
#define DPI_COLOR_CODING_16BIT_1	0x0
#define DPI_COLOR_CODING_16BIT_2	0x1
#define DPI_COLOR_CODING_16BIT_3	0x2
#define DPI_COLOR_CODING_18BIT_1	0x3
#define DPI_COLOR_CODING_18BIT_2	0x4
#define DPI_COLOR_CODING_24BIT		0x5

#define DSI_DPI_CFG_POL			0x14

#define DSI_DPI_LP_CMD_TIM		0x18
#define OUTVACT_LPCMD_TIME(p)		(((p) & 0xff) << 16)
#define INVACT_LPCMD_TIME(p)		((p) & 0xff)

#define DSI_DBI_VCID			0x1c
#define DSI_DBI_CFG			0x20
#define DSI_DBI_PARTITIONING_EN		0x24
#define DSI_DBI_CMDSIZE			0x28

#define DSI_PCKHDL_CFG			0x2c
#define CRC_RX_EN			BIT(4)
#define ECC_RX_EN			BIT(3)
#define BTA_EN				BIT(2)
#define EOTP_RX_EN			BIT(1)
#define EOTP_TX_EN			BIT(0)

#define DSI_GEN_VCID			0x30

#define DSI_MODE_CFG			0x34

#define DSI_VID_MODE_CFG		0x38
#define ENABLE_LOW_POWER		0x3f

#define VID_MODE_TYPE_NON_BURST_SYNC_PULSES	0x0
#define VID_MODE_TYPE_NON_BURST_SYNC_EVENTS	0x1
#define VID_MODE_TYPE_BURST			0x2

#define DSI_VID_PKT_SIZE		0x3c

#define DSI_VID_NUM_CHUNKS		0x40

#define DSI_VID_NULL_SIZE		0x44

#define DSI_VID_HSA_TIME		0x48
#define DSI_VID_HBP_TIME		0x4c
#define DSI_VID_HLINE_TIME		0x50
#define DSI_VID_VSA_LINES		0x54
#define DSI_VID_VBP_LINES		0x58
#define DSI_VID_VFP_LINES		0x5c
#define DSI_VID_VACTIVE_LINES		0x60
#define DSI_EDPI_CMD_SIZE		0x64

#define DSI_CMD_MODE_CFG		0x68

#define DSI_DPI_CFG			0x0c
#define DSI_TMR_LINE_CFG		0x28
#define DSI_VTIMING_CFG			0x2c
#define DSI_PHY_TMR_CFG_V101		0x30
#define DSI_PHY_IF_CFG_V101		0x58
#define DSI_PHY_IF_CTRL			0x5c
#define DSI_PHY_RSTZ_V101		0x54
#define DSI_PHY_STATUS_V101		0x60
#define DSI_PHY_TST_CTRL0_V101		0x64
#define DSI_GEN_HDR_V101		0x34
#define DSI_GEN_PLD_DATA_V101		0x38
#define DSI_CMD_MODE_CFG_V101		0x24
#define DSI_CMD_PKT_STATUS_V101		0x3c
#define DSI_VID_PKT_CFG			0x20
#define DSI_VID_MODE_CFG_V101		0x1c
#define DSI_TO_CNT_CFG_V101		0x40
#define DSI_PCKHDL_CFG_V101		0x18

#define MAX_RD_PKT_SIZE_LP		BIT(24)
#define DCS_LW_TX_LP			BIT(19)
#define DCS_SR_0P_TX_LP			BIT(18)
#define DCS_SW_1P_TX_LP			BIT(17)
#define DCS_SW_0P_TX_LP			BIT(16)
#define GEN_LW_TX_LP			BIT(14)
#define GEN_SR_2P_TX_LP			BIT(13)
#define GEN_SR_1P_TX_LP			BIT(12)
#define GEN_SR_0P_TX_LP			BIT(11)
#define GEN_SW_2P_TX_LP			BIT(10)
#define GEN_SW_1P_TX_LP			BIT(9)
#define GEN_SW_0P_TX_LP			BIT(8)
#define TEAR_FX_EN			BIT(0)

#define CMD_MODE_ALL_LP			(MAX_RD_PKT_SIZE_LP | \
					 DCS_LW_TX_LP | \
					 DCS_SR_0P_TX_LP | \
					 DCS_SW_1P_TX_LP | \
					 DCS_SW_0P_TX_LP | \
					 GEN_LW_TX_LP | \
					 GEN_SR_2P_TX_LP | \
					 GEN_SR_1P_TX_LP | \
					 GEN_SR_0P_TX_LP | \
					 GEN_SW_2P_TX_LP | \
					 GEN_SW_1P_TX_LP | \
					 GEN_SW_0P_TX_LP)

#define EN_TEAR_FX_V101			BIT(14)
#define DCS_LW_TX_LP_V101		BIT(12)
#define GEN_LW_TX_LP_V101		BIT(11)
#define MAX_RD_PKT_SIZE_LP_V101		BIT(10)
#define DCS_SW_2P_TX_LP_V101		BIT(9)
#define DCS_SW_1P_TX_LP_V101		BIT(8)
#define DCS_SW_0P_TX_LP_V101		BIT(7)
#define GEN_SR_2P_TX_LP_V101		BIT(6)
#define GEN_SR_1P_TX_LP_V101		BIT(5)
#define GEN_SR_0P_TX_LP_V101		BIT(4)
#define GEN_SW_2P_TX_LP_V101		BIT(3)
#define GEN_SW_1P_TX_LP_V101		BIT(2)
#define GEN_SW_0P_TX_LP_V101		BIT(1)

#define CMD_MODE_ALL_LP_V101		(DCS_LW_TX_LP_V101 | \
					 GEN_LW_TX_LP_V101 | \
					 MAX_RD_PKT_SIZE_LP_V101 | \
					 DCS_SW_2P_TX_LP_V101 | \
					 DCS_SW_1P_TX_LP_V101 | \
					 DCS_SW_0P_TX_LP_V101 | \
					 GEN_SR_2P_TX_LP_V101 | \
					 GEN_SR_1P_TX_LP_V101 | \
					 GEN_SR_0P_TX_LP_V101 | \
					 GEN_SW_2P_TX_LP_V101 | \
					 GEN_SW_1P_TX_LP_V101 | \
					 GEN_SW_0P_TX_LP_V101)

#define DSI_GEN_HDR			0x6c
#define DSI_GEN_PLD_DATA		0x70

#define DSI_CMD_PKT_STATUS		0x74
#define GEN_RD_CMD_BUSY			BIT(6)
#define GEN_PLD_R_FULL			BIT(5)
#define GEN_PLD_R_EMPTY			BIT(4)
#define GEN_PLD_W_FULL			BIT(3)
#define GEN_PLD_W_EMPTY			BIT(2)
#define GEN_CMD_FULL			BIT(1)
#define GEN_CMD_EMPTY			BIT(0)

#define DSI_TO_CNT_CFG			0x78

#define DSI_HS_RD_TO_CNT		0x7c
#define DSI_LP_RD_TO_CNT		0x80
#define DSI_HS_WR_TO_CNT		0x84
#define DSI_LP_WR_TO_CNT		0x88
#define DSI_BTA_TO_CNT			0x8c

#define DSI_LPCLK_CTRL			0x94
#define DSI_PHY_TMR_LPCLK_CFG		0x98
#define DSI_PHY_TMR_CFG			0x9c
#define DSI_PHY_RSTZ			0xa0
#define DSI_PHY_IF_CFG			0xa4

#define DSI_PHY_STATUS			0xb0
#define PHY_STOP_STATE_CLK_LANE		BIT(2)
#define PHY_LOCK			BIT(0)

#define DSI_PHY_TST_CTRL0		0xb4
#define DSI_PHY_TST_CTRL1		0xb8

#define DSI_INT_ST0			0xbc
#define DSI_INT_ST1			0xc0
#define DSI_INT_MSK0			0xc4
#define DSI_INT_MSK1			0xc8

#define DSI_ERROR_ST0_V101		0x44
#define DSI_ERROR_ST1_V101		0x48
#define DSI_ERROR_MSK0_V101		0x4c
#define DSI_ERROR_MSK1_V101		0x50

#define DSI_PHY_TMR_RD_CFG		0xf4

#define PHY_STATUS_TIMEOUT_US		10000
#define CMD_PKT_STATUS_TIMEOUT_US	20000

#ifdef CONFIG_DEBUG_FS
#define VPG_DEFS(name, dsi) \
	((void __force *)&((*dsi).vpg_defs.name))

#define REGISTER(name, field, dsi) \
	{ #name, VPG_DEFS(name, dsi), field, dsi }

struct debugfs_entries {
	const char				*name;
	bool					*reg;
	struct regmap_field			*field;
	struct dw_mipi_dsi			*dsi;
};
#endif /* CONFIG_DEBUG_FS */

struct dw_mipi_dsi {
	struct drm_bridge bridge;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge *panel_bridge;
	struct device *dev;
	void __iomem *base;
	struct regmap *regs;

	struct clk *pclk;

	bool device_found;
	unsigned int lane_mbps; /* per lane */
	u32 channel;
	u32 lanes;
	u32 format;
	unsigned long mode_flags;
	u32 hw_version;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
	struct debugfs_entries *debugfs_vpg;
	struct {
		bool vpg;
		bool vpg_horizontal;
		bool vpg_ber_pattern;
	} vpg_defs;
#endif /* CONFIG_DEBUG_FS */

	struct dw_mipi_dsi *master; /* dual-dsi master ptr */
	struct dw_mipi_dsi *slave; /* dual-dsi slave ptr */

	const struct dw_mipi_dsi_plat_data *plat_data;

	struct regmap_field	*field_dpi_18loosely_en;
	struct regmap_field	*field_dpi_color_coding;
	struct regmap_field	*field_dpi_vid;
	struct regmap_field	*field_dpi_vsync_active_low;
	struct regmap_field	*field_dpi_hsync_active_low;
	struct regmap_field	*field_cmd_mode_ack_rqst_en;
	struct regmap_field	*field_cmd_mode_all_lp_en;
	struct regmap_field	*field_cmd_mode_en;
	struct regmap_field	*field_cmd_pkt_status;
	struct regmap_field	*field_vid_mode_en;
	struct regmap_field	*field_vid_mode_type;
	struct regmap_field	*field_vid_mode_low_power;
	struct regmap_field	*field_vid_mode_en_lp_cmd;
	struct regmap_field	*field_vid_mode_vpg_en;
	struct regmap_field	*field_vid_mode_vpg_mode;
	struct regmap_field	*field_vid_mode_vpg_horiz;
	struct regmap_field	*field_vid_pkt_size;
	struct regmap_field	*field_vid_hsa_time;
	struct regmap_field	*field_vid_hbp_time;
	struct regmap_field	*field_vid_hline_time;
	struct regmap_field	*field_vid_vsa_time;
	struct regmap_field	*field_vid_vbp_time;
	struct regmap_field	*field_vid_vfp_time;
	struct regmap_field	*field_vid_vactive_time;
	struct regmap_field	*field_phy_txrequestclkhs;
	struct regmap_field	*field_phy_auto_clklane_ctrl;
	struct regmap_field	*field_phy_bta_time;
	struct regmap_field	*field_phy_max_rd_time;
	struct regmap_field	*field_phy_lp2hs_time;
	struct regmap_field	*field_phy_hs2lp_time;
	struct regmap_field	*field_phy_clklp2hs_time;
	struct regmap_field	*field_phy_clkhs2lp_time;
	struct regmap_field	*field_phy_testclr;
	struct regmap_field	*field_phy_unshutdownz;
	struct regmap_field	*field_phy_unrstz;
	struct regmap_field	*field_phy_enableclk;
	struct regmap_field	*field_phy_forcepll;
	struct regmap_field	*field_phy_nlanes;
	struct regmap_field	*field_phy_stop_wait_time;
	struct regmap_field	*field_phy_status;
	struct regmap_field	*field_pckhdl_cfg;
	struct regmap_field	*field_hstx_timeout_counter;
	struct regmap_field	*field_lprx_timeout_counter;
	struct regmap_field	*field_int_stat0;
	struct regmap_field	*field_int_stat1;
	struct regmap_field	*field_int_mask0;
	struct regmap_field	*field_int_mask1;
	struct regmap_field	*field_gen_hdr;
	struct regmap_field	*field_gen_payload;
};

static const struct regmap_config dw_mipi_dsi_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.name = "dw-mipi-dsi",
};

struct dw_mipi_dsi_variant {
	/* Regmap field configs for DSI adapter */
	struct reg_field	cfg_dpi_18loosely_en;
	struct reg_field	cfg_dpi_color_coding;
	struct reg_field	cfg_dpi_vid;
	struct reg_field	cfg_dpi_vsync_active_low;
	struct reg_field	cfg_dpi_hsync_active_low;
	struct reg_field	cfg_cmd_mode_en;
	struct reg_field	cfg_cmd_mode_ack_rqst_en;
	struct reg_field	cfg_cmd_mode_all_lp_en;
	struct reg_field	cfg_cmd_pkt_status;
	struct reg_field	cfg_vid_mode_en;
	struct reg_field	cfg_vid_mode_type;
	struct reg_field	cfg_vid_mode_low_power;
	struct reg_field	cfg_vid_mode_en_lp_cmd;
	struct reg_field	cfg_vid_mode_vpg_en;
	struct reg_field	cfg_vid_mode_vpg_mode;
	struct reg_field	cfg_vid_mode_vpg_horiz;
	struct reg_field	cfg_vid_pkt_size;
	struct reg_field	cfg_vid_hsa_time;
	struct reg_field	cfg_vid_hbp_time;
	struct reg_field	cfg_vid_hline_time;
	struct reg_field	cfg_vid_vsa_time;
	struct reg_field	cfg_vid_vbp_time;
	struct reg_field	cfg_vid_vfp_time;
	struct reg_field	cfg_vid_vactive_time;
	struct reg_field	cfg_phy_txrequestclkhs;
	struct reg_field	cfg_phy_auto_clklane_ctrl;
	struct reg_field	cfg_phy_bta_time;
	struct reg_field	cfg_phy_max_rd_time;
	struct reg_field	cfg_phy_lp2hs_time;
	struct reg_field	cfg_phy_hs2lp_time;
	struct reg_field	cfg_phy_max_rd_time_v131;
	struct reg_field	cfg_phy_lp2hs_time_v131;
	struct reg_field	cfg_phy_hs2lp_time_v131;
	struct reg_field	cfg_phy_clklp2hs_time;
	struct reg_field	cfg_phy_clkhs2lp_time;
	struct reg_field	cfg_phy_testclr;
	struct reg_field	cfg_phy_unshutdownz;
	struct reg_field	cfg_phy_unrstz;
	struct reg_field	cfg_phy_enableclk;
	struct reg_field	cfg_phy_forcepll;
	struct reg_field	cfg_phy_nlanes;
	struct reg_field	cfg_phy_stop_wait_time;
	struct reg_field	cfg_phy_status;
	struct reg_field	cfg_pckhdl_cfg;
	struct reg_field	cfg_hstx_timeout_counter;
	struct reg_field	cfg_lprx_timeout_counter;
	struct reg_field	cfg_int_stat0;
	struct reg_field	cfg_int_stat1;
	struct reg_field	cfg_int_mask0;
	struct reg_field	cfg_int_mask1;
	struct reg_field	cfg_gen_hdr;
	struct reg_field	cfg_gen_payload;
};

static const struct dw_mipi_dsi_variant dw_mipi_dsi_v130_v131_layout = {
	.cfg_dpi_color_coding =		REG_FIELD(DSI_DPI_COLOR_CODING, 0, 3),
	.cfg_dpi_18loosely_en =		REG_FIELD(DSI_DPI_COLOR_CODING, 8, 8),
	.cfg_dpi_vid =			REG_FIELD(DSI_DPI_VCID, 0, 2),
	.cfg_dpi_vsync_active_low =	REG_FIELD(DSI_DPI_CFG_POL, 1, 1),
	.cfg_dpi_hsync_active_low =	REG_FIELD(DSI_DPI_CFG_POL, 2, 2),
	.cfg_cmd_mode_ack_rqst_en =	REG_FIELD(DSI_CMD_MODE_CFG, 1, 1),
	.cfg_cmd_mode_all_lp_en =	REG_FIELD(DSI_CMD_MODE_CFG, 8, 24),
	.cfg_cmd_mode_en =		REG_FIELD(DSI_MODE_CFG, 0, 31),
	.cfg_cmd_pkt_status =		REG_FIELD(DSI_CMD_PKT_STATUS, 0, 31),
	.cfg_vid_mode_en =		REG_FIELD(DSI_MODE_CFG, 0, 31),
	.cfg_vid_mode_type =		REG_FIELD(DSI_VID_MODE_CFG, 0, 1),
	.cfg_vid_mode_low_power =	REG_FIELD(DSI_VID_MODE_CFG, 8, 13),
	.cfg_vid_mode_en_lp_cmd =	REG_FIELD(DSI_VID_MODE_CFG, 15, 15),
	.cfg_vid_mode_vpg_en =		REG_FIELD(DSI_VID_MODE_CFG, 16, 16),
	.cfg_vid_mode_vpg_mode =	REG_FIELD(DSI_VID_MODE_CFG, 20, 20),
	.cfg_vid_mode_vpg_horiz =	REG_FIELD(DSI_VID_MODE_CFG, 24, 24),
	.cfg_vid_pkt_size =		REG_FIELD(DSI_VID_PKT_SIZE, 0, 10),
	.cfg_vid_hsa_time =		REG_FIELD(DSI_VID_HSA_TIME, 0, 31),
	.cfg_vid_hbp_time =		REG_FIELD(DSI_VID_HBP_TIME, 0, 31),
	.cfg_vid_hline_time =		REG_FIELD(DSI_VID_HLINE_TIME, 0, 31),
	.cfg_vid_vsa_time =		REG_FIELD(DSI_VID_VSA_LINES, 0, 31),
	.cfg_vid_vbp_time =		REG_FIELD(DSI_VID_VBP_LINES, 0, 31),
	.cfg_vid_vfp_time =		REG_FIELD(DSI_VID_VFP_LINES, 0, 31),
	.cfg_vid_vactive_time =		REG_FIELD(DSI_VID_VACTIVE_LINES, 0, 31),
	.cfg_phy_txrequestclkhs =	REG_FIELD(DSI_LPCLK_CTRL, 0, 0),
	.cfg_phy_auto_clklane_ctrl =	REG_FIELD(DSI_LPCLK_CTRL, 1, 1),
	.cfg_phy_bta_time =		REG_FIELD(DSI_BTA_TO_CNT, 0, 31),
	.cfg_phy_max_rd_time =		REG_FIELD(DSI_PHY_TMR_CFG, 0, 15),
	.cfg_phy_lp2hs_time =		REG_FIELD(DSI_PHY_TMR_CFG, 16, 23),
	.cfg_phy_hs2lp_time =		REG_FIELD(DSI_PHY_TMR_CFG, 24, 31),
	.cfg_phy_max_rd_time_v131 =	REG_FIELD(DSI_PHY_TMR_RD_CFG, 0, 15),
	.cfg_phy_lp2hs_time_v131 =	REG_FIELD(DSI_PHY_TMR_CFG, 0, 15),
	.cfg_phy_hs2lp_time_v131 =	REG_FIELD(DSI_PHY_TMR_CFG, 16, 31),
	.cfg_phy_clklp2hs_time =	REG_FIELD(DSI_PHY_TMR_LPCLK_CFG, 0, 15),
	.cfg_phy_clkhs2lp_time =	REG_FIELD(DSI_PHY_TMR_LPCLK_CFG, 16, 31),
	.cfg_phy_testclr =		REG_FIELD(DSI_PHY_TST_CTRL0, 0, 0),
	.cfg_phy_unshutdownz =		REG_FIELD(DSI_PHY_RSTZ, 0, 0),
	.cfg_phy_unrstz =		REG_FIELD(DSI_PHY_RSTZ, 1, 1),
	.cfg_phy_enableclk =		REG_FIELD(DSI_PHY_RSTZ, 2, 2),
	.cfg_phy_forcepll =		REG_FIELD(DSI_PHY_RSTZ, 3, 3),
	.cfg_phy_nlanes =		REG_FIELD(DSI_PHY_IF_CFG, 0, 1),
	.cfg_phy_stop_wait_time =	REG_FIELD(DSI_PHY_IF_CFG, 8, 15),
	.cfg_phy_status =		REG_FIELD(DSI_PHY_STATUS, 0, 2),
	.cfg_pckhdl_cfg =		REG_FIELD(DSI_PCKHDL_CFG, 0, 4),
	.cfg_hstx_timeout_counter =	REG_FIELD(DSI_TO_CNT_CFG, 16, 31),
	.cfg_lprx_timeout_counter =	REG_FIELD(DSI_TO_CNT_CFG, 0, 15),
	.cfg_int_stat0 =		REG_FIELD(DSI_INT_ST0, 0, 31),
	.cfg_int_stat1 =		REG_FIELD(DSI_INT_ST1, 0, 31),
	.cfg_int_mask0 =		REG_FIELD(DSI_INT_MSK0, 0, 31),
	.cfg_int_mask1 =		REG_FIELD(DSI_INT_MSK1, 0, 31),
	.cfg_gen_hdr =			REG_FIELD(DSI_GEN_HDR, 0, 31),
	.cfg_gen_payload =		REG_FIELD(DSI_GEN_PLD_DATA, 0, 31),
};

static const struct dw_mipi_dsi_variant dw_mipi_dsi_v101_layout = {
	.cfg_dpi_vid =			REG_FIELD(DSI_DPI_CFG, 0, 1),
	.cfg_dpi_color_coding =		REG_FIELD(DSI_DPI_CFG, 2, 4),
	.cfg_dpi_18loosely_en =		REG_FIELD(DSI_DPI_CFG, 10, 10),
	.cfg_dpi_vsync_active_low =	REG_FIELD(DSI_DPI_CFG, 6, 6),
	.cfg_dpi_hsync_active_low =	REG_FIELD(DSI_DPI_CFG, 7, 7),
	.cfg_cmd_mode_en =		REG_FIELD(DSI_CMD_MODE_CFG_V101, 0, 0),
	.cfg_cmd_mode_all_lp_en =	REG_FIELD(DSI_CMD_MODE_CFG_V101, 1, 12),
	.cfg_cmd_mode_ack_rqst_en =	REG_FIELD(DSI_CMD_MODE_CFG_V101, 13, 13),
	.cfg_cmd_pkt_status =		REG_FIELD(DSI_CMD_PKT_STATUS_V101, 0, 14),
	.cfg_vid_mode_en =		REG_FIELD(DSI_VID_MODE_CFG_V101, 0, 0),
	.cfg_vid_mode_type =		REG_FIELD(DSI_VID_MODE_CFG_V101, 1, 2),
	.cfg_vid_mode_low_power =	REG_FIELD(DSI_VID_MODE_CFG_V101, 3, 8),
	.cfg_vid_pkt_size =		REG_FIELD(DSI_VID_PKT_CFG, 0, 10),
	.cfg_vid_hsa_time =		REG_FIELD(DSI_TMR_LINE_CFG, 0, 8),
	.cfg_vid_hbp_time =		REG_FIELD(DSI_TMR_LINE_CFG, 9, 17),
	.cfg_vid_hline_time =		REG_FIELD(DSI_TMR_LINE_CFG, 18, 31),
	.cfg_vid_vsa_time =		REG_FIELD(DSI_VTIMING_CFG, 0, 3),
	.cfg_vid_vbp_time =		REG_FIELD(DSI_VTIMING_CFG, 4, 9),
	.cfg_vid_vfp_time =		REG_FIELD(DSI_VTIMING_CFG, 10, 15),
	.cfg_vid_vactive_time =		REG_FIELD(DSI_VTIMING_CFG, 16, 26),
	.cfg_phy_txrequestclkhs =	REG_FIELD(DSI_PHY_IF_CTRL, 0, 0),
	.cfg_phy_bta_time =		REG_FIELD(DSI_PHY_TMR_CFG_V101, 0, 11),
	.cfg_phy_lp2hs_time =		REG_FIELD(DSI_PHY_TMR_CFG_V101, 12, 19),
	.cfg_phy_hs2lp_time =		REG_FIELD(DSI_PHY_TMR_CFG_V101, 20, 27),
	.cfg_phy_testclr =		REG_FIELD(DSI_PHY_TST_CTRL0_V101, 0, 0),
	.cfg_phy_unshutdownz =		REG_FIELD(DSI_PHY_RSTZ_V101, 0, 0),
	.cfg_phy_unrstz =		REG_FIELD(DSI_PHY_RSTZ_V101, 1, 1),
	.cfg_phy_enableclk =		REG_FIELD(DSI_PHY_RSTZ_V101, 2, 2),
	.cfg_phy_nlanes =		REG_FIELD(DSI_PHY_IF_CFG_V101, 0, 1),
	.cfg_phy_stop_wait_time =	REG_FIELD(DSI_PHY_IF_CFG_V101, 2, 9),
	.cfg_phy_status =		REG_FIELD(DSI_PHY_STATUS_V101, 0, 2),
	.cfg_pckhdl_cfg =		REG_FIELD(DSI_PCKHDL_CFG_V101, 0, 4),
	.cfg_hstx_timeout_counter =	REG_FIELD(DSI_TO_CNT_CFG_V101, 0, 15),
	.cfg_lprx_timeout_counter =	REG_FIELD(DSI_TO_CNT_CFG_V101, 16, 31),
	.cfg_int_stat0 =		REG_FIELD(DSI_ERROR_ST0_V101, 0, 20),
	.cfg_int_stat1 =		REG_FIELD(DSI_ERROR_ST1_V101, 0, 17),
	.cfg_int_mask0 =		REG_FIELD(DSI_ERROR_MSK0_V101, 0, 20),
	.cfg_int_mask1 =		REG_FIELD(DSI_ERROR_MSK1_V101, 0, 17),
	.cfg_gen_hdr =			REG_FIELD(DSI_GEN_HDR_V101, 0, 31),
	.cfg_gen_payload =		REG_FIELD(DSI_GEN_PLD_DATA_V101, 0, 31),
};

/*
 * Check if either a link to a master or slave is present
 */
static inline bool dw_mipi_is_dual_mode(struct dw_mipi_dsi *dsi)
{
	return dsi->slave || dsi->master;
}

/*
 * The controller should generate 2 frames before
 * preparing the peripheral.
 */
static void dw_mipi_dsi_wait_for_two_frames(const struct drm_display_mode *mode)
{
	int refresh, two_frames;

	refresh = drm_mode_vrefresh(mode);
	two_frames = DIV_ROUND_UP(MSEC_PER_SEC, refresh) * 2;
	msleep(two_frames);
}

static inline struct dw_mipi_dsi *host_to_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct dw_mipi_dsi, dsi_host);
}

static inline struct dw_mipi_dsi *bridge_to_dsi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct dw_mipi_dsi, bridge);
}

static int dw_mipi_dsi_panel_or_bridge(struct dw_mipi_dsi *dsi,
				       struct device_node *node)
{
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	int ret;

	ret = drm_of_find_panel_or_bridge(node, 1, 0, &panel, &bridge);
	if (ret)
		return ret;

	if (panel) {
		bridge = drm_panel_bridge_add_typed(panel,
						    DRM_MODE_CONNECTOR_DSI);
		if (IS_ERR(bridge))
			return PTR_ERR(bridge);
	}

	dsi->panel_bridge = bridge;

	if (!dsi->panel_bridge)
		return -EPROBE_DEFER;

	return 0;
}

static int dw_mipi_dsi_host_attach(struct mipi_dsi_host *host,
				   struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi *dsi = host_to_dsi(host);
	const struct dw_mipi_dsi_plat_data *pdata = dsi->plat_data;
	int ret;

	if (device->lanes > dsi->plat_data->max_data_lanes) {
		dev_err(dsi->dev, "the number of data lanes(%u) is too many\n",
			device->lanes);
		return -EINVAL;
	}

	dsi->lanes = device->lanes;
	dsi->channel = device->channel;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;

	if (!dsi->device_found) {
		ret = dw_mipi_dsi_panel_or_bridge(dsi, host->dev->of_node);
		if (ret)
			return ret;

		dsi->device_found = true;
	}

	if (pdata->host_ops && pdata->host_ops->attach) {
		ret = pdata->host_ops->attach(pdata->priv_data, device);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int dw_mipi_dsi_host_detach(struct mipi_dsi_host *host,
				   struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi *dsi = host_to_dsi(host);
	const struct dw_mipi_dsi_plat_data *pdata = dsi->plat_data;
	int ret;

	if (pdata->host_ops && pdata->host_ops->detach) {
		ret = pdata->host_ops->detach(pdata->priv_data, device);
		if (ret < 0)
			return ret;
	}

	drm_of_panel_bridge_remove(host->dev->of_node, 1, 0);

	drm_bridge_remove(&dsi->bridge);

	return 0;
}

static void dw_mipi_message_config(struct dw_mipi_dsi *dsi,
				   const struct mipi_dsi_msg *msg)
{
	bool lpm = msg->flags & MIPI_DSI_MSG_USE_LPM;
	u32 cmd_mode_lp = 0;

	switch (dsi->hw_version) {
	case HWVER_130:
	case HWVER_131:
		cmd_mode_lp = CMD_MODE_ALL_LP;
		break;
	case HWVER_101:
		cmd_mode_lp = CMD_MODE_ALL_LP_V101;
		break;
	}

	/*
	 * TODO dw drv improvements
	 * largest packet sizes during hfp or during vsa/vpb/vfp
	 * should be computed according to byte lane, lane number and only
	 * if sending lp cmds in high speed is enable (PHY_TXREQUESTCLKHS)
	 */
	regmap_write(dsi->regs, DSI_DPI_LP_CMD_TIM, OUTVACT_LPCMD_TIME(16)
		  | INVACT_LPCMD_TIME(4));

	if (msg->flags & MIPI_DSI_MSG_REQ_ACK)
		regmap_field_write(dsi->field_cmd_mode_ack_rqst_en, 1);
	if (lpm)
		regmap_field_write(dsi->field_cmd_mode_all_lp_en, cmd_mode_lp);

	regmap_field_write(dsi->field_vid_mode_en_lp_cmd, lpm ? 1 : 0);
}

static int dw_mipi_dsi_gen_pkt_hdr_write(struct dw_mipi_dsi *dsi, u32 hdr_val)
{
	int ret;
	u32 val, mask;

	ret = regmap_field_read_poll_timeout(dsi->field_cmd_pkt_status,
					     val, !(val & GEN_CMD_FULL),
					     1000, CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev, "failed to get available command FIFO\n");
		return ret;
	}

	regmap_field_write(dsi->field_gen_hdr, hdr_val);

	mask = GEN_CMD_EMPTY | GEN_PLD_W_EMPTY;
	ret = regmap_field_read_poll_timeout(dsi->field_cmd_pkt_status,
				       val, (val & mask) == mask,
				       1000, CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev, "failed to write command FIFO\n");
		return ret;
	}

	return 0;
}

static int dw_mipi_dsi_write(struct dw_mipi_dsi *dsi,
			     const struct mipi_dsi_packet *packet)
{
	const u8 *tx_buf = packet->payload;
	int len = packet->payload_length, pld_data_bytes = sizeof(u32), ret;
	__le32 word;
	u32 val;

	while (len) {
		if (len < pld_data_bytes) {
			word = 0;
			memcpy(&word, tx_buf, len);
			regmap_field_force_write(dsi->field_gen_payload,
						 le32_to_cpu(word));
			len = 0;
		} else {
			memcpy(&word, tx_buf, pld_data_bytes);
			regmap_field_force_write(dsi->field_gen_payload,
						 le32_to_cpu(word));
			tx_buf += pld_data_bytes;
			len -= pld_data_bytes;
		}

		ret = regmap_field_read_poll_timeout(dsi->field_cmd_pkt_status,
						     val,
						     !(val & GEN_PLD_W_FULL),
						     1000,
						     CMD_PKT_STATUS_TIMEOUT_US);
		if (ret) {
			dev_err(dsi->dev,
				"failed to get available write payload FIFO\n");
			return ret;
		}
	}

	word = 0;
	memcpy(&word, packet->header, sizeof(packet->header));
	return dw_mipi_dsi_gen_pkt_hdr_write(dsi, le32_to_cpu(word));
}

static int dw_mipi_dsi_read(struct dw_mipi_dsi *dsi,
			    const struct mipi_dsi_msg *msg)
{
	int i, j, ret, len = msg->rx_len;
	u8 *buf = msg->rx_buf;
	u32 val;

	/* Wait end of the read operation */
	ret = regmap_field_read_poll_timeout(dsi->field_cmd_pkt_status,
					     val, !(val & GEN_RD_CMD_BUSY),
					     1000, CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev, "Timeout during read operation\n");
		return ret;
	}

	for (i = 0; i < len; i += 4) {
		/* Read fifo must not be empty before all bytes are read */
		ret = regmap_field_read_poll_timeout(dsi->field_cmd_pkt_status,
						     val,
						     !(val & GEN_PLD_R_EMPTY),
						     1000,
						     CMD_PKT_STATUS_TIMEOUT_US);
		if (ret) {
			dev_err(dsi->dev, "Read payload FIFO is empty\n");
			return ret;
		}

		regmap_field_read(dsi->field_gen_payload, &val);
		for (j = 0; j < 4 && j + i < len; j++)
			buf[i + j] = val >> (8 * j);
	}

	return ret;
}

static ssize_t dw_mipi_dsi_host_transfer(struct mipi_dsi_host *host,
					 const struct mipi_dsi_msg *msg)
{
	struct dw_mipi_dsi *dsi = host_to_dsi(host);
	struct mipi_dsi_packet packet;
	int ret, nb_bytes;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret) {
		dev_err(dsi->dev, "failed to create packet: %d\n", ret);
		return ret;
	}

	dw_mipi_message_config(dsi, msg);
	if (dsi->slave)
		dw_mipi_message_config(dsi->slave, msg);

	ret = dw_mipi_dsi_write(dsi, &packet);
	if (ret)
		return ret;
	if (dsi->slave) {
		ret = dw_mipi_dsi_write(dsi->slave, &packet);
		if (ret)
			return ret;
	}

	if (msg->rx_buf && msg->rx_len) {
		ret = dw_mipi_dsi_read(dsi, msg);
		if (ret)
			return ret;
		nb_bytes = msg->rx_len;
	} else {
		nb_bytes = packet.size;
	}

	return nb_bytes;
}

static const struct mipi_dsi_host_ops dw_mipi_dsi_host_ops = {
	.attach = dw_mipi_dsi_host_attach,
	.detach = dw_mipi_dsi_host_detach,
	.transfer = dw_mipi_dsi_host_transfer,
};

static void dw_mipi_dsi_video_mode_config(struct dw_mipi_dsi *dsi)
{
	/*
	 * TODO dw drv improvements
	 * enabling low power is panel-dependent, we should use the
	 * panel configuration here...
	 */
	regmap_field_write(dsi->field_vid_mode_low_power, ENABLE_LOW_POWER);

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		regmap_field_write(dsi->field_vid_mode_type,
				   VID_MODE_TYPE_BURST);
	else if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		regmap_field_write(dsi->field_vid_mode_type,
				   VID_MODE_TYPE_NON_BURST_SYNC_PULSES);
	else
		regmap_field_write(dsi->field_vid_mode_type,
				   VID_MODE_TYPE_NON_BURST_SYNC_EVENTS);

#ifdef CONFIG_DEBUG_FS
	if (dsi->hw_version > HWVER_101 && dsi->vpg_defs.vpg) {
		regmap_field_write(dsi->field_vid_mode_vpg_en, 1);
		regmap_field_write(dsi->field_vid_mode_vpg_horiz,
				   dsi->vpg_defs.vpg_horizontal ? 1 : 0);
		regmap_field_write(dsi->field_vid_mode_vpg_mode,
				   dsi->vpg_defs.vpg_ber_pattern ? 1 : 0);
	}
#endif /* CONFIG_DEBUG_FS */
}

static void dw_mipi_dsi_set_mode(struct dw_mipi_dsi *dsi,
				 unsigned long mode_flags)
{
	regmap_write(dsi->regs, DSI_PWR_UP, RESET);

	if (mode_flags & MIPI_DSI_MODE_VIDEO) {
		regmap_field_write(dsi->field_cmd_mode_en, 0);
		dw_mipi_dsi_video_mode_config(dsi);

		if (dsi->hw_version == HWVER_101)
			regmap_field_write(dsi->field_vid_mode_en, 1);
	} else {
		regmap_field_write(dsi->field_cmd_mode_en, 1);

		if (dsi->hw_version == HWVER_101)
			regmap_field_write(dsi->field_vid_mode_en, 0);
	}

	regmap_field_write(dsi->field_phy_txrequestclkhs, 1);
	if (dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		regmap_field_write(dsi->field_phy_auto_clklane_ctrl, 1);

	regmap_write(dsi->regs, DSI_PWR_UP, POWERUP);
}

static void dw_mipi_dsi_disable(struct dw_mipi_dsi *dsi)
{
	regmap_write(dsi->regs, DSI_PWR_UP, RESET);
	regmap_field_write(dsi->field_phy_unrstz, 0);
}

static void dw_mipi_dsi_init(struct dw_mipi_dsi *dsi)
{
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;
	unsigned int esc_rate; /* in MHz */
	u32 esc_clk_division;
	int ret;

	/*
	 * The maximum permitted escape clock is 20MHz and it is derived from
	 * lanebyteclk, which is running at "lane_mbps / 8".
	 */
	if (phy_ops->get_esc_clk_rate) {
		ret = phy_ops->get_esc_clk_rate(dsi->plat_data->priv_data,
						&esc_rate);
		if (ret)
			DRM_DEBUG_DRIVER("Phy get_esc_clk_rate() failed\n");
	} else
		esc_rate = 20; /* Default to 20MHz */

	/*
	 * We want :
	 *     (lane_mbps >> 3) / esc_clk_division < X
	 * which is:
	 *     (lane_mbps >> 3) / X > esc_clk_division
	 */
	esc_clk_division = (dsi->lane_mbps >> 3) / esc_rate + 1;

	regmap_write(dsi->regs, DSI_PWR_UP, RESET);

	/*
	 * TODO dw drv improvements
	 * timeout clock division should be computed with the
	 * high speed transmission counter timeout and byte lane...
	 */
	regmap_write(dsi->regs, DSI_CLKMGR_CFG, TO_CLK_DIVISION(10) |
		  TX_ESC_CLK_DIVISION(esc_clk_division));
}

static void dw_mipi_dsi_dpi_config(struct dw_mipi_dsi *dsi,
				   const struct drm_display_mode *mode)
{
	u32 color = 0;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		color = DPI_COLOR_CODING_24BIT;
		break;
	case MIPI_DSI_FMT_RGB666:
		color = DPI_COLOR_CODING_18BIT_2;
		regmap_field_write(dsi->field_dpi_18loosely_en, 1);
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		color = DPI_COLOR_CODING_18BIT_1;
		break;
	case MIPI_DSI_FMT_RGB565:
		color = DPI_COLOR_CODING_16BIT_1;
		break;
	}

	regmap_field_write(dsi->field_dpi_vid, DPI_VCID(dsi->channel));
	regmap_field_write(dsi->field_dpi_color_coding, color);

	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		regmap_field_write(dsi->field_dpi_vsync_active_low, 1);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		regmap_field_write(dsi->field_dpi_hsync_active_low, 1);
}

static void dw_mipi_dsi_packet_handler_config(struct dw_mipi_dsi *dsi)
{
	regmap_field_write(dsi->field_pckhdl_cfg,
			   CRC_RX_EN | ECC_RX_EN | BTA_EN);
}

static void dw_mipi_dsi_video_packet_config(struct dw_mipi_dsi *dsi,
					    const struct drm_display_mode *mode)
{
	/*
	 * TODO dw drv improvements
	 * only burst mode is supported here. For non-burst video modes,
	 * we should compute DSI_VID_PKT_SIZE, DSI_VCCR.NUMC &
	 * DSI_VNPCR.NPSIZE... especially because this driver supports
	 * non-burst video modes, see dw_mipi_dsi_video_mode_config()...
	 */
	regmap_field_write(dsi->field_vid_pkt_size,
			   dw_mipi_is_dual_mode(dsi) ?
				mode->hdisplay / 2 : mode->hdisplay);
}

static void dw_mipi_dsi_command_mode_config(struct dw_mipi_dsi *dsi)
{
	/*
	 * TODO dw drv improvements
	 * compute high speed transmission counter timeout according
	 * to the timeout clock division (TO_CLK_DIVISION) and byte lane...
	 */
	regmap_field_write(dsi->field_hstx_timeout_counter, 1000);
	regmap_field_write(dsi->field_lprx_timeout_counter, 1000);

	/*
	 * TODO dw drv improvements
	 * the Bus-Turn-Around Timeout Counter should be computed
	 * according to byte lane...
	 */
	regmap_field_write(dsi->field_phy_bta_time, 0xd00);

	regmap_field_write(dsi->field_cmd_mode_en, 1);
}

/* Get lane byte clock cycles. */
static u32 dw_mipi_dsi_get_hcomponent_lbcc(struct dw_mipi_dsi *dsi,
					   const struct drm_display_mode *mode,
					   u32 hcomponent)
{
	u32 frac, lbcc;

	lbcc = hcomponent * dsi->lane_mbps * MSEC_PER_SEC / 8;

	frac = lbcc % mode->clock;
	lbcc = lbcc / mode->clock;
	if (frac)
		lbcc++;

	return lbcc;
}

static void dw_mipi_dsi_line_timer_config(struct dw_mipi_dsi *dsi,
					  const struct drm_display_mode *mode)
{
	u32 htotal, hsa, hbp, lbcc;

	htotal = mode->htotal;
	hsa = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	/*
	 * TODO dw drv improvements
	 * computations below may be improved...
	 */
	lbcc = dw_mipi_dsi_get_hcomponent_lbcc(dsi, mode, htotal);
	regmap_field_write(dsi->field_vid_hline_time, lbcc);

	lbcc = dw_mipi_dsi_get_hcomponent_lbcc(dsi, mode, hsa);
	regmap_field_write(dsi->field_vid_hsa_time, lbcc);

	lbcc = dw_mipi_dsi_get_hcomponent_lbcc(dsi, mode, hbp);
	regmap_field_write(dsi->field_vid_hbp_time, lbcc);
}

static void dw_mipi_dsi_vertical_timing_config(struct dw_mipi_dsi *dsi,
					const struct drm_display_mode *mode)
{
	u32 vactive, vsa, vfp, vbp;

	vactive = mode->vdisplay;
	vsa = mode->vsync_end - mode->vsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	regmap_field_write(dsi->field_vid_vactive_time, vactive);
	regmap_field_write(dsi->field_vid_vsa_time, vsa);
	regmap_field_write(dsi->field_vid_vfp_time, vfp);
	regmap_field_write(dsi->field_vid_vbp_time, vbp);
}

static void dw_mipi_dsi_dphy_timing_config(struct dw_mipi_dsi *dsi)
{
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;
	struct dw_mipi_dsi_dphy_timing timing;
	int ret;

	ret = phy_ops->get_timing(dsi->plat_data->priv_data,
				  dsi->lane_mbps, &timing);
	if (ret)
		DRM_DEV_ERROR(dsi->dev, "Retrieving phy timings failed\n");

	/*
	 * TODO dw drv improvements
	 * data & clock lane timers should be computed according to panel
	 * blankings and to the automatic clock lane control mode...
	 * note: DSI_PHY_TMR_CFG.MAX_RD_TIME should be in line with
	 * DSI_CMD_MODE_CFG.MAX_RD_PKT_SIZE_LP (see CMD_MODE_ALL_LP)
	 */
	regmap_field_write(dsi->field_phy_lp2hs_time, timing.data_lp2hs);
	regmap_field_write(dsi->field_phy_hs2lp_time, timing.data_hs2lp);

	if (dsi->hw_version > HWVER_101) {
		regmap_field_write(dsi->field_phy_max_rd_time, 10000);
		regmap_field_write(dsi->field_phy_clkhs2lp_time,
				   timing.clk_hs2lp);
		regmap_field_write(dsi->field_phy_clklp2hs_time,
				   timing.clk_lp2hs);
	}
}

static void dw_mipi_dsi_dphy_interface_config(struct dw_mipi_dsi *dsi)
{
	/*
	 * TODO dw drv improvements
	 * stop wait time should be the maximum between host dsi
	 * and panel stop wait times
	 */
	regmap_field_write(dsi->field_phy_stop_wait_time, 0x20);
	regmap_field_write(dsi->field_phy_nlanes, dsi->lanes - 1);
}

static void dw_mipi_dsi_dphy_init(struct dw_mipi_dsi *dsi)
{
	/* Clear PHY state */
	regmap_field_write(dsi->field_phy_enableclk, 0);
	regmap_field_write(dsi->field_phy_unrstz, 0);
	regmap_field_write(dsi->field_phy_unshutdownz, 0);

	if (dsi->hw_version > HWVER_101)
		regmap_field_write(dsi->field_phy_forcepll, 0);

	regmap_field_write(dsi->field_phy_testclr, 0);
	regmap_field_write(dsi->field_phy_testclr, 1);
	regmap_field_write(dsi->field_phy_testclr, 0);
}

static void dw_mipi_dsi_dphy_enable(struct dw_mipi_dsi *dsi)
{
	u32 val;
	int ret;

	regmap_field_write(dsi->field_phy_enableclk, 1);
	regmap_field_write(dsi->field_phy_unrstz, 1);
	regmap_field_write(dsi->field_phy_unshutdownz, 1);

	if (dsi->hw_version > HWVER_101)
		regmap_field_write(dsi->field_phy_forcepll, 1);

	ret = regmap_field_read_poll_timeout(dsi->field_phy_status,
					     val, val & PHY_LOCK,
					     1000, PHY_STATUS_TIMEOUT_US);
	if (ret)
		DRM_DEBUG_DRIVER("failed to wait phy lock state\n");

	ret = regmap_field_read_poll_timeout(dsi->field_phy_status,
					     val, val & PHY_STOP_STATE_CLK_LANE,
					     1000, PHY_STATUS_TIMEOUT_US);
	if (ret)
		DRM_DEBUG_DRIVER("failed to wait phy clk lane stop state\n");
}

static void dw_mipi_dsi_clear_err(struct dw_mipi_dsi *dsi)
{
	u32 val;

	regmap_field_read(dsi->field_int_stat0, &val);
	regmap_field_read(dsi->field_int_stat1, &val);
	regmap_field_write(dsi->field_int_mask0, 0);
	regmap_field_write(dsi->field_int_mask1, 0);
}

static void dw_mipi_dsi_bridge_post_atomic_disable(struct drm_bridge *bridge,
						   struct drm_bridge_state *old_bridge_state)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;

	/*
	 * Switch to command mode before panel-bridge post_disable &
	 * panel unprepare.
	 * Note: panel-bridge disable & panel disable has been called
	 * before by the drm framework.
	 */
	dw_mipi_dsi_set_mode(dsi, 0);

	/*
	 * TODO Only way found to call panel-bridge post_disable &
	 * panel unprepare before the dsi "final" disable...
	 * This needs to be fixed in the drm_bridge framework and the API
	 * needs to be updated to manage our own call chains...
	 */
	if (dsi->panel_bridge->funcs->post_disable)
		dsi->panel_bridge->funcs->post_disable(dsi->panel_bridge);

	if (phy_ops->power_off)
		phy_ops->power_off(dsi->plat_data->priv_data);

	if (dsi->slave) {
		dw_mipi_dsi_disable(dsi->slave);
		clk_disable_unprepare(dsi->slave->pclk);
		pm_runtime_put(dsi->slave->dev);
	}
	dw_mipi_dsi_disable(dsi);

	clk_disable_unprepare(dsi->pclk);
	pm_runtime_put(dsi->dev);
}

static unsigned int dw_mipi_dsi_get_lanes(struct dw_mipi_dsi *dsi)
{
	/* this instance is the slave, so add the master's lanes */
	if (dsi->master)
		return dsi->master->lanes + dsi->lanes;

	/* this instance is the master, so add the slave's lanes */
	if (dsi->slave)
		return dsi->lanes + dsi->slave->lanes;

	/* single-dsi, so no other instance to consider */
	return dsi->lanes;
}

static void dw_mipi_dsi_mode_set(struct dw_mipi_dsi *dsi,
				 const struct drm_display_mode *adjusted_mode)
{
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;
	void *priv_data = dsi->plat_data->priv_data;
	int ret;
	u32 lanes = dw_mipi_dsi_get_lanes(dsi);

	clk_prepare_enable(dsi->pclk);

	ret = phy_ops->get_lane_mbps(priv_data, adjusted_mode, dsi->mode_flags,
				     lanes, dsi->format, &dsi->lane_mbps);
	if (ret)
		DRM_DEBUG_DRIVER("Phy get_lane_mbps() failed\n");

	pm_runtime_get_sync(dsi->dev);
	dw_mipi_dsi_init(dsi);
	dw_mipi_dsi_dpi_config(dsi, adjusted_mode);
	dw_mipi_dsi_packet_handler_config(dsi);
	dw_mipi_dsi_video_mode_config(dsi);
	dw_mipi_dsi_video_packet_config(dsi, adjusted_mode);
	dw_mipi_dsi_command_mode_config(dsi);
	dw_mipi_dsi_line_timer_config(dsi, adjusted_mode);
	dw_mipi_dsi_vertical_timing_config(dsi, adjusted_mode);

	dw_mipi_dsi_dphy_init(dsi);
	dw_mipi_dsi_dphy_timing_config(dsi);
	dw_mipi_dsi_dphy_interface_config(dsi);

	dw_mipi_dsi_clear_err(dsi);

	ret = phy_ops->init(priv_data);
	if (ret)
		DRM_DEBUG_DRIVER("Phy init() failed\n");

	dw_mipi_dsi_dphy_enable(dsi);

	dw_mipi_dsi_wait_for_two_frames(adjusted_mode);

	/* Switch to cmd mode for panel-bridge pre_enable & panel prepare */
	dw_mipi_dsi_set_mode(dsi, 0);

	if (phy_ops->power_on)
		phy_ops->power_on(dsi->plat_data->priv_data);
}

static void dw_mipi_dsi_bridge_mode_set(struct drm_bridge *bridge,
					const struct drm_display_mode *mode,
					const struct drm_display_mode *adjusted_mode)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);

	dw_mipi_dsi_mode_set(dsi, adjusted_mode);
	if (dsi->slave)
		dw_mipi_dsi_mode_set(dsi->slave, adjusted_mode);
}

static void dw_mipi_dsi_bridge_atomic_enable(struct drm_bridge *bridge,
					     struct drm_bridge_state *old_bridge_state)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);

	/* Switch to video mode for panel-bridge enable & panel enable */
	dw_mipi_dsi_set_mode(dsi, MIPI_DSI_MODE_VIDEO);
	if (dsi->slave)
		dw_mipi_dsi_set_mode(dsi->slave, MIPI_DSI_MODE_VIDEO);
}

static enum drm_mode_status
dw_mipi_dsi_bridge_mode_valid(struct drm_bridge *bridge,
			      const struct drm_display_info *info,
			      const struct drm_display_mode *mode)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);
	const struct dw_mipi_dsi_plat_data *pdata = dsi->plat_data;
	enum drm_mode_status mode_status = MODE_OK;

	if (pdata->mode_valid)
		mode_status = pdata->mode_valid(pdata->priv_data, mode,
						dsi->mode_flags,
						dw_mipi_dsi_get_lanes(dsi),
						dsi->format);

	return mode_status;
}

static int dw_mipi_dsi_bridge_attach(struct drm_bridge *bridge,
				     enum drm_bridge_attach_flags flags)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found\n");
		return -ENODEV;
	}

	/* Set the encoder type as caller does not know it */
	bridge->encoder->encoder_type = DRM_MODE_ENCODER_DSI;

	if (!dsi->device_found) {
		int ret;

		ret = dw_mipi_dsi_panel_or_bridge(dsi, dsi->dev->of_node);
		if (ret)
			return ret;

		dsi->device_found = true;
	}

	/* Attach the panel-bridge to the dsi bridge */
	return drm_bridge_attach(bridge->encoder, dsi->panel_bridge, bridge,
				 flags);
}

static const struct drm_bridge_funcs dw_mipi_dsi_bridge_funcs = {
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset		= drm_atomic_helper_bridge_reset,
	.atomic_enable		= dw_mipi_dsi_bridge_atomic_enable,
	.atomic_post_disable	= dw_mipi_dsi_bridge_post_atomic_disable,
	.mode_set		= dw_mipi_dsi_bridge_mode_set,
	.mode_valid		= dw_mipi_dsi_bridge_mode_valid,
	.attach			= dw_mipi_dsi_bridge_attach,
};

#ifdef CONFIG_DEBUG_FS

static int dw_mipi_dsi_debugfs_write(void *data, u64 val)
{
	struct debugfs_entries *vpg = data;
	struct dw_mipi_dsi *dsi;

	if (!vpg)
		return -ENODEV;

	dsi = vpg->dsi;

	*vpg->reg = (bool)val;

	regmap_field_write(vpg->field, *vpg->reg ? 1 : 0);

	return 0;
}

static int dw_mipi_dsi_debugfs_show(void *data, u64 *val)
{
	struct debugfs_entries *vpg = data;

	if (!vpg)
		return -ENODEV;

	*val = *vpg->reg;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_x32, dw_mipi_dsi_debugfs_show,
			 dw_mipi_dsi_debugfs_write, "%llu\n");

static void debugfs_create_files(void *data)
{
	struct dw_mipi_dsi *dsi = data;
	struct debugfs_entries debugfs[] = {
		REGISTER(vpg, dsi->field_vid_mode_vpg_en, dsi),
		REGISTER(vpg_horizontal, dsi->field_vid_mode_vpg_horiz, dsi),
		REGISTER(vpg_ber_pattern, dsi->field_vid_mode_vpg_mode, dsi),
	};
	int i;

	dsi->debugfs_vpg = kmemdup(debugfs, sizeof(debugfs), GFP_KERNEL);
	if (!dsi->debugfs_vpg)
		return;

	for (i = 0; i < ARRAY_SIZE(debugfs); i++)
		debugfs_create_file(dsi->debugfs_vpg[i].name, 0644,
				    dsi->debugfs, &dsi->debugfs_vpg[i],
				    &fops_x32);
}

static void dw_mipi_dsi_debugfs_init(struct dw_mipi_dsi *dsi)
{
	dsi->debugfs = debugfs_create_dir(dev_name(dsi->dev), NULL);
	if (IS_ERR(dsi->debugfs)) {
		dev_err(dsi->dev, "failed to create debugfs root\n");
		return;
	}

	debugfs_create_files(dsi);
}

static void dw_mipi_dsi_debugfs_remove(struct dw_mipi_dsi *dsi)
{
	debugfs_remove_recursive(dsi->debugfs);
	kfree(dsi->debugfs_vpg);
}

#else

static void dw_mipi_dsi_debugfs_init(struct dw_mipi_dsi *dsi) { }
static void dw_mipi_dsi_debugfs_remove(struct dw_mipi_dsi *dsi) { }

#endif /* CONFIG_DEBUG_FS */

static int dw_mipi_dsi_get_hw_version(struct dw_mipi_dsi *dsi)
{
	regmap_read(dsi->regs, DSI_VERSION, &dsi->hw_version);
	dsi->hw_version &= VERSION;
	if (!dsi->hw_version) {
		dev_err(dsi->dev,
			"Failed to read DSI version. Is pclk enabled?\n");
		return -ENODEV;
	}
	return 0;
}

#define INIT_FIELD(f) INIT_FIELD_CFG(field_##f, cfg_##f)
#define INIT_FIELD_CFG(f, conf)	({					\
		dsi->f = devm_regmap_field_alloc(dsi->dev, dsi->regs,	\
						 variant->conf);	\
		if (IS_ERR(dsi->f))					\
			dev_warn(dsi->dev, "Ignoring regmap field " #f "\n"); })

static int dw_mipi_dsi_regmap_fields_init(struct dw_mipi_dsi *dsi)
{
	const struct dw_mipi_dsi_variant *variant;

	switch (dsi->hw_version) {
	case HWVER_130:
	case HWVER_131:
		variant = &dw_mipi_dsi_v130_v131_layout;
		break;
	case HWVER_101:
		variant = &dw_mipi_dsi_v101_layout;
		break;
	default:
		DRM_ERROR("Unrecognized DSI host controller HW revision\n");
		return -ENODEV;
	}

	INIT_FIELD(dpi_18loosely_en);
	INIT_FIELD(dpi_vid);
	INIT_FIELD(dpi_color_coding);
	INIT_FIELD(dpi_vsync_active_low);
	INIT_FIELD(dpi_hsync_active_low);
	INIT_FIELD(cmd_mode_ack_rqst_en);
	INIT_FIELD(cmd_mode_all_lp_en);
	INIT_FIELD(cmd_mode_en);
	INIT_FIELD(cmd_pkt_status);
	INIT_FIELD(vid_mode_en);
	INIT_FIELD(vid_mode_type);
	INIT_FIELD(vid_mode_low_power);
	INIT_FIELD(vid_pkt_size);
	INIT_FIELD(vid_hsa_time);
	INIT_FIELD(vid_hbp_time);
	INIT_FIELD(vid_hline_time);
	INIT_FIELD(vid_vsa_time);
	INIT_FIELD(vid_vbp_time);
	INIT_FIELD(vid_vfp_time);
	INIT_FIELD(vid_vactive_time);
	INIT_FIELD(phy_txrequestclkhs);
	INIT_FIELD(phy_auto_clklane_ctrl);
	INIT_FIELD(phy_testclr);
	INIT_FIELD(phy_unshutdownz);
	INIT_FIELD(phy_unrstz);
	INIT_FIELD(phy_enableclk);
	INIT_FIELD(phy_nlanes);
	INIT_FIELD(phy_stop_wait_time);
	INIT_FIELD(phy_status);
	INIT_FIELD(pckhdl_cfg);
	INIT_FIELD(hstx_timeout_counter);
	INIT_FIELD(lprx_timeout_counter);
	INIT_FIELD(int_stat0);
	INIT_FIELD(int_stat1);
	INIT_FIELD(int_mask0);
	INIT_FIELD(int_mask1);
	INIT_FIELD(gen_hdr);
	INIT_FIELD(gen_payload);
	INIT_FIELD(phy_bta_time);
	INIT_FIELD(vid_mode_en_lp_cmd);
	INIT_FIELD(vid_mode_vpg_en);
	INIT_FIELD(vid_mode_vpg_mode);
	INIT_FIELD(vid_mode_vpg_horiz);
	INIT_FIELD(phy_clklp2hs_time);
	INIT_FIELD(phy_clkhs2lp_time);
	INIT_FIELD(phy_forcepll);

	if (dsi->hw_version == HWVER_131) {
		INIT_FIELD_CFG(field_phy_max_rd_time, cfg_phy_max_rd_time_v131);
		INIT_FIELD_CFG(field_phy_lp2hs_time, cfg_phy_lp2hs_time_v131);
		INIT_FIELD_CFG(field_phy_hs2lp_time, cfg_phy_hs2lp_time_v131);
	} else {
		INIT_FIELD(phy_max_rd_time);
		INIT_FIELD(phy_lp2hs_time);
		INIT_FIELD(phy_hs2lp_time);
	}

	return 0;
}

static struct dw_mipi_dsi *
__dw_mipi_dsi_probe(struct platform_device *pdev,
		    const struct dw_mipi_dsi_plat_data *plat_data)
{
	struct device *dev = &pdev->dev;
	struct reset_control *apb_rst;
	struct dw_mipi_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return ERR_PTR(-ENOMEM);

	dsi->dev = dev;
	dsi->plat_data = plat_data;

	if (!plat_data->phy_ops->init || !plat_data->phy_ops->get_lane_mbps ||
	    !plat_data->phy_ops->get_timing) {
		DRM_ERROR("Phy not properly configured\n");
		return ERR_PTR(-ENODEV);
	}

	if (!plat_data->base) {
		dsi->base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(dsi->base))
			return ERR_PTR(-ENODEV);

	} else {
		dsi->base = plat_data->base;
	}

	dsi->regs = devm_regmap_init_mmio(dev, dsi->base,
					  &dw_mipi_dsi_regmap_cfg);
	if (IS_ERR(dsi->regs)) {
		ret = PTR_ERR(dsi->regs);
		DRM_ERROR("Failed to create DW MIPI DSI regmap: %d\n", ret);
		return ERR_PTR(ret);
	}

	dsi->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(dsi->pclk)) {
		ret = PTR_ERR(dsi->pclk);
		dev_err(dev, "Unable to get pclk: %d\n", ret);
		return ERR_PTR(ret);
	}

	/*
	 * Note that the reset was not defined in the initial device tree, so
	 * we have to be prepared for it not being found.
	 */
	apb_rst = devm_reset_control_get_optional_exclusive(dev, "apb");
	if (IS_ERR(apb_rst)) {
		ret = PTR_ERR(apb_rst);

		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Unable to get reset control: %d\n", ret);

		return ERR_PTR(ret);
	}

	if (apb_rst) {
		ret = clk_prepare_enable(dsi->pclk);
		if (ret) {
			dev_err(dev, "%s: Failed to enable pclk\n", __func__);
			return ERR_PTR(ret);
		}

		reset_control_assert(apb_rst);
		usleep_range(10, 20);
		reset_control_deassert(apb_rst);

		clk_disable_unprepare(dsi->pclk);
	}

	ret = dw_mipi_dsi_get_hw_version(dsi);
	if (ret) {
		dev_err(dev, "Could not read HW version\n");
		return ERR_PTR(ret);
	}

	ret = dw_mipi_dsi_regmap_fields_init(dsi);
	if (ret) {
		dev_err(dev, "Failed to init register layout map: %d\n", ret);
		return ERR_PTR(ret);
	}

	dw_mipi_dsi_debugfs_init(dsi);
	pm_runtime_enable(dev);

	dsi->dsi_host.ops = &dw_mipi_dsi_host_ops;
	dsi->dsi_host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->dsi_host);
	if (ret) {
		dev_err(dev, "Failed to register MIPI host: %d\n", ret);
		pm_runtime_disable(dev);
		dw_mipi_dsi_debugfs_remove(dsi);
		return ERR_PTR(ret);
	}

	dsi->bridge.driver_private = dsi;
	dsi->bridge.funcs = &dw_mipi_dsi_bridge_funcs;
#ifdef CONFIG_OF
	dsi->bridge.of_node = pdev->dev.of_node;
#endif
	drm_bridge_add(&dsi->bridge);

	return dsi;
}

static void __dw_mipi_dsi_remove(struct dw_mipi_dsi *dsi)
{
	mipi_dsi_host_unregister(&dsi->dsi_host);

	pm_runtime_disable(dsi->dev);
	dw_mipi_dsi_debugfs_remove(dsi);
}

void dw_mipi_dsi_set_slave(struct dw_mipi_dsi *dsi, struct dw_mipi_dsi *slave)
{
	/* introduce controllers to each other */
	dsi->slave = slave;
	dsi->slave->master = dsi;

	/* migrate settings for already attached displays */
	dsi->slave->lanes = dsi->lanes;
	dsi->slave->channel = dsi->channel;
	dsi->slave->format = dsi->format;
	dsi->slave->mode_flags = dsi->mode_flags;
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_set_slave);

/*
 * Probe/remove API, used from platforms based on the DRM bridge API.
 */
struct dw_mipi_dsi *
dw_mipi_dsi_probe(struct platform_device *pdev,
		  const struct dw_mipi_dsi_plat_data *plat_data)
{
	return __dw_mipi_dsi_probe(pdev, plat_data);
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_probe);

void dw_mipi_dsi_remove(struct dw_mipi_dsi *dsi)
{
	__dw_mipi_dsi_remove(dsi);
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_remove);

MODULE_AUTHOR("Chris Zhong <zyw@rock-chips.com>");
MODULE_AUTHOR("Philippe Cornu <philippe.cornu@st.com>");
MODULE_DESCRIPTION("DW MIPI DSI host controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dw-mipi-dsi");
