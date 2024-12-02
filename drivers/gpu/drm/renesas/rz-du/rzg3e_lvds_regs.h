/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ/G3E LVDS Interface Registers Definitions
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 *
 */

#ifndef __RZG3S_LVDS_REGS_H__
#define __RZG3S_LVDS_REGS_H__

#define LVDS_CMN			0x0000
#define LVDS_CMN_RST_PHY1_SEL		(1 << 25)
#define LVDS_CMN_RST_PHY1_SEL_CH0	(0 << 25)
#define LVDS_CMN_RST_PHY1_SEL_CH1	(1 << 25)
#define LVDS_CMN_RST_PHY0_SEL		(1 << 24)
#define LVDS_CMN_RST_PHY0_SEL_LOW	(0 << 24)
#define LVDS_CMN_RST_PHY0_SEL_CH0	(1 << 24)
#define LVDS_CMN_CLK_SEL		(1 << 16)
#define LVDS_CMN_CLK_SEL_CLK_CH0	(0 << 16)
#define LVDS_CMN_CLK_SEL_CLK_CH1	(1 << 16)
#define LVDS_CMN_ST_SWAP		(1 << 9)
#define LVDS_CMN_ST_SWAP_0EVEN_1ODD	(0 << 9)
#define LVDS_CMN_ST_SWAP_0ODD_1EVEN	(1 << 9)
#define LVDS_CMN_ST_ON			(1 << 8)
#define LVDS_CMN_ST_ON_SINGLE		(0 << 8)
#define LVDS_CMN_ST_ON_DUAL		(1 << 8)
#define LVDS_CMN_PHY_RESET		(1 << 0)
#define LVDS_CMN_PHY_RESET_RST		(0 << 0)
#define LVDS_CMN_PHY_RESET_N		(1 << 0)

#define LVDS_PHY_OFFSET		(0x0)

#define LVDS_PHY_CH_SKW_ADJ_MSK        (0x7FFF)
#define LVDS_PHY_CH_SKW_ADJ_SHT        (16)
#define LVDS_PHY_CH_SKW_ADJ            (LVDS_PHY_CH_SKW_ADJ_MSK << LVDS_PHY_CH_SKW_ADJ_SHT)
#define LVDS_PHY_CH_IO_EN_MSK          (0x1F)
#define LVDS_PHY_CH_IO_EN_SHT          (0)
#define LVDS_PHY_CH_IO_EN              (LVDS_PHY_CH_IO_EN_MSK << LVDS_PHY_CH_IO_EN_SHT)

/* bit_name : CH_SKW_ADJ, CH_IO_EN */
#define LVDS_PHY_GET(bit_name, m)	\
	(((m) >> LVDS_PHY_ ## bit_name ## _SHT) & LVDS_PHY_ ## bit_name ## _MSK)
#define LVDS_PHY_SET(bit_name, r, m)   \
	(((r) & (LVDS_PHY_ ## bit_name ## _MSK << LVDS_PHY_ ## bit_name ## _SHT))\
	 | (((m) & LVDS_PHY_ ## bit_name ## _MSK) << LVDS_PHY_ ## bit_name ## _SHT))

#define LVDS_CTL_OFFSET		(0x4)

#define LVDS_CTL_CTL_SEL_MSK       (0xFF)
#define LVDS_CTL_CTL_SEL_SHT       (16)
#define LVDS_CTL_CTL_SEL           (LVDS_CTL_CTL_SEL_MSK << LVDS_CTL_CTL_SEL_SHT)
#define LVDS_CTL_FMT_SEL_MSK       (0x0F)
#define LVDS_CTL_FMT_SEL_SHT       (8)
#define LVDS_CTL_FMT_SEL           (LVDS_CTL_FMT_SEL_MSK << LVDS_CTL_FMT_SEL_SHT)
#define LVDS_CTL_CH_SEL_MSK        (0xFF)
#define LVDS_CTL_CH_SEL_SHT        (0)
#define LVDS_CTL_CH_SEL            (LVDS_CTL_CH_SEL_MSK << LVDS_CTL_CH_SEL_SHT)

/* bit_name : CTL_SEL, FMT_SEL, CH_SEL */
#define LVDS_CTL_GET(bit_name, m)	\
	(((m) >> LVDS_CTL_ ## bit_name ## _SHT) & LVDS_CTL_ ## bit_name ## _MSK)
#define LVDS_CTL_SET(bit_name, r, m)   \
	(((r) & (LVDS_CTL_ ## bit_name ## _MSK << LVDS_CTL_ ## bit_name ## _SHT))\
	 | (((m) & LVDS_CTL_ ## bit_name ## _MSK) << LVDS_CTL_ ## bit_name ## _SHT))

#define LVDS_UNUSED			(0)
#define LVDS_SINGLE_LINK_CH0		(LVDS_CMN_RST_PHY1_SEL_CH0 | \
					 LVDS_CMN_RST_PHY0_SEL_CH0 | \
					 LVDS_CMN_CLK_SEL_CLK_CH1 | \
					 LVDS_CMN_ST_ON_SINGLE)
#define LVDS_SINGLE_LINK_CH1		(LVDS_CMN_RST_PHY1_SEL_CH1 | \
					 LVDS_CMN_RST_PHY0_SEL_LOW | \
					 LVDS_CMN_CLK_SEL_CLK_CH1 | \
					 LVDS_CMN_ST_ON_SINGLE)
#define LVDS_SINGLE_LINK_DUAL		(LVDS_CMN_RST_PHY1_SEL_CH1 | \
					 LVDS_CMN_RST_PHY0_SEL_CH0 | \
					 LVDS_CMN_CLK_SEL_CLK_CH1 | \
					 LVDS_CMN_ST_ON_SINGLE)
#define LVDS_SINGLE_LINK_MULTI		(LVDS_CMN_RST_PHY1_SEL_CH0 | \
					 LVDS_CMN_RST_PHY0_SEL_CH0 | \
					 LVDS_CMN_CLK_SEL_CLK_CH0 | \
					 LVDS_CMN_ST_ON_SINGLE)
#define LVDS_DUAL_LINK			(LVDS_CMN_RST_PHY1_SEL_CH0 | \
					 LVDS_CMN_RST_PHY0_SEL_CH0 | \
					 LVDS_CMN_CLK_SEL_CLK_CH1 | \
					 LVDS_CMN_ST_ON_DUAL)

bool rzg3e_lvds_link_dual_link(struct drm_bridge *bridge);
bool rzg3e_lvds_link_is_connected(struct drm_bridge *bridge);
int rzg3e_lvds_reset_deassert_pclk_enable(struct device *dev);
void rzg3e_lvds_reset_assert_pclk_disable(struct device *dev);
void rzg3e_lvds_write(struct device *dev, u32 reg, u32 data);
void rzg3e_lvds_set_lvds_ch_dot_clk(struct device *dev, u32 id, struct clk *clk);
u32 rzg3e_lvds_single_link_mode(struct device *dev, u32 id);

#endif /* __RZG3S_LVDS_REGS_H__ */
