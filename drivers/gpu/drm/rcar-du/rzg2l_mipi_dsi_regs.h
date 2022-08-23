/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ/G2L MIPI DSI Interface Registers Definitions
 *
 * Copyright (C) 2020 Renesas Electronics Corporation
 */

#ifndef __RZG2L_MIPI_DSI_REGS_H__
#define __RZG2L_MIPI_DSI_REGS_H__

/* DPHY Registers */
#define DSIDPHYCTRL0			0x00
#define DSIDPHYCTRL0_CAL_EN_HSRX_OFS	(1 << 16)
#define DSIDPHYCTRL0_CMN_MASTER_EN	(1 << 8)
#define DSIDPHYCTRL0_RE_VDD_DETVCCQLV18	(1 << 2)
#define DSIDPHYCTRL0_EN_LDO1200		(1 << 1)
#define DSIDPHYCTRL0_EN_BGR		(1 << 0)

#define DSIDPHYTIM0			0x04
#define DSIDPHYTIM0_TCLK_MISS(x)	((x) << 24)
#define DSIDPHYTIM0_T_INIT(x)		((x) << 0)

#define DSIDPHYTIM1			0x08
#define DSIDPHYTIM1_THS_PREPARE(x)	((x) << 24)
#define DSIDPHYTIM1_TCLK_PREPARE(x)	((x) << 16)
#define DSIDPHYTIM1_THS_SETTLE(x)	((x) << 8)
#define DSIDPHYTIM1_TCLK_SETTLE(x)	((x) << 0)

#define DSIDPHYTIM2			0x0C
#define DSIDPHYTIM2_TCLK_TRAIL(x)	((x) << 24)
#define DSIDPHYTIM2_TCLK_POST(x)	((x) << 16)
#define DSIDPHYTIM2_TCLK_PRE(x)		((x) << 8)
#define DSIDPHYTIM2_TCLK_ZERO(x)	((x) << 0)

#define DSIDPHYTIM3			0x10
#define DSIDPHYTIM3_TLPX(x)		((x) << 24)
#define DSIDPHYTIM3_THS_EXIT(x)		((x) << 16)
#define DSIDPHYTIM3_THS_TRAIL(x)	((x) << 8)
#define DSIDPHYTIM3_THS_ZERO(x)		((x) << 0)

#define DSIDPHYCTRL1			0x40
#define DSIDPHYCTRL1_TRIM_REGSEL	(1 << 0)

#define DSIDPHYTRIM0			0x44

/* --------------------------------------------------------*/
/* Link Registers */
/* Link Status Register */
#define LINKSR				0x10
#define LINKSR_LPBUSY			(1 << 13)
#define LINKSR_HSBUSY			(1 << 12)
#define LINKSR_VICHRUN1			(1 << 8)
#define LINKSR_SQCHRUN1			(1 << 4)
#define LINKSR_SQCHRUN0			(1 << 0)

/* Tx Set Register */
#define TXSETR				0x100
#define TXSETR_NUMLANECAP		(0x3 << 16)
#define TXSETR_DLEN			(1 << 9)
#define TXSETR_CLEN			(1 << 8)
#define TXSETR_NUMLANEUSE(x)		(((x) & 0x3) << 0)

/* HS Clock Set Register */
#define HSCLKSETR			0x104
#define HSCLKSETR_HSCLKMODE_CONT	(1 << 1)
#define HSCLKSETR_HSCLKMODE_NON_CONT	(0 << 1)
#define HSCLKSETR_HSCLKRUN_HS		(1 << 0)
#define HSCLKSETR_HSCLKRUN_LP		(0 << 0)

/* Reset Control Register */
#define RSTCR				0x110
#define RSTCR_SWRST			(1 << 0)
#define RSTCR_FCETXSTP			(1 << 16)

/* Reset Status Register */
#define RSTSR				0x114
#define RSTSR_DL0DIR			(1 << 15)
#define RSTSR_DLSTPST			(0xf << 8)
#define RSTSR_SWRSTV1			(1 << 4)
#define RSTSR_SWRSTIB			(1 << 3)
#define RSTSR_SWRSTAPB			(1 << 2)
#define RSTSR_SWRSTLP			(1 << 1)
#define RSTSR_SWRSTHS			(1 << 0)

/* Tx Packet Payload Data x Register */

#define TXPPDxR(x)			(0x160 + (x) * 4)

/* Rx Result Save Slot 0 Register */
#define RXRSS0R				0x240
#define RXRSS0R_RXSUC			BIT(25)
#define RXRSS0R_FMT			BIT(24)
#define RXRSS0R_VC			GENMASK(23, 22)
#define RXRSS0R_DT			GENMASK(21, 16)
#define RXRSS0R_DATA1			GENMASK(15, 8)
#define RXRSS0R_DATA0			GENMASK(7, 0)
#define RXRSS0R_DATA			(RXRSS0R_DATA1 | RXRSS0R_DATA0)

/* Rx Packet Payload Data x Register */
#define RXPPDxR(x)			(0x2C0 + (x) * 4)

/* Clock Lane Stop Time Set Register */
#define CLSTPTSETR			0x314
#define CLSTPTSETR_CLKKPT(x)		((x) << 24)
#define CLSTPTSETR_CLKBFHT(x)		((x) << 16)
#define CLSTPTSETR_CLKSTPT(x)		((x) << 2)

/* LP Transition Time Set Register */
#define LPTRNSTSETR			0x318
#define LPTRNSTSETR_GOLPBKT(x)		((x) << 0)

/* Physical Lane Status Register */
#define PLSR				0x320
#define PLSR_CLHS2LP			(1 << 27)
#define PLSR_CLLP2HS			(1 << 26)

/* Video-Input Channel 1 Set 0 Register */
#define VICH1SET0R			0x400
#define VICH1SET0R_VSEN			(1 << 12)
#define VICH1SET0R_HFPNOLP		(1 << 10)
#define VICH1SET0R_HBPNOLP		(1 << 9)
#define VICH1SET0R_HSANOLP		(1 << 8)
#define VICH1SET0R_VSTPAFT		(1 << 1)
#define VICH1SET0R_VSTART		(1 << 0)

/* Video-Input Channel 1 Set 1 Register */
#define VICH1SET1R			0x404
#define VICH1SET1R_DLY(x)		(((x) & 0xfff) << 2)

/* Video-Input Channel 1 Status Register */
#define VICH1SR				0x410
#define VICH1SR_VIRDY			(1 << 3)
#define VICH1SR_RUNNING			(1 << 2)
#define VICH1SR_STOP			(1 << 1)
#define VICH1SR_START			(1 << 0)

/* Video-Input Channel 1 Pixel Packet Set Register */
#define VICH1PPSETR			0x420
#define VICH1PPSETR_DT_RGB16		(0x0E << 16)
#define VICH1PPSETR_DT_RGB18		(0x1E << 16)
#define VICH1PPSETR_DT_RGB18_LS		(0x2E << 16)
#define VICH1PPSETR_DT_RGB24		(0x3E << 16)
#define VICH1PPSETR_DT_YCbCr16		(0x2C << 16)
#define VICH1PPSETR_DT_YCbCr20_LS	(0x0C << 16)
#define VICH1PPSETR_DT_YCbCr24		(0x1C << 16)
#define VICH1PPSETR_TXESYNC_PULSE	(1 << 15)
#define VICH1PPSETR_VC(x)		((x) << 22)

/* Video-Input Channel 1 Vertical Size Set Register */
#define VICH1VSSETR			0x428
#define VICH1VSSETR_VACTIVE(x)		(((x) & 0x7fff) << 16)
#define VICH1VSSETR_VSPOL_LOW		(1 << 15)
#define VICH1VSSETR_VSPOL_HIGH		(0 << 15)
#define VICH1VSSETR_VSA(x)		(((x) & 0xfff) << 0)

/* Video-Input Channel 1 Vertical Porch Set Register */
#define VICH1VPSETR			0x42C
#define VICH1VPSETR_VFP(x)		(((x) & 0x1fff) << 16)
#define VICH1VPSETR_VBP(x)		(((x) & 0x1fff) << 0)

/* Video-Input Channel 1 Horizontal Size Set Register */
#define VICH1HSSETR			0x430
#define VICH1HSSETR_HACTIVE(x)		(((x) & 0x7fff) << 16)
#define VICH1HSSETR_HSPOL_LOW		(1 << 15)
#define VICH1HSSETR_HSPOL_HIGH		(0 << 15)
#define VICH1HSSETR_HSA(x)		(((x) & 0xfff) << 0)

/* Video-Input Channel 1 Horizontal Porch Set Register */
#define VICH1HPSETR			0x434
#define VICH1HPSETR_HFP(x)		(((x) & 0x1fff) << 16)
#define VICH1HPSETR_HBP(x)		(((x) & 0x1fff) << 0)

/* Sequence Channel 0 Set 0 Register */
#define SQCH0SET0R			0x5C0
#define SQCH0SET0R_START		BIT(0)

/* Sequence Channel 0 Set 1 Register */
#define SQCH0SET1R			0x5C4

/* Sequence Channel 0 Status Register */
#define SQCH0SR				0x5D0
#define SQCH0SR_RUNNING			BIT(2)
#define SQCH0SR_ADESFIN			BIT(8)

/* Sequence Channel 0 Status Clear Register */
#define SQCH0SCR			0x5D4
#define SQCH0SCR_ADESFIN		BIT(8)

/* Sequence Channel 0 Descriptor 00-A Register */
#define SQCH0DSC00AR			0x780
#define SQCH0DSC00AR_NXACT_TERM		(0 << 28)
#define SQCH0DSC00AR_NXACT_OPER		(1 << 28)
#define SQCH0DSC00AR_BTA_NO_BTA		(0 << 26)
#define SQCH0DSC00AR_BTA_NON_READ_BTA	(1 << 26)
#define SQCH0DSC00AR_BTA_READ_BTA	(2 << 26)
#define SQCH0DSC00AR_BTA_ONLY_BTA	(3 << 26)
#define SQCH0DSC00AR_SPD_HIGH		(0 << 25)
#define SQCH0DSC00AR_SPD_LOW		(1 << 25)
#define SQCH0DSC00AR_FMT_SHORT		(0 << 24)
#define SQCH0DSC00AR_FMT_LONG		(1 << 24)
#define SQCH0DSC00AR_VC_DT(x)		((x) << 16)
#define SQCH0DSC00AR_DATA0(x)		(x)
#define SQCH0DSC00AR_DATA1(x)		((x) << 8)

/* Sequence Channel 0 Descriptor 00-B Register */
#define SQCH0DSC00BR			0x784
#define SQCH0DSC00BR_DTSEL_PAYLOAD_SIZE	(0 << 24)
#define SQCH0DSC00BR_DTSEL_MEM_SPACE	(1 << 24)

/* Sequence Channel 0 Descriptor 00-C Register */
#define SQCH0DSC00CR			0x788
#define SQCH0DSC00CR_FINACT		BIT(0)

/* Sequence Channel 0 Descriptor 00-D Register */
#define SQCH0DSC00DR			0x78C

#endif /* __RZG2L_MIPI_DSI_REGS_H__ */
