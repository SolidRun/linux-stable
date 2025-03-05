/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * rzg2l-cru-regs.h--RZ/G2L (and alike SoCs) CRU Registers Definitions
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 */

#ifndef __RZG2L_CRU_REGS_H__
#define __RZG2L_CRU_REGS_H__

/* HW CRU Registers Definition */

/* CRU Control Register */
#define CRUnCTRL_VINSEL(x)		((x) << 0)

/* CRU Interrupt Enable Register */
#define CRUnIE_EFE			BIT(17)
#define CRUnIE2_FEE(x)			BIT(((x) * 3) + 1)

/* CRU Interrupt Status Register */
#define CRUnINTS_SFS			BIT(16)
#define CRUnINTS2_FSS(x)		BIT((x) * 3)

/* CRU Reset Register */
#define CRUnRST_VRESETN			BIT(0)

/* Memory Bank Base Address (Lower) Register for CRU Image Data */
#define AMnMBxADDRL(base, x)		((base) + (x) * 2)

/* Memory Bank Base Address (Higher) Register for CRU Image Data */
#define AMnMBxADDRH(base, x)		((base) + (x) * 2)

/* Memory Bank Enable Register for CRU Image Data */
#define AMnMBVALID_MBVALID(x)		GENMASK(x, 0)

/* Memory Bank Status Register for CRU Image Data */
#define AMnMBS_MBSTS			0x7

/* AXI-VD Bus Master Transfer Setting Register */
#define AMnAXIATTR_AXILEN_MASK		GENMASK(3, 0)
#define AMnAXIATTR_AXILEN		(0xf)

/* AXI Master FIFO Pointer Register for CRU Image Data */
#define AMnFIFOPNTR_FIFOWPNTR		GENMASK(7, 0)
#define AMnFIFOPNTR_FIFOWPNTR_B0	AMnFIFOPNTR_FIFOWPNTR
#define AMnFIFOPNTR_FIFOWPNTR_B1	GENMASK(15, 8)
#define AMnFIFOPNTR_FIFORPNTR_Y		GENMASK(23, 16)
#define AMnFIFOPNTR_FIFORPNTR_B0	AMnFIFOPNTR_FIFORPNTR_Y
#define AMnFIFOPNTR_FIFORPNTR_UV	GENMASK(31, 24)
#define AMnFIFOPNTR_FIFORPNTR_B1	AMnFIFOPNTR_FIFORPNTR_UV

#define AMnIS_IS(x)			((x) << 7)

/* AXI Master Transfer Stop Register for CRU Image Data */
#define AMnAXISTP_AXI_STOP		BIT(0)

/* AXI Master Transfer Stop Status Register for CRU Image Data */
#define AMnAXISTPACK_AXI_STOP_ACK	BIT(0)

/* CRU Image Processing Enable Register */
#define ICnEN_ICEN			BIT(0)

#define ICnSVC_SVC0(x)			(x)
#define ICnSVC_SVC1(x)			((x) << 4)
#define ICnSVC_SVC2(x)			((x) << 8)
#define ICnSVC_SVC3(x)			((x) << 12)

/* CRU Image Processing Main Control Register */
#define ICnMC_DEMTHR			BIT(3)
#define ICnMC_CSCTHR			BIT(5)
#define ICnMC_INF(x)			((x) << 16)
#define ICnMC_VCSEL(x)			((x) << 22)
#define ICnMC_INF_MASK			GENMASK(21, 16)

#define ICnMC_RAWSTTYP_RGRG		0
#define ICnMC_RAWSTTYP_GRGR		BIT(24)
#define ICnMC_RAWSTTYP_GBGB		BIT(25)
#define ICnMC_RAWSTTYP_BGBG		(BIT(25) | BIT(24))
#define ICnMC_RAWSTTYP_MASK		(BIT(25) | BIT(24))

/* CRU Module Status Register */
#define ICnMS_IA			BIT(2)

/* CRU Test Image Generation Control 1 Register */
#define ICnTICTRL1_TIEN			BIT(0)
#define ICnTICTRL1_TIMODE		BIT(1)
#define ICnTICTRL1_TIPTNY1(x)		((x) << 4)
#define ICnTICTRL1_TIPTNU1(x)		((x) << 8)
#define ICnTICTRL1_TIPTNV1(x)		((x) << 12)

/* CRU Test Image Generation Control 2 Register */
#define ICnTICTRL2_TIPTNY2(x)		((x) << 0)
#define ICnTICTRL2_TIPTNU2(x)		((x) << 8)
#define ICnTICTRL2_TIPTNV2(x)		((x) << 16)

/* CRU Test Image Size Setting 1 Register */
#define ICnTISIZE1_TIPPL(x)		((x) << 0)

/* CRU Test Image Size Setting 2 Register */
#define ICnTISIZE2_TIN(x)		((x) << 0)
#define ICnTISIZE2_TIM(x)		((x) << 16)

/* CRU Data Output Mode Register */
#define ICnDMR_RGBMODE_RGB24		(0 << 0)
#define ICnDMR_RGBMODE_XRGB32		(1 << 0)
#define ICnDMR_RGBMODE_ABGR32		(2 << 0)
#define ICnDMR_RGBMODE_ARGB32		(3 << 0)
#define ICnDMR_YCMODE_YUYV		(0 << 4)
#define ICnDMR_YCMODE_UYVY		(1 << 4)
#define ICnDMR_YCMODE_NV16		(2 << 4)
#define ICnDMR_YCMODE_GREY		(3 << 4)

#endif /* __RZG2L_CRU_REGS_H__ */
