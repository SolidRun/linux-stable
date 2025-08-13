/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * rzg2l-cru-regs.h--RZ/G2L (and alike SoCs) CRU Registers Definitions
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 */

#ifndef __RZG2L_CRU_REGS_H__
#define __RZG2L_CRU_REGS_H__

/* HW CRU Registers Definition */

#define CRUnCTRL_VINSEL(x)		((x) << 0)

#define CRUnIE_EFE			BIT(17)

#define CRUnIE2_FSxE(x)			BIT(((x) * 3))
#define CRUnIE2_FExE(x)			BIT(((x) * 3) + 1)

#define CRUnINTS_SFS			BIT(16)

#define CRUnINTS2_FSxS(x)		BIT(((x) * 3))

#define CRUnRST_VRESETN			BIT(0)

/* Memory Bank Base Address (Lower) Register for CRU Image Data */
#define AMnMBxADDRL(base, x)			((base) + (x) * 2)

/* Memory Bank Base Address (Higher) Register for CRU Image Data */
#define AMnMBxADDRH(base, x)			((base) + (x) * 2)

#define AMnMBVALID_MBVALID(x)		GENMASK(x, 0)

#define AMnMBS_MBSTS			0x7

#define AMnAXIATTR_AXILEN_MASK		GENMASK(3, 0)
#define AMnAXIATTR_AXILEN		(0xf)

#define AMnFIFOPNTR_FIFOWPNTR		GENMASK(7, 0)
#define AMnFIFOPNTR_FIFOWPNTR_B0	AMnFIFOPNTR_FIFOWPNTR
#define AMnFIFOPNTR_FIFOWPNTR_B1	GENMASK(15, 8)
#define AMnFIFOPNTR_FIFORPNTR_Y		GENMASK(23, 16)
#define AMnFIFOPNTR_FIFORPNTR_B0	AMnFIFOPNTR_FIFORPNTR_Y
#define AMnFIFOPNTR_FIFORPNTR_UV	GENMASK(31, 24)
#define AMnFIFOPNTR_FIFORPNTR_B1	AMnFIFOPNTR_FIFORPNTR_UV

#define AMnIS_IS_MASK			GENMASK(14, 7)
#define AMnIS_IS(x)			((x) << 7)

#define AMnAXISTP_AXI_STOP		BIT(0)

#define AMnAXISTPACK_AXI_STOP_ACK	BIT(0)

/* Memory Bank Base Address (Lower) Register for CRU Statistics Data */
#define AMnSDMBxADDRL(base, x)		((base) + (x) * 2)

/* Memory Bank Base Address (Higher) Register for CRU Statistics Data */
#define AMnSDMBxADDRH(base, x)		((base) + (x) * 2)

/* Memory Bank Enable Register for CRU Image Data */
#define AMnSDMBVALID_SDMBVALID(x)	GENMASK(x, 0)

/* Memory Bank Status Register for CRU Image Data */
#define AMnSDMBS_SDMBSTS		0x7

/* AXI Master Transfer Constant Register for CRU Statistics data */
#define AMnSDAXIATTR_SDAXILEN(x)	(x)

/* AXI Master FIFO Pointer Register for CRU Statistics Data */
#define AMnSDFIFOPNTR_SDFIFOWPNTR	GENMASK(4, 0)
#define AMnSDFIFOPNTR_SDFIFORPNTR	GENMASK(20, 16)

/* AXI Master Transfer Stop Register for CRU Image Data */
#define AMnSDAXISTP_SDAXI_STOP		BIT(0)

/* AXI Master Transfer Stop Status Register for CRU Image Data */
#define AMnSDAXISTPACK_SDAXI_STOP_ACK	BIT(0)

#define ICnEN_ICEN			BIT(0)

#define ICnSVC_SVC0(x)			(x)
#define ICnSVC_SVC1(x)			((x) << 4)
#define ICnSVC_SVC2(x)			((x) << 8)
#define ICnSVC_SVC3(x)			((x) << 12)

/* CRU Image Processing Register Setting Change Control Register */
#define ICnREGC_REFEN			BIT(0)

#define ICnMC_DEMTHR			BIT(3)
#define ICnMC_LMXTHR			BIT(4)
#define ICnMC_CSCTHR			BIT(5)
#define ICnMC_STITHR			BIT(7)
#define ICnMC_INF(x)			((x) << 16)
#define ICnMC_VCSEL(x)			((x) << 22)
#define ICnMC_INF_MASK			GENMASK(21, 16)

#define ICnMC_RAWSTTYP_RGRG		0
#define ICnMC_RAWSTTYP_GRGR		BIT(24)
#define ICnMC_RAWSTTYP_GBGB		BIT(25)
#define ICnMC_RAWSTTYP_BGBG		(BIT(25) | BIT(24))
#define ICnMC_RAWSTTYP_MASK		(BIT(25) | BIT(24))

#define ICnMS_IA			BIT(2)

#define ICnDMR_RGBMODE_RGB24		(0 << 0)
#define ICnDMR_RGBMODE_XRGB32		(1 << 0)
#define ICnDMR_RGBMODE_ABGR32		(2 << 0)
#define ICnDMR_RGBMODE_ARGB32		(3 << 0)
#define ICnDMR_YCMODE_YUYV		(0 << 4)
#define ICnDMR_YCMODE_UYVY		(1 << 4)
#define ICnDMR_YCMODE_NV16		(2 << 4)
#define ICnDMR_YCMODE_GREY		(3 << 4)

/* CRU Linear Matrix Offset register */
#define ICnLMXOF_ROF(x)			(((x) & GENMASK(7, 0)) << 0)
#define ICnLMXOF_GOF(x)			(((x) & GENMASK(7, 0)) << 8)
#define ICnLMXOF_BOF(x)			(((x) & GENMASK(7, 0)) << 16)

/* CRU Linear Matrix R Coefficient 1 Register */
#define ICnLMXRC1_RR(x)			(((x) & GENMASK(12, 0)) << 0)

/* CRU Linear Matrix R Coefficient 2 Register */
#define ICnLMXRC2_RG(x)			(((x) & GENMASK(12, 0)) << 0)
#define ICnLMXRC2_RB(x)			(((x) & GENMASK(12, 0)) << 16)

/* CRU Linear Matrix G Coefficient 1 Register */
#define ICnLMXGC1_GR(x)			(((x) & GENMASK(12, 0)) << 0)

/* CRU Linear Matrix G Coefficient 2 Register */
#define ICnLMXGC2_GG(x)			(((x) & GENMASK(12, 0)) << 0)
#define ICnLMXGC2_GB(x)			(((x) & GENMASK(12, 0)) << 16)

/* CRU Linear Matrix B Coefficient 1 Register */
#define ICnLMXBC1_BR(x)			(((x) & GENMASK(12, 0)) << 0)

/* CRU Linear Matrix B Coefficient 2 Register */
#define ICnLMXBC2_BG(x)			(((x) & GENMASK(12, 0)) << 0)
#define ICnLMXBC2_BB(x)			(((x) & GENMASK(12, 0)) << 16)

/* CRU Statistics Control 1 Register */
#define ICnSTIC1_STUNIT_MASK		0x3
#define ICnSTIC1_STUNIT(x)		(x)
#define ICnSTIC1_STSADPOS(x)		((x) << 16)

/* CRU Statistics Control 2 Register */
#define ICnSTIC2_STHPOS(x)		(x)

enum rzg2l_cru_common_regs {
	CRUnCTRL,	/* CRU Control */
	CRUnIE,		/* CRU Interrupt Enable */
	CRUnIE2,	/* CRU Interrupt Enable(2) */
	CRUnINTS,	/* CRU Interrupt Status */
	CRUnINTS2,	/* CRU Interrupt Status(2) */
	CRUnRST,	/* CRU Reset */
	AMnMB1ADDRL,	/* Bank 1 Address (Lower) for CRU Image Data */
	AMnMB1ADDRH,	/* Bank 1 Address (Higher) for CRU Image Data */
	AMnMB2ADDRL,    /* Bank 2 Address (Lower) for CRU Image Data */
	AMnMB2ADDRH,    /* Bank 2 Address (Higher) for CRU Image Data */
	AMnMB3ADDRL,    /* Bank 3 Address (Lower) for CRU Image Data */
	AMnMB3ADDRH,    /* Bank 3 Address (Higher) for CRU Image Data */
	AMnMB4ADDRL,    /* Bank 4 Address (Lower) for CRU Image Data */
	AMnMB4ADDRH,    /* Bank 4 Address (Higher) for CRU Image Data */
	AMnMB5ADDRL,    /* Bank 5 Address (Lower) for CRU Image Data */
	AMnMB5ADDRH,    /* Bank 5 Address (Higher) for CRU Image Data */
	AMnMB6ADDRL,    /* Bank 6 Address (Lower) for CRU Image Data */
	AMnMB6ADDRH,    /* Bank 6 Address (Higher) for CRU Image Data */
	AMnMB7ADDRL,    /* Bank 7 Address (Lower) for CRU Image Data */
	AMnMB7ADDRH,    /* Bank 7 Address (Higher) for CRU Image Data */
	AMnMB8ADDRL,    /* Bank 8 Address (Lower) for CRU Image Data */
	AMnMB8ADDRH,    /* Bank 8 Address (Higher) for CRU Image Data */
	AMnUVAOFL,	/* UV Data Address Offset (Lower) Register for CRU Image Data*/
	AMnUVAOFH,	/* UV Data Address Offset (Higher) Register for CRU Image Data*/
	AMnMBVALID,	/* Memory Bank Enable for CRU Image Data */
	AMnMBS,		/* Memory Bank Status for CRU Image Data */
	AMnMADRSL,	/* VD Memory Address Lower Status Register */
	AMnMADRSH,	/* VD Memory Address Higher Status Register */
	AMnAXIATTR,	/* AXI Master Transfer Setting Register for CRU Image Data */
	AMnFIFO,	/* AXI Master FIFO Setting Register */
	AMnFIFOPNTR,	/* AXI Master FIFO Pointer for CRU Image Data */
	AMnAXISTP,	/* AXI Master Transfer Stop for CRU Image Data */
	AMnAXISTPACK,	/* AXI Master Transfer Stop Status for CRU Image Data */
	AMnIS,		/* Image Stride Setting Register */
	AMnSDMB1ADDRL,	/* Memory Bank 1 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB1ADDRH,	/* Memory Bank 1 Base Address Higher Register for CRU Statistics Data */
	AMnSDMB2ADDRL,	/* Memory Bank 2 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB2ADDRH,	/* Memory Bank 2 Base Address Higher Register for CRU Statistics Data */
	AMnSDMB3ADDRL,	/* Memory Bank 3 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB3ADDRH,	/* Memory Bank 3 Base Address Higher Register for CRU Statistics Data */
	AMnSDMB4ADDRL,	/* Memory Bank 4 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB4ADDRH,	/* Memory Bank 4 Base Address Higher Register for CRU Statistics Data */
	AMnSDMB5ADDRL,	/* Memory Bank 5 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB5ADDRH,	/* Memory Bank 5 Base Address Higher Register for CRU Statistics Data */
	AMnSDMB6ADDRL,	/* Memory Bank 6 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB6ADDRH,	/* Memory Bank 6 Base Address Higher Register for CRU Statistics Data */
	AMnSDMB7ADDRL,	/* Memory Bank 7 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB7ADDRH,	/* Memory Bank 7 Base Address Higher Register for CRU Statistics Data */
	AMnSDMB8ADDRL,	/* Memory Bank 8 Base Address Lower Register for CRU Statistics Data */
	AMnSDMB8ADDRH,	/* Memory Bank 8 Base Address Higher Register for CRU Statistics Data */
	AMnSDMBVALID,	/* Memory Bank Enable Register for CRU Image Data */
	AMnSDMBS,	/* Memory Bank Status Register for CRU Image Data */
	AMnSDAXIATTR,	/* AXI Master Transfer Constant Register for CRU Statistics data */
	AMnSDFIFOPNTR,	/* AXI Master FIFO Pointer Register for CRU Statistics Data */
	AMnSDAXISTP,	/* AXI Master Transfer Stop Register for CRU Image Data */
	AMnSDAXISTPACK,	/* AXI Master Transfer Stop Status Register for CRU Image Data */
	ICnEN,		/* CRU Image Processing Enable */
	ICnREGC,	/* CRU Image Processing Register Setting Change Control Register */
	ICnSVCNUM,	/* CRU SVC Number Register */
	ICnSVC,		/* CRU VC Select Register */
	ICnMC,		/* CRU Image Processing Main Control */
	ICnLMXOF,	/* CRU Linear Matrix Offset register */
	ICnLMXRC1,	/* CRU Linear Matrix R Coefficient 1 Register */
	ICnLMXRC2,	/* CRU Linear Matrix R Coefficient 2 Register */
	ICnLMXGC1,	/* CRU Linear Matrix G Coefficient 1 Register */
	ICnLMXGC2,	/* CRU Linear Matrix G Coefficient 2 Register */
	ICnLMXBC1,	/* CRU Linear Matrix B Coefficient 1 Register */
	ICnLMXBC2,	/* CRU Linear Matrix B Coefficient 2 Register */
	ICnSTIC1,	/* CRU Statistics Control 1 Register */
	ICnSTIC2,	/* CRU Statistics Control 2 Register */
	ICnIPMC_C0,	/* CRU Image Converter Main Control 0 */
	ICnMS,		/* CRU Module Status */
	ICnDMR,		/* CRU Data Output Mode */
	RZG2L_CRU_MAX_REG,
};

#endif /* __RZG2L_CRU_REGS_H__ */
