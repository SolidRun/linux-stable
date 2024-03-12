/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe driver for Renesas RZ/V2H Series SoCs
 *  Copyright (C) 2022 Renesas Electronics Ltd
 *
 */

#ifndef _PCIE_RZV2H_H
#define _PCIE_RZV2H_H

#define RZ_SIP_SVC_GET_SYSPCIE 0x82000020
#define RZ_SIP_SVC_SET_SYSPCIE 0x82000021

#define PCIE_MAX_CHANNEL	2

/* PCI Express to AXI Access */
#define AXI_WINDOW_BASEL_REG(x)					(0x1000 + ((x) * 0x20))
	#define AXI_WINDOW_ENABLE					(0x00000001)
#define AXI_WINDOW_BASEU_REG(x)					(0x1004 + ((x) * 0x20))
#define AXI_WINDOW_MASKL_REG(x)					(0x1008 + ((x) * 0x20))
#define AXI_WINDOW_MASKU_REG(x)					(0x100C + ((x) * 0x20))
#define AXI_DESTINATIONL_REG(x)					(0x1010 + ((x) * 0x20))
#define AXI_DESTINATIONU_REG(x)					(0x1014 + ((x) * 0x20))

/* AXI to PCI Express Access */
#define PCIE_WINDOW_BASEL_REG(x)					(0x1100 + ((x) * 0x20))
	#define PCIE_WINDOW_ENABLE					(0x00000001)
#define PCIE_WINDOW_BASEU_REG(x)					(0x1104 + ((x) * 0x20))
#define PCIE_WINDOW_MASKL_REG(x)					(0x1108 + ((x) * 0x20))
#define PCIE_WINDOW_MASKU_REG(x)					(0x110C + ((x) * 0x20))
#define PCIE_DESTINATION_LO_REG(x)				(0x1110 + ((x) * 0x20))
#define PCIE_DESTINATION_HI_REG(x)				(0x1114 + ((x) * 0x20))

/* Request Issuing */
#define REQUEST_DATA_REG(x)						(0x0080 + ((x) * 0x04))
#define REQUEST_RCV_DATA_REG					0x008C
#define REQUEST_ADDR_1_REG						0x0090
#define REQUEST_ADDR_2_REG						0x0094
#define REQUEST_BYTE_ENABLE_REG					0x0098
#define REQUEST_ISSUE_REG						0x009C
	#define MOR_STATUS							0x00070000
	#define TR_TYPE_CFREAD_TP0					0x00000400
	#define TR_TYPE_CFWRITE_TP0					0x00000500
	#define TR_TYPE_CFREAD_TP1					0x00000600
	#define TR_TYPE_CFWRITE_TP1					0x00000700
	#define REQ_ISSUE							0x00000001


/* Interruption */
#define MSI_RCV_WINDOW_ADDRL_REG				0x0100
		#define MSI_RCV_WINDOW_ENABLE			0x00000001
#define MSI_RCV_WINDOW_ADDRU_REG				0x0104
#define MSI_RCV_WINDOW_MASK_REG					0x0108
	#define MSI_RCV_NUM							32
	#define MSI_RCV_WINDOW_SIZE					(MSI_RCV_NUM * sizeof(unsigned int))
	#define MSI_RCV_WINDOW_MASK_MIN				((unsigned long)MSI_RCV_WINDOW_SIZE - 1)
	#define MSI_RCV_WINDOW_INVALID				0xFFFFFFFF
#define PCI_INTX_RCV_INTERRUPT_ENABLE_REG		0x0110
	#define MSI_RECEIVE_INTERRUPT_ENABLE		0x00000010
	#define INTX_RECEIVE_INTERRUPT_ENABLE		0x0000000F
#define PCI_INTX_RCV_INTERRUPT_STATUS_REG		0x0114
	#define MSI_RECEIVE_INTERRUPT_STATUS		0x00000010
	#define INTX_RECEIVE_INTERRUPT_STATUS		0x0000000F
	#define ALL_RECEIVE_INTERRUPT_STATUS		0x00000001F
#define PCI_INTX_OUT_STATUS_REG					0x0118

/* Message */
#define MSG_RCV_INTERRUPT_ENABLE_REG			0x0120
	#define	MESSAGE_ENABLE						0x01000000
#define MSG_RCV_INTERRUPT_STATUS_REG			0x0124
#define MSG_CODE_REG							0x0130
#define MSG_DATA_REG							0x0134
#define MSG_HEADER_3RDDW_REG					0x0138
#define MSG_HEADER_4THDW_REG					0x013C

/* Interrupt Table */
#define INTERRUPT_TABLE_REG						0x0140

/* Error Event */
#define PCIE_EVENT_INTERRUPT_EANBLE_0_REG		0x0200
	#define	DL_UPDOWN_ENABLE					0x00000200
#define PCIE_EVENT_INTERRUPT_STATUS_0_REG		0x0204
	#define	DL_UPDOWN_STATUS					0x00000200
#define AXI_MASTER_ERR_INTERRUPT_EANBLE_REG		0x0210
#define AXI_MASTER_ERR_INTERRUPT_STATUS_REG		0x0214
#define AXI_SLAVE_ERR_INTERRUPT_EANBLE_1_REG	0x0210
#define AXI_SLAVE_ERR_INTERRUPT_STATUS_1_REG	0x0214

/* Macro Control */
#define PERMISSION_REG							0x0300
	#define CFG_HWINIT_EN						0x00000004
#define PCI_RC_RESET_REG						0x0310
	#define RESET_ALL_DEASSERT					0x0000007F
	#define RESET_CONFIG_DEASSERT				0x0000001C
	#define RESET_ALL_ASSERT				    0x00000000
	#define RESET_LOAD_CFG_RELEASE				0x00000018
	#define RESET_PS_GP_RELEASE  				0x0000003B
#define RST_OUT_B                                                       (0x01 <<  6)
	#define RST_PS_B							(0x01 <<  5)
	#define RST_LOAD_B							(0x01 <<  4)
	#define RST_CFG_B                                                       (0x01 <<  3)
	#define RST_RSM_B							(0x01 <<  2)
	#define RST_GP_B							(0x01 <<  1)
	#define RST_B								(0x01 <<  0)
	#define RST_ALL_BIT							(RST_OUT_B  |   \
												RST_PS_B    |   \
												RST_LOAD_B  |   \
												RST_CFG_B   |   \
												RST_RSM_B   |   \
												RST_GP_B    |   \
												RST_B)
#define MODE_SET_0_REG							0x0314
#define MODE_SET_1_REG							0x0318
#define GENERAL_PURPOSE_OUTPUT_REG(x)			(0x0380 + ((x) * 0x04))
#define GENERAL_PURPOSE_INPUT_REG(x)			(0x0390 + ((x) * 0x04))
#define PCIE_CORE_MODE_SET_1_REG				0x0400
#define PCIE_CORE_CONTROL_1_REG					0x0404
#define PCIE_CORE_STATUS_1_REG					0x0408
	#define LTSSM_STATE_SHIFT					(10)
	#define LTSSM_STATE_MASK					(0x1F << LTSSM_STATE_SHIFT)
	#define LTSSM_ST_ALL_SHIFT					(8)
	#define LTSSM_ST_ALL_MASK					(0x7F << LTSSM_ST_ALL_SHIFT)
	#define LTSSM_ST_L2_IDLE					(0x3A)
	#define LTSSM_ST_DETECT						(0x0C << LTSSM_ST_ALL_SHIFT)
	#define DL_DOWN_STATUS						0x00000001
#define PCIE_LOOPBACK_TEST_REG					0x040C
#define PCIE_CORE_CONTROL_2_REG					0x0410
	#define STATE_RECEIVER_DETECTED				(0xff << 8)
#define PCIE_LINK_SPEED_CHANGE_MASK				(3 << 8)
#define PCIE_LINK_SPEED_CHANGE_5_0GTS				(1 << 8)
#define PCIE_LINK_SPEED_CHANGE_8_0GTS				(2 << 8)
#define PCIE_LINK_CHANGE_REASON_INTENTIONAL			BIT(4)
#define PCIE_LINK_SPEED_CHANGE_REQ				BIT(0)
#define PCIE_CORE_STATUS_2_REG					0x0414
#define PCIE_LINK_DATA_RATE					GENMASK(3, 1)
#define PCIE_LINK_DATA_RATE_2_5GTS				(1 << 1)
#define PCIE_LINK_DATA_RATE_5_0GTS				(3 << 1)
#define PCIE_LINK_DATA_RATE_8_0GTS				(7 << 1)

/* MSI receive register group */
#define PCI_RC_MSIRCVE(x)			(0x600 + 0x10 * (x))
#define PCI_RC_MSIRCVE_EN			BIT(0)
#define PCI_RC_MSIRCVMSK(x)			(0x608 + 0x10 * (x))
#define PCI_RC_MSIRCVMSK_MSI_MASK		0xFFFFFFFF
#define PCI_RC_MSIRCVSTAT(x)			(0x60C + 0x10 * (x))

/* PCIe RC Control */
#define PCI_RC_BASE_ADD							0x85030000
#define PCI_RC_MSGRCVIE_REG						0x0120
	#define  INT_MR_SET    						0x01050000
#define PCI_RC_MSGRCVIS_REG						0x0124
	#define  INT_MR_CLR							0x010F0000
#define PCI_RC_PEIE0_REG						0x0200
	#define INT_EN0_SET							0x00000000
#define PCI_RC_PEIS0_REG						0x0204
	#define INT_ST0_CLR							0x00001200
	#define INT_SPEED_CHANGE_DONE					BIT(29)
#define PCI_RC_PEIE1_REG						0x0208
	#define INT_EN1_SET							0x00000000
#define PCI_RC_PEIS1_REG						0x020c
	#define INT_ST1_CLR							0x00030303
#define PCI_RC_AMEIE_REG						0x0210
	#define INT_EN_AXIM_SET						0x00000F0F
#define PCI_RC_AMEIS_REG						0x0214
	#define INT_ST_AXIM_CLR						0x00000F0F
#define PCI_RC_ASEIE1_REG						0x0220
	#define INT_EN_AXIS_SET						0x00000F03
#define PCI_RC_ASEIS1_REG						0x0224
	#define INT_ST_AXIS_CLR						0x00000F03
#define PCI_RC_PERM_REG							0x0300
#define PCI_RC_VID_REG							0x6000
#define PCI_RC_RID_CC_REG						0x6008
	#define  REVID_CLSCODE_INIT					0xFFFFFFDF
#define PCI_RC_LINKCS						0x6070
	#define PCI_RC_LINKCS_CUR_LINK_SPEED_MASK		(0xF << 16)
	#define PCI_RC_LINKCS_CUR_LINK_SPEED_2_5GTS		(1 << 16)
	#define PCI_RC_LINKCS_CUR_LINK_SPEED_5_0GTS		(2 << 16)
	#define PCI_RC_LINKCS_CUR_LINK_SPEED_8_0GTS		(3 << 16)
#define PCI_RC_LINKCS2						0x6090
	#define PCI_RC_LINKCS2_TARGET_LINK_SPEED_MASK		0xF
	#define PCI_RC_LINKCS2_TARGET_LINK_SPEED_2_5GTS		1
	#define PCI_RC_LINKCS2_TARGET_LINK_SPEED_5_0GTS		2
	#define PCI_RC_LINKCS2_TARGET_LINK_SPEED_8_0GTS		3
#define PCI_RC_BARMSK00L_REG					0x60A0
	#define  BASEADR_MKL_ALLM					0xFFFFFFFF
#define PCI_RC_BARMSK00U_REG	 				0x60A4
	#define  BASEADR_MKU_ALLM					0xFFFFFFFF
#define PCI_RC_BSIZE00_01_REG	 				0x60C8
	#define  BASESZ_INIT						0x00000000

/* DMAC Common Control */
#define DMA_CONTROL_REG							0x0800
#define DMA_INTERRUPT_EANBLE_REG				0x0808
#define DMA_INTERRUPT_STATUS_REG				0x080C

/* DMAC Channel Control */
#define DMA_CHANNEL_CONTROL_REG					0x0900
#define QUE_ENTRY_LOWER_REG						0x0908
#define QUE_ENTRY_UPPER_REG						0x090C

/* DMAC DMA Setting */
#define DMA_DESCRIPTOR_CONTROL_REG				0x0920
#define DMA_SOURCE_ADDR_REG						0x0924
#define DMA_DESTINATION_ADDR_REG				0x0928
#define DMA_SIZE_REG							0x092C
#define DMA_PCIE_UPPER_ADDR_REG					0x0930
#define DMA_TRANSACTION_CONTROL_REG				0x0934
#define DMA_DESCRIPTOR_LINK_POINTER_REG			0x093C

/* DMAC DMA Status */
#define DMA_REST_SIZE_REG						0x0950
#define AXI_REQUEST_ADDR_REG					0x0954
#define PCIE_REQUEST_ADDR_LOWER_REG				0x0958
#define PCIE_REQUEST_ADDR_UPPER_REG				0x095C
#define QUE_STATUS_REG							0x0960
#define DMAC_ERROR_STATUS_REG					0x0968

/* PCIe Configuration Register */
#define PCIE_CONFIGURATION_REG					0x6000
	#define PCI_RC_VID_ADR						0x00
	#define PCI_RC_RID_CC_ADR					0x08
	#define PCI_PM_CAPABILITIES					0x40
	#define PCI_RC_BARMSK00L_ADR				0xA0
	#define PCI_RC_BARMSK00U_ADR				0xA4
	#define PCI_RC_BSIZE00_01_ADR				0xC8

/* PCIe Configuration Special Register offset */
#define PCIE_CONF_OFFSET_BAR0_MASK_LO			0x00A0
#define PCIE_CONF_OFFSET_BAR0_MASK_UP			0x00A4

/* PCI/AXI Window alignment */
#define RZV2H_WINDOW_SIZE_MIN					0x00001000
#define RZV2H_WINDOW_SIZE_MAX					0xFFFFFFFF
#define RZV2H_PCI_WINDOW_SIZE_MAX				0x40000000

#define INT_PCI_MSI_NR	32
#define INT_PCI_INTX_NR	1

#define RZV2H_PCI_MAX_RESOURCES 4
#define MAX_NR_INBOUND_MAPS		8

#define PCIE_CONF_BUS(b)	(((b) & 0xff) << 24)
#define PCIE_CONF_DEV(d)	(((d) & 0x1f) << 19)
#define PCIE_CONF_FUNC(f)	(((f) & 0x7) << 16)

#define STS_CHECK_LOOP	500

/* ----------------------------------------------------
  PCIe Configuration setting value
-------------------------------------------------------*/
#define PCIE_CONF_VENDOR_ID						0x1912
#define PCIE_CONF_DEVICE_ID						0x1135

#define PCIE_CONF_REVISION_ID					0x00

#define PCIE_CONF_BASE_CLASS					0x06
#define PCIE_CONF_SUB_CLASS						0x04
#define PCIE_CONF_PROGRAMING_IF					0x00

#define PM_CAPABILITIES_INIT					0x4803E001

#define PCIE_CONF_PRIMARY_BUS					0x00
#define PCIE_CONF_SECOUNDARY_BUS				0x01
#define PCIE_CONF_SUBORDINATE_BUS				0x01

#define PCIE_CONF_MEMORY_BASE					0xFFFF
#define PCIE_CONF_MEMORY_LIMIT					0xFFFF

#define PCIE_CONF_BAR0_MASK_LO					0xFFFFFFFF
#define PCIE_CONF_BAR0_MASK_UP					0xFFFFFFFF

#define LINK_WIDTH_CHANGE_ENABLE 				0x01000000
#define LINK_WIDTH_CHANGE_REQ_ON				0x00010000
#define LINK_WIDTH_CHANGE_DONE					0x20000000
#define LINK_WIDTH_CHANGE_REQ_OFF				0x00000000

#define  LAM_PREFETCH	BIT(3)
#define  LAM_64BIT		BIT(2)
#define  LAR_ENABLE		BIT(1)
#define  PAR_ENABLE		BIT(31)
#define  IO_SPACE		BIT(8)

/* ----------------------------------------------------
  RAMA Area
-------------------------------------------------------*/
#define RAMA_ADDRESS			 				0x80100000
#define RAMA_SIZE				 				0x32000

struct rzv2h_axi_window_set {
	u32	base[RZV2H_PCI_MAX_RESOURCES];
	u32	base_u[RZV2H_PCI_MAX_RESOURCES];
	u32	mask[RZV2H_PCI_MAX_RESOURCES];
	u32	dest[RZV2H_PCI_MAX_RESOURCES];
	u32	dest_u[RZV2H_PCI_MAX_RESOURCES];
};

struct rzv2h_pci_window_set {
	u32	base[RZV2H_PCI_MAX_RESOURCES];
	u32	mask[RZV2H_PCI_MAX_RESOURCES];
	u32	dest_u[RZV2H_PCI_MAX_RESOURCES];
	u32	dest_l[RZV2H_PCI_MAX_RESOURCES];
};

struct rzv2h_interrupt_set {
	u32	msi_win_addrl;
	u32	msi_win_addru;
	u32	msi_win_mask;
	u32	intx_ena;
	u32	msi_ena;
};

struct rzv2h_save_reg {
	struct rzv2h_axi_window_set		axi_window;
	struct rzv2h_pci_window_set		pci_window;
	struct rzv2h_interrupt_set		interrupt;
};

struct rzv2h_pcie {
	struct device			*dev;
	void __iomem			*base;
	struct rzv2h_save_reg	save_reg;
};

void rzv2h_pci_write_reg(struct rzv2h_pcie *pcie, u32 val, unsigned long reg);
u32 rzv2h_pci_read_reg(struct rzv2h_pcie *pcie, unsigned long reg);
void rzv2h_rmw(struct rzv2h_pcie *pcie, int where, u32 mask, u32 data);
u32 rzv2h_read_conf(struct rzv2h_pcie *pcie, int where);
void rzv2h_write_conf(struct rzv2h_pcie *pcie, u32 data, int where);
void rzv2h_pcie_set_outbound(struct rzv2h_pcie *pcie, int win,
			    struct resource_entry *window);
void rzv2h_pcie_set_inbound(struct rzv2h_pcie *pcie, u64 cpu_addr,
			   u64 pci_addr, u64 flags, int idx, bool host);

#endif
