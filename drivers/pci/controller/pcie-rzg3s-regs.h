/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe driver for Renesas RZ/G3S SoCs
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 *
 */

#ifndef _PCIE_RZG3S_H
#define _PCIE_RZG3S_H

/* AXI registers */
#define RZG3S_PCI_REQDATA(id)			(0x80 + (id) * 0x4)
#define RZG3S_PCI_REQRCVDAT			0x8c

#define RZG3S_PCI_REQADR1			0x90
#define RZG3S_PCI_REQADR1_BUS			GENMASK(31, 24)
#define RZG3S_PCI_REQADR1_DEV			GENMASK(23, 19)
#define RZG3S_PCI_REQADR1_FUNC			GENMASK(18, 16)
#define RZG3S_PCI_REQADR1_REG			GENMASK(11, 0)

#define RZG3S_PCI_REQBE				0x98
#define RZG3S_PCI_REQBE_BYTE_EN			GENMASK(3, 0)

#define RZG3S_PCI_REQISS			0x9c
#define RZG3S_PCI_REQISS_MOR_STATUS		GENMASK(18, 16)
#define RZG3S_PCI_REQISS_TR_TYPE		GENMASK(11, 8)
#define RZG3S_PCI_REQISS_TR_TP0_RD		FIELD_PREP(RZG3S_PCI_REQISS_TR_TYPE, 0x4)
#define RZG3S_PCI_REQISS_TR_TP0_WR		FIELD_PREP(RZG3S_PCI_REQISS_TR_TYPE, 0x5)
#define RZG3S_PCI_REQISS_TR_TP1_RD		FIELD_PREP(RZG3S_PCI_REQISS_TR_TYPE, 0x6)
#define RZG3S_PCI_REQISS_TR_TP1_WR		FIELD_PREP(RZG3S_PCI_REQISS_TR_TYPE, 0x7)

#define RZG3S_PCI_REQISS_REQ_ISSUE		BIT(0)

#define RZG3S_PCI_MSIRCVWADRL			0x100
#define RZG3S_PCI_MSIRCVWADRL_MSG_DATA_ENA	BIT(1)
#define RZG3S_PCI_MSIRCVWADRL_ENA		BIT(0)

#define RZG3S_PCI_MSIRCVWADRU			0x104
#define RZG3S_PCI_MSIRCVWMSKL			0x108
#define RZG3S_PCI_MSIRCVWMSKU			0x10c

#define RZG3S_PCI_PINTRCVIE			0x110
#define RZG3S_PCI_PINTRCVIE_INTX(i)		BIT(i)
#define RZG3S_PCI_PINTRCVIE_MSI			BIT(4)

#define RZG3S_PCI_PINTRCVIS			0x114
#define RZG3S_PCI_PINTRCVIS_INTX(i)		BIT(i)
#define RZG3S_PCI_PINTRCVIS_MSI			BIT(4)

#define RZG3S_PCI_MSGRCVIE			0x120
#define RZG3S_PCI_MSGRCVIE_MSG_RCV		BIT(24)

#define RZG3S_PCI_MSGRCVIS			0x124
#define RZG3S_PCI_MSGRCVIS_MRI			BIT(24)

#define RZG3S_PCI_PEIE0				0x200
#define RZG3S_PCI_PEIE0_DL_UPDOWN		BIT(9)

#define RZG3S_PCI_PEIS0				0x204
#define RZG3S_PCI_PEIS0_RX_DLLP_PM_ENTER	BIT(12)
#define RZG3S_PCI_PEIS0_DL_UPDOWN		BIT(9)

#define RZG3S_PCI_PEIE1				0x208
#define RZG3S_PCI_PEIS1				0x20c
#define RZG3S_PCI_AMEIE				0x210
#define RZG3S_PCI_AMEIS				0x214
#define RZG3S_PCI_ASEIE1			0x220
#define RZG3S_PCI_ASEIS1			0x224

#define RZG3S_PCI_PCSTAT1			0x408
#define RZG3S_PCI_PCSTAT1_LTSSM_STATE		GENMASK(14, 10)
#define RZG3S_PCI_PCSTAT1_DL_DOWN_STS		BIT(0)

#define RZG3S_PCI_PCCTRL2			0x410
#define RZG3S_PCI_PCCTRL2_LS_CHG		GENMASK(9, 8)
#define RZG3S_PCI_PCCTRL2_LINK_REASON		BIT(4)
#define RZG3S_PCI_PCCTRL2_LS_CHG_REQ		BIT(0)

#define RZG3S_PCI_PCSTAT2			0x414
#define RZG3S_PCI_PCSTAT2_LS_CHG_DONE		BIT(28)
#define RZG3S_PCI_PCSTAT2_STATE_RX_DETECT	GENMASK(15, 8)
#define RZG3S_PCI_PCSTAT2_SDRIRE		GENMASK(7, 0)

#define RZG3S_PCI_PERM				0x300
#define RZG3S_PCI_PERM_CFG_HWINIT_EN		BIT(2)
#define RZG3S_PCI_PERM_PIPE_PHY_REG_EN		BIT(1)

#define RZG3S_PCI_MSIRE(id)			(0x600 + (id) * 0x10)
#define RZG3S_PCI_MSIRE_ENA			BIT(0)

#define RZG3S_PCI_MSIRM(id)			(0x608 + (id) * 0x10)
#define RZG3S_PCI_MSIRS(id)			(0x60c + (id) * 0x10)

#define RZG3S_PCI_AWBASEL(id)			(0x1000 + (id) * 0x20)
#define RZG3S_PCI_AWBASEL_WIN_ENA		BIT(0)

#define RZG3S_PCI_AWBASEU(id)			(0x1004 + (id) * 0x20)
#define RZG3S_PCI_AWMASKL(id)			(0x1008 + (id) * 0x20)
#define RZG3S_PCI_AWMASKU(id)			(0x100c + (id) * 0x20)
#define RZG3S_PCI_ADESTL(id)			(0x1010 + (id) * 0x20)
#define RZG3S_PCI_ADESTU(id)			(0x1014 + (id) * 0x20)

#define RZG3S_PCI_PWBASEL(id)			(0x1100 + (id) * 0x20)
#define RZG3S_PCI_PWBASEL_ENA			BIT(0)

#define RZG3S_PCI_PWBASEU(id)			(0x1104 + (id) * 0x20)
#define RZG3S_PCI_PDESTL(id)			(0x1110 + (id) * 0x20)
#define RZG3S_PCI_PDESTU(id)			(0x1114 + (id) * 0x20)
#define RZG3S_PCI_PWMASKL(id)			(0x1108 + (id) * 0x20)
#define RZG3S_PCI_PWMASKU(id)			(0x110c + (id) * 0x20)

/* PHY control registers */
#define RZG3S_PCI_PHY_XCFGD(id)			(0x2000 + (id) * 0x10)
#define RZG3S_PCI_PHY_XCFGD_NUM			39

#define RZG3S_PCI_PHY_XCFGA_CMN(id)		(0x2400 + (id) * 0x10)
#define RZG3S_PCI_PHY_XCFGA_CMN_NUM		16

#define RZG3S_PCI_PHY_XCFGA_RX(id)		(0x2500 + (id) * 0x10)
#define RZG3S_PCI_PHY_XCFGA_RX_NUM		13

#define RZG3S_PCI_PHY_XCFGA_TX			0x25d0

#define RZG3S_PCI_PHY_XCFG_CTRL			0x2a20
#define RZG3S_PCI_PHY_XCFG_CTRL_PHYREG_SEL	BIT(0)

/* PCIe registers */
#define RZG3S_PCI_CFG_BASE			0x6000
#define RZG3S_PCI_CFG_BARMSK00L			0xa0
#define RZG3S_PCI_CFG_BARMSK00U			0xa4

#define RZG3S_PCI_CFG_PCIEC			0x60

/* PCIe Configuration Register */
#define RZG3S_PCIE_CONF_REVISION_ID		0x00
#define RZG3S_PCIE_CONF_PROGRAMING_IF		0x00
#define RZG3S_PCIE_CONF_BASE_CLASS		0x06
#define RZG3S_PCIE_CONF_SUB_CLASS		0x04

/* System controller registers */
#define RZG3S_SYS_PCIE_RST_RSM_B		0xd74
#define RZG3S_SYS_PCIE_RST_RSM_B_MASK		BIT(0)

/* Maximum number of windows */
#define RZG3S_MAX_WINDOWS			8

/* Number of MSI interrupts per register */
#define RZG3S_PCI_MSI_INT_PER_REG		32
/* The number of MSI interrupts */
#define RZG3S_PCI_MSI_INT_NR			RZG3S_PCI_MSI_INT_PER_REG

/* Timeouts experimentally determined. */
#define RZG3S_REQ_ISSUE_TIMEOUT_US		2500

/* Only in RZ/V2H */
#define PCIE_MAX_CHANNEL			2

/* PCIE SYS Registers */
#define RZV2H_SYS_PCIE_MISC_CH(x)		(0x1020 + ((x) * 0x30))
#define	RZV2H_ALLOW_ENTER_MASK			BIT(0)

#define RZV2H_SYS_PCIE_MODE_CH(x)		(0x1024 + ((x) * 0x30))
#define RZV2H_MODE_PORT_SYS_MASK		BIT(0)
#define RZV2H_MODE_PORT_SYS_RC			1

#define RZV2H_SYS_PCIE_LANE_MODE		0x1060
#define RZV2H_SYS_PCIE_LANE_MODE_MASK		GENMASK(9, 8)
#define RZV2H_LINK_MASTER_4_LANE_MODE		1	/* 4 lane * 1 mode */
#define RZV2H_LINK_MASTER_2_LANE_MODE		3	/* 2 lane * 2 mode */

/* AXI registers */
#define RZV2H_PCI_RESET_REG			0x0310
#define RZV2H_RST_B				BIT(0)
#define RZV2H_RST_GP_B				BIT(1)
#define RZV2H_RST_RSM_B				BIT(2)
#define RZV2H_RST_CFG_B				BIT(3)
#define RZV2H_RST_LOAD_B			BIT(4)
#define RZV2H_RST_PS_B				BIT(5)
#define RZV2H_RST_OUT_B				BIT(6)
#define RZV2H_RST_PREG_B			BIT(7)
#define RZV2H_RESET_ALL_ASSERT			GENMASK(6, 0)
#define RZV2H_RESET_LOAD_CFG_RELEASE		(BIT(3) | BIT(4))
#define RZV2H_RESET_PS_GP_RELEASE		(BIT(0) | BIT(1) | BIT(5))
#define RZV2H_RESET_OUT_RSM_RELEASE		(BIT(2) | BIT(6))
#define RZV2H_RESET_CONFIG_DEASSERT		(BIT(2) | BIT(3) | BIT(4))
#define RZV2H_RESET_ALL_DEASSERT		0U
#define RZV2H_RESET_DETECT			(RZV2H_RESET_PS_GP_RELEASE |\
						 BIT(3))

#define PCIE_LINK_DATA_RATE			GENMASK(3, 1)
#define PCIE_LINK_DATA_RATE_2_5GTS		BIT(1)
#define PCIE_LINK_DATA_RATE_5_0GTS		GENMASK(2, 1)
#define PCIE_LINK_DATA_RATE_8_0GTS		GENMASK(3, 1)

#define PCIE_LINK_WAIT_DL_MS			5
#define PCIE_LINK_WAIT_DL_MAX_RETRIES		50

/*
 * RZV2H PCIe Endpoint Register
 */

/* PCIe IRQ Control */
#define INT_MR_SET_EP				0x010A0000
#define INT_MR_CLR				0x010F0000
#define INT_EN0_SET				0x00000000
#define INT_ST0_CLR				0x00001200
#define INT_EN1_SET				0x00000000
#define INT_ST1_CLR				0x00030303
#define INT_EN_AXIM_SET				0x00000F0F
#define INT_ST_AXIM_CLR				0x00000F0F
#define INT_EN_AXIS_SET				0x00000F03
#define INT_ST_AXIS_CLR				0x00000F03

#define MODE_SET_1_REG				0x0318
#define PCIE_CORE_MODE_SET_1_REG		0x0400
#define MODE_PORT				(0x01 << 1)

/* RZ/V2H macro */
#define RZV2H_PCI_MAX_RESOURCES			4
#define RZV2H_PCI_MAX_RESOURCES_EP		2
#define MAX_NR_INBOUND_MAPS			8
#define MAX_NR_INBOUND_MAPS_EP			6

/* Macro Control */
#define PERMISSION_REG				0x0300
#define PIPE_PHY_REG_EN				0x00000002
#define CFG_HWINIT_EN				0x00000004
#define CFG_PIPEPHY_EN				0x00000002

/* PCIe Configuration Register */
#define PCIE_CONFIGURATION_REG_EP(f)		(0x6000 + (0x1000 * (f)))
#define PCI_EP_VID_ADR				0x00
#define PCI_EP_COMMAND_AND_STATUS		0x04
#define PCI_EP_BUS_MASTER_ENABLE		BIT(2)
#define PCI_EP_MEMORY_SPACE_ENABLE		BIT(1)
#define PCI_EP_RID_CC_ADR			0x08
#define PCI_EP_SUBSYS_ID_ADR			0x2C
#define PCI_EP_INTERRUPT_ADR			0x3C
#define PCI_EP_OWN_PM_STS_CTRL_REG		0x44
#define PCI_EP_DEVICE_CONTROL_ADDR		0x68
#define	DEVICE_CONTROL_INIT			0x2030
#define PCI_EP_BAR_MASK_ADR(idx)		(0xA0 + ((idx) * 0x4))
#define PCI_EP_BSIZE00_0001_ADR			0xC8
#define PCIE_CFG_BASE_SIZE_0001_EP_F0		0U
#define PCI_EP_BSIZE00_0203_ADR			0xCC
#define PCIE_CFG_BASE_SIZE_0203_EP_F0		0U
#define PCI_EP_BSIZE00_0405_ADR			0xD0
#define PCIE_CFG_BASE_SIZE_0405_EP_F0		0U
#define PCI_EP_BSIZE00_0006_ADR			0xD4
#define PCIE_CFG_BASE_SIZE_0006_EP_F0		0U

/* MSI Endpoint register */
#define PCI_EP_MSICAP(x)			(0xE0 + ((x) * 0x4))
#define MSICAP0_MSIE				BIT(16)
#define MSICAP0_MMESCAP_OFFSET			17
#define MSICAP0_MMESE_OFFSET			20
#define MSICAP0_MMESE_MASK			GENMASK(22, 20)

/* Macro */
#define RZV2H_EPC_MAX_FUNCTIONS						2

/* PCIe Endpoint Configuration setting value */
#define PCIE_CONF_REVISION_ID			0x00
#define PCIE_CONF_BASE_CLASS			0x06
#define PCIE_CONF_SUB_CLASS			0x04
#define PCIE_CONF_PROGRAMING_IF			0x00

#define PCIE_CONF_VENDOR_ID			0x1912

#define  LAM_64BIT				BIT(2)
#define  LAR_ENABLE				BIT(1)

/* PCIE SYSC Registers */
#define SYS_PCIE_INTX_CH(x)			(0x1000 + ((x) * 0x30))
#define SYS_PCIE_MSI1_CH(x)			(0x1004 + ((x) * 0x30))
#define SYS_PCIE_MSI2_CH(x)			(0x1008 + ((x) * 0x30))
#define SYS_PCIE_MSI3_CH(x)			(0x100c + ((x) * 0x30))
#define SYS_PCIE_MSI4_CH(x)			(0x1010 + ((x) * 0x30))
#define SYS_PCIE_MSI5_CH(x)			(0x1014 + ((x) * 0x30))
#define SYS_PCIE_PME_CH(x)			(0x1018 + ((x) * 0x30))
#define SYS_PCIE_ACK_CH(x)			(0x101c + ((x) * 0x30))
#define SYS_PCIE_MISC_CH(x)			(0x1020 + ((x) * 0x30))
#define	ALLOW_ENTER_L1				0x01
#define SYS_PCIE_MODE_CH(x)			(0x1024 + ((x) * 0x30))
#define MODE_PORT_SYS_RC			0x01
#define	MODE_PORT_SYS_EP			0x00
#define SYS_PCIE_LANE_MODE			0x1060
#define LINK_MASTER_4_LANE_MODE			0x100	/* 4 lane * 1 mode */
#define LINK_MASTER_2_LANE_MODE			0x300	/* 2 lane * 2 mode */

#endif
