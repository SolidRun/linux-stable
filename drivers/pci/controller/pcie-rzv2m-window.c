// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe driver for Renesas RZ/V2M Series SoCs
 *  Copyright (C) 2022 Renesas Electronics Europe Ltd
 */

#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "pcie-rzv2m.h"

/* ----------------------------------------------------
  Static Function
-------------------------------------------------------*/
static void rzv2m_cpg_set_clockctrl(struct rzv2m_pcie *pcie, unsigned char reg_num, unsigned short int target, unsigned short int set_value);
static void rzv2m_cpg_set_resetctrl(struct rzv2m_pcie *pcie, unsigned char reg_num, unsigned short int target, unsigned short int set_value);

/* ----------------------------------------------------
  PCIe Setting Function
-------------------------------------------------------*/
void rzv2m_pci_write_reg(struct rzv2m_pcie *pcie, u32 val, unsigned long reg)
{
	writel(val, pcie->base + reg);
}

u32 rzv2m_pci_read_reg(struct rzv2m_pcie *pcie, unsigned long reg)
{
	return readl(pcie->base + reg);
}

void rzv2m_rmw(struct rzv2m_pcie *pcie, int where, u32 mask, u32 data)
{
	u32 val = rzv2m_pci_read_reg(pcie, where);

	val &= ~(mask);
	val |= (data);
	rzv2m_pci_write_reg(pcie, val, where);
}

u32 rzv2m_read_conf(struct rzv2m_pcie *pcie, int where)
{
	int shift = 8 * (where & 3);
	u32 val = rzv2m_pci_read_reg(pcie, PCIE_CONFIGURATION_REG + (where & ~3) );

	return val >> shift;
}

void rzv2m_write_conf(struct rzv2m_pcie *pcie, u32 data, int where)
{
	rzv2m_pci_write_reg(pcie, CFG_HWINIT_EN, PERMISSION_REG );
	rzv2m_pci_write_reg(pcie, data, PCIE_CONFIGURATION_REG + where );
	rzv2m_pci_write_reg(pcie, 0, PERMISSION_REG );
}

void rzv2m_pcie_set_outbound(struct rzv2m_pcie *pcie, int win,
			     struct resource_entry *window,
			     bool has_64bits_regs)
{
	/* Setup PCIe address space mappings for each resource */
	struct resource *res = window->res;
	resource_size_t res_start;
	resource_size_t size;
	u32 mask;

	/*
	 * The PAMR mask is calculated in units of 128Bytes, which
	 * keeps things pretty simple.
	 */
	size = resource_size(res);
	mask = size - 1;

	if (res->flags & IORESOURCE_IO)
		res_start = pci_pio_to_address(res->start) - window->offset;
	else
		res_start = res->start - window->offset;

	rzv2m_pci_write_reg(pcie, res_start,
			    PCIE_WINDOW_BASE_REG(win, has_64bits_regs));
	rzv2m_pci_write_reg(pcie, upper_32_bits(res_start),
			    (PCIE_DESTINATION_REG(win, has_64bits_regs)) + 0x4);
	rzv2m_pci_write_reg(pcie, lower_32_bits(res_start),
			    PCIE_DESTINATION_REG(win, has_64bits_regs));

	rzv2m_pci_write_reg(pcie, mask,
			    PCIE_WINDOW_MASK_REG(win, has_64bits_regs));

	rzv2m_rmw(pcie, PCIE_WINDOW_BASE_REG(win, has_64bits_regs),
		  PCIE_WINDOW_ENABLE, PCIE_WINDOW_ENABLE);
}

void rzv2m_pcie_set_inbound(struct rzv2m_pcie *pcie, u64 cpu_addr,
			   u64 pci_addr, u64 flags, int idx, bool host,
			   bool has_64bits_regs)
{
	/*
	 * Set up 64-bit inbound regions as the range parser doesn't
	 * distinguish between 32 and 64-bit types.
	 */
	rzv2m_pci_write_reg(pcie, lower_32_bits(pci_addr),
			    AXI_WINDOW_BASE_REG(idx, has_64bits_regs));
	pcie->save_reg.axi_window.base_u[idx] = upper_32_bits(pci_addr);
	rzv2m_pci_write_reg(pcie, lower_32_bits(cpu_addr),
			    AXI_DESTINATION_REG(idx, has_64bits_regs));
	pcie->save_reg.axi_window.dest_u[idx] = upper_32_bits(cpu_addr);
	rzv2m_pci_write_reg(pcie, flags,
			    AXI_WINDOW_MASK_REG(idx, has_64bits_regs));
	rzv2m_rmw(pcie, AXI_WINDOW_BASE_REG(idx, has_64bits_regs),
		  AXI_WINDOW_ENABLE, AXI_WINDOW_ENABLE);

	rzv2m_pci_write_reg(pcie, RAMA_ADDRESS,
			    AXI_WINDOW_BASE_REG(1, has_64bits_regs));
	rzv2m_pci_write_reg(pcie, RAMA_ADDRESS,
			    AXI_DESTINATION_REG(1, has_64bits_regs));
	rzv2m_pci_write_reg(pcie, RAMA_SIZE - 1,
			    AXI_WINDOW_MASK_REG(1, has_64bits_regs));
	rzv2m_rmw(pcie, AXI_WINDOW_BASE_REG(1, has_64bits_regs),
		  AXI_WINDOW_ENABLE, AXI_WINDOW_ENABLE);
}

void rzg3s_pcie_set_inbound(struct rzv2m_pcie *pcie, u64 cpu_addr,
			   u64 pci_addr, u64 flags, int idx, bool host,
			   bool has_64bits_regs)
{
	/*
	 * Set up 64-bit inbound regions as the range parser doesn't
	 * distinguish between 32 and 64-bit types.
	 */
	rzv2m_pci_write_reg(pcie, lower_32_bits(pci_addr),
			    AXI_WINDOW_BASE_REG(idx, has_64bits_regs));
	rzv2m_pci_write_reg(pcie, upper_32_bits(pci_addr),
			    AXI_WINDOW_BASE_REG(idx, has_64bits_regs) + 0x4);
	pcie->save_reg.axi_window.base_u[idx] = upper_32_bits(pci_addr);
	rzv2m_pci_write_reg(pcie, lower_32_bits(cpu_addr),
			    AXI_DESTINATION_REG(idx, has_64bits_regs));
	rzv2m_pci_write_reg(pcie, upper_32_bits(cpu_addr),
			    AXI_DESTINATION_REG(idx, has_64bits_regs) + 0x4);
	pcie->save_reg.axi_window.dest_u[idx] = upper_32_bits(cpu_addr);
	rzv2m_pci_write_reg(pcie, lower_32_bits(flags),
			    AXI_WINDOW_MASK_REG(idx, has_64bits_regs));
	rzv2m_pci_write_reg(pcie, upper_32_bits(flags),
			    AXI_WINDOW_MASK_REG(idx, has_64bits_regs) + 0x4);
	rzv2m_rmw(pcie, AXI_WINDOW_BASE_REG(idx, has_64bits_regs),
		  AXI_WINDOW_ENABLE, AXI_WINDOW_ENABLE);
}

u32 rzv2m_read_conf_ep(struct rzv2m_pcie *pcie, int where, u8 fn)
{
	int shift = 8 * (where & 3);
	u32 val = rzv2m_pci_read_reg(pcie, PCIE_CONFIGURATION_REG_EP(fn) + (where & ~3) );

	return val >> shift;
}

void rzv2m_write_conf_ep(struct rzv2m_pcie *pcie, u32 data, int where, u8 fn)
{
	rzv2m_pci_write_reg(pcie, CFG_HWINIT_EN, PERMISSION_REG );
	rzv2m_pci_write_reg(pcie, data, PCIE_CONFIGURATION_REG_EP(fn) + where );
	rzv2m_pci_write_reg(pcie, 0, PERMISSION_REG );
}

void rzv2m_pci_bit_write_reg(struct rzv2m_pcie *pcie, u32 val, unsigned long reg)
{
	u32 data = rzv2m_pci_read_reg(pcie, reg);
	writel(data | val , pcie->base + reg);
}

void rzv2m_pci_bit_clear_reg(struct rzv2m_pcie *pcie, u32 val, unsigned long reg)
{
	u32 data = rzv2m_pci_read_reg(pcie, reg);
	writel(data | (~val) , pcie->base + reg);
}

void rzv2m_pcie_init(struct rzv2m_pcie *pcie)
{
	// set SYS register
	rzv2m_sys_write_reg(pcie, CNT_MOE, SYS_PCI_MODE_EN_B_REG);

	if (IS_ENABLED(PCIE_RZV2M_EP)) {
		rzv2m_sys_write_reg(pcie, SET_EP, SYS_PCI_MODE_REG);
	} else {
		rzv2m_sys_write_reg(pcie, SET_RC, SYS_PCI_MODE_REG);
	}
	msleep(1);

	/* assert RESET */
	rzv2m_cpg_set_resetctrl(pcie, 3, 0x1000, 0x0000);
	msleep(1);

	/* disable supply clock */
	rzv2m_cpg_set_clockctrl(pcie, 4, 0x0007, 0x0000);
	msleep(1);

	/* deassert RESET */
	rzv2m_cpg_set_resetctrl(pcie, 3, 0x1000, 0x1000);
	msleep(1);

	/* enable supply clock */
	rzv2m_cpg_set_clockctrl(pcie, 4, 0x0007, 0x0007);
	msleep(1);
}

int PCIE_INT_Initialize(struct rzv2m_pcie *pcie)
{
	/* Clear Event Interrupt Status 0 */
	rzv2m_pci_write_reg(pcie, INT_ST0_CLR, PCI_RC_PEIS0_REG);		/* Set PCI_RC 0204h */

	/* Set Event Interrupt Enable 0 */
	rzv2m_pci_write_reg(pcie, INT_EN0_SET, PCI_RC_PEIE0_REG);		/* Set PCI_RC 0200h */

	/* Clear  Event Interrupt Status 1 */
	rzv2m_pci_write_reg(pcie, INT_ST1_CLR, PCI_RC_PEIS1_REG);		/* Set PCI_RC 020ch */

	/* Set Event Interrupt Enable 1 */
	rzv2m_pci_write_reg(pcie, INT_EN1_SET, PCI_RC_PEIE1_REG);		/* Set PCI_RC 0208h */

	/* Clear AXI Master Error Interrupt Status */
	rzv2m_pci_write_reg(pcie, INT_ST_AXIM_CLR, PCI_RC_AMEIS_REG);	/* Set PCI_RC 0214h */

	/* Set AXI Master Error Interrupt Enable */
	rzv2m_pci_write_reg(pcie, INT_EN_AXIM_SET, PCI_RC_AMEIE_REG);	/* Set PCI_RC 0210h */

	/* Clear AXI Slave Error Interrupt Status */
	rzv2m_pci_write_reg(pcie, INT_ST_AXIS_CLR, PCI_RC_ASEIS1_REG);	/* Set PCI_RC 0224h */

	/* Set AXI Slave Error Interrupt Enable */
	rzv2m_pci_write_reg(pcie, INT_EN_AXIS_SET, PCI_RC_ASEIE1_REG);	/* Set PCI_RC 0220h */

	/* Clear Message Receive Interrupt Status */
	rzv2m_pci_write_reg(pcie, INT_MR_CLR, PCI_RC_MSGRCVIS_REG);		/* Set PCI_RC 0124h */

	/* Set Message Receive Interrupt Enable */
	rzv2m_pci_write_reg(pcie, INT_MR_SET, PCI_RC_MSGRCVIE_REG);		/* Set PCI_RC 0120h */

	return 0;
}

/* ----------------------------------------------------
  SYS Setting Function
-------------------------------------------------------*/
int rzv2m_pcie_sys_get_resources(struct rzv2m_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct resource res;
	int err;

	err = of_address_to_resource(dev->of_node, 1, &res);
	if (err)
		return err;

	pcie->sys_base = devm_ioremap_resource(dev, &res);

	if (IS_ERR(pcie->sys_base)) {
		err = PTR_ERR(pcie->sys_base);
		return err;
	}

	return 0;
}

void rzv2m_sys_write_reg(struct rzv2m_pcie *pcie, unsigned long val,  unsigned long reg)
{
	writel(val, pcie->sys_base + reg);
}

u32 rzv2m_sys_read_reg(struct rzv2m_pcie *pcie, unsigned long reg)
{
	return readl(pcie->sys_base + reg);
}

void sys_pcie_reg_bit_set(struct rzv2m_pcie *pcie, unsigned long val,  unsigned long reg)
{
	u32 data;

	data = readl(pcie->sys_base + reg);
	data |= val;
	writel(data, pcie->sys_base + reg);
}

void sys_pcie_reg_bit_clear(struct rzv2m_pcie *pcie, unsigned long val,  unsigned long reg)
{
	u32 data;

	data = readl(pcie->sys_base + reg);
	data &= (~val);
	writel(data, pcie->sys_base + reg);
}

void sys_pcie_tgl_pme_tim(struct rzv2m_pcie *pcie)
{
	u32 data;

	data = readl(pcie->sys_base + PCI_PME_TIM);
	data ^= 0x01;
	writel(data, pcie->sys_base + PCI_PME_TIM);
} /* End of function sys_pcie_tgl_pme_tim() */

_Bool sys_pcie_get_pme_sts_clr(struct rzv2m_pcie *pcie, u8 func_no)
{
	_Bool result = false;

	if (0 != (rzv2m_sys_read_reg(pcie, PCI_PME_STS_CLR) & PME_STS_CLR_FN(func_no)))
	{
		result = true;
	}
	return result;
} /* End of function sys_pcie_get_pme_sts_clr() */

/* ----------------------------------------------------
  PHY Setting Function
-------------------------------------------------------*/
int rzv2m_pcie_phy_get_resources(struct rzv2m_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct resource res;
	int err;

	err = of_address_to_resource(dev->of_node, 2, &res);
	if (err)
		return err;

	pcie->phy_base = devm_ioremap_resource(dev, &res);

	if (IS_ERR(pcie->phy_base)) {
		err = PTR_ERR(pcie->phy_base);
		return err;
	}

	return 0;
}

void rzv2m_pciphy_write_reg(struct rzv2m_pcie *pcie, unsigned long val,  unsigned long reg)
{
	writel(val, pcie->phy_base + reg);
}

int PCIE_phyInitialize_L0(struct rzv2m_pcie *pcie)
{
	rzv2m_sys_write_reg(pcie, SET_LANE0, SYS_PCI_LANE_SEL_REG);					// Set Lane0 reg

	/* PHY Initialize setting for Setting of PMA Register */
	rzv2m_pciphy_write_reg(pcie, 0x0032, PCI_PHYA_PLLPMSSDIV_REG);				/* PCI_PHYA 0D8h */
	rzv2m_pciphy_write_reg(pcie, 0x0001, PCI_PHYA_RXCDRREFDIVSELPLL_REG);		/* PCI_PHYA 480h */
	rzv2m_pciphy_write_reg(pcie, 0x0000, PCI_PHYA_RXCDRREFDIVSELDATA_REG);		/* PCI_PHYA 488h */
	rzv2m_pciphy_write_reg(pcie, 0x0004, PCI_PHYA_TXDDESKEW_REG);				/* PCI_PHYA 6ECh */
	rzv2m_pciphy_write_reg(pcie, 0x0004, PCI_PHYA_TXMISC_REG);					/* PCI_PHYA 73Ch */

	/* PHY parameters for TX : Reference value for signal adjustment */
	rzv2m_pciphy_write_reg(pcie, 0x0011, PCI_PHYA_PLLALPFRSELFINE_REG);			/* PCI_PHYA 080h */
	rzv2m_pciphy_write_reg(pcie, 0x003F, PCI_PHYA_TXDRVLVCTLG1_REG);			/* PCI_PHYA 404h */
	rzv2m_pciphy_write_reg(pcie, 0x001D, PCI_PHYA_TXDRVLVLCTLG2_REG);			/* PCI_PHYA 408h */
	rzv2m_pciphy_write_reg(pcie, 0x002B, PCI_PHYA_TXDRVPOSTLVCTLG1_REG);		/* PCI_PHYA 414h */
	rzv2m_pciphy_write_reg(pcie, 0x000A, PCI_PHYA_TXDRVPOSTLVCTLG2_REG);		/* PCI_PHYA 418h */
	rzv2m_pciphy_write_reg(pcie, 0x0007, PCI_PHYA_TXDRVIDRVEN_REG);				/* PCI_PHYA 42Ch */
	rzv2m_pciphy_write_reg(pcie, 0x00B7, PCI_PHYA_ATXDRVIDRVCTL_REG);			/* PCI_PHYA 430h */
	rzv2m_pciphy_write_reg(pcie, 0x00FF, PCI_PHYA_TXJEQEVENCTL_REG);			/* PCI_PHYA 44Ch */
	rzv2m_pciphy_write_reg(pcie, 0x0000, PCI_PHYA_TXJEQODDCTL_REG);				/* PCI_PHYA 454h */
	rzv2m_pciphy_write_reg(pcie, 0x0057, PCI_PHYA_ATXDRVACCDRV_REG);			/* PCI_PHYA 7F0h */

	/* PHY parameters for RX : Reference value for signal adjustment */
	rzv2m_pciphy_write_reg(pcie, 0x0073, PCI_PHYA_RXCTLEEN_REG);				/* PCI_PHYA 4B8h */
	rzv2m_pciphy_write_reg(pcie, 0x006F, PCI_PHYA_RXCTLEITAILCTLG1_REG);		/* PCI_PHYA 4C0h */
	rzv2m_pciphy_write_reg(pcie, 0x006C, PCI_PHYA_RXCTLEITAILCTLG2_REG);		/* PCI_PHYA 4C4h */
	rzv2m_pciphy_write_reg(pcie, 0x0013, PCI_PHYA_RXCTLERX1CTLG1_REG);			/* PCI_PHYA 4ECh */
	rzv2m_pciphy_write_reg(pcie, 0x00F2, PCI_PHYA_RXCTLERS1CTLG2_REG);			/* PCI_PHYA 4F0h */
	rzv2m_pciphy_write_reg(pcie, 0x0007, PCI_PHYA_ARXCTLEIBLEEDCTL_REG);		/* PCI_PHYA 514h */
	rzv2m_pciphy_write_reg(pcie, 0x00FF, PCI_PHYA_RXRTERM_REG);					/* PCI_PHYA 5A0h */
	rzv2m_pciphy_write_reg(pcie, 0x00F8, PCI_PHYA_RXRTERMVCMEN_REG);			/* PCI_PHYA 5ACh */
	rzv2m_pciphy_write_reg(pcie, 0x0065, PCI_PHYA_RXCDRFBBCTL_REG);				/* PCI_PHYA 678h */

	return 0;

} /* End of function PCIE_phyInitialize_L0() */


int PCIE_phyInitialize_L1(struct rzv2m_pcie *pcie)
{
	rzv2m_sys_write_reg(pcie, SET_LANE1, SYS_PCI_LANE_SEL_REG);					// Set Lane1 reg

	/* PHY Initialize setting for Setting of PMA Register */
	rzv2m_pciphy_write_reg(pcie, 0x0032, PCI_PHYA_PLLPMSSDIV_REG);				/* PCI_PHYA 0D8h */
	rzv2m_pciphy_write_reg(pcie, 0x0001, PCI_PHYA_RXCDRREFDIVSELPLL_REG);		/* PCI_PHYA 480h */
	rzv2m_pciphy_write_reg(pcie, 0x0000, PCI_PHYA_RXCDRREFDIVSELDATA_REG);		/* PCI_PHYA 488h */
	rzv2m_pciphy_write_reg(pcie, 0x0004, PCI_PHYA_TXDDESKEW_REG);				/* PCI_PHYA 6ECh */
	rzv2m_pciphy_write_reg(pcie, 0x0004, PCI_PHYA_TXMISC_REG);					/* PCI_PHYA 73Ch */

	/* PHY parameters for TX : Reference value for signal adjustment */
	rzv2m_pciphy_write_reg(pcie, 0x0011, PCI_PHYA_PLLALPFRSELFINE_REG);			/* PCI_PHYA 080h */
	rzv2m_pciphy_write_reg(pcie, 0x003F, PCI_PHYA_TXDRVLVCTLG1_REG);			/* PCI_PHYA 404h */
	rzv2m_pciphy_write_reg(pcie, 0x001D, PCI_PHYA_TXDRVLVLCTLG2_REG);			/* PCI_PHYA 408h */
	rzv2m_pciphy_write_reg(pcie, 0x002B, PCI_PHYA_TXDRVPOSTLVCTLG1_REG);		/* PCI_PHYA 414h */
	rzv2m_pciphy_write_reg(pcie, 0x000A, PCI_PHYA_TXDRVPOSTLVCTLG2_REG);		/* PCI_PHYA 418h */
	rzv2m_pciphy_write_reg(pcie, 0x0007, PCI_PHYA_TXDRVIDRVEN_REG);				/* PCI_PHYA 42Ch */
	rzv2m_pciphy_write_reg(pcie, 0x00B7, PCI_PHYA_ATXDRVIDRVCTL_REG);			/* PCI_PHYA 430h */
	rzv2m_pciphy_write_reg(pcie, 0x00FF, PCI_PHYA_TXJEQEVENCTL_REG);			/* PCI_PHYA 44Ch */
	rzv2m_pciphy_write_reg(pcie, 0x0000, PCI_PHYA_TXJEQODDCTL_REG);				/* PCI_PHYA 454h */
	rzv2m_pciphy_write_reg(pcie, 0x0057, PCI_PHYA_ATXDRVACCDRV_REG);			/* PCI_PHYA 7F0h */

	/* PHY parameters for RX : Reference value for signal adjustment */
	rzv2m_pciphy_write_reg(pcie, 0x0073, PCI_PHYA_RXCTLEEN_REG);				/* PCI_PHYA 4B8h */
	rzv2m_pciphy_write_reg(pcie, 0x006F, PCI_PHYA_RXCTLEITAILCTLG1_REG);		/* PCI_PHYA 4C0h */
	rzv2m_pciphy_write_reg(pcie, 0x006C, PCI_PHYA_RXCTLEITAILCTLG2_REG);		/* PCI_PHYA 4C4h */
	rzv2m_pciphy_write_reg(pcie, 0x0013, PCI_PHYA_RXCTLERX1CTLG1_REG);			/* PCI_PHYA 4ECh */
	rzv2m_pciphy_write_reg(pcie, 0x00F2, PCI_PHYA_RXCTLERS1CTLG2_REG);			/* PCI_PHYA 4F0h */
	rzv2m_pciphy_write_reg(pcie, 0x0007, PCI_PHYA_ARXCTLEIBLEEDCTL_REG);		/* PCI_PHYA 514h */
	rzv2m_pciphy_write_reg(pcie, 0x00FF, PCI_PHYA_RXRTERM_REG);					/* PCI_PHYA 5A0h */
	rzv2m_pciphy_write_reg(pcie, 0x00F8, PCI_PHYA_RXRTERMVCMEN_REG);			/* PCI_PHYA 5ACh */
	rzv2m_pciphy_write_reg(pcie, 0x0065, PCI_PHYA_RXCDRFBBCTL_REG);				/* PCI_PHYA 678h */

	return 0;

} /* End of function PCIE_phyInitialize_L1() */

/* ----------------------------------------------------
  CPG Setting Function
-------------------------------------------------------*/
int rzv2m_pcie_cpg_get_resources(struct rzv2m_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct resource res;
	int err;

	err = of_address_to_resource(dev->of_node, 3, &res);
	if (err)
		return err;

	pcie->cpg_base = devm_ioremap_resource(dev, &res);

	if (IS_ERR(pcie->cpg_base)) {
		err = PTR_ERR(pcie->cpg_base);
		return err;
	}

	return 0;
}

void rzv2m_cpg_write_reg(struct rzv2m_pcie *pcie, unsigned int val,  unsigned int reg)
{
	writel(val, pcie->cpg_base + reg);
}

static void rzv2m_cpg_set_clockctrl(struct rzv2m_pcie *pcie, unsigned char reg_num, unsigned short int target, unsigned short int set_value)
{
	unsigned int offset = CPG_CLK_ON1;
	unsigned int value;

	offset += ((reg_num - 1) * sizeof(unsigned int));

	value = ((unsigned int)target << CPG_REG_WEN_SHIFT)
			| (set_value & CPG_SET_DATA_MASK);

	rzv2m_cpg_write_reg(pcie, value, offset);
}

static void rzv2m_cpg_set_resetctrl(struct rzv2m_pcie *pcie, unsigned char reg_num, unsigned short int target, unsigned short int set_value)
{
	unsigned int offset = CPG_RST1;
	unsigned int value;

	offset += ((reg_num - 1) * sizeof(unsigned int));

	value = ((unsigned int)target << CPG_REG_WEN_SHIFT) | (set_value & CPG_SET_DATA_MASK);

	rzv2m_cpg_write_reg(pcie, value, offset);
}
