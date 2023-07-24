// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe driver for Renesas RZ/V2M Series SoCs
 *  Copyright (C) 2022 Renesas Electronics Ltd
 *
 * Based on:
 *  arch/sh/drivers/pci/pcie-sh7786.c
 *  arch/sh/drivers/pci/ops-sh7786.c
 *  Copyright (C) 2009 - 2011  Paul Mundt
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "pcie-rzv2m.h"

struct rzv2m_msi {
	DECLARE_BITMAP(used, INT_PCI_MSI_NR);
	struct irq_domain *domain;
	struct msi_controller chip;
	unsigned long pages;
	unsigned long virt_pages;
	struct mutex lock;
	int irq;
};

static u32 r_configuration_space[] = {
	0x00000004,
	0x00000000,
	0xfff0fff0,
	0x48035001
};

static u32 r_msi_capability[] = {
	0x01807005,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000
};

static u32 r_msi_and_msix_capability[] = {
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000
};

static u32 r_virtual_channel_enhanced_capability_header[] = {
	0x00010002,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x800000FF,
	0x00020000,
	0x00020000
};

static u32 r_device_serial_number_capability[] = {
	0x00010003,
	0x00000000,
	0x00000000
};


static inline struct rzv2m_msi *to_rzv2m_msi(struct msi_controller *chip)
{
	return container_of(chip, struct rzv2m_msi, chip);
}

/* Structure representing the PCIe interface */
struct rzv2m_pcie_host {
	struct rzv2m_pcie	pcie;
	struct device		*dev;
	struct phy		*phy;
	void __iomem		*base;
	struct clk		*bus_clk;
	struct			rzv2m_msi msi;
	int			(*phy_init_fn)(struct rzv2m_pcie_host *host);
	struct irq_domain	*intx_domain;
};

static int rzv2m_pcie_hw_init(struct rzv2m_pcie *pcie);

static int rzv2m_pcie_request_issue(struct rzv2m_pcie *pcie, struct pci_bus *bus)
{
	int i;
	u32 sts;

	rzv2m_rmw(pcie, REQUEST_ISSUE_REG, REQ_ISSUE, REQ_ISSUE);
	for (i = 0; i < STS_CHECK_LOOP; i++) {
		sts = rzv2m_pci_read_reg(pcie, REQUEST_ISSUE_REG);
		if( !(sts & REQ_ISSUE) ) break;

		udelay(5);
	}

	if( sts & MOR_STATUS ) {
		dev_dbg(&bus->dev, "rzv2m_pcie_conf_access: Request failed(%d)\n",((sts & MOR_STATUS)>>16) );
	
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int rzv2m_pcie_read_config_access(struct rzv2m_pcie_host *host,
		struct pci_bus *bus, unsigned int devfn, int where, u32 *data)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	unsigned int dev, func, reg, ret;

	dev = PCI_SLOT(devfn);
	func = PCI_FUNC(devfn);
	reg = where & ~3;

	/*
	 * While each channel has its own memory-mapped extended config
	 * space, it's generally only accessible when in endpoint mode.
	 * When in root complex mode, the controller is unable to target
	 * itself with either type 0 or type 1 accesses, and indeed, any
	 * controller initiated target transfer to its own config space
	 * result in a completer abort.
	 *
	 * Each channel effectively only supports a single device, but as
	 * the same channel <-> device access works for any PCI_SLOT()
	 * value, we cheat a bit here and bind the controller's config
	 * space to devfn 0 in order to enable self-enumeration. In this
	 * case the regular ECAR/ECDR path is sidelined and the mangled
	 * config access itself is initiated as an internal bus transaction.
	 */
	if ( pci_is_root_bus(bus) && (devfn == 0) ) {
		if (dev != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;

		if (reg==0x10)
		{
			*data = r_configuration_space[0];
		}
		else if (reg==0x14)
		{
			*data = r_configuration_space[1];
		}
		else if (reg==0x20)
		{
			*data = r_configuration_space[2];
		}
		else if (reg==0x40)
		{
			*data = r_configuration_space[3];
		}
		else if ( (reg>=0x50) && (reg<=0x64) )
		{
			//MSI Capability register
			*data = r_msi_capability[(reg-0x50)/4];
		}
		else if ( (reg>=0x70) && (reg<=0xA8) )
		{
			//PCI Express Capability register
			*data = rzv2m_read_conf(pcie, reg-0x10);
		}
		else if ( (reg>=0xE0) && (reg<=0xFC) )
		{
			//MSI-X Capability register
			*data = r_msi_and_msix_capability[(reg-0xE0)/4];
		}
		else if ( (reg>=0x100) && (reg<=0x118) )
		{
			*data = r_virtual_channel_enhanced_capability_header[(reg-0x100)/4];
		}
		else if ( (reg>=0x1B0) && (reg<=0x1B8) )
		{
			*data = r_device_serial_number_capability[(reg-0x1B0)/4];
		}
		else
		{
			*data = rzv2m_read_conf(pcie, reg);
		}

		return PCIBIOS_SUCCESSFUL;
	}

	reg &= 0x0FFC;

	if (bus->number == 1) {
		/* Type 0 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFREAD_TP0, REQUEST_ISSUE_REG );
	}
	else {
		/* Type 1 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_DEV(dev) | PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFREAD_TP1, REQUEST_ISSUE_REG );
	}

	ret = rzv2m_pcie_request_issue(pcie, bus);
	if(ret != PCIBIOS_SUCCESSFUL)
		return ret;

	*data = rzv2m_pci_read_reg(pcie, REQUEST_RCV_DATA_REG);

	return PCIBIOS_SUCCESSFUL;
}

static int rzv2m_pcie_write_config_access(struct rzv2m_pcie_host *host,
		struct pci_bus *bus, unsigned int devfn, int where, u32 data)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	unsigned int dev, func, reg;

	dev = PCI_SLOT(devfn);
	func = PCI_FUNC(devfn);
	reg = where & ~3;

	/*
	 * While each channel has its own memory-mapped extended config
	 * space, it's generally only accessible when in endpoint mode.
	 * When in root complex mode, the controller is unable to target
	 * itself with either type 0 or type 1 accesses, and indeed, any
	 * controller initiated target transfer to its own config space
	 * result in a completer abort.
	 *
	 * Each channel effectively only supports a single device, but as
	 * the same channel <-> device access works for any PCI_SLOT()
	 * value, we cheat a bit here and bind the controller's config
	 * space to devfn 0 in order to enable self-enumeration. In this
	 * case the regular ECAR/ECDR path is sidelined and the mangled
	 * config access itself is initiated as an internal bus transaction.
	 */
	if ( pci_is_root_bus(bus) && (devfn == 0) ) {
		if (dev != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;

		if (reg==0x20)
		{
			r_configuration_space[2] = data;
		}
		else if (reg==0x40)
		{
			r_configuration_space[3] = data;
		}
		else if ( (reg>=0x50) && (reg<=0x64) )
		{
			//MSI capability register
			r_msi_capability[(reg-0x50)/4] = data;
		}
		else if ( (reg>=0x70) && (reg<=0xA8) )
		{
			//PCI Express Capability register
			rzv2m_write_conf(pcie, data, reg-0x10);
		}
		else if ( (reg>=0xE0) && (reg<=0xFC) )
		{
			//MSI-X capability register
			r_msi_and_msix_capability[(reg-0xE0)/4] = data;
		}
		else if ( (reg>=0x100) && (reg<=0x118) )
		{
			r_virtual_channel_enhanced_capability_header[(reg-0x100)/4] = data;
		}
		else if ( (reg>=0x1B0) && (reg<=0x1B8) )
		{
			r_device_serial_number_capability[(reg-0x1B0)/4] = data;
		}
		else
		{
			rzv2m_write_conf(pcie, data, reg);
		}

		return PCIBIOS_SUCCESSFUL;
	}

	reg &= 0x0FFC;

	rzv2m_pci_write_reg(pcie, 0, REQUEST_DATA_REG(0) );
	rzv2m_pci_write_reg(pcie, 0, REQUEST_DATA_REG(1) );

        if (reg == 0x54)
                rzv2m_pci_write_reg(pcie, RAMA_ADDRESS, REQUEST_DATA_REG(2) );
        else
                rzv2m_pci_write_reg(pcie, data, REQUEST_DATA_REG(2) );

	if (bus->number == 1) {
		/* Type 0 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFWRITE_TP0, REQUEST_ISSUE_REG );
	}
	else {
		/* Type 1 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_DEV(dev) | PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFWRITE_TP1, REQUEST_ISSUE_REG );
	}

	return rzv2m_pcie_request_issue(pcie, bus);
}

static int rzv2m_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
			       int where, int size, u32 *val)
{
	struct rzv2m_pcie_host *host = bus->sysdata;
	int ret;

	if ( (bus->number == 0) && (devfn >= 0x08) && (where == 0x0) )
		return PCIBIOS_DEVICE_NOT_FOUND;

	ret = rzv2m_pcie_read_config_access(host, bus, devfn, where, val);
	if (ret != PCIBIOS_SUCCESSFUL) {
		*val = 0xffffffff;
		return ret;
	}

	if (size == 1)
		*val = (*val >> (BITS_PER_BYTE * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (BITS_PER_BYTE * (where & 2))) & 0xffff;

	dev_dbg(&bus->dev, "pcie-config-read: bus=%3d devfn=0x%04x where=0x%04x size=%d val=0x%08x\n",
		bus->number, devfn, where, size, *val);

	return ret;
}

static int rzv2m_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val)
{
	struct rzv2m_pcie_host *host = bus->sysdata;
	unsigned int shift;
	u32 data;
	int ret;

	ret = rzv2m_pcie_read_config_access(host, bus, devfn, where, &data);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	dev_dbg(&bus->dev, "pcie-config-write: bus=%3d devfn=0x%04x where=0x%04x size=%d val=0x%08x\n",
		bus->number, devfn, where, size, val);

	if (size == 1) {
		shift = BITS_PER_BYTE * (where & 3);
		data &= ~(0xff << shift);
		data |= ((val & 0xff) << shift);
	} else if (size == 2) {
		shift = BITS_PER_BYTE * (where & 2);
		data &= ~(0xffff << shift);
		data |= ((val & 0xffff) << shift);
	} else
		data = val;

	ret = rzv2m_pcie_write_config_access(host, bus, devfn, where, data);

	return ret;
}

static struct pci_ops rzv2m_pcie_ops = {
	.read	= rzv2m_pcie_read_conf,
	.write	= rzv2m_pcie_write_conf,
};

static void rzv2m_pcie_hw_enable(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *win;
	LIST_HEAD(res);
	int i = 0;

	/* Setup PCI resources */
	resource_list_for_each_entry(win, &bridge->windows) {
		struct resource *res = win->res;

		if (!res->flags)
			continue;

		switch (resource_type(res)) {
		case IORESOURCE_IO:
		case IORESOURCE_MEM:
			rzv2m_pcie_set_outbound(pcie, i, win);
			i++;
			break;
		}
	}
}

static int rzv2m_pcie_enable(struct rzv2m_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);

	rzv2m_pcie_hw_enable(host);

	pci_add_flags(PCI_REASSIGN_ALL_BUS);

	bridge->sysdata = host;
	bridge->ops = &rzv2m_pcie_ops;
	if (IS_ENABLED(CONFIG_PCI_MSI))
		bridge->msi = &host->msi.chip;

	return pci_host_probe(bridge);
}

static void rzv2m_pcie_fixup_pcibridge(struct pci_dev *pdev)
{
	u16 linkcap, linkstat;

	pcie_capability_read_word(pdev, PCI_EXP_LNKCAP, &linkcap);
	if ((linkcap & PCI_EXP_LNKCAP_SLS) <= PCI_EXP_LNKCAP_SLS_2_5GB) {
		return;
	}

	pcie_capability_read_word(pdev, PCI_EXP_LNKSTA, &linkstat);
	if ((linkstat & PCI_EXP_LNKSTA_CLS) == PCI_EXP_LNKSTA_CLS_2_5GB) {
		pcie_capability_set_word(pdev, PCI_EXP_LNKCTL,
					 PCI_EXP_LNKCTL_RL);
	}

}
DECLARE_PCI_FIXUP_EARLY(PCIE_CONF_VENDOR_ID, PCIE_CONF_DEVICE_ID, rzv2m_pcie_fixup_pcibridge);

static void rzv2m_pcie_setting_config(struct rzv2m_pcie *pcie)
{
   rzv2m_pci_write_reg(pcie, RESET_CONFIG_DEASSERT, PCI_RESET_REG);

	// Configuration space(Root complex) setting
	// Vendor and Device ID      : PCI Express Configuration Registers Adr 6000h
	rzv2m_write_conf(pcie,
			( (PCIE_CONF_DEVICE_ID << 16) |
			  (PCIE_CONF_VENDOR_ID) ),
			PCI_RC_VID_ADR);

	// Revision ID and Class Code : PCI Express Configuration Registers Adr 6008h
	rzv2m_write_conf(pcie,
			( (PCIE_CONF_BASE_CLASS   << 24) |
			  (PCIE_CONF_SUB_CLASS    << 16) |
			  (PCIE_CONF_PROGRAMING_IF << 8) |
			  (PCIE_CONF_REVISION_ID) ),
			PCI_RC_RID_CC_ADR);

	rzv2m_write_conf(pcie,
			( (PCIE_CONF_SUBORDINATE_BUS << 16) |
			  (PCIE_CONF_SECOUNDARY_BUS  <<  8) |
			  (PCIE_CONF_PRIMARY_BUS) ),
			PCI_PRIMARY_BUS);

	rzv2m_write_conf(pcie,
			( (PCIE_CONF_MEMORY_LIMIT << 16) | 
			  (PCIE_CONF_MEMORY_BASE) ),
			PCI_MEMORY_BASE);

	rzv2m_write_conf(pcie, PCIE_CONF_BAR0_MASK_LO, PCIE_CONF_OFFSET_BAR0_MASK_LO);
	rzv2m_write_conf(pcie, PCIE_CONF_BAR0_MASK_UP, PCIE_CONF_OFFSET_BAR0_MASK_UP);
}

static int PCIE_CFG_Initialize(struct rzv2m_pcie *pcie)
{
	// Vendor and Device ID      : PCI Express Configuration Registers Adr 6000h
	rzv2m_write_conf(pcie,
			( (PCIE_CONF_DEVICE_ID << 16) |
			  (PCIE_CONF_VENDOR_ID) ),
			PCI_RC_VID_ADR);

	// Revision ID and Class Code : PCI Express Configuration Registers Adr 6008h
	rzv2m_write_conf(pcie,
			( (PCIE_CONF_BASE_CLASS   << 24) |
			  (PCIE_CONF_SUB_CLASS    << 16) |
			  (PCIE_CONF_PROGRAMING_IF << 8) |
			  (PCIE_CONF_REVISION_ID) ),
			PCI_RC_RID_CC_ADR);

	// Base Address Register Mask00 (Lower) (Function #1) : PCI Express Configuration Registers Adr 60A0h
	rzv2m_write_conf(pcie, BASEADR_MKL_ALLM, PCI_RC_BARMSK00L_ADR);

	// Base Address Register Mask00 (upper) (Function #1) : PCI Express Configuration Registers Adr 60A4h
	rzv2m_write_conf(pcie, BASEADR_MKU_ALLM, PCI_RC_BARMSK00U_ADR);

	// Base Size 00/01 : PCI Express Configuration Registers Adr 60C8h
	rzv2m_write_conf(pcie, BASESZ_INIT, PCI_RC_BSIZE00_01_ADR);

	// Bus Number : PCI Express Configuration Registers Adr 6018h
	rzv2m_write_conf(pcie,
			( (PCIE_CONF_SUBORDINATE_BUS << 16) |
			  (PCIE_CONF_SECOUNDARY_BUS  <<  8) |
			  (PCIE_CONF_PRIMARY_BUS) ),
			PCI_PRIMARY_BUS);

	rzv2m_write_conf(pcie,
			( (PCIE_CONF_MEMORY_LIMIT << 16) |
			  (PCIE_CONF_MEMORY_BASE) ),
			PCI_MEMORY_BASE);

	rzv2m_write_conf(pcie, PM_CAPABILITIES_INIT, PCI_PM_CAPABILITIES);

  return 0;
}

static int rzv2m_pcie_hw_init(struct rzv2m_pcie *pcie)
{
	unsigned int timeout = 50;

	/* Set to the PCIe reset state   : step6 */
	rzv2m_pci_write_reg(pcie, RESET_ALL_ASSERT, PCI_RESET_REG);			/* Set PCI_RC 310h */

	/* Set PMA and Phy Register for Lane0 : step7, 9 */
	PCIE_phyInitialize_L0(pcie);

	/* Set PMA and Phy Register for Lane1 : step8, 9 */
	PCIE_phyInitialize_L1(pcie);

	/* Release the PCIe reset : step10 : RST_LOAD_B, RST_CFG_B)*/
	rzv2m_pci_write_reg(pcie, RESET_LOAD_CFG_RELEASE, PCI_RESET_REG);	/* Set PCI_RC 310h */

	/* Setting of HWINT related registers : step11 */
	PCIE_CFG_Initialize(pcie);

	/* Set L1 state                       : step12  */
	rzv2m_sys_write_reg(pcie, SET_ASPM_L1_ST, SYS_PCI_ALLOW_ENTER_L1_REG);		/* Set SYS 064h */

	/* Set Interrupt settings             : step13  */
	PCIE_INT_Initialize(pcie);

	/* Release the PCIe reset : step14 : RST_PS_B, RST_GP_B, RST_B */
	rzv2m_pci_write_reg(pcie, RESET_PS_GP_RELEASE, PCI_RESET_REG);		/* Set PCI_RC 310h */

   /* Wait 500us over : step 15*/
	msleep(1);

   /* Release the PCIe reset : step16 : RST_OUT_B, RST_RSM_B) */
	rzv2m_pci_write_reg(pcie, RESET_ALL_DEASSERT,  PCI_RESET_REG);		/* Set PCI_RC 310h */

	rzv2m_pci_write_reg(pcie, 0x3ff2,  MODE_SET_1_REG);						/* Set PCI_RC 318h */

	/* This will timeout if we don't have a link. */
	while (timeout--) {
		if (!(rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG) & DL_DOWN_STATUS))
			return 0;

		msleep(5);
	}

	return -ETIMEDOUT;
}

/* INTx Functions */

/**
 * rzv2m_pcie_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */

static int rzv2m_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			      irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* INTx IRQ Domain operations */
static const struct irq_domain_ops intx_domain_ops = {
	.map = rzv2m_pcie_intx_map,
};

static int rzv2m_msi_alloc(struct rzv2m_msi *chip)
{
	int msi;

	mutex_lock(&chip->lock);

	msi = find_first_zero_bit(chip->used, INT_PCI_MSI_NR);
	if (msi < INT_PCI_MSI_NR)
		set_bit(msi, chip->used);
	else
		msi = -ENOSPC;

	mutex_unlock(&chip->lock);

	return msi;
}

static int rzv2m_msi_alloc_region(struct rzv2m_msi *chip, int no_irqs)
{
	int msi;

	mutex_lock(&chip->lock);
	msi = bitmap_find_free_region(chip->used, INT_PCI_MSI_NR,
				      order_base_2(no_irqs));
	mutex_unlock(&chip->lock);

	return msi;
}

static void rzv2m_msi_free(struct rzv2m_msi *chip, unsigned long irq)
{
	mutex_lock(&chip->lock);
	bitmap_release_region(chip->used, irq, 1);
	clear_bit(irq, chip->used);
	mutex_unlock(&chip->lock);
}

static irqreturn_t rzv2m_pcie_msi_irq(int irq, void *data)
{
	struct rzv2m_pcie_host *host = data;
	struct rzv2m_pcie *pcie = &host->pcie;
	struct rzv2m_msi *msi = &host->msi;
	unsigned long reg;
	unsigned int irq_v;
	unsigned int i = 0;
	unsigned int hwirq;
	irqreturn_t ret = IRQ_NONE;

	reg = rzv2m_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_STATUS_REG);
	/* clear the interrupt */
	rzv2m_pci_write_reg(pcie, ALL_RECEIVE_INTERRUPT_STATUS, PCI_INTX_RCV_INTERRUPT_STATUS_REG);

	// MSI Only
	if (!(reg & MSI_RECEIVE_INTERRUPT_STATUS))
		return IRQ_NONE;

	for (i = 0; i < MSI_RCV_NUM; i++) {
		hwirq = *(unsigned int *)(msi->pages + i*0x4);
		if (hwirq != MSI_RCV_WINDOW_INVALID) {
			/* Invalidate MSI Window */
			*(unsigned int *)(msi->pages + i*0x4) = MSI_RCV_WINDOW_INVALID;
			irq_v = irq_find_mapping(msi->domain, hwirq);
			if (irq_v) {
				if (test_bit(hwirq, msi->used)) {
					generic_handle_irq(irq_v);
					ret = IRQ_HANDLED;
				} else
					dev_info(pcie->dev, "unhandled MSI\n");
			} else {
				/* Unknown MSI, just clear it */
				dev_dbg(pcie->dev, "unexpected MSI\n");
			}
		}
	}

	return ret;
}

static int rzv2m_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			      struct msi_desc *desc)
{
	struct rzv2m_msi *msi = to_rzv2m_msi(chip);
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;
	unsigned long msi_notice_addr;

	hwirq = rzv2m_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_find_mapping(msi->domain, hwirq);
	if (!irq) {
		rzv2m_msi_free(msi, hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	msi_notice_addr = (unsigned long)msi->virt_pages + (hwirq * sizeof(unsigned int));
	msg.address_lo = lower_32_bits(msi_notice_addr);
	msg.address_hi = upper_32_bits(msi_notice_addr);
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

static int rzv2m_msi_setup_irqs(struct msi_controller *chip,
			       struct pci_dev *pdev, int nvec, int type)
{
	struct rzv2m_msi *msi = to_rzv2m_msi(chip);
	struct msi_desc *desc;
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;
	int i;
	unsigned long msi_notice_addr;

	/* MSI-X interrupts are not supported */
	if (type == PCI_CAP_ID_MSIX)
		return -EINVAL;

	WARN_ON(!list_is_singular(&pdev->dev.msi_list));
	desc = list_entry(pdev->dev.msi_list.next, struct msi_desc, list);

	hwirq = rzv2m_msi_alloc_region(msi, nvec);
	if (hwirq < 0)
		return -ENOSPC;

	irq = irq_find_mapping(msi->domain, hwirq);
	if (!irq)
		return -ENOSPC;

	for (i = 0; i < nvec; i++) {
		/*
		 * irq_create_mapping() called from rzv2m_pcie_probe() pre-
		 * allocates descs,  so there is no need to allocate descs here.
		 * We can therefore assume that if irq_find_mapping() above
		 * returns non-zero, then the descs are also successfully
		 * allocated.
		 */
		if (irq_set_msi_desc_off(irq, i, desc)) {
			/* TODO: clear */
			return -EINVAL;
		}
	}

	desc->nvec_used = nvec;
	desc->msi_attrib.multiple = order_base_2(nvec);

	msi_notice_addr = (unsigned long)msi->virt_pages + (hwirq * sizeof(unsigned int));
	msg.address_lo = lower_32_bits(msi_notice_addr);
	msg.address_hi = upper_32_bits(msi_notice_addr);
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

static void rzv2m_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{
	struct rzv2m_msi *msi = to_rzv2m_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);

	rzv2m_msi_free(msi, d->hwirq);
}

static struct irq_chip rzv2m_msi_irq_chip = {
	.name = "RZV2MA PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static int rzv2m_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &rzv2m_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = rzv2m_msi_map,
};

static void rzv2m_pcie_unmap_msi(struct rzv2m_pcie_host *host)
{
	struct rzv2m_msi *msi = &host->msi;
	int i, irq;

	for (i = 0; i < INT_PCI_MSI_NR; i++) {
		irq = irq_find_mapping(msi->domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(msi->domain);
}

static void rzv2m_pcie_hw_enable_msi(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	struct device *dev = pcie->dev;
	struct rzv2m_msi *msi = &host->msi;
	unsigned long base;
	unsigned long pci_base;
	unsigned long msi_base;
	unsigned long msi_base_mask;
	int idx;

        msi->pages = ioremap(RAMA_ADDRESS, RAMA_SIZE);
        base = RAMA_ADDRESS;

	msi_base = 0;
	for(idx=0; idx < RZV2M_PCI_MAX_RESOURCES; idx++) {
		if( !(rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(idx)) & AXI_WINDOW_ENABLE) ) {
			continue;
		}
		pci_base = rzv2m_pci_read_reg(pcie, AXI_DESTINATION_REG(idx)) + 0x100000000*pcie->save_reg.axi_window.dest_u[idx];
		msi_base_mask = rzv2m_pci_read_reg(pcie, AXI_WINDOW_MASK_REG(idx));
		if( (pci_base <= base) && 
			(pci_base + msi_base_mask >= base) ) {

			msi_base  = base & msi_base_mask;
			msi_base |= rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(idx)) + 0x100000000*pcie->save_reg.axi_window.base_u[idx];
			msi->virt_pages = msi_base & ~AXI_WINDOW_ENABLE;
			msi_base |= MSI_RCV_WINDOW_ENABLE;
			break;
		}
	}
	if (!msi_base) {
		dev_err(dev,"MSI Address setting failed (Address:0x%lx)\n",base);
		goto err;
	}

	for (idx = 0; idx < MSI_RCV_NUM; idx++)
		*(unsigned int *)(msi->pages + idx*0x4) = MSI_RCV_WINDOW_INVALID;

	rzv2m_pci_write_reg(pcie, lower_32_bits(msi_base), MSI_RCV_WINDOW_ADDR_REG);
	rzv2m_pci_write_reg(pcie, MSI_RCV_WINDOW_MASK_MIN, MSI_RCV_WINDOW_MASK_REG);
	rzv2m_rmw(pcie, MSI_RCV_WINDOW_ADDR_REG, MSI_RCV_WINDOW_ENABLE, MSI_RCV_WINDOW_ENABLE);

	/* enable all MSI interrupts */
	rzv2m_rmw(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG,
					 MSI_RECEIVE_INTERRUPT_ENABLE,
					 MSI_RECEIVE_INTERRUPT_ENABLE );

	return;

err:
	rzv2m_pcie_unmap_msi(host);
}

static int rzv2m_pcie_enable_msi(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	struct device *dev = pcie->dev;
	struct rzv2m_msi *msi = &host->msi;
	int err, i;

	mutex_init(&msi->lock);

	host->intx_domain = irq_domain_add_linear(dev->of_node, PCI_NUM_INTX,
						  &intx_domain_ops,
						  pcie);

	if (!host->intx_domain) {
		dev_err(dev, "failed to create INTx IRQ domain\n");
	}

	for (i = 0; i < PCI_NUM_INTX; i++)
		irq_create_mapping(host->intx_domain, i);

	msi->chip.dev = dev;
	msi->chip.setup_irq = rzv2m_msi_setup_irq;
	msi->chip.teardown_irq = rzv2m_msi_teardown_irq;

	msi->domain = irq_domain_add_linear(dev->of_node, INT_PCI_MSI_NR,
					    &msi_domain_ops, &msi->chip);
	if (!msi->domain) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	for (i = 0; i < INT_PCI_MSI_NR; i++)
		irq_create_mapping(msi->domain, i);

	/* Two irqs are for MSI, but they are also used for non-MSI irqs */
	err = devm_request_irq(dev, msi->irq, rzv2m_pcie_msi_irq,
			       IRQF_SHARED | IRQF_NO_THREAD | IRQF_ONESHOT,
			       rzv2m_msi_irq_chip.name, host);
	if (err < 0) {
		dev_err(dev, "failed to request IRQ: %d\n", err);
		goto err;
	}

	/* setup MSI data target */
	rzv2m_pcie_hw_enable_msi(host);

	return 0;

err:
	rzv2m_pcie_unmap_msi(host);
	return err;
}

static int rzv2m_pcie_get_resources(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	struct device *dev = pcie->dev;
	struct resource res;
	int err, i;

	host->phy = devm_phy_optional_get(dev, "pcie");
	if (IS_ERR(host->phy))
		return PTR_ERR(host->phy);

	err = of_address_to_resource(dev->of_node, 0, &res);
	if (err)
		return err;

	pcie->base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);

	i = irq_of_parse_and_map(dev->of_node, 0);
	if (!i) {
		dev_err(dev, "cannot get platform resources for msi interrupt\n");
		err = -ENOENT;
		goto err_irq;
	}
	host->msi.irq = i;

err_irq:
	return err;
}

static int rzv2m_pcie_inbound_ranges(struct rzv2m_pcie *pcie,
				    struct resource_entry *entry,
				    int *index)
{
	u64 restype = entry->res->flags;
	u64 cpu_addr = entry->res->start;
	u64 cpu_end = entry->res->end;
	u64 pci_addr = entry->res->start - entry->offset;
	u32 flags = LAM_64BIT | LAR_ENABLE;
	u64 mask;
	u64 size = resource_size(entry->res);
	int idx = *index;

	if (restype & IORESOURCE_PREFETCH)
		flags |= LAM_PREFETCH;

	while (cpu_addr < cpu_end) {
		if (idx >= MAX_NR_INBOUND_MAPS - 1) {
			dev_err(pcie->dev, "Failed to map inbound regions!\n");
			return -EINVAL;
		}
		mask = size - 1;
		mask &= ~0xf;

		rzv2m_pcie_set_inbound(pcie, cpu_addr, pci_addr,
				      lower_32_bits(mask) | flags, idx, true);

		pci_addr += size;
		cpu_addr += size;
		idx += 2;
	}
	*index = idx;

	return 0;
}

static int rzv2m_pcie_parse_map_dma_ranges(struct rzv2m_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *entry;
	int index = 0, err = 0;

	resource_list_for_each_entry(entry, &bridge->dma_ranges) {
		err = rzv2m_pcie_inbound_ranges(&host->pcie, entry, &index);
		if (err)
			break;
	}

	return err;
}

static const struct of_device_id rzv2m_pcie_of_match[] = {
	{ .compatible = "renesas,rzv2ma-pcie", },
	{},
};

static int rzv2m_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzv2m_pcie_host *host;
	struct rzv2m_pcie *pcie;
	u32 data, data2;
	int err;
	struct pci_host_bridge *bridge;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*host));
	if (!bridge)
		return -ENOMEM;

	host = pci_host_bridge_priv(bridge);
	pcie = &host->pcie;
	pcie->dev = dev;
	platform_set_drvdata(pdev, host);

	pm_runtime_enable(pcie->dev);
	err = pm_runtime_get_sync(pcie->dev);
	if (err < 0) {
		dev_err(pcie->dev, "pm_runtime_get_sync failed\n");
		return err;
	}

	err = rzv2m_pcie_get_resources(host);
	if (err < 0) {
		dev_err(dev, "failed to request resources: %d\n", err);
		return err;
	}

	err = rzv2m_pcie_sys_get_resources(pcie);
	if (err < 0) {
		dev_err(dev, "failed to request pci sys resources: %d\n", err);
		return err;
	}

	err = rzv2m_pcie_phy_get_resources(pcie);
	if (err < 0) {
		dev_err(dev, "failed to request pci phy resources: %d\n", err);
		return err;
	}

	err = rzv2m_pcie_parse_map_dma_ranges(host);
	if (err)
		return err;

	rzv2m_pcie_setting_config(pcie);

	err = rzv2m_pcie_hw_init(pcie);
	if (err) {
		dev_info(&pdev->dev, "PCIe link down\n");
		return 0;
	}

	data = rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_2_REG);
	dev_info(&pdev->dev, "PCIe Linx status [0x%x]n", data);

    switch((data >> 8) & 0xFF)
	{
		case 0x01:
		case 0x02:
				/*- Detect Lane 0 or Lane 1 -*/
				data = 0x01;
				break;

		case 0x03:
				/*- Detect Lane 0 and Lane 1 -*/
				data = 0x02;
				break;

		default:
				/*- unknown -*/
				data = 0xff;
				break;
	}

	data2 = rzv2m_read_conf(pcie, PCI_RC_LINK_CONTROL_STATUS);
	switch((data2 >> 19) & 0xFF)
	{
		case 0x01:
				/*- setting Speed Gen1 -*/
				data2 = 0x01;
				break;
		case 0x02:
				/*- setting Speed Gen2 -*/
				data2 = 0x02;
				break;
		default:
				/*- unknown -*/
				data2 = 0xff;
				break;
	}
	dev_info(&pdev->dev, "PCIe : link up Lane number x%d / Speed Gen %d\n", data, data2);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = rzv2m_pcie_enable_msi(host);
		if (err < 0) {
			dev_err(dev,
				"failed to enable MSI support: %d\n",
				err);
			return err;
		}
	}

	return rzv2m_pcie_enable(host);
}

static int rzv2m_pcie_suspend(struct device *dev)
{
	struct rzv2m_pcie_host *host = dev_get_drvdata(dev);
	struct rzv2m_pcie *pcie = &host->pcie;
	int idx;

	for(idx=0; idx < RZV2M_PCI_MAX_RESOURCES; idx++) {
		/* Save AXI window setting	*/
		pcie->save_reg.axi_window.base[idx] = rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(idx));
		pcie->save_reg.axi_window.mask[idx] = rzv2m_pci_read_reg(pcie, AXI_WINDOW_MASK_REG(idx));
		pcie->save_reg.axi_window.dest[idx] = rzv2m_pci_read_reg(pcie, AXI_DESTINATION_REG(idx));

		/* Save PCIe window setting	*/
		pcie->save_reg.pci_window.base[idx]   = rzv2m_pci_read_reg(pcie, PCIE_WINDOW_BASE_REG(idx));
		pcie->save_reg.pci_window.mask[idx]   = rzv2m_pci_read_reg(pcie, PCIE_WINDOW_MASK_REG(idx));
		pcie->save_reg.pci_window.dest_u[idx] = rzv2m_pci_read_reg(pcie, PCIE_DESTINATION_HI_REG(idx));
		pcie->save_reg.pci_window.dest_l[idx] = rzv2m_pci_read_reg(pcie, PCIE_DESTINATION_LO_REG(idx));
	}
	/* Save MSI setting*/
	pcie->save_reg.interrupt.msi_win_addr	= rzv2m_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDR_REG);
	pcie->save_reg.interrupt.msi_win_mask	= rzv2m_pci_read_reg(pcie, MSI_RCV_WINDOW_MASK_REG);
	pcie->save_reg.interrupt.intx_ena	= rzv2m_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	pcie->save_reg.interrupt.msi_ena	= rzv2m_pci_read_reg(pcie, MSG_RCV_INTERRUPT_ENABLE_REG);

	return 0;
}

static int rzv2m_pcie_resume(struct device *dev)
{
	struct rzv2m_pcie_host *host = dev_get_drvdata(dev);
	struct rzv2m_pcie *pcie = &host->pcie;
	int idx, err;

	rzv2m_pcie_setting_config(pcie);

	if( rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(0)) != 
		pcie->save_reg.axi_window.base[0] ) {

		err = rzv2m_pcie_hw_init(pcie);
		if (err) {
			dev_info(pcie->dev, "resume PCIe link down\n");
			return err;
		}

		for(idx=0; idx < RZV2M_PCI_MAX_RESOURCES; idx++) {
			/* Restores AXI window setting	*/
			rzv2m_pci_write_reg(pcie, pcie->save_reg.axi_window.mask[idx], AXI_WINDOW_MASK_REG(idx));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.axi_window.dest[idx], AXI_DESTINATION_REG(idx));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.axi_window.base[idx], AXI_WINDOW_BASE_REG(idx));

			/* Restores PCIe window setting	*/
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.mask[idx], PCIE_WINDOW_MASK_REG(idx));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.dest_u[idx], PCIE_DESTINATION_HI_REG(idx));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.dest_l[idx], PCIE_DESTINATION_LO_REG(idx));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.base[idx], PCIE_WINDOW_BASE_REG(idx));

		}
		/* Restores MSI setting*/
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_win_mask, MSI_RCV_WINDOW_MASK_REG);
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_win_addr, MSI_RCV_WINDOW_ADDR_REG);
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.intx_ena, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_ena, MSG_RCV_INTERRUPT_ENABLE_REG);
	}

	return 0;
}

static struct dev_pm_ops rzv2m_pcie_pm_ops = {
	.suspend_noirq =	rzv2m_pcie_suspend,
	.resume_noirq =		rzv2m_pcie_resume,
};

static struct platform_driver rzv2m_pcie_driver = {
	.driver = {
		.name = "rzv2ma-pcie",
		.of_match_table = rzv2m_pcie_of_match,
		.pm = &rzv2m_pcie_pm_ops,
		.suppress_bind_attrs = true,
	},
	.probe = rzv2m_pcie_probe,
};
builtin_platform_driver(rzv2m_pcie_driver);

static int rzv2m_pcie_pci_notifier(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_BOUND_DRIVER:
		/* Force the DMA mask to lower 32-bits */
		dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static struct notifier_block device_nb = {
	.notifier_call = rzv2m_pcie_pci_notifier,
};

static int __init register_rzv2m_pcie_pci_notifier(void)
{
	return bus_register_notifier(&pci_bus_type, &device_nb);
}

arch_initcall(register_rzv2m_pcie_pci_notifier);
MODULE_AUTHOR("Phil Edworthy <phil.edworthy@renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/V2M Series PCIe driver");
MODULE_LICENSE("GPL v2");
