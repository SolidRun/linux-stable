// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe driver for Renesas RZ/V2M Series SoCs
 *  Copyright (C) 2022 Renesas Electronics Ltd
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
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/arm-smccc.h>
#include <uapi/linux/psci.h>

#include "pcie-rzv2h.h"

struct rzv2h_msi {
	DECLARE_BITMAP(used, INT_PCI_MSI_NR);
	struct irq_domain *domain;
	unsigned long pages;
	unsigned long virt_pages;
	struct mutex map_lock;
	spinlock_t mask_lock;
	int irq;
};

enum {
	RZV2_PCIE_THREAD_IDLE,
	RZV2_PCIE_THREAD_RESET
};

static int	pcie_thread_status;
static int	pcie_receiver_detection;
static struct	task_struct *pcie_kthread_tsk;
static struct	rzv2h_pcie *tmp_pcie;

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
	0x1B010003,
	0x00000000,
	0x00000000
};

/* Structure representing the PCIe interface */
struct rzv2h_pcie_host {
	struct rzv2h_pcie	pcie;
	struct device		*dev;
	struct phy		*phy;
	void __iomem		*base;
	struct clk		*bus_clk;
	struct			rzv2h_msi msi;
	int			(*phy_init_fn)(struct rzv2h_pcie_host *host);
	struct irq_domain	*intx_domain;
	struct reset_control    *rst;
	int			channel;
};

static struct rzv2h_pcie_host *msi_to_host(struct rzv2h_msi *msi)
{
	return container_of(msi, struct rzv2h_pcie_host, msi);
}

static int rzv2h_pcie_hw_init(struct rzv2h_pcie *pcie, int channel);

static int rzv2h_pcie_request_issue(struct rzv2h_pcie *pcie, struct pci_bus *bus)
{
	int i;
	u32 sts;

	rzv2h_rmw(pcie, REQUEST_ISSUE_REG, REQ_ISSUE, REQ_ISSUE);
	for (i = 0; i < STS_CHECK_LOOP; i++) {
		sts = rzv2h_pci_read_reg(pcie, REQUEST_ISSUE_REG);
		if (!(sts & REQ_ISSUE))
			break;

		udelay(5);
	}

	if (sts & MOR_STATUS) {
		dev_info(&bus->dev, "rzv2h_pcie_conf_access: Request failed(%d)\n", ((sts & MOR_STATUS)>>16));

		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int rzv2h_pcie_read_config_access(struct rzv2h_pcie_host *host,
		struct pci_bus *bus, unsigned int devfn, int where, u32 *data)
{
	struct rzv2h_pcie *pcie = &host->pcie;
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
	if (pci_is_root_bus(bus) && (devfn == 0)) {
		if (dev != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;

		if (reg == 0x10) {
			*data = r_configuration_space[0];
		} else if (reg == 0x14) {
			*data = r_configuration_space[1];
		} else if (reg == 0x20) {
			*data = r_configuration_space[2];
		} else if (reg == 0x40) {
			*data = r_configuration_space[3];
		} else if ((reg >= 0x50) && (reg <= 0x64)) {
			//MSI Capability register
			*data = r_msi_capability[(reg-0x50)/4];
		} else if ((reg >= 0x70) && (reg <= 0xA8)) {
			//PCI Express Capability register
			*data = rzv2h_read_conf(pcie, reg - 0x10);
		} else if ((reg >= 0xE0) && (reg <= 0xFC)) {
			//MSI-X Capability register
			*data = r_msi_and_msix_capability[(reg - 0xE0)/4];
		} else if ((reg >= 0x100) && (reg <= 0x118)) {
			*data = r_virtual_channel_enhanced_capability_header[(reg - 0x100) / 4];
		} else if ((reg >= 0x1B0) && (reg <= 0x1B8)) {
			*data = r_device_serial_number_capability[(reg - 0x1B0) / 4];
		} else {
			*data = rzv2h_read_conf(pcie, reg);
		}

		return PCIBIOS_SUCCESSFUL;
	}

	reg &= 0x0FFC;

	if (bus->number == 1) {
		/* Type 0 */
		rzv2h_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2h_pci_write_reg(pcie, TR_TYPE_CFREAD_TP0, REQUEST_ISSUE_REG);
	} else {
		/* Type 1 */
		rzv2h_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_DEV(dev) | PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2h_pci_write_reg(pcie, TR_TYPE_CFREAD_TP1, REQUEST_ISSUE_REG);
	}

	ret = rzv2h_pcie_request_issue(pcie, bus);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	*data = rzv2h_pci_read_reg(pcie, REQUEST_RCV_DATA_REG);

	return PCIBIOS_SUCCESSFUL;
}

static int rzv2h_pcie_write_config_access(struct rzv2h_pcie_host *host,
		struct pci_bus *bus, unsigned int devfn, int where, u32 data)
{
	struct rzv2h_pcie *pcie = &host->pcie;
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
	if (pci_is_root_bus(bus) && (devfn == 0)) {
		if (dev != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;

		if (reg == 0x20) {
			r_configuration_space[2] = data;
		} else if (reg == 0x40) {
			r_configuration_space[3] = data;
		} else if ((reg >= 0x50) && (reg <= 0x64)) {
			//MSI capability register
			r_msi_capability[(reg-0x50)/4] = data;
		} else if ((reg >= 0x70) && (reg <= 0xA8)) {
			//PCI Express Capability register
			rzv2h_write_conf(pcie, data, reg-0x10);
		} else if ((reg >= 0xE0) && (reg <= 0xFC)) {
			//MSI-X capability register
			r_msi_and_msix_capability[(reg - 0xE0) / 4] = data;
		} else if ((reg >= 0x100) && (reg <= 0x118)) {
			r_virtual_channel_enhanced_capability_header[(reg - 0x100) / 4] = data;
		} else if ((reg >= 0x1B0) && (reg <= 0x1B8)) {
			r_device_serial_number_capability[(reg - 0x1B0) / 4] = data;
		} else {
			rzv2h_write_conf(pcie, data, reg);
		}

		return PCIBIOS_SUCCESSFUL;
	}

	reg &= 0x0FFC;

	rzv2h_pci_write_reg(pcie, 0, REQUEST_DATA_REG(0));
	rzv2h_pci_write_reg(pcie, 0, REQUEST_DATA_REG(1));
	rzv2h_pci_write_reg(pcie, data, REQUEST_DATA_REG(2));

	if (bus->number == 1) {
		/* Type 0 */
		rzv2h_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2h_pci_write_reg(pcie, TR_TYPE_CFWRITE_TP0, REQUEST_ISSUE_REG);
	} else {
		/* Type 1 */
		rzv2h_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_DEV(dev) | PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2h_pci_write_reg(pcie, TR_TYPE_CFWRITE_TP1, REQUEST_ISSUE_REG);
	}

	return rzv2h_pcie_request_issue(pcie, bus);
}

static int rzv2h_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
			       int where, int size, u32 *val)
{
	struct rzv2h_pcie_host *host = bus->sysdata;
	int ret;

	if ((bus->number == 0) && (devfn >= 0x08) && (where == 0x0))
		return PCIBIOS_DEVICE_NOT_FOUND;

	ret = rzv2h_pcie_read_config_access(host, bus, devfn, where, val);
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

static int rzv2h_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val)
{
	struct rzv2h_pcie_host *host = bus->sysdata;
	unsigned int shift;
	u32 data;
	int ret;

	ret = rzv2h_pcie_read_config_access(host, bus, devfn, where, &data);
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

	ret = rzv2h_pcie_write_config_access(host, bus, devfn, where, data);

	return ret;
}

static struct pci_ops rzv2h_pcie_ops = {
	.read	= rzv2h_pcie_read_conf,
	.write	= rzv2h_pcie_write_conf,
};

static void rzv2h_pcie_force_speedup(struct rzv2h_pcie *pcie)
{
	struct device *dev = pcie->dev;
	unsigned int timeout = 1000;
	u32 pcstat2, pcctrl2, linkcs2;
	bool is_8_0gts = 0;

	/* Mask Target Link Speed to set it after found the maximum speed */
	linkcs2 = rzv2h_pci_read_reg(pcie, PCI_RC_LINKCS2);
	linkcs2 &= (~PCI_RC_LINKCS2_TARGET_LINK_SPEED_MASK);

	/* Check the maximum supported Link speed */
	pcstat2 = rzv2h_pci_read_reg(pcie, PCIE_CORE_STATUS_2_REG);
	switch (pcstat2 & PCIE_LINK_DATA_RATE) {
	case (PCIE_LINK_DATA_RATE_8_0GTS):
		is_8_0gts = 1;
		rzv2h_pci_write_reg(pcie, linkcs2 |
				    PCI_RC_LINKCS2_TARGET_LINK_SPEED_8_0GTS,
				    PCI_RC_LINKCS2);
		break;
	case (PCIE_LINK_DATA_RATE_5_0GTS):
		rzv2h_pci_write_reg(pcie, linkcs2 |
				    PCI_RC_LINKCS2_TARGET_LINK_SPEED_5_0GTS,
				    PCI_RC_LINKCS2);
		break;
	default:
		return;
	}

	/* Start to setup changing Link Speed:
	 * - Set speed change reason as intentional change
	 * - Set expected Link Speed
	 * - Enable Link Speed Change Request
	 */
	pcctrl2 = rzv2h_pci_read_reg(pcie, PCIE_CORE_CONTROL_2_REG);
	pcctrl2 |= PCIE_LINK_CHANGE_REASON_INTENTIONAL;
	pcctrl2 |= PCIE_LINK_SPEED_CHANGE_REQ;
	pcctrl2 &= (~PCIE_LINK_SPEED_CHANGE_MASK);
	pcctrl2 |= (is_8_0gts) ? PCIE_LINK_SPEED_CHANGE_8_0GTS :
		    PCIE_LINK_SPEED_CHANGE_5_0GTS;

	rzv2h_pci_write_reg(pcie, pcctrl2, PCIE_CORE_CONTROL_2_REG);

	/* Wait for Link Speed Changing Done */
	while (timeout--) {
		if (rzv2h_pci_read_reg(pcie, PCI_RC_PEIS0_REG) &
			INT_SPEED_CHANGE_DONE) {
			u32 linkcs;

			rzv2h_pci_write_reg(pcie, INT_SPEED_CHANGE_DONE,
						PCI_RC_PEIS0_REG);

			pcctrl2 = rzv2h_pci_read_reg(pcie,
						     PCIE_CORE_CONTROL_2_REG);
			pcctrl2 &= (~PCIE_LINK_SPEED_CHANGE_REQ);
			rzv2h_pci_write_reg(pcie, pcctrl2,
					    PCIE_CORE_CONTROL_2_REG);

			/* Confirm current link speed after changing */
			linkcs = rzv2h_pci_read_reg(pcie, PCI_RC_LINKCS);
			linkcs &= PCI_RC_LINKCS_CUR_LINK_SPEED_MASK;
			if (((is_8_0gts) &&
			     (linkcs == PCI_RC_LINKCS_CUR_LINK_SPEED_8_0GTS)) ||
			    ((!(is_8_0gts)) &&
			     (linkcs == PCI_RC_LINKCS_CUR_LINK_SPEED_5_0GTS))) {
				dev_info(dev, "Current link speed is %s GT/s\n",
					 is_8_0gts ? "8.0" : "5.0");
				return;
			}
		}

		udelay(100);
	};

	dev_err(dev, "Link Speed change failed\n");

	return;
};

static void rzv2h_pcie_hw_enable(struct rzv2h_pcie_host *host)
{
	struct rzv2h_pcie *pcie = &host->pcie;
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *win;
	LIST_HEAD(res);
	int i = 0;

	/* Try setting to maximum link speed */
	rzv2h_pcie_force_speedup(pcie);

	/* Setup PCI resources */
	resource_list_for_each_entry(win, &bridge->windows) {
		struct resource *res = win->res;

		if (!res->flags)
			continue;

		switch (resource_type(res)) {
		case IORESOURCE_IO:
		case IORESOURCE_MEM:
			rzv2h_pcie_set_outbound(pcie, i, win);
			i++;
			break;
		}
	}
}

static int rzv2h_pcie_enable(struct rzv2h_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);

	rzv2h_pcie_hw_enable(host);

	pci_add_flags(PCI_REASSIGN_ALL_BUS);

	bridge->sysdata = host;
	bridge->ops = &rzv2h_pcie_ops;

	return pci_host_probe(bridge);
}

static void rzv2h_pcie_setting_config(struct rzv2h_pcie *pcie)
{
	rzv2h_pci_write_reg(pcie, RESET_CONFIG_DEASSERT, PCI_RC_RESET_REG);

	// Configuration space(Root complex) setting
	// Vendor and Device ID      : PCI Express Configuration Registers Adr 6000h
	rzv2h_write_conf(pcie,
			((PCIE_CONF_DEVICE_ID << 16) |
			  (PCIE_CONF_VENDOR_ID)),
			PCI_RC_VID_ADR);

	// Revision ID and Class Code : PCI Express Configuration Registers Adr 6008h
	rzv2h_write_conf(pcie,
			((PCIE_CONF_BASE_CLASS << 24) |
			  (PCIE_CONF_SUB_CLASS << 16) |
			  (PCIE_CONF_PROGRAMING_IF << 8) |
			  (PCIE_CONF_REVISION_ID)),
			PCI_RC_RID_CC_ADR);

	rzv2h_write_conf(pcie,
			((PCIE_CONF_SUBORDINATE_BUS << 16) |
			  (PCIE_CONF_SECOUNDARY_BUS << 8) |
			  (PCIE_CONF_PRIMARY_BUS)),
			PCI_PRIMARY_BUS);

	rzv2h_write_conf(pcie,
			((PCIE_CONF_MEMORY_LIMIT << 16) |
			  (PCIE_CONF_MEMORY_BASE)),
			PCI_MEMORY_BASE);

	rzv2h_write_conf(pcie, PCIE_CONF_BAR0_MASK_LO, PCIE_CONF_OFFSET_BAR0_MASK_LO);
	rzv2h_write_conf(pcie, PCIE_CONF_BAR0_MASK_UP, PCIE_CONF_OFFSET_BAR0_MASK_UP);
}

static int PCIE_CFG_Initialize(struct rzv2h_pcie *pcie)
{
	// Vendor and Device ID: PCI Express Configuration Registers Adr 6000h
	rzv2h_write_conf(pcie,
			((PCIE_CONF_DEVICE_ID << 16) |
			  (PCIE_CONF_VENDOR_ID)),
			PCI_RC_VID_ADR);

	// Revision ID and Class Code: PCI Express Configuration Registers Adr 6008h
	rzv2h_write_conf(pcie,
			((PCIE_CONF_BASE_CLASS << 24) |
			  (PCIE_CONF_SUB_CLASS << 16) |
			  (PCIE_CONF_PROGRAMING_IF << 8) |
			  (PCIE_CONF_REVISION_ID)),
			PCI_RC_RID_CC_ADR);

	// Base Address Register Mask00 (Lower) (Function #1) : PCI Express Configuration Registers Adr 60A0h
	rzv2h_write_conf(pcie, BASEADR_MKL_ALLM, PCI_RC_BARMSK00L_ADR);

	// Base Address Register Mask00 (upper) (Function #1) : PCI Express Configuration Registers Adr 60A4h
	rzv2h_write_conf(pcie, BASEADR_MKU_ALLM, PCI_RC_BARMSK00U_ADR);

	// Base Size 00/01 : PCI Express Configuration Registers Adr 60C8h
	rzv2h_write_conf(pcie, BASESZ_INIT, PCI_RC_BSIZE00_01_ADR);

	// Bus Number : PCI Express Configuration Registers Adr 6018h
	rzv2h_write_conf(pcie,
			((PCIE_CONF_SUBORDINATE_BUS << 16) |
			  (PCIE_CONF_SECOUNDARY_BUS << 8) |
			  (PCIE_CONF_PRIMARY_BUS)),
			PCI_PRIMARY_BUS);

	rzv2h_write_conf(pcie,
			((PCIE_CONF_MEMORY_LIMIT << 16) |
			  (PCIE_CONF_MEMORY_BASE)),
			PCI_MEMORY_BASE);

	rzv2h_write_conf(pcie, PM_CAPABILITIES_INIT, PCI_PM_CAPABILITIES);

	return 0;
}

static int PCIE_INT_Initialize(struct rzv2h_pcie *pcie)
{
	/* Clear Event Interrupt Status 0 */
	rzv2h_pci_write_reg(pcie, INT_ST0_CLR, PCI_RC_PEIS0_REG);		/* Set PCI_RC 0204h */

	/* Set Event Interrupt Enable 0 */
	rzv2h_pci_write_reg(pcie, INT_EN0_SET, PCI_RC_PEIE0_REG);		/* Set PCI_RC 0200h */

	/* Clear  Event Interrupt Status 1 */
	rzv2h_pci_write_reg(pcie, INT_ST1_CLR, PCI_RC_PEIS1_REG);		/* Set PCI_RC 020ch */

	/* Set Event Interrupt Enable 1 */
	rzv2h_pci_write_reg(pcie, INT_EN1_SET, PCI_RC_PEIE1_REG);		/* Set PCI_RC 0208h */

	/* Clear AXI Master Error Interrupt Status */
	rzv2h_pci_write_reg(pcie, INT_ST_AXIM_CLR, PCI_RC_AMEIS_REG);	/* Set PCI_RC 0214h */

	/* Set AXI Master Error Interrupt Enable */
	rzv2h_pci_write_reg(pcie, INT_EN_AXIM_SET, PCI_RC_AMEIE_REG);	/* Set PCI_RC 0210h */

	/* Clear AXI Slave Error Interrupt Status */
	rzv2h_pci_write_reg(pcie, INT_ST_AXIS_CLR, PCI_RC_ASEIS1_REG);	/* Set PCI_RC 0224h */

	/* Set AXI Slave Error Interrupt Enable */
	rzv2h_pci_write_reg(pcie, INT_EN_AXIS_SET, PCI_RC_ASEIE1_REG);	/* Set PCI_RC 0220h */

	/* Clear Message Receive Interrupt Status */
	rzv2h_pci_write_reg(pcie, INT_MR_CLR, PCI_RC_MSGRCVIS_REG);		/* Set PCI_RC 0124h */

	/* Set Message Receive Interrupt Enable */
	rzv2h_pci_write_reg(pcie, INT_MR_SET, PCI_RC_MSGRCVIE_REG);		/* Set PCI_RC 0120h */

	return 0;
}

static int rzv2h_pcie_hw_init(struct rzv2h_pcie *pcie, int channel)
{
	unsigned int timeout = 50;
	struct arm_smccc_res local_res;

	/* Set to the PCIe reset state   : step6 */
	rzv2h_pci_write_reg(pcie, RESET_ALL_ASSERT, PCI_RC_RESET_REG);			/* Set PCI_RC 310h */

	/* Release the PCIe reset : step10 : RST_LOAD_B, RST_CFG_B)*/
	rzv2h_pci_write_reg(pcie, RESET_LOAD_CFG_RELEASE, PCI_RC_RESET_REG);	/* Set PCI_RC 310h */

	/* Setting of HWINT related registers : step11 */
	PCIE_CFG_Initialize(pcie);

	if (!channel)
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1020, 0x1, 0, 0, 0, 0, 0, &local_res);
	else
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1050, 0x1, 0, 0, 0, 0, 0, &local_res);

	/* Set Interrupt settings: step13  */
	PCIE_INT_Initialize(pcie);

	/* Release the PCIe reset : step14 : RST_PS_B, RST_GP_B, RST_B */
	rzv2h_pci_write_reg(pcie, RESET_PS_GP_RELEASE, PCI_RC_RESET_REG);		/* Set PCI_RC 310h */

	msleep(1);

	/* Release the PCIe reset : step16 : RST_OUT_B, RST_RSM_B) */
	rzv2h_pci_write_reg(pcie, RESET_ALL_DEASSERT,  PCI_RC_RESET_REG);		/* Set PCI_RC 310h */

	rzv2h_pci_write_reg(pcie, 0x3ff2, MODE_SET_1_REG);						/* Set PCI_RC 318h */

	/* This will timeout if we don't have a link. */
	while (timeout--) {
		if (!(rzv2h_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG) & DL_DOWN_STATUS))
			return 0;

		msleep(5);
	}

	return -ETIMEDOUT;
}

static void rzv2h_pcie_reset_assert(void)
{
	unsigned long reg;

	reg = rzv2h_pci_read_reg(tmp_pcie, PCI_RC_RESET_REG) & ~(RST_GP_B | RST_PS_B | RST_CFG_B | RST_B);
	rzv2h_pci_write_reg(tmp_pcie, reg,  PCI_RC_RESET_REG);
}

static void rzv2h_pcie_reset_deassert(void)
{
	rzv2h_rmw(tmp_pcie, PCI_RC_RESET_REG,
					 (RST_GP_B | RST_PS_B | RST_CFG_B | RST_B),
					 (RST_GP_B | RST_PS_B | RST_CFG_B | RST_B));
}

static int pcie_kthread(void *arg)
{
	unsigned long reg;
	unsigned long tmp_cnt = 0;
	unsigned int timeout = 50;

	while (!kthread_should_stop()) {
		if (pcie_thread_status == RZV2_PCIE_THREAD_RESET) {
			dev_info(tmp_pcie->dev, "PCIe link down\n");

			mdelay(1000);
			reg = rzv2h_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_1_REG);
			if ((reg & LTSSM_ST_ALL_MASK) == LTSSM_ST_DETECT) {
				reg = rzv2h_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_2_REG);

				if (((reg & STATE_RECEIVER_DETECTED) >> 8) == pcie_receiver_detection) {
					tmp_cnt++;
					if (tmp_cnt >= 3) {
						rzv2h_pcie_reset_assert();
						msleep(1);
						rzv2h_pcie_reset_deassert();

						tmp_cnt = 0;
						pcie_thread_status	= RZV2_PCIE_THREAD_IDLE;
						pcie_receiver_detection = 0x00;

						while (timeout--) {
							if (!(rzv2h_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_1_REG) & DL_DOWN_STATUS))
								break;

							msleep(5);
						}

						if (timeout) {
							reg = rzv2h_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_2_REG);
							dev_info(tmp_pcie->dev, "PCIe reset and Linx status [0x%lx]", reg);
						} else
							dev_err(tmp_pcie->dev, "PCIe reset and link down\n");
					}
				} else {
					pcie_thread_status	= RZV2_PCIE_THREAD_IDLE;
					pcie_receiver_detection = 0x00;
				}
			}
		} else
			mdelay(1000);
	}
	return 0;
}

static void rzv2h_pcie_enable_dl_updown(struct rzv2h_pcie_host *host)
{
	struct rzv2h_pcie *pcie = &host->pcie;

	pcie_thread_status	= RZV2_PCIE_THREAD_IDLE;
	pcie_receiver_detection = 0x00;

	pcie_kthread_tsk = kthread_run(pcie_kthread, NULL, "pcie kthread");
	if (IS_ERR(pcie_kthread_tsk))
		pr_err("pcie kthread run failed\n");
	else
		pr_info("pcie kthread pid:%d\n", pcie_kthread_tsk->pid);

	/* enable DL_UpDown interrupts */
	rzv2h_rmw(pcie, PCIE_EVENT_INTERRUPT_EANBLE_0_REG,
					 DL_UPDOWN_ENABLE,
					 DL_UPDOWN_ENABLE);

	return;
}

static void rzv2h_pcie_dl_updown(struct rzv2h_pcie_host *host)
{
	struct rzv2h_pcie *pcie = &host->pcie;
	unsigned long reg;

	reg = rzv2h_pci_read_reg(pcie, PCIE_EVENT_INTERRUPT_STATUS_0_REG);
	// clear the interrupt
	rzv2h_rmw(pcie, PCIE_EVENT_INTERRUPT_STATUS_0_REG,
					 DL_UPDOWN_STATUS,
					 DL_UPDOWN_STATUS);

	if (reg & DL_UPDOWN_STATUS) {
		// DL_UpDown interrupt
		tmp_pcie = &host->pcie;

		reg = rzv2h_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG);
		if (reg & DL_DOWN_STATUS) {
			// DL_Down_Status
			reg = rzv2h_pci_read_reg(pcie, PCIE_CORE_STATUS_2_REG);
			pcie_receiver_detection = (reg & STATE_RECEIVER_DETECTED) >> 8;

			pcie_thread_status = RZV2_PCIE_THREAD_RESET;
		}
	}
}

/* INTx Functions */

/**
 * rzv2h_pcie_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */

static int rzv2h_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			      irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* INTx IRQ Domain operations */
static const struct irq_domain_ops intx_domain_ops = {
	.map = rzv2h_pcie_intx_map,
};

static irqreturn_t rzv2h_pcie_msi_irq(int irq, void *data)
{
	struct rzv2h_pcie_host *host = data;
	struct rzv2h_pcie *pcie = &host->pcie;
	struct rzv2h_msi *msi = &host->msi;
	unsigned long reg, msi_stat;

	rzv2h_pcie_dl_updown(host);
	reg = rzv2h_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_STATUS_REG);

	msi_stat = rzv2h_pci_read_reg(pcie, PCI_RC_MSIRCVSTAT(0));
	if (!msi_stat)
		return IRQ_NONE;

	while (msi_stat) {
		unsigned int index = find_first_bit(&msi_stat, 32);
		unsigned int msi_irq;

		rzv2h_pci_write_reg(pcie, 1 << index, PCI_RC_MSIRCVSTAT(0));
		msi_irq = irq_find_mapping(msi->domain->parent, index);
		if (msi_irq) {
			generic_handle_irq(msi_irq);
		} else {
			/* Unknown MSI, just clear it */
			dev_dbg(pcie->dev, "unexpected MSI\n");
		}

		/* see if there's any more pending in this vector */
		msi_stat = rzv2h_pci_read_reg(pcie, PCI_RC_MSIRCVSTAT(0));
	}

	return IRQ_HANDLED;
}

static void rzv2h_msi_top_irq_ack(struct irq_data *d)
{
	irq_chip_ack_parent(d);
}

static void rzv2h_msi_top_irq_mask(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void rzv2h_msi_top_irq_unmask(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip rzv2h_msi_top_chip = {
	.name           = "PCIe MSI",
	.irq_ack        = rzv2h_msi_top_irq_ack,
	.irq_mask       = rzv2h_msi_top_irq_mask,
	.irq_unmask     = rzv2h_msi_top_irq_unmask,
};

static void rzv2h_msi_irq_ack(struct irq_data *d)
{
	struct rzv2h_msi *msi = irq_data_get_irq_chip_data(d);
	struct rzv2h_pcie *pcie = &msi_to_host(msi)->pcie;

	rzv2h_pci_write_reg(pcie, ALL_RECEIVE_INTERRUPT_STATUS, PCI_INTX_RCV_INTERRUPT_STATUS_REG);
}

static void rzv2h_msi_irq_mask(struct irq_data *d)
{
	struct rzv2h_msi *msi = irq_data_get_irq_chip_data(d);
	struct rzv2h_pcie *pcie = &msi_to_host(msi)->pcie;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&msi->mask_lock, flags);
	value = rzv2h_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	value &= ~BIT(d->hwirq);
	rzv2h_pci_write_reg(pcie, value, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	spin_unlock_irqrestore(&msi->mask_lock, flags);
}

static void rzv2h_msi_irq_unmask(struct irq_data *d)
{
	struct rzv2h_msi *msi = irq_data_get_irq_chip_data(d);
	struct rzv2h_pcie *pcie = &msi_to_host(msi)->pcie;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&msi->mask_lock, flags);
	value = rzv2h_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	value |= BIT(d->hwirq);
	rzv2h_pci_write_reg(pcie, value, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	spin_unlock_irqrestore(&msi->mask_lock, flags);
}

static int rzv2h_msi_set_affinity(struct irq_data *d, const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void rzv2h_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct rzv2h_msi *msi = irq_data_get_irq_chip_data(data);
	struct rzv2h_pcie *pcie = &msi_to_host(msi)->pcie;

	msg->address_lo = rzv2h_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDRL_REG) &
						(~PCIE_WINDOW_ENABLE);
	msg->address_hi = rzv2h_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDRU_REG);
	msg->data = data->hwirq;
}

static struct irq_chip rzv2h_msi_bottom_chip = {
	.name                   = "RZV2H MSI",
	.irq_ack                = rzv2h_msi_irq_ack,
	.irq_mask               = rzv2h_msi_irq_mask,
	.irq_unmask             = rzv2h_msi_irq_unmask,
	.irq_set_affinity       = rzv2h_msi_set_affinity,
	.irq_compose_msi_msg    = rzv2h_compose_msi_msg,
};

static int rzv2h_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *args)
{
	struct rzv2h_msi *msi = domain->host_data;
	unsigned int i;
	int hwirq;

	mutex_lock(&msi->map_lock);

	hwirq = bitmap_find_free_region(msi->used, INT_PCI_MSI_NR, order_base_2(nr_irqs));

	mutex_unlock(&msi->map_lock);

	if (hwirq < 0)
		return -ENOSPC;

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, hwirq + i,
				&rzv2h_msi_bottom_chip, domain->host_data,
				handle_edge_irq, NULL, NULL);

	return 0;
}

static void rzv2h_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct rzv2h_msi *msi = domain->host_data;

	mutex_lock(&msi->map_lock);

	bitmap_release_region(msi->used, d->hwirq, order_base_2(nr_irqs));

	mutex_unlock(&msi->map_lock);
}

static const struct irq_domain_ops rzv2h_msi_domain_ops = {
	.alloc  = rzv2h_msi_domain_alloc,
	.free   = rzv2h_msi_domain_free,
};

static struct msi_domain_info rzv2h_msi_info = {
	.flags  = (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
			MSI_FLAG_MULTI_PCI_MSI),
	.chip   = &rzv2h_msi_top_chip,
};

static int rzv2h_allocate_domains(struct rzv2h_msi *msi)
{
	struct rzv2h_pcie *pcie = &msi_to_host(msi)->pcie;
	struct fwnode_handle *fwnode = dev_fwnode(pcie->dev);
	struct irq_domain *parent;

	parent = irq_domain_create_linear(fwnode, INT_PCI_MSI_NR,
					&rzv2h_msi_domain_ops, msi);

	if (!parent) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}
	irq_domain_update_bus_token(parent, DOMAIN_BUS_NEXUS);

	msi->domain = pci_msi_create_irq_domain(fwnode, &rzv2h_msi_info, parent);
	if (!msi->domain) {
		dev_err(pcie->dev, "failed to create MSI domain\n");
		irq_domain_remove(parent);
		return -ENOMEM;
	}

	return 0;
}

static void rzv2h_free_domains(struct rzv2h_msi *msi)
{
	struct irq_domain *parent = msi->domain->parent;

	irq_domain_remove(msi->domain);
	irq_domain_remove(parent);
}

static void rzv2h_pcie_hw_enable_msi(struct rzv2h_pcie_host *host)
{
	struct rzv2h_pcie *pcie = &host->pcie;
	struct device *dev = pcie->dev;
	struct rzv2h_msi *msi = &host->msi;
	unsigned long base;
	unsigned long pci_base;
	unsigned long msi_base;
	unsigned long msi_base_mask;
	int idx;

	msi->pages = __get_free_pages(GFP_KERNEL | GFP_DMA, 0);
	base = dma_map_single(pcie->dev, (void *)msi->pages,
			      MSI_RCV_WINDOW_MASK_MIN + 1,
			      DMA_BIDIRECTIONAL);

	msi_base = 0;
	for (idx = 0; idx < MAX_NR_INBOUND_MAPS; idx++) {
		if (!(rzv2h_pci_read_reg(pcie, AXI_WINDOW_BASEL_REG(idx)) &
		      AXI_WINDOW_ENABLE)) {
			continue;
		}

		pci_base = rzv2h_pci_read_reg(pcie, AXI_DESTINATIONL_REG(idx));
		pci_base |= (((unsigned long) rzv2h_pci_read_reg(pcie, AXI_DESTINATIONU_REG(idx))) << 32);
		msi_base_mask = rzv2h_pci_read_reg(pcie, AXI_WINDOW_MASKL_REG(idx));
		msi_base_mask |= (((unsigned long) rzv2h_pci_read_reg(pcie, AXI_WINDOW_MASKU_REG(idx))) << 32);
		if ((pci_base <= base) && (pci_base + msi_base_mask >= base)) {

			msi_base  = base & msi_base_mask;
			msi_base |= rzv2h_pci_read_reg(pcie, AXI_WINDOW_BASEL_REG(idx));
			msi_base |= (((unsigned long) rzv2h_pci_read_reg(pcie, AXI_WINDOW_BASEU_REG(idx))) << 32);
			msi->virt_pages = msi_base & ~AXI_WINDOW_ENABLE;
			break;
		}
	}

	if (!msi_base) {
		dev_err(dev, "MSI Address setting failed (Address:0x%lx)\n", base);
		goto err;
	}

	rzv2h_pci_write_reg(pcie, lower_32_bits(msi_base),
			    MSI_RCV_WINDOW_ADDRL_REG);
	rzv2h_pci_write_reg(pcie, upper_32_bits(msi_base),
			    MSI_RCV_WINDOW_ADDRU_REG);
	rzv2h_pci_write_reg(pcie, MSI_RCV_WINDOW_MASK_MIN,
			    MSI_RCV_WINDOW_MASK_REG);
	rzv2h_rmw(pcie, MSI_RCV_WINDOW_ADDRL_REG, MSI_RCV_WINDOW_ENABLE,
		  MSI_RCV_WINDOW_ENABLE);

	/* Enable all MSI interrupts and MSI registers group */
	rzv2h_rmw(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG,
		  MSI_RECEIVE_INTERRUPT_ENABLE,
		  MSI_RECEIVE_INTERRUPT_ENABLE);

	rzv2h_pci_write_reg(pcie, PCI_RC_MSIRCVE_EN, PCI_RC_MSIRCVE(0));
	rzv2h_pci_write_reg(pcie, ~PCI_RC_MSIRCVMSK_MSI_MASK,
			    PCI_RC_MSIRCVMSK(0));

	return;

err:
	rzv2h_free_domains(&host->msi);

}

static int rzv2h_pcie_enable_msi(struct rzv2h_pcie_host *host)
{
	struct rzv2h_pcie *pcie = &host->pcie;
	struct device *dev = pcie->dev;
	struct rzv2h_msi *msi = &host->msi;
	struct resource res;
	int err;

	mutex_init(&msi->map_lock);
	spin_lock_init(&msi->mask_lock);

	err = of_address_to_resource(dev->of_node, 0, &res);
	if (err)
		return err;

	err = rzv2h_allocate_domains(msi);
	if (err)
		return err;


	/* Two irqs are for MSI, but they are also used for non-MSI irqs */
	err = devm_request_irq(dev, msi->irq, rzv2h_pcie_msi_irq,
	/* Temporarily set only shared IRQ flag */
			       IRQF_SHARED,
			       rzv2h_msi_bottom_chip.name, host);
	if (err < 0) {
		dev_err(dev, "failed to request IRQ: %d\n", err);
		goto err;
	}

	/* setup MSI data target */
	rzv2h_pcie_hw_enable_msi(host);

	return 0;

err:
	rzv2h_free_domains(msi);
	return err;
}

static int rzv2h_pcie_get_resources(struct rzv2h_pcie_host *host)
{
	struct rzv2h_pcie *pcie = &host->pcie;
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

static int rzv2h_pcie_inbound_ranges(struct rzv2h_pcie *pcie,
				    struct resource_entry *entry,
				    int *index)
{
	u64 cpu_addr = entry->res->start;
	u64 cpu_end = entry->res->end;
	u64 pci_addr = entry->res->start - entry->offset;
	u64 mask;
	u64 size = resource_size(entry->res);
	u64 size_idx = 0;
	int idx = *index;

	while (cpu_addr < cpu_end) {
		if (idx >= MAX_NR_INBOUND_MAPS - 1) {
			dev_err(pcie->dev, "Failed to map inbound regions!\n");
			return -EINVAL;
		}

		size = resource_size(entry->res) - size_idx;

		/*
		 * If the size of the range is larger than the alignment of
		 * the start address, we have to use multiple entries to
		 * perform the mapping.
		 */
		if (cpu_addr > 0) {
			unsigned long nr_zeros = __ffs64(cpu_addr);
			u64 alignment = 1ULL << nr_zeros;

			size = min(size, alignment);
		}

		/* Supports max 4GiB inbound region for each window */
		size = min(size, 1ULL << 32);
		mask = roundup_pow_of_two(size) - 1;
		mask |= 0xfff;

		rzv2h_pcie_set_inbound(pcie, cpu_addr, pci_addr,
				       mask, idx, true);

		pci_addr += size;
		cpu_addr += size;
		size_idx = size;
		idx++;
	}
	*index = idx;

	return 0;
}

static int rzv2h_pcie_parse_map_dma_ranges(struct rzv2h_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *entry;
	int index = 0, err = 0;

	resource_list_for_each_entry(entry, &bridge->dma_ranges) {
		err = rzv2h_pcie_inbound_ranges(&host->pcie, entry, &index);
		if (err)
			break;
	}

	return err;
}

static const struct of_device_id rzv2h_pcie_of_match[] = {
	{ .compatible = "renesas,rzv2h-pcie", },
	{ .compatible = "renesas,rzg3e-pcie", },
	{ .compatible = "renesas,rzv2n-pcie", },
	{},
};

static int rzv2h_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzv2h_pcie_host *host;
	struct rzv2h_pcie *pcie;
	u32 data;
	int err, channel;
	struct pci_host_bridge *bridge;
	struct arm_smccc_res local_res;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*host));
	if (!bridge)
		return -ENOMEM;

	host = pci_host_bridge_priv(bridge);
	pcie = &host->pcie;
	pcie->dev = dev;
	platform_set_drvdata(pdev, host);

	err = of_property_read_u32(dev->of_node, "pcie,channel", &channel);
	if (err) {
		dev_err(pcie->dev, "%pOF: No pcie,channel property found\n",
			dev->of_node);
		return -EINVAL;
	}
	host->channel = channel;

	if (host->channel >= PCIE_MAX_CHANNEL) {
		dev_err(pcie->dev, "%pOF: Invalid pcie,channel '%u'\n",
				dev->of_node, host->channel);
		return -EINVAL;
	}

	/* Set the Root Complex mode by the PCI Device Type setting register */
	if (!host->channel)
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1024, 0x1, 0, 0, 0, 0, 0, &local_res);
	else
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1054, 0x1, 0, 0, 0, 0, 0, &local_res);

	pm_runtime_enable(pcie->dev);
	err = pm_runtime_get_sync(pcie->dev);
	if (err < 0) {
		dev_err(pcie->dev, "pm_runtime_get_sync failed\n");
		return err;
	}

	err = rzv2h_pcie_get_resources(host);
	if (err < 0) {
		dev_err(dev, "failed to request resources: %d\n", err);
		return err;
	}

	host->rst = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(host->rst)) {
		dev_err(dev, "PCIE cannot get reset\n");
		return PTR_ERR(host->rst);
	}

	err = reset_control_deassert(host->rst);
	if (err) {
		dev_err(dev, "PCIE failed to deassert reset %d\n", err);
		return err;
	}

	udelay(200);

	err = rzv2h_pcie_parse_map_dma_ranges(host);
	if (err)
		return err;

	err = rzv2h_pcie_hw_init(pcie, host->channel);
	if (err) {
		dev_info(&pdev->dev, "PCIe link down\n");
		return 0;
	}

	data = rzv2h_pci_read_reg(pcie, PCI_RC_LINKCS);
	dev_info(&pdev->dev, "PCIe x%d: link up Lane number\n", (data >> 20) & 0x3F);

	rzv2h_pcie_enable_dl_updown(host);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = rzv2h_pcie_enable_msi(host);
		if (err < 0) {
			dev_err(dev,
				"failed to enable MSI support: %d\n",
				err);
			return err;
		}
	}

	return rzv2h_pcie_enable(host);
}

static int rzv2h_pcie_suspend(struct device *dev)
{
	struct rzv2h_pcie_host *host = dev_get_drvdata(dev);
	struct rzv2h_pcie *pcie = &host->pcie;
	int idx;

	for (idx = 0; idx < RZV2H_PCI_MAX_RESOURCES; idx++) {
		/* Save AXI window setting	*/
		pcie->save_reg.axi_window.base[idx] = rzv2h_pci_read_reg(pcie, AXI_WINDOW_BASEL_REG(idx));
		pcie->save_reg.axi_window.mask[idx] = rzv2h_pci_read_reg(pcie, AXI_WINDOW_MASKL_REG(idx));
		pcie->save_reg.axi_window.dest[idx] = rzv2h_pci_read_reg(pcie, AXI_DESTINATIONL_REG(idx));

		/* Save PCIe window setting	*/
		pcie->save_reg.pci_window.base[idx]   = rzv2h_pci_read_reg(pcie, PCIE_WINDOW_BASEL_REG(idx));
		pcie->save_reg.pci_window.mask[idx]   = rzv2h_pci_read_reg(pcie, PCIE_WINDOW_MASKL_REG(idx));
		pcie->save_reg.pci_window.dest_u[idx] = rzv2h_pci_read_reg(pcie, PCIE_DESTINATION_HI_REG(idx));
		pcie->save_reg.pci_window.dest_l[idx] = rzv2h_pci_read_reg(pcie, PCIE_DESTINATION_LO_REG(idx));
	}
	/* Save MSI setting*/
	pcie->save_reg.interrupt.msi_win_addrl	= rzv2h_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDRL_REG);
	pcie->save_reg.interrupt.msi_win_addru	= rzv2h_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDRU_REG);
	pcie->save_reg.interrupt.msi_win_mask	= rzv2h_pci_read_reg(pcie, MSI_RCV_WINDOW_MASK_REG);
	pcie->save_reg.interrupt.intx_ena	= rzv2h_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	pcie->save_reg.interrupt.msi_ena	= rzv2h_pci_read_reg(pcie, MSG_RCV_INTERRUPT_ENABLE_REG);

	return 0;
}

static int rzv2h_pcie_resume(struct device *dev)
{
	struct rzv2h_pcie_host *host = dev_get_drvdata(dev);
	struct rzv2h_pcie *pcie = &host->pcie;
	int idx, err;

	rzv2h_pcie_setting_config(pcie);

	if (rzv2h_pci_read_reg(pcie, AXI_WINDOW_BASEL_REG(0)) !=
		pcie->save_reg.axi_window.base[0]) {

		err = rzv2h_pcie_hw_init(pcie, host->channel);
		if (err) {
			dev_info(pcie->dev, "resume PCIe link down\n");
			return err;
		}

		for (idx = 0; idx < RZV2H_PCI_MAX_RESOURCES; idx++) {
			/* Restores AXI window setting	*/
			rzv2h_pci_write_reg(pcie, pcie->save_reg.axi_window.mask[idx], AXI_WINDOW_MASKL_REG(idx));
			rzv2h_pci_write_reg(pcie, pcie->save_reg.axi_window.dest[idx], AXI_DESTINATIONL_REG(idx));
			rzv2h_pci_write_reg(pcie, pcie->save_reg.axi_window.base[idx], AXI_WINDOW_BASEL_REG(idx));

			/* Restores PCIe window setting	*/
			rzv2h_pci_write_reg(pcie, pcie->save_reg.pci_window.mask[idx], PCIE_WINDOW_MASKL_REG(idx));
			rzv2h_pci_write_reg(pcie, pcie->save_reg.pci_window.dest_u[idx], PCIE_DESTINATION_HI_REG(idx));
			rzv2h_pci_write_reg(pcie, pcie->save_reg.pci_window.dest_l[idx], PCIE_DESTINATION_LO_REG(idx));
			rzv2h_pci_write_reg(pcie, pcie->save_reg.pci_window.base[idx], PCIE_WINDOW_BASEL_REG(idx));

		}
		/* Restores MSI setting*/
		rzv2h_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_win_mask, MSI_RCV_WINDOW_MASK_REG);
		pcie->save_reg.interrupt.msi_win_addrl	= rzv2h_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDRL_REG);
		pcie->save_reg.interrupt.msi_win_addru	= rzv2h_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDRU_REG);
		rzv2h_pci_write_reg(pcie, pcie->save_reg.interrupt.intx_ena, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
		rzv2h_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_ena, MSG_RCV_INTERRUPT_ENABLE_REG);
	}

	return 0;
}

static struct dev_pm_ops rzv2h_pcie_pm_ops = {
	.suspend_noirq =	rzv2h_pcie_suspend,
	.resume_noirq =		rzv2h_pcie_resume,
};

static struct platform_driver rzv2h_pcie_driver = {
	.driver = {
		.name = "rzv2h-pcie",
		.of_match_table = rzv2h_pcie_of_match,
		.pm = &rzv2h_pcie_pm_ops,
		.suppress_bind_attrs = true,
	},
	.probe = rzv2h_pcie_probe,
};
builtin_platform_driver(rzv2h_pcie_driver);

MODULE_DESCRIPTION("Renesas RZ/V2H Series PCIe driver");
MODULE_LICENSE("GPL v2");
