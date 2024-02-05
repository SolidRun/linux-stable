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

#include <linux/arm-smccc.h>
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
#include <linux/reset.h>
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

enum {
	RZV2_PCIE_THREAD_IDLE,
	RZV2_PCIE_THREAD_RESET
};

static int	pcie_thread_status;
static int	pcie_receiver_detection;
static struct task_struct *pcie_kthread_tsk;
static struct rzv2m_pcie *tmp_pcie;
struct rzv2m_pcie_host;

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

/* PCIE HW info data for specific SoC */
struct rzv2m_pcie_hw_info {
	bool 		is_sram_used;
	int		(*phy_init_fn)(struct rzv2m_pcie_host *);
	int		(*hw_init_fn)(struct rzv2m_pcie_host *);
	void		(*hw_enable_msi)(struct rzv2m_pcie_host *);
	void		(*set_inbound)(struct rzv2m_pcie *, u64 cpu_addr,
				       u64 pci_addr, u64 flags, int idx,
				       bool host, bool has_64bits_regs);
	irqreturn_t	(*irq_handler)(struct rzv2m_pcie_host *,
				       struct rzv2m_msi *);
	u32		flags;
	u32		irq_flags;
};

/* Structure representing the PCIe interface */
struct rzv2m_pcie_host {
	struct rzv2m_pcie		pcie;
	struct device			*dev;
	struct phy			*phy;
	void __iomem			*base;
	struct clk			*bus_clk;
	struct rzv2m_msi		msi;
	struct irq_domain		*intx_domain;
	const struct rzv2m_pcie_hw_info	*info;
	struct reset_control		*rst;
	bool				has_64bits_regs;
};

static int rzv2m_pcie_hw_init(struct rzv2m_pcie_host *host);

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
		dev_info(&bus->dev, "PCIE request issue failed, err%d\n",
			(sts & MOR_STATUS) >> 16);
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

		if (reg == 0x10) {
			*data = r_configuration_space[0];
		} else if (reg == 0x14) {
			*data = r_configuration_space[1];
		} else if (reg == 0x20) {
			*data = r_configuration_space[2];
		} else if (reg == 0x40) {
			*data = r_configuration_space[3];
		} else if ((reg >= 0x50) && (reg <= 0x64)) {
			/* MSI Capability register */
			*data = r_msi_capability[(reg - 0x50) / 4];
		} else if ((reg >= 0x70) && (reg <= 0xA8)) {
			/* PCI Express Capability register */
			*data = rzv2m_read_conf(pcie, reg - 0x10);
		} else if ((reg >= 0xE0) && (reg <= 0xFC)) {
			/* MSI-X Capability register */
			*data = r_msi_and_msix_capability[(reg - 0xE0) / 4];
		} else if ((reg >= 0x100) && (reg <= 0x118)) {
			*data = r_virtual_channel_enhanced_capability_header[(reg - 0x100) / 4];
		} else if ((reg >= 0x1B0) && (reg <= 0x1B8)) {
			*data = r_device_serial_number_capability[(reg-0x1B0)/4];
		} else {
			*data = rzv2m_read_conf(pcie, reg);
		}

		return PCIBIOS_SUCCESSFUL;
	}

	reg &= 0x0FFC;

	if (bus->number == 1) {
		/* Type 0 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFREAD_TP0,
				    REQUEST_ISSUE_REG);
	}
	else {
		/* Type 1 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
				    PCIE_CONF_DEV(dev) | PCIE_CONF_FUNC(func) |
				    reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFREAD_TP1,
				    REQUEST_ISSUE_REG);
	}

	ret = rzv2m_pcie_request_issue(pcie, bus);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	*data = rzv2m_pci_read_reg(pcie, REQUEST_RCV_DATA_REG);

	return PCIBIOS_SUCCESSFUL;
}

static int rzv2m_pcie_write_config_access(struct rzv2m_pcie_host *host,
					  struct pci_bus *bus,
					  unsigned int devfn, int where,
					  u32 data)
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

		if (reg == 0x20) {
			r_configuration_space[2] = data;
		} else if (reg == 0x40) {
			r_configuration_space[3] = data;
		} else if ((reg >= 0x50) && (reg <= 0x64)) {
			/* MSI capability register */
			r_msi_capability[(reg-0x50)/4] = data;
		} else if ((reg >= 0x70) && (reg <= 0xA8)) {
			/* PCI Express Capability register */
			rzv2m_write_conf(pcie, data, reg - 0x10);
		} else if ((reg >= 0xE0) && (reg <= 0xFC)) {
			/* MSI-X capability register */
			r_msi_and_msix_capability[(reg - 0xE0) / 4] = data;
		} else if ((reg >= 0x100) && (reg <= 0x118)) {
			r_virtual_channel_enhanced_capability_header[(reg - 0x100) / 4] = data;
		} else if ((reg >= 0x1B0) && (reg <= 0x1B8)) {
			r_device_serial_number_capability[(reg - 0x1B0) / 4] = data;
		} else {
			rzv2m_write_conf(pcie, data, reg);
		}

		return PCIBIOS_SUCCESSFUL;
	}

	reg &= 0x0FFC;

	rzv2m_pci_write_reg(pcie, 0, REQUEST_DATA_REG(0));
	rzv2m_pci_write_reg(pcie, 0, REQUEST_DATA_REG(1));
	if (host->info->flags & PCIE_SRAM_USE)
		rzv2m_pci_write_reg(pcie, (reg == 0x54) ? RAMA_ADDRESS : data,
				    REQUEST_DATA_REG(2));
	else
		rzv2m_pci_write_reg(pcie, data, REQUEST_DATA_REG(2));

	if (bus->number == 1) {
		/* Type 0 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
			PCIE_CONF_FUNC(func) | reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFWRITE_TP0,
				    REQUEST_ISSUE_REG);
	} else {
		/* Type 1 */
		rzv2m_pci_write_reg(pcie, PCIE_CONF_BUS(bus->number) |
				    PCIE_CONF_DEV(dev) | PCIE_CONF_FUNC(func) |
				    reg, REQUEST_ADDR_1_REG);
		rzv2m_pci_write_reg(pcie, TR_TYPE_CFWRITE_TP1,
				    REQUEST_ISSUE_REG);
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
	bool has_64bits_regs = host->has_64bits_regs;

	/* Setup PCI resources */
	resource_list_for_each_entry(win, &bridge->windows) {
		struct resource *res = win->res;

		if (!res->flags)
			continue;

		switch (resource_type(res)) {
		case IORESOURCE_IO:
		case IORESOURCE_MEM:
			rzv2m_pcie_set_outbound(pcie, i, win, has_64bits_regs);
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

	/* Configuration space(Root complex) setting */

	/* Vendor and Device ID*/
	rzv2m_write_conf(pcie,
			(PCIE_CONF_DEVICE_ID << 16) |
			 PCIE_CONF_VENDOR_ID,
			 PCI_RC_VID_ADR);

	/* Revision ID and Class Code */
	rzv2m_write_conf(pcie,
			(PCIE_CONF_BASE_CLASS << 24) |
			(PCIE_CONF_SUB_CLASS << 16) |
			(PCIE_CONF_PROGRAMING_IF << 8) |
			 PCIE_CONF_REVISION_ID,
			 PCI_RC_RID_CC_ADR);

	rzv2m_write_conf(pcie,
			(PCIE_CONF_SUBORDINATE_BUS << 16) |
			(PCIE_CONF_SECOUNDARY_BUS << 8) |
			 PCIE_CONF_PRIMARY_BUS,
			 PCI_PRIMARY_BUS);

	rzv2m_write_conf(pcie,
			(PCIE_CONF_MEMORY_LIMIT << 16) | 
			 PCIE_CONF_MEMORY_BASE,
			 PCI_MEMORY_BASE);

	rzv2m_write_conf(pcie, PCIE_CONF_BAR0_MASK_LO,
			 PCIE_CONF_OFFSET_BAR0_MASK_LO);
	rzv2m_write_conf(pcie, PCIE_CONF_BAR0_MASK_UP,
			 PCIE_CONF_OFFSET_BAR0_MASK_UP);
}

static int PCIE_CFG_Initialize(struct rzv2m_pcie *pcie)
{
	/* Vendor and Device ID */
	rzv2m_write_conf(pcie,
			(PCIE_CONF_DEVICE_ID << 16) | PCIE_CONF_VENDOR_ID,
			 PCI_RC_VID_ADR);

	/* Revision ID and Class Code */
	rzv2m_write_conf(pcie,
			(PCIE_CONF_BASE_CLASS << 24) |
			(PCIE_CONF_SUB_CLASS << 16) |
			(PCIE_CONF_PROGRAMING_IF << 8) |
			(PCIE_CONF_REVISION_ID),
			 PCI_RC_RID_CC_ADR);

	/* Base Address Register Mask00 (Lower) */
	rzv2m_write_conf(pcie, BASEADR_MKL_ALLM, PCI_RC_BARMSK00L_ADR);

	/* Base Address Register Mask00 (upper) (Function #1) */
	rzv2m_write_conf(pcie, BASEADR_MKU_ALLM, PCI_RC_BARMSK00U_ADR);

	/* Base Size 00/01 */
	rzv2m_write_conf(pcie, BASESZ_INIT, PCI_RC_BSIZE00_01_ADR);

	/* Bus Number */
	rzv2m_write_conf(pcie,
			(PCIE_CONF_SUBORDINATE_BUS << 16) |
			(PCIE_CONF_SECOUNDARY_BUS  <<  8) |
			 PCIE_CONF_PRIMARY_BUS,
			 PCI_PRIMARY_BUS);

	rzv2m_write_conf(pcie,
			(PCIE_CONF_MEMORY_LIMIT << 16) |
			 PCIE_CONF_MEMORY_BASE,
			 PCI_MEMORY_BASE);

	rzv2m_write_conf(pcie, PM_CAPABILITIES_INIT, PCI_PM_CAPABILITIES);

	return 0;
}


static void rzv2m_pcie_reset_assert(void)
{
	unsigned long reg;

	reg = rzv2m_pci_read_reg(tmp_pcie, PCI_RESET_REG) & ~(RST_GP_B | RST_PS_B | RST_CFG_B | RST_B);
	rzv2m_pci_write_reg(tmp_pcie, reg,  PCI_RESET_REG);
}

static void rzv2m_pcie_reset_deassert(void)
{
	rzv2m_rmw(tmp_pcie, PCI_RESET_REG,
					 (RST_GP_B | RST_PS_B | RST_CFG_B | RST_B),
					 (RST_GP_B | RST_PS_B | RST_CFG_B | RST_B) );
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
			reg = rzv2m_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_1_REG);
			if ( (reg & LTSSM_ST_ALL_MASK) == LTSSM_ST_DETECT) {
				reg = rzv2m_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_2_REG);

				if (((reg & STATE_RECEIVER_DETECTED) >> 8) == pcie_receiver_detection) {
					tmp_cnt++;
					if (tmp_cnt >= 3) {
						rzv2m_pcie_reset_assert();
						msleep(1);
						rzv2m_pcie_reset_deassert();

						tmp_cnt = 0;
						pcie_thread_status	= RZV2_PCIE_THREAD_IDLE;
						pcie_receiver_detection = 0x00;

						while (timeout--) {
							if (!(rzv2m_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_1_REG) & DL_DOWN_STATUS))
								break;

							msleep(5);
						}

						if (timeout) {
							reg = rzv2m_pci_read_reg(tmp_pcie, PCIE_CORE_STATUS_2_REG);
							dev_info(tmp_pcie->dev, "PCIe reset and Linx status [0x%lx]", reg);
						}
						else {
							dev_err(tmp_pcie->dev, "PCIe reset and link down\n");
						}
					}
				}
				else {
					pcie_thread_status	= RZV2_PCIE_THREAD_IDLE;
					pcie_receiver_detection = 0x00;
				}
			}
		}
		else {
			mdelay(1000);
		}
	}
    return 0;
}

static void rzv2m_pcie_enable_dl_updown(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;

	pcie_thread_status	= RZV2_PCIE_THREAD_IDLE;
	pcie_receiver_detection = 0x00;

	pcie_kthread_tsk = kthread_run(pcie_kthread, NULL, "pcie kthread");
	if (IS_ERR(pcie_kthread_tsk)) {
		pr_err("pcie kthread run failed\n");
	}
    else {
		pr_info("pcie kthread pid:%d\n", pcie_kthread_tsk->pid);
	}

	/* enable DL_UpDown interrupts */
	rzv2m_rmw(pcie, PCIE_EVENT_INTERRUPT_EANBLE_0_REG,
					 DL_UPDOWN_ENABLE,
					 DL_UPDOWN_ENABLE );

	return;
}

static void rzv2m_pcie_dl_updown(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	unsigned long reg;

	reg = rzv2m_pci_read_reg(pcie, PCIE_EVENT_INTERRUPT_STATUS_0_REG);
	// clear the interrupt
	rzv2m_rmw(pcie, PCIE_EVENT_INTERRUPT_STATUS_0_REG,
					 DL_UPDOWN_STATUS,
					 DL_UPDOWN_STATUS );

	if (reg & DL_UPDOWN_STATUS) {
		// DL_UpDown interrupt
		tmp_pcie = &host->pcie;

		reg = rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG);
		if (reg & DL_DOWN_STATUS) {
			// DL_Down_Status
			reg = rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_2_REG);
			pcie_receiver_detection = (reg & STATE_RECEIVER_DETECTED) >> 8;

			pcie_thread_status = RZV2_PCIE_THREAD_RESET;
		}
	}
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
	irqreturn_t ret = IRQ_NONE;

	rzv2m_pcie_dl_updown(host);

	reg = rzv2m_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_STATUS_REG);
	/* clear the interrupt */
	rzv2m_pci_write_reg(pcie, ALL_RECEIVE_INTERRUPT_STATUS,
			    PCI_INTX_RCV_INTERRUPT_STATUS_REG);

	/* MSI Only */
	if (!(reg & MSI_RECEIVE_INTERRUPT_STATUS))
		return ret;

	ret = host->info->irq_handler(host, msi);

	return ret;
}

static int rzv2m_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			      struct msi_desc *desc)
{
	struct rzv2m_msi *msi = to_rzv2m_msi(chip);
	struct rzv2m_pcie_host *host = container_of(chip,
						    struct rzv2m_pcie_host,
						    msi.chip);
	struct rzv2m_pcie *pcie = &host->pcie;
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;

	hwirq = rzv2m_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_find_mapping(msi->domain, hwirq);
	if (!irq) {
		rzv2m_msi_free(msi, hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	msg.address_lo = rzv2m_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDR_REG) &
			 (~PCIE_WINDOW_ENABLE);
	msg.address_hi = rzv2m_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDR_REG + 4);
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

static int rzv2m_msi_setup_irqs(struct msi_controller *chip,
			       struct pci_dev *pdev, int nvec, int type)
{
	struct rzv2m_msi *msi = to_rzv2m_msi(chip);
	struct rzv2m_pcie_host *host = container_of(chip,
						    struct rzv2m_pcie_host,
						    msi.chip);
	struct rzv2m_pcie *pcie = &host->pcie;
	struct msi_desc *desc;
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;
	int i;

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

	msg.address_lo = rzv2m_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDR_REG) &
			 (~PCIE_WINDOW_ENABLE);
	msg.address_hi = rzv2m_pci_read_reg(pcie, MSI_RCV_WINDOW_ADDR_REG + 4);
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
	.name = "RZV2M PCIe MSI",
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
	msi->chip.setup_irqs = rzv2m_msi_setup_irqs;
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
			       host->info->irq_flags,
			       rzv2m_msi_irq_chip.name, host);
	if (err < 0) {
		dev_err(dev, "failed to request IRQ: %d\n", err);
		goto err;
	}

	/* setup MSI data target */
	if (host->info->hw_enable_msi)
		host->info->hw_enable_msi(host);
	else
		return -ENOENT;

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

static int rzv2m_pcie_inbound_ranges(struct rzv2m_pcie_host *host,
				    struct resource_entry *entry,
				    int *index, bool has_64bits_regs)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	u64 cpu_addr = entry->res->start;
	u64 cpu_end = entry->res->end;
	u64 pci_addr = entry->res->start - entry->offset;
	u64 mask;
	u64 size = resource_size(entry->res);
	int idx = *index;

	while (cpu_addr < cpu_end) {
		if (idx >= MAX_NR_INBOUND_MAPS - 1) {
			dev_err(pcie->dev, "Failed to map inbound regions!\n");
			return -EINVAL;
		}
		mask = size - 1;
		mask &= ~0xf;

		host->info->set_inbound(pcie, cpu_addr, pci_addr,
					lower_32_bits(mask), idx, true,
					has_64bits_regs);

		pci_addr += size;
		cpu_addr += size;
		idx++;
	}
	*index = idx;

	return 0;
}

static int rzv2m_pcie_parse_map_dma_ranges(struct rzv2m_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *entry;
	int index = 0, err = 0;
	bool has_64bits_regs = host->has_64bits_regs;

	resource_list_for_each_entry(entry, &bridge->dma_ranges) {
		err = rzv2m_pcie_inbound_ranges(host, entry, &index,
						has_64bits_regs);
		if (err)
			break;
	}

	return err;
}

static int rzv2m_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzv2m_pcie_host *host;
	struct rzv2m_pcie *pcie;
	u32 reg;
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


	host->info = of_device_get_match_data(&pdev->dev);

	err = rzv2m_pcie_get_resources(host);
	if (err < 0) {
		dev_err(dev, "Failed to get PCIE res, err %d\n", err);
		return err;
	}

	if (host->info->flags & PCIE_HAS_SYS_BASE) {
		err = rzv2m_pcie_sys_get_resources(pcie);
		if (err < 0) {
			dev_err(dev, "Failed to get PCIE SYS res, err %d\n",
				err);
			return err;
		}
	}

	if (host->info->flags & PCIE_HAS_PHY_BASE) {
		err = rzv2m_pcie_phy_get_resources(pcie);
		if (err < 0) {
			dev_err(dev, "Failed to get PCIE PHY res, err %d\n",
				err);
			return err;
		}
	}

	if (host->info->flags & PCIE_HAS_RST_CTRL) {
		host->rst = devm_reset_control_array_get(dev, 0, 0);
		if (IS_ERR(host->rst)) {
			dev_err(dev, "PCIE cannot get reset controller\n");
			return PTR_ERR(host->rst);
		}
	
		err = reset_control_deassert(host->rst);
		if (err) {
			dev_err(dev, "PCIE failed to deassert reset %d\n", err);
			return err;
		}
	}

	if (!host->info->irq_handler) {
		dev_err(dev, "Missing handler of PCIE IRQ\n");
		return -ENOENT;
	}

	if (!host->info->set_inbound) {
		dev_err(dev, "Missing PCIE inbound setting\n");
		return -ENOENT;
	}
	if (host->info->flags & PCIE_HAS_64BITS_ADDR_REG)
		host->has_64bits_regs = true;
		
	pm_runtime_enable(pcie->dev);
	err = pm_runtime_get_sync(pcie->dev);
	if (err < 0) {
		dev_err(pcie->dev, "pm_runtime_get_sync failed\n");
		return err;
	}

	err = rzv2m_pcie_parse_map_dma_ranges(host);
	if (err)
		return err;

	rzv2m_pcie_setting_config(pcie);

	if (host->info->phy_init_fn) {
		err = host->info->phy_init_fn(host);
		if (err) {
			dev_err(dev, "failed to init PCIe PHY\n");
			return err;
		}
	}

	if (host->info->hw_init_fn) {
		err = host->info->hw_init_fn(host);
		if (err) {
			dev_info(&pdev->dev, "PCIe link down\n");
			return err;
		}
	}

	reg = rzv2m_read_conf(pcie, PCI_RC_LINK_CONTROL_STATUS);
	dev_info(&pdev->dev, "PCIe : link up Lane number x%ld / Speed Gen %ld\n",
		 PCI_RC_LINK_CONTROL_STATUS_LINK_WIDTH(reg),
		 PCI_RC_LINK_CONTROL_STATUS_LINK_SPEED(reg));

	rzv2m_pcie_enable_dl_updown(host);

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
	bool has_64bits_regs = host->has_64bits_regs;

	for(idx=0; idx < RZV2M_PCI_MAX_RESOURCES; idx++) {
		/* Save AXI window setting	*/
		pcie->save_reg.axi_window.base[idx] = rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(idx, has_64bits_regs));
		pcie->save_reg.axi_window.mask[idx] = rzv2m_pci_read_reg(pcie, AXI_WINDOW_MASK_REG(idx, has_64bits_regs));
		pcie->save_reg.axi_window.dest[idx] = rzv2m_pci_read_reg(pcie, AXI_DESTINATION_REG(idx, has_64bits_regs));

		/* Save PCIe window setting	*/
		pcie->save_reg.pci_window.base[idx]   = rzv2m_pci_read_reg(pcie, PCIE_WINDOW_BASE_REG(idx, has_64bits_regs));
		pcie->save_reg.pci_window.mask[idx]   = rzv2m_pci_read_reg(pcie, PCIE_WINDOW_MASK_REG(idx, has_64bits_regs));
		pcie->save_reg.pci_window.dest_u[idx] = rzv2m_pci_read_reg(pcie, PCIE_DESTINATION_REG(idx, has_64bits_regs) + 0x4);
		pcie->save_reg.pci_window.dest_l[idx] = rzv2m_pci_read_reg(pcie, PCIE_DESTINATION_REG(idx, has_64bits_regs));
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
	bool has_64bits_regs = host->has_64bits_regs;

	rzv2m_pcie_setting_config(pcie);

	if( rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(0, has_64bits_regs)) != 
		pcie->save_reg.axi_window.base[0] ) {

		err = rzv2m_pcie_hw_init(host);
		if (err) {
			dev_info(pcie->dev, "resume PCIe link down\n");
			return err;
		}

		for(idx=0; idx < RZV2M_PCI_MAX_RESOURCES; idx++) {
			/* Restores AXI window setting	*/
			rzv2m_pci_write_reg(pcie, pcie->save_reg.axi_window.mask[idx], AXI_WINDOW_MASK_REG(idx, has_64bits_regs));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.axi_window.dest[idx], AXI_DESTINATION_REG(idx, has_64bits_regs));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.axi_window.base[idx], AXI_WINDOW_BASE_REG(idx, has_64bits_regs));

			/* Restores PCIe window setting	*/
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.mask[idx], PCIE_WINDOW_MASK_REG(idx, has_64bits_regs));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.dest_u[idx], PCIE_DESTINATION_REG(idx, has_64bits_regs) + 0x4);
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.dest_l[idx], PCIE_DESTINATION_REG(idx, has_64bits_regs));
			rzv2m_pci_write_reg(pcie, pcie->save_reg.pci_window.base[idx], PCIE_WINDOW_BASE_REG(idx, has_64bits_regs));

		}
		/* Restores MSI setting*/
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_win_mask, MSI_RCV_WINDOW_MASK_REG);
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_win_addr, MSI_RCV_WINDOW_ADDR_REG);
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.intx_ena, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
		rzv2m_pci_write_reg(pcie, pcie->save_reg.interrupt.msi_ena, MSG_RCV_INTERRUPT_ENABLE_REG);
	}

	return 0;
}

static int rzv2m_pcie_hw_init(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	u32 reg;
	unsigned int timeout = 50;

	/* Set to the PCIe reset state */
	rzv2m_pci_write_reg(pcie, RESET_ALL_ASSERT, PCI_RESET_REG);

	/* Set PMA and Phy Register for Lane0 */
	PCIE_phyInitialize_L0(pcie);

	/* Set PMA and Phy Register for Lane1 */
	PCIE_phyInitialize_L1(pcie);

	/* Release RST_LOAD_B, RST_CFG_B */
	rzv2m_pci_write_reg(pcie, RESET_LOAD_CFG_RELEASE, PCI_RESET_REG);

	/* Setting of HWINT related registers */
	PCIE_CFG_Initialize(pcie);

	/* Set L1 state */
	rzv2m_sys_write_reg(pcie, SET_ASPM_L1_ST, SYS_PCI_ALLOW_ENTER_L1_REG);

	/* Set Interrupt settings */
	PCIE_INT_Initialize(pcie);

	/* Release RST_PS_B, RST_GP_B, RST_B */
	rzv2m_pci_write_reg(pcie, RESET_PS_GP_RELEASE, PCI_RESET_REG);

	/* Wait 500us over*/
	msleep(1);

	/* Release RST_OUT_B, RST_RSM_B */
	rzv2m_pci_write_reg(pcie, RESET_ALL_DEASSERT,  PCI_RESET_REG);

	rzv2m_pci_write_reg(pcie, 0x3ff2,  MODE_SET_1_REG);

	/* This will timeout if we don't have a link. */
	while (timeout--) {
		reg = rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG);
		if (!(reg & DL_DOWN_STATUS))
			return 0;

		msleep(5);
	}

	return -ETIMEDOUT;
}

static void rzv2m_pcie_hw_enable_msi(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	struct device *dev = pcie->dev;
	struct rzv2m_msi *msi = &host->msi;
	unsigned long base;
	unsigned long pci_base;
	unsigned long msi_base = 0;
	unsigned long msi_base_mask;
	int idx;
	u32 reg;
	bool has_64bits_regs = host->has_64bits_regs;

	if (host->info->flags & PCIE_SRAM_USE) {
		msi->pages = ioremap(RAMA_ADDRESS, RAMA_SIZE);
		base = RAMA_ADDRESS;
	} else {
		msi->pages = __get_free_pages(GFP_KERNEL, 0);
		base = dma_map_single(pcie->dev, (void *)msi->pages,
				      MSI_RCV_WINDOW_MASK_MIN + 1,
				      DMA_BIDIRECTIONAL);
	}

	for(idx = 0; idx < RZV2M_PCI_MAX_RESOURCES; idx++) {
		reg = rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(idx, has_64bits_regs));
		if( !(reg & AXI_WINDOW_ENABLE))
			continue;

		pci_base = rzv2m_pci_read_reg(pcie, AXI_DESTINATION_REG(idx, has_64bits_regs)) + 0x100000000 * pcie->save_reg.axi_window.dest_u[idx];
		msi_base_mask = rzv2m_pci_read_reg(pcie, AXI_WINDOW_MASK_REG(idx, has_64bits_regs));
		if( (pci_base <= base) && 
			(pci_base + msi_base_mask >= base) ) {

			msi_base  = base & msi_base_mask;
			msi_base |= rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(idx, has_64bits_regs)) + 0x100000000 * pcie->save_reg.axi_window.base_u[idx];
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
		*(unsigned int *)(msi->pages + idx * 0x4) = MSI_RCV_WINDOW_INVALID;

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

static irqreturn_t rzv2m_pcie_irq_handler(struct rzv2m_pcie_host *host,
				   struct rzv2m_msi *msi)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	unsigned int hwirq, i, irq_v;
	irqreturn_t ret = IRQ_NONE;

	for (i = 0; i < MSI_RCV_NUM; i++) {
		hwirq = *(unsigned int *)(msi->pages + i * 0x4);
		if (hwirq != MSI_RCV_WINDOW_INVALID) {
			/* Invalidate MSI Window */
			*(unsigned int *)(msi->pages + i * 0x4) = MSI_RCV_WINDOW_INVALID;
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
};

static int rzg3s_pcie_phy_init(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;

	rzv2m_pci_write_reg(pcie, PIPE_PHY_REG_EN, PERMISSION_REG);

	/* PHY Control Pin (XCFGD) registers setting: 0x2000 - 0x2260 */
	rzv2m_pci_write_reg(pcie, 0, 0x2000);
	rzv2m_pci_write_reg(pcie, 0, 0x2010);
	rzv2m_pci_write_reg(pcie, 0, 0x2020);
	rzv2m_pci_write_reg(pcie, 0, 0x2030);
	rzv2m_pci_write_reg(pcie, 0, 0x2040);
	rzv2m_pci_write_reg(pcie, 0, 0x2050);
	rzv2m_pci_write_reg(pcie, 0, 0x2060);
	rzv2m_pci_write_reg(pcie, 0, 0x2070);
	rzv2m_pci_write_reg(pcie, 0xE0006801, 0x2080);
	rzv2m_pci_write_reg(pcie, 0x007F7E30, 0x2090);
	rzv2m_pci_write_reg(pcie, 0x183E0000, 0x20A0);
	rzv2m_pci_write_reg(pcie, 0x978FF500, 0x20B0);
	rzv2m_pci_write_reg(pcie, 0xEC000000, 0x20C0);
	rzv2m_pci_write_reg(pcie, 0x009F1400, 0x20D0);
	rzv2m_pci_write_reg(pcie, 0x0000D009, 0x20E0);
	rzv2m_pci_write_reg(pcie, 0, 0x20F0);
	rzv2m_pci_write_reg(pcie, 0, 0x2100);
	rzv2m_pci_write_reg(pcie, 0x78000000, 0x2110);
	rzv2m_pci_write_reg(pcie, 0, 0x2120);
	rzv2m_pci_write_reg(pcie, 0x00880000, 0x2130);
	rzv2m_pci_write_reg(pcie, 0x000005C0, 0x2140);
	rzv2m_pci_write_reg(pcie, 0x07000000, 0x2150);
	rzv2m_pci_write_reg(pcie, 0x00780920, 0x2160);
	rzv2m_pci_write_reg(pcie, 0xC9400CE2, 0x2170);
	rzv2m_pci_write_reg(pcie, 0x90000C0C, 0x2180);
	rzv2m_pci_write_reg(pcie, 0x000C1414, 0x2190);
	rzv2m_pci_write_reg(pcie, 0x00005034, 0x21A0);
	rzv2m_pci_write_reg(pcie, 0x00006000, 0x21B0);
	rzv2m_pci_write_reg(pcie, 0x00000001, 0x21C0);
	rzv2m_pci_write_reg(pcie, 0, 0x21D0);
	rzv2m_pci_write_reg(pcie, 0, 0x21E0);
	rzv2m_pci_write_reg(pcie, 0, 0x21F0);
	rzv2m_pci_write_reg(pcie, 0, 0x2220);
	rzv2m_pci_write_reg(pcie, 0, 0x2210);
	rzv2m_pci_write_reg(pcie, 0, 0x2220);
	rzv2m_pci_write_reg(pcie, 0, 0x2230);
	rzv2m_pci_write_reg(pcie, 0, 0x2240);
	rzv2m_pci_write_reg(pcie, 0, 0x2250);
	rzv2m_pci_write_reg(pcie, 0, 0x2260);

	/* PHY Control Pin (XCFGA CMN) registers setting: 0x2400 - 0x24F0*/
	rzv2m_pci_write_reg(pcie, 0x00000D10, 0x2400);
	rzv2m_pci_write_reg(pcie, 0x08310100, 0x2410);
	rzv2m_pci_write_reg(pcie, 0x00C21404, 0x2420);
	rzv2m_pci_write_reg(pcie, 0x013C0010, 0x2430);
	rzv2m_pci_write_reg(pcie, 0x01874440, 0x2440);
	rzv2m_pci_write_reg(pcie, 0x1A216082, 0x2450);
	rzv2m_pci_write_reg(pcie, 0x00103440, 0x2460);
	rzv2m_pci_write_reg(pcie, 0x00000080, 0x2470);
	rzv2m_pci_write_reg(pcie, 0x00000010, 0x2480);
	rzv2m_pci_write_reg(pcie, 0x0C1000C1, 0x2490);
	rzv2m_pci_write_reg(pcie, 0x1000C100, 0x24A0);
	rzv2m_pci_write_reg(pcie, 0x0222000C, 0x24B0);
	rzv2m_pci_write_reg(pcie, 0x00640019, 0x24C0);
	rzv2m_pci_write_reg(pcie, 0x00A00028, 0x24D0);
	rzv2m_pci_write_reg(pcie, 0x01D11228, 0x24E0);
	rzv2m_pci_write_reg(pcie, 0x0201001D, 0x24F0);

	/* PHY Control Pin (XCFGA RX) registers setting: 0x2500 - 0x25C0 */
	rzv2m_pci_write_reg(pcie, 0x07D55000, 0x2500);
	rzv2m_pci_write_reg(pcie, 0x030E3F00, 0x2510);
	rzv2m_pci_write_reg(pcie, 0x00000288, 0x2520);
	rzv2m_pci_write_reg(pcie, 0x102C5880, 0x2530);
	rzv2m_pci_write_reg(pcie, 0x0000000B, 0x2540);
	rzv2m_pci_write_reg(pcie, 0x04141441, 0x2550);
	rzv2m_pci_write_reg(pcie, 0x00641641, 0x2560);
	rzv2m_pci_write_reg(pcie, 0x00D63D63, 0x2570);
	rzv2m_pci_write_reg(pcie, 0x00641641, 0x2580);
	rzv2m_pci_write_reg(pcie, 0x01970377, 0x2590);
	rzv2m_pci_write_reg(pcie, 0x00190287, 0x25A0);
	rzv2m_pci_write_reg(pcie, 0x00190028, 0x25B0);
	rzv2m_pci_write_reg(pcie, 0x00000028, 0x25C0);

	/* PHY Control Pin (XCFGA TX) registers setting: 0x25D0 */
	rzv2m_pci_write_reg(pcie, 0x00000107, 0x25D0);

	/* PHY Control Pin (XCFG*) select registers setting: 0x2A20 */
	rzv2m_pci_write_reg(pcie, 0x00000001, 0x2A20);

	rzv2m_pci_write_reg(pcie, 0, PERMISSION_REG);

	return 0;
}

static int rzg3s_pcie_hw_init(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	struct arm_smccc_res local_res;
	unsigned int timeout = 50;
	u32 reg;

	/* Set RST_RSM_B after PCIe power is applied according to HW manual */
	arm_smccc_smc(RZG3S_SIP_SVC_SET_PCIE_RST_RSMB, RZG3S_SYS_PCIE_RST_RSM_B,
		      RZG3S_SYS_PCIE_RST_RSM_B_EN, 0, 0, 0, 0, 0, &local_res);

	/* Clear all PCIe reset bits */
	rzv2m_pci_write_reg(pcie, RESET_ALL_ASSERT, PCI_RESET_REG);
	udelay(100);

	/* Release RST_CFG_B, RST_LOAD_B, RST_OUT_B */
	rzv2m_pci_write_reg(pcie, RST_CFG_B | RST_LOAD_B | RST_OUT_B, PCI_RESET_REG);
	udelay(100);

	/* Setting of HWINT related registers */
	PCIE_CFG_Initialize(pcie);

	/* Set Interrupt settings */
	PCIE_INT_Initialize(pcie);

	/* Release RST_PS_B, RST_GP_B, RST_B */
	rzv2m_pci_write_reg(pcie,
			    RST_OUT_B | RST_PS_B | RST_LOAD_B |
			    RST_CFG_B | RST_GP_B | RST_B,
			    PCI_RESET_REG);
	udelay(100);

	/* Release all */
	rzv2m_pci_write_reg(pcie, RESET_ALL_DEASSERT, PCI_RESET_REG);

	/* This will timeout if we don't have a link. */
	while (timeout--) {
		reg = rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG);
		if (!(reg & DL_DOWN_STATUS))
			return 0;

		msleep(5);
	}

	return -ETIMEDOUT;
}

static void rzg3s_pcie_hw_enable_msi(struct rzv2m_pcie_host *host)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	struct device *dev = pcie->dev;
	struct rzv2m_msi *msi = &host->msi;
	unsigned long base;
	unsigned long pci_base;
	unsigned long msi_base = 0;
	unsigned long msi_base_mask;
	int idx;
	bool has_64bits_regs = host->has_64bits_regs;
	u32 reg;

	msi->pages = __get_free_pages(GFP_KERNEL | GFP_DMA, 0);
	base = dma_map_single(pcie->dev, (void *)msi->pages,
			     (MSI_RCV_WINDOW_MASK_MIN + 1), DMA_BIDIRECTIONAL);

	for (idx = 0; idx < MAX_NR_INBOUND_MAPS; idx++) {
		reg = rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASE_REG(idx, has_64bits_regs));
		if (!(reg & AXI_WINDOW_ENABLE)) {
			continue;
		}
		pci_base = rzv2m_pci_read_reg(pcie, AXI_DESTINATION_REG(idx, has_64bits_regs));
		pci_base |= ((unsigned long)rzv2m_pci_read_reg(pcie, AXI_DESTINATION_REG(idx, has_64bits_regs) + 0x4)) << 32;
		msi_base_mask = rzv2m_pci_read_reg(pcie, AXI_WINDOW_MASK_REG(idx, has_64bits_regs));
		if ((pci_base <= base) && (pci_base + msi_base_mask >= base)) {
			msi_base  = base & msi_base_mask;
			msi_base |= rzv2m_pci_read_reg(pcie, AXI_WINDOW_BASEL_REG(idx));
			msi->virt_pages = msi_base & ~AXI_WINDOW_ENABLE;
			break;
		}
	}
	if (!msi_base) {
		dev_err(dev, "MSI Address setting failed (Address:0x%lx)\n", base);
		goto err;
	}

	/* open MSI window */
	rzv2m_pci_write_reg(pcie, lower_32_bits(base), MSI_RCV_WINDOW_ADDR_REG);
	rzv2m_pci_write_reg(pcie, upper_32_bits(base), MSI_RCV_WINDOW_ADDR_REG + 0x4);
	rzv2m_pci_write_reg(pcie, MSI_RCV_WINDOW_MASK_MIN, MSI_RCV_WINDOW_MASK_REG);
	rzv2m_rmw(pcie, MSI_RCV_WINDOW_ADDR_REG, MSI_RCV_WINDOW_ENABLE, MSI_RCV_WINDOW_ENABLE);

	/* enable all MSI interrupts */
	rzv2m_rmw(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG,
					 MSI_RECEIVE_INTERRUPT_ENABLE,
					 MSI_RECEIVE_INTERRUPT_ENABLE);

	rzv2m_pci_write_reg(pcie, PCI_RC_MSIRCVE_EN, PCI_RC_MSIRCVE(0));
	rzv2m_pci_write_reg(pcie, ~PCI_RC_MSIRCVMSK_MSI_MASK, PCI_RC_MSIRCVMSK(0));

	return;

err:
	rzv2m_pcie_unmap_msi(host);
}

static irqreturn_t rzg3s_pcie_irq_handler(struct rzv2m_pcie_host *host,
				   struct rzv2m_msi *msi)
{
	struct rzv2m_pcie *pcie = &host->pcie;
	unsigned long msi_stat;

	/* Clear Message Receive Interrupt Status */
	rzv2m_pci_write_reg(pcie, INT_MR_CLR, PCI_RC_MSGRCVIS_REG);

	msi_stat = rzv2m_pci_read_reg(pcie, PCI_RC_MSIRCVSTAT(0));

	while (msi_stat) {
		unsigned int index = find_first_bit(&msi_stat, 32);
		unsigned int msi_irq;

		rzv2m_pci_write_reg(pcie, 1 << index, PCI_RC_MSIRCVSTAT(0));
		msi_irq = irq_find_mapping(msi->domain, index);
		if (msi_irq) {
			if (test_bit(index, msi->used))
				generic_handle_irq(msi_irq);
			else
				dev_info(pcie->dev, "unhandled MSI\n");
		} else {
			/* Unknown MSI, just clear it */
			dev_dbg(pcie->dev, "unexpected MSI\n");
		}

		/* see if there's any more pending in this vector */
		msi_stat = rzv2m_pci_read_reg(pcie, PCI_RC_MSIRCVSTAT(0));
	}

	return IRQ_HANDLED;
};

static const struct rzv2m_pcie_hw_info rzv2m_pcie_info = {
	.hw_init_fn = rzv2m_pcie_hw_init,
	.hw_enable_msi = rzv2m_pcie_hw_enable_msi,
	.flags = PCIE_HAS_SYS_BASE | PCIE_HAS_PHY_BASE,
	.irq_handler = rzv2m_pcie_irq_handler,
	.set_inbound = rzv2m_pcie_set_inbound,
	.irq_flags = IRQF_SHARED | IRQF_NO_THREAD | IRQF_ONESHOT,
};

static const struct rzv2m_pcie_hw_info rzv2ma_pcie_info = {
	.hw_init_fn = rzv2m_pcie_hw_init,
	.hw_enable_msi = rzv2m_pcie_hw_enable_msi,
	.flags = PCIE_SRAM_USE | PCIE_HAS_SYS_BASE | PCIE_HAS_PHY_BASE,
	.irq_handler = rzv2m_pcie_irq_handler,
	.set_inbound = rzv2m_pcie_set_inbound,
	.irq_flags = IRQF_SHARED | IRQF_NO_THREAD | IRQF_ONESHOT,
};

static const struct rzv2m_pcie_hw_info rzg3s_pcie_info = {
	.phy_init_fn = rzg3s_pcie_phy_init,
	.hw_init_fn = rzg3s_pcie_hw_init,
	.hw_enable_msi = rzg3s_pcie_hw_enable_msi,
	.flags = PCIE_HAS_RST_CTRL | PCIE_HAS_64BITS_ADDR_REG,
	.irq_handler = rzg3s_pcie_irq_handler,
	.set_inbound = rzg3s_pcie_set_inbound,
	.irq_flags = IRQF_SHARED,
};

static const struct of_device_id rzv2m_pcie_of_match[] = {
	{ .compatible = "renesas,rzv2m-pcie",	.data = &rzv2m_pcie_info,  },
	{ .compatible = "renesas,rzv2ma-pcie",	.data = &rzv2ma_pcie_info, },
	{ .compatible = "renesas,rzg3s-pcie",	.data = &rzg3s_pcie_info, },
	{},
};

static struct dev_pm_ops rzv2m_pcie_pm_ops = {
	.suspend_noirq =	rzv2m_pcie_suspend,
	.resume_noirq =		rzv2m_pcie_resume,
};

static struct platform_driver rzv2m_pcie_driver = {
	.driver = {
		.name = "rzv2m-pcie",
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
