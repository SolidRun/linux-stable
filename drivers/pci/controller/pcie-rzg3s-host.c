// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe driver for Renesas RZ/G3S SoCs
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 *
 * Based on:
 *  drivers/pci/controller/pcie-rcar-host.c
 *  Copyright (C) 2009 - 2011  Paul Mundt
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/units.h>

#include "../pci.h"

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

/**
 * struct rzg3s_pcie_msi - RZ/G3S PCIe MSI data structure
 * @domain: IRQ domain
 * @map: bitmap with the allocated MSIs
 * @dma_addr: address of the allocated MSI window
 * @window_base: base address of the MSI window
 * @pages: allocated pages for MSI window mapping
 * @map_lock: lock for bitmap with the allocated MSIs
 */
struct rzg3s_pcie_msi {
	struct irq_domain *domain;
	DECLARE_BITMAP(map, RZG3S_PCI_MSI_INT_NR);
	dma_addr_t dma_addr;
	dma_addr_t window_base;
	unsigned long pages;
	struct mutex map_lock;
};

struct rzg3s_pcie_host;

/**
 * struct rzg3s_pcie_soc_data - SoC specific data
 * @init_phy: PHY initialization function
 * @power_resets: array with the resets that need to be de-asserted after
 *                power-on
 * @cfg_resets: array with the resets that need to be de-asserted after
 *              configuration
 * @num_power_resets: number of power resets
 * @num_cfg_resets: number of configuration resets
 */
struct rzg3s_pcie_soc_data {
	int (*init_phy)(struct rzg3s_pcie_host *host);
	const char * const *power_resets;
	const char * const *cfg_resets;
	u8 num_power_resets;
	u8 num_cfg_resets;
};

/**
 * struct rzg3s_pcie_host - RZ/G3S PCIe data structure
 * @axi: base address for AXI registers
 * @pcie: base address for PCIe registers
 * @dev: struct device
 * @power_resets: reset control signals that should be set after power up
 * @cfg_resets: reset control signals that should be set after configuration
 * @sysc: SYSC regmap
 * @intx_domain: INTx IRQ domain
 * @data: SoC specific data
 * @msi: MSI data structure
 * @hw_lock: lock for access to the HW resources
 * @intx_irqs: INTx interrupts
 * @vendor_id: Vendor ID
 * @device_id: Device ID
 */
struct rzg3s_pcie_host {
	void __iomem *axi;
	void __iomem *pcie;
	struct device *dev;
	struct reset_control_bulk_data *power_resets;
	struct reset_control_bulk_data *cfg_resets;
	struct regmap *sysc;
	struct irq_domain *intx_domain;
	const struct rzg3s_pcie_soc_data *data;
	struct rzg3s_pcie_msi msi;
	raw_spinlock_t hw_lock;
	int intx_irqs[PCI_NUM_INTX];
	u32 vendor_id;
	u32 device_id;
};

#define rzg3s_msi_to_host(_msi)	container_of(_msi, struct rzg3s_pcie_host, msi)

static void rzg3s_pcie_update_bits(void __iomem *base, u32 offset, u32 mask,
				   u32 val)
{
	u32 tmp;

	tmp = readl(base + offset);
	tmp &= ~mask;
	tmp |= val & mask;
	writel(tmp, base + offset);
}

static bool rzg3s_pcie_child_issue_request(struct rzg3s_pcie_host *host)
{
	u32 val;
	int ret;

	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_REQISS,
			       RZG3S_PCI_REQISS_REQ_ISSUE,
			       RZG3S_PCI_REQISS_REQ_ISSUE);
	ret = readl_poll_timeout_atomic(host->axi + RZG3S_PCI_REQISS, val,
					!(val & RZG3S_PCI_REQISS_REQ_ISSUE),
					5, RZG3S_REQ_ISSUE_TIMEOUT_US);

	return !!ret || (val & RZG3S_PCI_REQISS_MOR_STATUS);
}

static int rzg3s_pcie_child_read_conf(struct rzg3s_pcie_host *host,
				      struct pci_bus *bus,
				      unsigned int devfn, int where,
				      u32 *data)
{
	int ret;

	bus->ops->map_bus(bus, devfn, where);

	/* Set the type of request */
	writel(RZG3S_PCI_REQISS_TR_TP0_RD, host->axi + RZG3S_PCI_REQISS);

	/* Issue the request and wait to finish */
	ret = rzg3s_pcie_child_issue_request(host);
	if (ret)
		return PCIBIOS_SET_FAILED;

	/* Read the data */
	*data = readl(host->axi + RZG3S_PCI_REQRCVDAT);

	return PCIBIOS_SUCCESSFUL;
}

/* Serialization is provided by 'pci_lock' in drivers/pci/access.c */
static int rzg3s_pcie_child_read(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 *val)
{
	struct rzg3s_pcie_host *host = bus->sysdata;
	int ret;

	ret = rzg3s_pcie_child_read_conf(host, bus, devfn, where, val);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	if (size <= 2)
		*val = (*val >> (8 * (where & 3))) & ((1 << (size * 8)) - 1);

	return PCIBIOS_SUCCESSFUL;
}

static int rzg3s_pcie_child_write_conf(struct rzg3s_pcie_host *host,
				       struct pci_bus *bus,
				       unsigned int devfn, int where,
				       u32 data)
{
	int ret;

	bus->ops->map_bus(bus, devfn, where);

	/* Set the write data  */
	writel(0, host->axi + RZG3S_PCI_REQDATA(0));
	writel(0, host->axi + RZG3S_PCI_REQDATA(1));
	writel(data, host->axi + RZG3S_PCI_REQDATA(2));

	/* Set the type of request */
	writel(RZG3S_PCI_REQISS_TR_TP0_WR, host->axi + RZG3S_PCI_REQISS);

	/* Issue the request and wait to finish */
	ret = rzg3s_pcie_child_issue_request(host);
	if (ret)
		return PCIBIOS_SET_FAILED;

	return PCIBIOS_SUCCESSFUL;
}

/* Serialization is provided by 'pci_lock' in drivers/pci/access.c */
static int rzg3s_pcie_child_write(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 val)
{
	struct rzg3s_pcie_host *host = bus->sysdata;
	u32 data, shift;
	int ret;

	if (size == 4)
		return rzg3s_pcie_child_write_conf(host, bus, devfn, where, val);

	/*
	 * Controller does 32 bit accesses. To do byte accesses software need
	 * to do read/modify/write. This may have potential side effects. For
	 * example, software may perform a 16-bit write. If the hardware only
	 * supports 32-bit accesses, we must do a 32-bit read, merge in the 16
	 * bits we intend to write, followed by a 32-bit write. If the 16 bits
	 * we *don't* intend to write happen to have any RW1C
	 * (write-one-to-clear) bits set, we just inadvertently cleared
	 * something we shouldn't have.
	 */
	if (!bus->unsafe_warn) {
		dev_warn(&bus->dev, "%d-byte config write to %04x:%02x:%02x.%d offset %#x may corrupt adjacent RW1C bits\n",
			 size, pci_domain_nr(bus), bus->number,
			 PCI_SLOT(devfn), PCI_FUNC(devfn), where);
		bus->unsafe_warn = 1;
	}

	ret = rzg3s_pcie_child_read_conf(host, bus, devfn, where, &data);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	if (size == 1) {
		shift = BITS_PER_BYTE * (where & 3);
		data &= ~(0xff << shift);
		data |= ((val & 0xff) << shift);
	} else if (size == 2) {
		shift = BITS_PER_BYTE * (where & 2);
		data &= ~(0xffff << shift);
		data |= ((val & 0xffff) << shift);
	} else {
		data = val;
	}

	return rzg3s_pcie_child_write_conf(host, bus, devfn, where, data);
}

static void __iomem *rzg3s_pcie_child_map_bus(struct pci_bus *bus,
					      unsigned int devfn,
					      int where)
{
	struct rzg3s_pcie_host *host = bus->sysdata;
	unsigned int dev, func, reg;

	dev = PCI_SLOT(devfn);
	func = PCI_FUNC(devfn);
	reg = where & ~0x3;

	/* Set the destination */
	writel(FIELD_PREP(RZG3S_PCI_REQADR1_BUS, bus->number) |
	       FIELD_PREP(RZG3S_PCI_REQADR1_DEV, dev) |
	       FIELD_PREP(RZG3S_PCI_REQADR1_FUNC, func) |
	       FIELD_PREP(RZG3S_PCI_REQADR1_REG, reg),
	       host->axi + RZG3S_PCI_REQADR1);

	/* Set byte enable */
	writel(RZG3S_PCI_REQBE_BYTE_EN, host->axi + RZG3S_PCI_REQBE);

	/*
	 * rzg3s_pcie_child_map_bus() is used to configure the controller before
	 * executing requests. It is called only within this driver and not
	 * through subsystem calls. Since it does not return an address that
	 * needs to be used later, return NULL.
	 */
	return NULL;
}

static struct pci_ops rzg3s_pcie_child_ops = {
	.read		= rzg3s_pcie_child_read,
	.write		= rzg3s_pcie_child_write,
	.map_bus	= rzg3s_pcie_child_map_bus,
};

static void __iomem *rzg3s_pcie_root_map_bus(struct pci_bus *bus,
					     unsigned int devfn,
					     int where)
{
	struct rzg3s_pcie_host *host = bus->sysdata;

	if (devfn)
		return NULL;

	return host->pcie + where;
}

static int rzg3s_pcie_root_write(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	struct rzg3s_pcie_host *host = bus->sysdata;

	/* Enable access control to the CFGU */
	writel(RZG3S_PCI_PERM_CFG_HWINIT_EN, host->axi + RZG3S_PCI_PERM);

	pci_generic_config_write(bus, devfn, where, size, val);

	/* Disable access control to the CFGU */
	writel(0, host->axi + RZG3S_PCI_PERM);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops rzg3s_pcie_root_ops = {
	.read		= pci_generic_config_read,
	.write		= rzg3s_pcie_root_write,
	.map_bus	= rzg3s_pcie_root_map_bus,
};

static void rzg3s_pcie_intx_irq_handler(struct irq_desc *desc)
{
	struct rzg3s_pcie_host *host = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int irq = irq_desc_get_irq(desc);
	u32 intx = irq - host->intx_irqs[0];

	chained_irq_enter(chip, desc);
	generic_handle_domain_irq(host->intx_domain, intx);
	chained_irq_exit(chip, desc);
}

static irqreturn_t rzg3s_pcie_msi_irq(int irq, void *data)
{
	u8 regs = RZG3S_PCI_MSI_INT_NR / RZG3S_PCI_MSI_INT_PER_REG;
	DECLARE_BITMAP(bitmap, RZG3S_PCI_MSI_INT_NR);
	struct rzg3s_pcie_host *host = data;
	struct rzg3s_pcie_msi *msi = &host->msi;
	unsigned long bit;
	u32 status;

	status = readl(host->axi + RZG3S_PCI_PINTRCVIS);
	if (!(status & RZG3S_PCI_PINTRCVIS_MSI))
		return IRQ_NONE;

	/* Clear the MSI */
	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PINTRCVIS,
			       RZG3S_PCI_PINTRCVIS_MSI,
			       RZG3S_PCI_PINTRCVIS_MSI);
	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_MSGRCVIS,
			       RZG3S_PCI_MSGRCVIS_MRI, RZG3S_PCI_MSGRCVIS_MRI);

	for (u8 reg_id = 0; reg_id < regs; reg_id++) {
		status = readl(host->axi + RZG3S_PCI_MSIRS(reg_id));
		bitmap_write(bitmap, status, reg_id * RZG3S_PCI_MSI_INT_PER_REG,
			     RZG3S_PCI_MSI_INT_PER_REG);
	}

	for_each_set_bit(bit, bitmap, RZG3S_PCI_MSI_INT_NR) {
		int ret;

		ret = generic_handle_domain_irq(msi->domain->parent, bit);
		if (ret) {
			u8 reg_bit = bit % RZG3S_PCI_MSI_INT_PER_REG;
			u8 reg_id = bit / RZG3S_PCI_MSI_INT_PER_REG;

			/* Unknown MSI, just clear it */
			writel(BIT(reg_bit),
			       host->axi + RZG3S_PCI_MSIRS(reg_id));
		}
	}

	return IRQ_HANDLED;
}

static void rzg3s_pcie_msi_top_irq_ack(struct irq_data *d)
{
	irq_chip_ack_parent(d);
}

static void rzg3s_pcie_msi_top_irq_mask(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void rzg3s_pcie_msi_top_irq_unmask(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip rzg3s_pcie_msi_top_chip = {
	.name		= "PCIe MSI",
	.irq_ack	= rzg3s_pcie_msi_top_irq_ack,
	.irq_mask	= rzg3s_pcie_msi_top_irq_mask,
	.irq_unmask	= rzg3s_pcie_msi_top_irq_unmask,
};

static void rzg3s_pcie_msi_irq_ack(struct irq_data *d)
{
	struct rzg3s_pcie_msi *msi = irq_data_get_irq_chip_data(d);
	struct rzg3s_pcie_host *host = rzg3s_msi_to_host(msi);
	u8 reg_bit = d->hwirq % RZG3S_PCI_MSI_INT_PER_REG;
	u8 reg_id = d->hwirq / RZG3S_PCI_MSI_INT_PER_REG;

	guard(raw_spinlock_irqsave)(&host->hw_lock);

	writel(BIT(reg_bit), host->axi + RZG3S_PCI_MSIRS(reg_id));
}

static void rzg3s_pcie_msi_irq_mask(struct irq_data *d)
{
	struct rzg3s_pcie_msi *msi = irq_data_get_irq_chip_data(d);
	struct rzg3s_pcie_host *host = rzg3s_msi_to_host(msi);
	u8 reg_bit = d->hwirq % RZG3S_PCI_MSI_INT_PER_REG;
	u8 reg_id = d->hwirq / RZG3S_PCI_MSI_INT_PER_REG;

	guard(raw_spinlock_irqsave)(&host->hw_lock);

	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_MSIRM(reg_id), BIT(reg_bit),
			       BIT(reg_bit));
}

static void rzg3s_pcie_msi_irq_unmask(struct irq_data *d)
{
	struct rzg3s_pcie_msi *msi = irq_data_get_irq_chip_data(d);
	struct rzg3s_pcie_host *host = rzg3s_msi_to_host(msi);
	u8 reg_bit = d->hwirq % RZG3S_PCI_MSI_INT_PER_REG;
	u8 reg_id = d->hwirq / RZG3S_PCI_MSI_INT_PER_REG;

	guard(raw_spinlock_irqsave)(&host->hw_lock);

	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_MSIRM(reg_id), BIT(reg_bit),
			       0);
}

static void rzg3s_pcie_irq_compose_msi_msg(struct irq_data *data,
					   struct msi_msg *msg)
{
	struct rzg3s_pcie_msi *msi = irq_data_get_irq_chip_data(data);
	struct rzg3s_pcie_host *host = rzg3s_msi_to_host(msi);
	u32 drop_mask = RZG3S_PCI_MSIRCVWADRL_ENA |
			RZG3S_PCI_MSIRCVWADRL_MSG_DATA_ENA;
	u32 lo, hi;

	/*
	 * Enable and msg data enable bits are part of the address lo. Drop
	 * them.
	 */
	lo = readl(host->axi + RZG3S_PCI_MSIRCVWADRL) & ~drop_mask;
	hi = readl(host->axi + RZG3S_PCI_MSIRCVWADRU);

	msg->address_lo = lo;
	msg->address_hi = hi;
	msg->data = data->hwirq;
}

static struct irq_chip rzg3s_pcie_msi_bottom_chip = {
	.name			= "rzg3s-pcie-msi",
	.irq_ack		= rzg3s_pcie_msi_irq_ack,
	.irq_mask		= rzg3s_pcie_msi_irq_mask,
	.irq_unmask		= rzg3s_pcie_msi_irq_unmask,
	.irq_compose_msi_msg	= rzg3s_pcie_irq_compose_msi_msg,
};

static int rzg3s_pcie_msi_domain_alloc(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs,
				       void *args)
{
	struct rzg3s_pcie_msi *msi = domain->host_data;
	int hwirq;

	scoped_guard(mutex, &msi->map_lock) {
		hwirq = bitmap_find_free_region(msi->map, RZG3S_PCI_MSI_INT_NR,
						order_base_2(nr_irqs));
	}

	if (hwirq < 0)
		return -ENOSPC;

	for (unsigned int i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    &rzg3s_pcie_msi_bottom_chip,
				    domain->host_data, handle_edge_irq, NULL,
				    NULL);
	}

	return 0;
}

static void rzg3s_pcie_msi_domain_free(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct rzg3s_pcie_msi *msi = domain->host_data;

	guard(mutex)(&msi->map_lock);

	bitmap_release_region(msi->map, d->hwirq, order_base_2(nr_irqs));
}

static const struct irq_domain_ops rzg3s_pcie_msi_domain_ops = {
	.alloc	= rzg3s_pcie_msi_domain_alloc,
	.free	= rzg3s_pcie_msi_domain_free,
};

static struct msi_domain_info rzg3s_pcie_msi_info = {
	.flags	= MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_MULTI_PCI_MSI | MSI_FLAG_NO_AFFINITY,
	.chip	= &rzg3s_pcie_msi_top_chip,
};

static int rzg3s_pcie_msi_allocate_domains(struct rzg3s_pcie_msi *msi)
{
	struct rzg3s_pcie_host *host = rzg3s_msi_to_host(msi);
	struct device *dev = host->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct irq_domain *parent;

	parent = irq_domain_create_linear(fwnode, RZG3S_PCI_MSI_INT_NR,
					  &rzg3s_pcie_msi_domain_ops, msi);
	if (!parent)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create IRQ domain\n");
	irq_domain_update_bus_token(parent, DOMAIN_BUS_NEXUS);

	msi->domain = pci_msi_create_irq_domain(fwnode, &rzg3s_pcie_msi_info,
						parent);
	if (!msi->domain) {
		irq_domain_remove(parent);
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create MSI domain\n");
	}

	return 0;
}

static int rzg3s_pcie_msi_hw_setup(struct rzg3s_pcie_host *host)
{
	u8 regs = RZG3S_PCI_MSI_INT_NR / RZG3S_PCI_MSI_INT_PER_REG;
	struct rzg3s_pcie_msi *msi = &host->msi;

	/*
	 * Set MSI window size. HW will set the window to
	 * RZG3S_PCI_MSI_INT_NR * 4 bytes.
	 */
	writel(RZG3S_PCI_MSI_INT_NR - 1, host->axi + RZG3S_PCI_MSIRCVWMSKL);

	/* Set MSI window address and enable MSI window */
	writel(upper_32_bits(msi->window_base),
	       host->axi + RZG3S_PCI_MSIRCVWADRU);
	writel(lower_32_bits(msi->window_base) | RZG3S_PCI_MSIRCVWADRL_ENA |
	       RZG3S_PCI_MSIRCVWADRL_MSG_DATA_ENA,
	       host->axi + RZG3S_PCI_MSIRCVWADRL);

	/* Set MSI receive enable */
	for (u8 reg_id = 0; reg_id < regs; reg_id++) {
		writel(RZG3S_PCI_MSIRE_ENA,
		       host->axi + RZG3S_PCI_MSIRE(reg_id));
	}

	/* Enable message receive interrupts */
	writel(RZG3S_PCI_MSGRCVIE_MSG_RCV, host->axi + RZG3S_PCI_MSGRCVIE);

	/* Enable MSI */
	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PINTRCVIE,
			       RZG3S_PCI_PINTRCVIE_MSI,
			       RZG3S_PCI_PINTRCVIE_MSI);

	return 0;
}

static int rzg3s_pcie_msi_setup(struct rzg3s_pcie_host *host)
{
	size_t size = RZG3S_PCI_MSI_INT_NR * sizeof(u32);
	struct rzg3s_pcie_msi *msi = &host->msi;
	struct device *dev = host->dev;
	int id, ret;

	msi->pages = __get_free_pages(GFP_KERNEL | GFP_DMA, 0);
	if (!msi->pages)
		return -ENOMEM;

	msi->dma_addr = dma_map_single(dev, (void *)msi->pages, size * 2,
				       DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, msi->dma_addr)) {
		ret = -ENOMEM;
		goto free_pages;
	}

	/*
	 * According to the RZ/G3S HW manual (Rev.1.10, section 34.4.5.2 Setting
	 * the MSI Window) the MSI window need to be within any AXI window. Find
	 * an AXI window to setup the MSI window.
	 */
	for (id = 0; id < RZG3S_MAX_WINDOWS; id++) {
		u64 base, basel, baseu;
		u64 mask, maskl, masku;

		basel = readl(host->axi + RZG3S_PCI_AWBASEL(id));
		/* Skip checking this AXI window if it's not enabled */
		if (!(basel & RZG3S_PCI_AWBASEL_WIN_ENA))
			continue;

		baseu = readl(host->axi + RZG3S_PCI_AWBASEU(id));
		base = baseu << 32 | basel;

		maskl = readl(host->axi + RZG3S_PCI_AWMASKL(id));
		masku = readl(host->axi + RZG3S_PCI_AWMASKU(id));
		mask = masku << 32 | maskl;

		if (msi->dma_addr < base || msi->dma_addr > base + mask)
			continue;

		break;
	}

	if (id == RZG3S_MAX_WINDOWS) {
		ret = -EINVAL;
		goto dma_unmap;
	}

	/* The MSI base address need to be aligned to the MSI size */
	msi->window_base = ALIGN(msi->dma_addr, size);
	if (msi->window_base < msi->dma_addr) {
		ret = -EINVAL;
		goto dma_unmap;
	}

	rzg3s_pcie_msi_hw_setup(host);

	return 0;

dma_unmap:
	dma_unmap_single(dev, msi->dma_addr, size * 2, DMA_BIDIRECTIONAL);
free_pages:
	free_pages(msi->pages, 0);
	return ret;
}

static void rzg3s_pcie_msi_teardown(void *data)
{
	u8 regs = RZG3S_PCI_MSI_INT_NR / RZG3S_PCI_MSI_INT_PER_REG;
	size_t size = RZG3S_PCI_MSI_INT_NR * sizeof(u32);
	struct rzg3s_pcie_host *host = data;
	struct rzg3s_pcie_msi *msi = &host->msi;

	/* Disable MSI */
	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PINTRCVIE,
			       RZG3S_PCI_PINTRCVIE_MSI, 0);

	/* Disable message receive interrupts */
	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_MSGRCVIE,
			       RZG3S_PCI_MSGRCVIE_MSG_RCV, 0);

	/* Disable MSI receive enable */
	for (u8 reg_id = 0; reg_id < regs; reg_id++)
		writel(0, host->axi + RZG3S_PCI_MSIRE(reg_id));

	/* Disable MSI window */
	writel(0, host->axi + RZG3S_PCI_MSIRCVWADRL);

	/* Free unused memory */
	dma_unmap_single(host->dev, msi->dma_addr, size * 2, DMA_BIDIRECTIONAL);
	free_pages(msi->pages, 0);
}

static void rzg3s_pcie_msi_free_domains(void *data)
{
	struct irq_domain *domain = data;
	struct irq_domain *parent = domain->parent;

	irq_domain_remove(domain);
	irq_domain_remove(parent);
}

static int rzg3s_pcie_msi_enable(struct rzg3s_pcie_host *host)
{
	struct platform_device *pdev = to_platform_device(host->dev);
	struct rzg3s_pcie_msi *msi = &host->msi;
	struct device *dev = host->dev;
	const char *devname;
	int irq, ret;

	ret = devm_mutex_init(dev, &msi->map_lock);
	if (ret)
		return ret;

	irq = platform_get_irq_byname(pdev, "msi");
	if (irq < 0)
		return dev_err_probe(dev, irq ? irq : -EINVAL,
				     "Failed to get MSI IRQ!\n");

	devname = devm_kasprintf(dev, GFP_KERNEL, "%s-msi", dev_name(dev));
	if (!devname)
		return -ENOMEM;

	ret = rzg3s_pcie_msi_allocate_domains(msi);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, rzg3s_pcie_msi_free_domains, msi->domain);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, irq, rzg3s_pcie_msi_irq, 0, devname, host);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ: %d\n", ret);

	ret = rzg3s_pcie_msi_setup(host);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to setup MSI!\n");

	return devm_add_action_or_reset(dev, rzg3s_pcie_msi_teardown, host);
}

static void rzg3s_pcie_intx_irq_ack(struct irq_data *d)
{
	struct rzg3s_pcie_host *host = irq_data_get_irq_chip_data(d);

	guard(raw_spinlock_irqsave)(&host->hw_lock);

	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PINTRCVIS,
			       RZG3S_PCI_PINTRCVIS_INTX(d->hwirq),
			       RZG3S_PCI_PINTRCVIS_INTX(d->hwirq));
}

static void rzg3s_pcie_intx_irq_mask(struct irq_data *d)
{
	struct rzg3s_pcie_host *host = irq_data_get_irq_chip_data(d);

	guard(raw_spinlock_irqsave)(&host->hw_lock);

	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PINTRCVIE,
			       RZG3S_PCI_PINTRCVIE_INTX(d->hwirq), 0);
}

static void rzg3s_pcie_intx_irq_unmask(struct irq_data *d)
{
	struct rzg3s_pcie_host *host = irq_data_get_irq_chip_data(d);

	guard(raw_spinlock_irqsave)(&host->hw_lock);

	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PINTRCVIE,
			       RZG3S_PCI_PINTRCVIE_INTX(d->hwirq),
			       RZG3S_PCI_PINTRCVIE_INTX(d->hwirq));
}

static struct irq_chip rzg3s_pcie_intx_irq_chip = {
	.name = "PCIe INTx",
	.irq_ack = rzg3s_pcie_intx_irq_ack,
	.irq_mask = rzg3s_pcie_intx_irq_mask,
	.irq_unmask = rzg3s_pcie_intx_irq_unmask,
};

static int rzg3s_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &rzg3s_pcie_intx_irq_chip,
				 handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops rzg3s_pcie_intx_domain_ops = {
	.map = rzg3s_pcie_intx_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static void rzg3s_pcie_intx_teardown(void *data)
{
	struct rzg3s_pcie_host *host = data;

	irq_domain_remove(host->intx_domain);
}

static int rzg3s_pcie_intx_setup(struct rzg3s_pcie_host *host)
{
	struct device *dev = host->dev;

	for (int i = 0; i < PCI_NUM_INTX; i++) {
		struct platform_device *pdev = to_platform_device(dev);
		char irq_name[5] = {0};
		int irq;

		scnprintf(irq_name, ARRAY_SIZE(irq_name), "int%c", 97 + i);

		irq = platform_get_irq_byname(pdev, irq_name);
		if (irq < 0)
			return dev_err_probe(dev, -EINVAL,
					     "Failed to parse and map INT%c IRQ\n",
					     65 + i);

		host->intx_irqs[i] = irq;
		irq_set_chained_handler_and_data(irq,
						 rzg3s_pcie_intx_irq_handler,
						 host);
	}

	host->intx_domain = irq_domain_create_linear(of_fwnode_handle(dev->of_node),
						     PCI_NUM_INTX,
						     &rzg3s_pcie_intx_domain_ops,
						     host);
	if (!host->intx_domain)
		return dev_err_probe(dev, -EINVAL,
				     "Failed to add irq domain for INTx IRQs\n");
	irq_domain_update_bus_token(host->intx_domain, DOMAIN_BUS_WIRED);

	return devm_add_action_or_reset(dev, rzg3s_pcie_intx_teardown, host);
}

static int rzg3s_pcie_set_max_link_speed(struct rzg3s_pcie_host *host)
{
	u32 cs2, link_speed, remote_supported_link_speeds, tmp;
	u32 pcie_cap = RZG3S_PCI_CFG_PCIEC;
	u8 ltssm_state_l0 = 0xc;
	u16 lcs;
	int ret;

	/*
	 * According to the RZ/G3S HW manual (Rev.1.10, section 34.6.3 Caution
	 * when Changing the Speed Spontaneously) link speed change can be done
	 * only when the link training and status state machine in the PCIe Core
	 * Link is L0.
	 */
	ret = readl_poll_timeout(host->axi + RZG3S_PCI_PCSTAT1, tmp,
				 FIELD_GET(RZG3S_PCI_PCSTAT1_LTSSM_STATE, tmp) == ltssm_state_l0,
				 PCIE_LINK_WAIT_SLEEP_MS,
				 PCIE_LINK_WAIT_SLEEP_MS *
				 PCIE_LINK_WAIT_MAX_RETRIES * MILLI);
	if (ret) {
		dev_dbg(host->dev,
			"Could not set max link speed! LTSSM not in L0, state=%lx\n",
			FIELD_GET(RZG3S_PCI_PCSTAT1_LTSSM_STATE, tmp));
		return ret;
	}

	lcs = readw(host->pcie + pcie_cap + PCI_EXP_LNKSTA);
	cs2 = readl(host->axi + RZG3S_PCI_PCSTAT2);

	link_speed = FIELD_GET(PCI_EXP_LNKSTA_CLS, lcs);
	remote_supported_link_speeds = FIELD_GET(RZG3S_PCI_PCSTAT2_SDRIRE, cs2);

	/*
	 * Return if link is @ 5.0 GT/s or the connected device doesn't support
	 * it.
	 */
	if (link_speed == PCI_EXP_LNKSTA_CLS_5_0GB ||
	    !(remote_supported_link_speeds != GENMASK(PCI_EXP_LNKSTA_CLS_5_0GB - 1, 0)))
		return 0;

	/* Set target Link speed to 5.0 GT/s */
	rzg3s_pcie_update_bits(host->pcie, pcie_cap + PCI_EXP_LNKCTL2,
			       PCI_EXP_LNKCTL2_TLS,
			       FIELD_PREP(PCI_EXP_LNKCTL2_TLS,
					  PCI_EXP_LNKCTL2_TLS_5_0GT));

	/* Request link speed change */
	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PCCTRL2,
			       RZG3S_PCI_PCCTRL2_LS_CHG_REQ |
			       RZG3S_PCI_PCCTRL2_LS_CHG,
			       RZG3S_PCI_PCCTRL2_LS_CHG_REQ |
			       FIELD_PREP(RZG3S_PCI_PCCTRL2_LS_CHG,
					  PCI_EXP_LNKCTL2_TLS_5_0GT - 1));

	ret = readl_poll_timeout(host->axi + RZG3S_PCI_PCSTAT2, cs2,
				 (cs2 & RZG3S_PCI_PCSTAT2_LS_CHG_DONE),
				 PCIE_LINK_WAIT_SLEEP_MS,
				 PCIE_LINK_WAIT_SLEEP_MS *
				 PCIE_LINK_WAIT_MAX_RETRIES * MILLI);

	/*
	 * According to the RZ/G3S HW manual (Rev.1.10, section 34.6.3 Caution
	 * when Changing the Speed Spontaneously) the PCI_PCCTRL2_LS_CHG_REQ
	 * should be de-asserted after checking for PCI_PCSTAT2_LS_CHG_DONE.
	 */
	rzg3s_pcie_update_bits(host->axi, RZG3S_PCI_PCCTRL2,
			       RZG3S_PCI_PCCTRL2_LS_CHG_REQ, 0);

	return ret;
}

static int rzg3s_pcie_config_init(struct rzg3s_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *ft;
	struct resource *bus;
	u8 subordinate_bus;
	u8 secondary_bus;
	u8 primary_bus;

	ft = resource_list_first_type(&bridge->windows, IORESOURCE_BUS);
	if (!ft)
		return -ENODEV;

	bus = ft->res;
	primary_bus = bus->start;
	secondary_bus = bus->start + 1;
	subordinate_bus = bus->end;

	/* Enable access control to the CFGU */
	writel(RZG3S_PCI_PERM_CFG_HWINIT_EN, host->axi + RZG3S_PCI_PERM);

	/* Update vendor ID and device ID */
	writew(host->vendor_id, host->pcie + PCI_VENDOR_ID);
	writew(host->device_id, host->pcie + PCI_DEVICE_ID);

	/* HW manual recommends to write 0xffffffff on initialization */
	writel(0xffffffff, host->pcie + RZG3S_PCI_CFG_BARMSK00L);
	writel(0xffffffff, host->pcie + RZG3S_PCI_CFG_BARMSK00U);

	/* Update bus info. */
	writeb(primary_bus, host->pcie + PCI_PRIMARY_BUS);
	writeb(secondary_bus, host->pcie + PCI_SECONDARY_BUS);
	writeb(subordinate_bus, host->pcie + PCI_SUBORDINATE_BUS);

	/* Disable access control to the CFGU */
	writel(0, host->axi + RZG3S_PCI_PERM);

	return 0;
}

static void rzg3s_pcie_irq_init(struct rzg3s_pcie_host *host)
{
	/*
	 * According to the HW manual of the RZ/G3S (Rev.1.10, sections
	 * corresponding to all registers written with ~0U), the hardware
	 * ignores value written to unused bits. Writing ~0U to these registers
	 * should be safe.
	 */

	/* Clear the link state and PM transitions */
	writel(RZG3S_PCI_PEIS0_DL_UPDOWN | RZG3S_PCI_PEIS0_RX_DLLP_PM_ENTER,
	       host->axi + RZG3S_PCI_PEIS0);

	/* Disable all interrupts */
	writel(0, host->axi + RZG3S_PCI_PEIE0);

	/* Clear all parity and ecc error interrupts */
	writel(~0U, host->axi + RZG3S_PCI_PEIS1);

	/* Disable all parity and ecc error interrupts */
	writel(0, host->axi + RZG3S_PCI_PEIE1);

	/* Clear all AXI master error interrupts */
	writel(~0U, host->axi + RZG3S_PCI_AMEIS);

	/* Clear all AXI slave error interrupts */
	writel(~0U, host->axi + RZG3S_PCI_ASEIS1);

	/* Clear all message receive interrupts */
	writel(~0U, host->axi + RZG3S_PCI_MSGRCVIS);
}

static void rzg3s_pcie_power_resets_action(void *data)
{
	struct rzg3s_pcie_host *host = data;

	reset_control_bulk_assert(host->data->num_power_resets,
				  host->power_resets);
}

static void rzg3s_pcie_cfg_resets_action(void *data)
{
	struct rzg3s_pcie_host *host = data;

	reset_control_bulk_assert(host->data->num_cfg_resets,
				  host->cfg_resets);
}

static int rzg3s_pcie_power_resets_deassert(struct rzg3s_pcie_host *host)
{
	const struct rzg3s_pcie_soc_data *data = host->data;

	/*
	 * According to the RZ/G3S HW manual (Rev.1.10, section
	 * 34.5.1.2 De-asserting the Reset) the PCIe IP needs to wait 5ms from
	 * power on to the de-assertion of reset.
	 */
	usleep_range(5000, 5100);
	return reset_control_bulk_deassert(data->num_power_resets,
					   host->power_resets);
}

static int rzg3s_pcie_resets_prepare(struct rzg3s_pcie_host *host)
{
	const struct rzg3s_pcie_soc_data *data = host->data;
	int ret;

	host->power_resets = devm_kmalloc_array(host->dev,
						data->num_power_resets,
						sizeof(*host->power_resets),
						GFP_KERNEL);
	if (!host->power_resets)
		return -ENOMEM;

	for (unsigned int i = 0; i < data->num_power_resets; i++)
		host->power_resets[i].id = data->power_resets[i];

	host->cfg_resets = devm_kmalloc_array(host->dev,
					      data->num_cfg_resets,
					      sizeof(*host->cfg_resets),
					      GFP_KERNEL);
	if (!host->cfg_resets)
		return -ENOMEM;

	for (unsigned int i = 0; i < data->num_cfg_resets; i++)
		host->cfg_resets[i].id = data->cfg_resets[i];

	ret = devm_reset_control_bulk_get_exclusive(host->dev,
						    data->num_power_resets,
						    host->power_resets);
	if (ret)
		return ret;

	ret = devm_reset_control_bulk_get_exclusive(host->dev,
						    data->num_cfg_resets,
						    host->cfg_resets);
	if (ret)
		return ret;

	ret = rzg3s_pcie_power_resets_deassert(host);
	if (ret)
		return ret;

	return devm_add_action_or_reset(host->dev,
					rzg3s_pcie_power_resets_action, host);
}

static int rzg3s_pcie_host_init(struct rzg3s_pcie_host *host, bool probe)
{
	u32 val;
	int ret;

	/* Initialize the PCIe related registers */
	ret = rzg3s_pcie_config_init(host);
	if (ret)
		return ret;

	/* Initialize the interrupts */
	rzg3s_pcie_irq_init(host);

	ret = reset_control_bulk_deassert(host->data->num_cfg_resets,
					  host->cfg_resets);
	if (ret)
		return ret;

	/* Wait for link up */
	ret = readl_poll_timeout(host->axi + RZG3S_PCI_PCSTAT1, val,
				 !(val & RZG3S_PCI_PCSTAT1_DL_DOWN_STS),
				 PCIE_LINK_WAIT_SLEEP_MS,
				 PCIE_LINK_WAIT_SLEEP_MS *
				 PCIE_LINK_WAIT_MAX_RETRIES * MILLI);
	if (ret) {
		reset_control_bulk_assert(host->data->num_cfg_resets,
					  host->cfg_resets);
		return ret;
	}

	val = readl(host->axi + RZG3S_PCI_PCSTAT2);
	dev_info(host->dev, "PCIe link status [0x%x]\n", val);

	val = FIELD_GET(RZG3S_PCI_PCSTAT2_STATE_RX_DETECT, val);
	dev_info(host->dev, "PCIe x%d: link up\n", hweight32(val));

	if (probe) {
		ret = devm_add_action_or_reset(host->dev,
					       rzg3s_pcie_cfg_resets_action,
					       host);
	}

	return ret;
}

static void rzg3s_pcie_set_inbound_window(struct rzg3s_pcie_host *host,
					  u64 cpu_addr, u64 pci_addr, u64 size,
					  int id)
{
	/* Set CPU window base address */
	writel(upper_32_bits(cpu_addr), host->axi + RZG3S_PCI_ADESTU(id));
	writel(lower_32_bits(cpu_addr), host->axi + RZG3S_PCI_ADESTL(id));

	/* Set window size */
	writel(upper_32_bits(size), host->axi + RZG3S_PCI_AWMASKU(id));
	writel(lower_32_bits(size), host->axi + RZG3S_PCI_AWMASKL(id));

	/* Set PCIe window base address and enable the window */
	writel(upper_32_bits(pci_addr), host->axi + RZG3S_PCI_AWBASEU(id));
	writel(lower_32_bits(pci_addr) | RZG3S_PCI_AWBASEL_WIN_ENA,
	       host->axi + RZG3S_PCI_AWBASEL(id));
}

static int rzg3s_pcie_set_inbound_windows(struct rzg3s_pcie_host *host,
					  struct resource_entry *entry,
					  int *index)
{
	u64 pci_addr = entry->res->start - entry->offset;
	u64 cpu_addr = entry->res->start;
	u64 cpu_end = entry->res->end;
	u64 size_id = 0;
	int id = *index;
	u64 size;

	while (cpu_addr < cpu_end) {
		if (id >= RZG3S_MAX_WINDOWS)
			return dev_err_probe(host->dev, -EINVAL,
					     "Failed to set inbound windows!\n");

		size = resource_size(entry->res) - size_id;

		/*
		 * According to the RZ/G3S HW manual (Rev.1.10,
		 * section 34.3.1.71 AXI Window Mask (Lower) Registers) the min
		 * size is 4K.
		 */
		size = max(size, SZ_4K);

		/*
		 * According the RZ/G3S HW manual (Rev.1.10, sections:
		 * - 34.3.1.69 AXI Window Base (Lower) Registers
		 * - 34.3.1.71 AXI Window Mask (Lower) Registers
		 * - 34.3.1.73 AXI Destination (Lower) Registers)
		 * the CPU addr, PCIe addr, size should be 4K aligned and be a
		 * power of 2.
		 */
		size = ALIGN(size, SZ_4K);

		/*
		 * According to the RZ/G3S HW manual (Rev.1.10, section
		 * 34.3.1.71 AXI Window Mask (Lower) Registers) HW expects first
		 * 12 LSB bits to be 0xfff. Subtract 1 from size for this.
		 */
		size = roundup_pow_of_two(size) - 1;

		cpu_addr = ALIGN(cpu_addr, SZ_4K);
		pci_addr = ALIGN(pci_addr, SZ_4K);

		rzg3s_pcie_set_inbound_window(host, cpu_addr, pci_addr, size,
					      id);

		pci_addr += size;
		cpu_addr += size;
		size_id = size;
		id++;
	}
	*index = id;

	return 0;
}

static int rzg3s_pcie_parse_map_dma_ranges(struct rzg3s_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *entry;
	int i = 0, ret;

	resource_list_for_each_entry(entry, &bridge->dma_ranges) {
		ret = rzg3s_pcie_set_inbound_windows(host, entry, &i);
		if (ret)
			return ret;
	}

	return 0;
}

static void rzg3s_pcie_set_outbound_window(struct rzg3s_pcie_host *host,
					   struct resource_entry *win,
					   int id)
{
	struct resource *res = win->res;
	resource_size_t size = resource_size(res);
	resource_size_t res_start;

	if (res->flags & IORESOURCE_IO)
		res_start = pci_pio_to_address(res->start) - win->offset;
	else
		res_start = res->start - win->offset;

	/*
	 * According to the RZ/G3S HW manual (Rev.1.10, section 34.3.1.75 PCIe
	 * Window Base (Lower) Registers) the window base address need to be 4K
	 * aligned.
	 */
	res_start = ALIGN(res_start, SZ_4K);

	size = ALIGN(size, SZ_4K);
	size = roundup_pow_of_two(size) - 1;

	/* Set PCIe destination */
	writel(upper_32_bits(res_start), host->axi + RZG3S_PCI_PDESTU(id));
	writel(lower_32_bits(res_start), host->axi + RZG3S_PCI_PDESTL(id));

	/* Set PCIe window mask */
	writel(upper_32_bits(size), host->axi + RZG3S_PCI_PWMASKU(id));
	writel(lower_32_bits(size), host->axi + RZG3S_PCI_PWMASKL(id));

	/* Set PCIe window base and enable the window */
	writel(upper_32_bits(res_start), host->axi + RZG3S_PCI_PWBASEU(id));
	writel(lower_32_bits(res_start) | RZG3S_PCI_PWBASEL_ENA,
	       host->axi + RZG3S_PCI_PWBASEL(id));
}

static int rzg3s_pcie_parse_map_ranges(struct rzg3s_pcie_host *host)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);
	struct resource_entry *win;
	int i = 0;

	resource_list_for_each_entry(win, &bridge->windows) {
		struct resource *res = win->res;

		if (i >= RZG3S_MAX_WINDOWS)
			return dev_err_probe(host->dev, -EINVAL,
					     "Failed to set outbound windows!\n");

		if (!res->flags)
			continue;

		switch (resource_type(res)) {
		case IORESOURCE_IO:
		case IORESOURCE_MEM:
			rzg3s_pcie_set_outbound_window(host, win, i);
			i++;
			break;
		}
	}

	return 0;
}

static int rzg3s_soc_pcie_init_phy(struct rzg3s_pcie_host *host)
{
	static const u32 xcfgd_settings[RZG3S_PCI_PHY_XCFGD_NUM] = {
		[8]  = 0xe0006801, 0x007f7e30, 0x183e0000, 0x978ff500,
		       0xec000000, 0x009f1400, 0x0000d009,
		[17] = 0x78000000,
		[19] = 0x00880000, 0x000005c0, 0x07000000, 0x00780920,
		       0xc9400ce2, 0x90000c0c, 0x000c1414, 0x00005034,
		       0x00006000, 0x00000001,
	};
	static const u32 xcfga_cmn_settings[RZG3S_PCI_PHY_XCFGA_CMN_NUM] = {
		0x00000d10, 0x08310100, 0x00c21404, 0x013c0010, 0x01874440,
		0x1a216082, 0x00103440, 0x00000080, 0x00000010, 0x0c1000c1,
		0x1000c100, 0x0222000c, 0x00640019, 0x00a00028, 0x01d11228,
		0x0201001d,
	};
	static const u32 xcfga_rx_settings[RZG3S_PCI_PHY_XCFGA_RX_NUM] = {
		0x07d55000, 0x030e3f00, 0x00000288, 0x102c5880, 0x0000000b,
		0x04141441, 0x00641641, 0x00d63d63, 0x00641641, 0x01970377,
		0x00190287, 0x00190028, 0x00000028,
	};

	/*
	 * Enable access permission for physical layer control and status
	 * registers.
	 */
	writel(RZG3S_PCI_PERM_PIPE_PHY_REG_EN, host->axi + RZG3S_PCI_PERM);

	for (u8 i = 0; i < RZG3S_PCI_PHY_XCFGD_NUM; i++)
		writel(xcfgd_settings[i], host->axi + RZG3S_PCI_PHY_XCFGD(i));

	for (u8 i = 0; i < RZG3S_PCI_PHY_XCFGA_CMN_NUM; i++) {
		writel(xcfga_cmn_settings[i],
		       host->axi + RZG3S_PCI_PHY_XCFGA_CMN(i));
	}

	for (u8 i = 0; i < RZG3S_PCI_PHY_XCFGA_RX_NUM; i++) {
		writel(xcfga_rx_settings[i],
		       host->axi + RZG3S_PCI_PHY_XCFGA_RX(i));
	}

	writel(0x107, host->axi + RZG3S_PCI_PHY_XCFGA_TX);

	/* Select PHY settings values */
	writel(RZG3S_PCI_PHY_XCFG_CTRL_PHYREG_SEL,
	       host->axi + RZG3S_PCI_PHY_XCFG_CTRL);

	/*
	 * Disable access permission for physical layer control and status
	 * registers.
	 */
	writel(0, host->axi + RZG3S_PCI_PERM);

	return 0;
}

static void rzg3s_pcie_pm_runtime_put(void *data)
{
	pm_runtime_put_sync(data);
}

static void rzg3s_pcie_sysc_signal_action(void *data)
{
	struct regmap *sysc = data;

	/*
	 * SYSC RST_RSM_B signal need to be asserted before turning off the
	 * power to the PHY.
	 */
	regmap_update_bits(sysc, RZG3S_SYS_PCIE_RST_RSM_B,
			   RZG3S_SYS_PCIE_RST_RSM_B_MASK,
			   FIELD_PREP(RZG3S_SYS_PCIE_RST_RSM_B_MASK, 0));
}

static int
rzg3s_pcie_host_setup(struct rzg3s_pcie_host *host,
		      int (*intx_setup)(struct rzg3s_pcie_host *host),
		      int (*msi_setup)(struct rzg3s_pcie_host *host),
		      bool probe)
{
	struct device *dev = host->dev;
	int ret;

	/* Set inbound windows */
	ret = rzg3s_pcie_parse_map_dma_ranges(host);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set inbound windows!\n");

	/* Set outbound windows */
	ret = rzg3s_pcie_parse_map_ranges(host);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set outbound windows!\n");

	/* Set the PHY, if any */
	if (host->data->init_phy) {
		ret = host->data->init_phy(host);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to set the PHY!\n");
	}

	if (intx_setup) {
		ret = intx_setup(host);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to setup INTx\n");
	}

	/* Set the MSIs */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = msi_setup(host);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to setup MSIs\n");
	}

	/* Initialize the host */
	ret = rzg3s_pcie_host_init(host, probe);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to initialize the HW!\n");

	/* Try to set maximum supported link speed */
	ret = rzg3s_pcie_set_max_link_speed(host);
	if (ret)
		dev_info(dev, "Failed to set max link speed\n");

	return 0;
}

static void rzg3s_pcie_host_remove_action(void *data)
{
	struct rzg3s_pcie_host *host = data;
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(host);

	pci_lock_rescan_remove();
	pci_stop_root_bus(bridge->bus);
	pci_remove_root_bus(bridge->bus);
	pci_unlock_rescan_remove();
}

static int rzg3s_pcie_probe(struct platform_device *pdev)
{
	struct pci_host_bridge *bridge;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *sysc_np __free(device_node) =
		of_parse_phandle(np, "renesas,sysc", 0);
	struct rzg3s_pcie_host *host;
	int ret;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*host));
	if (!bridge)
		return -ENOMEM;

	host = pci_host_bridge_priv(bridge);
	host->dev = dev;
	host->data = device_get_match_data(dev);
	platform_set_drvdata(pdev, host);

	host->axi = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->axi))
		return PTR_ERR(host->axi);
	host->pcie = host->axi + RZG3S_PCI_CFG_BASE;

	ret = of_property_read_u32(np, "vendor-id", &host->vendor_id);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "device-id", &host->device_id);
	if (ret)
		return ret;

	host->sysc = syscon_node_to_regmap(sysc_np);
	if (IS_ERR(host->sysc))
		return PTR_ERR(host->sysc);

	ret = regmap_update_bits(host->sysc, RZG3S_SYS_PCIE_RST_RSM_B,
				 RZG3S_SYS_PCIE_RST_RSM_B_MASK,
				 FIELD_PREP(RZG3S_SYS_PCIE_RST_RSM_B_MASK, 1));
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, rzg3s_pcie_sysc_signal_action,
				       host->sysc);
	if (ret)
		return ret;

	ret = rzg3s_pcie_resets_prepare(host);
	if (ret)
		return ret;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, rzg3s_pcie_pm_runtime_put, dev);
	if (ret)
		return ret;

	raw_spin_lock_init(&host->hw_lock);

	ret = rzg3s_pcie_host_setup(host, rzg3s_pcie_intx_setup,
				    rzg3s_pcie_msi_enable, true);
	if (ret)
		return ret;

	msleep(PCIE_RESET_CONFIG_WAIT_MS);

	bridge->sysdata = host;
	bridge->ops = &rzg3s_pcie_root_ops;
	bridge->child_ops = &rzg3s_pcie_child_ops;
	ret = pci_host_probe(bridge);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, rzg3s_pcie_host_remove_action,
					host);
}

static int rzg3s_pcie_suspend_noirq(struct device *dev)
{
	struct rzg3s_pcie_host *host = dev_get_drvdata(dev);
	const struct rzg3s_pcie_soc_data *data = host->data;
	struct regmap *sysc = host->sysc;
	int ret;

	ret = pm_runtime_put_sync(dev);
	if (ret)
		return ret;

	ret = reset_control_bulk_assert(data->num_power_resets,
					host->power_resets);
	if (ret)
		goto rpm_restore;

	ret = reset_control_bulk_assert(data->num_cfg_resets,
					host->cfg_resets);
	if (ret)
		goto power_resets_restore;

	ret = regmap_update_bits(sysc, RZG3S_SYS_PCIE_RST_RSM_B,
				 RZG3S_SYS_PCIE_RST_RSM_B_MASK,
				 FIELD_PREP(RZG3S_SYS_PCIE_RST_RSM_B_MASK, 0));
	if (ret)
		goto cfg_resets_restore;

	return 0;

	/* Restore the previous state if any error happens */
cfg_resets_restore:
	reset_control_bulk_deassert(data->num_cfg_resets,
				    host->cfg_resets);
power_resets_restore:
	reset_control_bulk_deassert(data->num_power_resets,
				    host->power_resets);
rpm_restore:
	pm_runtime_resume_and_get(dev);
	return ret;
}

static int rzg3s_pcie_resume_noirq(struct device *dev)
{
	struct rzg3s_pcie_host *host = dev_get_drvdata(dev);
	const struct rzg3s_pcie_soc_data *data = host->data;
	struct regmap *sysc = host->sysc;
	int ret;

	ret = regmap_update_bits(sysc, RZG3S_SYS_PCIE_RST_RSM_B,
				 RZG3S_SYS_PCIE_RST_RSM_B_MASK,
				 FIELD_PREP(RZG3S_SYS_PCIE_RST_RSM_B_MASK, 1));
	if (ret)
		return ret;

	ret = rzg3s_pcie_power_resets_deassert(host);
	if (ret)
		goto assert_rst_rsm_b;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		goto assert_power_resets;

	ret = rzg3s_pcie_host_setup(host, NULL, rzg3s_pcie_msi_hw_setup, false);
	if (ret)
		goto rpm_put;

	return 0;

	/*
	 * If any error happens there is no way to recover the IP. Put it in the
	 * lowest possible power state.
	 */
rpm_put:
	pm_runtime_put_sync(dev);
assert_power_resets:
	reset_control_bulk_assert(data->num_power_resets,
				  host->power_resets);
assert_rst_rsm_b:
	regmap_update_bits(sysc, RZG3S_SYS_PCIE_RST_RSM_B,
			   RZG3S_SYS_PCIE_RST_RSM_B_MASK,
			   FIELD_PREP(RZG3S_SYS_PCIE_RST_RSM_B_MASK, 0));
	return ret;
}

static const struct dev_pm_ops rzg3s_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(rzg3s_pcie_suspend_noirq,
				  rzg3s_pcie_resume_noirq)
};

static const char * const rzg3s_soc_power_resets[] = {
	"aresetn", "rst_cfg_b", "rst_load_b",
};

static const char * const rzg3s_soc_cfg_resets[] = {
	"rst_b", "rst_ps_b", "rst_gp_b", "rst_rsm_b",
};

static const struct rzg3s_pcie_soc_data rzg3s_soc_data = {
	.power_resets = rzg3s_soc_power_resets,
	.num_power_resets = ARRAY_SIZE(rzg3s_soc_power_resets),
	.cfg_resets = rzg3s_soc_cfg_resets,
	.num_cfg_resets = ARRAY_SIZE(rzg3s_soc_cfg_resets),
	.init_phy = rzg3s_soc_pcie_init_phy,
};

static const struct of_device_id rzg3s_pcie_of_match[] = {
	{
		.compatible = "renesas,r9a08g045s33-pcie",
		.data = &rzg3s_soc_data,
	},
	{},
};

static struct platform_driver rzg3s_pcie_driver = {
	.driver = {
		.name = "rzg3s-pcie-host",
		.of_match_table = rzg3s_pcie_of_match,
		.pm = pm_ptr(&rzg3s_pcie_pm_ops),
		.suppress_bind_attrs = true,
	},
	.probe = rzg3s_pcie_probe,
};
module_platform_driver(rzg3s_pcie_driver);

MODULE_DESCRIPTION("Renesas RZ/G3S PCIe host driver");
MODULE_AUTHOR("Claudiu Beznea <claudiu.beznea.uj@bp.renesas.com>");
MODULE_LICENSE("GPL");
