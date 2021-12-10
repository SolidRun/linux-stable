// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe ecam driver for NXP's Layerscape SOCs, adopted from
 * Amazon's Graviton driver.
 *
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Copyright 2021 SolidRun Ltd. All Rights Reserved.
 *
 * Author: Jonathan Chocron <jonnyc@amazon.com>
 * Author: Jon Nettleton <jon@solid-run.com>
 */

#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pci-acpi.h>
#include "../pci.h"

#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)

struct ls_pcie_ecam  {
	void __iomem *ccsr_base;
};

static void __iomem *ls_pcie_ecam_map_bus(struct pci_bus *bus, unsigned int devfn,
				     int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct ls_pcie_ecam *pcie = cfg->priv;
	void __iomem *ccsr_base = pcie->ccsr_base;

	if (bus->number == 0) {
		/*
		 * 
		 * No devices/functions on the root bus num, so we do this here.
		 */
		if (PCI_SLOT(devfn) > 0)
			return NULL;
		else
			return ccsr_base + where;
	}

	return pci_ecam_map_bus(bus, devfn, where);
}

static int ls_pcie_ecam_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct acpi_device *adev = to_acpi_device(dev);
	struct acpi_pci_root *root = acpi_driver_data(adev);
	struct ls_pcie_ecam *ls_pcie;
	struct resource *res;
	int ret;

	ls_pcie = devm_kzalloc(dev, sizeof(*ls_pcie), GFP_KERNEL);
	if (!ls_pcie)
		return -ENOMEM;

	res = devm_kzalloc(dev, sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;

	ret = acpi_get_rc_resources(dev, "NXP0016", root->segment, res);
	if (ret) {
		dev_err(dev, "can't get rc csr base address for SEG %d\n",
			root->segment);
		return ret;
	}

	dev_dbg(dev, "Root port ccsr res: %pR\n", res);

	ls_pcie->ccsr_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(ls_pcie->ccsr_base))
		return PTR_ERR(ls_pcie->ccsr_base);

	cfg->priv = ls_pcie;

	return 0;
}

const struct pci_ecam_ops ls_pcie_ecam_ops = {
	.init         =  ls_pcie_ecam_init,
	.pci_ops      = {
		.map_bus    = ls_pcie_ecam_map_bus,
		.read       = pci_generic_config_read,
		.write      = pci_generic_config_write,
	}
};

#endif /* defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS) */
