// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe driver for Renesas RZ/V2H Series SoCs
 *  Copyright (C) 2022 Renesas Electronics Europe Ltd
 */

#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "pcie-rzv2h.h"

/* ----------------------------------------------------
  Static Function
-------------------------------------------------------*/

void rzv2h_pci_write_reg(struct rzv2h_pcie *pcie, u32 val, unsigned long reg)
{
	writel(val, pcie->base + reg);
}

u32 rzv2h_pci_read_reg(struct rzv2h_pcie *pcie, unsigned long reg)
{
	return readl(pcie->base + reg);
}

void rzv2h_rmw(struct rzv2h_pcie *pcie, int where, u32 mask, u32 data)
{
	u32 val = rzv2h_pci_read_reg(pcie, where);

	val &= ~(mask);
	val |= (data);
	rzv2h_pci_write_reg(pcie, val, where);
}

u32 rzv2h_read_conf(struct rzv2h_pcie *pcie, int where)
{
	int shift = 8 * (where & 3);
	u32 val = rzv2h_pci_read_reg(pcie, PCIE_CONFIGURATION_REG + (where & ~3));

	return val >> shift;
}

u32 rzv2h_read_phy_conf(struct rzv2h_pcie *pcie, int where)
{
	int val;
	rzv2h_pci_write_reg(pcie, CFG_PIPEPHY_EN, PERMISSION_REG);
	val =  rzv2h_pci_read_reg(pcie, where);
	rzv2h_pci_write_reg(pcie, 0, PERMISSION_REG);
	return val;
}

void rzv2h_write_phy_conf(struct rzv2h_pcie *pcie, u32 data, int where)
{
	rzv2h_pci_write_reg(pcie, CFG_PIPEPHY_EN, PERMISSION_REG);
	rzv2h_pci_write_reg(pcie, data,  where);
	rzv2h_pci_write_reg(pcie, 0, PERMISSION_REG);
}

void rzv2h_write_conf(struct rzv2h_pcie *pcie, u32 data, int where)
{
	rzv2h_pci_write_reg(pcie, CFG_HWINIT_EN, PERMISSION_REG);
	rzv2h_pci_write_reg(pcie, data, PCIE_CONFIGURATION_REG + where);
	rzv2h_pci_write_reg(pcie, 0, PERMISSION_REG);
}

void rzv2h_pcie_set_outbound(struct rzv2h_pcie *pcie, int win,
			    struct resource_entry *window)
{
	/* Setup PCIe address space mappings for each resource */
	struct resource *res = window->res;
	resource_size_t size;
	u64 mask;
	u64 offset;

	size = resource_size(res);
	mask = size - 1;
	offset = res->start - window->offset;

	rzv2h_pci_write_reg(pcie, (u32)(res->start >> 32), PCIE_WINDOW_BASEU_REG(win));
	rzv2h_rmw(pcie, PCIE_WINDOW_BASEL_REG(win), 0xFFFFF000, (u32)(res->start & 0xFFFFF000));

	rzv2h_pci_write_reg(pcie, (u32)(mask >> 32), PCIE_WINDOW_MASKU_REG(win));
	rzv2h_pci_write_reg(pcie, (u32)(mask & 0xFFFFFFFF), PCIE_WINDOW_MASKL_REG(win));

	rzv2h_pci_write_reg(pcie, (u32)(offset >> 32), PCIE_DESTINATION_HI_REG(win));
	rzv2h_pci_write_reg(pcie, (u32)(offset & 0xFFFFFFFF), PCIE_DESTINATION_LO_REG(win));

	rzv2h_rmw(pcie, PCIE_WINDOW_BASEL_REG(win), PCIE_WINDOW_ENABLE, PCIE_WINDOW_ENABLE);
}

void rzv2h_pcie_ep_set_outbound(struct rzv2h_pcie *pcie, phys_addr_t cpu_addr, u64 pci_addr, size_t size)
{
	u64 mask;

	mask = size - 1;

	cpu_addr = (cpu_addr - 0x30000000) | 0x30000000UL;
	/* PW0 addr: PCIE_WINDOW_BASEU_REG: 0x1104   PCIE_WINDOW_BASEL_REG: 0x1100 */
	rzv2h_pci_write_reg(pcie, (u32)(cpu_addr >> 32), PCIE_WINDOW_BASEU_REG(0));
	rzv2h_rmw(pcie, PCIE_WINDOW_BASEL_REG(0), 0xFFFFF000, (u32)(cpu_addr & 0xFFFFF000));

	/* PW0 mask: PCIE_WINDOW_MASKU_REG: 0x110C PCIE_WINDOW_MASKL_REG: 0x1108 */
	rzv2h_pci_write_reg(pcie, (u32)(mask >> 32), PCIE_WINDOW_MASKU_REG(0));
	rzv2h_pci_write_reg(pcie, (u32)(mask & 0xFFFFFFFF), PCIE_WINDOW_MASKL_REG(0));

	/* PD0 addr: PCIE_DESTINATION_HI_REG: 0x1114 PCIE_DESTINATION_LO_REG: 0x1110 */
	rzv2h_pci_write_reg(pcie, (u32)(pci_addr >> 32), PCIE_DESTINATION_HI_REG(0));
	rzv2h_pci_write_reg(pcie, (u32)(pci_addr & 0xFFFFFFFF), PCIE_DESTINATION_LO_REG(0));

	rzv2h_rmw(pcie, PCIE_WINDOW_BASEL_REG(0), PCIE_WINDOW_ENABLE, PCIE_WINDOW_ENABLE);
}

void rzv2h_pcie_set_inbound_ep(struct rzv2h_pcie *pcie, u64 cpu_addr,
			   u64 pci_addr, u64 flags, int idx, bool host)
{
	/*
	 * Set up 64-bit inbound regions as the range parser doesn't
	 * distinguish between 32 and 64-bit types.
	 */

	rzv2h_pci_write_reg(pcie, 0x40000000, AXI_WINDOW_BASEL_REG(idx));
	rzv2h_pci_write_reg(pcie, 0x0, AXI_WINDOW_BASEU_REG(idx));
	pcie->save_reg.axi_window.base_u[idx] = upper_32_bits(pci_addr);
	rzv2h_pci_write_reg(pcie, 0x40000000, AXI_DESTINATIONL_REG(idx));
	rzv2h_pci_write_reg(pcie, 0x0, AXI_DESTINATIONU_REG(idx));
	pcie->save_reg.axi_window.dest_u[idx] = upper_32_bits(cpu_addr);
	rzv2h_pci_write_reg(pcie, 0xFFFFFFFF, AXI_WINDOW_MASKL_REG(idx));
	rzv2h_pci_write_reg(pcie, 0x0, AXI_WINDOW_MASKU_REG(idx));
	rzv2h_rmw(pcie, AXI_WINDOW_BASEL_REG(idx), AXI_WINDOW_ENABLE, AXI_WINDOW_ENABLE);
}

void rzv2h_pcie_set_inbound(struct rzv2h_pcie *pcie, u64 cpu_addr,
			   u64 pci_addr, u64 flags, int idx, bool host)
{
	/*
	 * Set up 64-bit inbound regions as the range parser doesn't
	 * distinguish between 32 and 64-bit types.
	 */
	rzv2h_pci_write_reg(pcie, lower_32_bits(pci_addr), AXI_WINDOW_BASEL_REG(idx));
	rzv2h_pci_write_reg(pcie, upper_32_bits(pci_addr), AXI_WINDOW_BASEU_REG(idx));
	pcie->save_reg.axi_window.base_u[idx] = upper_32_bits(pci_addr);
	rzv2h_pci_write_reg(pcie, lower_32_bits(cpu_addr), AXI_DESTINATIONL_REG(idx));
	rzv2h_pci_write_reg(pcie, upper_32_bits(cpu_addr), AXI_DESTINATIONU_REG(idx));
	pcie->save_reg.axi_window.dest_u[idx] = upper_32_bits(cpu_addr);
	rzv2h_pci_write_reg(pcie, lower_32_bits(flags), AXI_WINDOW_MASKL_REG(idx));
	rzv2h_pci_write_reg(pcie, upper_32_bits(flags), AXI_WINDOW_MASKU_REG(idx));
	rzv2h_rmw(pcie, AXI_WINDOW_BASEL_REG(idx), AXI_WINDOW_ENABLE, AXI_WINDOW_ENABLE);
}

u32 rzv2h_read_conf_ep(struct rzv2h_pcie *pcie, int where, u8 fn)
{
	int shift = 8 * (where & 3);
	u32 val = rzv2h_pci_read_reg(pcie, PCIE_CONFIGURATION_REG_EP(fn) + (where & ~3) );

	return val >> shift;
}

void rzv2h_write_conf_ep(struct rzv2h_pcie *pcie, u32 data, int where, u8 fn)
{
	rzv2h_pci_write_reg(pcie, CFG_HWINIT_EN, PERMISSION_REG );
	rzv2h_pci_write_reg(pcie, data, PCIE_CONFIGURATION_REG_EP(fn) + where );
	rzv2h_pci_write_reg(pcie, 0, PERMISSION_REG );
}

void rzv2h_pci_bit_write_reg(struct rzv2h_pcie *pcie, u32 val, unsigned long reg)
{
	u32 data = rzv2h_pci_read_reg(pcie, reg);
	writel(data | val , pcie->base + reg);
}

void rzv2h_pci_bit_clear_reg(struct rzv2h_pcie *pcie, u32 val, unsigned long reg)
{
	u32 data = rzv2h_pci_read_reg(pcie, reg);
	writel(data | (~val) , pcie->base + reg);
}
