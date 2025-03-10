// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe endpoint driver for Renesas RZ/V2H Series SoCs
 *  Copyright (c) 2023 Renesas Electronics Europe GmbH
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci-epc.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/arm-smccc.h>
#include <uapi/linux/psci.h>

#include "pcie-rzv2h.h"

/* Structure representing the PCIe interface */
struct rzv2h_pcie_endpoint {
	struct rzv2h_pcie		pcie;
	phys_addr_t			*ob_mapped_addr;
	struct pci_epc_mem_window	*ob_window;
	u8				max_functions;
	unsigned int			bar_to_atu[MAX_NR_INBOUND_MAPS_EP];
	unsigned long			*ib_window_map;
	u32				num_ib_windows;
	u32				num_ob_windows;
	int				channel;
};

/* ----------------------------------------------------
  PCIe Setting Function
-------------------------------------------------------*/
static void rzv2h_pcie_setting_config_ep(struct rzv2h_pcie *pcie, u8 fn)
{
	rzv2h_write_conf_ep(pcie, 0xfffff, PCI_EP_BARMSK00L_ADR, fn);
	rzv2h_write_conf_ep(pcie, 0x0, PCI_EP_BARMSK00U_ADR, fn);
	rzv2h_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_0001_EP_F0, PCI_EP_BSIZE00_01_ADR, fn);
	rzv2h_write_conf_ep(pcie, 0xfff, PCI_EP_BARMSK01L_ADR, fn);
	rzv2h_write_conf_ep(pcie, 0x0, PCI_EP_BARMSK01U_ADR, fn);
	rzv2h_write_conf_ep(pcie, 0xff, PCI_EP_BARMSK02L_ADR, fn);
	rzv2h_write_conf_ep(pcie, 0x0, PCI_EP_BARMSK02U_ADR, fn);
	rzv2h_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_0203_EP_F0, PCI_EP_BSIZE00_0203_ADR, fn);
	rzv2h_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_0405_EP_F0, PCI_EP_BSIZE00_0405_ADR, fn);
	rzv2h_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_06_EP_F0, PCIE_CFG_BASE_SIZE_06_EP_F0, fn);

	mdelay(20);
}

static int PCIE_EP_INT_Initialize(struct rzv2h_pcie *pcie)
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
	rzv2h_pci_write_reg(pcie, INT_MR_SET_EP, PCI_RC_MSGRCVIE_REG);		/* Set PCI_RC 0120h */

	return 0;
}

static int rzv2h_pcie_hw_init_ep(struct rzv2h_pcie *pcie, int channel)
{
	unsigned int timeout = 50;
	struct arm_smccc_res local_res;
	/* Set to the PCIe reset state, OFF clock setting   : step2 and step3 */

	void __iomem *cpg_base = ioremap_cache(0x10420000, 0x1000);
	iowrite32(0x00040000, cpg_base + 0x92c);
	iowrite32(0x00300000, cpg_base + 0x630);

	msleep(10);

	/* set lane mode : step4 */
	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1060, 0x100, 0, 0, 0, 0, 0, &local_res);

	/* set Release the reset, ON clock setting : step5 and step 6 */

	iowrite32(0x00040004, cpg_base + 0x92c);
	iowrite32(0x00300030, cpg_base + 0x630);
	iounmap(cpg_base);

	msleep(10);

	/* Set to the PCIe reset state.	: step 7*/
	rzv2h_pci_write_reg(pcie, 0x00000000, PCI_RESET_REG);

	/* Release the PCIe reset : step 8*/

	rzv2h_pci_write_reg(pcie, 0x18, PCI_RESET_REG);

	/* set BAR : step 9*/

	rzv2h_pcie_setting_config_ep(pcie, 1);

	/* SYS setting allow_enter_l1  : step 11 */

	if (!channel) {
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1020, 0x1, 0, 0, 0, 0, 0, &local_res);}
	else
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1050, 0x1, 0, 0, 0, 0, 0, &local_res);

	/* Set Interrupt settings             : step14  */
	PCIE_EP_INT_Initialize(pcie);

	/* Release the PCIe reset : step14 : RST_PS_B, RST_GP_B, RST_B : step 15 */
	rzv2h_pci_write_reg(pcie, 0x3B, PCI_RESET_REG);		/* Set PCI_RC 310h */

   /* Wait 500us over : step 16*/
	msleep(1);

   /* Release the PCIe reset : step17 : RST_OUT_B, RST_RSM_B) */
	rzv2h_pci_write_reg(pcie, 0x7F,  PCI_RESET_REG);		/* Set PCI_RC 310h */

	rzv2h_pci_write_reg(pcie, 0x3ff2,  MODE_SET_1_REG);						/* Set PCI_RC 318h */

	return 0;
}

/* description: this function confirm window pcie use
 * pcie v2h only 1 window memory 0, so this function return 0, and phys_base == addr = 0x30000000
 */

static int rzv2h_pcie_ep_get_window(struct rzv2h_pcie_endpoint *ep,
				   phys_addr_t addr)
{
	int i;

	for (i = 0; i < ep->num_ob_windows; i++)
		if (ep->ob_window[i].phys_base == addr)
			return i;

	return -EINVAL;
}

static int rzv2h_pcie_parse_outbound_ranges(struct rzv2h_pcie_endpoint *ep,
					   struct platform_device *pdev)
{
	struct rzv2h_pcie *pcie = &ep->pcie;
	char outbound_name[10];
	struct resource *res;
	unsigned int i = 0;

	ep->num_ob_windows = 0;
	for (i = 0; i < RZV2H_PCI_MAX_RESOURCES_EP; i++) {
		sprintf(outbound_name, "memory%u", i);
		res = platform_get_resource_byname(pdev,
						   IORESOURCE_MEM,
						   outbound_name);
		if (!res) {
			dev_err(pcie->dev, "missing outbound window %u\n", i);
			return -EINVAL;
		}
		if (!devm_request_mem_region(&pdev->dev, res->start,
					     resource_size(res),
					     outbound_name)) {
			dev_err(pcie->dev, "Cannot request memory region %s.\n",
				outbound_name);
			return -EIO;
		}

		ep->ob_window[i].phys_base = res->start;
		ep->ob_window[i].size = resource_size(res);
		/* controller doesn't support multiple allocation
		 * from same window, so set page_size to window size
		 */
		ep->ob_window[i].page_size = resource_size(res);
	}
	ep->num_ob_windows = i;

	return 0;
}

static int rzv2h_pcie_ep_get_pdata(struct rzv2h_pcie_endpoint *ep,
				  struct platform_device *pdev)
{
	struct rzv2h_pcie *pcie = &ep->pcie;
	struct pci_epc_mem_window *window;
	struct device *dev = pcie->dev;
	struct resource res;
	int err;

	err = of_address_to_resource(dev->of_node, 0, &res);
	if (err)
		return err;

	pcie->base = devm_ioremap_resource(dev, &res);

	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);

	ep->ob_window = devm_kcalloc(dev, RZV2H_PCI_MAX_RESOURCES_EP,
				     sizeof(*window), GFP_KERNEL);
	if (!ep->ob_window)
		return -ENOMEM;

	rzv2h_pcie_parse_outbound_ranges(ep, pdev);

	err = of_property_read_u8(dev->of_node, "max-functions",
				  &ep->max_functions);
	if (err < 0 || ep->max_functions > RZV2H_EPC_MAX_FUNCTIONS)
		ep->max_functions = RZV2H_EPC_MAX_FUNCTIONS;

	return 0;
}

static int rzv2h_pcie_ep_write_header(struct pci_epc *epc, u8 fn, u8 vfn,
				     struct pci_epf_header *hdr)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2h_pcie *pcie = &ep->pcie;
	u32 val;

	val |= hdr->deviceid << 16;
	val |= hdr->vendorid;

	// write Vendor/Device ID. - 0x6000
	rzv2h_write_conf_ep(pcie, val, PCI_EP_VID_ADR, 1);

	// write Revision ID / class code - 0x6008
	val = hdr->revid;
	val |= hdr->progif_code << 8;
	val |= hdr->subclass_code << 16;
	val |= hdr->baseclass_code << 24;
	rzv2h_write_conf_ep(pcie, val, PCI_EP_RID_CC_ADR, 1);

	// write Subsystem ID - 0x602c
	rzv2h_write_conf_ep(pcie, 0x1234ABCD, PCI_EP_SUBSYS_ID_ADR, 1);


	// write intx pin.
	if (hdr->interrupt_pin > PCI_INTERRUPT_INTA)
		return -EINVAL;
	val = rzv2h_read_conf_ep(pcie, PCI_EP_INTERRUPT_ADR, 1);
	val |= (hdr->interrupt_pin << 8);
	rzv2h_write_conf_ep(pcie, val, PCI_EP_INTERRUPT_ADR, 1);

	return 0;
}

static int rzv2h_pcie_ep_set_bar(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
				struct pci_epf_bar *epf_bar)
{
	int flags = epf_bar->flags | LAR_ENABLE | LAM_64BIT;
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
	u64 size = 1ULL << fls64(epf_bar->size - 1);
	dma_addr_t cpu_addr = epf_bar->phys_addr;
	enum pci_barno bar = epf_bar->barno;
	struct rzv2h_pcie *pcie = &ep->pcie;
	u32 mask;
	int idx;

	idx = find_first_zero_bit(ep->ib_window_map, ep->num_ib_windows);
	if (idx >= ep->num_ib_windows) {
		dev_err(pcie->dev, "no free inbound window\n");
		return -EINVAL;
	}

	if ((flags & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO)
		flags |= IO_SPACE;

	ep->bar_to_atu[bar] = idx;
	/* use 64-bit BARs */
	set_bit(idx, ep->ib_window_map);
	set_bit(idx + 1, ep->ib_window_map);

	if (cpu_addr > 0) {
		unsigned long nr_zeros = __ffs64(cpu_addr);
		u64 alignment = 1ULL << nr_zeros;

		size = min(size, alignment);
	}

	size = min(size, 1ULL << 32);

	mask = roundup_pow_of_two(size) - 1;
	mask &= ~0xf;

	rzv2h_pcie_set_inbound(pcie, cpu_addr,
			      0x0, mask | flags, idx, false);
	
	return 0;
}

static void rzv2h_pcie_ep_clear_bar(struct pci_epc *epc, u8 fn, u8 vfn,
				   struct pci_epf_bar *epf_bar)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
	enum pci_barno bar = epf_bar->barno;
	u32 atu_index = ep->bar_to_atu[bar];

	rzv2h_pcie_set_inbound(&ep->pcie, 0x0, 0x0, 0x0, bar, false);

	clear_bit(atu_index, ep->ib_window_map);
	clear_bit(atu_index + 1, ep->ib_window_map);
}

static int rzv2h_pcie_ep_set_msi(struct pci_epc *epc, u8 fn, u8 vfn,
				 u8 interrupts)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2h_pcie *pcie = &ep->pcie;
	u32 flags, ret;

	flags = rzv2h_read_conf(pcie, PCI_EP_MSICAP(0));
	flags |= interrupts << MSICAP0_MMESCAP_OFFSET;
	rzv2h_write_conf(pcie, flags, PCI_EP_MSICAP(0));
	ret = rzv2h_read_conf(pcie, PCI_EP_MSICAP(0));

	return 0;
}

static int rzv2h_pcie_ep_get_msi(struct pci_epc *epc, u8 fn, u8 vfn)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2h_pcie *pcie = &ep->pcie;
	u32 flags;

	flags = rzv2h_read_conf(pcie, PCI_EP_MSICAP(0));
	if (!(flags & MSICAP0_MSIE))
		return -EINVAL;

	return ((flags & MSICAP0_MMESE_MASK) >> MSICAP0_MMESE_OFFSET);
}

static int rzv2h_pcie_ep_map_addr(struct pci_epc *epc, u8 fn, u8 vfn,
				 phys_addr_t addr, u64 pci_addr, size_t size)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2h_pcie *pcie = &ep->pcie;
	struct resource_entry win;
	struct resource res;
	int window;
	u32 data;

	/* check if we have a link. */

	data = rzv2h_pci_read_reg(pcie, PCI_RC_LINKCS);
	
	window = rzv2h_pcie_ep_get_window(ep, addr); /* window 0 use */

	if (window < 0) {
		dev_err(pcie->dev, "failed to get corresponding window\n");
		return -EINVAL;
	}

	memset(&win, 0x0, sizeof(win));
	memset(&res, 0x0, sizeof(res));
	res.start = pci_addr;
	res.end = pci_addr + size - 1;
	res.flags = IORESOURCE_MEM;
	win.res = &res;

	rzv2h_pcie_ep_set_outbound(pcie, addr, pci_addr, size);
	ep->ob_mapped_addr[window] = addr;

	return 0;
}

static void rzv2h_pcie_ep_unmap_addr(struct pci_epc *epc, u8 fn, u8 vfn,
				    phys_addr_t addr)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct resource_entry win;
	struct resource res;
	int idx;

	for (idx = 0; idx < ep->num_ob_windows; idx++)
		if (ep->ob_mapped_addr[idx] == addr)
			break;

	if (idx >= ep->num_ob_windows)
		return;

	memset(&win, 0x0, sizeof(win));
	memset(&res, 0x0, sizeof(res));
	win.res = &res;
	//rzv2h_pcie_ep_set_outbound(&ep->pcie, idx, &win);

	ep->ob_mapped_addr[idx] = 0;
}

static int rzv2h_pcie_ep_assert_intx(struct rzv2h_pcie_endpoint *ep,
				    u8 fn, u8 intx)
{
	struct rzv2h_pcie *pcie = &ep->pcie;
	u32 val;
	struct arm_smccc_res local_res;

	/* Check MSI enable bit */
	val = rzv2h_read_conf_ep(pcie, PCI_EP_MSICAP(0), 1);
	if ((val & MSICAP0_MSIE)) {
		dev_err(pcie->dev, "MSI is enabled, cannot assert INTx\n");
		return -EINVAL;
	}

	val = rzv2h_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1000, 0x01, 0, 0, 0, 0, 0, &local_res);
	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1000, 0x00, 0, 0, 0, 0, 0, &local_res);

	return 0;
}

static int rzv2h_pcie_ep_assert_msi(struct rzv2h_pcie_endpoint *ep,
				   u8 fn, u8 interrupt_num)
{
	struct rzv2h_pcie *pcie = &ep->pcie;
	u64 pci_addr, pci_addr_mask = 0xff;
	u32 val, ret, timeout = 200;
	u16 data = 0xffff;
	struct arm_smccc_res local_res;

	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1004, 0, 0, 0, 0, 0, 0, &local_res);
	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1000, 0, 0, 0, 0, 0, 0, &local_res);

	ret = rzv2h_read_conf_ep(pcie, PCI_EP_COM_STA, 1);
	ret &= PCI_EP_IRQ_STA;
	
	if (ret) {
		dev_err(pcie->dev, "INTx is not enabled, cannot assert MSI\n");
		return -EINVAL;
	}

	/* Check MSI enable bit */
	val = rzv2h_read_conf_ep(pcie, PCI_EP_MSICAP(0), 1);
	if (!(val & MSICAP0_MSIE)) {
		dev_err(pcie->dev, "MSI is not enabled, cannot assert MSI\n");
		return -EINVAL;
	}

	/* Get the number of enabled MSIs */

	pci_addr = rzv2h_pci_read_reg(pcie, PCI_MSI_ADDR_HI);
	pci_addr <<= 32;
	pci_addr |= rzv2h_pci_read_reg(pcie, PCI_MSI_ADDR_LO);
	pci_addr &= GENMASK_ULL(63, 2);

	//writel(data, ep->ob_window[0].phys_base + (pci_addr & pci_addr_mask));
	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1010, 0x1F, 0, 0, 0, 0, 0, &local_res);
	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1008, 0x1F, 0, 0, 0, 0, 0, &local_res);
	arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1004, 0x1F, 0, 0, 0, 0, 0, &local_res);
	ret = rzv2h_read_conf_ep(pcie, PCI_EP_MSICAP(0), 1);
	ret |= MSICAP0_MSIE;
	rzv2h_write_conf_ep(pcie, ret, PCI_EP_MSICAP(0), 1);

	return 0;
}

static int rzv2h_pcie_ep_raise_irq(struct pci_epc *epc, u8 fn, u8 vfn,
				  enum pci_epc_irq_type type,
				  u16 interrupt_num)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return rzv2h_pcie_ep_assert_intx(ep, 1, 0);

	case PCI_EPC_IRQ_MSI:
		return rzv2h_pcie_ep_assert_msi(ep, 1, interrupt_num);

	default:
		return -EINVAL;
	}
}

static int rzv2h_pcie_ep_start(struct pci_epc *epc)
{

	return 0;
}

static void rzv2h_pcie_ep_stop(struct pci_epc *epc)
{
	struct rzv2h_pcie_endpoint *ep = epc_get_drvdata(epc);
}

static const struct pci_epc_features rzv2h_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
	/* use 64-bit BARs so mark BAR[1,3,5] as reserved */
	.reserved_bar = 1 << BAR_1 | 1 << BAR_3 | 1 << BAR_5,
	.bar_fixed_64bit = 1 << BAR_0 | 1 << BAR_2 | 1 << BAR_4,
	.bar_fixed_size[0] = SZ_1M,
	.bar_fixed_size[2] = 4096,
	.bar_fixed_size[4] = 256,
};

static const struct pci_epc_features*
rzv2h_pcie_ep_get_features(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	return &rzv2h_pcie_epc_features;
}

static const struct pci_epc_ops rzv2h_pcie_epc_ops = {
	.write_header	= rzv2h_pcie_ep_write_header,
	.set_bar	= rzv2h_pcie_ep_set_bar,
	.clear_bar	= rzv2h_pcie_ep_clear_bar,
	.set_msi	= rzv2h_pcie_ep_set_msi,
	.get_msi	= rzv2h_pcie_ep_get_msi,
	.map_addr	= rzv2h_pcie_ep_map_addr,
	.unmap_addr	= rzv2h_pcie_ep_unmap_addr,
	.raise_irq	= rzv2h_pcie_ep_raise_irq,
	.start		= rzv2h_pcie_ep_start,
	.stop		= rzv2h_pcie_ep_stop,
	.get_features	= rzv2h_pcie_ep_get_features,
};

static const struct of_device_id rzv2h_pcie_ep_of_match[] = {
	{ .compatible = "renesas,rzv2h-pcie-ep", },
	{ .compatible = "renesas,rzv2n-pcie-ep", },
	{},
	{ },
};

static int rzv2h_pcie_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzv2h_pcie_endpoint *ep;
	struct rzv2h_pcie *pcie;
	struct pci_epc *epc;
	int err, channel;
	struct arm_smccc_res local_res;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	
	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	pcie = &ep->pcie;
	pcie->dev = dev;

	pm_runtime_enable(dev);
	err = pm_runtime_resume_and_get(dev);
	if (err < 0) {
		dev_err(dev, "pm_runtime_resume_and_get failed\n");
		goto err_pm_disable;
	}

	err = of_property_read_u32(dev->of_node, "pcie,channel", &channel);
	if (err) {
		dev_err(pcie->dev, "%pOF: No pcie,channel property found\n",
			dev->of_node);
		return -EINVAL;
	}

	ep->channel = channel;
	if (ep->channel >= PCIE_MAX_CHANNEL) {
		dev_err(pcie->dev, "%pOF: Invalid pcie,channel '%u'\n",
				dev->of_node, ep->channel);
		return -EINVAL;
	}

	if (!ep->channel)
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1024, 0x0, 0, 0, 0, 0, 0, &local_res);
	else
		arm_smccc_smc(RZ_SIP_SVC_SET_SYSPCIE, 0x1054, 0x0, 0, 0, 0, 0, 0, &local_res);

	err = rzv2h_pcie_ep_get_pdata(ep, pdev);
	if (err < 0) {
		dev_err(dev, "failed to request resources: %d\n", err);
		goto err_pm_put;
	}

	ep->num_ib_windows = MAX_NR_INBOUND_MAPS_EP;
	ep->ib_window_map =
			devm_kcalloc(dev, BITS_TO_LONGS(ep->num_ib_windows),
				     sizeof(long), GFP_KERNEL);
	if (!ep->ib_window_map) {
		err = -ENOMEM;
		dev_err(dev, "failed to allocate memory for inbound map\n");
		goto err_pm_put;
	}

	ep->ob_mapped_addr = devm_kcalloc(dev, ep->num_ob_windows,
					  sizeof(*ep->ob_mapped_addr),
					  GFP_KERNEL);
	if (!ep->ob_mapped_addr) {
		err = -ENOMEM;
		dev_err(dev, "failed to allocate memory for outbound memory pointers\n");
		goto err_pm_put;
	}

	epc = devm_pci_epc_create(dev, &rzv2h_pcie_epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		err = PTR_ERR(epc);
		goto err_pm_put;
	}

	epc->max_functions = ep->max_functions;
	epc_set_drvdata(epc, ep);

	rzv2h_pcie_hw_init_ep(pcie, ep->channel);

	err = pci_epc_multi_mem_init(epc, ep->ob_window, ep->num_ob_windows);

	if (err < 0) {
		dev_err(dev, "failed to initialize the epc memory space\n");
		goto err_pm_put;
	}

	return 0;

err_pm_put:
	pm_runtime_put(dev);

err_pm_disable:
	pm_runtime_disable(dev);

	return err;
}

static struct platform_driver rzv2h_pcie_ep_driver = {
	.driver = {
		.name = "rzv2h-pcie-ep",
		.of_match_table = rzv2h_pcie_ep_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = rzv2h_pcie_ep_probe,
};
builtin_platform_driver(rzv2h_pcie_ep_driver);
