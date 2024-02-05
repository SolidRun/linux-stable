// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe endpoint driver for Renesas RZ/V2M Series SoCs
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

#include "pcie-rzv2m.h"

/* Structure representing the PCIe interface */
struct rzv2m_pcie_endpoint {
	struct rzv2m_pcie	pcie;
	phys_addr_t			*ob_mapped_addr;
	struct pci_epc_mem_window	*ob_window;
	u8					max_functions;
	unsigned int		bar_to_atu[MAX_NR_INBOUND_MAPS];
	unsigned long		*ib_window_map;
	u32					num_ib_windows;
	u32					num_ob_windows;
};

static int pcie_pm_wakeup_l1(struct rzv2m_pcie *pcie, u8 fn);
static int pcie_pm_wakeup_l2(struct rzv2m_pcie *pcie, u8 fn);

/* ----------------------------------------------------
  PCIe Setting Function
-------------------------------------------------------*/
static void rzv2m_pcie_setting_config_ep(struct rzv2m_pcie *pcie, u8 fn)
{
   rzv2m_pci_write_reg(pcie, RESET_CONFIG_DEASSERT, PCI_RESET_REG);

	// Configuration space(Root complex) setting
   // Vendor and Device ID      : PCI Express Configuration Registers Adr 6000h
	rzv2m_write_conf_ep(pcie,
			( (PCIE_CONF_DEVICE_ID << 16) | 
			  (PCIE_CONF_VENDOR_ID) ),
			PCI_EP_VID_ADR, fn);

   // Revision ID and Class Code : PCI Express Configuration Registers Adr 6008h
	rzv2m_write_conf_ep(pcie,
			( (PCIE_CONF_BASE_CLASS   << 24) |
			  (PCIE_CONF_SUB_CLASS    << 16) |
			  (PCIE_CONF_PROGRAMING_IF << 8) |
			  (PCIE_CONF_REVISION_ID) ),
			PCI_EP_RID_CC_ADR, fn);

	rzv2m_write_conf_ep(pcie, PCIE_CFG_BAR_MASK0_L_EP_F0, PCI_EP_BARMSK00L_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BAR_MASK0_H_EP_F0, PCI_EP_BARMSK00U_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_0001_EP_F0, PCI_EP_BSIZE00_01_ADR, fn);
	
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BAR_MASK1_L_EP_F0, PCI_EP_BARMSK01L_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BAR_MASK1_H_EP_F0, PCI_EP_BARMSK01U_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BAR_MASK2_L_EP_F0, PCI_EP_BARMSK02L_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BAR_MASK2_H_EP_F0, PCI_EP_BARMSK02U_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_0203_EP_F0, PCI_EP_BSIZE00_0203_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_0405_EP_F0, PCI_EP_BSIZE00_0405_ADR, fn);
	rzv2m_write_conf_ep(pcie, PCIE_CFG_BASE_SIZE_06_EP_F0, PCIE_CFG_BASE_SIZE_06_EP_F0, fn);
}

static int PCIE_CFG_Initialize_ep(struct rzv2m_pcie *pcie)
{
   // Vendor and Device ID      : PCI Express Configuration Registers Adr 6000h
	rzv2m_write_conf_ep(pcie,
			( (PCIE_CONF_DEVICE_ID << 16) | 
			  (PCIE_CONF_VENDOR_ID) ),
			PCI_EP_VID_ADR, 0);

   // Revision ID and Class Code : PCI Express Configuration Registers Adr 6008h
	rzv2m_write_conf_ep(pcie,
			( (PCIE_CONF_BASE_CLASS   << 24) |
			  (PCIE_CONF_SUB_CLASS    << 16) |
			  (PCIE_CONF_PROGRAMING_IF << 8) |
			  (PCIE_CONF_REVISION_ID) ),
			PCI_EP_RID_CC_ADR, 0);

   // Base Address Register Mask00 (Lower) (Function #1) : PCI Express Configuration Registers Adr 60A0h
	rzv2m_write_conf_ep(pcie, BASEADR_MKL_ALLM, PCI_RC_BARMSK00L_ADR, 0);

   // Base Address Register Mask00 (upper) (Function #1) : PCI Express Configuration Registers Adr 60A4h
	rzv2m_write_conf_ep(pcie, BASEADR_MKU_ALLM, PCI_RC_BARMSK00U_ADR, 0);

   // Base Size 00/01 : PCI Express Configuration Registers Adr 60C8h
	rzv2m_write_conf_ep(pcie, BASESZ_INIT, PCI_RC_BSIZE00_01_ADR, 0);

   // Bus Number : PCI Express Configuration Registers Adr 6018h
	rzv2m_write_conf_ep(pcie,
			( (PCIE_CONF_SUBORDINATE_BUS << 16) |
			  (PCIE_CONF_SECOUNDARY_BUS  <<  8) |
			  (PCIE_CONF_PRIMARY_BUS) ),
			PCI_PRIMARY_BUS, 0);

	rzv2m_write_conf_ep(pcie,
			( (PCIE_CONF_MEMORY_LIMIT << 16) | 
			  (PCIE_CONF_MEMORY_BASE) ),
			PCI_MEMORY_BASE, 0);

	rzv2m_write_conf_ep(pcie, PM_CAPABILITIES_INIT, PCI_PM_CAPABILITIES, 0);

  return 0;
}

/******************************************************************************
* Function Name: sys_pcie_set_pme_status
* Description  : SYS driver Setting CFG_PMCSR_PME_STATUS_F0/1.
* Arguments    : ast_flag - 
*                    Assert flag.(true:Assert / false:Deassert)
*              : func_no - 
*                    Function No.
* Return Value :  -
******************************************************************************/
void v2m_sys_pcie_set_pme_status(struct rzv2m_pcie *pcie, _Bool ast_flag, uint8_t func_no)
{
    if (false != ast_flag)
    {
        sys_pcie_reg_bit_set(pcie, PME_STS_FN(func_no), PCI_PME_STS);
    }
    else
    {
        sys_pcie_reg_bit_clear(pcie, PME_STS_FN(func_no), PCI_PME_STS);
    }

} /* End of function sys_pcie_set_pme_status() */


/******************************************************************************
* Function Name: PCIE_PM_Recovery
* Description  : PCIe driver Request PM(L2) recovery.
* Arguments    :  - 
* Return Value : result -
*                    Execution result
******************************************************************************/
static int PCIE_PM_Recovery(struct rzv2m_pcie *pcie)
{
	uint32_t reg_store_area[PCIE_CFGREG_HEADER_SIZE / 4];
	uint32_t cmd_sts_store_area;
	uint32_t ltssm_tmp;
	uint16_t reg_num;
	uint8_t idx;
	uint32_t reset_bit;
	int err=0;

	ltssm_tmp = rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG);
	ltssm_tmp = ((ltssm_tmp & LTSSM_STATE_MASK) >> LTSSM_STATE_SHIFT);
	if ((ltssm_tmp & (~0x01)) != PCIE_LTSSM_L2)
	{
		return -EINVAL;
	}

	/* Store Configuration Registers : Configuration Space */
	idx = 0;
	for (reg_num = PCIE_CFGREG_HEADER_START;
		 reg_num < PCIE_CFGREG_HEADER_END;
		 reg_num += 4)
	{
		reg_store_area[idx] = rzv2m_read_conf_ep(pcie, reg_num, 1);
		idx++;
	}
	
	/* Store Configuration Registers : Command and Status */
	cmd_sts_store_area = rzv2m_read_conf_ep(pcie, PCI_COMMAND, 1);

	/* Assert reset */
	reset_bit = (RST_PS_B | RST_CFG_B | RST_GP_B | RST_B);
	if (1 == PCIE_CFG_RECOVERY_EP_RESET)
	{
		reset_bit |= RST_OUT_B;
	}
	rzv2m_pci_bit_clear_reg(pcie, reset_bit, PCI_RESET_REG);

	msleep(PCIE_PRV_RESET_WAIT);
	
	/* Reset deassert for configuration register */
	rzv2m_pci_bit_write_reg(pcie, RST_CFG_B, PCI_RESET_REG);
	reset_bit &= (~RST_CFG_B);
	
	/* Restore Configuration Registers : Configuration Space */
	idx = 0;
	for (reg_num = PCIE_CFGREG_HEADER_START;
		 reg_num < PCIE_CFGREG_HEADER_END;
		 reg_num += 4)
	{
		rzv2m_write_conf_ep(pcie, reg_num, reg_store_area[idx], 1);
		idx++;
	}
	
	/* Restore Configuration Registers : Command and Status */
	rzv2m_write_conf_ep(pcie, PCI_COMMAND, cmd_sts_store_area, 1);

	/* Restart PCIe driver */
	rzv2m_pci_bit_write_reg(pcie, reset_bit, PCI_RESET_REG);

	rzv2m_pci_bit_clear_reg(pcie, UI_ENTER_L2, PCIE_CORE_CONTROL_1_REG);

	return err;
} /* End of function PCIE_PM_Recovery() */


/******************************************************************************
* Function Name: pcie_check_state_stable
* Description  : PCIe driver Check LTSSM state stable.
* Arguments    :  - 
* Return Value : sts -
*                    LTSSM
******************************************************************************/
static rzv2m_pcie_ltssm_t pcie_check_state_stable(struct rzv2m_pcie *pcie)
{
	rzv2m_pcie_ltssm_t sts;
	volatile uint32_t ltssm_tmp1;
	volatile uint32_t ltssm_tmp2;
	uint8_t i;
	
	i = PCIE_PRV_CHECK_CNT;
	sts = PCIE_LTSSM_UNKNOWN;
	while(i)
	{
		ltssm_tmp1 = ((rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG)
					   & LTSSM_ST_ALL_MASK) >> LTSSM_ST_ALL_SHIFT);
		ltssm_tmp2 = ((rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG)
					   & LTSSM_ST_ALL_MASK) >> LTSSM_ST_ALL_SHIFT);

		/* L2.idle check */
		if ((LTSSM_ST_L2_IDLE == ltssm_tmp1) &&
			(LTSSM_ST_L2_IDLE == ltssm_tmp2))
		{
			sts = PCIE_LTSSM_L2;
			break;
		}

		/* LTSSM = upper 5bit[14:10] */
		ltssm_tmp1 >>= 2;
		ltssm_tmp2 >>= 2;
		if(ltssm_tmp1 == ltssm_tmp2)
		{
			/* L1 check */
			if (PCIE_LTSSM_L1 == ltssm_tmp1)
			{
				sts = PCIE_LTSSM_L1;
			}
			else
			{
				sts = ltssm_tmp1 & (~0x03);
			}
			break;
		}
		i--;
	}

	return sts;
} /* End of function pcie_check_state_stable() */

/******************************************************************************
* Function Name: PCIE_PM_Wakeup
* Description  : PCIe driver Request wakeup L1/L2.
* Arguments    : fn - 
*                    Function No.
* Return Value : result -
*                    Execution result
******************************************************************************/
static int PCIE_PM_Wakeup(struct rzv2m_pcie *pcie, u8 fn)
{
	rzv2m_pcie_ltssm_t sts;
	int err=0;

	if (RZV2M_EPC_MAX_FUNCTIONS <= fn)
	{
		return -EINVAL;
	}

	sts = pcie_check_state_stable(pcie);
	switch (sts)
	{
		case PCIE_LTSSM_L1:
			{
				err = pcie_pm_wakeup_l1(pcie, fn);
			}
		break;

		case PCIE_LTSSM_L2:
			{
				err = pcie_pm_wakeup_l2(pcie, fn);
			}
		break;

		default:
			{
				err = -EINVAL;
			}
		break;
		
	}
	
	return err;
} /* End of function PCIE_PM_Wakeup() */

/******************************************************************************
* Function Name: pcie_pm_wakeup_l1
* Description  : PCIe driver initialize.
* Description  : PCIe driver Request wakeup L1.
* Arguments    : fn - 
*                    Function No.
* Return Value : result -
*                    Execution result
******************************************************************************/
static int pcie_pm_wakeup_l1(struct rzv2m_pcie *pcie, uint8_t fn)
{
	u32	pm_sts;
	u8	cnt;
	u8	loop_flg = 1;

	pm_sts = rzv2m_read_conf_ep(pcie, PCI_EP_OWN_PM_STS_CTRL_REG, fn);
	if (0 == (pm_sts & PCI_PME_ENABLE))
	{
		return -EINVAL;
	}
	
	/* PCI PME status assert */
	v2m_sys_pcie_set_pme_status(pcie, true, fn);
	
	while (loop_flg)
	{
		/* Toggle PME TIM */
		sys_pcie_tgl_pme_tim(pcie);
		cnt = 100;
		while (cnt)
		{
			if( true == sys_pcie_get_pme_sts_clr(pcie, fn))
			{
				loop_flg = 0;
				break;
			}
			msleep(1);
			cnt--;
		}
	}
	
	/* PCI PME status deassert */
	v2m_sys_pcie_set_pme_status(pcie, false, fn);

	return 0;
} /* End of function pcie_pm_wakeup_l1() */


/******************************************************************************
* Function Name: pcie_pm_wakeup_l2
* Description  : PCIe driver Request wakeup L2.
* Arguments    : fn - 
*                    Function No.
* Return Value : result -
*                    Execution result
******************************************************************************/
static int pcie_pm_wakeup_l2(struct rzv2m_pcie *pcie, uint8_t fn)
{
	/* PCI PME status assert */
	v2m_sys_pcie_set_pme_status(pcie, true, fn);

	/* Send Beacon */
	rzv2m_pci_bit_clear_reg(pcie, RST_B, PCI_RESET_REG);
	msleep(PCIE_PRV_RESET_WAIT);
	rzv2m_pci_bit_write_reg(pcie, RST_B, PCI_RESET_REG);

	/* Wait linkup */
	while (DL_DOWN_STATUS == (rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_1_REG) & DL_DOWN_STATUS))
	{
		; /* Do Nothing*/
	}

	/* PCI PME status deassert */
	v2m_sys_pcie_set_pme_status(pcie, false, fn);

	return 0;
} /* End of function pcie_pm_wakeup_l2() */


static int rzv2m_pcie_hw_init_ep(struct rzv2m_pcie *pcie)
{
	unsigned int timeout = 50;

	/* Set to the PCIe reset state   : step6 */
	rzv2m_pci_write_reg(pcie, RESET_ALL_ASSERT, PCI_RESET_REG);			/* Set PCI_RC 310h */

	/* Set PMA and Phy Register for Lane0 : step7, 9 */
	PCIE_phyInitialize_L0(pcie);

	/* Set PMA and Phy Register for Lane1 : step8, 9 */
	PCIE_phyInitialize_L1(pcie);

	msleep(20);

	/* Release the PCIe reset : step10 : RST_LOAD_B, RST_CFG_B)*/
	rzv2m_pci_write_reg(pcie, RESET_LOAD_CFG_RELEASE, PCI_RESET_REG);	/* Set PCI_RC 310h */

	/* Setting of HWINT related registers : step11 */
	PCIE_CFG_Initialize_ep(pcie);

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

static int rzv2m_pcie_ep_get_window(struct rzv2m_pcie_endpoint *ep,
				   phys_addr_t addr)
{
	int i;

	for (i = 0; i < ep->num_ob_windows; i++)
		if (ep->ob_window[i].phys_base == addr)
			return i;

	return -EINVAL;
}

static int rzv2m_pcie_parse_outbound_ranges(struct rzv2m_pcie_endpoint *ep,
					   struct platform_device *pdev)
{
	struct rzv2m_pcie *pcie = &ep->pcie;
	char outbound_name[10];
	struct resource *res;
	unsigned int i = 0;

	ep->num_ob_windows = 0;
	for (i = 0; i < RZV2M_PCI_MAX_RESOURCES; i++) {
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

static int rzv2m_pcie_ep_get_pdata(struct rzv2m_pcie_endpoint *ep,
				  struct platform_device *pdev)
{
	struct rzv2m_pcie *pcie = &ep->pcie;
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

	ep->ob_window = devm_kcalloc(dev, RZV2M_PCI_MAX_RESOURCES,
				     sizeof(*window), GFP_KERNEL);
	if (!ep->ob_window)
		return -ENOMEM;

	rzv2m_pcie_parse_outbound_ranges(ep, pdev);

	err = of_property_read_u8(dev->of_node, "max-functions",
				  &ep->max_functions);
	if (err < 0 || ep->max_functions > RZV2M_EPC_MAX_FUNCTIONS)
		ep->max_functions = RZV2M_EPC_MAX_FUNCTIONS;

	return 0;
}

static int rzv2m_pcie_ep_write_header(struct pci_epc *epc, u8 fn,
				     struct pci_epf_header *hdr)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2m_pcie *pcie = &ep->pcie;
	u32 val;

	if (!fn)
		val = hdr->vendorid;
	else
		val = rzv2m_read_conf_ep(pcie, PCI_EP_VID_ADR, fn);
	val |= hdr->deviceid << 16;
	rzv2m_write_conf_ep(pcie, val, PCI_EP_VID_ADR, fn);

	val = hdr->revid;
	val |= hdr->progif_code << 8;
	val |= hdr->subclass_code << 16;
	val |= hdr->baseclass_code << 24;
	rzv2m_write_conf_ep(pcie, val, PCI_EP_RID_CC_ADR, fn);

	if (!fn)
		val = hdr->subsys_vendor_id;
	else
		val = rzv2m_read_conf_ep(pcie, PCI_EP_SUBSYS_ID_ADR, fn);
	val |= hdr->subsys_id << 16;
	rzv2m_write_conf_ep(pcie, val, PCI_EP_SUBSYS_ID_ADR, fn);

	if (hdr->interrupt_pin > PCI_INTERRUPT_INTA)
		return -EINVAL;
	val = rzv2m_read_conf_ep(pcie, PCI_EP_INTERRUPT_ADR, fn);
	val |= (hdr->interrupt_pin << 8);
	rzv2m_write_conf_ep(pcie, val, PCI_EP_INTERRUPT_ADR, fn);

	return 0;
}

static int rzv2m_pcie_ep_set_bar(struct pci_epc *epc, u8 func_no,
				struct pci_epf_bar *epf_bar)
{
	int flags = epf_bar->flags | LAR_ENABLE | LAM_64BIT;
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
	u64 size = 1ULL << fls64(epf_bar->size - 1);
	dma_addr_t cpu_addr = epf_bar->phys_addr;
	enum pci_barno bar = epf_bar->barno;
	struct rzv2m_pcie *pcie = &ep->pcie;
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

	rzv2m_pcie_set_inbound(pcie, cpu_addr,
			      0x0, mask | flags, idx, false, 0);

	return 0;
}

static void rzv2m_pcie_ep_clear_bar(struct pci_epc *epc, u8 fn,
				   struct pci_epf_bar *epf_bar)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
	enum pci_barno bar = epf_bar->barno;
	u32 atu_index = ep->bar_to_atu[bar];

	rzv2m_pcie_set_inbound(&ep->pcie, 0x0, 0x0, 0x0, bar, false, 0);

	clear_bit(atu_index, ep->ib_window_map);
	clear_bit(atu_index + 1, ep->ib_window_map);
}

static int rzv2m_pcie_ep_set_msi(struct pci_epc *epc, u8 fn, u8 interrupts)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2m_pcie *pcie = &ep->pcie;
	u32 flags;

	flags = rzv2m_read_conf(pcie, PCI_EP_MSICAP(fn));
	flags |= interrupts << MSICAP0_MMESCAP_OFFSET;
	rzv2m_write_conf(pcie, flags, PCI_EP_MSICAP(fn));

	return 0;
}

static int rzv2m_pcie_ep_get_msi(struct pci_epc *epc, u8 fn)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2m_pcie *pcie = &ep->pcie;
	u32 flags;

	flags = rzv2m_read_conf(pcie, PCI_EP_MSICAP(fn));
	if (!(flags & MSICAP0_MSIE))
		return -EINVAL;

	return ((flags & MSICAP0_MMESE_MASK) >> MSICAP0_MMESE_OFFSET);
}

static int rzv2m_pcie_ep_map_addr(struct pci_epc *epc, u8 fn,
				 phys_addr_t addr, u64 pci_addr, size_t size)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2m_pcie *pcie = &ep->pcie;
	struct resource_entry win;
	struct resource res;
	int window;
	u32 data;

	/* check if we have a link. */
	data = rzv2m_pci_read_reg(pcie, PCIE_CORE_STATUS_2_REG);

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
				dev_err(pcie->dev, "PCIe x%d: link not up\n",data);
				break;
	}

	window = rzv2m_pcie_ep_get_window(ep, addr);
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

	rzv2m_pcie_set_outbound(pcie, window, &win, 0);

	ep->ob_mapped_addr[window] = addr;

	return 0;
}

static void rzv2m_pcie_ep_unmap_addr(struct pci_epc *epc, u8 fn,
				    phys_addr_t addr)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
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
	rzv2m_pcie_set_outbound(&ep->pcie, idx, &win, 0);

	ep->ob_mapped_addr[idx] = 0;
}

static int rzv2m_pcie_ep_assert_intx(struct rzv2m_pcie_endpoint *ep,
				    u8 fn, u8 intx)
{
	struct rzv2m_pcie *pcie = &ep->pcie;
	u32 val;

	/* Check MSI enable bit */
	val = rzv2m_read_conf_ep(pcie, PCI_EP_MSICAP(0), fn);
	if ((val & MSICAP0_MSIE)) {
		dev_err(pcie->dev, "MSI is enabled, cannot assert INTx\n");
		return -EINVAL;
	}

	val = rzv2m_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	if ((val & INTX_RECEIVE_INTERRUPT_ENABLE)) {
		dev_err(pcie->dev, "INTx is already asserted\n");
		return -EINVAL;
	}

	rzv2m_rmw(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG,
					 INTX_RECEIVE_INTERRUPT_ENABLE,
					 INTX_RECEIVE_INTERRUPT_ENABLE );

	return 0;
}

static int rzv2m_pcie_ep_assert_msi(struct rzv2m_pcie *pcie,
				   u8 fn, u8 interrupt_num)
{
	u32 val;

	/* Check MSI enable bit */
	val = rzv2m_read_conf_ep(pcie, PCI_EP_MSICAP(0), fn);
	if (!(val & MSICAP0_MSIE)) {
		dev_err(pcie->dev, "MSI is not enabled, cannot assert MSI\n");
		return -EINVAL;
	}

	val = rzv2m_pci_read_reg(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG);
	if ((val & MSI_RECEIVE_INTERRUPT_ENABLE)) {
		dev_err(pcie->dev, "MSI is already asserted\n");
		return -EINVAL;
	}

	rzv2m_rmw(pcie, PCI_INTX_RCV_INTERRUPT_ENABLE_REG,
					 MSI_RECEIVE_INTERRUPT_ENABLE,
					 MSI_RECEIVE_INTERRUPT_ENABLE );

	return 0;
}

static int rzv2m_pcie_ep_raise_irq(struct pci_epc *epc, u8 fn,
				  enum pci_epc_irq_type type,
				  u16 interrupt_num)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return rzv2m_pcie_ep_assert_intx(ep, fn, 0);

	case PCI_EPC_IRQ_MSI:
		return rzv2m_pcie_ep_assert_msi(&ep->pcie, fn, interrupt_num);

	default:
		return -EINVAL;
	}
}

static int rzv2m_pcie_ep_start(struct pci_epc *epc)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);
	struct rzv2m_pcie *pcie = &ep->pcie;

	PCIE_PM_Wakeup(pcie, 0);

	return 0;
}

static void rzv2m_pcie_ep_stop(struct pci_epc *epc)
{
	struct rzv2m_pcie_endpoint *ep = epc_get_drvdata(epc);

}

static const struct pci_epc_features rzv2m_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
	/* use 64-bit BARs so mark BAR[1,3,5] as reserved */
	.reserved_bar = 1 << BAR_1 | 1 << BAR_3 | 1 << BAR_5,
	.bar_fixed_64bit = 1 << BAR_0 | 1 << BAR_2 | 1 << BAR_4,
	.bar_fixed_size[0] = 128,
	.bar_fixed_size[2] = 256,
	.bar_fixed_size[4] = 256,
};

static const struct pci_epc_features*
rzv2m_pcie_ep_get_features(struct pci_epc *epc, u8 func_no)
{
	return &rzv2m_pcie_epc_features;
}

static const struct pci_epc_ops rzv2m_pcie_epc_ops = {
	.write_header	= rzv2m_pcie_ep_write_header,
	.set_bar	= rzv2m_pcie_ep_set_bar,
	.clear_bar	= rzv2m_pcie_ep_clear_bar,
	.set_msi	= rzv2m_pcie_ep_set_msi,
	.get_msi	= rzv2m_pcie_ep_get_msi,
	.map_addr	= rzv2m_pcie_ep_map_addr,
	.unmap_addr	= rzv2m_pcie_ep_unmap_addr,
	.raise_irq	= rzv2m_pcie_ep_raise_irq,
	.start		= rzv2m_pcie_ep_start,
	.stop		= rzv2m_pcie_ep_stop,
	.get_features	= rzv2m_pcie_ep_get_features,
};

static const struct of_device_id rzv2m_pcie_ep_of_match[] = {
	{ .compatible = "renesas,rzv2m-pcie", },
	{ .compatible = "renesas,rzv2ma-pcie", },
	{ },
};

static int rzv2m_pcie_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzv2m_pcie_endpoint *ep;
	struct rzv2m_pcie *pcie;
	struct pci_epc *epc;
	int err;

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

	err = rzv2m_pcie_ep_get_pdata(ep, pdev);
	if (err < 0) {
		dev_err(dev, "failed to request resources: %d\n", err);
		goto err_pm_put;
	}

	ep->num_ib_windows = MAX_NR_INBOUND_MAPS;
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

	epc = devm_pci_epc_create(dev, &rzv2m_pcie_epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		err = PTR_ERR(epc);
		goto err_pm_put;
	}

	epc->max_functions = ep->max_functions;
	epc_set_drvdata(epc, ep);

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

	rzv2m_pcie_setting_config_ep(pcie, 0);

	rzv2m_pcie_hw_init_ep(pcie);

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

static struct platform_driver rzv2m_pcie_ep_driver = {
	.driver = {
		.name = "rzv2m-pcie-ep",
		.of_match_table = rzv2m_pcie_ep_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = rzv2m_pcie_ep_probe,
};
builtin_platform_driver(rzv2m_pcie_ep_driver);

