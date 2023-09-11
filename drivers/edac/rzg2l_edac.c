// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G2L-ECC EDAC Driver.
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 *
 * Authors: LI NAQIN@RENESAS
 */

#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/edac.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include "edac_module.h"

#define RZG2L_EDAC_MOD_NAME		"rzg2l-edac"

/* DDR class Register */
#define DDR_CLS_REG			0x0

/* DDR class Macros */
#define DDR_CLS_OFF			8
#define DDR_CLS_MSK			(0xf << DDR_CLS_OFF)
#define DDR_CLS_DDR3			(0x6)
#define DDR_CLS_DDR4			(0xa)

/* ECC Mode Registers */
#define ECC_MODE_REG			0x174

/* ECC Mode Macros */
#define ECC_MODE_OFF			24
#define ECC_MODE_MSK			(0x3 << ECC_MODE_OFF)
#define ECC_MODE_ENABLE			0x1
#define ECC_MODE_ED			0x2
#define ECC_MODE_SECDED			0x3

/* ECC IRQ Registers */
#define ECC_INT_MSK_MASTER_REG		0x22c
#define ECC_INT_MSK_ECC_REG		0x274
#define ECC_INT_STS_ECC_REG		0x234
#define ECC_INT_ACK_ECC_REG		0x254

/* ECC IRQ Macros */
#define ECC_INT_MSK_MASTER_ECC_OFF	1
#define ECC_INT_MSK_MASTER_GLB_OFF	31
#define ECC_INT_MSK_MASTER_ECC		BIT(ECC_INT_MSK_MASTER_ECC_OFF)
#define ECC_INT_MSK_MASTER_GLB		BIT(ECC_INT_MSK_MASTER_GLB_OFF)
#define ECC_INT_MSK_ECC			(0xffff)
#define ECC_INT_MSK_ALL			GENMASK(31, 0)
#define ECC_INT_STS_ECC_MSK		(ECC_INT_MSK_ECC)
#define ECC_INT_STS_CE_MSK		0x103
#define ECC_INT_STS_UE_MSK		0xC
#define ECC_INT_STS_SCRB_MSK		0x80

/* ECC signature  Registers */
#define ECC_SIG_UE_ADDR1_REG		0x184
#define ECC_SIG_UE_ADDR2_REG		0x188
#define ECC_SIG_UE_SYND_REG		0x188
#define ECC_SIG_UE_DATA1_REG		0x18c
#define ECC_SIG_UE_DATA2_REG		0x190
#define ECC_SIG_UE_ID_REG		0x1a4
#define ECC_SIG_CE_ADDR1_REG		0x194
#define ECC_SIG_CE_ADDR2_REG		0x198
#define ECC_SIG_CE_SYND_REG		0x198
#define ECC_SIG_CE_DATA1_REG		0x19c
#define ECC_SIG_CE_DATA2_REG		0x1a0
#define ECC_SIG_CE_ID_REG		0x1a8

/* ECC signature  Macros */
#define ECC_SIG_SYND_OFF		8
#define ECC_SIG_SYND_MSK		(0xff << ECC_SIG_SYND_OFF)
#define ECC_SIG_ID_MSK			(0x3ffff)

struct rzg2l_edac_priv_data {
	void __iomem *reg;
};

static void init_mem_layout(struct mem_ctl_info *mci)
{
	struct rzg2l_edac_priv_data *priv = mci->pvt_info;
	struct dimm_info *dimm;
	struct sysinfo inf;
	u32 val;

	si_meminfo(&inf);
	dimm = edac_get_dimm(mci, 0, 0, 0);

	/* 1. set the dimm edac_mode */
	val = readl(priv->reg + ECC_MODE_REG) & ECC_MODE_MSK;
	val >>= ECC_MODE_OFF;
	switch(val) {
	case ECC_MODE_ENABLE:
		dimm->edac_mode = EDAC_RESERVED;
		break;
	case ECC_MODE_ED:
		dimm->edac_mode = EDAC_EC;
		break;
	case ECC_MODE_SECDED:
		dimm->edac_mode = EDAC_SECDED;
		break;
	default:
		dimm->edac_mode = EDAC_NONE;
	}

	/* 2. set mtype */
	val = readl(priv->reg + DDR_CLS_REG) & DDR_CLS_MSK;
	val >>= DDR_CLS_OFF;
	if (val == DDR_CLS_DDR3)
		dimm->mtype = MEM_DDR3;
	else if (val == DDR_CLS_DDR4)
		dimm->mtype = MEM_DDR4;
	else
		dimm->mtype = MEM_UNKNOWN;

	/* 3. set dtype */
	dimm->dtype = DEV_UNKNOWN;

	/* 4. set nr_pages */
	dimm->nr_pages =  (inf.totalram * inf.mem_unit) >> PAGE_SHIFT;

	/* 5. set minimum granularity for an error report, in bytes */
	dimm->grain = 8;
}

static irqreturn_t edac_ecc_isr(int irq, void *dev_id)
{
	struct mem_ctl_info *mci = dev_id;
	struct rzg2l_edac_priv_data *priv;
	u32 int_status;
	u32 err_id;
	u32 err_synd;
	u64 err_addr = 0x0;
	u64 err_data = 0x0;

	priv = mci->pvt_info;

	/* Check the intr status and confirm ECC error intr */
	int_status = readl(priv->reg + ECC_INT_STS_ECC_REG) & \
		     ECC_INT_STS_ECC_MSK;

	if (!(int_status & ECC_INT_MSK_ECC))
		return IRQ_NONE;

	if (int_status & ECC_INT_STS_CE_MSK) {
		err_addr = (u64)readl(priv->reg + ECC_SIG_CE_ADDR2_REG);
		err_addr = ((err_addr & 0x3) << 32) | \
				readl(priv->reg + ECC_SIG_CE_ADDR1_REG);

		err_data = (u64)readl(priv->reg + ECC_SIG_CE_DATA2_REG);
		err_data = (err_data << 32) | \
			   readl(priv->reg + ECC_SIG_CE_DATA1_REG);

		err_id = readl(priv->reg + ECC_SIG_CE_ID_REG) & ECC_SIG_ID_MSK;

		err_synd = readl(priv->reg + ECC_SIG_CE_SYND_REG) & \
			   ECC_SIG_SYND_MSK;
		err_synd >>= ECC_SIG_SYND_OFF;

		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci,
			     1,
			     err_addr >> PAGE_SHIFT,
			     err_addr & ~PAGE_MASK,
			     err_synd, 0, 0, -1,
			     mci->ctl_name, "");
	}

	if (int_status & ECC_INT_STS_UE_MSK) {
		err_addr = (u64)readl(priv->reg + ECC_SIG_UE_ADDR2_REG);
		err_addr = ((err_addr & 0x3) << 32) | \
				readl(priv->reg + ECC_SIG_UE_ADDR1_REG);

		err_data = (u64)readl(priv->reg + ECC_SIG_UE_DATA2_REG);
		err_data = (err_data << 32) | \
			   readl(priv->reg + ECC_SIG_UE_DATA1_REG);

		err_id = readl(priv->reg + ECC_SIG_UE_ID_REG) & ECC_SIG_ID_MSK;

		err_synd = readl(priv->reg + ECC_SIG_UE_SYND_REG) & \
			   ECC_SIG_SYND_MSK;
		err_synd >>= ECC_SIG_SYND_OFF;

		edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, mci,
			     1,
			     err_addr >> PAGE_SHIFT,
			     err_addr & ~PAGE_MASK,
			     err_synd, 0, 0, -1,
			     mci->ctl_name, "");
	}

	if (int_status & ECC_INT_STS_SCRB_MSK)
		edac_printk(KERN_ERR, EDAC_MC,
			"The scrub operation has completed.\n");

	/* For ecc scrub start complete and ECC writeback command not be
	 * executed.
	 * */
	if (int_status & ~(ECC_INT_STS_CE_MSK | ECC_INT_STS_UE_MSK))
		edac_printk(KERN_ERR, EDAC_MC,
			"Scrubbing and writeback failed IRQ cleared\n");

	writel(int_status, priv->reg + ECC_INT_ACK_ECC_REG);

	return IRQ_HANDLED;
}

static const struct of_device_id rzg2l_edac_of_match[] = {
	{ .compatible = "renesas,r9a07g044-edac"},
	{ .compatible = "renesas,r9a07g043-edac"},
	{ .compatible = "renesas,r9a07g043f-edac"},
	{},
};
MODULE_DEVICE_TABLE(of, rzg2l_edac_of_match);

static int rzg2l_edac_mc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *reg;
	struct mem_ctl_info *mci;
	struct edac_mc_layer layers[1];
	const struct of_device_id *id;
	struct rzg2l_edac_priv_data *priv_data;
	int irq, ret = -ENODEV;
	u32 val;

	id = of_match_device(rzg2l_edac_of_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(reg)) {
		edac_printk(KERN_ERR, RZG2L_EDAC_MOD_NAME,
			    "rzg2l DDR4 mc regs are not defined\n");
		return PTR_ERR(reg);
	}

	edac_printk(KERN_ERR, RZG2L_EDAC_MOD_NAME,
		    "IO mapped reg addr: 0x%llx\n", res->start);
	layers[0].type = EDAC_MC_LAYER_CHIP_SELECT;
	layers[0].size = 1;
	layers[0].is_virt_csrow = true;

	mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers,
			    sizeof(struct rzg2l_edac_priv_data));
	if (!mci) {
		edac_printk(KERN_ERR, RZG2L_EDAC_MOD_NAME,
			    "Failed memory allocation for mc instance\n");
		ret = -ENOMEM;
		goto edac_mc_alloc_err;
	}
	mci->pdev = &pdev->dev;
	priv_data = mci->pvt_info;
	priv_data->reg = reg;
	platform_set_drvdata(pdev, mci);
	/* Initialize controller capabilities */
	mci->mtype_cap = MEM_FLAG_DDR4;
	mci->edac_ctl_cap = EDAC_FLAG_NONE | EDAC_FLAG_SECDED;
	mci->scrub_cap = SCRUB_UNKNOWN;
	mci->scrub_mode = SCRUB_HW_PROG;
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->ctl_name = id->compatible;
	mci->dev_name = dev_name(&pdev->dev);
	mci->mod_name = RZG2L_EDAC_MOD_NAME;
	mci->ctl_page_to_phys = NULL;

	/* Interrupt feature is supported by cadence mc */
	edac_op_state = EDAC_OPSTATE_INT;
	init_mem_layout(mci);

	/* mask all the interrupts in master */
	val = ECC_INT_MSK_ALL;
	writel(val, priv_data->reg + ECC_INT_MSK_MASTER_REG);

	/* Setup Interrupt handler for ECC */
	irq = platform_get_irq_byname(pdev, "ecc_irq");
	if (!irq) {
		edac_printk(KERN_ERR, RZG2L_EDAC_MOD_NAME,
			    "irq number not defined for ecc.\n");
		goto err;
	}
	ret = devm_request_irq(dev, irq, edac_ecc_isr, 0,
			       "ecc_irq", mci);
	if (ret) {
		edac_printk(KERN_ERR, RZG2L_EDAC_MOD_NAME,
			    "request_irq fail for RZG2L_EDAC irq\n");
		goto err;
	}

	ret = edac_mc_add_mc(mci);
	if (ret) {
		edac_printk(KERN_ERR, RZG2L_EDAC_MOD_NAME,
			    "Failed to register with EDAC core\n");
		goto err;
	}

	/* Unmask all ECC interrupt */
	val = readl(priv_data->reg + ECC_INT_MSK_ECC_REG);
	val &= ~(ECC_INT_MSK_ECC);
	writel(val, priv_data->reg + ECC_INT_MSK_ECC_REG);

	/* Enable the ECC interrupt in master */
	val = readl(priv_data->reg + ECC_INT_MSK_MASTER_REG);
	val &= ~(ECC_INT_MSK_MASTER_ECC | ECC_INT_MSK_MASTER_GLB);
	writel(val, priv_data->reg + ECC_INT_MSK_MASTER_REG);

	/* Ack all ECC interrupt in advance */
	val = readl(priv_data->reg + ECC_INT_ACK_ECC_REG);
	val &= ~ECC_INT_MSK_ECC;
	writel(val, priv_data->reg + ECC_INT_ACK_ECC_REG);

	return 0;

err:
	edac_mc_free(mci);
edac_mc_alloc_err:
	return ret;
}

static int rzg2l_edac_mc_remove(struct platform_device *pdev)
{
	u32 val;
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);
	struct rzg2l_edac_priv_data *priv = mci->pvt_info;

	/* Mask all interrupt from DDR controller in int_mask_master */
	val = readl(priv->reg + ECC_INT_MSK_MASTER_REG);
	val |= ECC_INT_MSK_ALL;
	writel(val, priv->reg + ECC_INT_MSK_MASTER_REG);

	/* Mask all ECC interrupt */
	val = readl(priv->reg + ECC_INT_MSK_ECC_REG);
	val |= ECC_INT_MSK_ECC;
	writel(val, priv->reg + ECC_INT_MSK_ECC_REG);

	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);

	return 0;
}

static struct platform_driver rzg2l_edac_mc_driver = {
	.driver = {
		   .name = "renesas-rzg2l-edac",
		   .of_match_table = rzg2l_edac_of_match,
	},
	.probe = rzg2l_edac_mc_probe,
	.remove = rzg2l_edac_mc_remove,
};

module_platform_driver(rzg2l_edac_mc_driver);

MODULE_AUTHOR("Renesas");
MODULE_DESCRIPTION("Renesas RZ/G2L ECC driver");
MODULE_LICENSE("GPL v2");
