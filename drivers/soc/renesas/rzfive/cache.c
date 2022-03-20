// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/Five L2 cache controller Driver
 *
 * Copyright (C) 2022 Renesas Electronics Corp.
 *
 */
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/device.h>
#include <asm/cacheinfo.h>

#include <asm/sbi.h>
#include <asm/csr.h>

#include "proc.h"
#include "csr.h"

/* Register */
#define CONFIGURATION_REG	0x0
#define CONTROL_REG		0x8
#define STATUS_REG		0x80
#define CCTL_CMD_REG	0x40
#define L2_WBINVAL_ALL	0x12
#define L2_ENABLE		BIT(0)
#define CCTL_STS_CN_MASK(mhartid) (0xF << ((mhartid << 2)))

 /* prefetch */
 #define IPREPETCH_OFF	3
 #define DPREPETCH_OFF	5
 #define IPREPETCH_MSK	(3 << IPREPETCH_OFF)
 #define DPREPETCH_MSK	(3 << DPREPETCH_OFF)
 /* tag ram */
 #define TRAMOCTL_OFF	8
 #define TRAMICTL_OFF	10
 #define TRAMOCTL_MSK	(3 << TRAMOCTL_OFF)
 #define TRAMICTL_MSK	BIT(TRAMICTL_OFF)
 /* data ram */
 #define DRAMOCTL_OFF	11
 #define DRAMICTL_OFF	13
 #define DRAMOCTL_MSK	(3 << DRAMOCTL_OFF)
 #define DRAMICTL_MSK	BIT(DRAMICTL_OFF)

struct v5l2_plat {
	void __iomem	*base;
	u32		iprefetch;
	u32		dprefetch;
	u32		tram_ctl[2];
	u32		dram_ctl[2];
};

#define MAX_CACHE_LINE_SIZE 256
#define EVSEL_MASK	0xff
#define SEL_PER_CTL	8
#define SEL_OFF(id)	(8 * (id % 8))

static void __iomem *l2c_base;

DEFINE_PER_CPU(struct andesv5_cache_info, cpu_cache_info) = {
	.init_done = 0,
	.dcache_line_size = SZ_32
};

static uint32_t cpu_get_mcache_ctl_status(void)
{
	struct sbiret ret;
	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MCACHE_CTL_STATUS, 0, 0, 0, 0, 0, 0);
	return ret.value;
}

static uint32_t cpu_get_micm_cfg_status(void)
{
	struct sbiret ret;
	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MICM_CTL_STATUS, 0, 0, 0, 0, 0, 0);
	return ret.value;
}

static uint32_t cpu_get_mdcm_cfg_status(void)
{
	struct sbiret ret;
	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MDCM_CTL_STATUS, 0, 0, 0, 0, 0, 0);
	return ret.value;
}

static uint32_t cpu_get_mmsc_cfg_status(void)
{
	struct sbiret ret;
	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MMSC_CTL_STATUS, 0, 0, 0, 0, 0, 0);
	return ret.value;
}

static uint32_t cpu_get_misa_cfg_status(void)
{
	struct sbiret ret;
	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MISA_CTL_STATUS, 0, 0, 0, 0, 0, 0);
	return ret.value;
}

static void fill_cpu_cache_info(struct andesv5_cache_info *cpu_ci)
{
	struct cpu_cacheinfo *this_cpu_ci =
		get_cpu_cacheinfo(smp_processor_id());
	struct cacheinfo *this_leaf = this_cpu_ci->info_list;
	unsigned int i = 0;

	for(; i< this_cpu_ci->num_leaves ; i++, this_leaf++)
		if(this_leaf->type == CACHE_TYPE_DATA) {
			cpu_ci->dcache_line_size = this_leaf->coherency_line_size;
		}
	cpu_ci->init_done = true;
}

inline int get_cache_line_size(void)
{
	struct andesv5_cache_info *cpu_ci =
		&per_cpu(cpu_cache_info, smp_processor_id());

	if(unlikely(cpu_ci->init_done == false))
		fill_cpu_cache_info(cpu_ci);
	return cpu_ci->dcache_line_size;
}

static uint32_t cpu_l2c_get_cctl_status(void)
{
	return readl((void*)(l2c_base + L2C_REG_STATUS_OFFSET));
}

uint32_t cpu_l2c_ctl_status(void)
{
	return readl((void*)(l2c_base + L2C_REG_CTL_OFFSET));
}

static bool cpu_cache_controlable(void)
{
#if 0
	bool ucctl_flag = false;

	ucctl_flag = (cpu_get_micm_cfg_status() & MICM_CFG_ISZ_MASK) || \
				(cpu_get_mdcm_cfg_status() & MDCM_CFG_DSZ_MASK);
	pr_err("[%s] %d micm:%08x\n", __func__, __LINE__, cpu_get_micm_cfg_status() & MICM_CFG_ISZ_MASK);
	pr_err("[%s] %d mdcm:%08x\n", __func__, __LINE__, cpu_get_mdcm_cfg_status() & MDCM_CFG_DSZ_MASK);
	ucctl_flag &= (cpu_get_misa_cfg_status() & MISA_20_MASK);
	pr_err("[%s] %d misa:%08x\n", __func__, __LINE__, cpu_get_misa_cfg_status() & MISA_20_MASK);
	ucctl_flag &= (cpu_get_mmsc_cfg_status() & MMSC_CFG_CCTLCSR_MASK);
	pr_err("[%s] %d mmsc:%08x\n", __func__, __LINE__, cpu_get_mmsc_cfg_status() & MMSC_CFG_CCTLCSR_MASK);
	ucctl_flag &= (cpu_get_mcache_ctl_status() & MCACHE_CTL_CCTL_SUEN_MASK);
	pr_err("[%s] %d mcache:%08x\n", __func__, __LINE__, cpu_get_mcache_ctl_status() & MCACHE_CTL_CCTL_SUEN_MASK);
#endif

	return (((cpu_get_micm_cfg_status() & MICM_CFG_ISZ_MASK) || \
				(cpu_get_mdcm_cfg_status() & MDCM_CFG_DSZ_MASK)) && \
				(cpu_get_misa_cfg_status() & MISA_20_MASK) && \
				(cpu_get_mmsc_cfg_status() & MMSC_CFG_CCTLCSR_MASK) && \
				(cpu_get_mcache_ctl_status() & MCACHE_CTL_CCTL_SUEN_MASK));
}

void cpu_dcache_wb_range(unsigned long start, unsigned long end, int line_size)
{
	int mhartid = 0;
	unsigned long pa;
	bool ucctl_ok = false;
#ifdef CONFIG_SMP
	mhartid = smp_processor_id();
#endif

	ucctl_ok = cpu_cache_controlable();

	while (end > start) {

		if(ucctl_ok){
			custom_csr_write(CCTL_REG_UCCTLBEGINADDR_NUM, start);
			custom_csr_write(CCTL_REG_UCCTLCOMMAND_NUM, CCTL_L1D_VA_WB);
		}

		// when the l2 cahce is probed and enabled.
		if (l2c_base && (cpu_l2c_ctl_status() & L2_CACHE_CTL_mskCEN)) {
			pa = virt_to_phys((void*)start);
			writel(pa, (void*)(l2c_base+ L2C_REG_CN_ACC_OFFSET(mhartid)));
			writel(CCTL_L2_PA_WB, (void*)(l2c_base + L2C_REG_CN_CMD_OFFSET(mhartid)));
			while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_CN_MASK(mhartid))
					!= CCTL_L2_STATUS_IDLE);
		}

		start += line_size;
	}
}

void cpu_dcache_inval_range(unsigned long start, unsigned long end, int line_size)
{
	int mhartid = 0;
	unsigned long pa;
	bool ucctl_ok = false;
#ifdef CONFIG_SMP
	mhartid = smp_processor_id();
#endif

	ucctl_ok = cpu_cache_controlable();

	while (end > start) {

		if(ucctl_ok){
			custom_csr_write(CCTL_REG_UCCTLBEGINADDR_NUM, start);
			custom_csr_write(CCTL_REG_UCCTLCOMMAND_NUM, CCTL_L1D_VA_INVAL);
		}

		// when the l2 cahce is probed and enabled.
		if (l2c_base && (cpu_l2c_ctl_status() & L2_CACHE_CTL_mskCEN)) {
			pa = virt_to_phys((void*)start);
			writel(pa, (void*)(l2c_base + L2C_REG_CN_ACC_OFFSET(mhartid)));
			writel(CCTL_L2_PA_INVAL, (void*)(l2c_base + L2C_REG_CN_CMD_OFFSET(mhartid)));
			while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_CN_MASK(mhartid))
					!= CCTL_L2_STATUS_IDLE);
		}

		start += line_size;
	}
}

void cpu_dma_inval_range(unsigned long start, unsigned long end)
{
	unsigned long flags;
	unsigned long line_size = get_cache_line_size();
	unsigned long old_start = start;
	unsigned long old_end = end;
	char cache_buf[2][MAX_CACHE_LINE_SIZE]={0};

	if (unlikely(start == end))
		return;

	start = start & (~(line_size - 1));
	end = ((end + line_size - 1) & (~(line_size - 1)));

	local_irq_save(flags);
	if (unlikely(start != old_start)) {
		memcpy(&cache_buf[0][0], (void *)start, line_size);
	}
	if (unlikely(end != old_end)) {
		memcpy(&cache_buf[1][0], (void *)(old_end & (~(line_size - 1))), line_size);
	}
	cpu_dcache_inval_range(start, end, line_size);
	if (unlikely(start != old_start)) {
		memcpy((void *)start, &cache_buf[0][0], (old_start & (line_size - 1)));
	}
	if (unlikely(end != old_end)) {
		memcpy((void *)(old_end + 1), &cache_buf[1][(old_end & (line_size - 1)) + 1], end - old_end - 1);
	}
	local_irq_restore(flags);

}
EXPORT_SYMBOL(cpu_dma_inval_range);

void cpu_dma_wb_range(unsigned long start, unsigned long end)
{
	unsigned long flags;
	unsigned long line_size = get_cache_line_size();

	local_irq_save(flags);
	start = start & (~(line_size - 1));
	cpu_dcache_wb_range(start, end, line_size);
	local_irq_restore(flags);
}
EXPORT_SYMBOL(cpu_dma_wb_range);


int cpu_l1c_status(void)
{
	return 0;
}

void cpu_icache_enable(void *info)
{

}

void cpu_icache_disable(void *info)
{

}

void cpu_dcache_enable(void *info)
{

}

void cpu_dcache_disable(void *info)
{

}

#ifndef CONFIG_SMP
void cpu_l2c_inval_range(unsigned long pa, unsigned long size)
{
	unsigned long line_size = get_cache_line_size();
	unsigned long start = pa, end = pa + size;
	unsigned long align_start, align_end;

	if(!(cpu_l2c_ctl_status() & L2_CACHE_CTL_mskCEN))
		return;

	align_start = start & ~(line_size - 1);
	align_end  = (end + line_size - 1) & ~(line_size - 1);

	while(align_end > align_start){
		writel(align_start, (void*)(l2c_base + L2C_REG_C0_ACC_OFFSET));
		writel(CCTL_L2_PA_INVAL, (void*)(l2c_base + L2C_REG_C0_CMD_OFFSET));
		while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_C0_MASK)
				!= CCTL_L2_STATUS_IDLE);
		align_start += line_size;
	}
}
EXPORT_SYMBOL(cpu_l2c_inval_range);

void cpu_l2c_wb_range(unsigned long pa, unsigned long size)
{
	unsigned long line_size = get_cache_line_size();
	unsigned long start = pa, end = pa + size;
	unsigned long align_start, align_end;

	if(!(cpu_l2c_ctl_status() & L2_CACHE_CTL_mskCEN))
		return;

	align_start = start & ~(line_size - 1);
	align_end  = (end + line_size - 1) & ~(line_size - 1);

	while(align_end > align_start){
		writel(align_start, (void*)(l2c_base + L2C_REG_C0_ACC_OFFSET));
		writel(CCTL_L2_PA_WB, (void*)(l2c_base + L2C_REG_C0_CMD_OFFSET));
		while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_C0_MASK)
				!= CCTL_L2_STATUS_IDLE);
		align_start += line_size;
	}
}
EXPORT_SYMBOL(cpu_l2c_wb_range);
#else
void cpu_l2c_inval_range(unsigned long pa, unsigned long size)
{
	int mhartid = smp_processor_id();
	unsigned long line_size = get_cache_line_size();
	unsigned long start = pa, end = pa + size;
	unsigned long align_start, align_end;

	if(!(cpu_l2c_ctl_status() & L2_CACHE_CTL_mskCEN))
		return;

	align_start = start & ~(line_size - 1);
	align_end  = (end + line_size - 1) & ~(line_size - 1);

	while(align_end > align_start){
		writel(align_start, (void*)(l2c_base + L2C_REG_CN_ACC_OFFSET(mhartid)));
		writel(CCTL_L2_PA_INVAL, (void*)(l2c_base + L2C_REG_CN_CMD_OFFSET(mhartid)));
		while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_CN_MASK(mhartid))
				!= CCTL_L2_STATUS_IDLE);
		align_start += line_size;
	}
}
EXPORT_SYMBOL(cpu_l2c_inval_range);

void cpu_l2c_wb_range(unsigned long pa, unsigned long size)
{
	int mhartid = smp_processor_id();
	unsigned long line_size = get_cache_line_size();
	unsigned long start = pa, end = pa + size;
	unsigned long align_start, align_end;

	if(!(cpu_l2c_ctl_status() & L2_CACHE_CTL_mskCEN))
		return;

	align_start = start & ~(line_size - 1);
	align_end  = (end + line_size - 1) & ~(line_size - 1);

	while(align_end > align_start){
		writel(align_start, (void*)(l2c_base + L2C_REG_CN_ACC_OFFSET(mhartid)));
		writel(CCTL_L2_PA_WB, (void*)(l2c_base + L2C_REG_CN_CMD_OFFSET(mhartid)));
		while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_CN_MASK(mhartid))
				!= CCTL_L2_STATUS_IDLE);
		align_start += line_size;
	}
}
EXPORT_SYMBOL(cpu_l2c_wb_range);
#endif

// L2 cache
void cpu_l2c_disable(void)
{
#ifdef CONFIG_SMP
	int mhartid = get_cpu();
#else
	int mhartid = 0;
#endif
	unsigned int val;
	unsigned long flags;

	/*No l2 cache */
	if(!l2c_base)
		return;

	local_irq_save(flags);

	/*l2 cache has disabled*/
	if(!(cpu_l2c_ctl_status() & L2_CACHE_CTL_mskCEN))
		return;

	/*L2 write-back and invalidate all*/
	writel(CCTL_L2_WBINVAL_ALL, l2c_base + L2C_REG_CN_CMD_OFFSET(mhartid));
	while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_CN_MASK(mhartid))
			!= CCTL_L2_STATUS_IDLE);

	/*Disable L2 cache*/
	val = readl(l2c_base + L2C_REG_CTL_OFFSET);
	val &= (~L2_CACHE_CTL_mskCEN);

	writel(val, l2c_base + L2C_REG_CTL_OFFSET);
	while ((cpu_l2c_get_cctl_status() & CCTL_L2_STATUS_CN_MASK(mhartid))
			!= CCTL_L2_STATUS_IDLE);

	local_irq_restore(flags);
#ifdef CONFIG_SMP
    put_cpu();
#endif
}

void cpu_l2c_enable(struct v5l2_plat *priv)
{
	u32 ctl_val;

	ctl_val =cpu_l2c_ctl_status();
#if 0
	if (!(ctl_val & L2_ENABLE))
		ctl_val |= L2_ENABLE;
#endif

	if (priv->iprefetch != -EINVAL) {
		ctl_val &= ~(IPREPETCH_MSK);
		ctl_val |= (priv->iprefetch << IPREPETCH_OFF);
	}

	if (priv->dprefetch != -EINVAL) {
		ctl_val &= ~(DPREPETCH_MSK);
		ctl_val |= (priv->dprefetch << DPREPETCH_OFF);
	}

	if (priv->tram_ctl[0] != -EINVAL) {
		ctl_val &= ~(TRAMOCTL_MSK | TRAMICTL_MSK);
		ctl_val |= priv->tram_ctl[0] << TRAMOCTL_OFF;
		ctl_val |= priv->tram_ctl[1] << TRAMICTL_OFF;
	}

	if (priv->dram_ctl[0] != -EINVAL) {
		ctl_val &= ~(DRAMOCTL_MSK | DRAMICTL_MSK);
		ctl_val |= priv->dram_ctl[0] << DRAMOCTL_OFF;
		ctl_val |= priv->dram_ctl[1] << DRAMICTL_OFF;
	}

	writel(ctl_val, priv->base + CONTROL_REG);
}

static void cpu_l2c_of_to_plat(struct v5l2_plat *priv,
			    struct device_node *np)
{
	priv->iprefetch = -EINVAL;
	priv->dprefetch = -EINVAL;
	priv->tram_ctl[0] = -EINVAL;
	priv->dram_ctl[0] = -EINVAL;

	/* Instruction and data fetch prefetch depth */
	of_property_read_u32(np, "andes,inst-prefetch", &priv->iprefetch);
	of_property_read_u32(np, "andes,data-prefetch", &priv->dprefetch);

	/* Set tag RAM and data RAM setup and output cycle */
	of_property_read_u32_array(np, "andes,tag-ram-ctl", priv->tram_ctl, 2);
	of_property_read_u32_array(np, "andes,data-ram-ctl", priv->dram_ctl, 2);

	pr_info("L2CACHE: prefetch: %u\n", priv->iprefetch);
	pr_info("L2CACHE: data: %u\n", priv->dprefetch);
	pr_info("L2CACHE: tram: %u %u\n", priv->tram_ctl[0], priv->tram_ctl[1]);
	pr_info("L2CACHE: dram_ctl:%u %u\n", priv->dram_ctl[0], priv->dram_ctl[1]);
}

static const struct of_device_id sifive_l2_ids[] = {
	{ .compatible = "cache" },
	{ }
};

static int __init sifive_l2_init(void)
{
	struct device_node *np;
	struct v5l2_plat *priv;
	struct resource res;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	np = of_find_matching_node(NULL, sifive_l2_ids);
	if (!np)
		return -ENODEV;

	ret = of_address_to_resource(np, 0, &res);
	if (ret){
		return -ENODEV;
	}

	priv->base = ioremap(res.start, resource_size(&res));
	if (!priv->base)
		return -ENOMEM;
	l2c_base = priv->base;

	cpu_l2c_of_to_plat(priv, np);

	pr_info("L2CACHE: INFO\n");
	pr_info("L2CACHE: Configuration:%x\n", readl(priv->base + CONFIGURATION_REG));
	pr_info("L2CACHE: Control:%x\n", readl(priv->base + CONTROL_REG));
	return 0;
}
arch_initcall(sifive_l2_init);
