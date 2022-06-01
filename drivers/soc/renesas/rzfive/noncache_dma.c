// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/Five L2 cache controller Driver
 *
 * Copyright (C) 2022 Renesas Electronics Corp.
 *
 */
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direct.h>
#include <linux/scatterlist.h>
#include <linux/highmem.h>

#include "proc.h"

static inline void cache_op(phys_addr_t paddr, size_t size,
		void (*fn)(unsigned long start, unsigned long end))
{
	unsigned long start;

	start = (unsigned long)phys_to_virt(paddr);
	fn(start, start + size);
}

void arch_sync_dma_for_device(phys_addr_t paddr,
		size_t size, enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_FROM_DEVICE:
		cache_op(paddr, size, cpu_dma_inval_range);
		break;
	case DMA_TO_DEVICE:
	case DMA_BIDIRECTIONAL:
		cache_op(paddr, size, cpu_dma_wb_range);
		break;
	default:
		BUG();
	}
}

void arch_sync_dma_for_cpu(phys_addr_t paddr,
		size_t size, enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_TO_DEVICE:
		break;
	case DMA_FROM_DEVICE:
	case DMA_BIDIRECTIONAL:
		cache_op(paddr, size, cpu_dma_inval_range);
		break;
	default:
		BUG();
	}
}

void *arch_dma_alloc(struct device *dev, size_t size, dma_addr_t *handle,
               gfp_t gfp, unsigned long attrs)
{
	return dma_alloc_from_global_coherent(dev, size, handle);
}

void arch_dma_free(struct device *dev, size_t size, void *vaddr,
               dma_addr_t handle, unsigned long attrs)
{
	dma_release_from_global_coherent(0, vaddr);
}
