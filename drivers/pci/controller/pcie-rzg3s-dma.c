// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe DMA engine on Renesas RZ/G3S Series SoCs
 *
 * Based on dw-edma-core.c
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 */

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include "../../../drivers/dma/virt-dma.h"
#include "../../../drivers/dma/dmaengine.h"

#include "pcie-rzg3s-regs.h"

struct rzg3s_pcie_dma_chan;

enum rzg3s_pcie_dma_status {
	RZG3S_PCI_DMA_IDLE,
	RZG3S_PCI_DMA_ERR,
	RZG3S_PCI_DMA_BUSY
};

/*
 * According to the RZ/G3S HW manual (Rev.1.10, section 34.4.4.2 Descriptor-Type
 * transfer) The descriptor start address must be 16-byte aligned and
 * allocation of a single descriptor (0x00 to 0x24) to straddle a 4-K boundary is
 * prohibited, so the 6 lower-order bits [5:0] are fixed to 000000b.
 * Declaring __aligned(64) for descriptor struct is to meet above requirement.
 */
struct rzg3s_pcie_dma_hw_node {
	u32 param0;
	u32 param1;
	u32 size;
	u32 padding;
	u32 saddr_L;
	u32 saddr_U;
	u32 daddr_L;
	u32 daddr_U;
	u32 next_L;
	u32 next_U;
} __aligned(64);

struct rzg3s_pcie_dma_sw_node {
	dma_addr_t pdesc;
	struct rzg3s_pcie_dma_hw_node *desc;
};

struct rzg3s_pcie_dma_desc {
	struct virt_dma_desc vd;
	struct rzg3s_pcie_dma_chan *chan;
	enum dma_transfer_direction direction;
	unsigned int n_nodes;
	struct rzg3s_pcie_dma_sw_node node[];
};

struct rzg3s_pcie_dma_chan {
	struct virt_dma_chan vc;
	struct rzg3s_pcie_dmac *dmac;
	struct dma_pool *pool;
	struct dma_slave_config scfg;
	int index;
	struct rzg3s_pcie_dma_desc *desc;
	enum rzg3s_pcie_dma_status status;
};

struct rzg3s_pcie_dmac {
	struct dma_device engine;
	void __iomem *base;
	struct device *dev;
	int dma_irq;
	unsigned int n_channels;
	struct rzg3s_pcie_dma_chan *channels;
};

static inline struct rzg3s_pcie_dma_chan *to_rzg3s_pcie_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct rzg3s_pcie_dma_chan, vc.chan);
};

static inline struct rzg3s_pcie_dma_desc *to_rzg3s_pcie_dma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct rzg3s_pcie_dma_desc, vd);
};

static int rzg3s_pcie_dma_transfer_desc(struct rzg3s_pcie_dma_chan *chan)
{
	struct rzg3s_pcie_dmac *dmac = chan->dmac;
	struct virt_dma_desc *vdesc;
	dma_addr_t dsa;
	u32 mask;

	vdesc = vchan_next_desc(&chan->vc);
	if (!vdesc)
		return 0;

	chan->desc = to_rzg3s_pcie_dma_desc(vdesc);
	/* Set DMAC PCIe Max Read Request Size */
	writel(RZG3S_PCI_DMACTRL_D_PMRS_256, dmac->base + RZG3S_PCI_DMACTRL);

	/* Enable DMA INT */
	mask = (RZG3S_PCI_DMAINTE_CH_END_EN(chan->index) | RZG3S_PCI_DMAINTE_CH_ERR_EN(chan->index));
	rzg3s_pcie_update_bits(dmac->base, RZG3S_PCI_DMAINTE, mask, mask);

	/* Set starting address of Descriptor List */
	dsa = chan->desc->node[0].pdesc;

	writel(lower_32_bits(dsa), dmac->base + RZG3S_PCI_DESSAL(chan->index));
	writel(upper_32_bits(dsa), dmac->base + RZG3S_PCI_DESSAU(chan->index));

	/* Set QUE Entry */
	writel(RZG3S_PCI_QUEE_QUE_INTERRUPT_ON, dmac->base + RZG3S_PCI_QUEE(chan->index));
	/* Start transfer */
	writel(RZG3S_PCI_DMACHCTL_QUE_EN, dmac->base + RZG3S_PCI_DMACHCTL(chan->index));
	return 1;
}

static struct rzg3s_pcie_dma_desc *rzg3s_pcie_dma_alloc_desc(struct rzg3s_pcie_dma_chan *chan,
							     int sg_len)
{
	struct rzg3s_pcie_dma_desc *desc;
	int i;

	desc = kzalloc(struct_size(desc, node, sg_len), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->chan = chan;
	desc->n_nodes = sg_len;
	for (i = 0; i < sg_len; i++) {
		desc->node[i].desc = dma_pool_alloc(chan->pool,
				GFP_NOWAIT, &desc->node[i].pdesc);
		if (!desc->node[i].desc)
			goto err;
	}

	return desc;
err:
	while (--i >= 0)
		dma_pool_free(chan->pool, desc->node[i].desc,
			      desc->node[i].pdesc);
	kfree(desc);

	return NULL;
}

static void rzg3s_pcie_dma_release_desc(struct rzg3s_pcie_dma_desc *desc)
{
	struct rzg3s_pcie_dma_chan *chan = desc->chan;
	int i = 0;

	if (!desc)
		return;

	while (i < desc->n_nodes) {
		dma_pool_free(chan->pool, desc->node[i].desc,
			      desc->node[i].pdesc);
		i++;
	}

	kfree(desc);
}

static int rzg3s_pcie_dma_alloc_chan_resources(struct dma_chan *ch)
{
	struct rzg3s_pcie_dma_chan *chan = to_rzg3s_pcie_dma_chan(ch);

	/*
	 * Create the dma pool for descriptor allocation
	 * According to the RZ/G3S HW manual (Rev.1.10, section 34.4.4.2 Descriptor-Type
	 * transfer) The descriptor start address must be 16-byte aligned and
	 * allocation of a single descriptor (0x00 to 0x24) to straddle a 4-K boundary is
	 * prohibited, the 6 lower-order bits [5:0] are fixed to 000000b.
         */

	chan->pool = dma_pool_create(dev_name(&ch->dev->device),
					    chan->dmac->dev,
					    sizeof(struct rzg3s_pcie_dma_hw_node),
					    __alignof__(struct rzg3s_pcie_dma_hw_node),
					    0);

	if (!chan->pool) {
		dev_err(chan->dmac->dev, "unable to allocate desc pool\n");
		return -ENOMEM;
	}

	dev_dbg(chan->dmac->dev, "%s: alloc chan:%d",
		__func__, chan->vc.chan.chan_id);

	return 0;
}

static void rzg3s_pcie_dma_free_chan_resources(struct dma_chan *ch)
{
	struct rzg3s_pcie_dma_chan *chan = to_rzg3s_pcie_dma_chan(ch);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	chan->desc = NULL;
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	dma_pool_destroy(chan->pool);
	chan->pool = NULL;

	dev_dbg(chan->dmac->dev, "%s: freeing chan:%d\n",
		__func__, chan->vc.chan.chan_id);
}

static size_t rzg3s_pcie_dma_get_residue(struct rzg3s_pcie_dma_chan *chan,
					 struct virt_dma_desc *vdesc,
					 bool in_progress)
{
	struct rzg3s_pcie_dma_desc *desc = chan->desc;
	struct rzg3s_pcie_dmac *dmac = chan->dmac;
	size_t residue = 0;
	u32 reg;
	int i;

	if (in_progress) {
		reg = readl(dmac->base + RZG3S_PCI_DMARESTSIZ(chan->index));
		residue += reg;
	} else {
		for (i = 0; i < desc->n_nodes; i++)
			residue += desc->node[i].desc->size;
	}

	return residue;
}

static enum dma_status rzg3s_pcie_dma_tx_status(struct dma_chan *ch,
						dma_cookie_t cookie,
						struct dma_tx_state *txstate)
{
	struct rzg3s_pcie_dma_chan *chan = to_rzg3s_pcie_dma_chan(ch);
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;

	ret = dma_cookie_status(ch, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	spin_lock_irqsave(&chan->vc.lock, flags);
	vd = vchan_find_desc(&chan->vc, cookie);
	if (chan->desc && cookie == chan->desc->vd.tx.cookie)
		txstate->residue = rzg3s_pcie_dma_get_residue(chan, vd, true);
	else if (vd)
		txstate->residue = rzg3s_pcie_dma_get_residue(chan, vd, false);
	else
		txstate->residue = 0;

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return ret;
}

static struct dma_async_tx_descriptor *
rzg3s_pcie_dma_prep_slave_sg(struct dma_chan *ch, struct scatterlist *sgl,
			     unsigned int sg_len,
			     enum dma_transfer_direction direction,
			     unsigned long flags, void *context)
{
	struct rzg3s_pcie_dma_chan *chan = to_rzg3s_pcie_dma_chan(ch);
	struct rzg3s_pcie_dma_hw_node *hw_node;
	struct rzg3s_pcie_dma_desc *desc;
	struct scatterlist *sg;
	dma_addr_t remote_addr;
	int i;

	if (!is_slave_direction(direction)) {
		dev_err(chan->dmac->dev, "bad direction?\n");
		return NULL;
	}

	desc = rzg3s_pcie_dma_alloc_desc(chan, sg_len);
	if (!desc) {
		dev_err(chan->dmac->dev, "no memory for desc\n");
		return NULL;
	}

	/*
	 * Remote address is RC's memory address if device using this DMA is EP
	 * or, is address in EP BAR if device using this DMA is RC
	 */
	remote_addr = (direction == DMA_MEM_TO_DEV) ? chan->scfg.dst_addr : chan->scfg.src_addr;

	for_each_sg(sgl, sg, sg_len, i) {
		/*
		 * According to the RZ/G3S HW manual (Rev.1.10, section 34.3.1.56
		 * DMA Source Lower Address and 34.3.1.58 DMA Destination Lower Address)
		 * DMA Source and Destination address are 8-byte aligned
		 * (lower 3 bits are fixed to 000b) but attempt to write to these registers
		 * has shown that these addresses are 16-byte aligned (lower 4 bits are fixed to 0000b)
		 */
		if (!IS_ALIGNED(sg_dma_address(sg), SZ_16) || !IS_ALIGNED(sg_dma_len(sg), SZ_16))
			goto err_align;

		hw_node = desc->node[i].desc;

		hw_node->param0 = RZG3S_PCI_DMA_DSCFM | RZG3S_PCI_DMA_WBD | RZG3S_PCI_DMA_LV;
		if (i == (sg_len - 1))
			hw_node->param0 |= RZG3S_PCI_DMA_LE;

		if (direction == DMA_MEM_TO_DEV) {
			hw_node->param1 = RZG3S_PCI_DMA_CCH_L(RZG3S_PCI_DMA_CCH_L_DEFAULT) |
					  RZG3S_PCI_DMA_CCH_D(RZG3S_PCI_DMA_CCH_D_AXI_TO_PCIE) |
					  RZG3S_PCI_DMA_TC(RZG3S_PCI_DMA_TC_DEFAULT) |
					  RZG3S_PCI_DMA_ATB(RZG3S_PCI_DMA_ATB_DEFAULT) |
					  RZG3S_PCI_DMA_FUNC(RZG3S_PCI_DMA_FUNC_0) |
					  RZG3S_PCI_DMA_DIR(RZG3S_PCI_DMA_DIR_AXI_TO_PCIE);
			hw_node->daddr_L = lower_32_bits(remote_addr);
			hw_node->daddr_U = upper_32_bits(remote_addr);

			hw_node->saddr_L = lower_32_bits(sg_dma_address(sg));
			hw_node->saddr_U = upper_32_bits(sg_dma_address(sg));
		} else {
			hw_node->param1 = RZG3S_PCI_DMA_CCH_L(RZG3S_PCI_DMA_CCH_L_DEFAULT) |
					  RZG3S_PCI_DMA_CCH_D(RZG3S_PCI_DMA_CCH_D_PCIE_TO_AXI) |
					  RZG3S_PCI_DMA_TC(RZG3S_PCI_DMA_TC_DEFAULT) |
					  RZG3S_PCI_DMA_ATB(RZG3S_PCI_DMA_ATB_DEFAULT) |
					  RZG3S_PCI_DMA_FUNC(RZG3S_PCI_DMA_FUNC_0) |
					  RZG3S_PCI_DMA_DIR(RZG3S_PCI_DMA_DIR_PCIE_TO_AXI);
			hw_node->saddr_L = lower_32_bits(remote_addr);
			hw_node->saddr_U = upper_32_bits(remote_addr);

			hw_node->daddr_L = lower_32_bits(sg_dma_address(sg));
			hw_node->daddr_U = upper_32_bits(sg_dma_address(sg));
		}

		hw_node->size = sg_dma_len(sg);
		hw_node->next_L = lower_32_bits(desc->node[(i + 1) % sg_len].pdesc);
		hw_node->next_U = upper_32_bits(desc->node[(i + 1) % sg_len].pdesc);

		/*
		 * Unlike the typical assumption by other IPs,
		 * the peripheral memory isn't a FIFO memory. In this case, it's a
		 * linear memory and that why the source/destination addresses are increased
		 * by the same portion (data length)
		 */
		remote_addr += sg_dma_len(sg);
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
err_align:
	dev_err(chan->dmac->dev, "scatter data must be 16-byte aligned\n");
	rzg3s_pcie_dma_release_desc(desc);

	return NULL;
}

static int rzg3s_pcie_dma_terminate_all(struct dma_chan *ch)
{
	struct rzg3s_pcie_dma_chan *chan = to_rzg3s_pcie_dma_chan(ch);
	struct rzg3s_pcie_dmac *dmac = chan->dmac;
	unsigned long flags;
	LIST_HEAD(head);
	int ch_id;

	ch_id = chan->vc.chan.chan_id;
	dev_dbg(dmac->dev, "%s: terminate chan:%d\n", __func__, ch_id);

	spin_lock_irqsave(&chan->vc.lock, flags);

	writel(RZG3S_PCI_DMACHCTL_QUE_CLR, dmac->base + RZG3S_PCI_DMACHCTL(chan->index));

	chan->desc = NULL;
	vchan_get_all_descriptors(&chan->vc, &head);

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	vchan_dma_desc_free_list(&chan->vc, &head);

	return 0;
}

static void rzg3s_pcie_dma_issue_pending(struct dma_chan *ch)
{
	struct rzg3s_pcie_dma_chan *chan = to_rzg3s_pcie_dma_chan(ch);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	if (vchan_issue_pending(&chan->vc) && !chan->desc &&
	    chan->status == RZG3S_PCI_DMA_IDLE) {
		chan->status = RZG3S_PCI_DMA_BUSY;
		rzg3s_pcie_dma_transfer_desc(chan);
	}

	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static int rzg3s_pcie_dma_config(struct dma_chan *ch,
				 struct dma_slave_config *config)
{
	struct rzg3s_pcie_dma_chan *chan = to_rzg3s_pcie_dma_chan(ch);

	if (config->direction == DMA_MEM_TO_DEV) {
		if (!config->dst_addr)
			return -EINVAL;

		if (!IS_ALIGNED(config->dst_addr, SZ_16)) {
			dev_err(chan->dmac->dev, "dst_addr must be 16-byte aligned\n");
			return -EINVAL;
		}
	} else {
		if (!config->src_addr)
			return -EINVAL;

		if (!IS_ALIGNED(config->src_addr, SZ_16)) {
			dev_err(chan->dmac->dev, "src_addr must be 16-byte aligned\n");
			return -EINVAL;
		}
	}

	memcpy(&chan->scfg, config, sizeof(chan->scfg));
	return 0;
}

static void rzg3s_pcie_dma_free_desc(struct virt_dma_desc *vdesc)
{
	struct rzg3s_pcie_dma_desc *desc = to_rzg3s_pcie_dma_desc(vdesc);
	int i;

	for (i = 0; i < desc->n_nodes; i++)
		dma_pool_free(desc->chan->pool, desc->node[i].desc,
			      desc->node[i].pdesc);
	kfree(desc);
}

static irqreturn_t rzg3s_pcie_dma_irq_handler(int irq, void *dev_id)
{
	struct rzg3s_pcie_dmac *dmac = dev_id;
	struct rzg3s_pcie_dma_chan *chan;
	u32 reg;
	int i;

	reg = readl(dmac->base + RZG3S_PCI_DMAINTS);
	for (i = 0; i < dmac->n_channels; i++) {
		chan = &dmac->channels[i];

		if (!(reg & RZG3S_PCI_DMAINTS_CH_ALL(chan->index))) {
			continue;
		} else if (reg & RZG3S_PCI_DMAINTS_CH_END(chan->index))
			chan->status = RZG3S_PCI_DMA_IDLE;
		else if (reg & RZG3S_PCI_DMAINTS_CH_ERR(chan->index))
			chan->status = RZG3S_PCI_DMA_ERR;

		rzg3s_pcie_update_bits(dmac->base, RZG3S_PCI_DMAINTS,
				       RZG3S_PCI_DMAINTS_CH_ALL(chan->index),
				       RZG3S_PCI_DMAINTS_CH_ALL(chan->index));

		rzg3s_pcie_update_bits(dmac->base, RZG3S_PCI_DMAINTE,
				       RZG3S_PCI_DMAINTE_CH_ALL(chan->index),
				       ~RZG3S_PCI_DMAINTE_CH_ALL(chan->index));
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t rzg3s_pcie_dma_irq_handler_thread(int irq, void *dev_id)
{
	struct rzg3s_pcie_dmac *dmac = dev_id;
	struct rzg3s_pcie_dma_chan *chan;
	unsigned long flags;
	int i;

	for (i = 0; i < dmac->n_channels; i++) {
		chan = &dmac->channels[i];
		if (chan->status == RZG3S_PCI_DMA_BUSY)
			continue;
		else if (chan->status == RZG3S_PCI_DMA_IDLE) {
			spin_lock_irqsave(&chan->vc.lock, flags);
			if (chan->desc) {
				list_del(&chan->desc->vd.node);
				vchan_cookie_complete(&chan->desc->vd);

				chan->desc = NULL;
				/* Continue transferring if there are remaining descriptor lists */
				chan->status = rzg3s_pcie_dma_transfer_desc(chan) ? RZG3S_PCI_DMA_BUSY : RZG3S_PCI_DMA_IDLE;
			}
			spin_unlock_irqrestore(&chan->vc.lock, flags);
		} else if (chan->status == RZG3S_PCI_DMA_ERR) {
			spin_lock_irqsave(&chan->vc.lock, flags);
			if (chan->desc) {
				list_del(&chan->desc->vd.node);
				vchan_cookie_complete(&chan->desc->vd);
				chan->status = RZG3S_PCI_DMA_IDLE;
				chan->desc = NULL;
			}
			dev_err(dmac->dev, "ch:%d error DMA tranfer\n", chan->index);
			spin_unlock_irqrestore(&chan->vc.lock, flags);
		}
	}

	return IRQ_HANDLED;
}

static int rzg3s_pcie_dma_channel_setup(struct rzg3s_pcie_dmac *dmac)
{
	struct device *dev = dmac->dev;
	struct dma_device *engine = &dmac->engine;
	int i, ret;

	dmac->n_channels = RZG3S_PCI_DMA_MAX_CHANNEL;
	dmac->channels = devm_kcalloc(dev, dmac->n_channels,
				   sizeof(struct rzg3s_pcie_dma_chan), GFP_KERNEL);
	if (!dmac->channels) {
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&dmac->engine.channels);

	for (i = 0; i < dmac->n_channels; i++) {
		struct rzg3s_pcie_dma_chan *chan = &dmac->channels[i];

		chan->dmac = dmac;
		chan->index = i;
		chan->desc = NULL;
		chan->vc.desc_free = rzg3s_pcie_dma_free_desc;
		vchan_init(&chan->vc, engine);
	}

	/* Set DMA channel capabilities */
	dma_cap_zero(engine->cap_mask);
	dma_cap_set(DMA_SLAVE, engine->cap_mask);
	engine->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	engine->dev = dev;

	engine->device_alloc_chan_resources = rzg3s_pcie_dma_alloc_chan_resources;
	engine->device_free_chan_resources = rzg3s_pcie_dma_free_chan_resources;
	engine->device_tx_status = rzg3s_pcie_dma_tx_status;
	engine->device_prep_slave_sg = rzg3s_pcie_dma_prep_slave_sg;
	engine->device_config = rzg3s_pcie_dma_config;
	engine->device_terminate_all = rzg3s_pcie_dma_terminate_all;
	engine->device_issue_pending = rzg3s_pcie_dma_issue_pending;
	dma_set_max_seg_size(engine->dev, U32_MAX);

	ret = dma_async_device_register(engine);
	if (ret < 0) {
		dev_dbg(dev, "%s unable to register DMA engine\n", __func__);
		goto err;
	}
	return 0;
err:
	kfree(dmac->channels);
	return ret;
}

int rzg3s_pcie_dma_probe(struct rz_pcie *pci)
{
	struct platform_device *pdev = to_platform_device(pci->dev);
	struct device *dev = pci->dev;
	struct rzg3s_pcie_dmac *dmac;
	int ret = 0;

	dmac = devm_kzalloc(pci->dev, sizeof(struct rzg3s_pcie_dmac), GFP_KERNEL);
	if (!dmac)
		return -ENOMEM;

	dmac->dev = pci->dev;
	dmac->base = pci->base;
	pci->dmac = dmac;
	dmac->dma_irq = platform_get_irq_byname(pdev, "dma");

	if (dmac->dma_irq < 0) {
		dev_err(dev, "cannot get DMA_INT irq for PCIe\n");
		ret = dmac->dma_irq;
		goto err;
	}

	ret = devm_request_threaded_irq(dev, dmac->dma_irq, rzg3s_pcie_dma_irq_handler,
					rzg3s_pcie_dma_irq_handler_thread,
					IRQF_SHARED, "PCIE DMA INT", dmac);
	if (ret) {
		dev_err(dev, "failed to request IRQ %u (%d)\n", dmac->dma_irq, ret);
		goto err;
	}

	ret = rzg3s_pcie_dma_channel_setup(dmac);
	if (ret < 0)
		goto err;

	dev_info(dev, "DMA available\n");
	return 0;
err:
	kfree(dmac);
	return ret;
}

void rzg3s_pcie_dma_remove(struct rz_pcie *pci)
{
	struct rzg3s_pcie_dmac *dmac = pci->dmac;
	int i;

	dma_async_device_unregister(&dmac->engine);

	for (i = 0; i < dmac->n_channels; i++) {
		struct rzg3s_pcie_dma_chan *chan = &dmac->channels[i];

		if (!chan->desc)
			rzg3s_pcie_dma_release_desc(chan->desc);
		dma_pool_destroy(chan->pool);
	}

	kfree(dmac->channels);
	kfree(dmac);
}
