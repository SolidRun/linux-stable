// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Renesas RZ/G2L CRU
 *
 * Copyright (C) 2022 Renesas Electronics Corp.
 *
 * Based on Renesas R-Car VIN
 * Copyright (C) 2011-2013 Renesas Solutions Corp.
 * Copyright (C) 2013 Cogent Embedded, Inc., <source@cogentembedded.com>
 * Copyright (C) 2008 Magnus Damm
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>

#include "rzg2l-cru.h"

static inline struct rzg2l_cru_dev *notifier_to_cru(struct v4l2_async_notifier *n)
{
	return container_of(n, struct rzg2l_cru_dev, notifier);
}

static const struct media_device_ops rzg2l_cru_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

/* -----------------------------------------------------------------------------
 * Group async notifier
 */

static int rzg2l_cru_group_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct rzg2l_cru_dev *cru = notifier_to_cru(notifier);
	struct media_entity *source, *sink;
	int ret;

	ret = rzg2l_cru_ip_subdev_register(cru);
	if (ret)
		return ret;

	ret = v4l2_device_register_subdev_nodes(&cru->v4l2_dev);
	if (ret) {
		dev_err(cru->dev, "Failed to register subdev nodes\n");
		return ret;
	}

	ret = rzg2l_cru_video_register(cru);
	if (ret)
		return ret;

	/*
	 * CRU can be connected either to CSI2 or PARALLEL device
	 * For now we are only supporting CSI2
	 *
	 * Create media device link between CSI-2 <-> CRU IP
	 */
	source = &cru->csi.subdev->entity;
	sink = &cru->ip.subdev.entity;
	ret = media_create_pad_link(source, 1, sink, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(cru->dev, "Error creating link from %s to %s\n",
			source->name, sink->name);
		return ret;
	}
	cru->csi.channel = 0;
	cru->svc_channel = cru->csi.channel;
	cru->ip.remote = cru->csi.subdev;

	/* Create media device link between CRU IP <-> CRU OUTPUT */
	source = &cru->ip.subdev.entity;
	sink = &cru->vdev.entity;
	ret = media_create_pad_link(source, 1, sink, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(cru->dev, "Error creating link from %s to %s\n",
			source->name, sink->name);
		return ret;
	}

	return 0;
}

static void rzg2l_cru_group_notify_unbind(struct v4l2_async_notifier *notifier,
					  struct v4l2_subdev *subdev,
					  struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = notifier_to_cru(notifier);

	rzg2l_cru_ip_subdev_unregister(cru);

	mutex_lock(&cru->mdev_lock);

	if (cru->csi.asd == asd) {
		cru->csi.subdev = NULL;
		dev_dbg(cru->dev, "Unbind CSI-2 %s\n", subdev->name);
	}

	mutex_unlock(&cru->mdev_lock);
}

static int rzg2l_cru_group_notify_bound(struct v4l2_async_notifier *notifier,
					struct v4l2_subdev *subdev,
					struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = notifier_to_cru(notifier);

	mutex_lock(&cru->mdev_lock);

	if (cru->csi.asd == asd) {
		cru->csi.subdev = subdev;
		dev_dbg(cru->dev, "Bound CSI-2 %s\n", subdev->name);
	}

	mutex_unlock(&cru->mdev_lock);

	return 0;
}

static const struct v4l2_async_notifier_operations rzg2l_cru_async_ops = {
	.bound = rzg2l_cru_group_notify_bound,
	.unbind = rzg2l_cru_group_notify_unbind,
	.complete = rzg2l_cru_group_notify_complete,
};

static int rzg2l_cru_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rzg2l_cru_dev *cru = container_of(ctrl->handler,
						 struct rzg2l_cru_dev,
						 ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		if ((cru->state == RZG2L_CRU_DMA_STOPPED) ||
		    (cru->state == RZG2L_CRU_DMA_STOPPING))
			cru->num_buf = ctrl->val;
		else
			ret = -EBUSY;

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops rzg2l_cru_ctrl_ops = {
	.s_ctrl = rzg2l_cru_s_ctrl,
};

static int rzg2l_cru_mc_parse_of(struct rzg2l_cru_dev *cru)
{
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep, *fwnode;
	struct v4l2_async_subdev *asd;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(cru->dev), 1, 0, 0);
	if (!ep)
		return 0;

	fwnode = fwnode_graph_get_remote_endpoint(ep);
	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	fwnode_handle_put(ep);
	if (ret) {
		dev_err(cru->dev, "Failed to parse %pOF\n", to_of_node(fwnode));
		ret = -EINVAL;
		goto out;
	}

	if (!of_device_is_available(to_of_node(fwnode))) {
		dev_dbg(cru->dev, "OF device %pOF disabled, ignoring\n",
			to_of_node(fwnode));
		ret = -ENOTCONN;
		goto out;
	}

	asd = v4l2_async_nf_add_fwnode(&cru->notifier, fwnode,
				       struct v4l2_async_subdev);
	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto out;
	}

	cru->csi.asd = asd;

	dev_dbg(cru->dev, "Added OF device %pOF to slot %u\n",
		to_of_node(fwnode), vep.base.id);
out:
	fwnode_handle_put(fwnode);

	return ret;
}

static int rzg2l_cru_mc_parse_of_graph(struct rzg2l_cru_dev *cru)
{
	int ret;

	v4l2_async_nf_init(&cru->notifier);

	ret = rzg2l_cru_mc_parse_of(cru);
	if (ret)
		return ret;

	cru->notifier.ops = &rzg2l_cru_async_ops;

	if (list_empty(&cru->notifier.asd_list))
		return 0;

	ret = v4l2_async_nf_register(&cru->v4l2_dev, &cru->notifier);
	if (ret < 0) {
		dev_err(cru->dev, "Notifier registration failed\n");
		v4l2_async_nf_cleanup(&cru->notifier);
		return ret;
	}

	return 0;
}

static int rzg2l_cru_media_init(struct rzg2l_cru_dev *cru)
{
	struct media_device *mdev = NULL;
	const struct of_device_id *match;
	int ret;

	cru->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&cru->vdev.entity, 1, &cru->pad);
	if (ret)
		return ret;

	if (cru->info->max_cru_channels > 1) {
		ret = of_property_read_u32(cru->dev->of_node, "channel,id", &cru->id);
		if (ret) {
			if (cru->info->cru_type == RZV2H_CRU_TYPE) {
				dev_err(cru->dev, "%pOF: No channel,id property found\n",
						cru->dev->of_node);
				return -EINVAL;
			}
			cru->id = 0;
		}
	}

	if (cru->id >= cru->info->max_cru_channels) {
		dev_err(cru->dev, "%pOF: Invalid channel,id '%u'\n",
			cru->dev->of_node, cru->id);
		return -EINVAL;
	}

	mutex_init(&cru->mdev_lock);
	mdev = &cru->mdev;
	mdev->dev = cru->dev;
	mdev->ops = &rzg2l_cru_media_ops;

	match = of_match_node(cru->dev->driver->of_match_table,
			      cru->dev->of_node);

	strscpy(mdev->driver_name, KBUILD_MODNAME, sizeof(mdev->driver_name));
	strscpy(mdev->model, match->compatible, sizeof(mdev->model));

	cru->v4l2_dev.mdev = &cru->mdev;

	media_device_init(mdev);

	ret = rzg2l_cru_mc_parse_of_graph(cru);
	if (ret) {
		mutex_lock(&cru->mdev_lock);
		cru->v4l2_dev.mdev = NULL;
		mutex_unlock(&cru->mdev_lock);
	}

	return 0;
}

static int rzg2l_cru_probe(struct platform_device *pdev)
{
	struct rzg2l_cru_dev *cru;
	struct device *dev = &pdev->dev;
	struct v4l2_ctrl *ctrl;
	int irq, ret;

	cru = devm_kzalloc(dev, sizeof(*cru), GFP_KERNEL);
	if (!cru)
		return -ENOMEM;

	cru->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cru->base))
		return PTR_ERR(cru->base);

	cru->presetn = devm_reset_control_get_shared(dev, "presetn");
	if (IS_ERR(cru->presetn))
		return dev_err_probe(&pdev->dev, PTR_ERR(cru->presetn),
				     "Failed to get cpg presetn\n");

	cru->aresetn = devm_reset_control_get_exclusive(dev, "aresetn");
	if (IS_ERR(cru->aresetn))
		return dev_err_probe(&pdev->dev, PTR_ERR(cru->aresetn),
				     "Failed to get cpg aresetn\n");

	cru->vclk = devm_clk_get(dev, "video");
	if (IS_ERR(cru->vclk))
		return dev_err_probe(dev, PTR_ERR(cru->vclk),
				     "Failed to get video clock\n");

	cru->dev = dev;
	cru->info = of_device_get_match_data(dev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq,
			       cru->info->cru_type == RZG2L_CRU_TYPE ?
			       rzg2l_cru_irq : rzv2h_cru_irq, 0,
			       KBUILD_MODNAME, cru);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to request irq\n");

	platform_set_drvdata(pdev, cru);

	ret = rzg2l_cru_dma_register(cru);
	if (ret)
		return ret;

	cru->num_buf = RZG2L_CRU_HW_BUFFER_DEFAULT;
	pm_suspend_ignore_children(dev, true);
	pm_runtime_enable(dev);

	ret = rzg2l_cru_media_init(cru);
	if (ret)
		goto error_dma_unregister;

	/* Add the control about minimum amount of buffers */
	v4l2_ctrl_handler_init(&cru->ctrl_handler, 1);
	ctrl = v4l2_ctrl_new_std(&cru->ctrl_handler, &rzg2l_cru_ctrl_ops,
				 V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
				 1, RZG2L_CRU_HW_BUFFER_MAX, 1,
				 RZG2L_CRU_HW_BUFFER_DEFAULT);

	ctrl->flags &= ~V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_handler_setup(&cru->ctrl_handler);

	if (cru->ctrl_handler.error) {
		dev_err(&pdev->dev, "%s: control initialization error %d\n",
				    __func__, cru->ctrl_handler.error);
		ret = cru->ctrl_handler.error;
		goto free_ctrl;
	}

	cru->v4l2_dev.ctrl_handler = &cru->ctrl_handler;

	return 0;

free_ctrl:
	v4l2_ctrl_handler_free(&cru->ctrl_handler);

error_dma_unregister:
	rzg2l_cru_dma_unregister(cru);
	pm_runtime_disable(dev);

	return ret;
}

static int rzg2l_cru_remove(struct platform_device *pdev)
{
	struct rzg2l_cru_dev *cru = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	v4l2_async_nf_unregister(&cru->notifier);
	v4l2_async_nf_cleanup(&cru->notifier);

	rzg2l_cru_video_unregister(cru);
	media_device_cleanup(&cru->mdev);
	mutex_destroy(&cru->mdev_lock);

	rzg2l_cru_dma_unregister(cru);

	return 0;
}

static const u16 rzg2l_cru_regs[CRU_REGS_END] = {
	[CRUnCTRL] = 0x0,
	[CRUnIE] = 0x4,
	[CRUnINTS] = 0x8,
	[CRUnRST] = 0xC,
	[AMnMB1ADDRL] = 0x100,
	[AMnMB1ADDRH] = 0x104,
	[AMnMB2ADDRL] = 0x108,
	[AMnMB2ADDRH] = 0x10C,
	[AMnMB3ADDRL] = 0x110,
	[AMnMB3ADDRH] = 0x114,
	[AMnMB4ADDRL] = 0x118,
	[AMnMB4ADDRH] = 0x11C,
	[AMnMB5ADDRL] = 0x120,
	[AMnMB5ADDRH] = 0x124,
	[AMnMB6ADDRL] = 0x128,
	[AMnMB6ADDRH] = 0x12C,
	[AMnMB7ADDRL] = 0x130,
	[AMnMB7ADDRH] = 0x134,
	[AMnMB8ADDRL] = 0x138,
	[AMnMB8ADDRH] = 0x13C,
	[AMnMBVALID] = 0x148,
	[AMnMBS] = 0x14C,
	[AMnAXIATTR] = 0x158,
	[AMnFIFO] = 0x160,
	[AMnFIFOPNTR] = 0x168,
	[AMnAXISTP] = 0x174,
	[AMnAXISTPACK] = 0x178,
	[ICnEN] = 0x200,
	[ICnMC] = 0x208,
	[ICnMS] = 0x254,
	[ICnDMR] = 0x26C,
};

static const u16 rzv2h_cru_regs[CRU_REGS_END] = {
	[CRUnCTRL] = 0x0,
	[CRUnIE] = 0x4,
	[CRUnIE2] = 0x8,
	[CRUnINTS] = 0xC,
	[CRUnINTS2] = 0x10,
	[CRUnRST] = 0x18,
	[AMnMB1ADDRL] = 0x40,
	[AMnMB1ADDRH] = 0x44,
	[AMnMB2ADDRL] = 0x48,
	[AMnMB2ADDRH] = 0x4C,
	[AMnMB3ADDRL] = 0x50,
	[AMnMB3ADDRH] = 0x54,
	[AMnMB4ADDRL] = 0x58,
	[AMnMB4ADDRH] = 0x5c,
	[AMnMB5ADDRL] = 0x60,
	[AMnMB5ADDRH] = 0x64,
	[AMnMB6ADDRL] = 0x68,
	[AMnMB6ADDRH] = 0x6C,
	[AMnMB7ADDRL] = 0x70,
	[AMnMB7ADDRH] = 0x74,
	[AMnMB8ADDRL] = 0x78,
	[AMnMB8ADDRH] = 0x7C,
	[AMnMBVALID] = 0x88,
	[AMnMADRSL] = 0x8C,
	[AMnMADRSH] = 0x90,
	[AMnFIFO] = 0x0F0,
	[AMnFIFOPNTR] = 0xF8,
	[AMnAXIATTR] = 0x0EC,
	[AMnAXISTP] = 0x110,
	[AMnAXISTPACK] = 0x114,
	[AMnIS] = 0x128,
	[ICnEN] = 0x1F0,
	[ICnSVCNUM] = 0x1F8,
	[ICnSVC] = 0x1FC,
	[ICnIPMC_C0] = 0x200,
	[ICnMS] = 0x2D8,
	[ICnDMR] = 0x304,
	[ICnTICTRL1] = 0x35C,
	[ICnTICTRL2] = 0x360,
	[ICnTISIZE1] = 0x364,
	[ICnTISIZE2] = 0x368,
};

static const struct rzg2l_cru_info rzg2l_cru_info = {
	.cru_type = RZG2L_CRU_TYPE,
	.regs = rzg2l_cru_regs,
	.max_width = 2800,
	.max_height = 4095,
	.max_cru_channels = 1,
};

static const struct rzg2l_cru_info rzv2h_cru_info = {
	.cru_type = RZV2H_CRU_TYPE,
	.regs = rzv2h_cru_regs,
	.max_width = 4095,
	.max_height = 4095,
	.max_cru_channels = 4,
};
static const struct rzg2l_cru_info rzg3e_cru_info = {
	.cru_type = RZV2H_CRU_TYPE,
	.regs = rzv2h_cru_regs,
	.max_width = 4095,
	.max_height = 4095,
	.max_cru_channels = 1,
};

static const struct of_device_id rzg2l_cru_of_id_table[] = {
	{
		.compatible = "renesas,rzg2l-cru",
		.data = &rzg2l_cru_info,
	},
	{
		.compatible = "renesas,rzv2h-cru",
		.data = &rzv2h_cru_info,
	},
	{
		.compatible = "renesas,rzg3e-cru",
		.data = &rzg3e_cru_info,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzg2l_cru_of_id_table);

static struct platform_driver rzg2l_cru_driver = {
	.driver = {
		.name = "rzg2l-cru",
		.of_match_table = rzg2l_cru_of_id_table,
	},
	.probe = rzg2l_cru_probe,
	.remove = rzg2l_cru_remove,
};

module_platform_driver(rzg2l_cru_driver);

MODULE_AUTHOR("Lad Prabhakar <prabhakar.mahadev-lad.rj@bp.renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/G2L CRU driver");
MODULE_LICENSE("GPL");
