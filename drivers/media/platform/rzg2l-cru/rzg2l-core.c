// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Renesas RZ/G2L CRU
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 *
 * Based on the rcar_vin driver
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#include <media/v4l2-async.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>

#include "rzg2l-cru.h"

#define v4l2_dev_to_cru(d)	container_of(d, struct rzg2l_cru_dev, v4l2_dev)

static int rzg2l_cru_group_link_notify(struct media_link *link, u32 flags,
				       unsigned int notification)
{
	struct rzg2l_cru_group *group = container_of(link->graph_obj.mdev,
						struct rzg2l_cru_group, mdev);
	struct video_device *vdev;
	struct media_pad *csi_pad;
	struct rzg2l_cru_dev *cru = NULL;
	struct media_entity *entity;
	struct v4l2_subdev *sd;
	int ret;

	ret = v4l2_pipeline_link_notify(link, flags, notification);
	if (ret)
		return ret;

	/* Only care about link enablement for CRU nodes. */
	if (!(flags & MEDIA_LNK_FL_ENABLED) ||
	    !is_media_entity_v4l2_video_device(link->sink->entity))
		return 0;


	/*
	 * Don't allow link changes if any entity in the graph is
	 * streaming, modifying the CHSEL register fields can disrupt
	 * running streams.
	 */
	media_device_for_each_entity(entity, &group->mdev)
		if (entity->stream_count)
			return -EBUSY;

	vdev = media_entity_to_video_device(link->sink->entity);
	cru = container_of(vdev, struct rzg2l_cru_dev, vdev);

	csi_pad = media_entity_remote_pad(&group->cru->vdev.entity.pads[0]);
	if (csi_pad)
		return  -EMLINK;

	sd = media_entity_to_v4l2_subdev(link->source->entity);
	if (group->csi.subdev == sd) {
		cru->group->csi.channel = link->source->index - 1;
		cru->is_csi = true;
	} else {
		if (group->cru && group->cru->parallel &&
		    group->cru->parallel->subdev == sd)
			cru->is_csi = false;
		else
			return -ENODEV;
	}

	return 0;
}

static const struct media_device_ops rzg2l_cru_media_ops = {
	.link_notify = rzg2l_cru_group_link_notify,
};

static void rzg2l_cru_group_put(struct rzg2l_cru_dev *cru)
{
	cru->group = NULL;
	cru->v4l2_dev.mdev = NULL;
}

/* -----------------------------------------------------------------------------
 * Async notifier
 */

static int rzg2l_cru_find_pad(struct v4l2_subdev *sd, int direction)
{
	unsigned int pad;

	if (sd->entity.num_pads <= 1)
		return 0;

	for (pad = 0; pad < sd->entity.num_pads; pad++)
		if (sd->entity.pads[pad].flags & direction)
			return pad;

	return -EINVAL;
}

/* -----------------------------------------------------------------------------
 * Parallel async notifier
 */

/* The cru lock should be held when calling the subdevice attach and detach */
static int rzg2l_cru_parallel_subdevice_attach(struct rzg2l_cru_dev *cru,
					  struct v4l2_subdev *subdev)
{
	int ret;

	/* Find source and sink pad of remote subdevice */
	ret = rzg2l_cru_find_pad(subdev, MEDIA_PAD_FL_SOURCE);
	if (ret < 0)
		return ret;
	cru->parallel->source_pad = ret;

	ret = rzg2l_cru_find_pad(subdev, MEDIA_PAD_FL_SINK);
	cru->parallel->sink_pad = ret < 0 ? 0 : ret;

	cru->parallel->subdev = subdev;
	return 0;
}

static void rzg2l_cru_parallel_subdevice_detach(struct rzg2l_cru_dev *cru)
{
	rzg2l_cru_v4l2_unregister(cru);
	cru->parallel->subdev = NULL;
}

static int
rzg2l_cru_parallel_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct rzg2l_cru_dev *cru = v4l2_dev_to_cru(notifier->v4l2_dev);
	struct media_entity *source;
	struct media_entity *sink;
	int ret;

	ret = v4l2_device_register_subdev_nodes(&cru->v4l2_dev);
	if (ret < 0) {
		cru_err(cru, "Failed to register subdev nodes\n");
		return ret;
	}

	if (!video_is_registered(&cru->vdev)) {
		ret = rzg2l_cru_v4l2_register(cru);
		if (ret < 0)
			return ret;
	}

	/* If we're running with media-controller, link the subdevs. */
	source = &cru->parallel->subdev->entity;
	sink = &cru->vdev.entity;

	ret = media_create_pad_link(source, cru->parallel->source_pad,
				    sink, cru->parallel->sink_pad, 0);
	if (ret)
		cru_err(cru, "Error adding link from %s to %s: %d\n",
			source->name, sink->name, ret);

	return ret;
}

static void
rzg2l_cru_parallel_notify_unbind(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = v4l2_dev_to_cru(notifier->v4l2_dev);

	cru_dbg(cru, "unbind parallel subdev %s\n", subdev->name);

	mutex_lock(&cru->lock);
	rzg2l_cru_parallel_subdevice_detach(cru);
	mutex_unlock(&cru->lock);
}

static int rzg2l_cru_parallel_notify_bound(struct v4l2_async_notifier *notifier,
				      struct v4l2_subdev *subdev,
				      struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = v4l2_dev_to_cru(notifier->v4l2_dev);
	int ret;

	mutex_lock(&cru->lock);
	ret = rzg2l_cru_parallel_subdevice_attach(cru, subdev);
	mutex_unlock(&cru->lock);
	if (ret)
		return ret;

	v4l2_set_subdev_hostdata(subdev, cru);

	cru_dbg(cru, "bound subdev %s source pad: %u sink pad: %u\n",
		subdev->name, cru->parallel->source_pad,
		cru->parallel->sink_pad);

	return 0;
}

static const struct v4l2_async_notifier_operations
rzg2l_cru_parallel_notify_ops = {
	.bound = rzg2l_cru_parallel_notify_bound,
	.unbind = rzg2l_cru_parallel_notify_unbind,
	.complete = rzg2l_cru_parallel_notify_complete,
};

static int rzg2l_cru_parallel_parse_v4l2(struct device *dev,
				    struct v4l2_fwnode_endpoint *vep,
				    struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = dev_get_drvdata(dev);
	struct rzg2l_cru_parallel_entity *rvpe =
		container_of(asd, struct rzg2l_cru_parallel_entity, asd);

	if (vep->base.port || vep->base.id)
		return -ENOTCONN;

	cru->parallel = rvpe;
	cru->parallel->mbus_type = vep->bus_type;

	switch (cru->parallel->mbus_type) {
	case V4L2_MBUS_PARALLEL:
		cru_dbg(cru, "Found PARALLEL media bus\n");
		cru->parallel->mbus_flags = vep->bus.parallel.flags;
		break;
	case V4L2_MBUS_BT656:
		cru_dbg(cru, "Found BT656 media bus\n");
		cru->parallel->mbus_flags = 0;
		break;
	default:
		cru_err(cru, "Unknown media bus type\n");
		return -EINVAL;
	}

	return 0;
}

static int rzg2l_cru_parallel_init(struct rzg2l_cru_dev *cru)
{
	int ret;

	ret = v4l2_async_notifier_parse_fwnode_endpoints_by_port(
				cru->dev, &cru->notifier,
				sizeof(struct rzg2l_cru_parallel_entity),
				0, rzg2l_cru_parallel_parse_v4l2);
	if (ret)
		return ret;

	/* If using mc, it's fine not to have any input registered. */
	if (!cru->parallel)
		return 0;

	cru_dbg(cru, "Found parallel subdevice %pOF\n",
		to_of_node(cru->parallel->asd.match.fwnode));

	cru->notifier.ops = &rzg2l_cru_parallel_notify_ops;
	ret = v4l2_async_notifier_register(&cru->v4l2_dev, &cru->notifier);
	if (ret < 0) {
		cru_err(cru, "Notifier registration failed\n");
		v4l2_async_notifier_cleanup(&cru->notifier);
		return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Group async notifier
 */

static int rzg2l_cru_group_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct rzg2l_cru_dev *cru = v4l2_dev_to_cru(notifier->v4l2_dev);
	int ret, i;

	ret = v4l2_device_register_subdev_nodes(&cru->v4l2_dev);
	if (ret) {
		cru_err(cru, "Failed to register subdev nodes\n");
		return ret;
	}

	if (!video_is_registered(&cru->vdev)) {
		ret = rzg2l_cru_v4l2_register(cru->group->cru);
		if (ret)
			return ret;
	}

	/* Create all media device links between CRU and CSI-2's. */
	for (i = 1; i <= CSI2_VCHANNEL; i++) {
		struct media_pad *source_pad, *sink_pad;
		struct media_entity *source, *sink;
		unsigned int source_idx;

		source = &cru->group->csi.subdev->entity;
		source_idx = i;
		source_pad = &source->pads[source_idx];

		sink = &cru->group->cru->vdev.entity;
		sink_pad = &sink->pads[0];

		ret = media_create_pad_link(source, source_idx, sink, 0, 0);
		if (ret) {
			cru_err(cru, "Error adding link from %s to %s\n",
				source->name, sink->name);
			break;
		}
	}

	return ret;
}

static void rzg2l_cru_group_notify_unbind(struct v4l2_async_notifier *notifier,
					  struct v4l2_subdev *subdev,
					  struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = v4l2_dev_to_cru(notifier->v4l2_dev);

	rzg2l_cru_v4l2_unregister(cru->group->cru);

	if (cru->group->csi.fwnode == asd->match.fwnode) {
		cru->group->csi.subdev = NULL;
		cru_dbg(cru, "Unbind CSI-2 %s\n", subdev->name);
	}
}

static int rzg2l_cru_group_notify_bound(struct v4l2_async_notifier *notifier,
					struct v4l2_subdev *subdev,
					struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = v4l2_dev_to_cru(notifier->v4l2_dev);

	if (cru->group->csi.fwnode == asd->match.fwnode) {
		cru->group->csi.subdev = subdev;
		cru_dbg(cru, "Bound CSI-2 %s\n", subdev->name);
	}

	return 0;
}

static const struct v4l2_async_notifier_operations
rzg2l_cru_group_notify_ops = {
	.bound = rzg2l_cru_group_notify_bound,
	.unbind = rzg2l_cru_group_notify_unbind,
	.complete = rzg2l_cru_group_notify_complete,
};

static int rzg2l_cru_mc_parse_of_endpoint(struct device *dev,
					  struct v4l2_fwnode_endpoint *vep,
					  struct v4l2_async_subdev *asd)
{
	struct rzg2l_cru_dev *cru = dev_get_drvdata(dev);

	if (vep->base.port != 1)
		return -EINVAL;

	if (!of_device_is_available(to_of_node(asd->match.fwnode))) {
		cru_dbg(cru, "OF device %pOF disabled, ignoring\n",
			to_of_node(asd->match.fwnode));
		return -ENOTCONN;
	}

	if (cru->group->csi.fwnode) {
		cru_dbg(cru, "OF device %pOF already handled\n",
			to_of_node(asd->match.fwnode));
		return -ENOTCONN;
	}

	cru->group->csi.fwnode = asd->match.fwnode;

	cru_dbg(cru, "Add group OF device %pOF to slot %u\n",
		to_of_node(asd->match.fwnode), vep->base.id);

	return 0;
}

static int rzg2l_cru_mc_parse_of_graph(struct rzg2l_cru_dev *cru)
{
	int ret;

	cru->group->cru = cru;

	v4l2_async_notifier_init(&cru->group->notifier);
	ret = v4l2_async_notifier_parse_fwnode_endpoints_by_port(
				cru->group->cru->dev, &cru->group->notifier,
				sizeof(struct v4l2_async_subdev), 1,
				rzg2l_cru_mc_parse_of_endpoint);
	if (ret)
		return ret;

	if (list_empty(&cru->group->notifier.asd_list))
		return 0;

	cru->group->notifier.ops = &rzg2l_cru_group_notify_ops;
	ret = v4l2_async_notifier_register(&cru->v4l2_dev,
					   &cru->group->notifier);
	if (ret < 0) {
		cru_err(cru, "Notifier registration failed\n");
		v4l2_async_notifier_cleanup(&cru->group->notifier);
		return ret;
	}

	return ret;
}

static int rzg2l_cru_mc_init(struct rzg2l_cru_dev *cru)
{
	struct rzg2l_cru_group *group;
	struct media_device *mdev = NULL;
	const struct of_device_id *match;
	int ret;

	cru->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&cru->vdev.entity, 1, &cru->pad);
	if (ret)
		return ret;

	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	mdev = &group->mdev;
	mdev->dev = cru->dev;
	mdev->ops = &rzg2l_cru_media_ops;

	match = of_match_node(cru->dev->driver->of_match_table,
			      cru->dev->of_node);

	strlcpy(mdev->driver_name, KBUILD_MODNAME, sizeof(mdev->driver_name));
	strlcpy(mdev->model, match->compatible, sizeof(mdev->model));
	snprintf(mdev->bus_info, sizeof(mdev->bus_info), "platform:%s",
		 dev_name(mdev->dev));

	cru->group = group;
	cru->v4l2_dev.mdev = &group->mdev;

	media_device_init(mdev);

	ret = media_device_register(&group->mdev);
	if (ret) {
		media_device_unregister(&group->mdev);
		media_device_cleanup(&group->mdev);

		return ret;
	}

	ret = rzg2l_cru_mc_parse_of_graph(cru);
	if (ret)
		rzg2l_cru_group_put(cru);

	return ret;
}

static int rzg2l_cru_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rzg2l_cru_dev *cru = container_of(ctrl->handler,
						 struct rzg2l_cru_dev,
						 ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		if ((cru->state == STOPPED) || (cru->state == STOPPING))
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

static int rzg2l_cru_probe(struct platform_device *pdev)
{
	struct rzg2l_cru_dev *cru;
	struct resource *mem;
	int irq, ret;
	struct v4l2_ctrl *ctrl;

	cru = devm_kzalloc(&pdev->dev, sizeof(*cru), GFP_KERNEL);
	if (!cru)
		return -ENOMEM;

	cru->dev = &pdev->dev;
	cru->info = of_device_get_match_data(&pdev->dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL)
		return -EINVAL;

	cru->base = devm_ioremap_resource(cru->dev, mem);
	if (IS_ERR(cru->base))
		return PTR_ERR(cru->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	cru->rstc.cmn_rstb = devm_reset_control_get(&pdev->dev, "cmn_rstb");
	if (IS_ERR(cru->rstc.cmn_rstb)) {
		dev_err(&pdev->dev, "failed to get CRU_CMN_RSTB reset\n");
		return PTR_ERR(cru->rstc.cmn_rstb);
	}

	cru->rstc.presetn = devm_reset_control_get(&pdev->dev, "presetn");
	if (IS_ERR(cru->rstc.presetn)) {
		dev_err(&pdev->dev, "failed to get CRU_PRESETN reset\n");
		return PTR_ERR(cru->rstc.presetn);
	}

	cru->rstc.aresetn = devm_reset_control_get(&pdev->dev, "aresetn");
	if (IS_ERR(cru->rstc.aresetn)) {
		dev_err(&pdev->dev, "failed to get CRU_ARESETN reset\n");
		return PTR_ERR(cru->rstc.aresetn);
	}

	ret = rzg2l_cru_dma_register(cru, irq);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, cru);

	ret = rzg2l_cru_mc_init(cru);
	if (ret)
		goto error_dma_unregister;

	ret = rzg2l_cru_parallel_init(cru);
	if (ret)
		goto error_dma_unregister;

	/* Add the control about minimum amount of buffers */
	v4l2_ctrl_handler_init(&cru->ctrl_handler, 1);
	ctrl = v4l2_ctrl_new_std(&cru->ctrl_handler, &rzg2l_cru_ctrl_ops,
			  V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
			  2, HW_BUFFER_MAX, 1, HW_BUFFER_DEFAULT);

	ctrl->flags &= ~V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_handler_setup(&cru->ctrl_handler);

	cru->v4l2_dev.ctrl_handler = &cru->ctrl_handler;

	if (cru->ctrl_handler.error) {
		dev_err(&pdev->dev, "%s: control initialization error %d\n",
				    __func__, cru->ctrl_handler.error);
		ret = cru->ctrl_handler.error;
		goto free_ctrl;
	}

	cru->num_buf = HW_BUFFER_DEFAULT;

	pm_suspend_ignore_children(&pdev->dev, true);
	pm_runtime_enable(&pdev->dev);


	cru->work_queue = create_singlethread_workqueue(dev_name(cru->dev));
	if (!cru->work_queue) {
		ret = -ENOMEM;
		goto free_ctrl;
	}
	INIT_DELAYED_WORK(&cru->rzg2l_cru_resume,
			  rzg2l_cru_resume_start_streaming);

	return 0;

free_ctrl:
	v4l2_ctrl_handler_free(&cru->ctrl_handler);
error_dma_unregister:
	rzg2l_cru_dma_unregister(cru);

	return ret;
}

static const struct rzg2l_cru_info rzg2l_cru_info_generic = {
	.max_width = 2800,
	.max_height = 4096,
};

static const struct of_device_id rzg2l_cru_of_id_table[] = {
	{
		.compatible = "renesas,cru-r9a07g044",
		.data = &rzg2l_cru_info_generic,
	},
	{
		.compatible = "renesas,cru-r9a07g043",
		.data = &rzg2l_cru_info_generic,
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, rzg2l_cru_of_id_table);

static int rzg2l_cru_remove(struct platform_device *pdev)
{
	struct rzg2l_cru_dev *cru = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	v4l2_ctrl_handler_free(&cru->ctrl_handler);

	rzg2l_cru_v4l2_unregister(cru);

	v4l2_async_notifier_unregister(&cru->notifier);
	v4l2_async_notifier_cleanup(&cru->notifier);

	rzg2l_cru_dma_unregister(cru);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rzg2l_cru_suspend(struct device *dev)
{
	struct rzg2l_cru_dev *cru = dev_get_drvdata(dev);

	if (cru->state == STOPPED)
		return 0;

	rzg2l_cru_suspend_stop_streaming(cru);

	pm_runtime_put(cru->dev);

	return 0;
}

static int rzg2l_cru_resume(struct device *dev)
{
	struct rzg2l_cru_dev *cru = dev_get_drvdata(dev);

	if (cru->state == STOPPED)
		return 0;

	pm_runtime_get_sync(cru->dev);

	queue_delayed_work_on(0, cru->work_queue, &cru->rzg2l_cru_resume,
			      msecs_to_jiffies(CONNECTION_TIME));

	return 0;
}

static SIMPLE_DEV_PM_OPS(rzg2l_cru_pm_ops,
			 rzg2l_cru_suspend, rzg2l_cru_resume);
#define DEV_PM_OPS (&rzg2l_cru_pm_ops)
#else
#define DEV_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver rzg2l_cru_driver = {
	.driver = {
		.name = "rzg2l-cru",
		.of_match_table = rzg2l_cru_of_id_table,
		.pm = DEV_PM_OPS,
	},
	.probe = rzg2l_cru_probe,
	.remove = rzg2l_cru_remove,
};

module_platform_driver(rzg2l_cru_driver);

MODULE_AUTHOR("Hien Huynh <hien.huynh.px@renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/G2L Camera Data Receive Unit driver");
MODULE_LICENSE("GPL");
