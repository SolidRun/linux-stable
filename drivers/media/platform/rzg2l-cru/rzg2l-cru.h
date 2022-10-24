/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Driver for Renesas RZ/G2L CRU
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 *
 * Based on the rcar_vin driver
 */

#ifndef __RZG2L_CRU__
#define __RZG2L_CRU__

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <linux/reset.h>
#include <linux/clk.h>

/* Number of HW buffers */
#define HW_BUFFER_MAX		8
#define HW_BUFFER_DEFAULT	4

/* Address alignment mask for HW buffers */
#define HW_BUFFER_MASK	0x1ff

/* Maximum bumber of CSI2 virtual channels */
#define CSI2_VCHANNEL	4

/* Time until source device reconnects */
#define CONNECTION_TIME		2000
#define SETUP_WAIT_TIME		3000

/*
 * The base for the RZ/G2L CRU driver controls.
 * We reserve 16 controls for this driver
 * The last USER-class private control IDs is V4L2_CID_USER_ATMEL_ISC_BASE.
 */

#define V4L2_CID_USER_CRU_BASE	(V4L2_CID_USER_BASE + 0x10e0)

/* V4L2 private controls */
#define V4L2_CID_CRU_FRAME_SKIP	(V4L2_CID_USER_CRU_BASE + 0)

#define V4L2_CID_CRU_LIMIT	1

static const struct v4l2_ctrl_ops rzg2l_cru_ctrl_ops;

static const struct v4l2_ctrl_config rzg2l_cru_ctrls[V4L2_CID_CRU_LIMIT] = {
	{
		.id = V4L2_CID_CRU_FRAME_SKIP,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Skipping Frames Enable/Disable",
		.max = 1,
		.min = 0,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}
};

/* Minimum skipping frame for camera sensors stability */
#define CRU_FRAME_SKIP		3

/**
 * STOPPED  - No operation in progress
 * STARTING - Capture starting up
 * RUNNING  - Operation in progress have buffers
 * STOPPING - Stopping operation
 */
enum rzg2l_cru_dma_state {
	STOPPED = 0,
	STARTING,
	RUNNING,
	STOPPING,
};

enum rzg2l_cru_fmt_types {
	YUV = 0,
	RGB,
	BAYER_RAW,
	USER_DEFINED,
};

/**
 * struct rzg2l_cru_video_format - Data format stored in memory
 * @fourcc:	Pixelformat
 * @bpp:	Bytes per pixel
 */
struct rzg2l_cru_video_format {
	u32 fourcc;
	u8 bpp;
};

/**
 * struct rzg2l_cru_parallel_entity - Parallel video input endpoint descriptor
 * @asd:	sub-device descriptor for async framework
 * @subdev:	subdevice matched using async framework
 * @mbus_type:	media bus type
 * @mbus_flags:	media bus configuration flags
 * @source_pad:	source pad of remote subdevice
 * @sink_pad:	sink pad of remote subdevice
 *
 */
struct rzg2l_cru_parallel_entity {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;

	enum v4l2_mbus_type mbus_type;
	unsigned int mbus_flags;

	unsigned int source_pad;
	unsigned int sink_pad;
};

/**
 * struct rzg2l_cru_info - Information about the particular CRU implementation
 * @max_width:		max input width the CRU supports
 * @max_height:		max input height the CRU supports
 */
struct rzg2l_cru_info {
	unsigned int max_width;
	unsigned int max_height;
};

/**
 * struct rzg2l_cru_dev - Renesas CRU device structure
 * @dev:		(OF) device
 * @base:		device I/O register space remapped to virtual memory
 * @info:		info about CRU instance
 *
 * @vdev:		V4L2 video device associated with CRU
 * @v4l2_dev:		V4L2 device
 * @ctrl_handler:	V4L2 control handler
 * @notifier:		V4L2 asynchronous subdevs notifier
 *
 * @parallel:		parallel input subdevice descriptor
 *
 * @group:		Gen3 CSI group
 * @pad:		media pad for the video device entity
 *
 * @lock:		protects @queue
 * @queue:		vb2 buffers queue
 * @scratch:		cpu address for scratch buffer
 * @scratch_phys:	physical address of the scratch buffer
 *
 * @qlock:		protects @queue_buf, @buf_list, @sequence
 *			@state
 * @queue_buf:		Keeps track of buffers given to HW slot
 * @buf_list:		list of queued buffers
 * @sequence:		V4L2 buffers sequence number
 * @state:		keeps track of operation state
 *
 * @is_csi:		flag to mark the CRU as using a CSI-2 subdevice
 *
 * @input_is_yuv:	flag to mark the input format of CRU
 * @output_is_yuv:	flag to mark the output format of CRU
 *
 * @mbus_code:		media bus format code
 * @format:		active V4L2 pixel format
 *
 * @crop:		active cropping
 * @source:		active size of the video source
 * @std:		active video standard of the video source
 *
 * @work_queue:		work queue at resuming
 * @rzg2l_cru_resume:	delayed work at resuming
 * @setup_wait:		wait queue used to setup VIN
 * @suspend:		suspend flag
 */
struct rzg2l_cru_dev {
	struct device *dev;
	void __iomem *base;
	const struct rzg2l_cru_info *info;

	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	u8 num_buf;
	struct v4l2_async_notifier notifier;

	struct rzg2l_cru_parallel_entity *parallel;

	struct rzg2l_cru_group *group;
	struct media_pad pad;

	struct mutex lock;
	struct vb2_queue queue;
	void *scratch;
	dma_addr_t scratch_phys;

	spinlock_t qlock;
	struct vb2_v4l2_buffer *queue_buf[HW_BUFFER_MAX];
	struct list_head buf_list;
	unsigned int sequence;
	enum rzg2l_cru_dma_state state;

	bool is_csi;

	enum rzg2l_cru_fmt_types input_fmt;
	enum rzg2l_cru_fmt_types output_fmt;

	u32 mbus_code;
	struct v4l2_pix_format format;

	struct v4l2_rect crop;
	struct v4l2_rect compose;
	struct v4l2_rect source;
	v4l2_std_id std;

	struct {
		struct reset_control *presetn;
		struct reset_control *aresetn;
	} rstc;

	struct workqueue_struct *work_queue;
	struct delayed_work rzg2l_cru_resume;
	wait_queue_head_t setup_wait;
	bool suspend;
	bool is_frame_skip;

	struct task_struct *retry_thread;
};

#define cru_to_source(cru)		((cru)->parallel->subdev)

/* Debug */
#define cru_dbg(d, fmt, arg...)		dev_dbg(d->dev, fmt, ##arg)
#define cru_info(d, fmt, arg...)	dev_info(d->dev, fmt, ##arg)
#define cru_warn(d, fmt, arg...)	dev_warn(d->dev, fmt, ##arg)
#define cru_err(d, fmt, arg...)		dev_err(d->dev, fmt, ##arg)

/**
 * struct rzg2l_cru_group - CRU CSI2 group information
 * @mdev:	media device which represents the group
 *
 * @notifier:	group notifier for CSI-2 async subdevices
 * @cru:	CRU instances which are part of the group
 * @csi:	array of pairs of fwnode and subdev pointers
 *		to all CSI-2 subdevices.
 */
struct rzg2l_cru_group {
	struct media_device mdev;

	struct v4l2_async_notifier notifier;
	struct rzg2l_cru_dev *cru;

	struct {
		struct fwnode_handle *fwnode;
		struct v4l2_subdev *subdev;

		u32 channel;
	} csi;
};

int rzg2l_cru_dma_register(struct rzg2l_cru_dev *cru, int irq);
void rzg2l_cru_dma_unregister(struct rzg2l_cru_dev *cru);

int rzg2l_cru_v4l2_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_v4l2_unregister(struct rzg2l_cru_dev *cru);

const struct rzg2l_cru_video_format
*rzg2l_cru_format_from_pixel(u32 pixelformat);

void rzg2l_cru_resume_start_streaming(struct work_struct *work);
void rzg2l_cru_suspend_stop_streaming(struct rzg2l_cru_dev *cru);

int rzg2l_cru_init_csi_dphy(struct v4l2_subdev *sd);
#endif
