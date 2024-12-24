/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Driver for Renesas RZ/G2L CRU
 *
 * Copyright (C) 2022 Renesas Electronics Corp.
 */

#ifndef __RZG2L_CRU__
#define __RZG2L_CRU__

#include <linux/irqreturn.h>
#include <linux/reset.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>

enum rzg2l_cru_common_regs {
	CRUnCTRL,	/* CRU Control */
	CRUnIE,		/* CRU Interrupt Enable (1) */
	CRUnIE2,	/* CRU Interrupt Enable (2) */
	CRUnINTS,	/* CRU Interrupt Status (1) */
	CRUnINTS2,	/* CRU Interrupt Status (2) */
	CRUnRST,	/* CRU Reset */
	AMnMB1ADDRL,	/* Bank 1 Address (Lower) for CRU Image Data */
	AMnMB1ADDRH,	/* Bank 1 Address (Higher) for CRU Image Data */
	AMnMB2ADDRL,    /* Bank 2 Address (Lower) for CRU Image Data */
	AMnMB2ADDRH,    /* Bank 2 Address (Higher) for CRU Image Data */
	AMnMB3ADDRL,    /* Bank 3 Address (Lower) for CRU Image Data */
	AMnMB3ADDRH,    /* Bank 3 Address (Higher) for CRU Image Data */
	AMnMB4ADDRL,    /* Bank 4 Address (Lower) for CRU Image Data */
	AMnMB4ADDRH,    /* Bank 4 Address (Higher) for CRU Image Data */
	AMnMB5ADDRL,    /* Bank 5 Address (Lower) for CRU Image Data */
	AMnMB5ADDRH,    /* Bank 5 Address (Higher) for CRU Image Data */
	AMnMB6ADDRL,    /* Bank 6 Address (Lower) for CRU Image Data */
	AMnMB6ADDRH,    /* Bank 6 Address (Higher) for CRU Image Data */
	AMnMB7ADDRL,    /* Bank 7 Address (Lower) for CRU Image Data */
	AMnMB7ADDRH,    /* Bank 7 Address (Higher) for CRU Image Data */
	AMnMB8ADDRL,    /* Bank 8 Address (Lower) for CRU Image Data */
	AMnMB8ADDRH,    /* Bank 8 Address (Higher) for CRU Image Data */
	AMnMBVALID,	/* Memory Bank Enable for CRU Image Data */
	AMnMBS,		/* Memory Bank Status for CRU Image Data */
	AMnMADRSL,	/* VD Memory Address Lower Status Register */
	AMnMADRSH,	/* VD Memory Address Higher Status Register */
	AMnAXIATTR,	/* AXI-VD Bus Master Transfer Setting Register */
	AMnFIFO,        /* AXI-VD Master FIFO Setting Register */
	AMnFIFOPNTR,	/* AXI Master FIFO Pointer for CRU Image Data */
	AMnAXISTP,	/* AXI Master Transfer Stop for CRU Image Data */
	AMnAXISTPACK,	/* AXI Master Transfer Stop Status for CRU Image Data */
	AMnIS,		/* Image Stride Setting Register */
	ICnEN,		/* CRU Image Processing Enable */
	ICnSVCNUM,	/* CRU SVC Number Register */
	ICnSVC,		/* CRU VC Select Register */
	ICnMC,		/* CRU Image Processing Main Control */
	ICnIPMC_C0,	/* CRU Image Converter Main Control 0 */
	ICnMS,		/* CRU Module Status */
	ICnDMR,		/* CRU Data Output Mode */
	ICnTICTRL1,	/* CRU Test Image Generation Control 1 Register */
	ICnTICTRL2,	/* CRU Test Image Generation Control 2 Register */
	ICnTISIZE1,	/* CRU Test Image Size Setting 1 Register */
	ICnTISIZE2,	/* CRU Test Image Size Setting 2 Register */

	CRU_REGS_END,
};

enum rz_cru_type {
	RZG2L_CRU_TYPE,
	RZV2H_CRU_TYPE,
};

#define RZG2L_CRU_MAX			4


/* Number of HW buffers */
#define RZG2L_CRU_HW_BUFFER_MAX		8
#define RZG2L_CRU_HW_BUFFER_DEFAULT	3

/* Address alignment mask for HW buffers */
#define RZG2L_CRU_HW_BUFFER_MASK	0x1ff

/* Maximum number of CSI2 virtual channels */
#define RZG2L_CRU_CSI2_VCHANNEL		4

#define RZG2L_CRU_MIN_INPUT_WIDTH	320
#define RZG2L_CRU_MIN_INPUT_HEIGHT	240

/**
 * enum rzg2l_cru_dma_state - DMA states
 * @RZG2L_CRU_DMA_STOPPED:   No operation in progress
 * @RZG2L_CRU_DMA_STARTING:  Capture starting up
 * @RZG2L_CRU_DMA_RUNNING:   Operation in progress have buffers
 * @RZG2L_CRU_DMA_STOPPING:  Stopping operation
 */
enum rzg2l_cru_dma_state {
	RZG2L_CRU_DMA_STOPPED = 0,
	RZG2L_CRU_DMA_STARTING,
	RZG2L_CRU_DMA_RUNNING,
	RZG2L_CRU_DMA_STOPPING,
};

struct rzg2l_cru_csi {
	struct v4l2_async_subdev *asd;
	struct v4l2_subdev *subdev;
	u32 channel;
};

struct rzg2l_cru_ip {
	struct v4l2_subdev subdev;
	struct media_pad pads[2];
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *remote;
};

struct rzg2l_cru_info {
	const u16 *regs;
	u8 cru_type;
	unsigned int max_width;
	unsigned int max_height;
	int max_cru_channels;
};

/**
 * struct rzg2l_cru_dev - Renesas CRU device structure
 * @dev:		(OF) device
 * @base:		device I/O register space remapped to virtual memory
 * @info:		info about CRU instance
 *
 * @presetn:		CRU_PRESETN reset line
 * @aresetn:		CRU_ARESETN reset line
 *
 * @vclk:		CRU Main clock
 *
 * @vdev:		V4L2 video device associated with CRU
 * @v4l2_dev:		V4L2 device
 * @num_buf:		Holds the current number of buffers enabled
 * @svc_channel:	SVC0/1/2/3 to use
 * @notifier:		V4L2 asynchronous subdevs notifier
 *
 * @ctrl_handler:	V4L2 control handler associated with CRU
 *
 * @ip:			Image processing subdev info
 * @csi:		CSI info
 * @mdev:		media device
 * @mdev_lock:		protects the count, notifier and csi members
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
 * @format:		active V4L2 pixel format
 */
struct rzg2l_cru_dev {
	struct device *dev;
	void __iomem *base;
	const struct rzg2l_cru_info *info;

	struct reset_control *presetn;
	struct reset_control *aresetn;

	struct clk *vclk;

	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	u8 num_buf;

	u8 svc_channel;

	struct v4l2_async_notifier notifier;

	struct v4l2_ctrl_handler ctrl_handler;

	struct rzg2l_cru_ip ip;
	struct rzg2l_cru_csi csi;
	struct media_device mdev;
	struct mutex mdev_lock;
	struct media_pad pad;

	struct mutex lock;
	struct vb2_queue queue;
	void *scratch;
	dma_addr_t scratch_phys;

	spinlock_t qlock;
	struct vb2_v4l2_buffer *queue_buf[RZG2L_CRU_HW_BUFFER_MAX];
	struct list_head buf_list;
	unsigned int sequence;
	enum rzg2l_cru_dma_state state;

	struct v4l2_pix_format format;

	int id;
};

int rzg2l_cru_start_image_processing(struct rzg2l_cru_dev *cru);
void rzg2l_cru_stop_image_processing(struct rzg2l_cru_dev *cru);

int rzg2l_cru_dma_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_dma_unregister(struct rzg2l_cru_dev *cru);

int rzg2l_cru_video_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_video_unregister(struct rzg2l_cru_dev *cru);
irqreturn_t rzg2l_cru_irq(int irq, void *data);
irqreturn_t rzv2h_cru_irq(int irq, void *data);

const struct v4l2_format_info *rzg2l_cru_format_from_pixel(u32 format);

int rzg2l_cru_ip_subdev_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_ip_subdev_unregister(struct rzg2l_cru_dev *cru);
struct v4l2_mbus_framefmt *rzg2l_cru_ip_get_src_fmt(struct rzg2l_cru_dev *cru);

#endif
