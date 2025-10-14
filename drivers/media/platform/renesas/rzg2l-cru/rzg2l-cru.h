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

#define CONNECTION_TIME 2000
#define SETUP_WAIT_TIME 3000

/* Number of HW buffers */
#define RZG2L_CRU_HW_BUFFER_MAX		8
#define RZG2L_CRU_HW_BUFFER_DEFAULT	3

/* Address alignment mask for HW buffers */
#define RZG2L_CRU_HW_BUFFER_MASK	0x1ff

/* Maximum number of CSI2 virtual channels */
#define RZG2L_CRU_CSI2_VCHANNEL		4

#define RZG2L_CRU_MIN_INPUT_WIDTH	320
#define RZG2L_CRU_MIN_INPUT_HEIGHT	240

#define RZG2L_CRU_MAX			4

enum rzg2l_csi2_pads {
	RZG2L_CRU_IP_SINK = 0,
	RZG2L_CRU_IP_SOURCE,
};

/*
 * The base for the RZ/G2L CRU driver controls.
 * We reserve 16 controls for this driver
 * The last USER-class private control IDs is V4L2_CID_USER_DW100_BASE.
 */

#define V4L2_CID_USER_CRU_BASE	(V4L2_CID_USER_BASE + 0x11a0)

/* CRU V4L2 private controls */
enum rzg2l_cru_v4l2_priv_ctrls {
	V4L2_CID_CRU_FRAME_SKIP = V4L2_CID_USER_CRU_BASE,
	V4L2_CID_CRU_LINEAR_MATRIX,
	V4L2_CID_CRU_LINEAR_MATRIX_ROF,
	V4L2_CID_CRU_LINEAR_MATRIX_GOF,
	V4L2_CID_CRU_LINEAR_MATRIX_BOF,
	V4L2_CID_CRU_LINEAR_MATRIX_RR,
	V4L2_CID_CRU_LINEAR_MATRIX_RG,
	V4L2_CID_CRU_LINEAR_MATRIX_RB,
	V4L2_CID_CRU_LINEAR_MATRIX_GR,
	V4L2_CID_CRU_LINEAR_MATRIX_GG,
	V4L2_CID_CRU_LINEAR_MATRIX_GB,
	V4L2_CID_CRU_LINEAR_MATRIX_BR,
	V4L2_CID_CRU_LINEAR_MATRIX_BG,
	V4L2_CID_CRU_LINEAR_MATRIX_BB,
	V4L2_CID_CRU_STATISTICS,
	V4L2_CID_CRU_SD_BLKSIZE,
	V4L2_CID_CRU_SD_STHPOS,
	V4L2_CID_CRU_SD_STSADPOS,
};

static const struct v4l2_ctrl_ops rzg2l_cru_ctrl_ops;

static const char * const cru_statistics_blksize_menu[] = {
	"16x16",
	"32x32",
	"64x64",
	"128x128",
};

static const struct v4l2_ctrl_config rzg2l_cru_ctrls[] = {
	{
		.id = V4L2_CID_CRU_FRAME_SKIP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Disable or enable skipping frame with the amount of setting number",
		.max = 127,
		.min = 0,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix Processing Enable/Disable",
		.max = 1,
		.min = 0,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_ROF,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix R offset",
		.max = 127,
		.min = -128,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_GOF,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix G offset",
		.max = 127,
		.min = -128,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_BOF,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix B offset",
		.max = 127,
		.min = -128,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_RR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix RR coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_RG,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix RG coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_RB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix RB coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_GR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix GR coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_GG,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix GG coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_GB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix GB coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix BR coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_BG,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix BG coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_LINEAR_MATRIX_BB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Linear Matrix BB coefficient ",
		.max = 4095,
		.min = -4096,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_STATISTICS,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Statistics Data Enable/Disable",
		.max = 1,
		.min = 0,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_SD_BLKSIZE,
		.type = V4L2_CTRL_TYPE_MENU,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Statistics Data Unit Blocksize",
		.max = 3,
		.min = 0,
		.def = 0,
		.is_private = 1,
		.qmenu = cru_statistics_blksize_menu,
	}, {
		.id = V4L2_CID_CRU_SD_STHPOS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Statistics Horizontal Start Position",
		.max = 376,
		.min = 0,
		.step = 1,
		.def = 0,
		.is_private = 1,
	}, {
		.id = V4L2_CID_CRU_SD_STSADPOS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.ops = &rzg2l_cru_ctrl_ops,
		.name = "Statistics Input Data Bit Position",
		.max = 8,
		.min = 0,
		.step = 1,
		.def = 0,
		.is_private = 1,
	},
};

struct rzg2l_cru_dev;

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

/**
 * struct rzg2l_cru_parallel - Parallel video input endpoint descriptor
 * @asd:        sub-device descriptor for async framework
 * @subdev:     subdevice matched using async framework
 * @mbus_type:  media bus type
 * @mbus_flags: media bus configuration flags
 * @source_pad: source pad of remote subdevice
 * @sink_pad:   sink pad of remote subdevice
 *
 */
struct rzg2l_cru_parallel {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;

	enum v4l2_mbus_type mbus_type;
	unsigned int mbus_flags;

	unsigned int source_pad;
	unsigned int sink_pad;
};

struct rzg2l_cru_csi {
	struct v4l2_async_subdev *asd;
	struct v4l2_subdev *subdev;
};

struct rzg2l_cru_ip {
	struct v4l2_subdev subdev;
	struct media_pad pads[2];
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *remote;
};

/**
 * struct rzg2l_cru_ip_format - CRU IP format
 * @codes: Array of up to four media bus codes
 * @datatype: MIPI CSI2 data type
 * @format: 4CC format identifier (V4L2_PIX_FMT_*)
 * @icndmr: ICnDMR register value
 * @fmt_types: specifies the pixel encoding value(YUV, RGB or BAYER).
 */
struct rzg2l_cru_ip_format {
	/*
	 * RAW output formats might be produced by RAW media codes with any one
	 * of the 4 common bayer patterns.
	 */
	u32 codes[4];
	u32 datatype;
	u32 format;
	u32 icndmr;
	enum v4l2_pixel_encoding fmt_types;
};

enum rz_cru_type {
	RZG2L_CRU_TYPE,
	RZV2H_CRU_TYPE,
};

struct rzg2l_cru_info {
	unsigned int max_width;
	unsigned int max_height;
	u8 cru_type;
	int max_cru_channels;
	u16 image_conv;
	const u16 *regs;
	bool has_stride;
	irqreturn_t (*irq_handler)(int irq, void *data);
	void (*enable_interrupts)(struct rzg2l_cru_dev *cru);
	void (*disable_interrupts)(struct rzg2l_cru_dev *cru);
	bool (*fifo_empty)(struct rzg2l_cru_dev *cru);
	void (*csi_setup)(struct rzg2l_cru_dev *cru,
			  const struct rzg2l_cru_ip_format *ip_fmt,
			  u8 csi_vc);
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
 * @svc_channel:	SVC0/1/2/3 to use for RZ/G3E
 * @buf_addr:		Memory addresses where current video data is written.
 * @notifier:		V4L2 asynchronous subdevs notifier
 *
 * @ctrl_handler:	V4L2 control handler associated with CRU
 *
 * @ip:			Image processing subdev info
 * @parallel:		parallel input subdevice descriptor
 * @csi:		CSI info
 * @mdev:		media device
 * @mdev_lock:		protects the count, notifier and csi members
 * @pad:		media pad for the video device entity
 *
 * @is_csi:		flag to mark the CRU as using a CSI-2 subdevice
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

	u8 bpp;
	u8 svc_channel;
	dma_addr_t buf_addr[RZG2L_CRU_HW_BUFFER_MAX];

	u32 code;
	struct v4l2_async_notifier notifier;

	struct v4l2_ctrl_handler ctrl_handler;

	struct rzg2l_cru_parallel *parallel;
	struct rzg2l_cru_ip ip;
	struct rzg2l_cru_csi csi;
	struct media_device mdev;
	struct mutex mdev_lock;
	struct media_pad pad;

	bool is_csi;

	struct mutex lock;
	struct vb2_queue queue;
	void *scratch;
	dma_addr_t scratch_phys;

	spinlock_t qlock;
	struct vb2_v4l2_buffer *queue_buf[RZG2L_CRU_HW_BUFFER_MAX];
	struct list_head buf_list;
	unsigned int sequence;
	enum rzg2l_cru_dma_state state;

	struct v4l2_rect crop;
	struct v4l2_rect compose;
	struct v4l2_rect source;

	struct v4l2_pix_format format;
	u8 frame_skip;

	bool is_statistics;
	int sd_blksize;
	int sd_sthpos;
	int sd_stsadpos;

	bool is_linear_matrix_enable;
	int linear_matrix_rgb_offset[3];
	int linear_matrix_r[3];	/* RR, RG, RB */
	int linear_matrix_g[3]; /* GR, GG, GB */
	int linear_matrix_b[3]; /* BR, BG, BB */

	int id;
	struct workqueue_struct *work_queue;
	struct delayed_work rzg2l_cru_resume;
	wait_queue_head_t setup_wait;
	bool suspend;
};

int rzg2l_cru_start_image_processing(struct rzg2l_cru_dev *cru);
void rzg2l_cru_stop_image_processing(struct rzg2l_cru_dev *cru);

int rzg2l_cru_dma_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_dma_unregister(struct rzg2l_cru_dev *cru);

int rzg2l_cru_video_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_video_unregister(struct rzg2l_cru_dev *cru);
irqreturn_t rzg2l_cru_irq(int irq, void *data);
irqreturn_t rzg3e_cru_irq(int irq, void *data);

const struct v4l2_format_info *rzg2l_cru_format_from_pixel(u32 format);

int rzg2l_cru_ip_subdev_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_ip_subdev_unregister(struct rzg2l_cru_dev *cru);
struct v4l2_mbus_framefmt *rzg2l_cru_ip_get_src_fmt(struct rzg2l_cru_dev *cru);

const struct rzg2l_cru_ip_format *rzg2l_cru_ip_code_to_fmt(unsigned int code);
const struct rzg2l_cru_ip_format *rzg2l_cru_ip_format_to_fmt(u32 format);
const struct rzg2l_cru_ip_format *rzg2l_cru_ip_index_to_fmt(u32 index);
bool rzg2l_cru_ip_fmt_supports_mbus_code(const struct rzg2l_cru_ip_format *fmt,
					 unsigned int code);

void rzg2l_cru_resume_start_streaming(struct work_struct *work);
void rzg2l_cru_suspend_stop_streaming(struct rzg2l_cru_dev *cru);

void rzg2l_cru_enable_interrupts(struct rzg2l_cru_dev *cru);
void rzg2l_cru_disable_interrupts(struct rzg2l_cru_dev *cru);
void rzg3e_cru_enable_interrupts(struct rzg2l_cru_dev *cru);
void rzg3e_cru_disable_interrupts(struct rzg2l_cru_dev *cru);

bool rzg2l_fifo_empty(struct rzg2l_cru_dev *cru);
bool rzg3e_fifo_empty(struct rzg2l_cru_dev *cru);
void rzg2l_cru_csi2_setup(struct rzg2l_cru_dev *cru,
			  const struct rzg2l_cru_ip_format *ip_fmt,
			  u8 csi_vc);
void rzg3e_cru_csi2_setup(struct rzg2l_cru_dev *cru,
			  const struct rzg2l_cru_ip_format *ip_fmt,
			  u8 csi_vc);

#endif
