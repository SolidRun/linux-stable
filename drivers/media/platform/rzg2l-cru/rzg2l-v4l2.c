// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Renesas RZ/G2L CRU
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 *
 * Based on the rcar_cru driver
 */

#include <linux/pm_runtime.h>

#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-rect.h>

#include "rzg2l-cru.h"

#define RZG2L_CRU_DEFAULT_FORMAT	V4L2_PIX_FMT_YUYV
#define RZG2L_CRU_DEFAULT_WIDTH		800
#define RZG2L_CRU_DEFAULT_HEIGHT	600
#define RZG2L_CRU_DEFAULT_FIELD		V4L2_FIELD_NONE
#define RZG2L_CRU_DEFAULT_COLORSPACE	V4L2_COLORSPACE_SRGB

/* -----------------------------------------------------------------------------
 * Format Conversions
 */

static const struct rzg2l_cru_video_format rzg2l_cru_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_NV16,
		.bpp			= 1,
	},
	{
		.fourcc			= V4L2_PIX_FMT_GREY,
		.bpp			= 1,
	},
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_BGR24,
		.bpp			= 3,
	},
	{
		.fourcc			= V4L2_PIX_FMT_XBGR32,
		.bpp			= 4,
	},
	{
		.fourcc			= V4L2_PIX_FMT_ABGR32,
		.bpp			= 4,
	},
	{
		.fourcc			= V4L2_PIX_FMT_ARGB32,
		.bpp			= 4,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SRGGB8,
		.bpp			= 1,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR8,
		.bpp			= 1,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGRBG8,
		.bpp			= 1,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGBRG8,
		.bpp			= 1,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SRGGB10,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR10,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGRBG10,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGBRG10,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SRGGB12,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR12,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGRBG12,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGBRG12,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SRGGB14P,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR14P,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGRBG14P,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGBRG14P,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SRGGB16,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR16,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGRBG16,
		.bpp			= 2,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SGBRG16,
		.bpp			= 2,
	},
};

const struct rzg2l_cru_video_format
*rzg2l_cru_format_from_pixel(u32 pixelformat)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rzg2l_cru_formats); i++)
		if (rzg2l_cru_formats[i].fourcc == pixelformat)
			return rzg2l_cru_formats + i;

	return NULL;
}

static u32 rzg2l_cru_format_bytesperline(struct v4l2_pix_format *pix)
{
	const struct rzg2l_cru_video_format *fmt;

	fmt = rzg2l_cru_format_from_pixel(pix->pixelformat);

	if (WARN_ON(!fmt))
		return -EINVAL;

	return pix->width * fmt->bpp;
}

static u32 rzg2l_cru_format_sizeimage(struct v4l2_pix_format *pix)
{
	if (pix->pixelformat == V4L2_PIX_FMT_NV16)
		return pix->bytesperline * pix->height * 2;

	return pix->bytesperline * pix->height;
}

static void rzg2l_cru_format_align(struct rzg2l_cru_dev *cru,
				   struct v4l2_pix_format *pix)
{
	if (!rzg2l_cru_format_from_pixel(pix->pixelformat))
		pix->pixelformat = RZG2L_CRU_DEFAULT_FORMAT;

	switch (pix->field) {
	case V4L2_FIELD_TOP:
	case V4L2_FIELD_BOTTOM:
	case V4L2_FIELD_NONE:
	case V4L2_FIELD_INTERLACED_TB:
	case V4L2_FIELD_INTERLACED_BT:
	case V4L2_FIELD_INTERLACED:
		break;
	case V4L2_FIELD_ALTERNATE:
		/*
		 * Driver does not (yet) support outputting ALTERNATE to a
		 * userspace. It does support outputting INTERLACED so use
		 * the CRU hardware to combine the two fields.
		 */
		pix->field = V4L2_FIELD_INTERLACED;
		pix->height *= 2;
		break;
	default:
		pix->field = RZG2L_CRU_DEFAULT_FIELD;
		break;
	}

	/* Limit to CRU capabilities */
	v4l_bound_align_image(&pix->width, 320, cru->info->max_width, 1,
			      &pix->height, 240, cru->info->max_height, 2, 0);

	pix->bytesperline = rzg2l_cru_format_bytesperline(pix);
	pix->sizeimage = rzg2l_cru_format_sizeimage(pix);

	cru_dbg(cru, "Format %ux%u bpl: %u size: %u\n",
		pix->width, pix->height, pix->bytesperline, pix->sizeimage);
}

static void rzg2l_cru_mc_try_format(struct rzg2l_cru_dev *cru,
				    struct v4l2_pix_format *pix)
{
	/*
	 * The V4L2 specification clearly documents the colorspace fields
	 * as being set by drivers for capture devices. Using the values
	 * supplied by userspace thus wouldn't comply with the API. Until
	 * the API is updated force fixed values.
	 */
	pix->colorspace = RZG2L_CRU_DEFAULT_COLORSPACE;
	pix->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);
	pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	pix->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, pix->colorspace,
							  pix->ycbcr_enc);

	rzg2l_cru_format_align(cru, pix);
}


static int rzg2l_cru_get_sd_format(struct rzg2l_cru_dev *cru,
				   struct v4l2_pix_format *pix)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	/* Get cropping size */
	pad = media_entity_remote_pad(&cru->pad);
	if (!pad)
		return -EPIPE;

	sd = media_entity_to_v4l2_subdev(pad->entity);
	if (!sd)
		return -EPIPE;

	if (v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt))
		return -EPIPE;

	cru->source.width = pix->width = fmt.format.width;
	cru->source.height = pix->height = fmt.format.height;
	cru->crop = cru->source;

	if (fmt.format.field == V4L2_FIELD_ALTERNATE)
		cru->format.field = V4L2_FIELD_INTERLACED;
	else
		cru->format.field = fmt.format.field;

	cru->format.bytesperline =
				rzg2l_cru_format_bytesperline(&cru->format);

	return 0;
}

static int rzg2l_cru_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);

	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "RZG2L_CRU", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(cru->dev));
	return 0;
}

static int rzg2l_cru_mc_try_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);

	rzg2l_cru_mc_try_format(cru, &f->fmt.pix);

	return 0;
}

static int rzg2l_cru_mc_s_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);

	if (vb2_is_busy(&cru->queue))
		return -EBUSY;

	rzg2l_cru_mc_try_format(cru, &f->fmt.pix);

	cru->format = f->fmt.pix;

	cru->crop.top = 0;
	cru->crop.left = 0;
	cru->crop.width = cru->format.width;
	cru->crop.height = cru->format.height;
	cru->compose = cru->crop;

	return 0;
}

static int rzg2l_cru_g_fmt_vid_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);

	f->fmt.pix = cru->format;

	return 0;
}

static int rzg2l_cru_enum_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(rzg2l_cru_formats))
		return -EINVAL;

	f->pixelformat = rzg2l_cru_formats[f->index].fourcc;

	return 0;
}

static int rzg2l_cru_g_selection(struct file *file, void *fh,
			    struct v4l2_selection *s)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);
	int ret;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		ret = rzg2l_cru_get_sd_format(cru, &cru->format);
		if (ret)
			return ret;

		s->r.left = s->r.top = 0;
		s->r.width = cru->source.width;
		s->r.height = cru->source.height;
		break;
	case V4L2_SEL_TGT_CROP:
		s->r = cru->crop;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.left = s->r.top = 0;
		s->r.width = cru->format.width;
		s->r.height = cru->format.height;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r = cru->compose;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rzg2l_cru_s_selection(struct file *file, void *fh,
			    struct v4l2_selection *s)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);
	const struct rzg2l_cru_video_format *fmt;
	struct v4l2_rect r = s->r;
	struct v4l2_rect max_rect;
	struct v4l2_rect min_rect = {
		.width = 6,
		.height = 2,
	};

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	v4l2_rect_set_min_size(&r, &min_rect);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		/* Can't crop outside of source input */
		max_rect.top = max_rect.left = 0;
		max_rect.width = cru->source.width;
		max_rect.height = cru->source.height;
		v4l2_rect_map_inside(&r, &max_rect);

		v4l_bound_align_image(&r.width, 6, cru->source.width, 0,
				      &r.height, 2, cru->source.height, 0, 0);

		r.top  = clamp_t(s32, r.top, 0, cru->source.height - r.height);
		r.left = clamp_t(s32, r.left, 0, cru->source.width - r.width);

		cru->crop = s->r = r;

		cru_dbg(cru, "Cropped %dx%d@%d:%d of %dx%d\n",
			r.width, r.height, r.left, r.top,
			cru->source.width, cru->source.height);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		/* Make sure compose rect fits inside output format */
		max_rect.top = max_rect.left = 0;
		max_rect.width = cru->format.width;
		max_rect.height = cru->format.height;
		v4l2_rect_map_inside(&r, &max_rect);

		/*
		 * Composing is done by adding a offset to the buffer address,
		 * the HW wants this address to be aligned to HW_BUFFER_MASK.
		 * Make sure the top and left values meets this requirement.
		 */
		while ((r.top * cru->format.bytesperline) & HW_BUFFER_MASK)
			r.top--;

		fmt = rzg2l_cru_format_from_pixel(cru->format.pixelformat);
		while ((r.left * fmt->bpp) & HW_BUFFER_MASK)
			r.left--;

		cru->compose = s->r = r;

		cru_dbg(cru, "Compose %dx%d@%d:%d in %dx%d\n",
			r.width, r.height, r.left, r.top,
			cru->format.width, cru->format.height);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rzg2l_cru_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int rzg2l_cru_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;
	return 0;
}

static int rzg2l_cru_mc_enum_input(struct file *file, void *priv,
			      struct v4l2_input *i)
{
	if (i->index != 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(i->name, "Camera", sizeof(i->name));

	return 0;
}

static int rzg2l_cru_subscribe_event(struct v4l2_fh *fh,
				     const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_event_subscribe(fh, sub, 4, NULL);
	}
	return v4l2_ctrl_subscribe_event(fh, sub);
}

static const struct v4l2_ioctl_ops rzg2l_cru_mc_ioctl_ops = {
	.vidioc_querycap		= rzg2l_cru_querycap,
	.vidioc_try_fmt_vid_cap		= rzg2l_cru_mc_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= rzg2l_cru_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= rzg2l_cru_mc_s_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap	= rzg2l_cru_enum_fmt_vid_cap,

	.vidioc_g_selection		= rzg2l_cru_g_selection,
	.vidioc_s_selection		= rzg2l_cru_s_selection,

	.vidioc_enum_input		= rzg2l_cru_mc_enum_input,
	.vidioc_g_input			= rzg2l_cru_g_input,
	.vidioc_s_input			= rzg2l_cru_s_input,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_log_status		= v4l2_ctrl_log_status,
	.vidioc_subscribe_event		= rzg2l_cru_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* -----------------------------------------------------------------------------
 * Media controller file operations
 */

static int rzg2l_cru_mc_open(struct file *file)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);
	int ret;

	ret = mutex_lock_interruptible(&cru->lock);
	if (ret)
		return ret;

	ret = pm_runtime_get_sync(cru->dev);
	if (ret < 0)
		goto err_unlock;

	rzg2l_cru_get_sd_format(cru, &cru->format);

	ret = v4l2_pipeline_pm_get(&cru->vdev.entity);
	if (ret < 0)
		goto err_pm;

	file->private_data = cru;

	ret = v4l2_fh_open(file);
	if (ret)
		goto err_v4l2pm;

	mutex_unlock(&cru->lock);

	return 0;
err_v4l2pm:
	v4l2_pipeline_pm_put(&cru->vdev.entity);
err_pm:
	pm_runtime_put(cru->dev);
err_unlock:
	mutex_unlock(&cru->lock);

	return ret;
}

static int rzg2l_cru_mc_release(struct file *file)
{
	struct rzg2l_cru_dev *cru = video_drvdata(file);
	int ret;

	mutex_lock(&cru->lock);

	/* the release helper will cleanup any on-going streaming. */
	ret = _vb2_fop_release(file, NULL);

	v4l2_pipeline_pm_put(&cru->vdev.entity);
	pm_runtime_put(cru->dev);

	mutex_unlock(&cru->lock);

	return ret;
}

static const struct v4l2_file_operations rzg2l_cru_mc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.open		= rzg2l_cru_mc_open,
	.release	= rzg2l_cru_mc_release,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.read		= vb2_fop_read,
};

void rzg2l_cru_v4l2_unregister(struct rzg2l_cru_dev *cru)
{
	if (!video_is_registered(&cru->vdev))
		return;

	v4l2_info(&cru->v4l2_dev, "Removed %s\n",
		  video_device_node_name(&cru->vdev));

	/* Checks internaly if vdev have been init or not */
	video_unregister_device(&cru->vdev);
}

static void rzg2l_cru_notify(struct v4l2_subdev *sd,
			unsigned int notification, void *arg)
{
	struct rzg2l_cru_dev *cru =
		container_of(sd->v4l2_dev, struct rzg2l_cru_dev, v4l2_dev);

	switch (notification) {
	case V4L2_DEVICE_NOTIFY_EVENT:
		v4l2_event_queue(&cru->vdev, arg);
		break;
	default:
		break;
	}
}

int rzg2l_cru_v4l2_register(struct rzg2l_cru_dev *cru)
{
	struct video_device *vdev = &cru->vdev;
	int ret;

	cru->v4l2_dev.notify = rzg2l_cru_notify;

	/* video node */
	vdev->v4l2_dev = &cru->v4l2_dev;
	vdev->queue = &cru->queue;
	snprintf(vdev->name, sizeof(vdev->name), "CRU output");
	vdev->release = video_device_release_empty;
	vdev->lock = &cru->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;

	/* Set a default format */
	cru->format.pixelformat	= RZG2L_CRU_DEFAULT_FORMAT;
	cru->format.width = RZG2L_CRU_DEFAULT_WIDTH;
	cru->format.height = RZG2L_CRU_DEFAULT_HEIGHT;
	cru->format.field = RZG2L_CRU_DEFAULT_FIELD;
	cru->format.colorspace = RZG2L_CRU_DEFAULT_COLORSPACE;

	vdev->fops = &rzg2l_cru_mc_fops;
	vdev->ioctl_ops = &rzg2l_cru_mc_ioctl_ops;

	rzg2l_cru_format_align(cru, &cru->format);

	ret = video_register_device(&cru->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		cru_err(cru, "Failed to register video device\n");
		return ret;
	}

	video_set_drvdata(&cru->vdev, cru);

	v4l2_info(&cru->v4l2_dev, "Device registered as %s\n",
		  video_device_node_name(&cru->vdev));

	return ret;
}
