// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Renesas RZ/G2L MIPI CSI-2 Receiver
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/sys_soc.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include "rzg2l-cru.h"

/* LINK registers */
/* Module Configuration Register */
#define CSI2nMCG			0x0
#define CSI2nMCG_SDLN			GENMASK(11, 8)

/* Module Control Register 0 */
#define CSI2nMCT0			0x10
#define CSI2nMCT0_VDLN(x)		((x) << 0)

/* Module Control Register 2 */
#define CSI2nMCT2			0x18
#define CSI2nMCT2_FRRSKW(x)		((x) << 16)
#define CSI2nMCT2_FRRCLK(x)		((x) << 0)

/* Module Control Register 3 */
#define CSI2nMCT3			0x1C
#define CSI2nMCT3_RXEN			BIT(0)

/* Reset Control Register */
#define CSI2nRTCT			0x28
#define CSI2nRTCT_VSRST			BIT(0)

/* Reset Status Register */
#define CSI2nRTST			0x2C
#define CSI2nRTST_VSRSTS		BIT(0)

/* Receive Data Type Enable Low Register */
#define CSI2nDTEL			0x60

/* Receive Data Type Enable High Register */
#define CSI2nDTEH			0x64

/* DPHY registers */
/* D-PHY Control Register 0 */
#define CSIDPHYCTRL0			0x400
#define CSIDPHYCTRL0_EN_LDO1200		BIT(1)
#define CSIDPHYCTRL0_EN_BGR		BIT(0)

/* D-PHY Timing Register 0 */
#define CSIDPHYTIM0			0x404
#define CSIDPHYTIM0_TCLK_MISS(x)	((x) << 24)
#define CSIDPHYTIM0_T_INIT(x)		((x) << 0)

/* D-PHY Timing Register 1 */
#define CSIDPHYTIM1			0x408
#define CSIDPHYTIM1_THS_PREPARE(x)	((x) << 24)
#define CSIDPHYTIM1_TCLK_PREPARE(x)	((x) << 16)
#define CSIDPHYTIM1_THS_SETTLE(x)	((x) << 8)
#define CSIDPHYTIM1_TCLK_SETTLE(x)	((x) << 0)

/* D-PHY Skew Adjustment Function */
#define CSIDPHYSKW0			0x460

/* D-PHY Timing Setting Values for over 360 Mbps Transmission Rate */
#define DPHY_TIMING_T_INIT		79801
#define DPHY_TIMING_TCLK_MISS		4
#define DPHY_TIMING_TCLK_SETTLE		18
#define DPHY_TIMING_THS_SETTLE		18
#define DPHY_TIMING_TCLK_PREPARE	10
#define DPHY_TIMING_THS_PREPARE		10

struct timings {
	u32 t_init;
	u32 tclk_miss;
	u32 tclk_settle;
	u32 ths_settle;
	u32 tclk_prepare;
	u32 ths_prepare;
};

struct rzg2l_csi2_format {
	u32 code;
	unsigned int bpp;
};

static const struct rzg2l_csi2_format rzg2l_csi2_formats[] = {
	{ .code = MEDIA_BUS_FMT_RGB565_2X8_LE,	.bpp = 16 },
	{ .code = MEDIA_BUS_FMT_RGB888_1X24,	.bpp = 24 },
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16,	.bpp = 16 },
	{ .code = MEDIA_BUS_FMT_YUYV8_1X16,	.bpp = 16 },
	{ .code = MEDIA_BUS_FMT_UYVY8_2X8,	.bpp = 16 },
	{ .code = MEDIA_BUS_FMT_YUYV10_2X10,	.bpp = 20 },
	{ .code = MEDIA_BUS_FMT_SRGGB8_1X8,	.bpp = 8 },
	{ .code = MEDIA_BUS_FMT_SGRBG8_1X8,	.bpp = 8 },
	{ .code = MEDIA_BUS_FMT_SGBRG8_1X8,	.bpp = 8 },
	{ .code = MEDIA_BUS_FMT_SBGGR8_1X8,	.bpp = 8 },
	{ .code = MEDIA_BUS_FMT_SRGGB10_1X10,	.bpp = 10 },
	{ .code = MEDIA_BUS_FMT_SGRBG10_1X10,	.bpp = 10 },
	{ .code = MEDIA_BUS_FMT_SGBRG10_1X10,	.bpp = 10 },
	{ .code = MEDIA_BUS_FMT_SBGGR10_1X10,	.bpp = 10 },
	{ .code = MEDIA_BUS_FMT_SRGGB12_1X12,	.bpp = 12 },
	{ .code = MEDIA_BUS_FMT_SGRBG12_1X12,	.bpp = 12 },
	{ .code = MEDIA_BUS_FMT_SGBRG12_1X12,	.bpp = 12 },
	{ .code = MEDIA_BUS_FMT_SBGGR12_1X12,	.bpp = 12 },
	{ .code = MEDIA_BUS_FMT_SRGGB14_1X14,	.bpp = 14 },
	{ .code = MEDIA_BUS_FMT_SGRBG14_1X14,	.bpp = 14 },
	{ .code = MEDIA_BUS_FMT_SGBRG14_1X14,	.bpp = 14 },
	{ .code = MEDIA_BUS_FMT_SBGGR14_1X14,	.bpp = 14 },
	{ .code = MEDIA_BUS_FMT_SRGGB16_1X16,	.bpp = 16 },
	{ .code = MEDIA_BUS_FMT_SGRBG16_1X16,	.bpp = 16 },
	{ .code = MEDIA_BUS_FMT_SGBRG16_1X16,	.bpp = 16 },
	{ .code = MEDIA_BUS_FMT_SBGGR16_1X16,	.bpp = 16 },
};

static const struct rzg2l_csi2_format *rzg2l_csi2_code_to_fmt(unsigned int code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rzg2l_csi2_formats); i++)
		if (rzg2l_csi2_formats[i].code == code)
			return &rzg2l_csi2_formats[i];

	return NULL;
}

enum rzg2l_csi2_pads {
	RZG2L_CSI2_SINK,
	RZG2L_CSI2_SOURCE_VC0,
	RZG2L_CSI2_SOURCE_VC1,
	RZG2L_CSI2_SOURCE_VC2,
	RZG2L_CSI2_SOURCE_VC3,
	NR_OF_RZG2L_CSI2_PAD,
};

struct rzg2l_csi2 {
	struct device *dev;
	void __iomem *base;

	struct v4l2_subdev subdev;
	struct media_pad pads[NR_OF_RZG2L_CSI2_PAD];

	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *remote;

	struct v4l2_mbus_framefmt mf;

	struct clk *vclk;
	struct clk *sysclk;

	struct reset_control *cmn_rstb;

	struct mutex lock;
	int stream_count;
	int power_count;

	unsigned short lanes;

	unsigned long hsfreq;
};

static inline struct rzg2l_csi2 *sd_to_csi2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rzg2l_csi2, subdev);
}

static inline struct rzg2l_csi2 *notifier_to_csi2(struct v4l2_async_notifier *n)
{
	return container_of(n, struct rzg2l_csi2, notifier);
}

static u32 rzg2l_csi2_read(struct rzg2l_csi2 *priv, unsigned int reg)
{
	return ioread32(priv->base + reg);
}

static void rzg2l_csi2_write(struct rzg2l_csi2 *priv, unsigned int reg,
			     u32 data)
{
	iowrite32(data, priv->base + reg);
}

static void rzg2l_csi2_set(struct rzg2l_csi2 *priv, unsigned int reg, u32 set)
{
	rzg2l_csi2_write(priv, reg, rzg2l_csi2_read(priv, reg) | set);
}

static void rzg2l_csi2_clr(struct rzg2l_csi2 *priv, unsigned int reg, u32 clr)
{
	rzg2l_csi2_write(priv, reg, rzg2l_csi2_read(priv, reg) & ~clr);
}

/* -----------------------------------------------------------------------------
 * DPHY setting
 */

static int rzg2l_csi2_dphy_setting(struct rzg2l_csi2 *priv, bool on)
{
	struct timings dphy_timing = {
		.t_init         = DPHY_TIMING_T_INIT,
		.tclk_miss      = DPHY_TIMING_TCLK_MISS,
		.tclk_settle    = DPHY_TIMING_TCLK_SETTLE,
		.ths_settle     = DPHY_TIMING_THS_SETTLE,
		.tclk_prepare   = DPHY_TIMING_TCLK_PREPARE,
		.ths_prepare    = DPHY_TIMING_THS_PREPARE,
	};
	int ret = 0;

	if (on) {
		u32 dphytim0, dphytim1;

		/* Set DPHY timing parameters */
		if (priv->hsfreq <= 80) {
			dphy_timing.tclk_settle = 23;
			dphy_timing.ths_settle = 31;
			dphy_timing.ths_prepare = 19;
		} else if (priv->hsfreq <= 125) {
			dphy_timing.tclk_settle = 23;
			dphy_timing.ths_settle = 28;
			dphy_timing.ths_prepare = 19;
		} else if (priv->hsfreq <= 250) {
			dphy_timing.tclk_settle = 23;
			dphy_timing.ths_settle = 22;
			dphy_timing.ths_prepare = 16;
		} else if (priv->hsfreq <= 360) {
			dphy_timing.ths_settle = 19;
		}

		dphytim0 = CSIDPHYTIM0_TCLK_MISS(dphy_timing.tclk_miss) |
			   CSIDPHYTIM0_T_INIT(dphy_timing.t_init);
		dphytim1 = CSIDPHYTIM1_THS_PREPARE(dphy_timing.ths_prepare) |
			   CSIDPHYTIM1_TCLK_PREPARE(dphy_timing.tclk_prepare) |
			   CSIDPHYTIM1_THS_SETTLE(dphy_timing.ths_settle) |
			   CSIDPHYTIM1_TCLK_SETTLE(dphy_timing.tclk_settle);

		rzg2l_csi2_write(priv, CSIDPHYTIM0, dphytim0);
		rzg2l_csi2_write(priv, CSIDPHYTIM1, dphytim1);

		/* Set D-PHY Skew Adjustment with recommended value */
		rzg2l_csi2_write(priv, CSIDPHYSKW0, 0x00001111);

		/* Set the EN_BGR bit */
		rzg2l_csi2_set(priv, CSIDPHYCTRL0, CSIDPHYCTRL0_EN_BGR);

		/* Delay 20us to be stable */
		udelay(20);

		/* Set the EN_LDO1200 bit */
		rzg2l_csi2_set(priv, CSIDPHYCTRL0, CSIDPHYCTRL0_EN_LDO1200);

		/* Delay 10us to be stable */
		udelay(10);

		/* Turn on the DPHY Clock */
		ret = clk_prepare_enable(priv->sysclk);
		if (ret)
			return ret;
	} else {
		/* Reset DPHY */
		reset_control_assert(priv->cmn_rstb);

		/* Stop the DPHY clock */
		clk_disable_unprepare(priv->sysclk);

		/* Cancel the EN_LDO1200 register setting */
		rzg2l_csi2_clr(priv, CSIDPHYCTRL0, CSIDPHYCTRL0_EN_LDO1200);

		/* Cancel the EN_BGR register setting */
		rzg2l_csi2_clr(priv, CSIDPHYCTRL0, CSIDPHYCTRL0_EN_BGR);
	}

	return ret;
}

static int rzg2l_csi2_calc_mbps(struct rzg2l_csi2 *priv, unsigned int bpp)
{
	struct v4l2_subdev *source;
	struct v4l2_ctrl *ctrl;
	u64 mbps;

	if (!priv->remote)
		return -ENODEV;

	source = priv->remote;

	/* Read the pixel rate control from remote. */
	ctrl = v4l2_ctrl_find(source->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl) {
		dev_err(priv->dev, "no pixel rate control in subdev %s\n",
			source->name);
		return -EINVAL;
	}

	/*
	 * Calculate hsfreq in Mbps
	 * hsfreq = (pixel_rate * bits_per_sample) / number_of_lanes
	 */
	mbps = v4l2_ctrl_g_ctrl_int64(ctrl) * bpp;
	do_div(mbps, priv->lanes * 1000000);

	return mbps;
}

int rzg2l_cru_init_csi_dphy(struct v4l2_subdev *sd)
{
	struct rzg2l_csi2 *priv = sd_to_csi2(sd);
	const struct rzg2l_csi2_format *format;
	int ret, mbps;

	/* Code is validated in set_fmt. */
	format = rzg2l_csi2_code_to_fmt(priv->mf.code);

	mbps = rzg2l_csi2_calc_mbps(priv, format->bpp);
	if (mbps < 0)
		return mbps;

        priv->hsfreq = mbps;

	mutex_lock(&priv->lock);

	ret = rzg2l_csi2_dphy_setting(priv, 1);

	mutex_unlock(&priv->lock);

	return ret;
}

static int rzg2l_csi2_start(struct rzg2l_csi2 *priv)
{
	int lanes;
	u32 frrskw, frrclk, frrskw_coeff, frrclk_coeff;
	int ret, count;

	dev_dbg(priv->dev, "Input size (%ux%u%c)\n",
		priv->mf.width, priv->mf.height,
		priv->mf.field == V4L2_FIELD_NONE ? 'p' : 'i');

	/* Initialize LINK */
	/* Checking the maximum lanes support for CSI2 module */
	lanes = (rzg2l_csi2_read(priv, CSI2nMCG) & CSI2nMCG_SDLN) >> 8;
	if (lanes < priv->lanes) {
		dev_err(priv->dev,
			"Failed to support %d data lanes\n", priv->lanes);
		return -EINVAL;
	}
	rzg2l_csi2_write(priv, CSI2nMCT0, CSI2nMCT0_VDLN(priv->lanes));

	/* Set some parameters in accordance with the sensor transfer rate.
	 * Currently vclk is fixed with 266MHz.
	 */
	frrskw_coeff = 3 * 266 * 8;
	frrclk_coeff = 1.5 * 266 * 8;
	frrskw = DIV_ROUND_UP(frrskw_coeff, priv->hsfreq);
	frrclk = DIV_ROUND_UP(frrclk_coeff, priv->hsfreq);
	rzg2l_csi2_write(priv, CSI2nMCT2, CSI2nMCT2_FRRSKW(frrskw) |
					  CSI2nMCT2_FRRCLK(frrclk));

	/* Select a data type.
	 * Currently, we use the normal setting values to support format:
	 * FS, FE, LS, LE, Generic Short Packet Codes 1 to 8,
	 * Embedded 8-bit non-image data, Generic Long Packet Data Types 1 to 4,
	 * YUV422 8-bit,RGB565, RGB888, RAW8 to RAW20,
	 * User-defined 8-bit data types 1 to 8
	 */
	rzg2l_csi2_write(priv, CSI2nDTEL, 0xf77cff0f);
	rzg2l_csi2_write(priv, CSI2nDTEH, 0x00ffff1f);

	clk_disable_unprepare(priv->vclk);
	for (count = 0; count < 5; count++) {
		if (!(__clk_is_enabled(priv->vclk)))
			break;
		udelay(10);
	}

	if (count == 5)
		return -ETIMEDOUT;

	/* Enable LINK reception */
	rzg2l_csi2_set(priv, CSI2nMCT3, CSI2nMCT3_RXEN);

	ret = clk_prepare_enable(priv->vclk);
	if (ret)
		return ret;

	for (count = 0; count < 5; count++) {
		if (__clk_is_enabled(priv->vclk))
			break;
		udelay(10);
	}

	if (count == 5)
		return -ETIMEDOUT;

	/* Release reset state for DPHY */
	reset_control_deassert(priv->cmn_rstb);

	/* Waiting for releasing reset */
	mdelay(1);

	return 0;
}

static void rzg2l_csi2_stop(struct rzg2l_csi2 *priv)
{
	/* Stop DPHY reception */
	rzg2l_csi2_dphy_setting(priv, 0);

	/* Stop LINK reception */
	rzg2l_csi2_clr(priv, CSI2nMCT3, CSI2nMCT3_RXEN);

	/* Request a software reset of the LINK Video Pixel Interface */
	rzg2l_csi2_write(priv, CSI2nRTCT, CSI2nRTCT_VSRST);

	while (1) {
		/* Make sure that the execution status is not during a reset */
		if (!(rzg2l_csi2_read(priv, CSI2nRTST) & CSI2nRTST_VSRSTS))
			break;
	};
}

static int rzg2l_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rzg2l_csi2 *priv = sd_to_csi2(sd);
	struct v4l2_subdev *nextsd;
	int ret = 0;

	mutex_lock(&priv->lock);

	if (!priv->remote) {
		ret = -ENODEV;
		goto out;
	}

	nextsd = priv->remote;

	if (enable && priv->stream_count == 0) {
		ret = rzg2l_csi2_start(priv);
		if (ret)
			goto out;

		ret = v4l2_subdev_call(nextsd, video, s_stream, 1);
		if (ret) {
			rzg2l_csi2_stop(priv);
			goto out;
		}
	} else if (!enable && priv->stream_count == 1) {
		v4l2_subdev_call(nextsd, video, s_stream, 0);
		rzg2l_csi2_stop(priv);
	}

	priv->stream_count += enable ? 1 : -1;
out:
	mutex_unlock(&priv->lock);

	return ret;
}

static int rzg2l_csi2_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *format)
{
	struct rzg2l_csi2 *priv = sd_to_csi2(sd);
	struct v4l2_mbus_framefmt *framefmt;

	if (!rzg2l_csi2_code_to_fmt(format->format.code))
		format->format.code = rzg2l_csi2_formats[0].code;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		priv->mf = format->format;
	} else {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, 0);
		*framefmt = format->format;
	}

	return 0;
}

static int rzg2l_csi2_get_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *format)
{
	struct rzg2l_csi2 *priv = sd_to_csi2(sd);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		format->format = priv->mf;
	else
		format->format = *v4l2_subdev_get_try_format(sd, cfg, 0);

	return 0;
}

static const struct v4l2_subdev_video_ops rzg2l_csi2_video_ops = {
	.s_stream = rzg2l_csi2_s_stream,
};

static const struct v4l2_subdev_pad_ops rzg2l_csi2_pad_ops = {
	.set_fmt = rzg2l_csi2_set_pad_format,
	.get_fmt = rzg2l_csi2_get_pad_format,
};

static const struct v4l2_subdev_ops rzg2l_csi2_subdev_ops = {
	.video	= &rzg2l_csi2_video_ops,
	.pad	= &rzg2l_csi2_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Async handling and registration of subdevices and links.
 */

static int rzg2l_csi2_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
	struct rzg2l_csi2 *priv = notifier_to_csi2(notifier);
	int pad;

	pad = media_entity_get_fwnode_pad(&subdev->entity, asd->match.fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(priv->dev, "Failed to find pad for %s\n", subdev->name);
		return pad;
	}

	priv->remote = subdev;

	dev_dbg(priv->dev, "Bound %s pad: %d\n", subdev->name, pad);

	return media_create_pad_link(&subdev->entity, pad,
				     &priv->subdev.entity, 0,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static void rzg2l_csi2_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct rzg2l_csi2 *priv = notifier_to_csi2(notifier);

	priv->remote = NULL;

	dev_dbg(priv->dev, "Unbind %s\n", subdev->name);
}

static const struct v4l2_async_notifier_operations rzg2l_csi2_notify_ops = {
	.bound = rzg2l_csi2_notify_bound,
	.unbind = rzg2l_csi2_notify_unbind,
};

static int rzg2l_csi2_parse_v4l2(struct rzg2l_csi2 *priv,
			    struct v4l2_fwnode_endpoint *vep)
{
	/* Only port 0 endpoint 0 is valid. */
	if (vep->base.port || vep->base.id)
		return -ENOTCONN;

	if (vep->bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(priv->dev, "Unsupported bus: %u\n", vep->bus_type);
		return -EINVAL;
	}

	priv->lanes = vep->bus.mipi_csi2.num_data_lanes;
	if (priv->lanes != 1 && priv->lanes != 2 && priv->lanes != 4) {
		dev_err(priv->dev, "Unsupported number of data-lanes: %u\n",
			priv->lanes);
		return -EINVAL;
	}

	return 0;
}

static int rzg2l_csi2_parse_dt(struct rzg2l_csi2 *priv)
{
	struct v4l2_async_subdev *asd;
	struct fwnode_handle *fwnode;
	struct device_node *ep;
	struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = 0 };
	int ret;

	ep = of_graph_get_endpoint_by_regs(priv->dev->of_node, 0, 0);
	if (!ep) {
		dev_err(priv->dev, "Not connected to subdevice\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &v4l2_ep);
	if (ret) {
		dev_err(priv->dev, "Could not parse v4l2 endpoint\n");
		of_node_put(ep);
		return -EINVAL;
	}

	ret = rzg2l_csi2_parse_v4l2(priv, &v4l2_ep);
	if (ret) {
		of_node_put(ep);
		return ret;
	}

	fwnode = fwnode_graph_get_remote_endpoint(of_fwnode_handle(ep));
	of_node_put(ep);

	dev_dbg(priv->dev, "Found '%pOF'\n", to_of_node(fwnode));

	v4l2_async_notifier_init(&priv->notifier);
	priv->notifier.ops = &rzg2l_csi2_notify_ops;

	asd = v4l2_async_notifier_add_fwnode_subdev(&priv->notifier, fwnode,
						    sizeof(*asd));

	fwnode_handle_put(fwnode);
	if (IS_ERR(asd))
		return PTR_ERR(asd);

	ret = v4l2_async_subdev_notifier_register(&priv->subdev,
						  &priv->notifier);
	if (ret)
		v4l2_async_notifier_cleanup(&priv->notifier);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Platform Device Driver.
 */

static const struct media_entity_operations rzg2l_csi2_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int rzg2l_csi2_probe(struct platform_device *pdev)
{
	struct rzg2l_csi2 *priv;
	struct resource *res;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	mutex_init(&priv->lock);

	priv->stream_count = 0;
	priv->power_count = 0;

	platform_set_drvdata(pdev, priv);

	priv->vclk = devm_clk_get(priv->dev, "vclk");
	if (IS_ERR(priv->vclk)) {
		dev_err(priv->dev, "failed to get VCLK clock\n");
		return PTR_ERR(priv->vclk);
	}

	priv->sysclk = devm_clk_get(priv->dev, "sysclk");
	if (IS_ERR(priv->sysclk)) {
		dev_err(priv->dev, "failed to get SYSCLK clock\n");
		return PTR_ERR(priv->sysclk);
	}

	priv->cmn_rstb = devm_reset_control_get(&pdev->dev, "cmn_rstb");
	if (IS_ERR(priv->cmn_rstb)) {
		dev_err(&pdev->dev, "failed to get CMN_RSTB reset\n");
		return PTR_ERR(priv->cmn_rstb);
	}

	ret = rzg2l_csi2_parse_dt(priv);
	if (ret)
		return ret;

	priv->subdev.owner = THIS_MODULE;
	priv->subdev.dev = &pdev->dev;
	v4l2_subdev_init(&priv->subdev, &rzg2l_csi2_subdev_ops);
	v4l2_set_subdevdata(&priv->subdev, &pdev->dev);
	snprintf(priv->subdev.name, V4L2_SUBDEV_NAME_SIZE, "%s %s",
		 KBUILD_MODNAME, dev_name(&pdev->dev));
	priv->subdev.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	priv->subdev.entity.ops = &rzg2l_csi2_entity_ops;

	priv->pads[RZG2L_CSI2_SINK].flags = MEDIA_PAD_FL_SINK;
	for (i = RZG2L_CSI2_SOURCE_VC0; i < NR_OF_RZG2L_CSI2_PAD; i++)
		priv->pads[i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->subdev.entity, NR_OF_RZG2L_CSI2_PAD,
				     priv->pads);
	if (ret)
		goto error;

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		goto error;

	dev_info(priv->dev, "%d lanes found\n", priv->lanes);

	return 0;

error:
	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);

	return ret;
}


static const struct of_device_id rzg2l_csi2_of_table[] = {
	{ .compatible = "renesas,r9a07g044-csi2", },
	{ .compatible = "renesas,r9a07g043-csi2", },
	{ /* sentinel */ },
};

static int rzg2l_csi2_remove(struct platform_device *pdev)
{
	struct rzg2l_csi2 *priv = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
	v4l2_async_unregister_subdev(&priv->subdev);

	return 0;
}

static struct platform_driver rzg2l_csi2_pdrv = {
	.remove	= rzg2l_csi2_remove,
	.probe	= rzg2l_csi2_probe,
	.driver	= {
		.name	= "rzg2l-csi2",
		.of_match_table	= rzg2l_csi2_of_table,
	},
};

module_platform_driver(rzg2l_csi2_pdrv);

MODULE_AUTHOR("Hien Huynh <hien.huynh.px@renesas.com>");
MODULE_DESCRIPTION("Renesas RZG2L MIPI CSI2 receiver driver");
MODULE_LICENSE("GPL");
