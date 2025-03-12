// SPDX-License-Identifier: GPL-2.0+
/*
 * RZ/G2L Display Unit Encoder
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 *
 * Based on rcar_du_encoder.c
 */

#include <linux/export.h>
#include <linux/of.h>

#include <drm/drm_bridge.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_crtc.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_panel.h>

#include "rzg2l_du_drv.h"
#include "rzg2l_du_encoder.h"
#include "rzg2l_du_crtc.h"
#include "rzg3e_lvds.h"

/* -----------------------------------------------------------------------------
 * Encoder
 */

static enum drm_mode_status
rzg2l_du_encoder_mode_valid(struct drm_encoder *crtc,
			 const struct drm_display_mode *mode)
{
	struct rzg2l_du_encoder *renc = to_rzg2l_encoder(crtc);
	struct rzg2l_du_device *rcdu = renc->rcdu;

	bool interlaced = mode->flags & DRM_MODE_FLAG_INTERLACE;

	/* RZ/G2L DU does not support interlace mode */
	if (interlaced)
		return MODE_NO_INTERLACE;

	/* Check dotclock for Parallel Output IF if possible */
	if (renc->output == RZG2L_DU_OUTPUT_DPAD0) {
		if (rcdu->info->max_dclk && mode->clock > rcdu->info->max_dclk)
			return MODE_CLOCK_HIGH;

		if (rcdu->info->min_dclk && mode->clock < rcdu->info->min_dclk)
			return MODE_CLOCK_LOW;
	}

	return MODE_OK;
}

static const struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.mode_valid = rzg2l_du_encoder_mode_valid,
};

static const struct drm_encoder_funcs rzg2l_du_encoder_funcs = {
};

int rzg2l_du_encoder_init(struct rzg2l_du_device  *rcdu,
			  enum rzg2l_du_output output,
			  struct device_node *enc_node)
{
	struct rzg2l_du_encoder *renc;
	struct drm_connector *connector;
	struct drm_bridge *bridge;
	int ret;

	/* Locate the DRM bridge from the DT node. */
	bridge = of_drm_find_bridge(enc_node);
	if (!bridge)
		return -EPROBE_DEFER;

	if (output == RZG2L_DU_OUTPUT_LVDS0 ||
	    output == RZG2L_DU_OUTPUT_LVDS1)
		rcdu->lvds[output - RZG2L_DU_OUTPUT_LVDS0] = bridge;

	/*
	 * Create and initialize the encoder. Skip the LVDS1 output if
	 * the LVDS1 encoder is used as a companion for LVDS0 in dual-link
	 * mode.
	 */
	if ((output == RZG2L_DU_OUTPUT_LVDS1) && rzg3e_lvds_dual_link(bridge))
		return -ENOLINK;

	if (((output == RZG2L_DU_OUTPUT_LVDS0) ||
	    (output == RZG2L_DU_OUTPUT_LVDS1)) &&
	    (!rzg3e_lvds_is_connected(bridge)))
		return -ENOLINK;

	dev_dbg(rcdu->dev, "initializing encoder %pOF for output %s\n",
		enc_node, rzg2l_du_output_name(output));

	renc = drmm_encoder_alloc(&rcdu->ddev, struct rzg2l_du_encoder, base,
				  &rzg2l_du_encoder_funcs, DRM_MODE_ENCODER_NONE,
				  NULL);
	if (IS_ERR(renc))
		return PTR_ERR(renc);

	renc->output = output;
	renc->rcdu = rcdu;

	drm_encoder_helper_add(&renc->base, &encoder_helper_funcs);

	/* Attach the bridge to the encoder. */
	ret = drm_bridge_attach(&renc->base, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		dev_err(rcdu->dev,
			"failed to attach bridge %pOF for output %s (%d)\n",
			bridge->of_node, rzg2l_du_output_name(output), ret);
		return ret;
	}

	/* Create the connector for the chain of bridges. */
	connector = drm_bridge_connector_init(&rcdu->ddev, &renc->base);
	if (IS_ERR(connector)) {
		dev_err(rcdu->dev,
			"failed to created connector for output %s (%ld)\n",
			rzg2l_du_output_name(output), PTR_ERR(connector));
		return PTR_ERR(connector);
	}

	return drm_connector_attach_encoder(connector, &renc->base);
}
