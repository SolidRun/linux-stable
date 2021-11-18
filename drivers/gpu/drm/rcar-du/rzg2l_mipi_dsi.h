/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ/G2L MIPI DSI Encoder Header
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 */

#ifndef __RZG2L_MIPI_DSI_H__
#define __RZG2L_MIPI_DSI_H__

struct drm_bridge;

#if IS_ENABLED(CONFIG_DRM_RZG2L_MIPI_DSI)
int rzg2l_mipi_dsi_clk_enable(struct drm_bridge *bridge);
void rzg2l_mipi_dsi_clk_disable(struct drm_bridge *bridge);
#else
static inline int rzg2l_mipi_dsi_clk_enable(struct drm_bridge *bridge)
{
	return -EOPNOTSUPP;
}
static inline void rzg2l_mipi_dsi_clk_disable(struct drm_bridge *bridge)
{
}
#endif /* CONFIG_DRM_RZG2L_MIPI_DSI */
#endif /* __RZG2L_MIPI_DSI_H__ */
