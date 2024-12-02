/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ/G3E LVDS Encoder
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 *
 */

#ifndef __RZG3E_LVDS_H__
#define __RZG3E_LVDS_H__

struct drm_bridge;

#if IS_ENABLED(CONFIG_DRM_RZG3E_LVDS)
bool rzg3e_lvds_dual_link(struct drm_bridge *bridge);
bool rzg3e_lvds_is_connected(struct drm_bridge *bridge);
#else
static inline bool rzg3e_lvds_dual_link(struct drm_bridge *bridge)
{
	return false;
}
static inline bool rzg3e_lvds_is_connected(struct drm_bridge *bridge)
{
	return false;
}
#endif /* CONFIG_DRM_RZG3E_LVDS */

#endif /* __RZG3E_LVDS_H__ */
