/*
 * Copyright 2004-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*
 * @file linux/mxcfb.h
 *
 * @brief Global header file for the MXC Frame buffer
 *
 * @ingroup Framebuffer
 */
#ifndef __LINUX_MXCFB_H__
#define __LINUX_MXCFB_H__

#include <uapi/linux/mxcfb.h>

#define FB_SYNC_OE_LOW_ACT	0x80000000
#define FB_SYNC_CLK_LAT_FALL	0x40000000
#define FB_SYNC_DATA_INVERT	0x20000000
#define FB_SYNC_CLK_IDLE_EN	0x10000000
#define FB_SYNC_SHARP_MODE	0x08000000
#define FB_SYNC_SWAP_RGB	0x04000000

#define FB_MXC_SYNC_MASK	(FB_SYNC_OE_LOW_ACT | FB_SYNC_CLK_LAT_FALL | \
				 FB_SYNC_DATA_INVERT | FB_SYNC_CLK_IDLE_EN | \
				 FB_SYNC_SHARP_MODE | FB_SYNC_SWAP_RGB)

extern struct fb_videomode mxcfb_modedb[];
extern int mxcfb_modedb_sz;

enum {
	MXC_DISP_SPEC_DEV = 0,
	MXC_DISP_DDC_DEV = 1,
};

enum {
	MXCFB_REFRESH_OFF,
	MXCFB_REFRESH_AUTO,
	MXCFB_REFRESH_PARTIAL,
};

int mxcfb_set_refresh_mode(struct fb_info *fbi, int mode,
			   struct mxcfb_rect *update_region);
int mxc_elcdif_frame_addr_setup(dma_addr_t phys);
void mxcfb_elcdif_register_mode(const struct fb_videomode *modedb,
		int num_modes, int dev_mode);

#endif
