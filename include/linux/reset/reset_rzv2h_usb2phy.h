/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _RESET_RZV2H_USB2PHY_H
#define _RESET_RZV2H_USB2PHY_H

#include <linux/auxiliary_bus.h>

struct reset_rzv2h_usb2phy_adev {
	void __iomem *base;
};

#endif
