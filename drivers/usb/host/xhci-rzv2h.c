// SPDX-License-Identifier: GPL-2.0
/*
 * xHCI host controller driver for R-Car SoCs
 *
 * Copyright (C) 2014 Renesas Electronics Corporation
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/usb/phy.h>
#include <linux/reset.h>
#include <linux/clk.h>

#include "xhci.h"
#include "xhci-plat.h"
#include "xhci-rzv2h.h"

/* Interrupt Enable */
#define USB3_HOST_INTEN				0x1044

#define USB3_HOST_INTEN_XHC			BIT(0)
#define USB3_HOST_INTEN_HSE			BIT(2)
#define USB3_HOST_INTEN_ENA			(USB3_HOST_INTEN_XHC \
						| USB3_HOST_INTEN_HSE)

/* PIPE Status and Control Register */
#define USB3_HOST_U3P0PIPESC0			0x10C0
#define USB3_HOST_U3P0PIPESC1			0x10C4
#define USB3_HOST_U3P0PIPESC2			0x10C8
#define USB3_HOST_U3P0PIPESC3			0x10CC
#define USB3_HOST_U3P0PIPESC4			0x10D0

void xhci_rzv2h_start(struct usb_hcd *hcd)
{
	u32 int_en;

	if (hcd->regs) {
		/* Update the controller initial setting */
		writel(0x03130200, hcd->regs + USB3_HOST_U3P0PIPESC0);
		writel(0x00160200, hcd->regs + USB3_HOST_U3P0PIPESC1);
		writel(0x03150000, hcd->regs + USB3_HOST_U3P0PIPESC2);
		writel(0x03130200, hcd->regs + USB3_HOST_U3P0PIPESC3);
		writel(0x00180000, hcd->regs + USB3_HOST_U3P0PIPESC4);

		/* Interrupt Enable */
		int_en = readl(hcd->regs + USB3_HOST_INTEN);
		int_en |= USB3_HOST_INTEN_ENA;
		writel(int_en, hcd->regs + USB3_HOST_INTEN);
	}
}

int xhci_rzv2h_resume(struct usb_hcd *hcd)
{
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int ret = 0;

	ret = reset_control_deassert(xhci->reset);

	if (ret) {
		if (xhci->quirks & XHCI_SUSPEND_RESUME_CLKS) {
			clk_disable_unprepare(xhci->reg_clk);
			clk_disable_unprepare(xhci->clk);
		}

		return ret;
	}

	return ret;
}

int xhci_rzv2h_suspend(struct usb_hcd *hcd)
{
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	reset_control_assert(xhci->reset);

	return 0;
}
