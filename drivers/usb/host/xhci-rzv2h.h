/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Renesas Electronics Corporation
 */

#ifndef _XHCI_RZV2H_H
#define _XHCI_RZV2H_H

#if IS_ENABLED(CONFIG_USB_XHCI_RZV2H)
void xhci_rzv2h_start(struct usb_hcd *hcd);
int xhci_rzv2h_resume(struct usb_hcd *hcd);
#else
static inline void xhci_rzv2h_start(struct usb_hcd *hcd)
{
}

static inline int xhci_rzv2h_resume(struct usb_hcd *hcd)
{
	return 0;
}
#endif
#endif /* _XHCI_RZV2H_H */
