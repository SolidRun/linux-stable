/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * Device Tree constants for MaxLinear MXL8611x PHYs
 *
 * Copyright 2023 Variscite Ltd.
 * Copyright 2023 MaxLinear Inc.
 */

#ifndef _DT_BINDINGS_MXL_8611X_H
#define _DT_BINDINGS_MXL_8611X_H

#define MXL8611X_LEDX_CFG_TRAFFIC_ACT_BLINK_IND         (1 << 13)
#define MXL8611X_LEDX_CFG_LINK_UP_FULL_DUPLEX_ON        (1 << 12)
#define MXL8611X_LEDX_CFG_LINK_UP_HALF_DUPLEX_ON        (1 << 11)
#define MXL8611X_LEDX_CFG_LINK_UP_TX_ACT_ON             (1 << 10)
#define MXL8611X_LEDX_CFG_LINK_UP_RX_ACT_ON             (1 << 9)
#define MXL8611X_LEDX_CFG_LINK_UP_TX_ON                 (1 << 8)
#define MXL8611X_LEDX_CFG_LINK_UP_RX_ON                 (1 << 7)
#define MXL8611X_LEDX_CFG_LINK_UP_1GB_ON                (1 << 6)
#define MXL8611X_LEDX_CFG_LINK_UP_100MB_ON              (1 << 5)
#define MXL8611X_LEDX_CFG_LINK_UP_10MB_ON               (1 << 4)
#define MXL8611X_LEDX_CFG_LINK_UP_COLLISION             (1 << 3)
#define MXL8611X_LEDX_CFG_LINK_UP_1GB_BLINK             (1 << 2)
#define MXL8611X_LEDX_CFG_LINK_UP_100MB_BLINK           (1 << 1)
#define MXL8611X_LEDX_CFG_LINK_UP_10MB_BLINK            (1 << 0)

#endif
