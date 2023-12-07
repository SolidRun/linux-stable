/*
 * Driver for the Renesas RZ/V2M RZ/V2MA RZ/V2L DRP-AI unit
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef R_DRPAI_CORE_H
#define R_DRPAI_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <asm/current.h>
#else /* __KERNEL__ */
#include "r_typedefs.h"
#endif /* __KERNEL__ */

#define CH0                             (0)
#define CH1                             (1)
#define CH2                             (2)
#define CH3                             (3)
#define DRP_CH_NUM                      (1)
#define AIMAC_CH_NUM                    (1)
#define MAC256_CH_NUM                   (4)

/* Error code */
#define R_DRPAI_SUCCESS                 (0)
#define R_DRPAI_ERR_INVALID_ARG         (-1)
#define R_DRPAI_ERR_RESET               (-2)

#define DEVICE_RZV2M                    (0)
#define DEVICE_RZV2MA                   (1)
#define DEVICE_RZV2L                    (2)

#define CPG_RESET_SUCCESS               (0)

#define RST_CPG_WAIT (10)
#define RST_MAX_TIMEOUT (100)

/* Debug macro */
// #define   DRPAI_DRV_DEBUG
#ifdef  DRPAI_DRV_DEBUG
#define DRPAI_DEBUG_PRINT(fmt, ...) \
            pr_info("[%s: %d](pid: %d) "fmt, \
                            __func__, __LINE__, current->pid, ##__VA_ARGS__)
#else
#define DRPAI_DEBUG_PRINT(...)
#endif

typedef struct drpai_odif_intcnto
{
    uint32_t    ch0;
    uint32_t    ch1;
    uint32_t    ch2;
    uint32_t    ch3;
} drpai_odif_intcnto_t;

int32_t R_DRPAI_DRP_Open(void __iomem *drp_base_addr, int32_t ch, spinlock_t *lock);
int32_t R_DRPAI_DRP_Start(void __iomem *drp_base_addr, int32_t ch, uint32_t desc);
int32_t R_DRPAI_DRP_Nmlint(void __iomem *drp_base_addr, int32_t ch, drpai_odif_intcnto_t *odif_intcnto);
int32_t R_DRPAI_DRP_Errint(void __iomem *drp_base_addr, void __iomem *aimac_base_addr, int32_t ch);
int32_t R_DRPAI_AIMAC_Open(void __iomem *aimac_base_addr, int32_t ch);
int32_t R_DRPAI_AIMAC_Start(void __iomem *aimac_base_addr, int32_t ch, uint32_t desc, spinlock_t *lock);
int32_t R_DRPAI_AIMAC_Nmlint(void __iomem *aimac_base_addr, int32_t ch);
int32_t R_DRPAI_AIMAC_Errint(void __iomem *drp_base_addr, void __iomem *aimac_base_addr, int32_t ch);
int32_t R_DRPAI_Status(void __iomem *drp_base_addr, void __iomem *aimac_base_addr, int32_t ch, drpai_status_t *drpai_status);
int32_t R_DRPAI_DRP_Reset(void __iomem *drp_base_addr, void __iomem *aimac_base_addr, int32_t ch, spinlock_t *lock);
int32_t R_DRPAI_AIMAC_Reset(void __iomem *aimac_base_addr, int32_t ch);
int32_t R_DRPAI_CPG_Reset(struct reset_control *rst_ctrl);
int32_t R_DRPB_DRP_Open(void __iomem *drp_base_addr, int32_t ch, spinlock_t *lock);
int32_t R_DRPB_Status(void __iomem *drp_base_addr, int32_t ch, drpai_status_t *drpai_status);
int32_t R_DRPB_DRP_Reset(void __iomem *drp_base_addr, int32_t ch, spinlock_t *lock);
int32_t R_DRPB_CPG_Reset(struct reset_control *rst_ctrl);
int32_t R_DRPB_DRP_Nmlint(void __iomem *drp_base_addr, int32_t ch, drpai_odif_intcnto_t *odif_intcnto);
int32_t R_DRPB_DRP_Errint(void __iomem *drp_base_addr, int32_t ch);

#endif /* R_DRPAI_CORE_H */
