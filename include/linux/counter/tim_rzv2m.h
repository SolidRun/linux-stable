/*
 * Driver for the Renesas RZ/V2M Timer unit(TIM)
 *
 * Copyright (C) 2020 Renesas Electronics Corporation
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

#ifndef TIM_RZV2M_H_
#define TIM_RZV2M_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#include <linux/ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tim_ioctl_if {
    int32_t     mode;
    uint8_t     clk_div;
    uint32_t    cnt_val;
    uint64_t    tm_usec;
};

#define IOCTL_TIM_IO_TYPE   (55)
#define IOCTL_TIM_START     _IOW (IOCTL_TIM_IO_TYPE, 1,struct tim_ioctl_if)
#define IOCTL_TIM_STOP      _IO         (IOCTL_TIM_IO_TYPE, 2)
#define IOCTL_GET_VALUES    _IOR (IOCTL_TIM_IO_TYPE, 3,struct tim_ioctl_if)

enum rzv2m_start_mode
{
    IOCTL_START_FREERUN,
    IOCTL_START_INTERRUPT
};

#ifdef __cplusplus
}
#endif

#endif /* TIM_RZV2M_H_ */
