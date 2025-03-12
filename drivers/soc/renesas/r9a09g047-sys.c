// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G3E System controller (SYS) driver
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>

#include "rz-sysc.h"

/* Register Offsets */
#define SYS_LSI_MODE		0x300
/*
 * BOOTPLLCA[1:0]
 *	    [0,0] => 1.1GHZ
 *	    [0,1] => 1.5GHZ
 *	    [1,0] => 1.6GHZ
 *	    [1,1] => 1.7GHZ
 */
#define SYS_LSI_MODE_STAT_BOOTPLLCA55	GENMASK(12, 11)
#define SYS_LSI_MODE_CA55_1_7GHZ	0x3

#define SYS_LSI_PRR			0x308
#define SYS_LSI_PRR_CA55_DIS		BIT(8)
#define SYS_LSI_PRR_NPU_DIS		BIT(1)
#define SYS_ADC_CFG_PWE_B		0x1600
#define SYS_ADC_MSTP_ADA_B		BIT(0)

static const struct rz_sysc_signal_init_data rzg3e_sysc_signals_init_data[] __initconst = {
	{
		.name = "ADC_MSTP_ADA_B",
		.offset = SYS_ADC_CFG_PWE_B,
		.mask = SYS_ADC_MSTP_ADA_B,
		.refcnt_incr_val = 0
	}
};

static const struct rz_sysc_soc_id_init_data rzg3e_sys_soc_id_init_data __initconst = {
	.family = "RZ/G3E",
	.id = 0x8679447,
	.offset = 0x304,
	.revision_mask = 0xf0000000,
	.specific_id_mask = 0x0fffffff,
};

const struct rz_sysc_init_data rzg3e_sys_init_data = {
	.soc_id_init_data = &rzg3e_sys_soc_id_init_data,
};
