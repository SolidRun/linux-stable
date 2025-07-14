// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G3S System controller driver
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 */

#include <linux/bits.h>
#include <linux/init.h>

#include "rz-sysc.h"

static const struct rz_sysc_soc_id_init_data rzg3s_sysc_soc_id_init_data __initconst = {
	.family = "RZ/G3S",
	.id = 0x85e0447,
	.devid_offset = 0xa04,
	.revision_mask = GENMASK(31, 28),
	.specific_id_mask = GENMASK(27, 0),
};

static const struct regmap_config rzg3s_sysc_regmap __initconst = {
	.name = "rzg3s_sysc_regs",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
	.max_register = 0xe20,
};

const struct rz_sysc_init_data rzg3s_sysc_init_data __initconst = {
	.soc_id_init_data = &rzg3s_sysc_soc_id_init_data,
	.regmap_cfg = &rzg3s_sysc_regmap,
};
