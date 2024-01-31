/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Renesas RZ/G2L A/D Converter
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 */

#ifndef __RENESAS_RZG2L_ADC_H__
#define __RENESAS_RZG2L_ADC_H__

int rzg2l_adc_read_tsu(struct device *dev, int *val);

#endif /* __ADC_H__ */
