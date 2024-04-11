/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * This header provides constants for Renesas RZ/G2L family IRQC bindings.
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 */

#ifndef __DT_BINDINGS_IRQC_RZG2L_H
#define __DT_BINDINGS_IRQC_RZG2L_H

/* NMI maps to SPI0 */
#define RZG2L_NMI	0

/*
 * - RZ/G2L Series: IRQ0-7 map to SPI1-8
 * - RZ/[V2H, G3E]: IRQ0-15 map to SPI1-16
 */

#define RZG2L_IRQ0	1
#define RZG2L_IRQ1	2
#define RZG2L_IRQ2	3
#define RZG2L_IRQ3	4
#define RZG2L_IRQ4	5
#define RZG2L_IRQ5	6
#define RZG2L_IRQ6	7
#define RZG2L_IRQ7	8
#define RZG2L_IRQ8	9
#define RZG2L_IRQ9	10
#define RZG2L_IRQ10	11
#define RZG2L_IRQ11	12
#define RZG2L_IRQ12	13
#define RZG2L_IRQ13	14
#define RZG2L_IRQ14	15
#define RZG2L_IRQ15	16

#endif /* __DT_BINDINGS_IRQC_RZG2L_H */
