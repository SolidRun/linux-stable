/*
 * Renesas RZ/V2M GICD header file
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
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
#ifndef __IRQ_GIC__RZV2M_H__
#define __IRQ_GIC_RZV2M_H__
const u32 rzv2m_gic_init_target[] =
{
    0x01010101,    //offset address : 0x820
    0x01010101,    //offset address : 0x824
    0x01010101,    //offset address : 0x828
    0x01010101,    //offset address : 0x82C
    0x00000101,    //offset address : 0x830
    0x00000000,    //offset address : 0x834
    0x00000000,    //offset address : 0x838
    0x00000000,    //offset address : 0x83C
    0x00000000,    //offset address : 0x840
    0x01000000,    //offset address : 0x844
    0x01010101,    //offset address : 0x848
    0x02010002,    //offset address : 0x84C
    0x01010201,    //offset address : 0x850
    0x01010101,    //offset address : 0x854
    0x02020101,    //offset address : 0x858
    0x02020202,    //offset address : 0x85C
    0x02020202,    //offset address : 0x860
    0x02020202,    //offset address : 0x864
    0x01010101,    //offset address : 0x868
    0x02020202,    //offset address : 0x86C
    0x02020202,    //offset address : 0x870
    0x01010101,    //offset address : 0x874
    0x02010101,    //offset address : 0x878
    0x01010101,    //offset address : 0x87C
    0x01010101,    //offset address : 0x880
    0x01010101,    //offset address : 0x884
    0x02010101,    //offset address : 0x888
    0x02020202,    //offset address : 0x88C
    0x01020202,    //offset address : 0x890
    0x01010101,    //offset address : 0x894
    0x01010101,    //offset address : 0x898
    0x01010101,    //offset address : 0x89C
    0x02010101,    //offset address : 0x8A0
    0x02020202,    //offset address : 0x8A4
    0x02020202,    //offset address : 0x8A8
    0x02020202,    //offset address : 0x8AC
    0x02020202,    //offset address : 0x8B0
    0x02020202,    //offset address : 0x8B4
    0x02020202,    //offset address : 0x8B8
    0x02020202,    //offset address : 0x8BC
    0x02020202,    //offset address : 0x8C0
    0x02020000,    //offset address : 0x8C4
    0x02020202,    //offset address : 0x8C8
    0x02020202,    //offset address : 0x8CC
    0x02020202,    //offset address : 0x8D0
    0x02020202,    //offset address : 0x8D4
    0x01010202,    //offset address : 0x8D8
    0x02020202,    //offset address : 0x8DC
    0x02020202,    //offset address : 0x8E0
    0x02020202,    //offset address : 0x8E4
    0x02020202,    //offset address : 0x8E8
    0x02020202,    //offset address : 0x8EC
    0x02020202,    //offset address : 0x8F0
    0x02020202,    //offset address : 0x8F4
    0x02020202,    //offset address : 0x8F8
    0x02020202,    //offset address : 0x8FC
    0x01010202,    //offset address : 0x900
    0x02010201,    //offset address : 0x904
    0x02010201,    //offset address : 0x908
    0x02010201,    //offset address : 0x90C
    0x01010201,    //offset address : 0x910
    0x00010101,    //offset address : 0x914
    0x01000000,    //offset address : 0x918
    0x01010101,    //offset address : 0x91C
    0x01010101,    //offset address : 0x920
    0x01010101,    //offset address : 0x924
    0x01010101,    //offset address : 0x928
    0x01010101,    //offset address : 0x92C
    0x01010101,    //offset address : 0x930
    0x01010101,    //offset address : 0x934
    0x00000000,    //offset address : 0x938
    0x00000000,    //offset address : 0x93C
    0x00000000,    //offset address : 0x940
    0x00000000,    //offset address : 0x944
    0x00000000,    //offset address : 0x948
    0x00000000,    //offset address : 0x94C
    0x00000000,    //offset address : 0x950
    0x01010100,    //offset address : 0x954
    0x01010101,    //offset address : 0x958
    0x01010101,    //offset address : 0x95C
    0x01010101,    //offset address : 0x960
    0x01010101,    //offset address : 0x964
    0x02020101,    //offset address : 0x968
    0x02020002,    //offset address : 0x96C
    0x02020202,    //offset address : 0x970
    0x02020202,    //offset address : 0x974
    0x01010102,    //offset address : 0x978
    0x02020201,    //offset address : 0x97C
    0x01010102,    //offset address : 0x980
    0x01010101,    //offset address : 0x984
    0x01010101,    //offset address : 0x988
    0x01010101,    //offset address : 0x98C
    0x02020202,    //offset address : 0x990
    0x02020202,    //offset address : 0x994
    0x01010101,    //offset address : 0x998
    0x01010101,    //offset address : 0x99C
    0x01010202,    //offset address : 0x9A0
    0x02020201,    //offset address : 0x9A4
    0x00010101,    //offset address : 0x9A8
};
#endif /* __IRQ_GIC_RZV2M_H__ */
