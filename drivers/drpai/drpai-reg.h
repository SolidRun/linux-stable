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

#ifndef R_DRPAI_REG_H
#define R_DRPAI_REG_H

/*--------------------------------------------------------------------------------------------------
  DRP Initialization register
--------------------------------------------------------------------------------------------------*/
/* Clock */
#define STPC_CLKGEN_CTRL                (0x81D930)
#define STPC_CLKGEN_RST                 (0x81D840)
#define STPC_CLKGEN_STBYWAIT            (0x81D860)
#define STPC_CLKGEN_DIV                 (0x81D848)
#define STPC_CLKE                       (0x81D810)
/* Soft reset */
#define STPC_SFTRST                     (0x81D800)
/* DMAC */
#define IDIF_DMACTLCW                   (0x819900)
#define IDIF_DMACTLI0                   (0x819100)
#define IDIF_DMACTLI1                   (0x819200)
#define IDIF_DMACTLI2                   (0x819300)
#define IDIF_DMACTLI3                   (0x819400)
#define ODIF_DMACTLO0                   (0x81A100)
#define ODIF_DMACTLO1                   (0x81A200)
#define ODIF_DMACTLO2                   (0x81A300)
#define ODIF_DMACTLO3                   (0x81A400)
#define ODIF_DMACTLCR                   (0x81A900)
/* Normal Interrupt mask */
#define ODIF_INTMSK                     (0x81A004)
/* Err Interrupt mask */
#define IDIF_EINTMSK                    (0x81900C)
#define IDIF_EINTMSK_DSC                (0x819014)
#define ODIF_EINTMSK                    (0x81A00C)
#define IDMAC_INTME                     (0x81B024)
#define ODMAC_INTME                     (0x81C024)
#define RAC_EINTM                       (0x81D008)
/*--------------------------------------------------------------------------------------------------
  DRP boot sequence register
--------------------------------------------------------------------------------------------------*/
#define DSCC_DPA                        (0x818108)
#define DSCC_DCTL                       (0x818100)
/*--------------------------------------------------------------------------------------------------
  DPP NMLINT register
--------------------------------------------------------------------------------------------------*/
#define ODIF_INT                        (0x81A000)
#define ODIF_INTCNTO0                   (0x81A118)
#define ODIF_INTCNTO1                   (0x81A218)
#define ODIF_INTCNTO2                   (0x81A318)
#define ODIF_INTCNTO3                   (0x81A418)
/*--------------------------------------------------------------------------------------------------
  DRP ERRINT register
--------------------------------------------------------------------------------------------------*/
#define STPC_ERRINT_STS                 (0x81D808)
#define DRP_ERRINTSTATUS                (0x3B0048)
#define IDIF_EINT                       (0x819008)
#define IDIF_EINT_DSC                   (0x819010)
#define ODIF_EINT                       (0x81A008)
#define IDMAC_INTSE                     (0x81B020)
#define ODMAC_INTSE                     (0x81C020)
#define RAC_EINTS                       (0x81D004)
/*--------------------------------------------------------------------------------------------------
  AI-MAC Initialization register
--------------------------------------------------------------------------------------------------*/
/* Clock */
#define EXD0_STPC_CLKGEN_CTRL           (0x01D930)
#define EXD0_STPC_CLKGEN_RST            (0x01D840)
#define EXD0_STPC_CLKGEN_STBYWAIT       (0x01D860)
#define EXD0_STPC_CLKGEN_DIV            (0x01D848)
#define EXD0_STPC_CLKE                  (0x01D810)
#define EXD1_STPC_CLKE                  (0x05D810)
#define CLKRSTCON_CLKE                  (0x141808)
/* Soft reset */
#define EXD0_STPC_SFTRST                (0x01D800)
#define EXD1_STPC_SFTRST                (0x05D800)
#define CLKRSTCON_SFTRST                (0x141800)
/* DMAC */
#define EXD0_IDIF_DMACTLI0              (0x019100)
#define EXD0_IDIF_DMACTLI1              (0x019200)
#define EXD0_IDIF_DMACTLI2              (0x019300)
#define EXD0_IDIF_DMACTLI3              (0x019400)
#define EXD0_ODIF_DMACTLO0              (0x01A100)
#define EXD0_ODIF_DMACTLO1              (0x01A200)
#define EXD0_ODIF_DMACTLO2              (0x01A300)
#define EXD0_ODIF_DMACTLO3              (0x01A400)
#define EXD1_IDIF_DMACTLI0              (0x059100)
#define EXD1_IDIF_DMACTLI1              (0x059200)
#define EXD1_IDIF_DMACTLI2              (0x059300)
#define EXD1_IDIF_DMACTLI3              (0x059400)
#define EXD1_ODIF_DMACTLO0              (0x05A100)
#define EXD1_ODIF_DMACTLO1              (0x05A200)
#define EXD1_ODIF_DMACTLO2              (0x05A300)
#define EXD1_ODIF_DMACTLO3              (0x05A400)
/* Nml Interrupt mask */
#define EXD1_ODIF_INTMSK                (0x05A004)
/* Err Interrupt mask */
#define AID_IDIF_EINTMSK                (0x11900C)
#define AID_IDMAC_INTME                 (0x11B024)
#define AIMRAC_EINTM                    (0x142008)
#define CMDSEL_ERRMSK                   (0x140104)
#define PRAM_INTMASK                    (0x200008)
#define OSEL_DO_MSK0                    (0x180020)
#define OSEL_DO_MSK1                    (0x180120)
#define OSEL_DO_MSK2                    (0x180220)
#define OSEL_DO_MSK3                    (0x180320)
#define OSEL_DO_MSK4                    (0x180420)
#define OSEL_DO_MSK5                    (0x180520)
#define OSEL_DO_MSK6                    (0x180620)
#define OSEL_DO_MSK7                    (0x180720)
#define OSEL_DO_MSK8                    (0x180820)
#define OSEL_DO_EN0                     (0x180010)
#define OSEL_DO_EN1                     (0x180110)
#define OSEL_DO_EN2                     (0x180210)
#define OSEL_DO_EN3                     (0x180310)
#define OSEL_DO_EN4                     (0x180410)
#define OSEL_DO_EN5                     (0x180510)
#define OSEL_DO_EN6                     (0x180610)
#define OSEL_DO_EN7                     (0x180710)
#define OSEL_DO_EN8                     (0x180810)
#define MACTOP_ERR_MSK                  (0x1C000C)
#define EXD0_IDIF_EINTMSK               (0x01900C)
#define EXD1_IDIF_EINTMSK               (0x05900C)
#define EXD0_ODIF_EINTMSK               (0x01A00C)
#define EXD1_ODIF_EINTMSK               (0x05A00C)
#define EXD0_IDMAC_INTME                (0x01B024)
#define EXD1_IDMAC_INTME                (0x05B024)
#define EXD0_ODMAC_INTME                (0x01C024)
#define EXD1_ODMAC_INTME                (0x05C024)
#define EXD0_RAC_EINTM                  (0x01D008)
#define EXD1_RAC_EINTM                  (0x05D008)
/*--------------------------------------------------------------------------------------------------
  AI-MAC boot sequence register
--------------------------------------------------------------------------------------------------*/
#define AID_DSCC_DPA                    (0x118108)
#define AID_DSCC_DCTL                   (0x118100)
#define AID_IDIF_DMACTLI0               (0x119100)
/*--------------------------------------------------------------------------------------------------
  AI-MAC NMLINT register
--------------------------------------------------------------------------------------------------*/
#define EXD1_ODIF_INT                   (0x05A000)
/*--------------------------------------------------------------------------------------------------
  AI-MAC ERRINT register
--------------------------------------------------------------------------------------------------*/
#define INTMON_ERRINT                   (0x141008)
#define AID_IDIF_EINT                   (0x119008)
#define AID_IDMAC_INTSE                 (0x11B020)
#define AIMRAC_EINTS                    (0x142004)
#define CMDSEL_ERRSTS                   (0x140100)
#define PRAM_INT                        (0x200000)
#define OSEL_DO_ESTS0                   (0x180000)
#define OSEL_DO_ESTS1                   (0x180100)
#define OSEL_DO_ESTS2                   (0x180200)
#define OSEL_DO_ESTS3                   (0x180300)
#define OSEL_DO_ESTS4                   (0x180400)
#define OSEL_DO_ESTS5                   (0x180500)
#define OSEL_DO_ESTS6                   (0x180600)
#define OSEL_DO_ESTS7                   (0x180700)
#define OSEL_DO_ESTS8                   (0x180800)
#define OSEL_AF_ESTS0                   (0x180004)
#define OSEL_AF_ESTS1                   (0x180104)
#define OSEL_AF_ESTS2                   (0x180204)
#define OSEL_AF_ESTS3                   (0x180304)
#define OSEL_AF_ESTS4                   (0x180404)
#define OSEL_AF_ESTS5                   (0x180504)
#define OSEL_AF_ESTS6                   (0x180604)
#define OSEL_AF_ESTS7                   (0x180704)
#define OSEL_AF_ESTS8                   (0x180804)
#define OSEL_DO_FESTS0                  (0x181000)
#define OSEL_DO_FESTS1                  (0x181100)
#define OSEL_DO_FESTS2                  (0x181200)
#define OSEL_DO_FESTS3                  (0x181300)
#define OSEL_DO_FESTS4                  (0x181400)
#define OSEL_DO_FESTS5                  (0x181500)
#define OSEL_DO_FESTS6                  (0x181600)
#define OSEL_DO_FESTS7                  (0x181700)
#define OSEL_DO_FESTS8                  (0x181800)
#define OSEL_AF_FESTS0                  (0x181004)
#define OSEL_AF_FESTS1                  (0x181104)
#define OSEL_AF_FESTS2                  (0x181204)
#define OSEL_AF_FESTS3                  (0x181304)
#define OSEL_AF_FESTS4                  (0x181404)
#define OSEL_AF_FESTS5                  (0x181504)
#define OSEL_AF_FESTS6                  (0x181604)
#define OSEL_AF_FESTS7                  (0x181704)
#define OSEL_AF_FESTS8                  (0x181804)
#define MACTOP_ERR_STS                  (0x1C0008)
#define MACCTL_FERR_STS                 (0x1C0800)
#define EXD0_IDIF_EINT                  (0x019008)
#define EXD1_IDIF_EINT                  (0x059008)
#define EXD0_ODIF_EINT                  (0x01A008)
#define EXD1_ODIF_EINT                  (0x05A008)
#define EXD0_IDMAC_INTSE                (0x01B020)
#define EXD1_IDMAC_INTSE                (0x05B020)
#define EXD0_ODMAC_INTSE                (0x01C020)
#define EXD1_ODMAC_INTSE                (0x05C020)
#define EXD0_RAC_EINTS                  (0x01D004)
#define EXD1_RAC_EINTS                  (0x05D004)
/*--------------------------------------------------------------------------------------------------
  AI-MAC Reset register
--------------------------------------------------------------------------------------------------*/
#define SYNCTBL_TBL0                    (0x140800)
#define SYNCTBL_TBL1                    (0x140804)
#define SYNCTBL_TBL2                    (0x140808)
#define SYNCTBL_TBL3                    (0x14080C)
#define SYNCTBL_TBL4                    (0x140810)
#define SYNCTBL_TBL5                    (0x140814)
#define SYNCTBL_TBL6                    (0x140818)
#define SYNCTBL_TBL7                    (0x14081C)
#define SYNCTBL_TBL8                    (0x140820)
#define SYNCTBL_TBL9                    (0x140824)
#define SYNCTBL_TBL10                   (0x140828)
#define SYNCTBL_TBL11                   (0x14082C)
#define SYNCTBL_TBL12                   (0x140830)
#define SYNCTBL_TBL13                   (0x140834)
#define SYNCTBL_TBL14                   (0x140838)
#define SYNCTBL_TBL15                   (0x14083C)
/*--------------------------------------------------------------------------------------------------
  DRP Reset register
--------------------------------------------------------------------------------------------------*/
#define DRP_SYNCTBL_TBL0                (0x81E000)
#define DRP_SYNCTBL_TBL1                (0x81E004)
#define DRP_SYNCTBL_TBL2                (0x81E008)
#define DRP_SYNCTBL_TBL3                (0x81E00C)
#define DRP_SYNCTBL_TBL4                (0x81E010)
#define DRP_SYNCTBL_TBL5                (0x81E014)
#define DRP_SYNCTBL_TBL6                (0x81E018)
#define DRP_SYNCTBL_TBL7                (0x81E01C)
#define DRP_SYNCTBL_TBL8                (0x81E020)
#define DRP_SYNCTBL_TBL9                (0x81E024)
#define DRP_SYNCTBL_TBL10               (0x81E028)
#define DRP_SYNCTBL_TBL11               (0x81E02C)
#define DRP_SYNCTBL_TBL12               (0x81E030)
#define DRP_SYNCTBL_TBL13               (0x81E034)
#define DRP_SYNCTBL_TBL14               (0x81E038)
#define DRP_SYNCTBL_TBL15               (0x81E03C)
/*--------------------------------------------------------------------------------------------------
  DRP-AI Internal state register
--------------------------------------------------------------------------------------------------*/
#define DSCC_PAMON                      (0x818118)
#define AID_DSCC_PAMON                  (0x118118)
/*--------------------------------------------------------------------------------------------------
  Bit manipulation
--------------------------------------------------------------------------------------------------*/
#define DRPAI_BIT0                      (0x00000001u)
#define DRPAI_BIT1                      (0x00000002u)
#define DRPAI_BIT2                      (0x00000004u)
#define DRPAI_BIT3                      (0x00000008u)
#define DRPAI_BIT4                      (0x00000010u)
#define DRPAI_BIT5                      (0x00000020u)
#define DRPAI_BIT6                      (0x00000040u)
#define DRPAI_BIT7                      (0x00000080u)
#define DRPAI_BIT8                      (0x00000100u)
#define DRPAI_BIT9                      (0x00000200u)
#define DRPAI_BIT10                     (0x00000400u)
#define DRPAI_BIT11                     (0x00000800u)
#define DRPAI_BIT12                     (0x00001000u)
#define DRPAI_BIT13                     (0x00002000u)
#define DRPAI_BIT14                     (0x00004000u)
#define DRPAI_BIT15                     (0x00008000u)
#define DRPAI_BIT16                     (0x00010000u)
#define DRPAI_BIT17                     (0x00020000u)
#define DRPAI_BIT18                     (0x00040000u)
#define DRPAI_BIT19                     (0x00080000u)
#define DRPAI_BIT20                     (0x00100000u)
#define DRPAI_BIT21                     (0x00200000u)
#define DRPAI_BIT22                     (0x00400000u)
#define DRPAI_BIT23                     (0x00800000u)
#define DRPAI_BIT24                     (0x01000000u)
#define DRPAI_BIT25                     (0x02000000u)
#define DRPAI_BIT26                     (0x04000000u)
#define DRPAI_BIT27                     (0x08000000u)
#define DRPAI_BIT28                     (0x10000000u)
#define DRPAI_BIT29                     (0x20000000u)
#define DRPAI_BIT30                     (0x40000000u)
#define DRPAI_BIT31                     (0x80000000u)

#endif /* R_DRPAI_REG_H */
