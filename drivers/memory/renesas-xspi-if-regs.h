/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ xSPI Interface Registers Definitions
 *
 * Copyright (C) 2025 Renesas Electronics Corporation
 */

#ifndef __RENESAS_XSPI_IF_REGS_H__
#define __RENESAS_XSPI_IF_REGS_H__

#include <linux/bits.h>

/* xSPI Wrapper Configuration Register */
#define XSPI_WRAPCFG		0x0000
#define XSPI_WRAPCFG_CKSFTCS0(val)	(((val) & 0x1f) << 0)
#define XSPI_WRAPCFG_DSSFTCS0(val)	(((val) & 0x1f) << 8)
#define XSPI_WRAPCFG_CKSFTCS1(val)	(((val) & 0x1f) << 16)
#define XSPI_WRAPCFG_DSSFTCS1(val)	(((val) & 0x1f) << 24)

/* xSPI Common Configuration Register */
#define XSPI_COMCFG		0x0004
#define XSPI_COMCFG_OEASTEX	BIT(16)
#define XSPI_COMCFG_OENEGEX	BIT(17)

/* xSPI Bridge Configuration Register */
#define XSPI_BMCFG		0x0008
#define XSPI_BMCFG_WRMD		BIT(0)
#define XSPI_BMCFG_MWRCOMB	BIT(7)
#define XSPI_BMCFG_MWRSIZE(val)	(((val) & 0xff) << 8)
#define XSPI_BMCFG_PREEN	BIT(16)
#define XSPI_BMCFG_CMBTIM(val)	(((val) & 0xff) << 24)

/* xSPI Command Map Configuration Register 0 CS(0/1) */
#define XSPI_CMCFG0CS0		0x0010
#define XSPI_CMCFG0CS1		0x0020
#define XSPI_CMCFG0_FFMT(val)		(((val) & 0x03) << 0)
#define XSPI_CMCFG0_ADDSIZE(val)	(((val) & 0x03) << 2)
#define XSPI_CMCFG0_WPBSTMD	BIT(4)
#define XSPI_CMCFG0_ARYAMD	BIT(5)
#define XSPI_CMCFG0_ADDRPEN(val)	(((val) & 0xff) << 16)
#define XSPI_CMCFG0_ADDRPCD(val)	(((val) & 0xff) << 24)

/* xSPI Command Map Configuration Register 1 CS(0/1) */
#define XSPI_CMCFG1CS0		0x0014
#define XSPI_CMCFG1CS1		0x0024
#define XSPI_CMCFG1_RDCMD(val)	(((val) & 0xffff) << 0)
#define XSPI_CMCFG1_RDCMD_UPPER_BYTE(val)	(((val) & 0xff) << 8)
#define XSPI_CMCFG1_RDLATE(val)	(((val) & 0x1f) << 16)

/* xSPI Command Map Configuration Register 2 CS(0/1) */
#define XSPI_CMCFG2CS0		0x0018
#define XSPI_CMCFG2CS1		0x0028
#define XSPI_CMCFG2_WRCMD(val)	(((val) & 0xffff) << 0)
#define XSPI_CMCFG2_WRCMD_UPPER(val)	(((val) & 0xff) << 8)
#define XSPI_CMCFG2_WRLATE(val)	(((val) & 0x1f) << 16)

/* xSPI Link I/O Configuration Register CS(0/1) */
#define XSPI_LIOCFGCS0		0x0050
#define XSPI_LIOCFGCS1		0x0054
#define XSPI_LIOCFG_PRTMD(val)	(((val) & 0x3ff) << 0)
#define XSPI_LIOCFG_LATEMV	BIT(10)
#define XSPI_LIOCFG_WRMSKMD	BIT(11)
#define XSPI_LIOCFG_CSMIN(val)	(((val) & 0x0f) << 16)
#define XSPI_LIOCFG_CSASTEX	BIT(20)
#define XSPI_LIOCFG_CSNEGEX	BIT(21)
#define XSPI_LIOCFG_SDRDRV	BIT(22)
#define XSPI_LIOCFG_SDRSMPMD	BIT(23)
#define XSPI_LIOCFG_SDRSMPSFT(val)	(((val) & 0x0f) << 24)
#define XSPI_LIOCFG_DDRSMPEX(val)	(((val) & 0x0f) << 28)

/* xSPI Bridge Map Control Register 0 */
#define XSPI_BMCTL0		0x0060
#define XSPI_BMCTL0_CS0ACC(val)	(((val) & 0x03) << 0)
#define XSPI_BMCTL0_CS1ACC(val)	(((val) & 0x03) << 2)

/* xSPI Bridge Map Control Register 1 */
#define XSPI_BMCTL1		0x0064
#define XSPI_BMCTL1_MWRPUSH	BIT(8)
#define XSPI_BMCTL1_PBUFCLR	BIT(10)

/* xSPI Command Manual Control Register 0 */
#define XSPI_CDCTL0		0x0070
#define XSPI_CDCTL0_TRREQ	BIT(0)
#define XSPI_CDCTL0_PERMD	BIT(1)
#define XSPI_CDCTL0_CSSEL	BIT(3)
#define XSPI_CDCTL0_TRNUM(val)	(((val) & 0x03) << 4)
#define XSPI_CDCTL0_PERITV(val)	(((val) & 0x1f) << 16)
#define XSPI_CDCTL0_PERREP(val)	(((val) & 0x0f) << 24)

/* xSPI Command Manual Control Register 1 */
#define XSPI_CDCTL1		0x0074

/* xSPI Command Manual Control Register 2 */
#define XSPI_CDCTL2		0x0078

/* xSPI Command Manual Type Buf 0/1/2/3 */
#define XSPI_CDTBUF0		0x0080
#define XSPI_CDTBUF1		0x0090
#define XSPI_CDTBUF2		0x00A0
#define XSPI_CDTBUF3		0x00B0
#define XSPI_CDTBUF_CMDSIZE(val)	(((val) & 0x03) << 0)
#define XSPI_CDTBUF_ADDSIZE(val)	(((val) & 0x07) << 2)
#define XSPI_CDTBUF_DATASIZE(val)	(((val) & 0x0f) << 5)
#define XSPI_CDTBUF_LATE(val)		(((val) & 0x1f) << 9)
#define XSPI_CDTBUF_TRTYPE	BIT(15)
#define XSPI_CDTBUF_CMD(val)		(((val) & 0xffff) << 16)
#define XSPI_CDTBUF_CMD_FIELD(val)	(((val) & 0xff) << 24)
#define XSPI_CDTBUF_EXCMD_FIELD(val)	(((val) & 0xff) << 16)


/* xSPI Command Manual Address Buff 0/1/2/3 */
#define XSPI_CDABUF0		0x0084
#define XSPI_CDABUF1		0x0094
#define XSPI_CDABUF2		0x00A4
#define XSPI_CDABUF3		0x00B4

/* xSPI Command Manual Data 0 Buf 0/1/2/3 */
#define XSPI_CDD0BUF0		0x0088
#define XSPI_CDD0BUF1		0x0098
#define XSPI_CDD0BUF2		0x00A8
#define XSPI_CDD0BUF3		0x00B8

/* xSPI Command Manual Data 1 Buf 0/1/2/3 */
#define XSPI_CDD1BUF0		0x008C
#define XSPI_CDD1BUF1		0x009C
#define XSPI_CDD1BUF2		0x00AC
#define XSPI_CDD1BUF3		0x00BC

/* xSPI Command Calibration Control Register 0 CS(0/1) */
#define XSPI_CCCTL0CS0		0x0130
#define XSPI_CCCTL0CS1		0x0150
#define XSPI_CCCTL0_CAEN	BIT(0)
#define XSPI_CCCTL0_CANOWR	BIT(1)
#define XSPI_CCCTL0_CAITV(val)		(((val) & 0x1f) << 8)
#define XSPI_CCCTL0_CASFTSTA(val)	(((val) & 0x1f) << 16)
#define XSPI_CCCTL0_CASFTEND(val)	(((val) & 0x1f) << 24)

/* xSPI Command Calibration Control Register 1 CS(0/1) */
#define XSPI_CCCTL1CS0		0x0134
#define XSPI_CCCTL1CS1		0x0154
#define XSPI_CCCTL1_CACMDSIZE(val)	(((val) & 0x03) << 0)
#define XSPI_CCCTL1_CAADDSIZE(val)	(((val) & 0x07) << 2)
#define XSPI_CCCTL1_CADARASIZE(val)	(((val) & 0x0f) << 5)
#define XSPI_CCCTL1_CAWRLATE(val)	(((val) & 0x1f) << 16)
#define XSPI_CCCTL1_CARDLATE(val)	(((val) & 0x1f) << 24)

/* xSPI Command Calibration Control Register 2/3/4/5/6/7 CS(0/1) */
#define XSPI_CCCTL2CS0		0x0138
#define XSPI_CCCTL2CS1		0x0158
#define XSPI_CCCTL3CS0		0x013C
#define XSPI_CCCTL3CS1		0x015C
#define XSPI_CCCTL4CS0		0x0140
#define XSPI_CCCTL4CS1		0x0160
#define XSPI_CCCTL5CS0		0x0144
#define XSPI_CCCTL5CS1		0x0164
#define XSPI_CCCTL6CS0		0x0148
#define XSPI_CCCTL6CS1		0x0168
#define XSPI_CCCTL7CS0		0x014C
#define XSPI_CCCTL7CS1		0x016C

/* xSPI Common Status Register */
#define XSPI_COMSTT		0x0184
#define XSPI_COMSTT_MEMACC	BIT(0)
#define XSPI_COMSTT_PBUFNE	BIT(4)
#define XSPI_COMSTT_WRBUFNE	BIT(6)
#define XSPI_COMSTT_ECSCS0	BIT(16)
#define XSPI_COMSTT_INTCS0	BIT(17)
#define XSPI_COMSTT_RSTOCS0	BIT(18)
#define XSPI_COMSTT_ECSCS1	BIT(20)
#define XSPI_COMSTT_INTCS1	BIT(21)
#define XSPI_COMSTT_RSTOCS1	BIT(22)

/* xSPI Interrupt Status Register */
#define XSPI_INTS		0x0190
#define XSPI_INTS_CMDCMP	BIT(0)
#define XSPI_INTS_PATCPM	BIT(1)
#define XSPI_INTS_INICPM	BIT(2)
#define XSPI_INTS_PERTO		BIT(3)
#define XSPI_INTS_DSTOCS0	BIT(4)
#define XSPI_INTS_DSTOCS1	BIT(5)
#define XSPI_INTS_ECSCS0	BIT(8)
#define XSPI_INTS_ECSCS1	BIT(9)
#define XSPI_INTS_INTCS0	BIT(12)
#define XSPI_INTS_INTCS1	BIT(13)
#define XSPI_INTS_BUSERR	BIT(20)
#define XSPI_INTS_CAFAILCS0	BIT(28)
#define XSPI_INTS_CAFAILCS1	BIT(29)
#define XSPI_INTS_CASUCCS0	BIT(30)
#define XSPI_INTS_CASUCCS1	BIT(31)

/* xSPI Interrupt Clear Register */
#define XSPI_INTC		0x0194
#define XSPI_INTC_CMDCMPC	BIT(0)
#define XSPI_INTC_CPATCMPC	BIT(1)
#define XSPI_INTC_INICMPC	BIT(2)
#define XSPI_INTC_PERTOC	BIT(3)
#define XSPI_INTC_DSTOCS0C	BIT(4)
#define XSPI_INTC_DSTOCS1C	BIT(5)
#define XSPI_INTC_BUSERRCH0	BIT(20)
#define XSPI_INTC_CAFAILCS0C	BIT(28)
#define XSPI_INTC_CAFAILCS1C	BIT(29)
#define XSPI_INTC_CASUCCS0C	BIT(30)
#define XSPI_INTC_CASUCCS1C	BIT(31)

/* xSPI Interrupt Enable Register */
#define XSPI_INTE		0x0198
#define XSPI_INTE_CMDCMPE	BIT(0)
#define XSPI_INTE_PATCMPE	BIT(1)
#define XSPI_INTE_INICMPE	BIT(2)
#define XSPI_INTE_PERTOE	BIT(3)
#define XSPI_INTE_DSTOCS0E	BIT(4)
#define XSPI_INTE_DSTOCS1E	BIT(5)
#define XSPI_INTE_ECSCS0E	BIT(8)
#define XSPI_INTE_ECSCS1E	BIT(9)
#define XSPI_INTE_INTCS0E	BIT(12)
#define XSPI_INTE_INTCS1E	BIT(13)
#define XSPI_INTE_BUSERRE	BIT(20)
#define XSPI_INTE_CAFAILCS0E	BIT(28)
#define XSPI_INTE_CAFAILCS1E	BIT(29)
#define XSPI_INTE_CASUCCS0E	BIT(30)
#define XSPI_INTE_CASUCCS1E	BIT(31)

/* Maximum data size of MWRSIZE*/
#define MWRSIZE_MAX		64

/* xSPI Protocol mode */
#define PROTO_1S_2S_2S		0x48
#define PROTO_2S_2S_2S		0x49
#define PROTO_1S_4S_4S		0x090
#define PROTO_4S_4S_4S		0x092

#endif /* __RENESAS_XSPI_IF_REGS_H__ */
