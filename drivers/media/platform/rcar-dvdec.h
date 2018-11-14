/*
 * drivers/media/platform/rcar-dvdec.h
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _RCAR_DVDEC_H_
#define _RCAR_DVDEC_H_

#define V4L2_CID_USER_R8A7794X_INPUT	(V4L2_CID_USER_R8A7794X_BASE + 0)
#define V4L2_CID_USER_R8A7747X_INPUT	(V4L2_CID_USER_R8A7747X_BASE + 0)

struct reg_color {
	u16 TGCR1;
	u16 TGCR2;
	u16 TGCR3;
	u16 HAFCCR1;
	u16 HAFCCR2;
	u16 HAFCCR3;
	u16 VCDWCR1;
	u16 BTLCR;
	u16 ACCCR1;
	u16 AGCCR1;
	u16 YCSCR3;
	u16 YCSCR4;
	u16 YCSCR5;
	u16 YCSCR6;
	u16 YCSCR7;
	u16 YCSCR9;
	u16 YCSCR12;
};

struct dvdec_rect {
	u32 top;
	u32 height;
	u32 left;
	u32 width;
};

/* Color Format Id */
#define DVDEC_NTSC358	0
#define DVDEC_NTSC443	1
#define DVDEC_PAL443	2
#define DVDEC_PALM	3
#define DVDEC_PALN	4
#define DVDEC_SECAM	5
#define DVDEC_NTSC60	6
#define DVDEC_PAL60	7

/* Address of 24D. Digital Video Decoder registers  */
/* Offset value from 0xFEB81000 */
#define DVDEC_ADCCR1_REG			0x008
#define DVDEC_TGCR1_REG				0x00E
#define DVDEC_TGCR2_REG				0x010
#define DVDEC_TGCR3_REG				0x012
#define DVDEC_SYNSCR1_REG			0x01A
#define DVDEC_SYNSCR2_REG			0x01C
#define DVDEC_SYNSCR3_REG			0x01E
#define DVDEC_SYNSCR4_REG			0x020
#define DVDEC_SYNSCR5_REG			0x022
#define DVDEC_HAFCCR1_REG			0x024
#define DVDEC_HAFCCR2_REG			0x026
#define DVDEC_HAFCCR3_REG			0x028
#define DVDEC_VCDWCR1_REG			0x02A
#define DVDEC_DCPCR1_REG			0x030
#define DVDEC_DCPCR2_REG			0x032
#define DVDEC_DCPCR3_REG			0x034
#define DVDEC_DCPCR4_REG			0x036
#define DVDEC_DCPCR5_REG			0x038
#define DVDEC_DCPCR6_REG			0x03A
#define DVDEC_DCPCR7_REG			0x03C
#define DVDEC_DCPCR8_REG			0x03E
#define DVDEC_NSDCR_REG				0x040
#define DVDEC_BTLCR_REG				0x042
#define DVDEC_BTGPCR_REG			0x044
#define DVDEC_ACCCR1_REG			0x046
#define DVDEC_ACCCR2_REG			0x048
#define DVDEC_ACCCR3_REG			0x04A
#define DVDEC_TINTCR_REG			0x04C
#define DVDEC_YCDCR_REG				0x04E
#define DVDEC_AGCCR1_REG			0x050
#define DVDEC_AGCCR2_REG			0x052
#define DVDEC_PKLIMITCR_REG			0x054
#define DVDEC_RGORCR1_REG			0x056
#define DVDEC_RGORCR2_REG			0x058
#define DVDEC_RGORCR3_REG			0x05A
#define DVDEC_RGORCR4_REG			0x05C
#define DVDEC_RGORCR5_REG			0x05E
#define DVDEC_RGORCR6_REG			0x060
#define DVDEC_RGORCR7_REG			0x062

#define DVDEC_AFCPFCR_REG			0x07C
#define DVDEC_RUPDCR_REG			0x07E
#define DVDEC_VSYNCSR_REG			0x080
#define DVDEC_HSYNCSR_REG			0x082
#define DVDEC_DCPSR1_REG			0x084
#define DVDEC_DCPSR2_REG			0x086
#define DVDEC_NSDSR_REG				0x08C
#define DVDEC_CROMASR1_REG			0x08E
#define DVDEC_CROMASR2_REG			0x090
#define DVDEC_SYNCSSR_REG			0x092
#define DVDEC_AGCCSR1_REG			0x094
#define DVDEC_AGCCSR2_REG			0x096

#define DVDEC_YCSCR3_REG			0x104
#define DVDEC_YCSCR4_REG			0x106
#define DVDEC_YCSCR5_REG			0x108
#define DVDEC_YCSCR6_REG			0x10A
#define DVDEC_YCSCR7_REG			0x10C
#define DVDEC_YCSCR8_REG			0x10E
#define DVDEC_YCSCR9_REG			0x110
#define DVDEC_YCSCR11_REG			0x114
#define DVDEC_YCSCR12_REG			0x116

#define DVDEC_DCPCR9_REG			0x180

#define DVDEC_YCTWA_F0_REG			0x192
#define DVDEC_YCTWA_F1_REG			0x194
#define DVDEC_YCTWA_F2_REG			0x196
#define DVDEC_YCTWA_F3_REG			0x198
#define DVDEC_YCTWA_F4_REG			0x19A
#define DVDEC_YCTWA_F5_REG			0x19C
#define DVDEC_YCTWA_F6_REG			0x19E
#define DVDEC_YCTWA_F7_REG			0x1A0
#define DVDEC_YCTWA_F8_REG			0x1A2
#define DVDEC_YCTWB_F0_REG			0x1A4
#define DVDEC_YCTWB_F1_REG			0x1A6
#define DVDEC_YCTWB_F2_REG			0x1A8
#define DVDEC_YCTWB_F3_REG			0x1AA
#define DVDEC_YCTWB_F4_REG			0x1AC
#define DVDEC_YCTWB_F5_REG			0x1AE
#define DVDEC_YCTWB_F6_REG			0x1B0
#define DVDEC_YCTWB_F7_REG			0x1B2
#define DVDEC_YCTWB_F8_REG			0x1B4
#define DVDEC_YCTNA_F0_REG			0x1B6
#define DVDEC_YCTNA_F1_REG			0x1B8
#define DVDEC_YCTNA_F2_REG			0x1BA
#define DVDEC_YCTNA_F3_REG			0x1BC
#define DVDEC_YCTNA_F4_REG			0x1BE
#define DVDEC_YCTNA_F5_REG			0x1C0
#define DVDEC_YCTNA_F6_REG			0x1C2
#define DVDEC_YCTNA_F7_REG			0x1C4
#define DVDEC_YCTNA_F8_REG			0x1C6
#define DVDEC_YCTNB_F0_REG			0x1C8
#define DVDEC_YCTNB_F1_REG			0x1CA
#define DVDEC_YCTNB_F2_REG			0x1CC
#define DVDEC_YCTNB_F3_REG			0x1CE
#define DVDEC_YCTNB_F4_REG			0x1D0
#define DVDEC_YCTNB_F5_REG			0x1D2
#define DVDEC_YCTNB_F6_REG			0x1D4
#define DVDEC_YCTNB_F7_REG			0x1D6
#define DVDEC_YCTNB_F8_REG			0x1D8

#define DVDEC_YGAINCR_REG			0x200
#define DVDEC_CBGAINCR_REG			0x202
#define DVDEC_CRGAINCR_REG			0x204

#define DVDEC_PGA_UPDATE_REG			0x280
#define DVDEC_PGACR_REG				0x282
#define DVDEC_ADCCR2_REG			0x284
#define DVDEC_ADCCR3_REG			0x286


#define DVDEC_PMMR_REG				0xE6060000
#define DVDEC_ADCCR4_REG			0xE60600CC
#define MSTP8_REG					0xE6150990
#define MSTP8_DVDEC_MASK			0xFFFFF7FD

/* Burst Lock/Chroma Decoding Control Register */
#define DVDEC_BTLCR_DETECT_MASK			0x00FF
#define DVDEC_BTLCR_DETECT_AUTO			0x00C0
#define DVDEC_BTLCR_DETECT_NTSC_M		0x001F
#define DVDEC_BTLCR_DETECT_NTSC_443		0x002F
#define DVDEC_BTLCR_DETECT_PAL_443		0x007D
#define DVDEC_BTLCR_DETECT_PAL_M		0x0077
#define DVDEC_BTLCR_DETECT_PAL_N		0x007B
#define DVDEC_BTLCR_DETECT_PAL_60		0x007D
#define DVDEC_BTLCR_DETECT_SECAM		0x00BE

#define DVDEC_CROMASR1_COLORSYS_MASK		0xC000
#define DVDEC_CROMASR1_COLORSYS_NTSC		0x0000
#define DVDEC_CROMASR1_COLORSYS_PAL		0x4000
#define DVDEC_CROMASR1_COLORSYS_SECAM		0x8000
#define DVDEC_CROMASR1_COLORSYS_UNDETECTABLE	0xC000

#define DVDEC_CROMASR1_FSCMODE_MASK		0x2000
#define DVDEC_CROMASR1_FSCMODE_358		0x0000
#define DVDEC_CROMASR1_FSCMODE_443		0x2000

#define DVDEC_CROMASR2_ISSECAM			0x1000
#define DVDEC_CROMASR2_ISPAL			0x0800
#define DVDEC_CROMASR2_ISNTSC			0x0400

#define DVDEC_VSYNCSR_NOSIGNAL			0x0800
#define DVDEC_VSYNCSR_FVMODE_MASK		0x0200
#define DVDEC_VSYNCSR_FVMODE_50			0x0000
#define DVDEC_VSYNCSR_FVMODE_60			0x0200

#define DVDEC_YCTMINUS				0x1000

/* CONTRAST */
#define DVDEC_YGAINCR_MIN	0
#define DVDEC_YGAINCR_DEF	816
#define DVDEC_YGAINCR_MAX	1023
/* BLUE BALANCE */
#define DVDEC_CBGAINCR_MIN	0
#define DVDEC_CBGAINCR_DEF	663
#define DVDEC_CBGAINCR_MAX	1023
/* RED BALANCE */
#define DVDEC_CRGAINCR_MIN	0
#define DVDEC_CRGAINCR_DEF	663
#define DVDEC_CRGAINCR_MAX	1023

#define DVDEC_YCSCR12_MODE_MASK			0x000C
#define DVDEC_YCSCR12_MODE_BYPASS		0x0000
#define DVDEC_YCSCR12_MODE_CASCADE		0x0004
#define DVDEC_YCSCR12_MODE_TAKEOFF		0x0008

#define DVDEC_YCSCR12_NARROW_MASK		0x0001
#define DVDEC_YCSCR12_NARROW_BYPASS		0x0000
#define DVDEC_YCSCR12_NARROW_17TAP		0x0001

#define DVDEC_ADCCR2_ADC_VINSEL_MASK		0x0001

/* Power supply ON/OFF(ADCCR3 register) */
#define DVDEC_PSAV_ON		0x0200
#define DVDEC_PSAV_OFF		0x0210

/* R-Car E2X HW Users Manual rev0.5E */
/* Table 24D.42 Recommended Setting Common to Various Color Formats */
#define DVDEC_ADCCR1_INIT		(1 << 8)	/* AGCMODE        */

#define DVDEC_SYNSCR1_INIT		((3 << 13)	/* LPFVSYNC       */\
					| (5 << 10)	/* LPFHSYNC       */\
					| (2 << 4)	/* VELOCITYSHIFT_H */\
					| (2 << 2)	/* SLICERMODE_H   */\
					| (2 << 0))	/* SLICERMODE_V   */

#define DVDEC_SYNSCR2_INIT		((15 << 6)	/* SYNCMAXDUTY_H  */\
					| (10 << 0))	/* SYNCMINDUTY_H  */

#define DVDEC_SYNSCR3_INIT		((15 << 10)	/* SSCLIPSEL      */\
					| (146 << 0))	/* CSYNCSLICE_H   */

#define DVDEC_SYNSCR4_INIT		((15 << 6)	/* SYNCMAXDUTY_V  */\
					| (9 << 0))	/* SYNCMINDUTY_V  */

#define DVDEC_SYNSCR5_INIT		((0 << 15)	/* VSYNCDELAY     */\
					| (10 << 10)	/* VSYNCSLICE     */\
					| (146 << 0))	/* CSYNCSLICE_V   */

#define DVDEC_HAFCCR1_INIT		((12 << 12)	/* HAFCGAIN       */\
					| (0 << 10)	/* HAFCFREERUN    */\
					| (692 << 0))	/* HAFCTYP        */

#define DVDEC_HAFCCR2_INIT		((0 << 12)	/* HAFCSTART      */\
					| (1 << 11)	/* NOX2HOSC       */\
					| (0 << 10)	/* DOX2HOSC       */\
					| (742 << 0))	/* HAFCMAX	  */

#define DVDEC_HAFCCR3_INIT		((8 << 12)	/* HAFCEND        */\
					| (2 << 10)	/* HAFCMODE       */\
					| (642 << 0))	/* HAFCMIN        */

#define DVDEC_VCDWCR1_INIT		(0 << 15)	/* VCDFREERUN     */

#define DVDEC_VCDWCR1_AUTO		((0 << 15)	/* VCDFREERUN     */\
					| (0 << 14)	/* NOVCD50	  */\
					| (0 << 13)	/* NOVCD60	  */\
					| (0 << 12))	/* VCDDEFAULT	  */

#define DVDEC_VCDWCR1_IMASK		(0x8000)

#define DVDEC_VCDWCR1_AMASK		(0xF800)

#define DVDEC_DCPCR1_INIT		((1 << 15)	/* DCPMODE_Y      */\
					| (0 << 11)	/* DCPCHECK       */\
					| (984 << 0))	/* BLANKLEVEL_Y   */

#define DVDEC_DCPCR2_INIT		((0 << 15)	/* DCPMODE_C      */\
					| (0 << 6)	/* BLANKLEVEL_CB  */\
					| (0 << 0))	/* BLANKLEVEL_CR  */

#define DVDEC_DCPCR3_INIT		(0 << 12)	/* DCPRESPONSE    */

#define DVDEC_DCPCR4_INIT		(16 << 10)	/* DCPSTART       */

#define DVDEC_DCPCR5_INIT		(16 << 10)	/* DCPEND         */

#define DVDEC_DCPCR6_INIT		(27 << 8)	/* DCPWIDTH       */

#define DVDEC_DCPCR7_INIT		(162 << 8)	/* DCPPOS_Y       */

#define DVDEC_DCPCR8_INIT		(54 << 8)	/* DCPPOS_C       */

#define DVDEC_NSDCR_INIT		((0 << 12)	/* ACFINPUT       */\
					| (0 << 4)	/* ACFLAGTIME     */\
					| (3 << 0))	/* ACFFILTER      */

#define DVDEC_BTLCR_INIT		((1 << 14)	/* LOCKRANGE      */\
					| (3 << 12)	/* LOOPGAIN       */\
					| (1 << 10)	/* LOCKLIMIT      */\
					| (0 << 9))	/* BCOFREERUN     */

#define DVDEC_BTGPCR_INIT		((0 << 15)	/* BGPCHECK       */\
					| (54 << 8)	/* BGPWIDTH       */\
					| (110 << 0))	/* BGPSTART       */

#define DVDEC_ACCCR1_INIT		((5 << 12)	/* KILLEROFFSET   */\
					| (0 << 11)	/* ACCMODE        */\
					| (0 << 9)	/* ACCMAXGAIN     */\
					| (292 << 0))	/* ACCLEVEL       */

#define DVDEC_ACCCR2_INIT		((0 << 9)	/* CHROMASUBGAIN  */\
					| (210 << 0))	/* CHROMAMAINGAIN */

#define DVDEC_ACCCR3_INIT		((1 << 14)	/* ACCRESPONSE    */\
					| (8 << 8)	/* ACCPRECIS      */\
					| (0 << 7)	/* KILLERMODE     */\
					| (4 << 1))	/* KILLERLEVEL    */

#define DVDEC_TINTCR_INIT		((0 << 10)	/* TINTSUB        */\
					| (0 << 0))	/* TINTMAIN       */

#define DVDEC_YCDCR_INIT		((0 << 4)	/* LUMADELAY      */\
					| (0 << 2)	/* CHROMALPF      */\
					| (2 << 0))	/* DEMODMODE      */

#define DVDEC_AGCCR1_INIT		((0 << 13)	/* DUREDUCE       */\
					| (0 << 12)	/* NOREDUCE       */\
					| (4 << 9)	/* AGCRESPONSE    */\
					| (236 << 0))	/* AGCLEVEL       */

#define DVDEC_AGCCR2_INIT		(10 << 8)	/* AGCPRECIS      */
#define DVDEC_AGCCR2_IMASK		(0x3F00)

#define DVDEC_PKLIMITCR_INIT		((2 << 14)	/* PEAKLEVEL      */\
					| (2 << 12)	/* PEAKATTACK     */\
					| (3 << 10)	/* PEAKRELEASE    */\
					| (0 << 8)	/* PEAKRATIO      */\
					| (20 << 0))	/* MAXPEAKSAMPLES */

#define DVDEC_RGORCR1_INIT		(928 << 0)	/* RADJ_O_LEVEL0  */

#define DVDEC_RGORCR2_INIT		(32 << 0)	/* RADJ_U_LEVEL0  */

#define DVDEC_RGORCR3_INIT		(960 << 0)	/* RADJ_O_LEVEL1  */

#define DVDEC_RGORCR4_INIT		(48 << 0)	/* RADJ_U_LEVEL1  */

#define DVDEC_RGORCR5_INIT		(992 << 0)	/* RADJ_O_LEVEL2  */

#define DVDEC_RGORCR6_INIT		(64 << 0)	/* RADJ_U_LEVEL2  */

#define DVDEC_RGORCR7_INIT		((0 << 12)	/* TEST_MONI      */\
					| (0 << 9)	/* RADJ_MIX_K_FIX */\
					| (1 << 2)	/* UCMP_SW        */\
					| (1 << 1)	/* DCMP_SW        */\
					| (1 << 0))	/* HWIDE_SW       */

#define DVDEC_AFCPFCR_INIT		((0 << 4)	/* PHDET_FIX      */\
					| (5 << 0))	/* PHDET_DIV      */

#define DVDEC_RUPDCR_INIT		(1 << 15)	/* NEWSETTING     */

#define DVDEC_YCSCR8_INIT		((0 << 15)	/* HBPF_NARROW    */\
					| (0 << 14)	/* HVBPF_NARROW   */\
					| (0 << 13)	/* HBPF1_9TAP_ON  */\
					| (0 << 12)	/* HVBPF1_9TAP_ON */\
					| (0 << 11))	/* HFIL_TAP_SEL   */

#define DVDEC_YCSCR11_INIT		((108 << 9)	/* Reserved       */\
					| (0 << 0))	/* V_Y_LEVEL      */

#define DVDEC_DCPCR9_INIT		((7 << 13)	/* Reserved       */\
					| (0 << 12)	/* CLP_HOLD_ON_Y  */\
					| (0 << 11)	/* CLP_HOLD_ON_CB */\
					| (0 << 10))	/* CLP_HOLD_ON_CR */

#define DVDEC_YGAINCR_INIT		(816 << 0)	/* Y_GAIN2        */
#define DVDEC_CBGAINCR_INIT		(663 << 0)	/* CB_GAIN2       */
#define DVDEC_CRGAINCR_INIT		(663 << 0)	/* CR_GAIN2       */

#define DVDEC_PGA_UPDATE_INIT		(1 << 0)	/* PGA_VEN        */
#define DVDEC_PGACR_INIT		((0 << 14)	/* PGA_GAIN_SEL   */\
					| (0 << 8)	/* PGA_GAIN       */\
					| (8 << 0))	/* Reserved       */

#define DVDEC_ADCCR2_INIT		(0 << 0)	/* ADC_VINSEL     */

#endif /* _RCAR_DVDEC_H_ */