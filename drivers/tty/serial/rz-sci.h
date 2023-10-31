/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/bitops.h>
#include <linux/serial_core.h>
#include <linux/io.h>

#define SCI_MAJOR			240
#define SCI_MINOR_START			18

/*
 * RSCI register
 */
enum {
	RDR,								/* Receive Data Register */
	TDR,								/* Transmit Data Register */
	CCR0,								/* Common Control Register 0 */
	CCR1,								/* Common Control Register 1 */
	CCR2,								/* Common Control Register 2 */
	CCR3,								/* Common Control Register 3 */
	CCR4,								/* Common Control Register 4 */
	FCR,								/* FIFO Control Register */
	DCR,								/* Driver Control Register */
	CSR,								/* Common Status Register */
	FRSR,								/* FIFO Receive Status Register */
	FTSR,								/* FIFO Transmit Status Register */
	CFCLR,								/* Common Flag CLear Register */
	FFCLR,								/* FIFO Flag CLear Register */

	SCIx_NR_REGS,
};

/* RDR (Receive Data Register) */
#define RDR_FER				BIT(28)				/* Framing Error */
#define RDR_PER				BIT(27)				/* Parity Error */
#define RDR_ORER			BIT(24)				/* Overrun Error */
#define RDR_FFER			BIT(12)				/* FIFO Framing Error */
#define RDR_FPER			BIT(11)				/* FIFO Parity Error */
#define RDR_DR				BIT(10)				/* Incoming Data Ready */
#define RDR_MPB				BIT(9)				/* Multiprocessor Bit */
#define RDR_RDAT_9BIT_LSHIFT		0
#define RDR_RDAT_9BIT_VAL		0
#define	RDR_RDAT_9BIT_MSK		(RDR_RDAT_9BIT_VAL << RDR_RDAT_9BIT_LSHIFT)
#define	RDR_RDAT_MSK			GENMASK(8, 0)

/* TDR (Transmit Data Register) */
#define TDR_MPBT			BIT(9)				/* Multiprocessor Transfer */
#define TDR_TDAT_9BIT_LSHIFT		0
#define TDR_TDAT_9BIT_VAL		0x1FF
#define	TDR_TDAT_9BIT_MSK		(TDR_TDAT_9BIT_VAL << TDR_TDAT_9BIT_LSHIFT)

/* CCR0 (Common Control Register 0) */
#define CCR0_SSE			BIT(24)				/* SSn# Pin Function Enable */
#define CCR0_TEIE			BIT(21)				/* Transmit End Interrupt Enable */
#define CCR0_TIE			BIT(20)				/* Transmit Interrupt Enable */
#define CCR0_RIE			BIT(16)				/* Receive Interrupt Enable */
#define CCR0_IDSEL			BIT(10)				/* ID Frame Select */
#define CCR0_DCME			BIT(9)				/* Data Compare Match Enable */
#define CCR0_MPIE			BIT(8)				/* Multiprocessor Interrupt Enable */
#define CCR0_TE				BIT(4)				/* Transmit Enable */
#define CCR0_RE				BIT(0)				/* Receive Enable */

/* CCR1 (Common Control Register 1) */
#define CCR1_NFEN			BIT(28)				/* Digital Noise Filter Function */
#define CCR1_SHARPS			BIT(20)				/* Half -duplex Communication Select */
#define CCR1_SPLP			BIT(16)				/* Loopback Control */
#define CCR1_RINV			BIT(13)				/* RxD invert */
#define CCR1_TINV			BIT(12)				/* TxD invert */
#define CCR1_PM				BIT(9)				/* Parity Mode */
#define CCR1_PE				BIT(8)				/* Parity Enable */
#define CCR1_SPB2IO			BIT(5)				/* Serial Port Break I/O */
#define CCR1_SPB2DT			BIT(4)				/* Serial Port Break Data Select */
#define CCR1_CTSPEN			BIT(1)				/* CTS External Pin Enable */
#define CCR1_CTSE			BIT(0)				/* CTS Enable */

/* CCR2 (Common Control Register 2) */
#define	CCR2_INIT			0xFF000004
#define CCR2_CKS_TCLK			(0)				/* TCLK clock */
#define CCR2_CKS_TCLK_DIV4		BIT(20)				/* TCLK/4 clock */
#define CCR2_CKS_TCLK_DIV16		BIT(21)				/* TCLK16 clock */
#define CCR2_CKS_TCLK_DIV64		(BIT(21) | BIT(20))		/* TCLK/64 clock */
#define CCR2_BRME			BIT(16)				/* Bitrate Modulation Enable */
#define CCR2_ABCSE			BIT(6)				/* Asynchronous Mode Extended Base Clock Select */
#define CCR2_ABCS			BIT(5)				/* Asynchronous Mode Base Clock Select */
#define CCR2_BGDM			BIT(4)				/* Baud Rate Generator Double-Speed Mode Select */

/* CCR3 (Common Control Register 3) */
#define	CCR3_INIT			0x1203
#define CCR3_BLK			BIT(29)				/* Block Transfer Mode */
#define CCR3_GM				BIT(28)				/* GSM Mode */
#define CCR3_CKE1			BIT(25)				/* Clock Enable 1 */
#define CCR3_CKE0			BIT(24)				/* Clock Enable 0 */
#define CCR3_DEN			BIT(21)				/* Driver Enabled */
#define CCR3_FM				BIT(20)				/* FIFO Mode Select */
#define CCR3_MP				BIT(19)				/* Multi-Processor Mode */
#define CCR3_MOD_ASYNC			0				/* Asynchronous mode (Multi-processor mode) */
#define CCR3_MOD_IRDA			BIT(16)				/* Smart card interface mode */
#define CCR3_MOD_CLK_SYNC		BIT(17)				/* Clock synchronous mode */
#define CCR3_MOD_SPI			(BIT(17) | BIT(16))		/* Simple SPI mode */
#define CCR3_MOD_I2C			BIT(18)				/* Simple I2C mode */
#define CCR3_RXDESEL			BIT(15)				/* Asynchronous Start Bit Edge Detection Select */
#define CCR3_STP			BIT(14)				/* Stop bit Length */
#define CCR3_SINV			BIT(13)				/* Transmitted/Received Data Invert */
#define CCR3_LSBF			BIT(12)				/* LSB First select */
#define CCR3_CHR1			BIT(9)				/* Character Length */
#define CCR3_CHR0			BIT(8)				/* Character Length */
#define CCR3_BPEN			BIT(7)				/* Synchronizer Bypass Enable */
#define CCR3_CPOL			BIT(1)				/* Clock Polarity Select */
#define CCR3_CPHA			BIT(0)				/* Clock Phase Select */

/* FCR (FIFO Control Register) */
#define FCR_RFRST			BIT(23)				/* Receive FIFO Data Register Reset */
#define FCR_TFRST			BIT(15)				/* Transmit FIFO Data Register Reset */
#define FCR_DRES			BIT(0)				/* Incoming Data Ready Error Select */
#define FCR_RTRG4_0			GENMASK(20, 16)

/* CSR (Common Status Register) */
#define CSR_RDRF			BIT(31)				/* Receive Data Full */
#define CSR_TEND			BIT(30)				/* Transmit End Flag */
#define CSR_TDRE			BIT(29)				/* Transmit Data Empty */
#define CSR_FER				BIT(28)				/* Framing Error */
#define CSR_PER				BIT(27)				/* Parity Error */
#define CSR_MFF				BIT(26)				/* Mode Fault Error */
#define CSR_ORER			BIT(24)				/* Overrun Error */
#define CSR_DFER			BIT(18)				/* Data Compare Match Framing Error */
#define CSR_DPER			BIT(17)				/* Data Compare Match Parity Error */
#define CSR_DCMF			BIT(16)				/* Data Compare Match */
#define CSR_RXDMON			BIT(15)				/* Serial Input Data Monitor */
#define CSR_ERS				BIT(4)				/* Error Signal Status */

#define SCxSR_ERRORS(port)		(to_sci_port(port)->params->error_mask)
#define SCxSR_ERROR_CLEAR(port)		(to_sci_port(port)->params->error_clear)

#define RSCI_DEFAULT_ERROR_MASK		(CSR_PER | CSR_FER)

#define RSCI_RDxF_CLEAR			(CFCLR_RDRFC)
#define RSCI_ERROR_CLEAR		(CFCLR_PERC | CFCLR_FERC)
#define RSCI_TDxE_CLEAR			(CFCLR_TDREC)
#define RSCI_BREAK_CLEAR		(CFCLR_PERC | CFCLR_FERC | CFCLR_ORERC)

/* FRSR (FIFO Receive Status Register) */
#define FRSR_R5_0			GENMASK(13, 8)			/* Receive FIFO Data Count */
#define FRSR_DR				BIT(0)				/* Receive Data Ready */

/* CFCLR (Common Flag CLear Register) */
#define CFCLR_RDRFC			BIT(31)				/* RDRF Clear */
#define CFCLR_TDREC			BIT(29)				/* TDRE Clear */
#define CFCLR_FERC			BIT(28)				/* FER Clear */
#define CFCLR_PERC			BIT(27)				/* PER Clear */
#define CFCLR_MFFC			BIT(26)				/* MFF Clear */
#define CFCLR_ORERC			BIT(24)				/* ORER Clear */
#define CFCLR_DFERC			BIT(18)				/* DFER Clear */
#define CFCLR_DPERC			BIT(17)				/* DPER Clear */
#define CFCLR_DCMFC			BIT(16)				/* DCMF Clear */
#define CFCLR_ERSC			BIT(4)				/* ERS Clear */
#define CFCLR_CLRFLAG			(CFCLR_RDRFC | CFCLR_FERC | CFCLR_PERC | CFCLR_MFFC | CFCLR_ORERC | \
					CFCLR_DFERC | CFCLR_DPERC | CFCLR_DCMFC | CFCLR_ERSC)

/* FFCLR (FIFO Flag CLear Register) */
#define FFCLR_DRC			BIT(0)				/* DR Clear */
