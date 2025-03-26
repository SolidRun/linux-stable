// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RSCI SPI functionality driver.
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 *
 * Based on spi-rspi.c:
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/sh_dma.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl-state.h>

/* RSCI register*/
#define RSCI_RDR		0x00	/* Receive Data Register */
#define RSCI_TDR		0x04	/* Transmit Data Register */
#define RSCI_CCR0		0x08	/* Common Control Register 0 */
#define RSCI_CCR1		0x0c	/* Common Control Register 1 */
#define RSCI_CCR2		0x10	/* Common Control Register 2 */
#define RSCI_CCR3		0x14	/* Common Control Register 3 */
#define RSCI_CCR4		0x18	/* Common Control Register 4 */
#define RSCI_FCR		0x24	/* FIFO Control Register */
#define RSCI_CSR		0x48	/* Common Status Register */
#define RSCI_FRSR		0x50	/* FIFO Receive Status Register */
#define RSCI_FTSR		0x54	/* FIFO Transmit Status Register */
#define RSCI_CFCLR		0x68	/* Common Flag CLear Register */
#define RSCI_FFCLR		0x70	/* FIFO Flag Clear Register */

/* CCR0 - Common Control Register 0 */
#define CCR0_RE			BIT(0)
#define CCR0_TE			BIT(4)
#define CCR0_RIE		BIT(16)
#define CCR0_TIE		BIT(20)
#define CCR0_TEIE		BIT(21)
#define CCR0_SSE		BIT(24)

/* CCR1 - Common Control Register 1 */
#define CCR1_CTSE		BIT(0)
#define CCR1_SPLP		BIT(16)

/* CCR2 - Common Control Register 2 */
#define CCR2_BCP		(0x4 << 0)
#define CCR2_BGDM		BIT(4)
#define CCR2_BRR(x)		((x) << 8)
#define CCR2_CKS(x)		((x) << 20)
#define CCR2_MDDR		(0xFF << 24)

/* CCR3 - Common Control Register 3 */
#define CCR3_CPHA		BIT(0)
#define CCR3_CPOL		BIT(1)
#define CCR3_CHR_8BIT		(0x2 << 8)
#define CCR3_LSBF		BIT(12)
#define CCR3_MOD(x)		((x) << 16)
#define CCR3_FM			BIT(20)
#define CCR3_CKE(x)		((x) << 24)

/* CCR4 - Common Control Register 4 */
#define CCR4_ASEN		BIT(16)
#define CCR4_AST(x)		((x) << 24)

/* FIFO Control Register */
#define FCR_TTRG(x)		((x) << 8)
#define FCR_TFRST		BIT(15)
#define FCR_RTRG(x)		((x) << 16)
#define FCR_RFRST		BIT(23)
#define FCR_RSTRG(x)		((x) << 24)

/* Common Status Register */
#define CSR_ORER		BIT(24)
#define CSR_TDRE		BIT(29)
#define CSR_TEND		BIT(30)
#define CSR_RDRF		BIT(31)

/* Common Flag Clear Register */
#define CFCLR_ERSC		BIT(4)
#define CFCLR_DCMFC		BIT(16)
#define CFCLR_DPERC		BIT(17)
#define CFCLR_DFERC		BIT(18)
#define CFCLR_ORERC		BIT(24)
#define CFCLR_MFFC		BIT(26)
#define CFCLR_PERC		BIT(27)
#define CFCLR_FERC		BIT(28)
#define CFCLR_TDREC		BIT(29)
#define CFCLR_RDRFC		BIT(31)
#define CFCLR_INIT		(CFCLR_RDRFC | CFCLR_FERC | CFCLR_PERC | \
				 CFCLR_MFFC | CFCLR_ORERC | CFCLR_DFERC | \
				 CFCLR_DPERC | CFCLR_DCMFC | CFCLR_ERSC)

/* FIFO Flag Clear Register */
#define FFCLR_DRC		BIT(0)

enum {
	RSCI_SPI_MASTER,
	RSCI_SPI_SLAVE,
};

struct rsci_data {
	void __iomem *addr;
	u32 speed_hz;
	struct spi_controller *ctlr;
	struct platform_device *pdev;
	wait_queue_head_t wait;
	spinlock_t lock;	/* Protects RMW-access */
	struct clk *tclk;
	bool slave;
	u32 ccr0, ccr1, ccr2, ccr3, csr;
	int rx_irq, tx_irq;
	int bits_per_word;
	const struct spi_ops *ops;

	unsigned dma_callbacked:1;
	unsigned byte_access:1;
	struct reset_control *rstc;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_master, *pins_slave;
};

struct rsci_irq_desc {
	int res_num;
	irq_handler_t isr;
	char *name;
};

static void rsci_write8(const struct rsci_data *rsci, u8 data, u16 offset)
{
	iowrite8(data, rsci->addr + offset);
}

static void rsci_write32(const struct rsci_data *rsci, u32 data, u16 offset)
{
	iowrite32(data, rsci->addr + offset);
}

static u8 rsci_read8(const struct rsci_data *rsci, u16 offset)
{
	return ioread8(rsci->addr + offset);
}

static u32 rsci_read32(const struct rsci_data *rsci, u16 offset)
{
	return ioread32(rsci->addr + offset);
}

/* Operational functions */
struct spi_ops {
	int (*set_config_register)(struct rsci_data *rsci, int access_size);
	int (*transfer_one)(struct spi_controller *ctlr,
			    struct spi_device *spi, struct spi_transfer *xfer);
	u16 extra_mode_bits;
	u16 min_div;
	u16 max_div;
	u16 flags;
	u16 fifo_size;
	u8 num_hw_ss;
};

static void rsci_spi_set_rate(struct rsci_data *rsci)
{
	unsigned long clksrc;
	unsigned int sr = 1, prediv, scrate, c = 0, cks, brr;
	u32 ccr2;

	/*
	 * Find bit rate setting and clock select to match baud rate.
	 * NOTE: Currently the calculation formula is based on BGDM = 0
	 * and is described as in the specification.
	 */
	if (!spi_controller_is_slave(rsci->ctlr)) {
		clksrc = clk_get_rate(rsci->tclk);
		do {
			prediv = sr << (2 * c + 2);
			scrate = prediv * rsci->speed_hz;
			brr = DIV_ROUND_UP(clksrc, scrate) - 1;
			c++;
		} while (brr > 255 && c < 4);

		cks = c - 1;
		brr = clamp(brr, 0U, 255U);
		ccr2 = rsci_read32(rsci, RSCI_CCR2);
		ccr2 = CCR2_BCP | CCR2_MDDR | CCR2_BRR(brr) | CCR2_CKS(cks);
		ccr2 &= ~CCR2_BGDM;
		rsci_write32(rsci, ccr2, RSCI_CCR2);
		rsci->speed_hz = DIV_ROUND_UP(clksrc, prediv * (brr + 1));
	}
}

static int rsci_spi_common_set_config_register(struct rsci_data *rsci,
					       int access_size)
{
	u32 fcr, ccr0 = 0;

	/* Set CCR0 initial settings */
	rsci_write32(rsci, ccr0, RSCI_CCR0);

	/* Reset the number of the transmission/reception data stored in FIFO */
	rsci_write32(rsci, FCR_TFRST | FCR_RFRST, RSCI_FCR);

	/* Set FCR initial setting */
	fcr = FCR_TTRG(0x1F) | FCR_RTRG(0) | FCR_RSTRG(0x1f);
	rsci_write32(rsci, fcr, RSCI_FCR);

	/* Set CCR3 except MOD[2:0] initial setting */
	if (!spi_controller_is_slave(rsci->ctlr))
		rsci->ccr3 |= CCR3_CKE(0);
	else
		rsci->ccr3 |= CCR3_CKE(0x2);

	/* Enable FIFO mode */
	rsci->ccr3 |= CCR3_FM;
	rsci_write32(rsci, rsci->ccr3, RSCI_CCR3);

	/* Set character Length */
	rsci->ccr3 |= CCR3_CHR_8BIT;

	/* Set communication mode */
	rsci->ccr3 |= CCR3_MOD(0x3);
	rsci_write32(rsci, rsci->ccr3, RSCI_CCR3);

	/* Sets transfer bit rate */
	rsci_spi_set_rate(rsci);

	/* Sets CCR1 initial setting */
	rsci_write32(rsci, rsci->ccr1, RSCI_CCR1);

	/* Set CFCLR, FFCLR initial setting */
	rsci_write32(rsci, CFCLR_INIT, RSCI_CFCLR);
	rsci_write32(rsci, FFCLR_DRC, RSCI_FFCLR);

	/* Set SSn# pin function for slave */
	if (spi_controller_is_slave(rsci->ctlr))
		ccr0 |= CCR0_SSE;
	else
		ccr0 &= ~CCR0_SSE;
	rsci_write32(rsci, ccr0, RSCI_CCR0);

	/* CCR0 enable TE and RE */
	ccr0 |= CCR0_RE | CCR0_TE;
	rsci_write32(rsci, ccr0, RSCI_CCR0);

	return 0;
}

static void rsci_spi_endisable_irq(struct rsci_data *rsci, u32 enable,
				   u32 disable)
{
	if (enable)
		rsci_write32(rsci, rsci_read32(rsci, RSCI_CCR0) | enable,
			     RSCI_CCR0);
	else
		rsci_write32(rsci, rsci_read32(rsci, RSCI_CCR0) & ~disable,
			     RSCI_CCR0);
}

static int rsci_spi_wait_for_interrupt(struct rsci_data *rsci, u32 wait_mask,
				       u32 enable_bit)
{
	int ret;

	rsci->csr = rsci_read32(rsci, RSCI_CSR);
	if (rsci->csr & wait_mask)
		return 0;

	rsci_spi_endisable_irq(rsci, enable_bit, 0);

	ret = wait_event_timeout(rsci->wait, rsci->csr & wait_mask, 10 * HZ);
	if (ret == 0 && !(rsci->csr & wait_mask))
		return -ETIMEDOUT;

	return 0;
}

static inline int rsci_spi_wait_for_tx_empty(struct rsci_data *rsci)
{
	return rsci_spi_wait_for_interrupt(rsci, CSR_TDRE, CCR0_TIE);
}

static inline int rsci_spi_wait_for_rx_full(struct rsci_data *rsci)
{
	return rsci_spi_wait_for_interrupt(rsci, CSR_RDRF, CCR0_RIE);
}

static inline int rsci_spi_wait_for_tend(struct rsci_data *rsci)
{
	return rsci_spi_wait_for_interrupt(rsci, CSR_TEND, CCR0_TEIE);
}

static void rsci_spi_data_out_8(struct rsci_data *rsci, const void *tx,
				int count)
{
	const u8 *buf_8 = tx;

	rsci_write8(rsci, buf_8[count], RSCI_TDR);
}

static void rsci_spi_data_in_8(struct rsci_data *rsci, void *rx, int count)
{
	u8 *buf_8 = rx;

	buf_8[count] = rsci_read8(rsci, RSCI_RDR);
}

static int rsci_spi_pio_transfer(struct rsci_data *rsci, const void *tx,
				 void *rx, unsigned int words)
{
	void (*tx_fifo)(struct rsci_data *rsci, const void *tx, int count);
	void (*rx_fifo)(struct rsci_data *rsci, void *rx, int count);
	int ret, count, loop, loop_count, remained_words, words_per_loop;

	if (rsci->bits_per_word == 8) {
		tx_fifo = rsci_spi_data_out_8;
		rx_fifo = rsci_spi_data_in_8;
	} else
		return -EINVAL;

	if (words % rsci->ops->fifo_size)
		loop = (words / rsci->ops->fifo_size) + 1;
	else
		loop = words / rsci->ops->fifo_size;

	for (loop_count = 0; loop_count < loop; loop_count++) {
		remained_words = words - loop_count * rsci->ops->fifo_size;
		words_per_loop = (remained_words > rsci->ops->fifo_size) ?
					 rsci->ops->fifo_size : remained_words;
		if (tx) {
			for (count = 0; count < words_per_loop; count++) {
				ret = rsci_spi_wait_for_tx_empty(rsci);
				if (ret < 0) {
					dev_err(&rsci->ctlr->dev,
							 "transmit timeout\n");
					return ret;
				}

				tx_fifo(rsci, tx, count +
					    loop_count * rsci->ops->fifo_size);
			}

			ret = rsci_spi_wait_for_tend(rsci);
			if (ret < 0) {
				dev_err(&rsci->ctlr->dev,
					   "transmit end timeout %d\n", count);
				return ret;
			}
		}

		if (rx) {
			for (count = 0; count < words_per_loop; count++) {
				ret = rsci_spi_wait_for_rx_full(rsci);
				if (ret < 0) {
					dev_err(&rsci->ctlr->dev,
						"receive timeout %d\n", count);
					return ret;
				}
				rx_fifo(rsci, rx, count +
					    loop_count * rsci->ops->fifo_size);
			}
		}
	}

	return 0;
}

static void rsci_spi_dma_complete(void *arg)
{
	struct rsci_data *rsci = arg;

	rsci->dma_callbacked = 1;
	wake_up_interruptible(&rsci->wait);
}

static int rsci_spi_dma_transfer(struct rsci_data *rsci, struct sg_table *tx,
				 struct sg_table *rx)
{
	struct dma_async_tx_descriptor *desc_tx = NULL, *desc_rx = NULL;
	u32 irq_mask = 0;
	unsigned int other_irq = 0;
	dma_cookie_t cookie;
	int ret;

	/* First prepare and submit the DMA request(s), as this may fail */
	if (rx) {
		desc_rx = dmaengine_prep_slave_sg(rsci->ctlr->dma_rx, rx->sgl,
						  rx->nents, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_rx) {
			ret = -EAGAIN;
			goto no_dma_rx;
		}
		desc_rx->callback = rsci_spi_dma_complete;
		desc_rx->callback_param = rsci;
		cookie = dmaengine_submit(desc_rx);
		if (dma_submit_error(cookie)) {
			ret = cookie;
			goto no_dma_rx;
		}

		irq_mask |= CCR0_RIE;
	}

	if (tx) {
		desc_tx = dmaengine_prep_slave_sg(rsci->ctlr->dma_tx, tx->sgl,
						  tx->nents, DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_tx) {
			ret = -EAGAIN;
			goto no_dma_tx;
		}
		if (rx) {
			/* No callback */
			desc_tx->callback = NULL;
		} else {
			desc_tx->callback = rsci_spi_dma_complete;
			desc_tx->callback_param = rsci;
		}
		cookie = dmaengine_submit(desc_tx);
		if (dma_submit_error(cookie)) {
			ret = cookie;
			goto no_dma_tx;
		}

		irq_mask |= CCR0_TIE;
	}

	/*
	 * DMAC needs CCR0_xIE, but if CCR0_xIE is set, the IRQ routine will be
	 * called. So, this driver disables the IRQ while DMA transfer.
	 */
	if (tx)
		disable_irq(other_irq = rsci->tx_irq);
	if (rx && rsci->rx_irq != other_irq)
		disable_irq(rsci->rx_irq);

	rsci_spi_endisable_irq(rsci, irq_mask, 0);
	rsci->dma_callbacked = 0;

	/* Now start DMA */
	if (rx)
		dma_async_issue_pending(rsci->ctlr->dma_rx);
	if (tx) {
		dma_async_issue_pending(rsci->ctlr->dma_tx);
		rsci_write32(rsci, CFCLR_TDREC | CFCLR_RDRFC, RSCI_CFCLR);
	}
	ret = wait_event_interruptible_timeout(rsci->wait,
					       rsci->dma_callbacked, HZ);
	if (rsci->dma_callbacked) {
		ret = 0;
		if (tx)
			dmaengine_synchronize(rsci->ctlr->dma_tx);
		if (rx)
			dmaengine_synchronize(rsci->ctlr->dma_rx);
	} else {
		if (!ret) {
			dev_err(&rsci->ctlr->dev, "DMA timeout\n");
			ret = -ETIMEDOUT;
		}
		if (tx)
			dmaengine_terminate_sync(rsci->ctlr->dma_tx);
		if (rx)
			dmaengine_terminate_sync(rsci->ctlr->dma_rx);
	}

	rsci_spi_endisable_irq(rsci, 0, irq_mask);

	if (tx)
		enable_irq(rsci->tx_irq);
	if (rx && rsci->rx_irq != other_irq)
		enable_irq(rsci->rx_irq);
	return ret;

no_dma_tx:
	if (rx)
		dmaengine_terminate_sync(rsci->ctlr->dma_rx);
no_dma_rx:
	if (ret == -EAGAIN) {
		dev_warn_once(&rsci->ctlr->dev,
			"DMA not available, falling back to PIO\n");
	}
	return ret;
}

static void rsci_spi_receive_init(const struct rsci_data *rsci)
{
	u32 csr;

	csr = rsci_read32(rsci, RSCI_CSR);
	if (csr & CSR_RDRF)
		rsci_read8(rsci, RSCI_RDR);   /* dummy read */
	if (csr & CSR_ORER)
		rsci_write32(rsci, rsci_read32(rsci, RSCI_CFCLR) | CFCLR_ORERC,
								   RSCI_CFCLR);
}

static void rsci_spi_common_receive_init(const struct rsci_data *rsci)
{
	rsci_spi_receive_init(rsci);
}

static bool __rsci_spi_can_dma(const struct rsci_data *rsci,
			       const struct spi_transfer *xfer)
{
	return xfer->len > rsci->ops->fifo_size;
}

static bool rsci_spi_can_dma(struct spi_controller *ctlr,
			     struct spi_device *spi, struct spi_transfer *xfer)
{
	struct rsci_data *rsci = spi_controller_get_devdata(ctlr);

	return __rsci_spi_can_dma(rsci, xfer);
}

static int rsci_spi_dma_check_then_transfer(struct rsci_data *rsci,
					    struct spi_transfer *xfer)
{
	struct dma_slave_config cfg;

	if (!rsci->ctlr->can_dma || !__rsci_spi_can_dma(rsci, xfer))
		return -EAGAIN;

	memset(&cfg, 0, sizeof(cfg));

	cfg.dst_addr = rsci->pdev->resource->start + RSCI_TDR;
	cfg.src_addr = rsci->pdev->resource->start + RSCI_RDR;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	cfg.direction = DMA_MEM_TO_DEV;

	dmaengine_slave_config(rsci->ctlr->dma_tx, &cfg);

	cfg.direction = DMA_DEV_TO_MEM;

	dmaengine_slave_config(rsci->ctlr->dma_rx, &cfg);
	/* rx_buf can be NULL on RSCI SPI on SH in TX-only Mode */
	return rsci_spi_dma_transfer(rsci, &xfer->tx_sg,
				     xfer->rx_buf ? &xfer->rx_sg : NULL);
}

static int rsci_spi_common_transfer(struct rsci_data *rsci,
				    struct spi_transfer *xfer)
{
	int ret;

	xfer->effective_speed_hz = rsci->speed_hz;

	ret = rsci_spi_dma_check_then_transfer(rsci, xfer);
	if (ret != -EAGAIN)
		return ret;

	ret = rsci_spi_pio_transfer(rsci, xfer->tx_buf, xfer->rx_buf,
				    xfer->len);
	if (ret < 0)
		return ret;

	/* Wait for the last transmission */
	rsci_spi_wait_for_tx_empty(rsci);
	return 0;
}

static int rsci_spi_common_transfer_one(struct spi_controller *ctlr,
					struct spi_device *spi,
					struct spi_transfer *xfer)
{
	struct rsci_data *rsci = spi_controller_get_devdata(ctlr);

	rsci_spi_common_receive_init(rsci);
	return rsci_spi_common_transfer(rsci, xfer);
}

static int rsci_spi_setup(struct spi_device *spi)
{
	return 0;
}

static int rsci_spi_prepare_message(struct spi_controller *ctlr,
				    struct spi_message *msg)
{
	struct rsci_data *rsci = spi_controller_get_devdata(ctlr);
	struct spi_device *spi = msg->spi;
	const struct spi_transfer *xfer;

	/*
	 * As the Bit Rate Register must not be changed while the device is
	 * active, all transfers in a message must use the same bit rate.
	 * In theory, the sequencer could be enabled, and each Command Register
	 * could divide the base bit rate by a different value.
	 * However, most RSCI SPI variants do not have Transfer Data Length
	 * Multiplier Setting Registers, so each sequence step would be limited
	 * to a single word, making this feature unsuitable for large
	 * transfers, which would gain most from it.
	 */
	rsci->speed_hz = spi->max_speed_hz;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (xfer->speed_hz < rsci->speed_hz)
			rsci->speed_hz = xfer->speed_hz;
		rsci->bits_per_word = xfer->bits_per_word;
	}

	rsci->ccr3 = 0;
	if (spi->mode & SPI_CPOL)
		rsci->ccr3 |= CCR3_CPOL;
	if (spi->mode & SPI_CPHA)
		rsci->ccr3 |= CCR3_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		rsci->ccr3 |= CCR3_LSBF;

	rsci->ccr1 = 0;

	/* Set loop back mode */
	if (spi->mode & SPI_LOOP)
		rsci->ccr1 |= CCR1_SPLP;

	rsci->ops->set_config_register(rsci, rsci->bits_per_word);

	return 0;
}

static int rsci_spi_unprepare_message(struct spi_controller *ctlr,
				      struct spi_message *msg)
{
	struct rsci_data *rsci = spi_controller_get_devdata(ctlr);

	/* Disable SPI transfer function */
	rsci_write32(rsci, 0, RSCI_CCR0);

	return 0;
}

static irqreturn_t rsci_spi_irq_rx(int irq, void *_sr)
{
	struct rsci_data *rsci = _sr;
	u32 csr;

	rsci->csr = csr = rsci_read32(rsci, RSCI_CSR);
	if (csr & CSR_RDRF) {
		rsci_spi_endisable_irq(rsci, 0, CCR0_RIE);
		wake_up(&rsci->wait);
		return IRQ_HANDLED;
	}
	return 0;
}

static irqreturn_t rsci_spi_irq_tx(int irq, void *_sr)
{
	struct rsci_data *rsci = _sr;
	u32 csr;

	rsci->csr = csr = rsci_read32(rsci, RSCI_CSR);
	if (csr & CSR_TDRE) {
		rsci_spi_endisable_irq(rsci, 0, CCR0_TIE);
		wake_up(&rsci->wait);
		return IRQ_HANDLED;
	}
	return 0;
}

static irqreturn_t rsci_spi_irq_tei(int irq, void *_sr)
{
	struct rsci_data *rsci = _sr;
	u32 csr;

	rsci->csr = csr = rsci_read32(rsci, RSCI_CSR);
	if (csr & CSR_TEND) {
		rsci_spi_endisable_irq(rsci, 0, CCR0_TEIE);
		wake_up(&rsci->wait);
		return IRQ_HANDLED;
	}
	return 0;
}

static irqreturn_t rsci_spi_irq_err(int irq, void *_sr)
{
	struct rsci_data *rsci = _sr;
	u32 csr;

	rsci_spi_receive_init(rsci);
	rsci->csr = csr = rsci_read32(rsci, RSCI_CSR);
	if (!(csr & CSR_ORER))
		return IRQ_HANDLED;
	return 0;
}

static struct dma_chan *rsci_spi_request_dma_chan(struct device *dev,
						enum dma_transfer_direction dir,
						unsigned int id)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	chan = dma_request_slave_channel_compat(mask, shdma_chan_filter,
					(void *)(unsigned long)id, dev,
					dir == DMA_MEM_TO_DEV ? "tx" : "rx");
	if (!chan) {
		dev_warn(dev, "dma_request_slave_channel_compat failed\n");
		return NULL;
	}

	return chan;
}

static int rsci_spi_request_dma(struct device *dev, struct spi_controller *ctlr)
{
	unsigned int dma_tx_id, dma_rx_id;

	if (dev->of_node) {
		/* In the OF case we will get the slave IDs from the DT */
		dma_tx_id = 0;
		dma_rx_id = 0;
	} else {
		/* The driver assumes no error. */
		return 0;
	}

	ctlr->dma_tx = rsci_spi_request_dma_chan(dev, DMA_MEM_TO_DEV, dma_tx_id);
	if (!ctlr->dma_tx)
		return -ENODEV;

	ctlr->dma_rx = rsci_spi_request_dma_chan(dev, DMA_DEV_TO_MEM, dma_rx_id);
	if (!ctlr->dma_rx) {
		dma_release_channel(ctlr->dma_tx);
		ctlr->dma_tx = NULL;
		return -ENODEV;
	}

	ctlr->can_dma = rsci_spi_can_dma;
	dev_info(dev, "DMA available");
	return 0;
}

static void rsci_spi_release_dma(struct spi_controller *ctlr)
{
	if (ctlr->dma_tx)
		dma_release_channel(ctlr->dma_tx);
	if (ctlr->dma_rx)
		dma_release_channel(ctlr->dma_rx);
}

static int rsci_spi_remove(struct platform_device *pdev)
{
	struct rsci_data *rsci = platform_get_drvdata(pdev);

	rsci_spi_release_dma(rsci->ctlr);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct spi_ops rsci_common_ops = {
	.set_config_register	=	rsci_spi_common_set_config_register,
	.transfer_one		=	rsci_spi_common_transfer_one,
	.min_div		=	2,
	.max_div		=	4096,
	.flags			=	SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX,
	.fifo_size		=	32,	/* 32 for TX, 32 for RX */
	.num_hw_ss		=	0,
};

#ifdef CONFIG_OF
static const struct of_device_id rsci_of_match[] = {
	/* RSCI SPI on RZ/V2H and similar SoCs */
	{ .compatible = "renesas,rsci-spi", .data = &rsci_common_ops },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, rsci_of_match);

static void rsci_spi_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static int rsci_spi_mode(struct device *dev)
{
	return of_property_read_bool(dev->of_node, "spi-slave") ? RSCI_SPI_SLAVE
								: RSCI_SPI_MASTER;
}

static int rsci_spi_parse_dt(struct device *dev, struct spi_controller *ctlr)
{
	struct rsci_data *rsci = dev_get_drvdata(dev);
	struct reset_control *rstc;
	struct pinctrl_state *pin_state;
	u32 num_cs;
	int error;

	/* Parse DT properties */
	if (!rsci->slave) {
		error = of_property_read_u32(dev->of_node, "num-cs", &num_cs);
		if (error) {
			dev_err(dev, "of_property_read_u32 num-cs failed %d\n", error);
			return error;
		}

		ctlr->num_chipselect = num_cs;
	}

	rstc = devm_reset_control_array_get(dev, false, false);
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc),
				     "failed to get presetn reset\n");

	rsci->rstc = rstc;

	error = reset_control_deassert(rstc);
	if (error) {
		dev_err(dev, "failed to deassert reset %d\n", error);
		return error;
	}

	error = devm_add_action_or_reset(dev, rsci_spi_reset_control_assert, rstc);
	if (error) {
		dev_err(dev, "failed to register assert devm action, %d\n",
			error);
		return error;
	}

	rsci->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(rsci->pinctrl)) {
		rsci->pins_master = pinctrl_lookup_state(rsci->pinctrl,
							  "master");
		rsci->pins_slave = pinctrl_lookup_state(rsci->pinctrl,
							"slave");
	}

	if (rsci->slave)
		pin_state = rsci->pins_slave;
	else
		pin_state = rsci->pins_master;

	error = pinctrl_select_state(rsci->pinctrl, pin_state);
	if (error) {
		dev_err(dev, "failed to setting pin_state %d\n", error);
		return error;
	}

	return 0;
}
#else
#define rsci_of_match	NULL
static inline int rsci_spi_parse_dt(struct device *dev, struct spi_controller *ctlr)
{
	return -EINVAL;
}
#endif /* CONFIG_OF */

static struct rsci_irq_desc rsci_irqs[] = {
	{ .res_num = 0, .isr = rsci_spi_irq_err, .name = "rsci-spi-error" },
	{ .res_num = 1, .isr = rsci_spi_irq_rx,  .name = "rsci-spi-rxi" },
	{ .res_num = 2, .isr = rsci_spi_irq_tx,  .name = "rsci-spi-txi" },
	{ .res_num = 3, .isr = rsci_spi_irq_tei, .name = "rsci-spi-tei" },
};

static int rsci_spi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_controller *ctlr;
	struct rsci_data *rsci;
	bool slave;
	int ret, i;
	const struct spi_ops *ops;
	unsigned long clksrc;

	ret = rsci_spi_mode(&pdev->dev);
	if (ret == RSCI_SPI_MASTER) {
		ctlr = spi_alloc_master(&pdev->dev, sizeof(struct rsci_data));
		slave = 0;
	} else {
		ctlr = spi_alloc_slave(&pdev->dev, sizeof(struct rsci_data));
		slave = 1;
	}
	if (ctlr == NULL)
		return -ENOMEM;

	rsci = spi_controller_get_devdata(ctlr);
	platform_set_drvdata(pdev, rsci);
	rsci->slave = slave;

	ops = of_device_get_match_data(&pdev->dev);
	if (ops) {
		ret = rsci_spi_parse_dt(&pdev->dev, ctlr);
		if (ret)
			goto error1;
	} else {
		ops = (struct spi_ops *)pdev->id_entry->driver_data;
		ctlr->num_chipselect = 1; /* default */
	}

	rsci->ops = ops;
	rsci->ctlr = ctlr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rsci->addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rsci->addr)) {
		ret = PTR_ERR(rsci->addr);
		goto error1;
	}

	rsci->tclk = devm_clk_get(&pdev->dev, "tclk");
	if (IS_ERR(rsci->tclk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = PTR_ERR(rsci->tclk);
		goto error1;
	}

	rsci->pdev = pdev;
	pm_runtime_enable(&pdev->dev);
	pm_runtime_resume_and_get(&pdev->dev);

	init_waitqueue_head(&rsci->wait);
	spin_lock_init(&rsci->lock);

	ctlr->bus_num = pdev->id;
	ctlr->setup = rsci_spi_setup;
	ctlr->auto_runtime_pm = true;
	ctlr->transfer_one = ops->transfer_one;
	ctlr->prepare_message = rsci_spi_prepare_message;
	ctlr->unprepare_message = rsci_spi_unprepare_message;
	ctlr->mode_bits = SPI_CPHA | SPI_CPOL | SPI_CS_HIGH | SPI_LSB_FIRST |
						SPI_LOOP | ops->extra_mode_bits;
	clksrc = clk_get_rate(rsci->tclk);
	ctlr->min_speed_hz = DIV_ROUND_UP(clksrc, ops->max_div);
	ctlr->max_speed_hz = DIV_ROUND_UP(clksrc, ops->min_div);
	ctlr->flags = ops->flags;
	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->use_gpio_descriptors = true;
	ctlr->max_native_cs = rsci->ops->num_hw_ss;

	for (i = 0; i < ARRAY_SIZE(rsci_irqs); i++) {
		ret = platform_get_irq(pdev, rsci_irqs[i].res_num);
		if (ret < 0)
			return ret;

		if (rsci_irqs[i].res_num == 1)
			rsci->rx_irq = ret;
		else if (rsci_irqs[i].res_num == 2)
			rsci->tx_irq = ret;

		ret = devm_request_irq(&pdev->dev, ret, rsci_irqs[i].isr,
						0, rsci_irqs[i].name, rsci);
		if (ret) {
			dev_err(&pdev->dev, "failed to request irq %s\n", rsci_irqs[i].name);
			goto error2;
		}
	}

	ret = rsci_spi_request_dma(&pdev->dev, ctlr);
	if (ret < 0)
		dev_warn(&pdev->dev, "DMA not available, using PIO\n");

	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret < 0) {
		dev_err(&pdev->dev, "devm_spi_register_controller error.\n");
		goto error3;
	}

	dev_info(&pdev->dev, "probed\n");

	return 0;

error3:
	rsci_spi_release_dma(ctlr);
error2:
	pm_runtime_disable(&pdev->dev);
error1:
	spi_controller_put(ctlr);

	return ret;
}

static const struct platform_device_id spi_driver_ids[] = {
	{ "rsci-spi", (kernel_ulong_t)&rsci_common_ops },
	{},
};

MODULE_DEVICE_TABLE(platform, spi_driver_ids);

#ifdef CONFIG_PM_SLEEP
static int rsci_spi_suspend(struct device *dev)
{
	struct rsci_data *rsci = dev_get_drvdata(dev);

	reset_control_assert(rsci->rstc);
	pm_runtime_put(dev);

	return spi_controller_suspend(rsci->ctlr);
}

static int rsci_spi_resume(struct device *dev)
{
	struct rsci_data *rsci = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(rsci->rstc);
	if (ret < 0)
		return ret;

	pm_runtime_get(dev);

	return spi_controller_resume(rsci->ctlr);
}

static SIMPLE_DEV_PM_OPS(rsci_pm_ops, rsci_spi_suspend, rsci_spi_resume);
#define DEV_PM_OPS      (&rsci_pm_ops)
#else
#define DEV_PM_OPS      NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver rsci_spi_driver = {
	.probe	= rsci_spi_probe,
	.remove = rsci_spi_remove,
	.id_table = spi_driver_ids,
	.driver	= {
		.name = "rz_rsci_spi",
		.pm = DEV_PM_OPS,
		.of_match_table = of_match_ptr(rsci_of_match),
	},
};
module_platform_driver(rsci_spi_driver);

MODULE_DESCRIPTION("Renesas SPI RSCI bus driver");
MODULE_LICENSE("GPL v2");
