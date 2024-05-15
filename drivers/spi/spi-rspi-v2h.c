// SPDX-License-Identifier: GPL-2.0
/*
 * V2H RSPI driver
 *
 * Copyright (C) 2023 Renesas Electronics Corp.
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
#include <linux/reset.h>
#include <linux/sh_dma.h>
#include <linux/spi/spi.h>
#include <linux/spi/rspi.h>
#include <linux/spinlock.h>

/* V2H register*/
#define RSPI_SPDR		0x00	/* Data Register */
#define RSPI_SPCKD		0x04	/* Clock Delay Register */
#define RSPI_SSLND		0x05	/* Slave Select Negation Delay Register */
#define RSPI_SPND		0x06	/* Next-Access Delay Register */
#define RSPI_MRCKD		0x07	/* ClocK Digital control Register for Master Receive */
#define RSPI_SPCR		0x08	/* Control Register */
#define RSPI_PCRRM		0x0c	/* Control Register for Master Receive only */
#define RSPI_SPDRCR		0x0d	/* Control Register for Received Data Ready Detection */
#define RSPI_SPPCR		0x0e	/* Pin Control Register */
#define RSPI_SPCR2		0x0f	/* Control Register 2 */
#define RSPI_SSLP		0x10	/* Slave Select Polarity Register */
#define RSPI_SPBR		0x11	/* Bit Rate Register */
#define RSPI_SPSCR		0x13	/* Sequence Control Register */
#define RSPI_SPCMD0		0x14	/* Command Register 0 */
#define RSPI_SPCMD(i)		(RSPI_SPCMD0 + 4 * (i))
#define RSPI_SPDCR		0x40	/* Data Control Register */
#define RSPI_SPDCR2		0x44	/* Data Control Register 2 */
#define RSPI_SPSSR		0x51	/* Sequence Status Register */
#define RSPI_SPSR		0x52	/* Status Register */
#define RSPI_SPTFSR		0x58	/* Transfer FIFO Status Register */
#define RSPI_SPRFSR		0x5c	/* Receive FIFO Status Register */
#define RSPI_SPPSR		0x60	/* Poling Register */
#define RSPI_SPSRC		0x6a	/* Status Clear Register */
#define RSPI_SPFCR		0x6c	/* FIFO Clear Register */

/* SPCR - Control Register - V2H */
#define SPCR_SPE		BIT(0)		/* Function Enable */
#define SPCR_SPSCKSEL		(1 << 7)	/* Master Receive Clock Select */
#define SPCR_SPPE		(1 << 8)	/* Parity Enable */
#define SPCR_SPOE		(1 << 9)	/* Parity Mode */
#define SPCR_PTE		(1 << 11)	/* Parity Self-Diagnosis Enable */
#define SPCR_SCKASE		(1 << 12)	/* RSPCK Auto-Stop Function Enable */
#define SPCR_BFDS		(1 << 13)	/* Between Burst Transfer Frames Delay Select */
#define SPCR_MODFEN		(1 << 14)	/* Mode Fault Error Detection Enable */
#define SPCR_SPEIE		(1 << 16)	/* Error Interrupt Enable */
#define SPCR_SPRIE		(1 << 17)	/* Receive Buffer Full Interrupt Enable */
#define SPCR_SPIIE		0x00		/* Idle Interrupt Enable */
#define SPCR_SPDRES		(1 << 19)	/* Receive Data Ready Error Select */
#define SPCR_SPTIE		(1 << 20)	/* Transmit Buffer Empty Interrupt Enable */
#define SPCR_CENDIE		0x00		/* SPI Communication End Interrupt Enable */
#define SPCR_SPMS		(1 << 24)	/* Function Enable */
#define SPCR_SPFRF		(1 << 25)	/* Frame Format Select */
#define SPCR_MSTR		BIT(30)		/* Master/Slave Mode Select */
#define SPCR_BPEN		(1 << 31)	/* Synchronization Circuit Bypass Enable */
#define SPCR_TXMD		BIT(28)

/* SPPCR - Pin Control Register */
#define SPPCR_MOIFE		0x20	/* MOSI Idle Value Fixing Enable */
#define SPPCR_MOIFV		0x10	/* MOSI Idle Fixed Value */
#define SPPCR_SPOM		0x04
#define SPPCR_SPLP2		0x02	/* Loopback Mode 2 (non-inverting) */
#define SPPCR_SPLP		0x01	/* Loopback Mode (inverting) */

#define SPPCR_IO3FV		0x04	/* Single-/Dual-SPI Mode IO3 Output Fixed Value */
#define SPPCR_IO2FV		0x04	/* Single-/Dual-SPI Mode IO2 Output Fixed Value */

/* SPSR - Status Register - V2H*/
#define SPSR_SPRF		BIT(15)		/* Receive Buffer Full Flag */
#define SPSR_SPTEF		BIT(13)		/* Transmit Buffer Empty Flag */
#define SPSR_PERF		BIT(11)		/* Parity Error Flag */
#define SPSR_MODF		BIT(10)		/* Mode Fault Error Flag */
#define SPSR_IDLNF		BIT(9)		/* RSPI Idle Flag */
#define SPSR_OVRF		BIT(8)		/* Overrun Error Flag (RSPI only) */
#define SPSR_CENDF		BIT(14)		/* Communication completed */

/* V2H only */
#define SPDCR_BYSW		0x00	/* Byte Swap Operating Mode Select */
#define SPDCR_SINV		0x10	/* Serial data invert */

 /* SPDCR2 - Data control Register 2 */
#define SPDCR2_TTRG		0x0000	/* Transmission FIFO threshold setting */
#define SPDCR2_RTRG		0x0000	/* Receive FIFO threshold setting */

 /* SPSRC - Status Clear Register */
#define SPSRC_PERFC		BIT(11) /* Clear Parity Error */
#define SPSRC_MODFC		BIT(10) /* CLear Mode Fault Error Flag */
#define SPSRC_OVRFC		BIT(8)	/* Clear Overrun Error Flag */
#define SPSRC_SPDRFC		BIT(7)	/* Clear Data Ready Flag */
#define SPSRC_SPTEFC		BIT(13) /* SPI Transmit Buffer Empty Flag Clear */
#define SPSRC_SPRFC		BIT(15) /* SPI Receive Buffer Full Flag Clear */

 /* SPFCR - FIFO Clear Register */
#define SPFCR_SPFRST		BIT(0)	/* Clear FIFO Register */

 /* SPCKD - Clock Delay Register */
#define SPCKD_SCKDL_MASK	0x07	/* Clock Delay Setting (1-8) */

 /* SSLND - Slave Select Negation Delay Register */
#define SSLND_SLNDL_MASK	0x07	/* SSL Negation Delay Setting (1-8) */

 /* SPND - Next-Access Delay Register */
#define SPND_SPNDL_MASK		0x07	/* Next-Access Delay Setting (1-8) */

/* SSLP - Slave Select Polarity Register */
#define SSLP_SSLP(i)		BIT(i)	/* SSLi Signal Polarity Setting */

 /* SPCR2 - Control Register 2 */
#define SPCR2_PTE		0x08	/* Parity Self-Test Enable */
#define SPCR2_SPIE		0x04	/* Idle Interrupt Enable */
#define SPCR2_SPOE		0x02	/* Odd Parity Enable (vs. Even) */
#define SPCR2_SPPE		0x01	/* Parity Enable */

/* SPCMDn - Command Registers V2H */
#define SPCMD_SCKDEN		0x00008000	/* Clock Delay Setting Enable */
#define SPCMD_SLNDEN		0x00004000	/* SSL Negation Delay Setting Enable */
#define SPCMD_SPNDEN		0x00002000	/* Next-Access Delay Enable */
#define SPCMD_LSBF		0x00001000	/* LSB First */
#define SPCMD_SPB_MASK		GENMASK(20, 16)
#define SPCMD_SPB_8BIT		0x00070000
#define SPCMD_SPB_16BIT		0x000F0000
#define SPCMD_SPB_32BIT		0x001F0000
#define SPCMD_SSLKP		0x00000080	/* SSL Signal Level Keeping */
#define SPCMD_CPOL		0x00000002	/* Clock Polarity Setting */
#define SPCMD_CPHA		0x00000001	/* Clock Phase Setting */
#define SPCMD_BRDV(brdv)	((brdv) << 2)
#define SPCMD_SSLA(i)		((i) << 24)	/* SSL Assert Signal Setting */

struct rspi_data {
	void __iomem *addr;
	u32 speed_hz;
	struct spi_controller *ctlr;
	struct platform_device *pdev;
	wait_queue_head_t wait;
	spinlock_t lock;	/* Protects RMW-access to RSPI_SSLP */
	struct clk *clk;
	u32 spcmd;
	u16 spsr;
	u8 sppcr;
	int rx_irq, tx_irq;
	int bits_per_word;
	const struct spi_ops *ops;

	unsigned dma_callbacked:1;
	unsigned byte_access:1;
};

static void rspi_write8(const struct rspi_data *rspi, u8 data, u16 offset)
{
	iowrite8(data, rspi->addr + offset);
}

static void rspi_write16(const struct rspi_data *rspi, u16 data, u16 offset)
{
	iowrite16(data, rspi->addr + offset);
}

static void rspi_write32(const struct rspi_data *rspi, u32 data, u16 offset)
{
	iowrite32(data, rspi->addr + offset);
}

static u8 rspi_read8(const struct rspi_data *rspi, u16 offset)
{
	return ioread8(rspi->addr + offset);
}

static u16 rspi_read16(const struct rspi_data *rspi, u16 offset)
{
	return ioread16(rspi->addr + offset);
}

static u32 rspi_read32(const struct rspi_data *rspi, u16 offset)
{
	return ioread32(rspi->addr + offset);
}

static u16 rspi_read_data(const struct rspi_data *rspi)
{
	if (rspi->bits_per_word == 8)
		return rspi_read8(rspi, RSPI_SPDR);
	else if (rspi->bits_per_word == 16)
		return rspi_read16(rspi, RSPI_SPDR);
	else
		return rspi_read32(rspi, RSPI_SPDR);
}

/* Optional functions */
struct spi_ops {
	int (*set_config_register)(struct rspi_data *rspi, int access_size);
	int (*transfer_one)(struct spi_controller *ctlr,
				struct spi_device *spi, struct spi_transfer *xfer);
	u16 extra_mode_bits;
	u16 min_div;
	u16 max_div;
	u16 flags;
	u16 fifo_size;
	u8 num_hw_ss;
};

static void rspi_set_rate(struct rspi_data *rspi)
{
	unsigned long clksrc;
	int brdv = 0, spbr;

	if (!spi_controller_is_slave(rspi->ctlr)) {
		clksrc = clk_get_rate(rspi->clk);
		spbr = DIV_ROUND_UP(clksrc, 2 * rspi->speed_hz) - 1;
		while (spbr > 255 && brdv < 3) {
			brdv++;
			spbr = DIV_ROUND_UP(spbr + 1, 2) - 1;
		}

		rspi_write8(rspi, clamp(spbr, 0, 255), RSPI_SPBR);
		rspi->spcmd |= SPCMD_BRDV(brdv);
		rspi->speed_hz = DIV_ROUND_UP(clksrc, (2U << brdv) * (spbr + 1));
	}
}

static int rspi_v2h_set_config_register(struct rspi_data *rspi, int access_size)
{
	/* Sets output mode, MOSI signal, and (optionally) loopback */
	rspi_write8(rspi, rspi->sppcr, RSPI_SPPCR);

	/* Sets transfer bit rate */
	rspi_set_rate(rspi);

	/* Sets data control */
	rspi_write16(rspi, 0, RSPI_SPDCR);

	/* Sets RSPCK, SSL, next-access delay value */
	rspi_write8(rspi, 0x00, RSPI_SPCKD);
	rspi_write8(rspi, 0x00, RSPI_SSLND);
	rspi_write8(rspi, 0x00, RSPI_SPND);

	/* Resets sequencer */
	rspi_write8(rspi, 0, RSPI_SPSCR);
	rspi->spcmd &= ~SPCMD_SPB_MASK;
	if (rspi->bits_per_word == 8)
		rspi->spcmd |= SPCMD_SPB_8BIT;
	else if (rspi->bits_per_word == 16)
		rspi->spcmd |= SPCMD_SPB_16BIT;
	else
		rspi->spcmd |= SPCMD_SPB_32BIT;
	rspi_write32(rspi, rspi->spcmd, RSPI_SPCMD0);

	/* Sets RSPI mode */
	if (!spi_controller_is_slave(rspi->ctlr))
		rspi_write32(rspi, SPCR_MSTR, RSPI_SPCR);
	return 0;
}

static void rspi_enable_irq(const struct rspi_data *rspi, u32 enable)
{
	rspi_write32(rspi, rspi_read32(rspi, RSPI_SPCR) | enable, RSPI_SPCR);
}

static void rspi_disable_irq(const struct rspi_data *rspi, u32 disable)
{
	rspi_write32(rspi, rspi_read32(rspi, RSPI_SPCR) & ~disable, RSPI_SPCR);
}

static int rspi_wait_for_interrupt(struct rspi_data *rspi, u16 wait_mask,
							u32 enable_bit)
{
	int ret;

	rspi->spsr = rspi_read16(rspi, RSPI_SPSR);
	if (rspi->spsr & wait_mask)
		return 0;
	rspi_enable_irq(rspi, enable_bit);
	ret = wait_event_timeout(rspi->wait, rspi->spsr & wait_mask, 10 * HZ);
	if (ret == 0 && !(rspi->spsr & wait_mask))
		return -ETIMEDOUT;
	return 0;
}

static inline int rspi_wait_for_tx_empty(struct rspi_data *rspi)
{
	return rspi_wait_for_interrupt(rspi, SPSR_SPTEF, SPCR_SPTIE);
}

static inline int rspi_wait_for_rx_full(struct rspi_data *rspi)
{
	return rspi_wait_for_interrupt(rspi, SPSR_SPRF, SPCR_SPRIE);
}

static void rspi_data_out_8(struct rspi_data *rspi, const void *tx, int count)
{
	const u8 *buf_8 = tx;

	rspi_write8(rspi, buf_8[count], RSPI_SPDR);
}

static void rspi_data_out_16(struct rspi_data *rspi, const void *tx, int count)
{
	const u16 *buf_16 = tx;

	rspi_write16(rspi, buf_16[count], RSPI_SPDR);
}

static void rspi_data_out_32(struct rspi_data *rspi, const void *tx, int count)
{
	const u32 *buf_32 = tx;

	rspi_write32(rspi, buf_32[count], RSPI_SPDR);
}

static void rspi_data_in_8(struct rspi_data *rspi, void *rx, int count)
{
	u8 *buf_8 = rx;

	buf_8[count] = rspi_read8(rspi, RSPI_SPDR);
}

static void rspi_data_in_16(struct rspi_data *rspi, void *rx, int count)
{
	u16 *buf_16 = rx;

	buf_16[count] = rspi_read16(rspi, RSPI_SPDR);
}

static void rspi_data_in_32(struct rspi_data *rspi, void *rx, int count)
{
	u32 *buf_32 = rx;

	buf_32[count] = rspi_read32(rspi, RSPI_SPDR);
}

static int rspi_pio_transfer(struct rspi_data *rspi, const void *tx, void *rx,
								unsigned int n)
{
	int words = n / (rspi->bits_per_word / 8);
	void (*tx_fifo)(struct rspi_data *rspi, const void *tx, int count);
	void (*rx_fifo)(struct rspi_data *rspi, void *rx, int count);
	int ret, count;

	switch (rspi->bits_per_word) {
	case 8:
		tx_fifo = rspi_data_out_8;
		rx_fifo = rspi_data_in_8;
		break;
	case 16:
		tx_fifo = rspi_data_out_16;
		rx_fifo = rspi_data_in_16;
		break;
	case 32:
		tx_fifo = rspi_data_out_32;
		rx_fifo = rspi_data_in_32;
		break;
	default:
		return -EINVAL;
	}
	for (count = 0; count < words; count++) {
		if (tx) {
			ret = rspi_wait_for_tx_empty(rspi);
			if (ret < 0) {
				dev_err(&rspi->ctlr->dev, "transmit timeout\n");
				return ret;
			}
			tx_fifo(rspi, tx, count);
		}
	}

	for (count = 0; count < words; count++) {
		if (rx) {
			ret = rspi_wait_for_rx_full(rspi);
			if (ret < 0) {
				dev_err(&rspi->ctlr->dev, "receive timeout %d\n", count);
				return ret;
			}
			rx_fifo(rspi, rx, count);
		}
	}
	return 0;
}

static void rspi_dma_complete(void *arg)
{
	struct rspi_data *rspi = arg;

	rspi->dma_callbacked = 1;
	wake_up_interruptible(&rspi->wait);
}

static int rspi_dma_transfer(struct rspi_data *rspi, struct sg_table *tx,
						struct sg_table *rx)
{
	struct dma_async_tx_descriptor *desc_tx = NULL, *desc_rx = NULL;
	u32 irq_mask = 0;
	unsigned int other_irq = 0;
	dma_cookie_t cookie;
	int ret;

	/* First prepare and submit the DMA request(s), as this may fail */
	if (rx) {
		desc_rx = dmaengine_prep_slave_sg(rspi->ctlr->dma_rx, rx->sgl,
					rx->nents, DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_rx) {
			ret = -EAGAIN;
			goto no_dma_rx;
		}
		desc_rx->callback = rspi_dma_complete;
		desc_rx->callback_param = rspi;
		cookie = dmaengine_submit(desc_rx);
		if (dma_submit_error(cookie)) {
			ret = cookie;
			goto no_dma_rx;
		}

		irq_mask |= SPCR_SPRIE;
	}

	if (tx) {
		desc_tx = dmaengine_prep_slave_sg(rspi->ctlr->dma_tx, tx->sgl,
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
			desc_tx->callback = rspi_dma_complete;
			desc_tx->callback_param = rspi;
		}
		cookie = dmaengine_submit(desc_tx);
		if (dma_submit_error(cookie)) {
			ret = cookie;
			goto no_dma_tx;
		}

		irq_mask |= SPCR_SPTIE;
	}

	/*
	 * DMAC needs SPxIE, but if SPxIE is set, the IRQ routine will be
	 * called. So, this driver disables the IRQ while DMA transfer.
	 */
	if (tx)
		disable_irq(other_irq = rspi->tx_irq);
	if (rx && rspi->rx_irq != other_irq)
		disable_irq(rspi->rx_irq);

	rspi_enable_irq(rspi, irq_mask);
	rspi->dma_callbacked = 0;

	/* Now start DMA */
	if (rx)
		dma_async_issue_pending(rspi->ctlr->dma_rx);
	if (tx) {
		dma_async_issue_pending(rspi->ctlr->dma_tx);
		rspi_write16(rspi, SPSRC_SPTEFC | SPSRC_SPRFC, RSPI_SPSRC);
	}
	ret = wait_event_interruptible_timeout(rspi->wait,
					rspi->dma_callbacked, HZ);
	if (rspi->dma_callbacked) {
		ret = 0;
		if (tx)
			dmaengine_synchronize(rspi->ctlr->dma_tx);
		if (rx)
			dmaengine_synchronize(rspi->ctlr->dma_rx);
	} else {
		if (!ret) {
			dev_err(&rspi->ctlr->dev, "DMA timeout\n");
			ret = -ETIMEDOUT;
		}
		if (tx)
			dmaengine_terminate_sync(rspi->ctlr->dma_tx);
		if (rx)
			dmaengine_terminate_sync(rspi->ctlr->dma_rx);
	}

	rspi_disable_irq(rspi, irq_mask);

	if (tx)
		enable_irq(rspi->tx_irq);
	if (rx && rspi->rx_irq != other_irq)
		enable_irq(rspi->rx_irq);
	return ret;

no_dma_tx:
	if (rx)
		dmaengine_terminate_sync(rspi->ctlr->dma_rx);
no_dma_rx:
	if (ret == -EAGAIN) {
		dev_warn_once(&rspi->ctlr->dev,
				"DMA not available, falling back to PIO\n");
	}
	return ret;
}

static void rspi_receive_init(const struct rspi_data *rspi)
{
	u16 spsr;

	spsr = rspi_read16(rspi, RSPI_SPSR);
	if (spsr & SPSR_SPRF)
		rspi_read_data(rspi);	/* dummy read */
	if (spsr & SPSR_OVRF)
		rspi_write16(rspi, rspi_read16(rspi, RSPI_SPSR) & ~SPSR_OVRF,
								RSPI_SPSR);
}

static void rspi_v2h_receive_init(const struct rspi_data *rspi)
{
	rspi_receive_init(rspi);
}

static bool __rspi_can_dma(const struct rspi_data *rspi,
				const struct spi_transfer *xfer)
{
	return xfer->len > rspi->ops->fifo_size;
}

static bool rspi_can_dma(struct spi_controller *ctlr, struct spi_device *spi,
					struct spi_transfer *xfer)
{
	struct rspi_data *rspi = spi_controller_get_devdata(ctlr);

	return __rspi_can_dma(rspi, xfer);
}

static int rspi_dma_check_then_transfer(struct rspi_data *rspi,
					struct spi_transfer *xfer)
{
	struct dma_slave_config cfg;
	enum dma_slave_buswidth width;

	if (!rspi->ctlr->can_dma || !__rspi_can_dma(rspi, xfer))
		return -EAGAIN;

	memset(&cfg, 0, sizeof(cfg));

	if (rspi->bits_per_word == 8)
		width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (rspi->bits_per_word == 16)
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr = rspi->pdev->resource->start + RSPI_SPDR;
	cfg.src_addr = rspi->pdev->resource->start + RSPI_SPDR;
	cfg.dst_addr_width = width;
	cfg.src_addr_width = width;
	cfg.direction = DMA_MEM_TO_DEV;

	dmaengine_slave_config(rspi->ctlr->dma_tx, &cfg);

	cfg.direction = DMA_DEV_TO_MEM;

	dmaengine_slave_config(rspi->ctlr->dma_rx, &cfg);
	/* rx_buf can be NULL on RSPI on SH in TX-only Mode */
	return rspi_dma_transfer(rspi, &xfer->tx_sg,
				xfer->rx_buf ? &xfer->rx_sg : NULL);
}

static int rspi_common_transfer(struct rspi_data *rspi,
					struct spi_transfer *xfer)
{
	int ret;

	xfer->effective_speed_hz = rspi->speed_hz;

	ret = rspi_dma_check_then_transfer(rspi, xfer);
	if (ret != -EAGAIN)
		return ret;

	ret = rspi_pio_transfer(rspi, xfer->tx_buf, xfer->rx_buf, xfer->len);
	if (ret < 0)
		return ret;

	/* Wait for the last transmission */
	rspi_wait_for_tx_empty(rspi);
	return 0;
}

static int rspi_v2h_transfer_one(struct spi_controller *ctlr,
				struct spi_device *spi,
				struct spi_transfer *xfer)
{
	struct rspi_data *rspi = spi_controller_get_devdata(ctlr);

	rspi_v2h_receive_init(rspi);
	return rspi_common_transfer(rspi, xfer);
}

static int rspi_setup(struct spi_device *spi)
{
	struct rspi_data *rspi = spi_controller_get_devdata(spi->controller);
	u8 sslp;

	if (spi->cs_gpiod)
		return 0;

	pm_runtime_get_sync(&rspi->pdev->dev);
	spin_lock_irq(&rspi->lock);

	sslp = rspi_read8(rspi, RSPI_SSLP);
	if (spi->mode & SPI_CS_HIGH)
		sslp |= SSLP_SSLP(spi->chip_select);
	else
		sslp &= ~SSLP_SSLP(spi->chip_select);

	rspi_write8(rspi, sslp, RSPI_SSLP);

	spin_unlock_irq(&rspi->lock);
	pm_runtime_put(&rspi->pdev->dev);
	return 0;
}

static int rspi_prepare_message(struct spi_controller *ctlr,
					struct spi_message *msg)
{
	struct rspi_data *rspi = spi_controller_get_devdata(ctlr);
	struct spi_device *spi = msg->spi;
	const struct spi_transfer *xfer;

	/*
	 * As the Bit Rate Register must not be changed while the device is
	 * active, all transfers in a message must use the same bit rate.
	 * In theory, the sequencer could be enabled, and each Command Register
	 * could divide the base bit rate by a different value.
	 * However, most RSPI variants do not have Transfer Data Length
	 * Multiplier Setting Registers, so each sequence step would be limited
	 * to a single word, making this feature unsuitable for large
	 * transfers, which would gain most from it.
	 */
	rspi->speed_hz = spi->max_speed_hz;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (xfer->speed_hz < rspi->speed_hz)
			rspi->speed_hz = xfer->speed_hz;
		rspi->bits_per_word = xfer->bits_per_word;
	}

	if (!spi_controller_is_slave(rspi->ctlr))
		rspi->spcmd = SPCMD_SSLKP;
	if (spi->mode & SPI_CPOL)
		rspi->spcmd |= SPCMD_CPOL;
	if (spi->mode & SPI_CPHA)
		rspi->spcmd |= SPCMD_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		rspi->spcmd |= SPCMD_LSBF;

	/* Configure slave signal to assert */
	rspi->spcmd |= SPCMD_SSLA(spi->cs_gpiod ? rspi->ctlr->unused_native_cs
							: spi->chip_select);

	/* CMOS output mode and MOSI signal from previous transfer */
	rspi->sppcr = 0;
	if (spi->mode & SPI_LOOP)
		rspi->sppcr |= SPPCR_SPLP;

	rspi->ops->set_config_register(rspi, rspi->bits_per_word);

	/* Clear Error Sources */
	rspi_write16(rspi, rspi_read16(rspi, RSPI_SPSRC) | SPSRC_PERFC, RSPI_SPSRC);
	rspi_write16(rspi, rspi_read16(rspi, RSPI_SPSRC) | SPSRC_MODFC, RSPI_SPSRC);
	rspi_write16(rspi, rspi_read16(rspi, RSPI_SPSRC) | SPSRC_OVRFC, RSPI_SPSRC);
	rspi_write16(rspi, rspi_read16(rspi, RSPI_SPSRC) | SPSRC_SPDRFC, RSPI_SPSRC);

	/* FIFO Clear */
	rspi_write16(rspi, SPFCR_SPFRST, RSPI_SPFCR);

	/* Prohibit SPII and SPCEND interrupt */

	rspi_write32(rspi, rspi_read32(rspi, RSPI_SPCR) | SPCR_SPIIE, RSPI_SPCR);
	rspi_write32(rspi, rspi_read32(rspi, RSPI_SPCR) | SPCR_CENDIE, RSPI_SPCR);

	/* Enable SPI function in master mode */
	rspi_write32(rspi, rspi_read32(rspi, RSPI_SPCR) | SPCR_SPE, RSPI_SPCR);
	return 0;
}

static int rspi_unprepare_message(struct spi_controller *ctlr,
						struct spi_message *msg)
{
	struct rspi_data *rspi = spi_controller_get_devdata(ctlr);

	/* Disable SPI function */
	rspi_write32(rspi, rspi_read32(rspi, RSPI_SPCR) & ~SPCR_SPE, RSPI_SPCR);

	/* Reset sequencer for Single SPI Transfers */

	rspi_write32(rspi, rspi->spcmd, RSPI_SPCMD0);
	rspi_write8(rspi, 0, RSPI_SPSCR);

	return 0;
}

static irqreturn_t rspi_irq_mux(int irq, void *_sr)
{
	struct rspi_data *rspi = _sr;
	u16 spsr;
	irqreturn_t ret = IRQ_NONE;
	u32 disable_irq = 0;

	rspi->spsr = spsr = rspi_read16(rspi, RSPI_SPSR);
	if (spsr & SPSR_SPRF)
		disable_irq |= SPCR_SPRIE;
	if (spsr & SPSR_SPTEF)
		disable_irq |= SPCR_SPTIE;

	if (disable_irq) {
		ret = IRQ_HANDLED;
		rspi_disable_irq(rspi, disable_irq);
		wake_up(&rspi->wait);
	}
	return ret;
}

static irqreturn_t rspi_irq_rx(int irq, void *_sr)
{
	struct rspi_data *rspi = _sr;
	u16 spsr;

	rspi->spsr = spsr = rspi_read16(rspi, RSPI_SPSR);
	if (spsr & SPSR_SPRF) {
		rspi_disable_irq(rspi, SPCR_SPRIE);
		wake_up(&rspi->wait);
		return IRQ_HANDLED;
	}
	return 0;
}

static irqreturn_t rspi_irq_tx(int irq, void *_sr)
{
	struct rspi_data *rspi = _sr;
	u16 spsr;

	rspi->spsr = spsr = rspi_read16(rspi, RSPI_SPSR);
	if (spsr & SPSR_SPTEF) {
		rspi_disable_irq(rspi, SPCR_SPTIE);
		wake_up(&rspi->wait);
		return IRQ_HANDLED;
	}
	return 0;
}

static struct dma_chan *rspi_request_dma_chan(struct device *dev,
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

static int rspi_request_dma(struct device *dev, struct spi_controller *ctlr)
{
	const struct rspi_plat_data *rspi_pd = dev_get_platdata(dev);
	unsigned int dma_tx_id, dma_rx_id;

	if (dev->of_node) {
		/* In the OF case we will get the slave IDs from the DT */
		dma_tx_id = 0;
		dma_rx_id = 0;
	} else if (rspi_pd && rspi_pd->dma_tx_id && rspi_pd->dma_rx_id) {
		dma_tx_id = rspi_pd->dma_tx_id;
		dma_rx_id = rspi_pd->dma_rx_id;
	} else {
		/* The driver assumes no error. */
		return 0;
	}

	ctlr->dma_tx = rspi_request_dma_chan(dev, DMA_MEM_TO_DEV, dma_tx_id);
	if (!ctlr->dma_tx)
		return -ENODEV;

	ctlr->dma_rx = rspi_request_dma_chan(dev, DMA_DEV_TO_MEM, dma_rx_id);
	if (!ctlr->dma_rx) {
		dma_release_channel(ctlr->dma_tx);
		ctlr->dma_tx = NULL;
		return -ENODEV;
	}

	ctlr->can_dma = rspi_can_dma;
	dev_info(dev, "DMA available");
	return 0;
}

static void rspi_release_dma(struct spi_controller *ctlr)
{
	if (ctlr->dma_tx)
		dma_release_channel(ctlr->dma_tx);
	if (ctlr->dma_rx)
		dma_release_channel(ctlr->dma_rx);
}

static int rspi_remove(struct platform_device *pdev)
{
	struct rspi_data *rspi = platform_get_drvdata(pdev);

	rspi_release_dma(rspi->ctlr);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct spi_ops rspi_v2h_ops = {
	.set_config_register	=	rspi_v2h_set_config_register,
	.transfer_one		=	rspi_v2h_transfer_one,
	.min_div		=	2,
	.max_div		=	4096,
	.flags			=	SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX,
	.fifo_size		=	8,	/* 8 for TX, 32 for RX */
	.num_hw_ss		=	1,
};

#ifdef CONFIG_OF
static const struct of_device_id rspi_of_match[] = {
	/* RSPI on V2H */
	{ .compatible = "renesas,rspi-v2h", .data = &rspi_v2h_ops },
	{ .compatible = "renesas,rspi-g3e", .data = &rspi_v2h_ops },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, rspi_of_match);

static void rspi_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static int rspi_mode(struct device *dev)
{
	return of_property_read_bool(dev->of_node, "spi-slave") ? RSPI_SPI_SLAVE
								: RSPI_SPI_MASTER;
}

static int rspi_parse_dt(struct device *dev, struct spi_controller *ctlr)
{
	struct reset_control *rstc;
	u32 num_cs;
	int error;

	/* Parse DT properties */
	error = of_property_read_u32(dev->of_node, "num-cs", &num_cs);
	if (error) {
		dev_err(dev, "of_property_read_u32 num-cs failed %d\n", error);
		return error;
	}

	ctlr->num_chipselect = num_cs;

	rstc = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc),
						"failed to get reset ctrl\n");

	error = reset_control_deassert(rstc);
	if (error) {
		dev_err(dev, "failed to deassert reset %d\n", error);
		return error;
	}

	error = devm_add_action_or_reset(dev, rspi_reset_control_assert, rstc);
	if (error) {
		dev_err(dev, "failed to register assert devm action, %d\n", error);
		return error;
	}

	return 0;
}
#else
#define rspi_of_match	NULL
static inline int rspi_parse_dt(struct device *dev, struct spi_controller *ctlr)
{
	return -EINVAL;
}
#endif /* CONFIG_OF */

static int rspi_request_irq(struct device *dev, unsigned int irq,
				irq_handler_t handler, const char *suffix,
							void *dev_id)
{
	const char *name = devm_kasprintf(dev, GFP_KERNEL, "%s:%s",
						dev_name(dev), suffix);
	if (!name)
		return -ENOMEM;

	return devm_request_irq(dev, irq, handler, 0, name, dev_id);
}

static int rspi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_controller *ctlr;
	struct rspi_data *rspi;
	int ret;
	const struct rspi_plat_data *rspi_pd;
	const struct spi_ops *ops;
	unsigned long clksrc;

	ret = rspi_mode(&pdev->dev);
	if (ret == RSPI_SPI_MASTER)
		ctlr = spi_alloc_master(&pdev->dev, sizeof(struct rspi_data));
	else
		ctlr = spi_alloc_slave(&pdev->dev, sizeof(struct rspi_data));
	if (ctlr == NULL)
		return -ENOMEM;
	ops = of_device_get_match_data(&pdev->dev);
	if (ops) {
		ret = rspi_parse_dt(&pdev->dev, ctlr);
		if (ret)
			goto error1;
	} else {
		ops = (struct spi_ops *)pdev->id_entry->driver_data;
		rspi_pd = dev_get_platdata(&pdev->dev);
		if (rspi_pd && rspi_pd->num_chipselect)
			ctlr->num_chipselect = rspi_pd->num_chipselect;
		else
			ctlr->num_chipselect = 2; /* default */
	}

	rspi = spi_controller_get_devdata(ctlr);
	platform_set_drvdata(pdev, rspi);
	rspi->ops = ops;
	rspi->ctlr = ctlr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rspi->addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rspi->addr)) {
		ret = PTR_ERR(rspi->addr);
		goto error1;
	}

	rspi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(rspi->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = PTR_ERR(rspi->clk);
		goto error1;
	}

	rspi->pdev = pdev;
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_resume_and_get(&pdev->dev);

	init_waitqueue_head(&rspi->wait);
	spin_lock_init(&rspi->lock);

	ctlr->bus_num = pdev->id;
	ctlr->setup = rspi_setup;
	ctlr->auto_runtime_pm = true;
	ctlr->transfer_one = ops->transfer_one;
	ctlr->prepare_message = rspi_prepare_message;
	ctlr->unprepare_message = rspi_unprepare_message;
	ctlr->mode_bits = SPI_CPHA | SPI_CPOL | SPI_CS_HIGH | SPI_LSB_FIRST |
						SPI_LOOP | ops->extra_mode_bits;
	clksrc = clk_get_rate(rspi->clk);
	ctlr->min_speed_hz = DIV_ROUND_UP(clksrc, ops->max_div);
	ctlr->max_speed_hz = DIV_ROUND_UP(clksrc, ops->min_div);
	ctlr->flags = ops->flags;
	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->use_gpio_descriptors = true;
	ctlr->max_native_cs = rspi->ops->num_hw_ss;
	ret = platform_get_irq_byname_optional(pdev, "rx");

	if (ret < 0) {
		ret = platform_get_irq_byname_optional(pdev, "mux");
		if (ret < 0)
			ret = platform_get_irq(pdev, 0);
		if (ret >= 0)
			rspi->rx_irq = rspi->tx_irq = ret;
	} else {
		rspi->rx_irq = ret;
		ret = platform_get_irq_byname(pdev, "tx");
		if (ret >= 0)
			rspi->tx_irq = ret;
	}

	if (rspi->rx_irq == rspi->tx_irq) {
		/* Single multiplexed interrupt */
		ret = rspi_request_irq(&pdev->dev, rspi->rx_irq, rspi_irq_mux,
				"mux", rspi);
	} else {
		/* Multi-interrupt mode, only SPRI and SPTI are used */
		ret = rspi_request_irq(&pdev->dev, rspi->rx_irq, rspi_irq_rx,
				"rx", rspi);
		if (!ret)
			ret = rspi_request_irq(&pdev->dev, rspi->tx_irq,
					rspi_irq_tx, "tx", rspi);
	}
	if (ret < 0) {
		dev_err(&pdev->dev, "request_irq error\n");
		goto error2;
	}

	ret = rspi_request_dma(&pdev->dev, ctlr);
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
	rspi_release_dma(ctlr);
error2:
	pm_runtime_disable(&pdev->dev);
error1:
	spi_controller_put(ctlr);

	return ret;
}

static const struct platform_device_id spi_driver_ids[] = {
	{ "rspi", (kernel_ulong_t)&rspi_v2h_ops },
	{},
};

MODULE_DEVICE_TABLE(platform, spi_driver_ids);

#ifdef CONFIG_PM_SLEEP
static int rspi_suspend(struct device *dev)
{
	struct rspi_data *rspi = dev_get_drvdata(dev);

	return spi_controller_suspend(rspi->ctlr);
}

static int rspi_resume(struct device *dev)
{
	struct rspi_data *rspi = dev_get_drvdata(dev);

	return spi_controller_resume(rspi->ctlr);
}

static SIMPLE_DEV_PM_OPS(rspi_pm_ops, rspi_suspend, rspi_resume);
#define DEV_PM_OPS      (&rspi_pm_ops)
#else
#define DEV_PM_OPS      NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver rspi_driver = {
	.probe	= rspi_probe,
	.remove = rspi_remove,
	.id_table = spi_driver_ids,
	.driver	= {
		.name = "renesas_spi_v2h",
		.pm = DEV_PM_OPS,
		.of_match_table = of_match_ptr(rspi_of_match),
	},
};
module_platform_driver(rspi_driver);

MODULE_DESCRIPTION("Renesas RSPI bus driver");
MODULE_LICENSE("GPL v2");
