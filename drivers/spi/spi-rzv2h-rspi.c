// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Renesas RZ/V2H Renesas Serial Peripheral Interface (RSPI)
 *
 * Copyright (C) 2025 Renesas Electronics Corporation
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sh_dma.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>

/* Registers */
#define RSPI_SPDR		0x00
#define RSPI_SPCR		0x08
#define RSPI_SSLP		0x10
#define RSPI_SPBR		0x11
#define RSPI_SPSCR		0x13
#define RSPI_SPCMD		0x14
#define RSPI_SPDCR2		0x44
#define RSPI_SPSR		0x52
#define RSPI_SPSRC		0x6a
#define RSPI_SPFCR		0x6c

/* Register SPCR */
#define RSPI_SPCR_MSTR		BIT(30)
#define RSPI_SPCR_SPRIE		BIT(17)
#define RSPI_SPCR_SPTIE		BIT(20)
#define RSPI_SPCR_SCKASE	BIT(12)
#define RSPI_SPCR_SPE		BIT(0)

/* Register SPBR */
#define RSPI_SPBR_SPR_MIN	0
#define RSPI_SPBR_SPR_MAX	255

/* Register SPCMD */
#define RSPI_SPCMD_SSLA		GENMASK(25, 24)
#define RSPI_SPCMD_SPB		GENMASK(20, 16)
#define RSPI_SPCMD_LSBF		BIT(12)
#define RSPI_SPCMD_SSLKP	BIT(7)
#define RSPI_SPCMD_BRDV		GENMASK(3, 2)
#define RSPI_SPCMD_CPOL		BIT(1)
#define RSPI_SPCMD_CPHA		BIT(0)

#define RSPI_SPCMD_BRDV_MIN	0
#define RSPI_SPCMD_BRDV_MAX	3

/* Register SPDCR2 */
#define RSPI_SPDCR2_TTRG	GENMASK(11, 8)
#define RSPI_SPDCR2_RTRG	GENMASK(3, 0)
#define RSPI_FIFO_SIZE		16

/* Register SPSR */
#define RSPI_SPSR_SPRF		BIT(15)
#define RSPI_SPSR_SPTEF		BIT(13)

/* Register RSPI_SPSRC */
#define RSPI_SPSRC_CLEAR	0xfd80
#define SPSRC_SPRFC		BIT(15)
#define SPSRC_CENDFC		BIT(14)
#define SPSRC_SPTEFC		BIT(13)
#define SPSRC_UDRFC		BIT(12)
#define SPSRC_PERFC		BIT(11)
#define SPSRC_MODFC		BIT(10)
#define SPSRC_OVRFC		BIT(8)
#define SPSRC_SPDRFC		BIT(7)

#define RSPI_RESET_NUM		2
#define RSPI_CLK_NUM		3

struct rzv2h_rspi_priv {
	struct reset_control_bulk_data resets[RSPI_RESET_NUM];
	struct spi_controller *controller;
	void __iomem *base;
	struct clk *tclk;
	wait_queue_head_t wait;
	unsigned int bytes_per_word;
	u32 freq;
	u16 status;
	int rx_irq, tx_irq;
	phys_addr_t phys;

	unsigned dma_callbacked:1;
};

#define RZV2H_RSPI_TX(func, type)					\
static inline void rzv2h_rspi_tx_##type(struct rzv2h_rspi_priv *rspi,	\
					const void *txbuf,		\
					unsigned int index) {		\
	type buf = 0;							\
									\
	if (txbuf)							\
		buf = ((type *)txbuf)[index];				\
									\
	func(buf, rspi->base + RSPI_SPDR);				\
}

#define RZV2H_RSPI_RX(func, type)					\
static inline void rzv2h_rspi_rx_##type(struct rzv2h_rspi_priv *rspi,	\
					void *rxbuf,			\
					unsigned int index) {		\
	type buf = func(rspi->base + RSPI_SPDR);			\
									\
	if (rxbuf)							\
		((type *)rxbuf)[index] = buf;				\
}

RZV2H_RSPI_TX(writel, u32)
RZV2H_RSPI_TX(writew, u16)
RZV2H_RSPI_TX(writeb, u8)
RZV2H_RSPI_RX(readl, u32)
RZV2H_RSPI_RX(readw, u16)
RZV2H_RSPI_RX(readl, u8)

static void rzv2h_rspi_reg_rmw(const struct rzv2h_rspi_priv *rspi,
				int reg_offs, u32 bit_mask, u32 value)
{
	u32 tmp;

	value <<= __ffs(bit_mask);
	tmp = (readl(rspi->base + reg_offs) & ~bit_mask) | value;
	writel(tmp, rspi->base + reg_offs);
}

static inline void rzv2h_rspi_spe_disable(const struct rzv2h_rspi_priv *rspi)
{
	rzv2h_rspi_reg_rmw(rspi, RSPI_SPCR, RSPI_SPCR_SPE, 0);
}

static inline void rzv2h_rspi_spe_enable(const struct rzv2h_rspi_priv *rspi)
{
	rzv2h_rspi_reg_rmw(rspi, RSPI_SPCR, RSPI_SPCR_SPE, 1);
}

static inline void rzv2h_rspi_clear_fifos(const struct rzv2h_rspi_priv *rspi)
{
	writeb(1, rspi->base + RSPI_SPFCR);
}

static inline void rzv2h_rspi_clear_all_irqs(struct rzv2h_rspi_priv *rspi)
{
	writew(RSPI_SPSRC_CLEAR, rspi->base + RSPI_SPSRC);
	rspi->status = 0;
}

static void rspi_enable_irq(const struct rzv2h_rspi_priv *rspi, u32 enable)
{
	writel(readl(rspi->base + RSPI_SPCR) | enable, rspi->base + RSPI_SPCR);
}

static void rspi_disable_irq(const struct rzv2h_rspi_priv *rspi, u32 disable)
{
	writel(readl(rspi->base + RSPI_SPCR) & ~disable, rspi->base + RSPI_SPCR);
}

static inline int rzv2h_rspi_wait_for_interrupt(struct rzv2h_rspi_priv *rspi,
						u16 wait_mask,
						u32 enable_bit)
{
	int ret;

	rspi->status = readw(rspi->base + RSPI_SPSR);
	if (rspi->status & wait_mask)
		return 0;

	rspi_enable_irq(rspi, enable_bit);
	ret = wait_event_timeout(rspi->wait, rspi->status & wait_mask, HZ);
	if (ret == 0 && !(rspi->status & wait_mask))
		return -ETIMEDOUT;

	return 0;
}

static inline int rspi_wait_for_tx_empty(struct rzv2h_rspi_priv *rspi)
{
	return rzv2h_rspi_wait_for_interrupt(rspi, RSPI_SPSR_SPTEF, RSPI_SPCR_SPTIE);
}

static inline int rspi_wait_for_rx_full(struct rzv2h_rspi_priv *rspi)
{
	return rzv2h_rspi_wait_for_interrupt(rspi, RSPI_SPSR_SPRF, RSPI_SPCR_SPRIE);
}

static irqreturn_t rzv2h_rx_irq_handler(int irq, void *data)
{
	struct rzv2h_rspi_priv *rspi = data;
	u16 spsr;

	rspi->status = spsr = readw(rspi->base + RSPI_SPSR);
	if (spsr & RSPI_SPSR_SPRF) {
		rspi_disable_irq(rspi, RSPI_SPCR_SPRIE);
		wake_up(&rspi->wait);
		return IRQ_HANDLED;
	}
	return 0;
}

static irqreturn_t rzv2h_tx_irq_handler(int irq, void *data)
{
	struct rzv2h_rspi_priv *rspi = data;
	u16 spsr;

	rspi->status = spsr = readw(rspi->base + RSPI_SPSR);
	if (spsr & RSPI_SPSR_SPTEF) {
		rspi_disable_irq(rspi, RSPI_SPCR_SPTIE);
		wake_up(&rspi->wait);
		return IRQ_HANDLED;
	}
	return 0;
}

static bool __rspi_can_dma(const struct rzv2h_rspi_priv *rspi,
				const struct spi_transfer *xfer)
{
	return xfer->len > RSPI_FIFO_SIZE;
}

static bool rspi_can_dma(struct spi_controller *controller, struct spi_device *spi,
					struct spi_transfer *xfer)
{
	struct rzv2h_rspi_priv *rspi = spi_controller_get_devdata(controller);

	return __rspi_can_dma(rspi, xfer);
}

static void rspi_dma_complete(void *arg)
{
	struct rzv2h_rspi_priv *rspi = arg;

	rspi->dma_callbacked = 1;
	wake_up_interruptible(&rspi->wait);
}

static int rspi_dma_transfer(struct rzv2h_rspi_priv *rspi, struct sg_table *tx,
						struct sg_table *rx)
{
	struct dma_async_tx_descriptor *desc_tx = NULL, *desc_rx = NULL;
	u32 irq_mask = 0;
	dma_cookie_t cookie;
	int ret;

	/* First prepare and submit the DMA request(s), as this may fail */
	if (rx) {
		desc_rx = dmaengine_prep_slave_sg(rspi->controller->dma_rx, rx->sgl,
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

		irq_mask |= RSPI_SPCR_SPRIE;
	}

	if (tx) {
		desc_tx = dmaengine_prep_slave_sg(rspi->controller->dma_tx, tx->sgl,
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

		irq_mask |= RSPI_SPCR_SPTIE;
	}

	/*
	 * DMAC needs SPxIE, but if SPxIE is set, the IRQ routine will be
	 * called. So, this driver disables the IRQ while DMA transfer.
	 */
	if (tx)
		disable_irq(rspi->tx_irq);
	if (rx)
		disable_irq(rspi->rx_irq);

	rspi_enable_irq(rspi, irq_mask);
	rspi->dma_callbacked = 0;

	/* Now start DMA */
	if (rx)
		dma_async_issue_pending(rspi->controller->dma_rx);
	if (tx) {
		dma_async_issue_pending(rspi->controller->dma_tx);
		writew(SPSRC_SPTEFC | SPSRC_SPRFC, rspi->base + RSPI_SPSRC);
	}
	ret = wait_event_interruptible_timeout(rspi->wait,
					rspi->dma_callbacked, HZ);
	if (ret > 0 && rspi->dma_callbacked) {
		ret = 0;
		if (tx)
			dmaengine_synchronize(rspi->controller->dma_tx);
		if (rx)
			dmaengine_synchronize(rspi->controller->dma_rx);
	} else {
		if (!ret) {
			dev_err(&rspi->controller->dev, "DMA timeout\n");
			ret = -ETIMEDOUT;
		}
		if (tx)
			dmaengine_terminate_sync(rspi->controller->dma_tx);
		if (rx)
			dmaengine_terminate_sync(rspi->controller->dma_rx);
	}

	rspi_disable_irq(rspi, irq_mask);

	if (tx)
		enable_irq(rspi->tx_irq);
	if (rx)
		enable_irq(rspi->rx_irq);

	return ret;

no_dma_tx:
	if (rx)
		dmaengine_terminate_sync(rspi->controller->dma_rx);
no_dma_rx:
	if (ret == -EAGAIN) {
		dev_warn_once(&rspi->controller->dev,
				"DMA not available, falling back to PIO\n");
	}
	return ret;
}

static int rspi_dma_check_then_transfer(struct rzv2h_rspi_priv *rspi,
					struct spi_transfer *xfer)
{
	struct dma_slave_config cfg;
	enum dma_slave_buswidth width;

	if (!rspi->controller->can_dma || !__rspi_can_dma(rspi, xfer))
		return -EAGAIN;

	memset(&cfg, 0, sizeof(cfg));

	if (xfer->bits_per_word == 8)
		width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (xfer->bits_per_word == 16)
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	cfg.dst_addr = rspi->phys + RSPI_SPDR;
	cfg.src_addr = rspi->phys + RSPI_SPDR;
	cfg.dst_addr_width = width;
	cfg.src_addr_width = width;
	cfg.direction = DMA_MEM_TO_DEV;

	dmaengine_slave_config(rspi->controller->dma_tx, &cfg);

	cfg.direction = DMA_DEV_TO_MEM;

	dmaengine_slave_config(rspi->controller->dma_rx, &cfg);
	/* rx_buf can be NULL on RSPI on SH in TX-only Mode */
	return rspi_dma_transfer(rspi, &xfer->tx_sg,
				xfer->rx_buf ? &xfer->rx_sg : NULL);
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

static int rspi_request_dma(struct device *dev, struct spi_controller *controller)
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

	controller->dma_tx = rspi_request_dma_chan(dev, DMA_MEM_TO_DEV, dma_tx_id);
	if (!controller->dma_tx)
		return -ENODEV;

	controller->dma_rx = rspi_request_dma_chan(dev, DMA_DEV_TO_MEM, dma_rx_id);
	if (!controller->dma_rx) {
		dma_release_channel(controller->dma_tx);
		controller->dma_tx = NULL;
		return -ENODEV;
	}

	controller->can_dma = rspi_can_dma;
	dev_info(dev, "DMA available");
	return 0;
}

static void rspi_release_dma(struct spi_controller *controller)
{
	if (controller->dma_tx)
		dma_release_channel(controller->dma_tx);
	if (controller->dma_rx)
		dma_release_channel(controller->dma_rx);
}

static void rzv2h_rspi_send(struct rzv2h_rspi_priv *rspi, const void *txbuf,
			    unsigned int index)
{
	switch (rspi->bytes_per_word) {
	case 4:
		rzv2h_rspi_tx_u32(rspi, txbuf, index);
		break;
	case 2:
		rzv2h_rspi_tx_u16(rspi, txbuf, index);
		break;
	default:
		rzv2h_rspi_tx_u8(rspi, txbuf, index);
	}
}

static int rzv2h_rspi_receive(struct rzv2h_rspi_priv *rspi, void *rxbuf,
			      unsigned int index)
{
	switch (rspi->bytes_per_word) {
	case 4:
		rzv2h_rspi_rx_u32(rspi, rxbuf, index);
		break;
	case 2:
		rzv2h_rspi_rx_u16(rspi, rxbuf, index);
		break;
	default:
		rzv2h_rspi_rx_u8(rspi, rxbuf, index);
	}

	return 0;
}

static int rzv2h_rspi_pio_transfer(struct rzv2h_rspi_priv *rspi,
				   const void *txbuf, void *rxbuf,
				   unsigned int len, u16 error)
{
	unsigned int words = len / rspi->bytes_per_word;
	int ret, count;

	for (count = 0; count < words; count++) {
		if (txbuf) {
			ret = rspi_wait_for_tx_empty(rspi);
			if (ret < 0) {
				dev_err(&rspi->controller->dev, "transmit timeout\n");
				return ret;
			}
			rzv2h_rspi_send(rspi, txbuf, count);
		}
	}

	for (count = 0; count < words; count++) {
		if (rxbuf) {
			ret = rspi_wait_for_rx_full(rspi);
			if (ret < 0) {
				dev_err(&rspi->controller->dev, "receive timeout %d\n", count);
				return ret;
			}

			ret = rzv2h_rspi_receive(rspi, rxbuf, count);
			if (ret) {
				error = SPI_TRANS_FAIL_IO;
				return ret;
			}
		}
	}

	return 0;
}

static int rzv2h_rspi_transfer_one(struct spi_controller *controller,
				  struct spi_device *spi,
				  struct spi_transfer *transfer)
{
	struct rzv2h_rspi_priv *rspi = spi_controller_get_devdata(controller);
	int ret = 0;

	transfer->effective_speed_hz = rspi->freq;

	ret = rspi_dma_check_then_transfer(rspi, transfer);
	if (ret != -EAGAIN)
		return ret;

	ret = rzv2h_rspi_pio_transfer(rspi, transfer->tx_buf,
				      transfer->rx_buf, transfer->len,
				      transfer->error);
	if (ret < 0)
		return ret;

	rzv2h_rspi_clear_all_irqs(rspi);

	spi_finalize_current_transfer(controller);

	return ret;
}

static inline u32 rzv2h_rspi_calc_bitrate(unsigned long tclk_rate, u8 spr,
					  u8 brdv)
{
	return DIV_ROUND_UP(tclk_rate, (2 * (spr + 1) * (1 << brdv)));
}

static u32 rzv2h_rspi_setup_clock(struct rzv2h_rspi_priv *rspi, u32 hz)
{
	unsigned long tclk_rate;
	int spr;
	u8 brdv;

	/*
	 * From the manual:
	 * Bit rate = f(RSPI_n_TCLK)/(2*(n+1)*2^(N))
	 *
	 * Where:
	 * * RSPI_n_TCLK is fixed to 200MHz on V2H
	 * * n = SPR - is RSPI_SPBR.SPR (from 0 to 255)
	 * * N = BRDV - is RSPI_SPCMD.BRDV (from 0 to 3)
	 */
	tclk_rate = clk_get_rate(rspi->tclk);
	for (brdv = RSPI_SPCMD_BRDV_MIN; brdv <= RSPI_SPCMD_BRDV_MAX; brdv++) {
		spr = DIV_ROUND_UP(tclk_rate, hz * (1 << (brdv + 1)));
		spr--;
		if (spr >= RSPI_SPBR_SPR_MIN && spr <= RSPI_SPBR_SPR_MAX)
			goto clock_found;
	}

	return 0;

clock_found:
	rzv2h_rspi_reg_rmw(rspi, RSPI_SPCMD, RSPI_SPCMD_BRDV, brdv);
	writeb(spr, rspi->base + RSPI_SPBR);

	return rzv2h_rspi_calc_bitrate(tclk_rate, spr, brdv);
}

static int rzv2h_rspi_prepare_message(struct spi_controller *ctlr,
				      struct spi_message *message)
{
	struct rzv2h_rspi_priv *rspi = spi_controller_get_devdata(ctlr);
	const struct spi_device *spi = message->spi;
	struct spi_transfer *xfer;
	u32 speed_hz = U32_MAX;
	u8 bits_per_word;
	u32 conf32;
	u16 conf16;

	/* Make sure SPCR.SPE is 0 before amending the configuration */
	rzv2h_rspi_spe_disable(rspi);

	/* Configure the device to work in "host" mode */
	conf32 = RSPI_SPCR_MSTR;

	/* Auto-stop function */
	conf32 |= RSPI_SPCR_SCKASE;

	writel(conf32, rspi->base + RSPI_SPCR);

	/* Use SPCMD0 only */
	writeb(0x0, rspi->base + RSPI_SPSCR);

	/* Setup mode */
	conf32 = FIELD_PREP(RSPI_SPCMD_CPOL, !!(spi->mode & SPI_CPOL));
	conf32 |= FIELD_PREP(RSPI_SPCMD_CPHA, !!(spi->mode & SPI_CPHA));
	conf32 |= FIELD_PREP(RSPI_SPCMD_LSBF, !!(spi->mode & SPI_LSB_FIRST));
	conf32 |= FIELD_PREP(RSPI_SPCMD_SSLKP, 1);
	conf32 |= FIELD_PREP(RSPI_SPCMD_SSLA, spi_get_chipselect(spi, 0));
	writel(conf32, rspi->base + RSPI_SPCMD);
	if (spi->mode & SPI_CS_HIGH)
		writeb(BIT(spi_get_chipselect(spi, 0)), rspi->base + RSPI_SSLP);
	else
		writeb(0, rspi->base + RSPI_SSLP);

	/* Setup FIFO thresholds */
	conf16 = FIELD_PREP(RSPI_SPDCR2_TTRG, RSPI_FIFO_SIZE - 1);
	conf16 |= FIELD_PREP(RSPI_SPDCR2_RTRG, 0);
	writew(conf16, rspi->base + RSPI_SPDCR2);

	rzv2h_rspi_clear_fifos(rspi);

	list_for_each_entry(xfer, &message->transfers, transfer_list) {
		if (!xfer->speed_hz)
			continue;

		speed_hz = min(xfer->speed_hz, speed_hz);
		bits_per_word = xfer->bits_per_word;
	}

	if (speed_hz == U32_MAX)
		return -EINVAL;

	rspi->bytes_per_word = roundup_pow_of_two(BITS_TO_BYTES(bits_per_word));
	rzv2h_rspi_reg_rmw(rspi, RSPI_SPCMD, RSPI_SPCMD_SPB, bits_per_word - 1);

	rspi->freq = rzv2h_rspi_setup_clock(rspi, speed_hz);
	if (!rspi->freq)
		return -EINVAL;

	rzv2h_rspi_spe_enable(rspi);

	return 0;
}

static int rzv2h_rspi_unprepare_message(struct spi_controller *ctlr,
					struct spi_message *message)
{
	struct rzv2h_rspi_priv *rspi = spi_controller_get_devdata(ctlr);

	rzv2h_rspi_spe_disable(rspi);

	return 0;
}

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

static int rzv2h_rspi_probe(struct platform_device *pdev)
{
	struct spi_controller *controller;
	struct device *dev = &pdev->dev;
	struct rzv2h_rspi_priv *rspi;
	struct clk_bulk_data *clks;
	struct resource *res;
	unsigned long tclk_rate;
	int ret, i;

	controller = devm_spi_alloc_host(dev, sizeof(*rspi));
	if (!controller)
		return -ENOMEM;

	rspi = spi_controller_get_devdata(controller);
	platform_set_drvdata(pdev, rspi);

	rspi->controller = controller;

	rspi->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(rspi->base))
		return PTR_ERR(rspi->base);

	rspi->phys = res->start;

	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret != RSPI_CLK_NUM)
		return dev_err_probe(dev, ret >= 0 ? -EINVAL : ret,
				     "cannot get clocks\n");
	for (i = 0; i < RSPI_CLK_NUM; i++) {
		if (!strcmp(clks[i].id, "tclk")) {
			rspi->tclk = clks[i].clk;
			break;
		}
	}

	if (!rspi->tclk)
		return dev_err_probe(dev, -EINVAL, "Failed to get tclk\n");

	tclk_rate = clk_get_rate(rspi->tclk);

	rspi->resets[0].id = "presetn";
	rspi->resets[1].id = "tresetn";
	ret = devm_reset_control_bulk_get_exclusive(dev, RSPI_RESET_NUM,
						    rspi->resets);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get resets\n");


	ret = platform_get_irq_byname(pdev, "rx");
	if (ret < 0) {
		dev_err(dev, "Failed to get RX IRQ\n");
		return ret;

	}

	rspi->rx_irq = ret;

	ret = platform_get_irq_byname(pdev, "tx");
	if (ret < 0) {
		dev_err(dev, "Failed to get TX IRQ\n");
		return ret;
	}

	rspi->tx_irq = ret;

	/* Multi-interrupt mode, only SPRI and SPTI are used */
	ret = rspi_request_irq(dev, rspi->rx_irq, rzv2h_rx_irq_handler,
			       "rx", rspi);
	if (!ret)
		ret = rspi_request_irq(dev, rspi->tx_irq,
				       rzv2h_tx_irq_handler, "tx", rspi);

	if (ret < 0) {
		dev_err(dev, "request_irq error\n");
		goto quit_resets;
	}

	ret = reset_control_bulk_deassert(RSPI_RESET_NUM, rspi->resets);
	if (ret)
		return dev_err_probe(dev, ret, "failed to deassert resets\n");

	init_waitqueue_head(&rspi->wait);

	controller->mode_bits = SPI_CPHA | SPI_CPOL | SPI_CS_HIGH |
				SPI_LSB_FIRST;
	controller->bits_per_word_mask = SPI_BPW_RANGE_MASK(4, 32);
	controller->prepare_message = rzv2h_rspi_prepare_message;
	controller->unprepare_message = rzv2h_rspi_unprepare_message;
	controller->num_chipselect = 4;
	controller->transfer_one = rzv2h_rspi_transfer_one;
	controller->min_speed_hz = rzv2h_rspi_calc_bitrate(tclk_rate,
							   RSPI_SPBR_SPR_MAX,
							   RSPI_SPCMD_BRDV_MAX);
	controller->max_speed_hz = rzv2h_rspi_calc_bitrate(tclk_rate,
							   RSPI_SPBR_SPR_MIN,
							   RSPI_SPCMD_BRDV_MIN);

	device_set_node(&controller->dev, dev_fwnode(dev));

	ret = rspi_request_dma(dev, controller);
	if (ret < 0)
		dev_warn(dev, "DMA not available, using PIO\n");

	ret = spi_register_controller(controller);
	if (ret) {
		dev_err(dev, "register controller failed\n");
		goto quit_resets;
	}

	dev_info(dev, "probed\n");

	return 0;

quit_resets:
	rspi_release_dma(controller);
	reset_control_bulk_assert(RSPI_RESET_NUM, rspi->resets);

	return ret;
}

static int rzv2h_rspi_remove(struct platform_device *pdev)
{
	struct rzv2h_rspi_priv *rspi = platform_get_drvdata(pdev);

	spi_unregister_controller(rspi->controller);

	reset_control_bulk_assert(RSPI_RESET_NUM, rspi->resets);

	return 0;
}

static const struct of_device_id rzv2h_rspi_match[] = {
	{ .compatible = "renesas,r9a09g057-rspi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzv2h_rspi_match);

static struct platform_driver rzv2h_rspi_drv = {
	.probe = rzv2h_rspi_probe,
	.remove = rzv2h_rspi_remove,
	.driver = {
		.name = "rzv2h_rspi",
		.of_match_table = rzv2h_rspi_match,
	},
};
module_platform_driver(rzv2h_rspi_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fabrizio Castro <fabrizio.castro.jz@renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/V2H(P) Serial Peripheral Interface Driver");
