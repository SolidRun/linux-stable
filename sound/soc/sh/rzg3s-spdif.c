// SPDX-License-Identifier: GPL-2.0
//
// Renesas RZ/G3S ASoC Serial Sound Interface (SPDIF) Driver
//
// Copyright (C) 2023 Renesas Electronics Corp.
//

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/dmaengine.h>
#include <sound/asoundef.h>
#include <linux/dma-mapping.h>

/* Register offset	*/
#define SPDIF_TLCA		0x00
#define SPDIF_TRCA		0x04
#define SPDIF_TLCS		0x08
#define SPDIF_TRCS		0x0c
#define SPDIF_TUI		0x10
#define SPDIF_RLCA		0x14
#define SPDIF_RRCA		0x18
#define SPDIF_RLCS		0x1c
#define SPDIF_RRCS		0x20
#define SPDIF_RUI		0x24
#define SPDIF_CTRL		0x28
#define SPDIF_STAT		0x2c
#define SPDIF_TDAD		0x30
#define SPDIF_RDAD		0x34

/* Stat bits		 */
#define SPDIF_RIS_BIT		(0x00008000)
#define SPDIF_TIS_BIT		(0x00004000)
#define SPDIF_UBO_BIT		(0x00002000)
#define SPDIF_UBU_BIT		(0x00001000)
#define SPDIF_CE_BIT		(0x00000800)
#define SPDIF_PARE_BIT		(0x00000400)
#define SPDIF_PREE_BIT		(0x00000200)
#define SPDIF_CSE_BIT		(0x00000100)
#define SPDIF_ABO_BIT		(0x00000080)
#define SPDIF_ABU_BIT		(0x00000040)
#define SPDIF_RUIR_BIT		(0x00000020)
#define SPDIF_TUIR_BIT		(0x00000010)
#define SPDIF_CSRX_BIT		(0x00000008)
#define SPDIF_CBRX_BIT		(0x00000004)
#define SPDIF_CSTX_BIT		(0x00000002)
#define SPDIF_CBTX_BIT		(0x00000001)

/* Ctrl bits		*/
#define SPDIF_PB_BIT		(0x04000000)
#define SPDIF_RDE_BIT		(0x00200000)
#define SPDIF_TDE_BIT		(0x00100000)
#define SPDIF_NCSI_BIT		(0x00080000)
#define SPDIF_AOS_BIT		(0x00040000)
#define SPDIF_RME_BIT		(0x00020000)
#define SPDIF_TME_BIT		(0x00010000)
#define SPDIF_REIE_BIT		(0x00008000)
#define SPDIF_TEIE_BIT		(0x00004000)
#define SPDIF_UBOI_BIT		(SPDIF_UBO_BIT)
#define SPDIF_UBUI_BIT		(SPDIF_UBU_BIT)
#define SPDIF_CREI_BIT		(0x00000800)
#define SPDIF_PAEI_BIT		(0x00000400)
#define SPDIF_PREI_BIT		(0x00000200)
#define SPDIF_ABOI_BIT		(SPDIF_ABO_BIT)
#define SPDIF_ABUI_BIT		(SPDIF_ABU_BIT)
#define SPDIF_RUII_BIT		(SPDIF_RUIR_BIT)
#define SPDIF_TUII_BIT		(SPDIF_TUIR_BIT)
#define SPDIF_RCSI_BIT		(SPDIF_CSRX_BIT)
#define SPDIF_RCBI_BIT		(SPDIF_CBRX_BIT)
#define SPDIF_TCSI_BIT		(SPDIF_CSTX_BIT)
#define SPDIF_TCBI_BIT		(SPDIF_CBTX_BIT)

#define FS_MASK			(0x0F000000)
#define FS_32K			(12 << 24)
#define FS_44K			(0 << 24)
#define FS_48K			(4 << 24)
#define CALC_MASK		(0x30000000)
#define CALC_LV2		(0 << 28)
#define CALC_LV1		(1 << 28)
#define CALC_LV3		(2 << 28)

#define WIDTH_MASK		(0xF << 22)
#define WIDTH_16		(0 << 22)
#define WIDTH_24		(0xA << 22)

#define SPDIF_NUMOF_FRM		(192)
#define SPDIF_NUMOF_CH		(2)
#define SPDIF_AUDIO_BUFSZ	(SPDIF_NUMOF_FRM * SPDIF_NUMOF_CH)
#define SPDIF_USER_BUFSZ	((SPDIF_NUMOF_FRM * SPDIF_NUMOF_CH) / 32)
#define SPDIF_CH1		(0)
#define SPDIF_CH2		(1)
/* Pre allocated buffers sizes */
#define PREALLOC_BUFFER		(SZ_32K)
#define PREALLOC_BUFFER_MAX	(SZ_32K)

#define RZ_SPDIF_RATES		(SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				SNDRV_PCM_RATE_48000)
#define RZ_SPDIF_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static uint8_t user_msg[] = "Renesas SPDIF Interface Message";

struct spdif_t {
	union {
		uint32_t data32[SPDIF_USER_BUFSZ];
		unsigned char data[SPDIF_USER_BUFSZ * 4];
	} u_buf;
	int32_t  u_idx;				/* Index of User data		*/
	uint32_t s_buf[SPDIF_NUMOF_CH];		/* Channel Status Buffer	*/
};

struct rzg3s_spdif_stream {
	struct spdif_dev_data *priv;
	struct snd_pcm_substream *substream;
	int dma_buffer_pos;	/* The address for the next DMA descriptor */
	struct dma_chan *dma_ch;
	bool running;

	u32 *dma_pad_buf_tx;
	dma_addr_t dma_pad_addr_tx;
	u32 *dma_pad_buf_rx;
	dma_addr_t dma_pad_addr_rx;
	size_t dma_buf_size;

	int (*transfer)(struct spdif_dev_data *spdif, struct rzg3s_spdif_stream *strm);
};

struct spdif_dev_data {
	void __iomem *base;
	struct reset_control *rstc;
	struct device *dev;
	struct clk *clk;
	phys_addr_t phys;

	/* clock */
	unsigned long audio_clk_1;
	unsigned long audio_clk_2;

	spinlock_t lock;

	u32 mode;
	u32 rate;
	u32 bit_width;
	bool is_dma;
	bool is_passback;

	struct rzg3s_spdif_stream playback;
	struct rzg3s_spdif_stream capture;

	struct spdif_t spdin;
	struct spdif_t spdout;

	/* for PIO */
	int count;
	int tx_byte_pos;
	int tx_byte_per_period;
	int tx_next_period_byte;

	int rx_byte_pos;
	int rx_byte_per_period;
	int rx_next_period_byte;
};

static void rzg3s_spdif_dma_complete(void *data);
static int rzg3s_spdif_dma_slave_config(struct spdif_dev_data *spdif,
				     struct dma_chan *dma_ch, bool is_play);

static inline struct rzg3s_spdif_stream *
rzg3s_spdif_stream_get(struct spdif_dev_data *spdif, struct snd_pcm_substream *substream)
{
	struct rzg3s_spdif_stream *stream = &spdif->playback;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		stream = &spdif->capture;

	return stream;
}

static void rzg3s_spdif_set_substream(struct rzg3s_spdif_stream *strm,
				 struct snd_pcm_substream *substream)
{
	strm->substream = substream;
	strm->dma_buffer_pos = 0;
}

static void rzg3s_spdif_reg_writel(struct spdif_dev_data *priv, uint reg, u32 data)
{
	writel(data, (priv->base + reg));
}

static u32 rzg3s_spdif_reg_readl(struct spdif_dev_data *priv, uint reg)
{
	return readl(priv->base + reg);
}

static void rzg3s_spdif_status_clear(struct spdif_dev_data *spdif)
{
	rzg3s_spdif_reg_writel(spdif, SPDIF_STAT, 0);
}

static u32 rzg3s_spdif_status_get(struct spdif_dev_data *priv)
{
	return rzg3s_spdif_reg_readl(priv, SPDIF_STAT) & (rzg3s_spdif_reg_readl(priv, SPDIF_CTRL)
			& 0x00003FFF);
}

static bool rzg3s_spdif_stream_is_valid(struct spdif_dev_data *spdif,
				     struct rzg3s_spdif_stream *strm)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&spdif->lock, flags);
	ret = strm->substream && strm->substream->runtime;
	spin_unlock_irqrestore(&spdif->lock, flags);

	return ret;
}

static inline bool rzg3s_spdif_stream_is_play(struct snd_pcm_substream *substream)
{
	return substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
}

static inline bool rzg3s_spdif_is_stream_running(struct rzg3s_spdif_stream *strm)
{
	return strm->substream && strm->running;
}

static void rzg3s_spdif_bset(struct spdif_dev_data *priv, uint reg, uint bit_mask, u32 data)
{
	u32 ret;

	ret = rzg3s_spdif_reg_readl(priv, reg);
	ret = ret & ~bit_mask;
	ret = ret | data;
	rzg3s_spdif_reg_writel(priv, reg, ret);
}

static int rzg3s_spdif_stop(struct spdif_dev_data *spdif, struct rzg3s_spdif_stream *strm)
{
	u32 ctrl;

	ctrl = rzg3s_spdif_reg_readl(spdif, SPDIF_CTRL) & (~WIDTH_MASK);
	rzg3s_spdif_reg_writel(spdif, SPDIF_CTRL, ctrl);

	/* Cancel all remaining DMA transactions */
	if (spdif->is_dma)
		dmaengine_terminate_async(strm->dma_ch);

	/* Disable SPDIF to idle state */
	if (rzg3s_spdif_is_stream_running(&spdif->playback)) {
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, 0);
		while (!(rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT))
			;
	}
	if (rzg3s_spdif_is_stream_running(&spdif->capture)) {
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, 0);
		while (!(rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT))
			;
	}

	strm->running = 0;

	return 0;
}

static void rzg3s_spdif_pio_init(struct spdif_dev_data *spdif,
			      struct  rzg3s_spdif_stream *strm)
{
	struct snd_pcm_runtime *runtime = strm->substream->runtime;

	spdif->tx_byte_pos		= 0;
	spdif->tx_byte_per_period	= runtime->period_size *
					  runtime->channels *
					  samples_to_bytes(runtime, 1);
	spdif->tx_next_period_byte	= spdif->tx_byte_per_period;

	spdif->rx_byte_pos		= 0;
	spdif->rx_byte_per_period	= runtime->period_size *
					  runtime->channels *
					  samples_to_bytes(runtime, 1);
	spdif->rx_next_period_byte	= spdif->rx_byte_per_period;
}

static int rzg3s_spdif_start(struct spdif_dev_data *spdif, struct rzg3s_spdif_stream *strm)
{
	u32 ctrl;

	spdif->spdout.u_idx = 0;
	spdif->spdin.u_idx = 0;
	spdif->count = 2;
	strm->running = 1;

	/* PIO init only */
	rzg3s_spdif_pio_init(spdif, strm);

	/*	Reset the register
	 *	STAT - Status Register
	 *	CTRL - Control Register
	 */
	rzg3s_spdif_reg_writel(spdif, SPDIF_CTRL, 0);
	rzg3s_spdif_reg_writel(spdif, SPDIF_STAT, 0);

	if (rzg3s_spdif_is_stream_running(&spdif->playback)) {
		/* Enable transmitter module */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, SPDIF_TME_BIT);
		while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT)
			;
	}
	if (rzg3s_spdif_is_stream_running(&spdif->capture)) {
		/* Enable receiver module */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, SPDIF_RME_BIT);
		while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT)
			;
	}
	/* Setting TASS, RASS */
	ctrl = rzg3s_spdif_reg_readl(spdif, SPDIF_CTRL);

	switch (spdif->bit_width) {
	case 16:
		ctrl = (ctrl & (~WIDTH_MASK)) | WIDTH_16;
		break;
	case 24:
		ctrl = (ctrl & (~WIDTH_MASK)) | WIDTH_24;
		break;
	default:
		dev_err(spdif->dev, "SPDIF only support 16,24 bit\n");
		return -EINVAL;
	}

	ctrl |= spdif->is_passback ? SPDIF_PB_BIT : 0;
	rzg3s_spdif_reg_writel(spdif, SPDIF_CTRL, ctrl);

	return 0;
}

static int rzg3s_spdif_irq(struct spdif_dev_data *spdif,
			struct rzg3s_spdif_stream *strm, int enable)
{
	if (enable) {
		if (rzg3s_spdif_is_stream_running(&spdif->playback)) {
			/* Channel status information */
			rzg3s_spdif_reg_writel(spdif, SPDIF_TLCS, spdif->spdout.s_buf[SPDIF_CH1]);
			rzg3s_spdif_reg_writel(spdif, SPDIF_TRCS, spdif->spdout.s_buf[SPDIF_CH2]);
			while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CSTX_BIT)
				;

			/* Enable interrupt (User data empty) */
			if (spdif->count > 0)
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TUII_BIT, SPDIF_TUII_BIT);

			/* Enable error interrupt */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT, SPDIF_TEIE_BIT);

			if (spdif->is_dma) {
				/* Enable underrun interrupt */
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABUI_BIT, SPDIF_ABUI_BIT);
				/* Enable DMA transmitter */
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, SPDIF_TDE_BIT);
			} else
				/* Enable transmitter interrupt */
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT, SPDIF_TCBI_BIT);
		}
		if (rzg3s_spdif_is_stream_running(&spdif->capture)) {
			/* Enable interrupt (Channel status full) */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCSI_BIT, SPDIF_RCSI_BIT);

			/* Enable interrupt (User data full) */
			if (spdif->spdin.u_idx < SPDIF_USER_BUFSZ)
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RUII_BIT, SPDIF_RUII_BIT);

			/* Enable error interrupt */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_REIE_BIT, SPDIF_REIE_BIT);
			if (spdif->is_dma) {
				/* Enable overrun interrupt */
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABOI_BIT, SPDIF_ABOI_BIT);
				/* Enable DMA receiver */
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, SPDIF_RDE_BIT);
			} else
				/* Enable receiver interrupt */
				rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
		}
	} else {
		if (rzg3s_spdif_is_stream_running(&spdif->playback)) {
			/* Disable error interrupt */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT | SPDIF_ABUI_BIT, 0);

			/* Disable interrupt  */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT | SPDIF_TUII_BIT, 0);
			/* Disable DMA transmitter */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, 0);
		}
		if (rzg3s_spdif_is_stream_running(&spdif->capture)) {
			/* Disable error interrupt */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_REIE_BIT | SPDIF_ABOI_BIT, 0);

			/* Disable interrupt */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT | SPDIF_RCSI_BIT |
					SPDIF_RUII_BIT, 0);
			/* Disable DMA receiver */
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, 0);
		}
	}

	return 0;
}

static void rzg3s_spdif_quit(struct rzg3s_spdif_stream *strm)
{
	rzg3s_spdif_set_substream(strm, NULL);
}

static int rzg3s_spdif_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	struct rzg3s_spdif_stream *strm = rzg3s_spdif_stream_get(spdif, substream);
	int ret, i, num_transfer = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		/* Set substream to priv data */
		rzg3s_spdif_set_substream(strm, substream);

		ret = rzg3s_spdif_start(spdif, strm);
		if (ret < 0)
			goto dai_trigger_end;

		ret = rzg3s_spdif_irq(spdif, strm, 1);
		if (ret < 0)
			goto dai_trigger_end;

		/* For DMA, queue up multiple DMA descriptors */
		if (spdif->is_dma) {
			num_transfer = 3;
			for (i = 0; i < num_transfer; i++) {
				ret = strm->transfer(spdif, strm);
				if (ret)
					goto dai_trigger_end;
			}
		}

		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		ret = rzg3s_spdif_irq(spdif, strm, 0);
		rzg3s_spdif_stop(spdif, strm);
		rzg3s_spdif_quit(strm);
		break;
	default:
		ret = -EINVAL;
	}

dai_trigger_end:
	return ret;
}

static int rzg3s_spdif_dai_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = snd_soc_dai_get_drvdata(dai);
	struct rzg3s_spdif_stream *strm = rzg3s_spdif_stream_get(spdif, substream);
	u32 channel_status1, channel_status2;

	spdif->rate = params_rate(params);
	spdif->bit_width = params_width(params);

	if (params_channels(params) != 2) {
		dev_err(spdif->dev, "SPDIF only support 2 channel\n");
		return -EIO;
	}

	/*	Transmitter Channel X Status Register
	 *	b31:b30 Reserved - Read only bit       - 00:
	 *	b29:b28 CLAC     - Clock Accuracy      - 01:   Level 1 (50 ppm)
	 *	b27:b24 FS       - Sample Frequency    - 0000: 44.1kHz
	 *	b23:b20 CHNO     - Channel Number      - 0001: Left (0010: Right)
	 *	b19:b16 SRCNO    - Source Number       - 0000:
	 *	b15:b8  CATCD    - Category Code       - 00000000: General 2-channel format
	 *	b7:b6   Reserved - Read only bit       - 00:
	 *	b5:b1   CTL      - Control             - 0:  2 channel Audio
	 *	b0      Reserved - Read only bit       - 0:
	 */

	channel_status1 = 0x10100000;
	channel_status2 = 0x10200000;

	switch (spdif->rate) {
	case 32000:
		channel_status1 = (channel_status1 & (~FS_MASK)) | FS_32K;
		channel_status2 = (channel_status2 & (~FS_MASK)) | FS_32K;
		break;
	case 44100:
		channel_status1 = (channel_status1 & (~FS_MASK)) | FS_44K;
		channel_status2 = (channel_status2 & (~FS_MASK)) | FS_44K;
		break;
	case 48000:
		channel_status1 = (channel_status1 & (~FS_MASK)) | FS_48K;
		channel_status2 = (channel_status2 & (~FS_MASK)) | FS_48K;
		break;
	default:
		dev_err(spdif->dev, "SPDIF only support 32,44,48 KHz\n");
		return -EINVAL;
	}

	/* Buffer clear */
	memset(&spdif->spdout, 0, sizeof(struct spdif_t));
	memset(&spdif->spdin,  0, sizeof(struct spdif_t));

	/* Initialize user data */
	memcpy(spdif->spdout.u_buf.data, user_msg, sizeof(user_msg));

	spdif->spdout.s_buf[SPDIF_CH1] = channel_status1;
	spdif->spdout.s_buf[SPDIF_CH2] = channel_status2;

	/*
	 * Allocate a DMA buffer used specifically for S16_LE format in DMA transfer mode.
	 * This buffer provides the required padding when converting 16-bit samples
	 * to the 32-bit data width expected by the SPDIF hardware.
	 */
	if (spdif->is_dma && spdif->bit_width == 16) {
		strm->dma_buf_size = params_period_size(params) * 2 * sizeof(u32);

		if (rzg3s_spdif_stream_is_play(substream)) {
			strm->dma_pad_buf_tx = dma_alloc_coherent(spdif->dev,
								  strm->dma_buf_size,
								  &strm->dma_pad_addr_tx,
								  GFP_KERNEL);

			if (!strm->dma_pad_buf_tx)
				return -ENOMEM;
		} else {
			strm->dma_pad_buf_rx = dma_alloc_coherent(spdif->dev,
								  strm->dma_buf_size,
								  &strm->dma_pad_addr_rx,
								  GFP_KERNEL);

			if (!strm->dma_pad_buf_rx)
				return -ENOMEM;
		}
	}
	return 0;
}

static int rzg3s_spdif_dai_hw_free(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = snd_soc_dai_get_drvdata(dai);
	struct rzg3s_spdif_stream *strm = rzg3s_spdif_stream_get(spdif, substream);

	/* Free padding buffer after use */
	if (spdif->is_dma && spdif->bit_width == 16) {
		if (strm->dma_pad_buf_tx) {
			dma_free_coherent(spdif->dev, strm->dma_buf_size,
					  strm->dma_pad_buf_tx, strm->dma_pad_addr_tx);
			strm->dma_pad_buf_tx = NULL;
		}
		if (strm->dma_pad_buf_rx) {
			dma_free_coherent(spdif->dev, strm->dma_buf_size,
					  strm->dma_pad_buf_rx, strm->dma_pad_addr_rx);
			strm->dma_pad_buf_rx = NULL;
		}
	}

	return 0;
}

static const struct snd_soc_dai_ops rzg3s_spdif_dai_ops = {
	.trigger = rzg3s_spdif_dai_trigger,
	.hw_params = rzg3s_spdif_dai_hw_params,
	.hw_free = rzg3s_spdif_dai_hw_free,
};

static const struct snd_pcm_hardware rzg3s_spdif_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED	|
				  SNDRV_PCM_INFO_MMAP		|
				  SNDRV_PCM_INFO_MMAP_VALID,
	.buffer_bytes_max	= PREALLOC_BUFFER,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192,
	.channels_min		= 2,
	.channels_max		= 2,
	.periods_min		= 1,
	.periods_max		= 32,
	.fifo_size		= 32 * 2,
};

static int rzg3s_spdif_pcm_new(struct snd_soc_component *component,
			    struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       rtd->card->snd_card->dev,
				       PREALLOC_BUFFER, PREALLOC_BUFFER_MAX);

	return 0;
}

static snd_pcm_uframes_t rzg3s_spdif_pcm_pointer(struct snd_soc_component *component,
					      struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	struct rzg3s_spdif_stream *strm = rzg3s_spdif_stream_get(spdif, substream);

	/* Handle buffer pos for DMA transfer */
	if (spdif->is_dma)
		return strm->dma_buffer_pos;
	if (rzg3s_spdif_is_stream_running(&spdif->playback))
		return bytes_to_frames(runtime, READ_ONCE(spdif->tx_byte_pos));
	if (rzg3s_spdif_is_stream_running(&spdif->capture))
		return bytes_to_frames(runtime, READ_ONCE(spdif->rx_byte_pos));

	return 0;
}

static int rzg3s_spdif_pcm_open(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rzg3s_spdif_pcm_hardware);

	return snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
}

static struct snd_soc_dai_driver rzg3s_spdif_soc_dai = {
	.name = "rzg3s_spdif_dai",
	.playback = {
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= RZ_SPDIF_RATES,
		.formats	= RZ_SPDIF_FORMATS,
	},
	.capture = {
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= RZ_SPDIF_RATES,
		.formats	= RZ_SPDIF_FORMATS,
	},
	.ops = &rzg3s_spdif_dai_ops,
};

static int rzg3s_spdif_cs_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int rzg3s_spdif_ub_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(user_msg);

	return 0;
}

static int rzg3s_spdif_kctrl_cs_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct device *dev = component->dev;
	struct spdif_dev_data *spdif = dev_get_drvdata(dev);

	ucontrol->value.iec958.status[0] = IEC958_AES0_CON_NOT_COPYRIGHT |
					   IEC958_AES0_CON_EMPHASIS_NONE;
	ucontrol->value.iec958.status[1] = IEC958_AES1_CON_GENERAL;
	ucontrol->value.iec958.status[2] = IEC958_AES2_CON_SOURCE_UNSPEC |
					   IEC958_AES2_CON_CHANNEL_UNSPEC;
	ucontrol->value.iec958.status[3] = IEC958_AES3_CON_FS_NOTID |
					   IEC958_AES3_CON_CLOCK;

	if (spdif->spdin.s_buf[SPDIF_CH1] == 0)
		return 0;

	switch (spdif->spdin.s_buf[SPDIF_CH1] & FS_MASK) {
	case FS_32K:
		ucontrol->value.iec958.status[3] = IEC958_AES3_CON_FS_32000;
		break;
	case FS_44K:
		ucontrol->value.iec958.status[3] = IEC958_AES3_CON_FS_44100;
		break;
	case FS_48K:
		ucontrol->value.iec958.status[3] = IEC958_AES3_CON_FS_48000;
		break;
	}

	switch (spdif->spdin.s_buf[SPDIF_CH1] & CALC_MASK) {
	case CALC_LV2:
		ucontrol->value.iec958.status[3] |= IEC958_AES3_CON_CLOCK_1000PPM;
		break;
	case CALC_LV1:
		ucontrol->value.iec958.status[3] |= IEC958_AES3_CON_CLOCK_50PPM;
		break;
	case CALC_LV3:
		ucontrol->value.iec958.status[3] |= IEC958_AES3_CON_CLOCK_VARIABLE;
		break;
	}

	return 0;
}

static int rzg3s_spdif_kctrl_ub_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct device *dev = component->dev;
	struct spdif_dev_data *spdif = dev_get_drvdata(dev);

	memcpy(ucontrol->value.bytes.data, spdif->spdin.u_buf.data, SPDIF_USER_BUFSZ * 4);

	return 0;
}

static int rzg3s_spdif_kctrl_mode_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct device *dev = component->dev;
	struct spdif_dev_data *spdif = dev_get_drvdata(dev);

	ucontrol->value.integer.value[0] = spdif->is_passback;

	return 0;
}

static int rzg3s_spdif_kctrl_mode_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct device *dev = component->dev;
	struct spdif_dev_data *spdif = dev_get_drvdata(dev);
	int mode = ucontrol->value.integer.value[0];

	spdif->is_passback = mode;

	return 0;
}

static const struct snd_kcontrol_new rzg3s_spdif_snd_kcontrol[] = {
	/* Chanel status controller */
	{
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
		.access	= SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info	= rzg3s_spdif_cs_info,
		.get	= rzg3s_spdif_kctrl_cs_get,
	},
	/* User bits controller */
	{
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "IEC958 User Data Receive",
		.access	= SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info	= rzg3s_spdif_ub_info,
		.get	= rzg3s_spdif_kctrl_ub_get,
	},
	/* Pass Back mode controller */
	{
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "Renesas SPDIF Pass Back Mode",
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= snd_ctl_boolean_mono_info,
		.get	= rzg3s_spdif_kctrl_mode_get,
		.put	= rzg3s_spdif_kctrl_mode_put,
	},
};

static const struct snd_soc_component_driver rzg3s_spdif_component = {
	.name		= "rz-spdif",
	.open		= rzg3s_spdif_pcm_open,
	.pointer	= rzg3s_spdif_pcm_pointer,
	.pcm_construct	= rzg3s_spdif_pcm_new,
	.controls	= rzg3s_spdif_snd_kcontrol,
	.num_controls	= ARRAY_SIZE(rzg3s_spdif_snd_kcontrol),
};

static bool rzg3s_spdif_pio_recv(int irq, struct spdif_dev_data *spdif)
{
	struct snd_pcm_runtime *runtime = spdif->capture.substream->runtime;
	snd_pcm_format_t fmt = runtime->format;
	int byte_pos;
	bool elapsed = false;

	do {
		switch (fmt) {
		case SNDRV_PCM_FORMAT_S24_LE: {
			u64 *rx_buf = (u64 *)(runtime->dma_area + spdif->rx_byte_pos);

			*rx_buf = ((u64)(rzg3s_spdif_reg_readl(spdif, SPDIF_RLCA) & 0xFFFFFF)
					 << 32) | rzg3s_spdif_reg_readl(spdif, SPDIF_RRCA);

			byte_pos = spdif->rx_byte_pos + sizeof(*rx_buf);
			break;
		} case SNDRV_PCM_FORMAT_S16_LE: {
			u32 *rx_buf = (u32 *)(runtime->dma_area + spdif->rx_byte_pos);

			*rx_buf = ((rzg3s_spdif_reg_readl(spdif, SPDIF_RLCA) & 0xFFFF) << 16) |
				   (rzg3s_spdif_reg_readl(spdif, SPDIF_RRCA) & 0xFFFF);
			byte_pos = spdif->rx_byte_pos + sizeof(*rx_buf);
			break;
		} default:
			return false;
		}

	} while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CBRX_BIT);

	if (byte_pos >= spdif->rx_next_period_byte) {
		int period_pos = byte_pos / spdif->rx_byte_per_period;

		if (period_pos >= runtime->periods) {
			byte_pos = 0;
			period_pos = 0;
		}
		spdif->rx_next_period_byte = (period_pos + 1) * spdif->rx_byte_per_period;

		elapsed = true;
	}

	WRITE_ONCE(spdif->rx_byte_pos, byte_pos);

	return elapsed;
}

static bool rzg3s_spdif_pio_send(int irq, struct spdif_dev_data *spdif)
{
	struct snd_pcm_runtime *runtime = spdif->playback.substream->runtime;
	snd_pcm_format_t fmt = runtime->format;
	int byte_pos;
	bool elapsed = false;
	u32 data;

	do {
		switch (fmt) {
		case SNDRV_PCM_FORMAT_S24_LE: {
			u64 *tx_buf = (u64 *)(runtime->dma_area + spdif->tx_byte_pos);

			data = ((*tx_buf) >> 32 & 0xFFFFFF);
			rzg3s_spdif_reg_writel(spdif, SPDIF_TLCA, data);
			data = ((*tx_buf) & 0xFFFFFF);
			rzg3s_spdif_reg_writel(spdif, SPDIF_TRCA, data);

			byte_pos = spdif->tx_byte_pos + sizeof(*tx_buf);
			break;
		} case SNDRV_PCM_FORMAT_S16_LE: {
			u32 *tx_buf = (u32 *)(runtime->dma_area + spdif->tx_byte_pos);

			data = ((*tx_buf) >> 16 & 0xFFFF);
			rzg3s_spdif_reg_writel(spdif, SPDIF_TLCA, data);
			data = ((*tx_buf) & 0xFFFF);
			rzg3s_spdif_reg_writel(spdif, SPDIF_TRCA, data);

			byte_pos = spdif->tx_byte_pos + sizeof(*tx_buf);
			break;
		} default:
			return false;
		}
	} while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CBTX_BIT);

	if (byte_pos >= spdif->tx_next_period_byte) {
		int period_pos = byte_pos / spdif->tx_byte_per_period;

		if (period_pos >= runtime->periods) {
			byte_pos = 0;
			period_pos = 0;
		}
		spdif->tx_next_period_byte = (period_pos + 1) * spdif->tx_byte_per_period;

		elapsed = true;
	}

	WRITE_ONCE(spdif->tx_byte_pos, byte_pos);

	return elapsed;
}

static void rzg3s_spdif_out_restart(struct spdif_dev_data *spdif)
{
	if (spdif->is_dma)
		/* Disable DMA transmitter */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, 0);
	else
		/* Disable Transmitter interrupt */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT, 0);

	rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, 0);
	while (!(rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT))
		;

	rzg3s_spdif_status_clear(spdif);

	/* Retransmit */
	/* Enable transmitter module */
	rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, SPDIF_TME_BIT);
	while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT)
		;

	/* Enable interrupt (User data empty) */
	if (spdif->count > 0)
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TUII_BIT, SPDIF_TUII_BIT);

	/* Enable error interrupt */
	rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT, SPDIF_TEIE_BIT);

	if (spdif->is_dma) {
		/* Enable underrun interrupt */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABUI_BIT, SPDIF_ABUI_BIT);
		/* Enable DMA transmitter */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, SPDIF_TDE_BIT);
	} else
		/* Enable transmitter interrupt */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT, SPDIF_TCBI_BIT);
}

static void rzg3s_spdif_in_restart(struct spdif_dev_data *spdif)
{
	if (spdif->is_dma)
		/* Disable DMA receiver */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, 0);
	else
		/* Disable Receiver interrupt */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, 0);
	rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, 0);
	while (!(rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT))
		;

	rzg3s_spdif_status_clear(spdif);

	/* Retransmit */
	/* Enable receiver module */
	rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, SPDIF_RME_BIT);
	while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT)
		;

	/* Enable interrupt (Channel status full) */
	rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCSI_BIT, SPDIF_RCSI_BIT);

	/* Enable interrupt (User data full) */
	if (spdif->spdin.u_idx < SPDIF_USER_BUFSZ)
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RUII_BIT, SPDIF_RUII_BIT);

	/* Enable error interrupt */
	rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_REIE_BIT, SPDIF_REIE_BIT);

	if (spdif->is_dma) {
		/* Enable overrun interrupt */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABOI_BIT, SPDIF_ABOI_BIT);
		/* Enable DMA receiver */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, SPDIF_RDE_BIT);
	} else
		/* Enable Receiver interrupt */
		rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
}

static irqreturn_t rzg3s_spdif_irq_handler(int irq, void *arg)
{
	struct spdif_dev_data *spdif = arg;
	struct rzg3s_spdif_stream *strm_playback = NULL;
	struct rzg3s_spdif_stream *strm_capture = NULL;
	u32 stat, udata;
	bool tx_error = false;
	bool rx_error = false;

	spin_lock(&spdif->lock);
	if (spdif->playback.substream)
		strm_playback = &spdif->playback;
	if (spdif->capture.substream)
		strm_capture = &spdif->capture;

	if (!strm_playback && !strm_capture)
		return IRQ_HANDLED;

	stat = rzg3s_spdif_status_get(spdif);

	if (spdif->is_dma) {
		/* Clear error status */
		if (stat & SPDIF_ABU_BIT) {
			rzg3s_spdif_bset(spdif, SPDIF_STAT, SPDIF_ABU_BIT, 0);
			tx_error = true;
		}
		if (stat & SPDIF_ABO_BIT) {
			rzg3s_spdif_bset(spdif, SPDIF_STAT, SPDIF_ABO_BIT, 0);
			rx_error = true;
		}
	} else {
		if (stat & SPDIF_CBTX_BIT)
			if (rzg3s_spdif_pio_send(irq, spdif))
				snd_pcm_period_elapsed(strm_playback->substream);

		if (stat & SPDIF_CBRX_BIT)
			if (rzg3s_spdif_pio_recv(irq, spdif))
				snd_pcm_period_elapsed(strm_capture->substream);
	}

	/* Receiver channel status interrupt (CSRX) */
	if (stat & SPDIF_CSRX_BIT) {
		do {
			/* Read the status data */
			spdif->spdin.s_buf[SPDIF_CH1] = rzg3s_spdif_reg_readl(spdif, SPDIF_RLCS);
			spdif->spdin.s_buf[SPDIF_CH2] = rzg3s_spdif_reg_readl(spdif, SPDIF_RRCS);
		} while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CSRX_BIT);
	}

	/* Transmitter user information interrupt (TUIR) */
	if (stat & SPDIF_TUIR_BIT) {
		do {
			rzg3s_spdif_reg_writel(spdif, SPDIF_TUI,
					spdif->spdout.u_buf.data32[spdif->spdout.u_idx]);
		} while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TUIR_BIT);

		spdif->spdout.u_idx++;
		if (spdif->spdout.u_idx >= SPDIF_USER_BUFSZ) {
			spdif->spdout.u_idx = 0;
			spdif->count--;
		}
		if (spdif->count < 0)
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TUII_BIT, 0);
	}

	/* Receiver user information interrupt (RUIR) */
	if (stat & SPDIF_RUIR_BIT) {
		do {
			udata = rzg3s_spdif_reg_readl(spdif, SPDIF_RUI);
		} while (rzg3s_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RUIR_BIT);

		/* Detect the end and Store the data */
		if (udata == 0)
			spdif->spdin.u_idx = 0;
		else
			spdif->spdin.u_buf.data32[spdif->spdin.u_idx++] = udata;
		if (spdif->spdin.u_idx > SPDIF_USER_BUFSZ)
			rzg3s_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RUII_BIT, 0);
	}

	rzg3s_spdif_status_clear(spdif);

	spin_unlock(&spdif->lock);

	if (tx_error)
		rzg3s_spdif_out_restart(spdif);
	if (rx_error)
		rzg3s_spdif_in_restart(spdif);

	return IRQ_HANDLED;
}

static int rzg3s_spdif_dma_slave_config(struct spdif_dev_data *spdif,
				     struct dma_chan *dma_ch, bool is_play)
{
	struct dma_slave_config cfg;

	memset(&cfg, 0, sizeof(cfg));

	cfg.direction = is_play ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;
	cfg.dst_addr = spdif->phys + SPDIF_TDAD;
	cfg.src_addr = spdif->phys + SPDIF_RDAD;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_maxburst = 1;
	cfg.dst_maxburst = 1;

	return dmaengine_slave_config(dma_ch, &cfg);
}

static int rzg3s_spdif_dma_transfer(struct spdif_dev_data *spdif,
				 struct rzg3s_spdif_stream *strm)
{
	struct snd_pcm_substream *substream = strm->substream;
	struct dma_async_tx_descriptor *desc;
	struct snd_pcm_runtime *runtime;
	enum dma_transfer_direction dir;
	int amount;
	dma_addr_t dma_addr;
	size_t dma_size;

	if (!rzg3s_spdif_stream_is_valid(spdif, strm))
		return -EINVAL;

	runtime = substream->runtime;
	if (runtime->status->state == SNDRV_PCM_STATE_DRAINING)
		/*
		 * Stream is ending, so do not queue up any more DMA
		 * transfers otherwise we play partial sound clips
		 * because we can't shut off the DMA quick enough.
		 */
		return 0;

	dir = rzg3s_spdif_stream_is_play(substream) ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;

	/* Always transfer 1 period */
	amount = runtime->period_size;

	/* As mentioned in the HW manual, SPDIF registers are longword registers
	 * and must be accessed using 32-bit operations. DMA transfers audio
	 * by sending channel 1 first, then channel 2.
	 *
	 * However, with the S16_LE format, ALSA sends 32 bits per frame
	 * (16 bits per channel). Therefore, when using 16-bit audio formats,
	 * we must pad each 16-bit channel sample to 32 bits as the HW requirement.
	 */
	if (runtime->format == SNDRV_PCM_FORMAT_S16_LE) {
		if (rzg3s_spdif_stream_is_play(substream)) {
			/* For TX: Padding from 16-bit to 32-bit */
			const u16 *src = (const u16 *)(runtime->dma_area +
					 frames_to_bytes(runtime, strm->dma_buffer_pos));
			u32 *dst = strm->dma_pad_buf_tx;

			for (size_t i = 0; i < amount * 2; i++)
				dst[i] = (u32)src[i];

			dma_addr = strm->dma_pad_addr_tx;
		} else
			/* For RX: write directly to RX pad buffer */
			dma_addr = strm->dma_pad_addr_rx;

		dma_size = amount * 2 * sizeof(u32);
	} else {
		dma_addr = runtime->dma_addr + frames_to_bytes(runtime, strm->dma_buffer_pos);
		dma_size = frames_to_bytes(runtime, amount);
	}

	desc = dmaengine_prep_slave_single(strm->dma_ch,
					   dma_addr,
					   dma_size,
					   dir,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(spdif->dev, "dmaengine_prep_slave_single() fail\n");
		return -ENOMEM;
	}

	desc->callback = rzg3s_spdif_dma_complete;
	desc->callback_param = strm;

	if (dmaengine_submit(desc) < 0) {
		dev_err(spdif->dev, "dmaengine_submit() fail\n");
		return -EIO;
	}

	/* Update DMA pointer */
	strm->dma_buffer_pos += amount;
	if (strm->dma_buffer_pos >= runtime->buffer_size)
		strm->dma_buffer_pos = 0;

	/* Start DMA */
	dma_async_issue_pending(strm->dma_ch);

	return 0;
}

static void rzg3s_spdif_dma_complete(void *data)
{
	struct rzg3s_spdif_stream *strm = (struct rzg3s_spdif_stream *)data;
	struct snd_pcm_runtime *runtime;

	if (!strm->substream || !strm->substream->runtime)
		return;

	runtime = strm->substream->runtime;

	if ((runtime->format == SNDRV_PCM_FORMAT_S16_LE) &&
	    (strm->substream->stream == SNDRV_PCM_STREAM_CAPTURE)) {
		/* For RX: Unpad from 32-bit to 16-bit */
		const u32 *src = strm->dma_pad_buf_rx;
		u16 *dst = (u16 *)(runtime->dma_area +
				  frames_to_bytes(runtime, strm->dma_buffer_pos));

		for (size_t i = 0; i < runtime->period_size * 2; i++)
			dst[i] = (u16)(src[i] & 0xFFFF);
	}

	snd_pcm_period_elapsed(strm->substream);
	/* Queue up another DMA transaction */
	rzg3s_spdif_dma_transfer(strm->priv, strm);
}

static void rzg3s_spdif_release_dma_channels(struct spdif_dev_data *spdif)
{
	if (spdif->playback.dma_ch) {
		dma_release_channel(spdif->playback.dma_ch);
		spdif->playback.dma_ch = NULL;
	}
	if (spdif->capture.dma_ch) {
		dma_release_channel(spdif->capture.dma_ch);
		spdif->capture.dma_ch = NULL;
	}
}

static int rzg3s_spdif_dma_request(struct spdif_dev_data *spdif, struct device *dev)
{

	spdif->playback.dma_ch = dma_request_chan(dev, "tx");
	if (IS_ERR(spdif->playback.dma_ch))
		spdif->playback.dma_ch = NULL;

	spdif->capture.dma_ch = dma_request_chan(dev, "rx");
	if (IS_ERR(spdif->capture.dma_ch))
		spdif->capture.dma_ch = NULL;

	if (!(spdif->playback.dma_ch && spdif->capture.dma_ch))
		goto no_dma;

	if (spdif->playback.dma_ch &&
	   (rzg3s_spdif_dma_slave_config(spdif, spdif->playback.dma_ch, true) < 0))
		goto no_dma;

	if (spdif->capture.dma_ch &&
	   (rzg3s_spdif_dma_slave_config(spdif, spdif->capture.dma_ch, false) < 0))
		goto no_dma;

	return 0;

no_dma:
	rzg3s_spdif_release_dma_channels(spdif);

	return -ENODEV;
}

static int rzg3s_spdif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spdif_dev_data *spdif;
	struct clk *audio_clk;
	struct resource *res;
	int ret;

	spdif = devm_kzalloc(dev, sizeof(*spdif), GFP_KERNEL);
	if (!spdif)
		return -ENOMEM;

	spdif->dev = dev;
	spdif->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(spdif->base))
		return PTR_ERR(spdif->base);

	spdif->phys = res->start;
	spdif->clk = devm_clk_get(dev, "spdif-tx-rx");
	if (IS_ERR(spdif->clk))
		return PTR_ERR(spdif->clk);

	audio_clk = devm_clk_get(dev, "audio_clk1");
	if (IS_ERR(audio_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(audio_clk),
				     "no audio clk1");
	spdif->audio_clk_1 = clk_get_rate(audio_clk);

	audio_clk = devm_clk_get(dev, "audio_clk2");
	if (IS_ERR(audio_clk))
		return dev_err_probe(dev, PTR_ERR(audio_clk),
				     "no audio clk2");
	spdif->audio_clk_2 = clk_get_rate(audio_clk);

	if (!(spdif->audio_clk_1 || spdif->audio_clk_2))
		return dev_err_probe(dev, -EINVAL,
				     "no audio clk1 or audio clk2");

	/* Detect DMA support */
	ret = rzg3s_spdif_dma_request(spdif, &pdev->dev);
	if (ret < 0) {
		dev_warn(&pdev->dev, "DMA not available, using PIO\n");
		spdif->is_dma = false;
	} else {
		dev_info(&pdev->dev, "DMA enabled");
		spdif->playback.transfer = rzg3s_spdif_dma_transfer;
		spdif->capture.transfer = rzg3s_spdif_dma_transfer;
		spdif->is_dma = true;
	}

	spdif->playback.priv = spdif;
	spdif->capture.priv = spdif;

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto probe_err;

	ret = devm_request_irq(dev, ret, rzg3s_spdif_irq_handler,
				0, dev_name(dev), spdif);
	if (ret < 0) {
		dev_err(dev, "spdif irq request failed\n");
		goto probe_err;
	}

	spin_lock_init(&spdif->lock);
	dev_set_drvdata(dev, spdif);

	spdif->rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(spdif->rstc))
		return PTR_ERR(spdif->rstc);

	reset_control_deassert(spdif->rstc);
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_resume_and_get failed\n");
		goto probe_err;
	}

	ret = devm_snd_soc_register_component(dev, &rzg3s_spdif_component,
					      &rzg3s_spdif_soc_dai, 1);
	if (ret < 0) {
		dev_err(dev, "failed to register SPDIF snd component\n");
		goto probe_err;
	}

	return ret;

probe_err:
	pm_runtime_put(spdif->dev);
	pm_runtime_disable(spdif->dev);
	reset_control_assert(spdif->rstc);

	return ret;
}

static int rzg3s_spdif_remove(struct platform_device *pdev)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(&pdev->dev);

	pm_runtime_put(spdif->dev);
	pm_runtime_disable(spdif->dev);
	reset_control_assert(spdif->rstc);

	return 0;
}

static int __maybe_unused rzg3s_spdif_suspend(struct device *dev)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dev);

	if (spdif->rstc)
		reset_control_assert(spdif->rstc);

	pm_runtime_put_sync(spdif->dev);

	return 0;
}

static int __maybe_unused rzg3s_spdif_resume(struct device *dev)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dev);
	int ret;

	if (spdif->rstc) {
		ret = reset_control_deassert(spdif->rstc);
		if (ret)
			return ret;
	}

	pm_runtime_get_sync(spdif->dev);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rzg3s_spdif_pm_ops, rzg3s_spdif_suspend,
			 rzg3s_spdif_resume);

static const struct of_device_id rzg3s_spdif_of_match[] = {
	{ .compatible = "renesas,rz-spdif", },
	{/* Sentinel */},
};
MODULE_DEVICE_TABLE(of, rzg3s_spdif_of_match);

static struct platform_driver rzg3s_spdif_driver = {
	.driver	= {
		.name	= "rz-spdif",
		.of_match_table = rzg3s_spdif_of_match,
		.pm = &rzg3s_spdif_pm_ops,
	},
	.probe		= rzg3s_spdif_probe,
	.remove		= rzg3s_spdif_remove,
};

module_platform_driver(rzg3s_spdif_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas RZ/G3S ASoC Sony/Philips Digital Interface Format Driver");
MODULE_AUTHOR("Duy Dang <duy.dang.yb@renesas.com>");
