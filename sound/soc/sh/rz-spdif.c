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

struct rz_spdif_stream {
	struct spdif_dev_data *priv;
	struct snd_pcm_substream *substream;
	int dma_buffer_pos;	/* The address for the next DMA descriptor */
	struct dma_chan *dma_ch;

	int (*transfer)(struct spdif_dev_data *spdif, struct rz_spdif_stream *strm);
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

	struct rz_spdif_stream playback;
	struct rz_spdif_stream capture;

	struct spdif_t spdin;
	struct spdif_t spdout;

	/* for PIO */
	int count;
	int byte_pos;
	int byte_per_period;
	int next_period_byte;
};

static void rz_spdif_dma_complete(void *data);
static int rz_spdif_dma_slave_config(struct spdif_dev_data *spdif,
				     struct dma_chan *dma_ch, bool is_play);

static inline struct rz_spdif_stream *
rz_spdif_stream_get(struct spdif_dev_data *spdif, struct snd_pcm_substream *substream)
{
	struct rz_spdif_stream *stream = &spdif->playback;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		stream = &spdif->capture;

	return stream;
}

static void rz_spdif_set_substream(struct rz_spdif_stream *strm,
				 struct snd_pcm_substream *substream)
{
	strm->substream = substream;
	strm->dma_buffer_pos = 0;
}

static void rz_spdif_reg_writel(struct spdif_dev_data *priv, uint reg, u32 data)
{
	writel(data, (priv->base + reg));
}

static u32 rz_spdif_reg_readl(struct spdif_dev_data *priv, uint reg)
{
	return readl(priv->base + reg);
}

static void rz_spdif_status_clear(struct spdif_dev_data *spdif)
{
	rz_spdif_reg_writel(spdif, SPDIF_STAT, 0);
}

static u32 rz_spdif_status_get(struct spdif_dev_data *priv)
{
	return rz_spdif_reg_readl(priv, SPDIF_STAT) & (rz_spdif_reg_readl(priv, SPDIF_CTRL)
			& 0x00003FFF);
}

static bool rz_spdif_stream_is_valid(struct spdif_dev_data *spdif,
				     struct rz_spdif_stream *strm)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&spdif->lock, flags);
	ret = strm->substream && strm->substream->runtime;
	spin_unlock_irqrestore(&spdif->lock, flags);

	return ret;
}

static inline bool rz_spdif_stream_is_play(struct snd_pcm_substream *substream)
{
	return substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
}

static void rz_spdif_bset(struct spdif_dev_data *priv, uint reg, uint bit_mask, u32 data)
{
	u32 ret;

	ret = rz_spdif_reg_readl(priv, reg);
	ret = ret & ~bit_mask;
	ret = ret | data;
	rz_spdif_reg_writel(priv, reg, ret);
}

static int rz_spdif_stop(struct spdif_dev_data *spdif, struct rz_spdif_stream *strm)
{
	u32 ctrl;

	ctrl = rz_spdif_reg_readl(spdif, SPDIF_CTRL) & (~WIDTH_MASK);
	rz_spdif_reg_writel(spdif, SPDIF_CTRL, ctrl);

	/* Cancel all remaining DMA transactions */
	if (spdif->is_dma)
		dmaengine_terminate_async(strm->dma_ch);

	/* Disable SPDIF to idle state */
	if (rz_spdif_stream_is_play(strm->substream)) {
		rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, 0);
		while (!(rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT))
			;
	} else {
		rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, 0);
		while (!(rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT))
			;
	}

	return 0;
}

static void rz_spdif_pio_init(struct spdif_dev_data *spdif,
			      struct  rz_spdif_stream *strm)
{
	struct snd_pcm_runtime *runtime = strm->substream->runtime;

	spdif->byte_pos		= 0;
	spdif->byte_per_period	= runtime->period_size *
				  runtime->channels *
				  samples_to_bytes(runtime, 1);
	spdif->next_period_byte	= spdif->byte_per_period;
}

static int rz_spdif_start(struct spdif_dev_data *spdif, struct rz_spdif_stream *strm)
{
	u32 ctrl;

	spdif->spdout.u_idx = 0;
	spdif->spdin.u_idx = 0;
	spdif->count = 2;

	/* PIO init only */
	rz_spdif_pio_init(spdif, strm);

	/*	Reset the register
	 *	STAT - Status Register
	 *	CTRL - Control Register
	 */

	rz_spdif_reg_writel(spdif, SPDIF_CTRL, 0);
	rz_spdif_reg_writel(spdif, SPDIF_STAT, 0);

	if (rz_spdif_stream_is_play(strm->substream)) {
		/* Enable transmitter module */
		rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, SPDIF_TME_BIT);
		while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT)
			;
	} else {
		/* Enable receiver module */
		rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, SPDIF_RME_BIT);
		while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT)
			;
	}
	/* Setting TASS, RASS */
	ctrl = rz_spdif_reg_readl(spdif, SPDIF_CTRL);

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

	rz_spdif_reg_writel(spdif, SPDIF_CTRL, ctrl);

	return 0;
}

static int rz_spdif_irq(struct spdif_dev_data *spdif,
			struct rz_spdif_stream *strm, int enable)
{
	if (enable) {
		if (rz_spdif_stream_is_play(strm->substream)) {
			/* Channel status information */
			rz_spdif_reg_writel(spdif, SPDIF_TLCS, spdif->spdout.s_buf[SPDIF_CH1]);
			rz_spdif_reg_writel(spdif, SPDIF_TRCS, spdif->spdout.s_buf[SPDIF_CH2]);
			while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CSTX_BIT)
				;

			/* Enable interrupt (User data empty) */
			if (spdif->count > 0)
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TUII_BIT, SPDIF_TUII_BIT);

			/* Enable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT, SPDIF_TEIE_BIT);

			if (spdif->is_dma) {
				/* Enable underrun interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABUI_BIT, SPDIF_ABUI_BIT);
				/* Enable DMA transmitter */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, SPDIF_TDE_BIT);
			} else
				/* Enable transmitter interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT, SPDIF_TCBI_BIT);
		} else {
			/* Enable interrupt (Channel status full) */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCSI_BIT, SPDIF_RCSI_BIT);

			/* Enable interrupt (User data full) */
			if (spdif->spdin.u_idx < SPDIF_USER_BUFSZ)
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RUII_BIT, SPDIF_RUII_BIT);

			/* Enable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_REIE_BIT, SPDIF_REIE_BIT);
			if (spdif->is_dma) {
				/* Enable overrun interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABOI_BIT, SPDIF_ABOI_BIT);
				/* Enable DMA receiver */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, SPDIF_RDE_BIT);
			} else
				/* Enable receiver interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
		}
	} else {
		if (rz_spdif_stream_is_play(strm->substream)) {
			/* Disable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT | SPDIF_ABUI_BIT, 0);

			/* Disable interrupt  */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT | SPDIF_TUII_BIT, 0);
			/* Disable DMA transmitter */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, 0);
		} else {
			/* Disable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_REIE_BIT | SPDIF_ABOI_BIT, 0);

			/* Disable interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT | SPDIF_RCSI_BIT |
					SPDIF_RUII_BIT, 0);
			/* Disable DMA receiver */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, 0);
		}
	}

	return 0;
}

static void rz_spdif_quit(struct rz_spdif_stream *strm)
{
	rz_spdif_set_substream(strm, NULL);
}

static int rz_spdif_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	struct rz_spdif_stream *strm = rz_spdif_stream_get(spdif, substream);
	int ret, i, num_transfer = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		/* Set substream to priv data */
		rz_spdif_set_substream(strm, substream);

		ret = rz_spdif_start(spdif, strm);
		if (ret < 0)
			goto dai_trigger_end;

		ret = rz_spdif_irq(spdif, strm, 1);
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
		ret = rz_spdif_irq(spdif, strm, 0);
		rz_spdif_stop(spdif, strm);
		rz_spdif_quit(strm);
		break;
	default:
		ret = -EINVAL;
	}

dai_trigger_end:
	return ret;
}

static int rz_spdif_dai_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = snd_soc_dai_get_drvdata(dai);
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

	return 0;
}

static const struct snd_soc_dai_ops rz_spdif_dai_ops = {
	.trigger = rz_spdif_dai_trigger,
	.hw_params = rz_spdif_dai_hw_params,
};

static const struct snd_pcm_hardware rz_spdif_pcm_hardware = {
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

static int rz_spdif_pcm_new(struct snd_soc_component *component,
			    struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       rtd->card->snd_card->dev,
				       PREALLOC_BUFFER, PREALLOC_BUFFER_MAX);

	return 0;
}

static snd_pcm_uframes_t rz_spdif_pcm_pointer(struct snd_soc_component *component,
					      struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	struct rz_spdif_stream *strm = rz_spdif_stream_get(spdif, substream);

	/* Handle buffer pos for DMA transfer */
	if (spdif->is_dma)
		return strm->dma_buffer_pos;
	return bytes_to_frames(runtime, READ_ONCE(spdif->byte_pos));
}

static int rz_spdif_pcm_open(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rz_spdif_pcm_hardware);

	return snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
}

static struct snd_soc_dai_driver rz_spdif_tx_dai = {
	.name = "rz_spdif_dai_tx",
	.playback = {
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= RZ_SPDIF_RATES,
		.formats	= RZ_SPDIF_FORMATS,
	},
	.ops = &rz_spdif_dai_ops,
};

static struct snd_soc_dai_driver rz_spdif_rx_dai = {
	.name = "rz_spdif_dai_rx",
	.capture = {
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= RZ_SPDIF_RATES,
		.formats	= RZ_SPDIF_FORMATS,
	},
	.ops = &rz_spdif_dai_ops,
};

static const struct snd_soc_component_driver rz_spdif_component = {
	.name		= "rz-spdif",
	.open		= rz_spdif_pcm_open,
	.pointer	= rz_spdif_pcm_pointer,
	.pcm_construct	= rz_spdif_pcm_new,
};

static bool rz_spdif_pio_interrupt(int irq, struct spdif_dev_data *spdif)
{
	struct snd_pcm_runtime *runtime;
	u32 *buf;
	int shift = 0;
	int byte_pos;
	bool elapsed = false;

	if (spdif->playback.substream)
		runtime = spdif->playback.substream->runtime;
	else
		runtime = spdif->capture.substream->runtime;

	buf = (u32 *)(runtime->dma_area + spdif->byte_pos);

	if (snd_pcm_format_width(runtime->format) == 24)
		shift = 8;
	else if (snd_pcm_format_width(runtime->format) == 16)
		shift = 16;

	/*
	 * 16/24 data can be assesse to data register
	 * directly as 32bit data
	 * see rz_spdif_init()
	 */
	if (spdif->playback.substream) {
		do {
			/* Write data to both channel left, right */
			rz_spdif_reg_writel(spdif, SPDIF_TLCA, (*buf) >> shift);
			rz_spdif_reg_writel(spdif, SPDIF_TRCA, (*buf) >> shift);
		} while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CBTX_BIT);
	} else {
		do {
			/* Default record from right channel */
			*buf = (rz_spdif_reg_readl(spdif, SPDIF_RLCA) << shift);
			*buf = (rz_spdif_reg_readl(spdif, SPDIF_RRCA) << shift);
		} while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CBRX_BIT);
	}

	byte_pos = spdif->byte_pos + sizeof(*buf);

	if (byte_pos >= spdif->next_period_byte) {
		int period_pos = byte_pos / spdif->byte_per_period;

		if (period_pos >= runtime->periods) {
			byte_pos = 0;
			period_pos = 0;
		}
		spdif->next_period_byte = (period_pos + 1) * spdif->byte_per_period;

		elapsed = true;
	}

	WRITE_ONCE(spdif->byte_pos, byte_pos);

	return elapsed;
}

static irqreturn_t rz_spdif_irq_handler(int irq, void *arg)
{
	struct spdif_dev_data *spdif = arg;
	struct rz_spdif_stream *strm = NULL;
	u32 stat, udata;
	bool elapsed = false;
	bool error = false;
	int is_play;

	spin_lock(&spdif->lock);
	if (spdif->playback.substream)
		strm = &spdif->playback;
	else if (spdif->capture.substream)
		strm = &spdif->capture;
	else
		return IRQ_HANDLED;

	stat = rz_spdif_status_get(spdif);
	is_play = rz_spdif_stream_is_play(strm->substream);

	/* PIO only */
	if (!spdif->is_dma && ((stat & SPDIF_CBTX_BIT) || (stat & SPDIF_CBRX_BIT)))
		elapsed = rz_spdif_pio_interrupt(irq, spdif);

	if (spdif->is_dma && (stat & SPDIF_ABU_BIT)) {
		/* Clear the error status */
		rz_spdif_bset(spdif, SPDIF_STAT, SPDIF_ABU_BIT, 0);
		error = true;
	}

	if (spdif->is_dma && (stat & SPDIF_ABO_BIT)) {
		/* Clear the error status */
		rz_spdif_bset(spdif, SPDIF_STAT, SPDIF_ABO_BIT, 0);
		error = true;
	}

	/* Receiver channel status interrupt (CSRX) */
	if (stat & SPDIF_CSRX_BIT) {
		do {
			/* Read the status data */
			spdif->spdin.s_buf[SPDIF_CH1] = rz_spdif_reg_readl(spdif, SPDIF_RLCS);
			spdif->spdin.s_buf[SPDIF_CH2] = rz_spdif_reg_readl(spdif, SPDIF_RRCS);
		} while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_CSRX_BIT);
	}

	/* Transmitter user information interrupt (TUIR) */
	if (stat & SPDIF_TUIR_BIT) {
		do
			rz_spdif_reg_writel(spdif, SPDIF_TUI,
					spdif->spdout.u_buf.data32[spdif->spdout.u_idx]);
		while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TUIR_BIT);

		spdif->spdout.u_idx++;
		if (spdif->spdout.u_idx >= SPDIF_USER_BUFSZ) {
			spdif->spdout.u_idx = 0;
			spdif->count--;
		}
		if (spdif->count < 0)
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TUII_BIT, 0);
	}

	/* Receiver user information interrupt (RUIR) */
	if (stat & SPDIF_RUIR_BIT) {
		do
			udata = rz_spdif_reg_readl(spdif, SPDIF_RUI);
		while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RUIR_BIT);

		/* Detect the end and Store the data */
		if (udata == 0)
			spdif->spdin.u_idx = 0;
		else
			spdif->spdin.u_buf.data32[spdif->spdin.u_idx++] = udata;
		if (spdif->spdin.u_idx > SPDIF_USER_BUFSZ)
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RUII_BIT, 0);
	}

	rz_spdif_status_clear(spdif);

	spin_unlock(&spdif->lock);

	/* PIO elapse only */
	if (elapsed)
		snd_pcm_period_elapsed(strm->substream);

	if (error) {
		if (is_play) {
			if (spdif->is_dma)
				/* Disable DMA transmitter */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, 0);
			else
				/* Disable Transmitter interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT, 0);

			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, 0);
			while (!(rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT))
				;
		} else {
			if (spdif->is_dma)
				/* Disable DMA receiver */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, 0);
			else
				/* Disable Receiver interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, 0);
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, 0);
			while (!(rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT))
				;
		}

		rz_spdif_status_clear(spdif);

		/* Retransmit */
		if (is_play) {
			/* Enable transmitter module */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, SPDIF_TME_BIT);
			while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT)
				;

			/* Enable interrupt (User data empty) */
			if (spdif->count > 0)
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TUII_BIT, SPDIF_TUII_BIT);

			/* Enable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT, SPDIF_TEIE_BIT);

			if (spdif->is_dma) {
				/* Enable underrun interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABUI_BIT, SPDIF_ABUI_BIT);
				/* Enable DMA transmitter */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TDE_BIT, SPDIF_TDE_BIT);
			} else
				/* Enable transmitter interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT, SPDIF_TCBI_BIT);
		} else {
			/* Enable receiver module */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, SPDIF_RME_BIT);
			while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT)
				;

			/* Enable interrupt (Channel status full) */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCSI_BIT, SPDIF_RCSI_BIT);

			/* Enable interrupt (User data full) */
			if (spdif->spdin.u_idx < SPDIF_USER_BUFSZ)
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RUII_BIT, SPDIF_RUII_BIT);

			/* Enable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_REIE_BIT, SPDIF_REIE_BIT);

			if (spdif->is_dma) {
				/* Enable overrun interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_ABOI_BIT, SPDIF_ABOI_BIT);
				/* Enable DMA receiver */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RDE_BIT, SPDIF_RDE_BIT);
			} else
				/* Enable Receiver interrupt */
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
		}
	}

	return IRQ_HANDLED;
}

static int rz_spdif_dma_slave_config(struct spdif_dev_data *spdif,
				     struct dma_chan *dma_ch, bool is_play)
{
	struct dma_slave_config cfg;

	memset(&cfg, 0, sizeof(cfg));

	cfg.direction = is_play ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;
	cfg.dst_addr = spdif->phys + SPDIF_TDAD;
	cfg.src_addr = spdif->phys + SPDIF_RDAD;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	return dmaengine_slave_config(dma_ch, &cfg);
}

static int rz_spdif_dma_transfer(struct spdif_dev_data *spdif,
				 struct rz_spdif_stream *strm)
{
	struct snd_pcm_substream *substream = strm->substream;
	struct dma_async_tx_descriptor *desc;
	struct snd_pcm_runtime *runtime;
	enum dma_transfer_direction dir;
	u32 dma_paddr, dma_size;
	int amount;

	if (!rz_spdif_stream_is_valid(spdif, strm))
		return -EINVAL;

	runtime = substream->runtime;
	if (runtime->status->state == SNDRV_PCM_STATE_DRAINING)
		/*
		 * Stream is ending, so do not queue up any more DMA
		 * transfers otherwise we play partial sound clips
		 * because we can't shut off the DMA quick enough.
		 */
		return 0;

	dir = rz_spdif_stream_is_play(substream) ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;

	/* Always transfer 1 period */
	amount = runtime->period_size;

	/* DMA physical address and size */
	dma_paddr = runtime->dma_addr + frames_to_bytes(runtime,
							strm->dma_buffer_pos);
	dma_size = frames_to_bytes(runtime, amount);
	desc = dmaengine_prep_slave_single(strm->dma_ch, dma_paddr, dma_size,
					dir,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(spdif->dev, "dmaengine_prep_slave_single() fail\n");
		return -ENOMEM;
	}

	desc->callback = rz_spdif_dma_complete;
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

static void rz_spdif_dma_complete(void *data)
{
	struct rz_spdif_stream *strm = (struct rz_spdif_stream *)data;

	if (!strm->substream || !strm->substream->runtime)
		return;

	snd_pcm_period_elapsed(strm->substream);
	/* Queue up another DMA transaction */
	rz_spdif_dma_transfer(strm->priv, strm);
}

static void rz_spdif_release_dma_channels(struct spdif_dev_data *spdif)
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

static int rz_spdif_dma_request(struct spdif_dev_data *spdif, struct device *dev)
{
	if (spdif->mode) {
		spdif->playback.dma_ch = dma_request_chan(dev, "tx");
		if (IS_ERR(spdif->playback.dma_ch))
			spdif->playback.dma_ch = NULL;

		if (spdif->playback.dma_ch &&
		   (rz_spdif_dma_slave_config(spdif, spdif->playback.dma_ch, true) < 0))
			goto no_dma;
	} else {
		spdif->capture.dma_ch = dma_request_chan(dev, "rx");
		if (IS_ERR(spdif->capture.dma_ch))
			spdif->capture.dma_ch = NULL;

		if (spdif->capture.dma_ch &&
		   (rz_spdif_dma_slave_config(spdif, spdif->capture.dma_ch, false) < 0))
			goto no_dma;
	}
	return 0;

no_dma:
	rz_spdif_release_dma_channels(spdif);

	return -ENODEV;
}

static int rz_spdif_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *dai_drv;
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

	ret = of_property_read_u32(pdev->dev.of_node, "rz,spdif-mode", &spdif->mode);
	if (ret < 0) {
		dev_err(dev, "cannot get SPDIF mode\n");
		goto probe_err;
	}

	/* Detect DMA support */
	ret = rz_spdif_dma_request(spdif, &pdev->dev);
	if (ret < 0) {
		dev_warn(&pdev->dev, "DMA not available, using PIO\n");
		spdif->is_dma = false;
	} else {
		dev_info(&pdev->dev, "DMA enabled");
		if (spdif->mode) {
			spdif->playback.transfer = rz_spdif_dma_transfer;
			spdif->playback.priv = spdif;
		} else {
			spdif->capture.transfer = rz_spdif_dma_transfer;
			spdif->capture.priv = spdif;
		}

		spdif->is_dma = true;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto probe_err;

	ret = devm_request_irq(dev, ret, rz_spdif_irq_handler,
				0, dev_name(dev), spdif);
	if (ret < 0) {
		dev_err(dev, "spdif irq request failed\n");
		goto probe_err;
	}

	if (spdif->mode)
		dai_drv = &rz_spdif_tx_dai;
	else
		dai_drv = &rz_spdif_rx_dai;

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

	ret = devm_snd_soc_register_component(dev, &rz_spdif_component,
					      dai_drv, 1);
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

static int rz_spdif_remove(struct platform_device *pdev)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(&pdev->dev);

	pm_runtime_put(spdif->dev);
	pm_runtime_disable(spdif->dev);
	reset_control_assert(spdif->rstc);

	return 0;
}

static const struct of_device_id rz_spdif_of_match[] = {
	{ .compatible = "renesas,rz-spdif", },
	{/* Sentinel */},
};
MODULE_DEVICE_TABLE(of, rz_spdif_of_match);

static struct platform_driver rz_spdif_driver = {
	.driver	= {
		.name	= "rz-spdif",
		.of_match_table = rz_spdif_of_match,
	},
	.probe		= rz_spdif_probe,
	.remove		= rz_spdif_remove,
};

module_platform_driver(rz_spdif_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas RZ/G3S ASoC Sony/Philips Digital Interface Format Driver");
MODULE_AUTHOR("Duy Dang <duy.dang.yb@renesas.com>");
