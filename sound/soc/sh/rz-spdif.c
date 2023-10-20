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

#define SPDIF_OK		(0)
#define SPDIF_ERR		(-1)
#define SPDIF_NUMOF_FRM		(192)
#define SPDIF_NUMOF_CH		(2)
#define SPDIF_AUDIO_BUFSZ	(SPDIF_NUMOF_FRM * SPDIF_NUMOF_CH)
#define SPDIF_USER_BUFSZ	((SPDIF_NUMOF_FRM * SPDIF_NUMOF_CH) / 32)
#define SPDIF_CH1		(0)
#define SPDIF_CH2		(1)
/* Pre allocated buffers sizes */
#define PREALLOC_BUFFER		(SZ_32K)
#define PREALLOC_BUFFER_MAX	(SZ_32K)

#define SPDIF_CHAN_MIN		2
#define SPDIF_CHAN_MAX		2

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

	int (*transfer)(struct spdif_dev_data *spdif, struct rz_spdif_stream *strm);
};

struct spdif_dev_data {
	void __iomem *base;
	struct platform_device *pdev;
	struct reset_control *rstc;
	struct device *dev;
	struct clk *clk;

	/* clock */
	unsigned long audio_mck;
	unsigned long audio_clk_1;
	unsigned long audio_clk_2;

	spinlock_t lock;

	u32 mode;
	u32 rate;
	u32 channel;
	u32 bit_width;

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

static int rz_spdif_stop(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	u32 ctrl;

	ctrl = rz_spdif_reg_readl(spdif, SPDIF_CTRL) & (~WIDTH_MASK);
	rz_spdif_reg_writel(spdif, SPDIF_CTRL, ctrl);

	/* Disable SPDIF to idle state */
	if (rz_spdif_stream_is_play(substream)) {
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

static int rz_spdif_config_init(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int width = snd_pcm_format_width(runtime->format);
	u32 channel_status1, channel_status2;

	switch (width) {
	case 16:
		spdif->bit_width = 16;
		break;
	case 24:
		spdif->bit_width = 24;
		break;
	default:
		dev_info(dai->dev, "SPDIF only support 16,24 bit\n");
		return -EINVAL;
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

static int rz_spdif_clk_start(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);

	int chan = runtime->channels;
	unsigned int rate = runtime->rate;

	if (chan != 2) {
		dev_err(dai->dev, "SPDIF only support 2 channel\n");
		return -EIO;
	}

	switch (rate) {
	case 32000:
		spdif->rate = 32000;
		break;
	case 44100:
		spdif->rate = 44100;
		break;
	case 48000:
		spdif->rate = 48000;
		break;
	default:
		dev_err(dai->dev, "SPDIF only support 32,44,48 KHz\n");
		return -EINVAL;
	}

	return 0;
}

static void rz_spdif_register_setup(struct snd_pcm_substream *substream,
							struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);

	/*	Reset the register
	 *	STAT - Status Register
	 *	CTRL - Control Register
	 */

	rz_spdif_reg_writel(spdif, SPDIF_CTRL, 0);
	rz_spdif_reg_writel(spdif, SPDIF_STAT, 0);
}

static snd_pcm_uframes_t rz_spdif_pointer(struct snd_soc_component *component,
						     struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t pointer;
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);

	pointer = bytes_to_frames(runtime, READ_ONCE(spdif->byte_pos));

	return pointer;
}

static int rz_spdif_hw_params(struct snd_soc_component *component,
					struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	unsigned int channels = params_channels(params);
	unsigned int rates = params_rate(params);
	unsigned int fmt_width = snd_pcm_format_width(params_format(params));

	spdif->rate = rates;
	spdif->channel = channels;

	if (fmt_width > 24) {
		dev_err(dai->dev, "invalid combination of slot-width and format-data-width\n");
		return -EINVAL;
	}

	return 0;
}

static int rz_spdif_init(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	int ret;

	ret = rz_spdif_clk_start(substream, dai);
	if (ret < 0)
		return ret;

	ret = rz_spdif_config_init(substream, dai);
	if (ret < 0)
		return ret;

	rz_spdif_register_setup(substream, dai);

	return 0;
}

static int rz_spdif_pio_init(struct snd_pcm_substream *substream,
						struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);

	spdif->byte_pos		= 0;
	spdif->byte_per_period	= runtime->period_size *
				  runtime->channels *
				  samples_to_bytes(runtime, 1);
	spdif->next_period_byte	= spdif->byte_per_period;

	return rz_spdif_init(substream, dai);
}

static int rz_spdif_start(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	u32 ctrl;

	spdif->spdout.u_idx = 0;
	spdif->spdin.u_idx = 0;
	spdif->count = 2;

	if (rz_spdif_stream_is_play(substream)) {
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
	}

	rz_spdif_reg_writel(spdif, SPDIF_CTRL, ctrl);

	return 0;
}

static int rz_spdif_irq(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai, int enable)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);

	if (enable) {
		if (rz_spdif_stream_is_play(substream)) {
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

			/* Enable receiver interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
		}
	} else {
		if (rz_spdif_stream_is_play(substream)) {
			/* Disable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT | SPDIF_ABUI_BIT, 0);

			/* Disable interrupt  */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT | SPDIF_TUII_BIT, 0);
		} else {
			/* Disable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_REIE_BIT | SPDIF_ABOI_BIT, 0);

			/* Disable interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT | SPDIF_RCSI_BIT |
					SPDIF_RUII_BIT, 0);
		}
	}

	return 0;
}

static int rz_spdif_quit(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);

	spdif->bit_width = 0;
	spdif->rate = 0;

	return 0;
}

static int rz_spdif_dai_trigger(struct snd_pcm_substream *substream, int cmd,
					      struct snd_soc_dai *dai)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(dai->dev);
	struct rz_spdif_stream *strm = rz_spdif_stream_get(spdif, substream);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&spdif->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* Set substream to priv data */
		rz_spdif_set_substream(strm, substream);

		ret = rz_spdif_pio_init(substream, dai);
		if (ret < 0)
			goto dai_trigger_end;

		ret = rz_spdif_start(substream, dai);
		if (ret < 0)
			goto dai_trigger_end;

		ret = rz_spdif_irq(substream, dai, 1);
		if (ret < 0)
			goto dai_trigger_end;

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = rz_spdif_irq(substream, dai, 0);
		ret |= rz_spdif_stop(substream, dai);
		ret |= rz_spdif_quit(substream, dai);
		break;
	default:
		ret = -EINVAL;
	}

dai_trigger_end:
	spin_unlock_irqrestore(&spdif->lock, flags);

	return ret;
}

static const struct snd_pcm_hardware rz_spdif_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED	|
				  SNDRV_PCM_INFO_MMAP		|
				  SNDRV_PCM_INFO_MMAP_VALID,
	.buffer_bytes_max	= PREALLOC_BUFFER,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192,
	.channels_min		= 2,
	.channels_max		= 2,
	.periods_min		= 32,
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

static int rz_spdif_pcm_open(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rz_spdif_pcm_hardware);

	return snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
}

static const struct snd_soc_dai_ops rz_spdif_dai_ops = {
	.trigger = rz_spdif_dai_trigger,
};

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
		.channels_min = 2,
		.channels_max = 2,
		.rates = RZ_SPDIF_RATES,
		.formats = RZ_SPDIF_FORMATS,
	},
	.ops = &rz_spdif_dai_ops,
};

static const struct snd_soc_component_driver rz_spdif_component = {
	.name = "rz-spdif",
	.open = rz_spdif_pcm_open,
	.hw_params = rz_spdif_hw_params,
	.pointer = rz_spdif_pointer,
	.pcm_construct = rz_spdif_pcm_new,
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
	u32 stat, ctrl, udata;
	bool elapsed = false;
	bool error = false;

	spin_lock(&spdif->lock);

	stat = rz_spdif_status_get(spdif);
	ctrl = rz_spdif_reg_readl(spdif, SPDIF_CTRL);

	/* PIO only */
	if ((stat & SPDIF_CBTX_BIT) || (stat & SPDIF_CBRX_BIT))
		elapsed = rz_spdif_pio_interrupt(irq, spdif);

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

		/* Store the data */
		if (udata != 0)
			spdif->spdin.u_buf.data32[spdif->spdin.u_idx++] = udata;
		if (spdif->spdin.u_idx > SPDIF_USER_BUFSZ)
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RUII_BIT, 0);
	}

	rz_spdif_status_clear(spdif);

	spin_unlock(&spdif->lock);

	if (elapsed) {
		if (spdif->playback.substream)
			snd_pcm_period_elapsed(spdif->playback.substream);
		else
			snd_pcm_period_elapsed(spdif->capture.substream);
	}

	if (error) {
		if (spdif->playback.substream) {
			/* Disable Transmitter interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TCBI_BIT, 0);
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, 0);
			while (!(rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT))
				;
		} else {
			/* Disable Receiver interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, 0);
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RME_BIT, 0);
			while (!(rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_RIS_BIT))
				;
		}

		rz_spdif_status_clear(spdif);

		/* Retransmit */
		if (spdif->playback.substream) {
			/* Enable transmitter module */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TME_BIT, SPDIF_TME_BIT);
			while (rz_spdif_reg_readl(spdif, SPDIF_STAT) & SPDIF_TIS_BIT)
				;

			/* Enable interrupt (User data empty) */
			if (spdif->count > 0)
				rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TUII_BIT, SPDIF_TUII_BIT);

			/* Enable error interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_TEIE_BIT, SPDIF_TEIE_BIT);

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

			/* Enable Receiver interrupt */
			rz_spdif_bset(spdif, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
		}
	}

	return IRQ_HANDLED;
}

static int rz_spdif_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *dai_drv;
	struct spdif_dev_data *spdif;
	struct clk *audio_clk;
	struct device_node *node;
	struct device *dev;
	int ret;

	spdif = devm_kzalloc(&pdev->dev, sizeof(*spdif), GFP_KERNEL);
	if (!spdif)
		return -ENOMEM;

	spdif->pdev = pdev;
	dev = &pdev->dev;
	node = dev->of_node;

	spdif->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spdif->base))
		return PTR_ERR(spdif->base);

	spdif->clk = devm_clk_get(&pdev->dev, "spdif-tx-rx");
	if (IS_ERR(spdif->clk)) {
		ret = PTR_ERR(spdif->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to get clk spdif-tx-rx: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(spdif->clk);
	if (ret) {
		dev_err(dev, "failed to enable clk spdif-tx-rx: %d\n", ret);
		return ret;
	}

	audio_clk = devm_clk_get(&pdev->dev, "audio_clk1");
	if (IS_ERR(audio_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(audio_clk),
				     "no audio clk1");
	spdif->audio_clk_1 = clk_get_rate(audio_clk);

	audio_clk = devm_clk_get(&pdev->dev, "audio_clk2");
	if (IS_ERR(audio_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(audio_clk),
				     "no audio clk2");
	spdif->audio_clk_2 = clk_get_rate(audio_clk);

	if (!(spdif->audio_clk_1 || spdif->audio_clk_2))
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "no audio clk1 or audio clk2");

	spdif->audio_mck = spdif->audio_clk_1 ? spdif->audio_clk_1 : spdif->audio_clk_2;

	ret = of_property_read_u32(node, "rz,spdif-mode", &spdif->mode);
	if (ret < 0) {
		dev_err(dev, "cannot get SPDIF mode\n");
		goto clk_err;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto clk_err;

	ret = devm_request_irq(dev, ret, rz_spdif_irq_handler,
				0, "RZ_SPDIF_IRQ", spdif);
	if (ret < 0) {
		dev_err(dev, "spdif irq request failed\n");
		goto clk_err;
	}

	if (spdif->mode)
		dai_drv = &rz_spdif_tx_dai;
	else
		dai_drv = &rz_spdif_rx_dai;

	spin_lock_init(&spdif->lock);
	dev_set_drvdata(dev, spdif);

	spdif->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(spdif->rstc))
		return PTR_ERR(spdif->rstc);

	reset_control_deassert(spdif->rstc);
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret < 0) {
		pm_runtime_disable(spdif->dev);
		reset_control_assert(spdif->rstc);
		return dev_err_probe(spdif->dev, ret, "pm_runtime_resume_and_get failed\n");
	}

	ret = devm_snd_soc_register_component(dev, &rz_spdif_component,
					      dai_drv, 1);
	if (ret < 0) {
		pm_runtime_put(spdif->dev);
		pm_runtime_disable(spdif->dev);
		reset_control_assert(spdif->rstc);
		dev_err(&pdev->dev, "failed to register SPDIF snd component\n");
		goto clk_err;
	}

	return ret;

clk_err:
	clk_disable_unprepare(spdif->clk);
	return ret;
}

static int rz_spdif_remove(struct platform_device *pdev)
{
	struct spdif_dev_data *spdif = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(spdif->clk);

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
