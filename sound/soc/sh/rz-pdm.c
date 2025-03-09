// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ PDM Driver
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/iopoll.h>

/* Register offset	*/
/* Control registers	*/
#define PDMm_PDCSTRTR	0x0000
#define PDMm_PDCSTPTR	0x0004
#define PDMm_PDCCHGTR	0x0008
#define PDMm_PDCICR	0x000C
#define PDMm_PDCSR	0x0010
#define PDMm_PDCSCR	0x0014

/* Channel control registers	*/
#define PDMm_PDCSDCR	0x0020
#define PDMm_PDCDRCR	0x0024
#define PDMm_PDCDCR	0x0028

#define PDMm_PDVR	0x0080

/* Common registers	 */
#define PDMm_PDSTRTRCH(n)	(0x0100 + ((n) * 0x0100))
#define PDMm_PDSTPTRCH(n)	(0x0104 + ((n) * 0x0100))
#define PDMm_PDCHGTRCH(n)	(0x0108 + ((n) * 0x0100))
#define PDMm_PDICRCH(n)		(0x010C + ((n) * 0x0100))
#define PDMm_PDSDCRCH(n)	(0x0110 + ((n) * 0x0100))
#define PDMm_PDSRCH(n)		(0x0114 + ((n) * 0x0100))
#define PDMm_PDSCRCH(n)		(0x0118 + ((n) * 0x0100))

/* Mode setting		*/
#define PDMm_PDMDSRCH(n)	(0x0120 + ((n) * 0x0100))
#define PDMm_PDSFCRCH(n)	(0x0124 + ((n) * 0x0100))

/* High-pass filter	*/
#define PDMm_PDHFCxxRCH(n, x)	(0x0128 + ((n) * 0x0100) + ((x) * 4))

/* Compensation filter	xx: 00 -> 10	*/
#define PDMm_PDCFCHxxRCH(n, x)	(0x0138 + ((n) * 0x0100) + ((x) * 4))

/* Low-pass filter	010, 1xx: 100 -> 119	*/
#define PDMm_PDLFCHxxRCH(n, x)	(0x0164 + ((n) * 0x0100) + ((x) * 4))

/* Sound detection lower threshold */
#define PDMm_PDSDLTRCH(n)	(0x01B8 + ((n) * 0x0100))
/* Sound detection upper threshold */
#define PDMm_PDSDUTRCH(n)	(0x01BC + ((n) * 0x0100))

/* Data buffer control */
#define PDMm_PDDBCRCH(n)	(0x01C0 + ((n) * 0x0100))

/* Short circuit threshold */
#define PDMm_PDSCTSRCH(n)	(0x01C4 + ((n) * 0x0100))

/* Overvoltage detection lower threshold */
#define PDMm_PDOVLTRCH(n)	(0x01C8 + ((n) * 0x0100))
/* Overvoltage detection upper threshold */
#define PDMm_PDOVUTRCH(n)	(0x01CC + ((n) * 0x0100))

/* Data read control	*/
#define PDMm_PDDRCRCH(n)	(0x01E0 + ((n) * 0x0100))
/* Data clear		*/
#define PDMm_PDDCRCH(n)		(0x01E4 + ((n) * 0x0100))
/* Data read		*/
#define PDMm_PDDRRCH(n)		(0x01E8 + ((n) * 0x0100))
/* Data status		*/
#define PDMm_PDDSRCH(n)		(0x01EC + ((n) * 0x0100))

/* Register bits */
/* Sinc filter control register bit	*/
#define SFCR_SINCRNG(x)		(((x) & 0x1f) << 24)
#define SFCR_SINCDEC(x)		(((x) & 0xff) << 16)
#define SFCR_CKDIV(x)		(((x) & 0xf) << 0)

/* Mode setting register bit		*/
#define MDSR_DBIS(x)		(((x) & 0xf) << 28)
#define MDSR_SDMAMD(x)		(((x) & 0x3) << 24)
#define MDSR_LFIS(x)		(((x) & 0x3) << 16)
#define MDSR_CFIS(x)		(((x) & 0x3) << 12)
#define MDSR_HFIS(x)		(((x) & 0x3) << 8)
#define MDSR_SFMD(x)		(((x) & 0x7) << 4)
#define MDSR_INPSEL		BIT(0)

/* Status clear register channel	*/
#define BFOWDFC			BIT(27)
#define OVUDFC			BIT(18)
#define OVLDFC			BIT(17)
#define SCDFC			BIT(16)
#define SDFC			BIT(1)

/* Status register channel bit		*/
#define BFOWDF			BIT(27)
#define OVUDF			BIT(18)
#define OVLDF			BIT(17)
#define SCDF			BIT(16)
#define DRF			BIT(2)
#define SDF			BIT(1)
#define STATE			BIT(0)

/* Interrupt control register bit	*/
#define IEDE			BIT(16)
#define IDRE			BIT(2)
#define ISDE			BIT(1)

/* Data read control bit		*/
#define DATRE			BIT(0)

#define PDM_PCM_WIDTH_20_BITS_0_18	0
#define PDM_PCM_WIDTH_20_BITS_1_18	1
#define PDM_PCM_WIDTH_20_BITS_2_18	2
#define PDM_PCM_WIDTH_20_BITS_3_18	3

#define PDM_PCM_WIDTH_16_BITS_4_18	8
#define PDM_PCM_WIDTH_16_BITS_3_17	9
#define PDM_PCM_WIDTH_16_BITS_2_16	10
#define PDM_PCM_WIDTH_16_BITS_1_15	11
#define PDM_PCM_WIDTH_16_BITS_0_14	12

#define MAX_CHANNELS		3
#define MAX_RSTS		2
#define PREALLOC_BUFFER         (SZ_32K)
#define PREALLOC_BUFFER_MAX     (SZ_32K)
#define RZ_PDM_RATES		SNDRV_PCM_RATE_8000_48000
#define RZ_PDM_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_LE)

struct rz_pdm_stream {
	struct rz_pdm_priv *priv;
	struct snd_pcm_substream *substream;

	/* For PIO */
	int period_counter;
	int buffer_pos;

	int (*transfer)(struct rz_pdm_priv *pdm, u32 ch, struct rz_pdm_stream *strm);
};

struct rz_pdm_priv {
	void __iomem *base;
	struct platform_device *pdev;
	struct reset_control *rst[MAX_RSTS];
	struct device *dev;
	struct clk *clk;
	struct clk *sfr_clk;
	struct clk *cclk;

	phys_addr_t phys;
	int irq_sdet;
	int irq_dat;
	int irq_err;

	int is_running; /* 0 = stopped; 1 = running */

	spinlock_t lock;

	unsigned int rate;
	unsigned int width;
	unsigned int channels;
	unsigned int mdsr;

	struct rz_pdm_stream capture;

	/* Channel selection */
	unsigned long ch_mask;
};

struct rz_pdm_irq_desc {
	char *name;
	irqreturn_t (*handler)(int irq, void *data);
};

static const unsigned int rz_pdm_hpf[] = {
	0x3f61, 0x3ec1, 0x4000, 0xc000,
};

static const unsigned int rz_pdm_cpf[] = {
	0x1fe8, 0x0039, 0x003c, 0x1e56, 0x01dc, 0x06e1, 0x01dc, 0x1e56,
	0x003c, 0x0039, 0x1fe8,
};

static const unsigned int rz_pdm_lpf[] = {
	0x0400, 0x1ff8, 0x000a, 0x1ff0, 0x0018, 0x1fdc, 0x0034, 0x1fb3,
	0x0076, 0x1f2e, 0x0289, 0x0289, 0x1f2e, 0x0076, 0x1fb3, 0x0034,
	0x1fdc, 0x0018, 0x1ff0, 0x000a, 0x1ff8,
};

static void rz_pdm_set_substream(struct rz_pdm_stream *strm,
				 struct snd_pcm_substream *substream)
{
	strm->substream = substream;
}

static void rz_pdm_reg_writel(struct rz_pdm_priv *priv, uint reg, u32 data)
{
	writel(data, (priv->base + reg));
}

static u32 rz_pdm_reg_readl(struct rz_pdm_priv *priv, uint reg)
{
	return readl(priv->base + reg);
}

static void rz_pdm_reg_bset(struct rz_pdm_priv *priv, uint reg,
			    u32 bit_mask, u32 bset)
{
	u32 val;

	val = readl(priv->base + reg);
	val = (val & ~bit_mask) | bset;
	writel(val, (priv->base + reg));
}

static bool rz_pdm_stream_is_valid(struct rz_pdm_priv *pdm,
				   struct rz_pdm_stream *strm)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&pdm->lock, flags);
	ret = strm->substream && strm->substream->runtime;
	spin_unlock_irqrestore(&pdm->lock, flags);

	return ret;
}

static void rz_pdm_quit(struct rz_pdm_stream *strm)
{
	rz_pdm_set_substream(strm, NULL);
}

static int rz_pdm_set_clk_out(struct rz_pdm_priv *pdm, int ch)
{
	u32 sfcr = 0;

	/* SFCR_CKDIV should be 0 to let PDM_CLK equal to 1/2 core clock */
	sfcr |= SFCR_CKDIV(0);

	/* Sinc filter decimation ratio and Sinc filter output valid range */
	switch (pdm->rate) {
	case 8000:
		sfcr |= SFCR_SINCDEC(149) | SFCR_SINCRNG(4);
		break;
	case 10000:
		sfcr |= SFCR_SINCDEC(119) | SFCR_SINCRNG(5);
		break;
	case 12000:
		sfcr |= SFCR_SINCDEC(99) | SFCR_SINCRNG(6);
		break;
	case 15000:
		sfcr |= SFCR_SINCDEC(79) | SFCR_SINCRNG(7);
		break;
	case 16000:
		sfcr |= SFCR_SINCDEC(74) | SFCR_SINCRNG(9);
		break;
	case 20000:
		sfcr |= SFCR_SINCDEC(59) | SFCR_SINCRNG(10);
		break;
	case 24000:
		sfcr |= SFCR_SINCDEC(49) | SFCR_SINCRNG(11);
		break;
	case 25000:
		sfcr |= SFCR_SINCDEC(47) | SFCR_SINCRNG(11);
		break;
	case 30000:
		sfcr |= SFCR_SINCDEC(39) | SFCR_SINCRNG(12);
		break;
	case 40000:
		sfcr |= SFCR_SINCDEC(29) | SFCR_SINCRNG(13);
		break;
	case 48000:
		sfcr |= SFCR_SINCDEC(24) | SFCR_SINCRNG(14);
		break;
	default:
		dev_err(pdm->dev, "Not support %d sample rate\n", pdm->rate);
		return -EINVAL;
	}

	/* Sinc filter Control */
	rz_pdm_reg_writel(pdm, PDMm_PDSFCRCH(ch), sfcr);

	return 0;
}

static void rz_pdm_channel_init(struct rz_pdm_priv *pdm, int ch)
{
	int i;

	/* Filter Configuration */
	for (i = 0; i < ARRAY_SIZE(rz_pdm_hpf); i++)
		rz_pdm_reg_writel(pdm, PDMm_PDHFCxxRCH(ch, i), rz_pdm_hpf[i]);
	for (i = 0; i < ARRAY_SIZE(rz_pdm_cpf); i++)
		rz_pdm_reg_writel(pdm, PDMm_PDCFCHxxRCH(ch, i), rz_pdm_cpf[i]);
	for (i = 0; i < ARRAY_SIZE(rz_pdm_lpf); i++)
		rz_pdm_reg_writel(pdm, PDMm_PDLFCHxxRCH(ch, i), rz_pdm_lpf[i]);

	/* Set Channel’s Sound Detection Control */
	rz_pdm_reg_writel(pdm, PDMm_PDSDUTRCH(ch), 0);
	rz_pdm_reg_writel(pdm, PDMm_PDSDLTRCH(ch), 0);
	/* Set Channel’s Data Buffer Control */
	rz_pdm_reg_writel(pdm, PDMm_PDDBCRCH(ch), 0);
	/* Set Channel’s Error Detection Control */
	rz_pdm_reg_writel(pdm, PDMm_PDSCTSRCH(ch), 0x10000FFF);
	rz_pdm_reg_writel(pdm, PDMm_PDOVLTRCH(ch), 0x80000);
	rz_pdm_reg_writel(pdm, PDMm_PDOVUTRCH(ch), 0x7FFFF);
}


static int rz_pdm_start(struct rz_pdm_priv *pdm, struct rz_pdm_stream *strm)
{
	u32 ch, pdscr = 0;

	for_each_set_bit(ch, &pdm->ch_mask, MAX_CHANNELS) {
		/* Activate target channel’s filtering */
		rz_pdm_reg_writel(pdm, PDMm_PDSTRTRCH(ch), 1);
		/* Wait for settling time */
		udelay(4);

		/* Clear Channel’s Status */
		pdscr |= BFOWDFC | OVUDFC | OVLDFC | SCDFC | SDFC;
		rz_pdm_reg_writel(pdm, PDMm_PDSCRCH(ch), pdscr);
		/* Set Channel’s Status Detection Control */
		rz_pdm_reg_bset(pdm, PDMm_PDSDCRCH(ch), SDFC, SDFC);

		/* Set Channel’s Interrupt Control */
		rz_pdm_reg_bset(pdm, PDMm_PDICRCH(ch), IEDE, IEDE);
		rz_pdm_reg_bset(pdm, PDMm_PDICRCH(ch), ISDE, ISDE);
	}
	pdm->is_running = 1;

	return 0;
}

static int rz_pdm_stop(struct rz_pdm_priv *pdm, struct rz_pdm_stream *strm)
{
	u32 ch, pdscr = 0, tmp;
	int ret;

	pdm->is_running = 0;
	for_each_set_bit(ch, &pdm->ch_mask, MAX_CHANNELS) {
		/* Disable Channel’s Data Read */
		rz_pdm_reg_bset(pdm, PDMm_PDDRCRCH(ch), DATRE, 0);

		/* Stop target channel’s filtering */
		rz_pdm_reg_writel(pdm, PDMm_PDSTPTRCH(ch), 1);

		/* Wait for channel stop */
		ret = readl_poll_timeout_atomic(pdm->base + PDMm_PDSRCH(ch),
						tmp, !(tmp & STATE), 1, 100);
		if (ret)
			dev_warn_ratelimited(pdm->dev, "timeout waiting for PDM stop\n");

		/* Disable Channel's Interrupt */
		rz_pdm_reg_writel(pdm, PDMm_PDICRCH(ch), 0);

		/* Disable Channel’s Status Detection */
		rz_pdm_reg_writel(pdm, PDMm_PDSDCRCH(ch), 0);

		/* Clear Channel’s Status */
		pdscr |= BFOWDFC | OVUDFC | OVLDFC | SCDFC | SDFC;
		rz_pdm_reg_writel(pdm, PDMm_PDSCRCH(ch), pdscr);
	}

	return 0;
}

static int rz_pdm_hw_params_setup(struct rz_pdm_priv *pdm, u32 mdsr)
{
	int ret;
	u32 ch;

	for_each_set_bit(ch, &pdm->ch_mask, MAX_CHANNELS) {
		/* Set Channel’s Mode setting */
		rz_pdm_reg_writel(pdm, PDMm_PDMDSRCH(ch), mdsr);
		/* Set Channel’s Digital Filter Configuration */
		rz_pdm_channel_init(pdm, ch);

		/* Change setting of PDM_CLK */
		rz_pdm_reg_writel(pdm, PDMm_PDCHGTRCH(ch), 1);

		/* Clkout setup */
		ret = rz_pdm_set_clk_out(pdm, ch);
		if (ret)
			return ret;
	}

	return 0;
}

static int rz_pdm_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct rz_pdm_priv *pdm = snd_soc_dai_get_drvdata(dai);
	u32 mdsr = 0;

	pdm->rate = params_rate(params);
	pdm->width = params_width(params);
	pdm->channels = params_channels(params);

	if (pdm->channels > MAX_CHANNELS) {
		dev_err(pdm->dev, "%d channels not supported\n", pdm->channels);
		return -EINVAL;
	}

	/*		Mode settings bit mode, filters
	 * DBIS		-	Bit width
	 * SDMAMD	-	Moving average			- 00: Default
	 * LFIS		-	Low-pass filter shift		- 00: No shift
	 * CFIS		-	Compensation filter shift	- 00: No shift
	 * HFIS		-	High-pass filter shift		- 00: No shift
	 * SFMD		-	Sinc filter mode setting	- 00: Default
	 * INPSEL	-	Input data select		- 0: Rise-edge
	 */
	switch (pdm->width) {
	case 16:
		mdsr |= MDSR_DBIS(PDM_PCM_WIDTH_16_BITS_0_14);
		break;
	case 20:
		mdsr |= MDSR_DBIS(PDM_PCM_WIDTH_20_BITS_0_18);
		break;
	default:
		dev_err(pdm->dev, "Not support %d bits width\n", pdm->width);
		return -EINVAL;
	}

	mdsr |= MDSR_SDMAMD(0) | MDSR_LFIS(0) | MDSR_CFIS(0) | MDSR_HFIS(0) | MDSR_SFMD(0);
	mdsr &= ~MDSR_INPSEL;
	pdm->mdsr = mdsr;

	return rz_pdm_hw_params_setup(pdm, mdsr);
}

static void rz_pdm_stream_init(struct rz_pdm_stream *strm,
			       struct snd_pcm_substream *substream)
{
	rz_pdm_set_substream(strm, substream);
	strm->period_counter = 0;
	strm->buffer_pos = 0;
}

static int rz_pdm_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	struct rz_pdm_priv *pdm = dev_get_drvdata(dai->dev);
	struct rz_pdm_stream *strm = &pdm->capture;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		rz_pdm_stream_init(strm, substream);

		ret = rz_pdm_start(pdm, strm);
		if (ret < 0)
			goto dai_trigger_end;

		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		rz_pdm_stop(pdm, strm);

		/* Continue recording in case of suspending */
		if (cmd == SNDRV_PCM_TRIGGER_SUSPEND)
			pdm->is_running = 1;
		else
			rz_pdm_quit(strm);

		break;
	default:
		ret = -EINVAL;
	}

dai_trigger_end:
	return ret;
}


static const struct snd_soc_dai_ops rz_pdm_dai_ops = {
	.hw_params	= rz_pdm_dai_hw_params,
	.trigger	= rz_pdm_dai_trigger,
};

static const struct snd_pcm_hardware rz_pdm_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED	|
				  SNDRV_PCM_INFO_MMAP		|
				  SNDRV_PCM_INFO_MMAP_VALID,
	.buffer_bytes_max	= PREALLOC_BUFFER,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192,
	.channels_min		= 1,
	.channels_max		= MAX_CHANNELS,
	.periods_min		= 1,
	.periods_max		= 32,
	.fifo_size		= 64,
};

static int rz_pdm_pcm_new(struct snd_soc_component *component,
			  struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       rtd->card->snd_card->dev,
				       PREALLOC_BUFFER, PREALLOC_BUFFER_MAX);
	return 0;
}

static snd_pcm_uframes_t rz_pdm_pcm_pointer(struct snd_soc_component *component,
					    struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct rz_pdm_priv *pdm = snd_soc_dai_get_drvdata(dai);
	struct rz_pdm_stream *strm = &pdm->capture;

	return strm->buffer_pos;
}

static int rz_pdm_pcm_open(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rz_pdm_pcm_hardware);

	return snd_pcm_hw_constraint_integer(substream->runtime,
					     SNDRV_PCM_HW_PARAM_PERIODS);
}

static struct snd_soc_dai_driver rz_pdm_rx_dai = {
	.name = "rz_pdm_dai_rx",
	.capture = {
		.rates		= RZ_PDM_RATES,
		.formats	= RZ_PDM_FORMATS,
		.channels_min	= 1,
		.channels_max	= MAX_CHANNELS,
	},
	.ops = &rz_pdm_dai_ops,
};

static const struct snd_soc_component_driver rz_pdm_component = {
	.name		= "rz-pdm",
	.open		= rz_pdm_pcm_open,
	.pointer	= rz_pdm_pcm_pointer,
	.pcm_construct	= rz_pdm_pcm_new,
};

static void rz_pdm_pointer_update(struct rz_pdm_stream *strm, int frames)
{
	struct snd_pcm_substream *substream = strm->substream;
	struct snd_pcm_runtime *runtime;
	int current_period;

	if (!substream || !substream->runtime)
		return;

	runtime = substream->runtime;
	strm->buffer_pos += frames;
	WARN_ON(strm->buffer_pos > runtime->buffer_size);

	/* Reset at the end period */
	if (strm->buffer_pos == runtime->buffer_size)
		strm->buffer_pos = 0;

	current_period = strm->buffer_pos / runtime->period_size;
	if (strm->period_counter != current_period) {
		snd_pcm_period_elapsed(strm->substream);
		strm->period_counter = current_period;
	}
}

static int rz_pdm_pio_rx(struct rz_pdm_priv *pdm, u32 ch, struct rz_pdm_stream *strm)
{
	struct snd_pcm_substream *substream = strm->substream;
	struct snd_pcm_runtime *runtime;
	int fifo_samples, frames_left, samples;
	int i;

	if (!rz_pdm_stream_is_valid(pdm, strm))
		return -EINVAL;

	runtime = substream->runtime;

	do {
		/* Frames left in this period */
		frames_left = runtime->period_size -
			      (strm->buffer_pos % runtime->period_size);
		if (!frames_left)
			frames_left = runtime->period_size;

		/* Samples in data FIFO */
		fifo_samples = rz_pdm_reg_readl(pdm, PDMm_PDDSRCH(ch));

		/* Only read full frames at a time at 1 period */
		samples = 0;
		while (frames_left && (fifo_samples >= runtime->channels)) {
			samples += runtime->channels;
			fifo_samples -= runtime->channels;
			frames_left--;
		}

		/* Not enough samples yet */
		if (!samples)
			break;

		if (pdm->width == 16) {
			u16 *buf;

			buf = (u16 *)runtime->dma_area;
			buf += strm->buffer_pos * runtime->channels;

			for (i = 0; i < samples; i++)
				*buf++ = (u16)(rz_pdm_reg_readl(pdm, PDMm_PDDRRCH(ch))) & 0xFFFF;
		} else {
			u32 *buf;

			buf = (u32 *)runtime->dma_area;
			buf += strm->buffer_pos * runtime->channels;

			for (i = 0; i < samples; i++)
				*buf++ = rz_pdm_reg_readl(pdm, PDMm_PDDRRCH(ch)) & 0xFFFFF;
		}

		rz_pdm_pointer_update(strm, samples / runtime->channels);

	} while (!frames_left && fifo_samples >= runtime->channels);

	return 0;
}

static irqreturn_t rz_pdm_dat_irq_handler(int irq, void *data)
{
	struct rz_pdm_priv *pdm = data;
	struct rz_pdm_stream *strm_capture = &pdm->capture;
	u32 ch, stat;

	if (!strm_capture)
		return IRQ_HANDLED;

	for_each_set_bit(ch, &pdm->ch_mask, MAX_CHANNELS) {
		/* PDM channel's status get */
		stat = rz_pdm_reg_readl(pdm, PDMm_PDSRCH(ch));

		/* Data reception flag */
		if (stat & DRF)
			rz_pdm_pio_rx(pdm, ch, strm_capture);
	}

	return IRQ_HANDLED;
}

static irqreturn_t rz_pdm_err_irq_handler(int irq, void *data)
{
	struct rz_pdm_priv *pdm = data;
	struct rz_pdm_stream *strm_capture = &pdm->capture;
	u32 ch, stat;

	if (!strm_capture)
		return IRQ_HANDLED;

	for_each_set_bit(ch, &pdm->ch_mask, MAX_CHANNELS) {
		/* PDM channel's status get */
		stat = rz_pdm_reg_readl(pdm, PDMm_PDSRCH(ch));

		/* Buffer overwriting flag */
		if (stat & BFOWDF) {
			rz_pdm_reg_bset(pdm, PDMm_PDICRCH(ch), BFOWDFC, BFOWDFC);
			rz_pdm_reg_writel(pdm, PDMm_PDDCRCH(ch), 1);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t rz_pdm_sdet_irq_handler(int irq, void *data)
{
	struct rz_pdm_priv *pdm = data;
	int i;
	u32 ch, stat, fifo_num;

	for_each_set_bit(ch, &pdm->ch_mask, MAX_CHANNELS) {
		/* PDM channel's status get */
		stat = rz_pdm_reg_readl(pdm, PDMm_PDSRCH(ch));

		/* Sound detection flag */
		if (stat & SDF) {
			/* Disable sound detection */
			rz_pdm_reg_bset(pdm, PDMm_PDICRCH(ch), SDF, 0);
			/* Clear the sound detection flag */
			rz_pdm_reg_bset(pdm, PDMm_PDSCRCH(ch), SDFC, SDFC);

			/* Set Channel’s Data Read Enable */
			rz_pdm_reg_bset(pdm, PDMm_PDDRCRCH(ch), DATRE, DATRE);

			/* Data read start processing */
			fifo_num = rz_pdm_reg_readl(pdm, PDMm_PDDSRCH(ch));
			for (i = 0; i < fifo_num; i++)
				rz_pdm_reg_readl(pdm, PDMm_PDDSRCH(ch));

			rz_pdm_reg_writel(pdm, PDMm_PDDCRCH(ch), 1);
			/* Set Channel’s Interrupt Control */
			rz_pdm_reg_bset(pdm, PDMm_PDICRCH(ch), IDRE, IDRE);
		}
	}

	return IRQ_HANDLED;
}

static struct rz_pdm_irq_desc rz_pdm_irqs[MAX_CHANNELS][2] = {
	{ { "int_pdm_err0", rz_pdm_err_irq_handler }, { "int_pdm_dat0", rz_pdm_dat_irq_handler } },
	{ { "int_pdm_err1", rz_pdm_err_irq_handler }, { "int_pdm_dat1", rz_pdm_dat_irq_handler } },
	{ { "int_pdm_err2", rz_pdm_err_irq_handler }, { "int_pdm_dat2", rz_pdm_dat_irq_handler } }
};

static int rz_pdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rz_pdm_priv *pdm;
	struct resource *res;
	struct device_node *of_child;
	int ret, i, j;
	unsigned long channels_mask = 0;
	char name[9] = "channelX";
	char *rst_names[MAX_RSTS] = { "pclk", "cclk" };

	pdm = devm_kzalloc(dev, sizeof(*pdm), GFP_KERNEL);
	if (!pdm)
		return -ENOMEM;

	pdm->dev = dev;
	pdm->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(pdm->base))
		return PTR_ERR(pdm->base);

	pdm->phys = res->start;
	pdm->capture.transfer = rz_pdm_pio_rx;
	pdm->capture.priv = pdm;

	/* Channel Enable Detection */
	for (i = 0; i < MAX_CHANNELS; ++i) {
		name[7] = '0' + i;
		of_child = of_get_child_by_name(pdev->dev.of_node, name);
		if (of_child && of_device_is_available(of_child)) {
			channels_mask |= BIT(i);

			/* Data Interrupt and Error Interrupt per channel */
			for (j = 0; j < 2; ++j) {
				ret = platform_get_irq_byname(pdev, rz_pdm_irqs[i][j].name);
				if (ret < 0)
					return dev_err_probe(dev, ret,
							     "failed to get %s irq\n",
							     rz_pdm_irqs[i][j].name);

				ret = devm_request_irq(dev, ret, rz_pdm_irqs[i][j].handler, 0,
						       rz_pdm_irqs[i][j].name, pdm);
				if (ret)
					return dev_err_probe(dev, ret,
							     "failed to request %s irq\n",
							     rz_pdm_irqs[i][j].name);
			}
		}
		of_node_put(of_child);
	}
	pdm->ch_mask = channels_mask;

	spin_lock_init(&pdm->lock);
	dev_set_drvdata(dev, pdm);

	/* Sound Detection Interrupt */
	pdm->irq_sdet = platform_get_irq_byname(pdev, "int_pdm_sdet");
	if (pdm->irq_sdet < 0)
		goto probe_err;

	ret = devm_request_irq(dev, pdm->irq_sdet, rz_pdm_sdet_irq_handler,
			       IRQF_SHARED, "int_pdm_sdet", pdm);
	if (ret < 0) {
		dev_err(dev, "sound detection irq request failed\n");
		goto probe_err;
	}

	/* Reset control */
	for (i = 0; i < MAX_RSTS; i++) {
		pdm->rst[i] = devm_reset_control_get_optional(dev, rst_names[i]);
		if (IS_ERR(pdm->rst[i]))
			return dev_err_probe(dev, PTR_ERR(pdm->rst[i]),
					     "failed to get %s\n", rst_names[i]);

		reset_control_deassert(pdm->rst[i]);
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_resume_and_get failed\n");
		goto probe_err;
	}

	ret = devm_snd_soc_register_component(dev, &rz_pdm_component,
					      &rz_pdm_rx_dai, 1);
	if (ret < 0) {
		dev_err(dev, "failed to register PDM snd component\n");
		goto probe_err;
	}

	return 0;

probe_err:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	for (i = 0; i < MAX_RSTS; i++)
		reset_control_assert(pdm->rst[i]);

	return ret;
}

static int rz_pdm_remove(struct platform_device *pdev)
{
	struct rz_pdm_priv *pdm = dev_get_drvdata(&pdev->dev);
	int i;

	pm_runtime_put(pdm->dev);
	pm_runtime_disable(pdm->dev);
	for (i = 0; i < MAX_RSTS; i++)
		reset_control_assert(pdm->rst[i]);

	return 0;
}

static const struct of_device_id rz_pdm_of_match[] = {
	{ .compatible = "renesas,rz-pdm", },
	{/* Sentinel */},
};
MODULE_DEVICE_TABLE(of, rz_pdm_of_match);

static int __maybe_unused rz_pdm_suspend(struct device *dev)
{
	struct rz_pdm_priv *pdm = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX_RSTS; i++)
		reset_control_assert(pdm->rst[i]);

	pm_runtime_put_sync(pdm->dev);

	return 0;
}

static int __maybe_unused rz_pdm_resume(struct device *dev)
{
	struct rz_pdm_priv *pdm = dev_get_drvdata(dev);
	int ret, i;

	for (i = 0; i < MAX_RSTS; i++) {
		ret = reset_control_deassert(pdm->rst[i]);
		if (ret)
			return ret;
	}

	pm_runtime_get_sync(pdm->dev);

	/* In case of recording, Re-init hw params */
	if (pdm->is_running)
		return rz_pdm_hw_params_setup(pdm, pdm->mdsr);

	return 0;
}

DEFINE_SIMPLE_DEV_PM_OPS(rz_pdm_pm_ops, rz_pdm_suspend, rz_pdm_resume);

static struct platform_driver rz_pdm_driver = {
	.driver	= {
		.name		= "rz-pdm",
		.of_match_table	= rz_pdm_of_match,
		.pm		= &rz_pdm_pm_ops,
	},
	.probe		= rz_pdm_probe,
	.remove		= rz_pdm_remove,
};

module_platform_driver(rz_pdm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas RZ Pulse Density Modulation Sound Interface Driver");
MODULE_AUTHOR("Nghia Nguyen <nghia.nguyen.xm@renesas.com>");
