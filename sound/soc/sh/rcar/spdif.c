// SPDX-License-Identifier: GPL-2.0
//
// Renesas R-Car SPDIFU/SPDIF support
//
// Copyright (C) 2013 Renesas Solutions Corp.
// LongLuu <long.luu.ur@renesas.com>

/*
 * you can enable below define if you don't need
 * SPDIF interrupt status debug message when debugging
 * see rsnd_dbg_irq_status()
 *
 * #define RSND_DEBUG_NO_IRQ_STATUS 1
 */

#include <sound/simple_card_utils.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <sound/asoundef.h>
#include "rsnd.h"

#define RSND_SPDIF_NAME_SIZE 16
#define SPDIF_NAME "spdif"

/* STAT bits */
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

/* CTRL bits */
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

#define SPDIF_OK		(0)
#define SPDIF_ERR		(-1)
#define SPDIF_NUMOF_FRM		(192)
#define SPDIF_NUMOF_CH		(2)
#define SPDIF_AUDIO_BUFSZ	(SPDIF_NUMOF_FRM * SPDIF_NUMOF_CH)
#define SPDIF_USER_BUFSZ	((SPDIF_NUMOF_FRM * SPDIF_NUMOF_CH) / 32)
#define SPDIF_CH1		(0)
#define SPDIF_CH2		(1)

static uint8_t user_msg[] = "Renesas SPDIF Interface Message";

typedef struct {
	union {
			uint32_t data32[SPDIF_USER_BUFSZ];
			unsigned char data[SPDIF_USER_BUFSZ * 4];
	} u_buf;				/* User Buffer */
	int32_t  u_idx;				/* Index of User data */
	uint32_t s_buf[SPDIF_NUMOF_CH];		/* Channel Status Buffer */
} spdif_t;

struct rsnd_spdif {
	struct rsnd_mod mod;

	u32 flags;
	int irq;
	unsigned int usrcnt;

	u32 rate;
	u32 bit_width;
	spdif_t spdin;
	spdif_t spdout;
	int count;

	/* for PIO */
	int byte_pos;
	int byte_per_period;
	int next_period_byte;

	struct rsnd_kctrl_cfg_s cs_cfg;
	struct rsnd_kctrl_cfg_s ub_cfg;
};

/* flags */
#define RSND_SPDIF_PROBED			(1 << 0)

#define for_each_rsnd_spdif(pos, priv, i)					\
	for (i = 0;							\
	     (i < rsnd_spdif_nr(priv)) &&					\
		((pos) = ((struct rsnd_spdif *)(priv)->spdif + i));		\
	     i++)

#define rsnd_spdif_get(priv, id) ((struct rsnd_spdif *)(priv->spdif) + id)
#define rsnd_spdif_nr(priv) ((priv)->spdif_nr)
#define rsnd_mod_to_spdif(_mod) container_of((_mod), struct rsnd_spdif, mod)
#define rsnd_spdif_is_run_mods(mod, io) \
	(rsnd_spdif_run_mods(io) & (1 << rsnd_mod_id(mod)))

static int rsnd_spdif_is_dma_mode(struct rsnd_mod *mod);

static u32 rsnd_spdif_status_get(struct rsnd_mod *mod)
{
	return rsnd_mod_read(mod, SPDIF_STAT);
}

static void rsnd_spdif_status_clear(struct rsnd_mod *mod)
{
	rsnd_mod_write(mod, SPDIF_STAT, 0);
}

static u32 rsnd_spdif_run_mods(struct rsnd_dai_stream *io)
{
	struct rsnd_mod *spdif_mod = rsnd_io_to_mod_spdif(io);
	u32 mods;

	mods = 1 << rsnd_mod_id(spdif_mod);

	return mods;
}

static int rsnd_spdif_clk_start(struct rsnd_mod *mod,
				     struct rsnd_dai_stream *io)
{
	struct rsnd_priv *priv = rsnd_io_to_priv(io);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct device *dev = rsnd_priv_to_dev(priv);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	int chan = rsnd_runtime_channel_original(io);
	int ret;
	unsigned int clk_rate;
	unsigned int rate = runtime->rate;

	chan = rsnd_channel_normalization(chan);
	if (chan != 2) {
		dev_err(dev, "SPDIF only support 2 channel\n");
		return -EIO;
	}

	if (spdif->usrcnt > 0)
		return 0;

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
		dev_err(dev, "SPDIF only support 32,44,48 KHz\n");
		return -EINVAL;
	}

	clk_rate = rate * 512;

	ret = rsnd_adg_spdif_clk_try_start(mod, clk_rate);
	if (ret < 0) {
		dev_err(dev, "unsupported clock rate\n");
		return ret;
	}

	dev_dbg(dev, "%s outputs %d chan %u Hz\n",
		rsnd_mod_name(mod), chan, rate);

	return 0;
}

static void rsnd_spdif_clk_stop(struct rsnd_mod *mod,
				     struct rsnd_dai_stream *io)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);

	if (spdif->usrcnt > 1)
		return;

	rsnd_adg_spdif_clk_stop(mod);
}

static int rsnd_spdif_config_init(struct rsnd_mod *mod,
				struct rsnd_dai_stream *io)
{
	struct rsnd_dai *rdai = rsnd_io_to_rdai(io);
	struct rsnd_priv *priv = rsnd_rdai_to_priv(rdai);
	struct device *dev = rsnd_priv_to_dev(priv);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	int width;
	u32 channel_status1, channel_status2;

	width = snd_pcm_format_width(runtime->format);
	switch (width) {
	case 16:
		spdif->bit_width = 16;
		break;
	case 24:
		spdif->bit_width = 24;
		break;
	default:
		dev_err(dev, "SPDIF only support 16,24 bit\n");
		return -EINVAL;
	}

	/* Status Register
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

	/* ---- Buffer clear ---- */
	memset(&spdif->spdout, 0, sizeof(spdif_t));
	memset(&spdif->spdin,  0, sizeof(spdif_t));

	/* ---- Initialize User data ---- */
	memcpy(spdif->spdout.u_buf.data, user_msg, sizeof(user_msg));

	spdif->spdout.s_buf[SPDIF_CH1] = channel_status1;
	spdif->spdout.s_buf[SPDIF_CH2] = channel_status2;

	return 0;
}

static void rsnd_spdif_register_setup(struct rsnd_mod *mod,
				struct rsnd_dai_stream *io)
{
	/* Reset the register
	 * STAT - Status Register
	 * CTRL - Control Register
	 */
	rsnd_mod_write(mod, SPDIF_CTRL, 0);
	rsnd_mod_write(mod, SPDIF_STAT, 0);
}

/*
 *	SPDIF mod common functions
 */
static int rsnd_spdif_init(struct rsnd_mod *mod,
			 struct rsnd_dai_stream *io,
			 struct rsnd_priv *priv)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	int ret;

	if (!rsnd_spdif_is_run_mods(mod, io))
		return 0;

	ret = rsnd_spdif_clk_start(mod, io);
	if (ret < 0)
		return ret;

	spdif->usrcnt++;

	rsnd_mod_power_on(mod);

	ret = rsnd_spdif_config_init(mod, io);
	if (ret < 0)
		return ret;

	rsnd_spdif_register_setup(mod, io);

	return 0;
}

static int rsnd_spdif_quit(struct rsnd_mod *mod,
			 struct rsnd_dai_stream *io,
			 struct rsnd_priv *priv)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	struct device *dev = rsnd_priv_to_dev(priv);

	if (!rsnd_spdif_is_run_mods(mod, io))
		return 0;

	if (!spdif->usrcnt) {
		dev_err(dev, "%s usrcnt error\n", rsnd_mod_name(mod));
		return -EIO;
	}

	spdif->bit_width = 0;
	spdif->rate = 0;

	rsnd_mod_power_off(mod);

	spdif->usrcnt--;

	rsnd_spdif_clk_stop(mod, io);

	return 0;
}

static int rsnd_spdif_hw_params(struct rsnd_mod *mod,
			      struct rsnd_dai_stream *io,
			      struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct rsnd_dai *rdai = rsnd_io_to_rdai(io);
	unsigned int fmt_width = snd_pcm_format_width(params_format(params));

	if (fmt_width > rdai->chan_width) {
		struct rsnd_priv *priv = rsnd_io_to_priv(io);
		struct device *dev = rsnd_priv_to_dev(priv);

		dev_err(dev, "invalid combination of slot-width and format-data-width\n");
		return -EINVAL;
	}

	return 0;
}

static int rsnd_spdif_start(struct rsnd_mod *mod,
			  struct rsnd_dai_stream *io,
			  struct rsnd_priv *priv)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	u32 ctrl;

	if (!rsnd_spdif_is_run_mods(mod, io))
		return 0;

	spdif->spdout.u_idx = 0;
	spdif->spdin.u_idx = 0;
	spdif->count = 2;

	if (rsnd_io_is_play(io)) {
		/* ---- Enable Transmitter module ---- */
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TME_BIT, SPDIF_TME_BIT);
		while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_TIS_BIT)
			;
	} else {
		/* ---- Enable Receiver module---- */
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RME_BIT, SPDIF_RME_BIT);
		while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_RIS_BIT)
			;
	}

	/* Setting TASS, RASS */
	ctrl = rsnd_mod_read(mod, SPDIF_CTRL);

	switch (spdif->bit_width) {
	case 16:
		ctrl = (ctrl & (~WIDTH_MASK)) | WIDTH_16;
		break;
	case 24:
		ctrl = (ctrl & (~WIDTH_MASK)) | WIDTH_24;
		break;
	}

	rsnd_mod_write(mod, SPDIF_CTRL, ctrl);

	return 0;
}

static int rsnd_spdif_stop(struct rsnd_mod *mod,
			 struct rsnd_dai_stream *io,
			 struct rsnd_priv *priv)
{
	u32 ctrl;

	if (!rsnd_spdif_is_run_mods(mod, io))
		return 0;

	ctrl = rsnd_mod_read(mod, SPDIF_CTRL) & (~WIDTH_MASK);
	rsnd_mod_write(mod, SPDIF_CTRL, ctrl);

	/* disable SPDIF to idle state */
	if (rsnd_io_is_play(io)) {
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TME_BIT, 0);
		while (!(rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_TIS_BIT))
			;
	} else {
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RME_BIT, 0);
		while (!(rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_RIS_BIT))
			;
	}

	return 0;
}

static int rsnd_spdif_irq(struct rsnd_mod *mod,
			struct rsnd_dai_stream *io,
			struct rsnd_priv *priv,
			int enable)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);

	if (!rsnd_spdif_is_run_mods(mod, io))
		return 0;


	if (enable) {
		if (rsnd_io_is_play(io)) {
			/* ---- Channel status information ---- */
			rsnd_mod_write(mod, SPDIF_TLCS, spdif->spdout.s_buf[SPDIF_CH1]);
			rsnd_mod_write(mod, SPDIF_TRCS, spdif->spdout.s_buf[SPDIF_CH2]);
			while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_CSTX_BIT)
				;

			/* Enable interrupt (User data empty) */
			if (spdif->count > 0)
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TUII_BIT, SPDIF_TUII_BIT);

			/* Enable error interrupt */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TEIE_BIT, SPDIF_TEIE_BIT);

			if (rsnd_spdif_is_dma_mode(mod)) {
				/* Underrun */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_ABUI_BIT, SPDIF_ABUI_BIT);
				/* Enable DMA Transmitter */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TDE_BIT, SPDIF_TDE_BIT);
			} else
				/* Enable Transmitter interrupt */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TCBI_BIT, SPDIF_TCBI_BIT);
		} else {
			/* Enable interrupt (Channel status full) */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RCSI_BIT, SPDIF_RCSI_BIT);

			/* Enable interrupt (User data full) */
			if (spdif->spdin.u_idx < SPDIF_USER_BUFSZ)
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RUII_BIT, SPDIF_RUII_BIT);

			/* Enable error interrupt */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_REIE_BIT, SPDIF_REIE_BIT);

			if (rsnd_spdif_is_dma_mode(mod)) {
				/* Overrun */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_ABOI_BIT, SPDIF_ABOI_BIT);
				/* Enable DMA Receiver */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RDE_BIT, SPDIF_RDE_BIT);
			} else
				/* Enable Receiver interrupt */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
		}
	} else {
		if (rsnd_io_is_play(io)) {
			/* ---- Disable error interrupt ---- */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TEIE_BIT | SPDIF_ABUI_BIT, 0);
			/* ---- Disable interrupt ---- */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TCBI_BIT | SPDIF_TUII_BIT, 0);
			/* ---- Disable DMA ---- */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TDE_BIT, 0);
		} else {
			/* ---- Disable error interrupt ---- */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_REIE_BIT | SPDIF_ABOI_BIT, 0);
			/* ---- Disable interrupt ---- */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RCBI_BIT | SPDIF_RCSI_BIT |
							SPDIF_RUII_BIT, 0);
			/* ---- Disable DMA ---- */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RDE_BIT, 0);
		}
	}

	return 0;
}

static void rsnd_spdif_retransmit(struct rsnd_mod *mod, struct rsnd_dai_stream *io)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);

	if (rsnd_io_is_play(io)) {
		/* ---- Enable Transmitter module ---- */
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TME_BIT, SPDIF_TME_BIT);
		while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_TIS_BIT)
			;

		/* Enable interrupt (User data empty) */
		if (spdif->count > 0)
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TUII_BIT, SPDIF_TUII_BIT);

		/* Enable error interrupt */
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TEIE_BIT, SPDIF_TEIE_BIT);

		if (rsnd_spdif_is_dma_mode(mod)) {
			/* Underrun */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_ABUI_BIT, SPDIF_ABUI_BIT);
			/* Enable DMA Transmitter */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TDE_BIT, SPDIF_TDE_BIT);
		} else
			/* Enable Transmitter interrupt */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TCBI_BIT, SPDIF_TCBI_BIT);
	} else {
		/* ---- Enable Receiver module ---- */
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RME_BIT, SPDIF_RME_BIT);
		while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_RIS_BIT)
			;

		/* Enable interrupt (Channel status full) */
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RCSI_BIT, SPDIF_RCSI_BIT);

		/* Enable interrupt (User data full) */
		if (spdif->spdin.u_idx < SPDIF_USER_BUFSZ)
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RUII_BIT, SPDIF_RUII_BIT);

		/* Enable error interrupt */
		rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_REIE_BIT, SPDIF_REIE_BIT);

		if (rsnd_spdif_is_dma_mode(mod)) {
			/* Overrun */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_ABOI_BIT, SPDIF_ABOI_BIT);
			/* Enable DMA Receiver */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RDE_BIT, SPDIF_RDE_BIT);
		} else
			/* Enable Receiver interrupt */
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RCBI_BIT, SPDIF_RCBI_BIT);
	}
}

static bool rsnd_spdif_pio_interrupt(struct rsnd_mod *mod,
				   struct rsnd_dai_stream *io);
static void __rsnd_spdif_interrupt(struct rsnd_mod *mod,
				 struct rsnd_dai_stream *io)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	int is_dma = rsnd_spdif_is_dma_mode(mod);
	u32 stat, udata;
	bool elapsed = false;
	bool error = false;

	spin_lock(&priv->lock);

	/* ignore all cases if not working */
	if (!rsnd_io_is_working(io))
		goto rsnd_spdif_interrupt_out;

	stat = rsnd_spdif_status_get(mod);

	/* PIO only */
	if (!is_dma && (stat & SPDIF_CBTX_BIT))
		elapsed = rsnd_spdif_pio_interrupt(mod, io);

	if (!is_dma && (stat & SPDIF_CBRX_BIT))
		elapsed = rsnd_spdif_pio_interrupt(mod, io);


	if (is_dma && (stat & SPDIF_ABU_BIT)) {
		/* Clear the error status */
		rsnd_mod_bset(mod, SPDIF_STAT, SPDIF_ABU_BIT, 0);
		error = true;
	}

	if (is_dma && (stat & SPDIF_ABO_BIT)) {
		/* Clear the error status */
		rsnd_mod_bset(mod, SPDIF_STAT, SPDIF_ABO_BIT, 0);
		error = true;
	}

	/* ---- Receiver Channel Status Interrupt (CSRX) ---- */
	if (stat & SPDIF_CSRX_BIT) {
		/* Read the status data */
		do {
			spdif->spdin.s_buf[SPDIF_CH1] = rsnd_mod_read(mod, SPDIF_RLCS);
			spdif->spdin.s_buf[SPDIF_CH2] = rsnd_mod_read(mod, SPDIF_RRCS);
		} while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_CSRX_BIT);
	}

	/* ---- Transmitter User Information interrupt (TUIR) ---- */
	if (stat & SPDIF_TUIR_BIT) {
		do
			rsnd_mod_write(mod, SPDIF_TUI,
					spdif->spdout.u_buf.data32[spdif->spdout.u_idx]);
		while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_TUIR_BIT);

		spdif->spdout.u_idx++;
		if (spdif->spdout.u_idx >= SPDIF_USER_BUFSZ) {
			spdif->spdout.u_idx = 0;
			spdif->count--;
		}

		if (spdif->count < 0)
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TUII_BIT, 0);
	}

	/* ---- Receiver User Information Interrupt (RUIR) ---- */
	if (stat & SPDIF_RUIR_BIT) {
		do
			udata = rsnd_mod_read(mod, SPDIF_RUI);
		while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_RUIR_BIT);
		/* Detect the end and Store the data */
		if (!udata)
			spdif->spdin.u_idx = 0;
		else
			spdif->spdin.u_buf.data32[spdif->spdin.u_idx++] = udata;

		if (spdif->spdin.u_idx >= SPDIF_USER_BUFSZ)
			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RUII_BIT, 0);
	}

	rsnd_spdif_status_clear(mod);
rsnd_spdif_interrupt_out:
	spin_unlock(&priv->lock);

	if (elapsed)
		rsnd_dai_period_elapsed(io);

	if (error) {
		if (rsnd_io_is_play(io)) {
			if (rsnd_spdif_is_dma_mode(mod))
				/* Disable DMA Transmitter */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TDE_BIT, 0);
			else
				/* Disable Transmitter interrupt */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TCBI_BIT, 0);

			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_TME_BIT, 0);
			while (!(rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_TIS_BIT))
				;
		} else {
			if (rsnd_spdif_is_dma_mode(mod))
				/* Disable DMA Receiver */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RDE_BIT, 0);
			else
				/* Disable Receiver interrupt */
				rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RCBI_BIT, 0);

			rsnd_mod_bset(mod, SPDIF_CTRL, SPDIF_RME_BIT, 0);
			while (!(rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_RIS_BIT))
				;
		}

		rsnd_spdif_status_clear(mod);
		rsnd_spdif_retransmit(mod, io);
	}
}

static irqreturn_t rsnd_spdif_interrupt(int irq, void *data)
{
	struct rsnd_mod *mod = data;

	rsnd_mod_interrupt(mod, __rsnd_spdif_interrupt);

	return IRQ_HANDLED;
}

/*
 *		SPDIF PIO
 */

static int rsnd_spdif_cs_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int rsnd_spdif_ub_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = SPDIF_USER_BUFSZ * 4;

	return 0;
}

static int rsnd_kctrl_cs_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct rsnd_kctrl_cfg *cfg = snd_kcontrol_chip(kcontrol);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(cfg->mod);

	ucontrol->value.iec958.status[0] = IEC958_AES0_CON_NOT_COPYRIGHT |
					   IEC958_AES0_CON_EMPHASIS_NONE;
	ucontrol->value.iec958.status[1] = IEC958_AES1_CON_GENERAL;
	ucontrol->value.iec958.status[2] = IEC958_AES2_CON_SOURCE_UNSPEC |
					   IEC958_AES2_CON_CHANNEL_UNSPEC;
	ucontrol->value.iec958.status[3] = IEC958_AES3_CON_FS_NOTID | IEC958_AES3_CON_CLOCK;

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
		ucontrol->value.iec958.status[3] = IEC958_AES3_CON_CLOCK_1000PPM;
		break;
	case CALC_LV1:
		ucontrol->value.iec958.status[3] = IEC958_AES3_CON_CLOCK_50PPM;
		break;
	case CALC_LV3:
		ucontrol->value.iec958.status[3] = IEC958_AES3_CON_CLOCK_VARIABLE;
		break;
	}

	return 0;
}

static int rsnd_kctrl_ub_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct rsnd_kctrl_cfg *cfg = snd_kcontrol_chip(kcontrol);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(cfg->mod);

	memcpy(ucontrol->value.bytes.data, spdif->spdin.u_buf.data, SPDIF_USER_BUFSZ * 4);

	return 0;
}

static int rsnd_kctrl_new_cs(struct rsnd_mod *mod,
		   struct rsnd_dai_stream *io,
		   struct snd_soc_pcm_runtime *rtd,
		   const unsigned char *name,
		   int (*accept)(struct rsnd_dai_stream *io),
		   void (*update)(struct rsnd_dai_stream *io,
				  struct rsnd_mod *mod),
		   struct rsnd_kctrl_cfg *cfg,
		   const char * const *texts,
		   int size,
		   u32 max)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_kcontrol *kctrl;
	struct snd_kcontrol_new knew = {
		.iface		= SNDRV_CTL_ELEM_IFACE_PCM,
		.name		= name,
		.access		= SNDRV_CTL_ELEM_ACCESS_READ |
				  SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info		= rsnd_spdif_cs_info,
		.index		= rtd->num,
		.get		= rsnd_kctrl_cs_get,
	};
	int ret;

	list_for_each_entry(kctrl, &card->controls, list) {
		struct rsnd_kctrl_cfg *c = kctrl->private_data;

		if (c == cfg)
			return 0;
	}

	if (size > RSND_MAX_CHANNELS)
		return -EINVAL;

	kctrl = snd_ctl_new1(&knew, cfg);
	if (!kctrl)
		return -ENOMEM;

	ret = snd_ctl_add(card, kctrl);
	if (ret < 0)
		return ret;

	cfg->texts	= texts;
	cfg->max	= max;
	cfg->size	= size;
	cfg->accept	= accept;
	cfg->update	= update;
	cfg->card	= card;
	cfg->kctrl	= kctrl;
	cfg->io		= io;
	cfg->mod	= mod;

	return 0;
}

static int rsnd_kctrl_new_ub(struct rsnd_mod *mod,
		   struct rsnd_dai_stream *io,
		   struct snd_soc_pcm_runtime *rtd,
		   const unsigned char *name,
		   int (*accept)(struct rsnd_dai_stream *io),
		   void (*update)(struct rsnd_dai_stream *io,
				  struct rsnd_mod *mod),
		   struct rsnd_kctrl_cfg *cfg,
		   const char * const *texts,
		   int size,
		   u32 max)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_kcontrol *kctrl;
	struct snd_kcontrol_new knew = {
		.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name		= name,
		.access		= SNDRV_CTL_ELEM_ACCESS_READ |
				  SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info		= rsnd_spdif_ub_info,
		.index		= rtd->num,
		.get		= rsnd_kctrl_ub_get,
	};
	int ret;

	list_for_each_entry(kctrl, &card->controls, list) {
		struct rsnd_kctrl_cfg *c = kctrl->private_data;

		if (c == cfg)
			return 0;
	}

	if (size > RSND_MAX_CHANNELS)
		return -EINVAL;

	kctrl = snd_ctl_new1(&knew, cfg);
	if (!kctrl)
		return -ENOMEM;

	ret = snd_ctl_add(card, kctrl);
	if (ret < 0)
		return ret;

	cfg->texts	= texts;
	cfg->max	= max;
	cfg->size	= size;
	cfg->accept	= accept;
	cfg->update	= update;
	cfg->card	= card;
	cfg->kctrl	= kctrl;
	cfg->io		= io;
	cfg->mod	= mod;

	return 0;
}

static int rsnd_spdif_pcm_new(struct rsnd_mod *mod,
			    struct rsnd_dai_stream *io,
			    struct snd_soc_pcm_runtime *rtd)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	int ret;

	if (!rsnd_io_is_play(io))
		return 0;

	ret = rsnd_kctrl_new_cs(mod, io, rtd,
			       SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
			       rsnd_kctrl_accept_anytime,
			       NULL,
			       rsnd_kctrl_init_s(&spdif->cs_cfg),
			       NULL, 1, 1);

	if (ret < 0)
		return ret;

	ret = rsnd_kctrl_new_ub(mod, io, rtd,
			       "IEC958 User Data Receive",
			       rsnd_kctrl_accept_runtime,
			       NULL,
			       rsnd_kctrl_init_s(&spdif->ub_cfg),
			       NULL, 1, 1);

	return ret;
}

static int rsnd_spdif_common_probe(struct rsnd_mod *mod,
				 struct rsnd_dai_stream *io,
				 struct rsnd_priv *priv)
{
	struct device *dev = rsnd_priv_to_dev(priv);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	int ret = 0;

	if (!rsnd_flags_has(spdif, RSND_SPDIF_PROBED)) {
		ret = request_irq(spdif->irq,
				  rsnd_spdif_interrupt,
				  IRQF_SHARED,
				  dev_name(dev), mod);

		rsnd_flags_set(spdif, RSND_SPDIF_PROBED);
	}

	return ret;
}

static int rsnd_spdif_common_remove(struct rsnd_mod *mod,
				  struct rsnd_dai_stream *io,
				  struct rsnd_priv *priv)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);

	/* PIO will request IRQ again */
	if (rsnd_flags_has(spdif, RSND_SPDIF_PROBED)) {
		free_irq(spdif->irq, mod);

		rsnd_flags_del(spdif, RSND_SPDIF_PROBED);
	}

	rsnd_dma_detach(io, mod, &io->dma);

	return 0;
}

/*
 *	SPDIF PIO functions
 */
static bool rsnd_spdif_pio_interrupt(struct rsnd_mod *mod,
				   struct rsnd_dai_stream *io)
{
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	u32 *buf = (u32 *)(runtime->dma_area + spdif->byte_pos);
	int shift = 0;
	int byte_pos;
	bool elapsed = false;

	if (snd_pcm_format_width(runtime->format) == 24)
		shift = 8;

	if (snd_pcm_format_width(runtime->format) == 16)
		shift = 16;

	/*
	 * 16/24 data can be assesse to data register
	 * directly as 32bit data
	 * see rsnd_spdif_init()
	 */
	if (rsnd_io_is_play(io)) {
		do {
			/* Write data to both channel left, right */
			rsnd_mod_write(mod, SPDIF_TLCA, (*buf) >> shift);
			rsnd_mod_write(mod, SPDIF_TRCA, (*buf) >> shift);
		} while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_CBTX_BIT);
	} else {
		do {
			/* Default record from right channel */
			*buf = (rsnd_mod_read(mod, SPDIF_RLCA) << shift);
			*buf = (rsnd_mod_read(mod, SPDIF_RRCA) << shift);
		} while (rsnd_mod_read(mod, SPDIF_STAT) & SPDIF_CBRX_BIT);
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

static int rsnd_spdif_pio_init(struct rsnd_mod *mod,
			     struct rsnd_dai_stream *io,
			     struct rsnd_priv *priv)
{
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);

	spdif->byte_pos		= 0;
	spdif->byte_per_period	= runtime->period_size *
				  runtime->channels *
				  samples_to_bytes(runtime, 1);
	spdif->next_period_byte	= spdif->byte_per_period;

	return rsnd_spdif_init(mod, io, priv);
}

static int rsnd_spdif_pio_pointer(struct rsnd_mod *mod,
			    struct rsnd_dai_stream *io,
			    snd_pcm_uframes_t *pointer)
{
	struct rsnd_spdif *spdif = rsnd_mod_to_spdif(mod);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);

	*pointer = bytes_to_frames(runtime, READ_ONCE(spdif->byte_pos));

	return 0;
}

static struct rsnd_mod_ops rsnd_spdif_pio_ops = {
	.name		= SPDIF_NAME,
	.probe		= rsnd_spdif_common_probe,
	.remove		= rsnd_spdif_common_remove,
	.init		= rsnd_spdif_pio_init,
	.quit		= rsnd_spdif_quit,
	.start		= rsnd_spdif_start,
	.stop		= rsnd_spdif_stop,
	.irq		= rsnd_spdif_irq,
	.pointer	= rsnd_spdif_pio_pointer,
	.pcm_new	= rsnd_spdif_pcm_new,
	.hw_params	= rsnd_spdif_hw_params,
	.get_status	= rsnd_mod_get_status,
};

static int rsnd_spdif_dma_probe(struct rsnd_mod *mod,
			      struct rsnd_dai_stream *io,
			      struct rsnd_priv *priv)
{
	int ret;

	ret = rsnd_spdif_common_probe(mod, io, priv);
	if (ret)
		return ret;

	ret = rsnd_dma_attach(io, mod, &io->dma);

	return ret;
}

static int rsnd_spdif_fallback(struct rsnd_mod *mod,
			     struct rsnd_dai_stream *io,
			     struct rsnd_priv *priv)
{
	struct device *dev = rsnd_priv_to_dev(priv);

	/*
	 * fallback to PIO
	 *
	 * SPDIF .probe might be called again.
	 * see
	 *	rsnd_rdai_continuance_probe()
	 */
	mod->ops = &rsnd_spdif_pio_ops;

	dev_info(dev, "%s fallback to PIO mode\n", rsnd_mod_name(mod));

	return 0;
}

static struct dma_chan *rsnd_spdif_dma_req(struct rsnd_dai_stream *io,
					 struct rsnd_mod *mod)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	int is_play = rsnd_io_is_play(io);
	char *name;

	name = is_play ? "rx" : "tx";

	return rsnd_dma_request_channel(rsnd_spdif_of_node(priv),
					SPDIF_NAME, mod, name);
}

static struct rsnd_mod_ops rsnd_spdif_dma_ops = {
	.name		= SPDIF_NAME,
	.dma_req	= rsnd_spdif_dma_req,
	.probe		= rsnd_spdif_dma_probe,
	.remove		= rsnd_spdif_common_remove,
	.init		= rsnd_spdif_init,
	.quit		= rsnd_spdif_quit,
	.start		= rsnd_spdif_start,
	.stop		= rsnd_spdif_stop,
	.irq		= rsnd_spdif_irq,
	.pcm_new	= rsnd_spdif_pcm_new,
	.fallback	= rsnd_spdif_fallback,
	.hw_params	= rsnd_spdif_hw_params,
	.get_status	= rsnd_mod_get_status,
};

static int rsnd_spdif_is_dma_mode(struct rsnd_mod *mod)
{
	return mod->ops == &rsnd_spdif_dma_ops;
}

/*
 *		spdif mod function
 */

struct rsnd_mod *rsnd_spdif_mod_get(struct rsnd_priv *priv, int id)
{
	if (WARN_ON(id < 0 || id >= rsnd_spdif_nr(priv)))
		id = 0;

	return rsnd_mod_get(rsnd_spdif_get(priv, id));
}

int rsnd_spdif_probe(struct rsnd_priv *priv)
{
	struct device_node *node;
	struct device_node *np;
	struct device *dev = rsnd_priv_to_dev(priv);
	struct rsnd_mod_ops *ops;
	struct clk *clk;
	struct reset_control *rstc;
	struct rsnd_spdif *spdif;
	char name[RSND_SPDIF_NAME_SIZE];
	int i, nr, ret;

	node = rsnd_spdif_of_node(priv);
	if (!node)
		return -EINVAL;

	nr = of_get_child_count(node);
	if (!nr) {
		ret = -EINVAL;
		goto rsnd_spdif_probe_done;
	}

	spdif	= devm_kcalloc(dev, nr, sizeof(*spdif), GFP_KERNEL);
	if (!spdif) {
		ret = -ENOMEM;
		goto rsnd_spdif_probe_done;
	}

	priv->spdif	= spdif;
	priv->spdif_nr	= nr;

	i = 0;
	for_each_child_of_node(node, np) {
		if (!of_device_is_available(np))
			goto skip;

		spdif = rsnd_spdif_get(priv, i);

		snprintf(name, RSND_SPDIF_NAME_SIZE, "%s.%d",
			 SPDIF_NAME, i);

		rstc = devm_reset_control_get_optional(dev, name);
		if (IS_ERR(rstc))
			dev_dbg(dev, "failed to get cpg reset\n");

		clk = devm_clk_get(dev, name);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			of_node_put(np);
			goto rsnd_spdif_probe_done;
		}

		spdif->irq = irq_of_parse_and_map(np, 0);
		if (!spdif->irq) {
			ret = -EINVAL;
			of_node_put(np);
			goto rsnd_spdif_probe_done;
		}

		if (of_property_read_bool(np, "pio-transfer"))
			ops = &rsnd_spdif_pio_ops;
		else
			ops = &rsnd_spdif_dma_ops;

		ret = rsnd_mod_init(priv, rsnd_mod_get(spdif), ops, clk, rstc,
				    RSND_MOD_SPDIF, i);
		if (ret) {
			of_node_put(np);
			goto rsnd_spdif_probe_done;
		}
skip:
		i++;
	}

	ret = 0;

rsnd_spdif_probe_done:
	of_node_put(node);

	return ret;
}

void rsnd_spdif_remove(struct rsnd_priv *priv)
{
	struct rsnd_spdif *spdif;
	int i;

	for_each_rsnd_spdif(spdif, priv, i) {
		rsnd_mod_quit(rsnd_mod_get(spdif));
	}
}
