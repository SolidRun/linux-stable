// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RPC-IF core driver
 *
 * Copyright (C) 2018-2019 Renesas Solutions Corp.
 * Copyright (C) 2019 Macronix International Co., Ltd.
 * Copyright (C) 2019-2020 Cogent Embedded, Inc.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <memory/renesas-rpc-if.h>
#include <memory/renesas-xspi-if.h>

/* xSPI Wrapper Configuration Register */
#define XSPI_WRAPCFG		0x0000
#define XSPI_WRAPCFG_CKSFTCS0(val)	(((val) & 0x1f) << 0)
#define XSPI_WRAPCFG_DSSFTCS0(val)	(((val) & 0x1f) << 8)
#define XSPI_WRAPCFG_CKSFTCS1(val)	(((val) & 0x1f) << 16)
#define XSPI_WRAPCFG_DSSFTCS1(val)	(((val) & 0x1f) << 24)

/* xSPI Common Configuration Register */
#define XSPI_COMCFG		0x0004
#define XSPI_COMCFG_OEASTEX	BIT(16)
#define XSPI_COMCFG_OENEGEX	BIT(17)

/* xSPI Bridge Configuration Register */
#define XSPI_BMCFG		0x0008
#define XSPI_BMCFG_WRMD		BIT(0)
#define XSPI_BMCFG_MWRCOMB	BIT(7)
#define XSPI_BMCFG_MWRSIZE(val)	(((val) & 0xff) << 8)
#define XSPI_BMCFG_PREEN	BIT(16)
#define XSPI_BMCFG_CMBTIM(val)	(((val) & 0xff) << 24)

/* xSPI Command Map Configuration Register 0 CS(0/1) */
#define XSPI_CMCFG0CS0		0x0010
#define XSPI_CMCFG0CS1		0x0020
#define XSPI_CMCFG0_FFMT(val)		(((val) & 0x03) << 0)
#define XSPI_CMCFG0_ADDSIZE(val)	(((val) & 0x03) << 2)
#define XSPI_CMCFG0_WPBSTMD	BIT(4)
#define XSPI_CMCFG0_ARYAMD	BIT(5)
#define XSPI_CMCFG0_ADDRPEN(val)	(((val) & 0xff) << 16)
#define XSPI_CMCFG0_ADDRPCD(val)	(((val) & 0xff) << 24)

/* xSPI Command Map Configuration Register 1 CS(0/1) */
#define XSPI_CMCFG1CS0		0x0014
#define XSPI_CMCFG1CS1		0x0024
#define XSPI_CMCFG1_RDCMD(val)	(((val) & 0xffff) << 0)
#define XSPI_CMCFG1_RDCMD_UPPER_BYTE(val)	(((val) & 0xff) << 8)
#define XSPI_CMCFG1_RDLATE(val)	(((val) & 0x1f) << 16)

/* xSPI Command Map Configuration Register 2 CS(0/1) */
#define XSPI_CMCFG2CS0		0x0018
#define XSPI_CMCFG2CS1		0x0028
#define XSPI_CMCFG2_WRCMD(val)	(((val) & 0xffff) << 0)
#define XSPI_CMCFG2_WRCMD_UPPER(val)	(((val) & 0xff) << 8)
#define XSPI_CMCFG2_WRLATE(val)	(((val) & 0x1f) << 16)

/* xSPI Link I/O Configuration Register CS(0/1) */
#define XSPI_LIOCFGCS0		0x0050
#define XSPI_LIOCFGCS1		0x0054
#define XSPI_LIOCFG_PRTMD(val)	(((val) & 0x3ff) << 0)
#define XSPI_LIOCFG_LATEMV	BIT(10)
#define XSPI_LIOCFG_WRMSKMD	BIT(11)
#define XSPI_LIOCFG_CSMIN(val)	(((val) & 0x0f) << 16)
#define XSPI_LIOCFG_CSASTEX	BIT(20)
#define XSPI_LIOCFG_CSNEGEX	BIT(21)
#define XSPI_LIOCFG_SDRDRV	BIT(22)
#define XSPI_LIOCFG_SDRSMPMD	BIT(23)
#define XSPI_LIOCFG_SDRSMPSFT(val)	(((val) & 0x0f) << 24)
#define XSPI_LIOCFG_DDRSMPEX(val)	(((val) & 0x0f) << 28)

/* xSPI Bridge Map Control Register 0 */
#define XSPI_BMCTL0		0x0060
#define XSPI_BMCTL0_CS0ACC(val)	(((val) & 0x03) << 0)
#define XSPI_BMCTL0_CS1ACC(val)	(((val) & 0x03) << 2)

/* xSPI Bridge Map Control Register 1 */
#define XSPI_BMCTL1		0x0064
#define XSPI_BMCTL1_MWRPUSH	BIT(8)
#define XSPI_BMCTL1_PBUFCLR	BIT(10)

/* xSPI Command Manual Control Register 0 */
#define XSPI_CDCTL0		0x0070
#define XSPI_CDCTL0_TRREQ	BIT(0)
#define XSPI_CDCTL0_PERMD	BIT(1)
#define XSPI_CDCTL0_CSSEL	BIT(3)
#define XSPI_CDCTL0_TRNUM(val)	(((val) & 0x03) << 4)
#define XSPI_CDCTL0_PERITV(val)	(((val) & 0x1f) << 16)
#define XSPI_CDCTL0_PERREP(val)	(((val) & 0x0f) << 24)

/* xSPI Command Manual Control Register 1 */
#define XSPI_CDCTL1		0x0074

/* xSPI Command Manual Control Register 2 */
#define XSPI_CDCTL2		0x0078

/* xSPI Command Manual Type Buf 0/1/2/3 */
#define XSPI_CDTBUF0		0x0080
#define XSPI_CDTBUF1		0x0090
#define XSPI_CDTBUF2		0x00A0
#define XSPI_CDTBUF3		0x00B0
#define XSPI_CDTBUF_CMDSIZE(val)	(((val) & 0x03) << 0)
#define XSPI_CDTBUF_ADDSIZE(val)	(((val) & 0x07) << 2)
#define XSPI_CDTBUF_DATASIZE(val)	(((val) & 0x0f) << 5)
#define XSPI_CDTBUF_LATE(val)		(((val) & 0x1f) << 9)
#define XSPI_CDTBUF_TRTYPE	BIT(15)
#define XSPI_CDTBUF_CMD(val)		(((val) & 0xffff) << 16)
#define XSPI_CDTBUF_CMD_FIELD(val)	(((val) & 0xff) << 24)
#define XSPI_CDTBUF_EXCMD_FIELD(val)	(((val) & 0xff) << 16)


/* xSPI Command Manual Address Buff 0/1/2/3 */
#define XSPI_CDABUF0		0x0084
#define XSPI_CDABUF1		0x0094
#define XSPI_CDABUF2		0x00A4
#define XSPI_CDABUF3		0x00B4

/* xSPI Command Manual Data 0 Buf 0/1/2/3 */
#define XSPI_CDD0BUF0		0x0088
#define XSPI_CDD0BUF1		0x0098
#define XSPI_CDD0BUF2		0x00A8
#define XSPI_CDD0BUF3		0x00B8

/* xSPI Command Calibration Control Register 0 CS(0/1) */
#define XSPI_CCCTL0CS0		0x0130
#define XSPI_CCCTL0CS1		0x0150
#define XSPI_CCCTL0_CAEN	BIT(0)
#define XSPI_CCCTL0_CANOWR	BIT(1)
#define XSPI_CCCTL0_CAITV(val)		(((val) & 0x1f) << 8)
#define XSPI_CCCTL0_CASFTSTA(val)	(((val) & 0x1f) << 16)
#define XSPI_CCCTL0_CASFTEND(val)	(((val) & 0x1f) << 24)

/* xSPI Command Calibration Control Register 1 CS(0/1) */
#define XSPI_CCCTL1CS0		0x0134
#define XSPI_CCCTL1CS1		0x0154
#define XSPI_CCCTL1_CACMDSIZE(val)	(((val) & 0x03) << 0)
#define XSPI_CCCTL1_CAADDSIZE(val)	(((val) & 0x07) << 2)
#define XSPI_CCCTL1_CADARASIZE(val)	(((val) & 0x0f) << 5)
#define XSPI_CCCTL1_CAWRLATE(val)	(((val) & 0x1f) << 16)
#define XSPI_CCCTL1_CARDLATE(val)	(((val) & 0x1f) << 24)

/* xSPI Command Calibration Control Register 2/3/4/5/6/7 CS(0/1) */
#define XSPI_CCCTL2CS0		0x0138
#define XSPI_CCCTL2CS1		0x0158
#define XSPI_CCCTL3CS0		0x013C
#define XSPI_CCCTL3CS1		0x015C
#define XSPI_CCCTL4CS0		0x0140
#define XSPI_CCCTL4CS1		0x0160
#define XSPI_CCCTL5CS0		0x0144
#define XSPI_CCCTL5CS1		0x0164
#define XSPI_CCCTL6CS0		0x0148
#define XSPI_CCCTL6CS1		0x0168
#define XSPI_CCCTL7CS0		0x014C
#define XSPI_CCCTL7CS1		0x016C

/* xSPI Common Status Register */
#define XSPI_COMSTT		0x0184
#define XSPI_COMSTT_MEMACC	BIT(0)
#define XSPI_COMSTT_PBUFNE	BIT(4)
#define XSPI_COMSTT_WRBUFNE	BIT(6)

/* xSPI Interrupt Status Register */
#define XSPI_INTS		0x0190
#define XSPI_INTS_CMDCMP	BIT(0)
#define XSPI_INTS_PATCPM	BIT(1)
#define XSPI_INTS_INICPM	BIT(2)
#define XSPI_INTS_PERTO		BIT(3)
#define XSPI_INTS_DSTOCS0	BIT(4)
#define XSPI_INTS_DSTOCS1	BIT(5)
#define XSPI_INTS_BUSERR	BIT(20)
#define XSPI_INTS_CAFAILCS0	BIT(28)
#define XSPI_INTS_CAFAILCS1	BIT(29)
#define XSPI_INTS_CASUCCS0	BIT(30)
#define XSPI_INTS_CASUCCS1	BIT(31)

/* xSPI Interrupt Clear Register */
#define XSPI_INTC		0x0194
#define XSPI_INTC_CMDCMPC	BIT(0)
#define XSPI_INTC_CPATCMPC	BIT(1)
#define XSPI_INTC_INICMPC	BIT(2)
#define XSPI_INTC_PERTOC	BIT(3)
#define XSPI_INTC_DSTOCS0C	BIT(4)
#define XSPI_INTC_DSTOCS1C	BIT(5)
#define XSPI_INTC_BUSERRCH0	BIT(20)
#define XSPI_INTC_CAFAILCS0C	BIT(28)
#define XSPI_INTC_CAFAILCS1C	BIT(29)
#define XSPI_INTC_CASUCCS0C	BIT(30)
#define XSPI_INTC_CASUCCS1C	BIT(31)

/* xSPI Interrupt Enable Register */
#define XSPI_INTE		0x0198
#define XSPI_INTE_CMDCMPE	BIT(0)
#define XSPI_INTE_PATCMPE	BIT(1)
#define XSPI_INTE_INICMPE	BIT(2)
#define XSPI_INTE_PERTOE	BIT(3)
#define XSPI_INTE_DSTOCS0E	BIT(4)
#define XSPI_INTE_DSTOCS1E	BIT(5)
#define XSPI_INTE_BUSERRE	BIT(20)
#define XSPI_INTE_CAFAILCS0E	BIT(28)
#define XSPI_INTE_CAFAILCS1E	BIT(29)
#define XSPI_INTE_CASUCCS0E	BIT(30)
#define XSPI_INTE_CASUCCS1E	BIT(31)

/* Maximum data size of MWRSIZE*/
#define MWRSIZE_MAX		64

/* xSPI Protocol mode */
#define PROTO_1S_4S_4S		0x090
#define PROTO_4S_4S_4S		0x092

static const struct regmap_range xspi_volatile_ranges[] = {
	regmap_reg_range(XSPI_CDD0BUF0, XSPI_CDD0BUF0),
};

static const struct regmap_access_table xspi_volatile_table = {
	.yes_ranges	= xspi_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(xspi_volatile_ranges),
};

static int xspi_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct rpcif *xspi = context;

	switch (reg) {
	case XSPI_CDD0BUF0:
		switch (xspi->xfer_size) {
		case 1:
			*val = readb(xspi->base + reg);
			return 0;

		case 2:
			*val = readw(xspi->base + reg);
			return 0;

		case 4:
		case 8:
			*val = readl(xspi->base + reg);
			return 0;

		default:
			return -EILSEQ;
		}
	}

	*val = readl(xspi->base + reg);
	return 0;

}

static int xspi_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct rpcif *xspi = context;

	switch (reg) {
	case XSPI_CDD0BUF0:
		switch (xspi->xfer_size) {
		case 1:
			writeb(val, xspi->base + reg);
			return 0;

		case 2:
			writew(val, xspi->base + reg);
			return 0;

		case 4:
		case 8:
			writel(val, xspi->base + reg);
			return 0;

		default:
			return -EILSEQ;
		}

	}

	writel(val, xspi->base + reg);
	return 0;
}

static const struct regmap_config xspi_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.reg_read	= xspi_reg_read,
	.reg_write	= xspi_reg_write,
	.fast_io	= true,
	.max_register	= XSPI_INTE,
	.volatile_table	= &xspi_volatile_table,
};

int xspi_sw_init(struct rpcif *xspi, struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;

	xspi->dev = dev;

	xspi->base = devm_platform_ioremap_resource_byname(pdev, "regs");
	if (IS_ERR(xspi->base))
		return PTR_ERR(xspi->base);

	xspi->regmap = devm_regmap_init(&pdev->dev, NULL, xspi, &xspi_regmap_config);
	if (IS_ERR(xspi->regmap)) {
		dev_err(&pdev->dev,
			"failed to init regmap for rpcif, error %ld\n",
			PTR_ERR(xspi->regmap));
		return	PTR_ERR(xspi->regmap);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dirmap");
	xspi->dirmap = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(xspi->dirmap))
		return PTR_ERR(xspi->dirmap);
	xspi->size = resource_size(res);

	xspi->type = (uintptr_t)of_device_get_match_data(dev);
	xspi->rstc = devm_reset_control_array_get_exclusive(&pdev->dev);

	return PTR_ERR_OR_ZERO(xspi->rstc);
}
EXPORT_SYMBOL(xspi_sw_init);

int xspi_hw_init(struct rpcif *xspi, bool hyperflash)
{
	pm_runtime_get_sync(xspi->dev);

	reset_control_reset(xspi->rstc);

	regmap_write(xspi->regmap, XSPI_WRAPCFG, 0x0);

	regmap_update_bits(xspi->regmap, XSPI_LIOCFGCS0,
			XSPI_LIOCFG_PRTMD(0x3ff) | XSPI_LIOCFG_CSMIN(0xf) |
			XSPI_LIOCFG_CSASTEX | XSPI_LIOCFG_CSNEGEX,
			XSPI_LIOCFG_PRTMD(0) | XSPI_LIOCFG_CSMIN(0) |
			XSPI_LIOCFG_CSASTEX | XSPI_LIOCFG_CSNEGEX);

	regmap_update_bits(xspi->regmap, XSPI_CCCTL0CS0, XSPI_CCCTL0_CAEN, 0);

	regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
			XSPI_CDCTL0_TRREQ | XSPI_CDCTL0_CSSEL, 0);

	regmap_update_bits(xspi->regmap, XSPI_INTE, XSPI_INTE_CMDCMPE,
			XSPI_INTE_CMDCMPE);

	return 0;
}
EXPORT_SYMBOL(xspi_hw_init);

static int wait_msg_xfer_end(struct rpcif *xspi)
{
	u32 sts;

	return regmap_read_poll_timeout(xspi->regmap, XSPI_INTS, sts,
					sts & XSPI_INTS_CMDCMP, 0,
					USEC_PER_SEC);
}

void xspi_prepare(struct rpcif *xspi, const struct rpcif_op *op, u64 *offs,
		   size_t *len)
{
	xspi->smadr = 0;
	xspi->addr_nbytes = 0;
	xspi->command = 0;
	xspi->option = 0;
	xspi->dummy = 0;
	xspi->xferlen = 0;
	xspi->proto = 0;

	if (op->cmd.buswidth)
		xspi->command = op->cmd.opcode;

	if (op->ocmd.buswidth)
		xspi->command = (xspi->command << 8) | op->ocmd.opcode;

	if (op->addr.buswidth) {
		xspi->addr_nbytes = op->addr.nbytes;
		if (offs && len)
			xspi->smadr = *offs;
		else
			xspi->smadr = op->addr.val;
	}

	if (op->dummy.buswidth)
		xspi->dummy = op->dummy.ncycles;

	xspi->dir = op->data.dir;
	if (op->data.buswidth) {
		u32 nbytes;

		xspi->buffer = op->data.buf.in;

		if (offs && len)
			nbytes = *len;
		else
			nbytes = op->data.nbytes;
		xspi->xferlen = nbytes;
	}

	if (op->cmd.buswidth == 1 &&
			(op->addr.buswidth == 4 || op->data.buswidth == 4))
		xspi->proto = PROTO_1S_4S_4S;
	else if (op->cmd.buswidth == 4 &&
			(op->addr.buswidth == 4 || op->data.buswidth == 4))
		xspi->proto = PROTO_4S_4S_4S;
}
EXPORT_SYMBOL(xspi_prepare);

int xspi_manual_xfer(struct rpcif *xspi)
{
	u32 pos = 0, max = xspi->bus_size == 2 ? 8 : 4;
	int ret = 0;

	pm_runtime_get_sync(xspi->dev);

	regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
			XSPI_CDCTL0_TRNUM(0x3),	XSPI_CDCTL0_TRNUM(0));

	regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
			XSPI_CDCTL0_TRREQ, 0);

	regmap_write(xspi->regmap, XSPI_CDTBUF0,
			XSPI_CDTBUF_CMDSIZE(0x1) |
			XSPI_CDTBUF_CMD_FIELD(xspi->command));

	regmap_write(xspi->regmap, XSPI_CDABUF0, 0);

	regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
			XSPI_CDTBUF_ADDSIZE(0x7),
			XSPI_CDTBUF_ADDSIZE(xspi->addr_nbytes));

	regmap_write(xspi->regmap, XSPI_CDABUF0, xspi->smadr);

	regmap_update_bits(xspi->regmap, XSPI_LIOCFGCS0, XSPI_LIOCFG_PRTMD(0x3ff),
			XSPI_LIOCFG_PRTMD(xspi->proto));

	switch (xspi->dir) {
	case RPCIF_DATA_OUT:
		while (pos < xspi->xferlen) {
			u32 bytes_left = xspi->xferlen - pos;
			u32 nbytes, data[2], *p = data;

			regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
					XSPI_CDTBUF_TRTYPE, XSPI_CDTBUF_TRTYPE);

			/* nbytes may only be 1, 2, 4, or 8 */
			nbytes = bytes_left >= max ? max : (1 << ilog2(bytes_left));

			regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
					XSPI_CDTBUF_DATASIZE(0xf),
					XSPI_CDTBUF_DATASIZE(nbytes));

			regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
					XSPI_CDTBUF_ADDSIZE(0x7),
					XSPI_CDTBUF_ADDSIZE(xspi->addr_nbytes));

			xspi->xfer_size = nbytes;

			memcpy(data, xspi->buffer + pos, nbytes);
			regmap_write(xspi->regmap, XSPI_CDD0BUF0, *p);
			regmap_write(xspi->regmap, XSPI_CDABUF0, xspi->smadr + pos);

			regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
					XSPI_CDCTL0_TRREQ, XSPI_CDCTL0_TRREQ);

			ret = wait_msg_xfer_end(xspi);
			if (ret)
				goto err_out;

			regmap_update_bits(xspi->regmap, XSPI_INTC,
					XSPI_INTC_CMDCMPC, XSPI_INTC_CMDCMPC);

			pos += nbytes;
		}
		regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
				XSPI_CDCTL0_TRREQ, 0);
		break;
	case RPCIF_DATA_IN:
		while (pos < xspi->xferlen) {
			u32 bytes_left = xspi->xferlen - pos;
			u32 nbytes, data[2], *p = data;

			regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
					XSPI_CDTBUF_TRTYPE, ~(u32)XSPI_CDTBUF_TRTYPE);

			/* nbytes may only be 1, 2, 4, or 8 */
			nbytes = bytes_left >= max ? max : (1 << ilog2(bytes_left));

			regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
					XSPI_CDTBUF_DATASIZE(0xf),
					XSPI_CDTBUF_DATASIZE(nbytes));

			regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
					XSPI_CDTBUF_ADDSIZE(0x7),
					XSPI_CDTBUF_ADDSIZE(xspi->addr_nbytes));

			xspi->xfer_size = nbytes;

			if (xspi->addr_nbytes)
				regmap_write(xspi->regmap, XSPI_CDABUF0, xspi->smadr + pos);

			regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
					XSPI_CDTBUF_LATE(0x1f),
					XSPI_CDTBUF_LATE(xspi->dummy));

			regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
					XSPI_CDCTL0_TRREQ, XSPI_CDCTL0_TRREQ);

			ret = wait_msg_xfer_end(xspi);
			if (ret)
				goto err_out;

			regmap_read(xspi->regmap, XSPI_CDD0BUF0, p);
			memcpy(xspi->buffer + pos, data, nbytes);

			regmap_update_bits(xspi->regmap, XSPI_INTC,
					XSPI_INTC_CMDCMPC, XSPI_INTC_CMDCMPC);

			pos += nbytes;
		}
		regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
				XSPI_CDCTL0_TRREQ, 0);
		break;
	default:
		regmap_update_bits(xspi->regmap, XSPI_CDTBUF0,
				XSPI_CDTBUF_TRTYPE, XSPI_CDTBUF_TRTYPE);
		regmap_update_bits(xspi->regmap, XSPI_CDCTL0,
				XSPI_CDCTL0_TRREQ, XSPI_CDCTL0_TRREQ);

		ret = wait_msg_xfer_end(xspi);
		if (ret)
			goto err_out;

		regmap_update_bits(xspi->regmap, XSPI_INTC,
				XSPI_INTC_CMDCMPC, XSPI_INTC_CMDCMPC);
	}

exit:
	pm_runtime_put(xspi->dev);
	return ret;

err_out:
	if (reset_control_reset(xspi->rstc))
		dev_err(xspi->dev, "Failed to reset HW\n");
	xspi_hw_init(xspi, 0);
	goto exit;
}
EXPORT_SYMBOL(xspi_manual_xfer);

ssize_t xspi_dirmap_write(struct rpcif *xspi, u64 offs, size_t len, const void *buf)
{
	loff_t from = offs & (xspi->size - 1);
	size_t size = xspi->size - from;
	u8 addsize = xspi->addr_nbytes - 1;
	u32 writebytes;

	if (len > size)
		len = size;

	if (len > MWRSIZE_MAX)
		writebytes = MWRSIZE_MAX;
	else
		writebytes = len;

	pm_runtime_get_sync(xspi->dev);

	regmap_update_bits(xspi->regmap, XSPI_CMCFG0CS0,
			XSPI_CMCFG0_FFMT(0x3) | XSPI_CMCFG0_ADDSIZE(0x3),
			XSPI_CMCFG0_FFMT(0) | XSPI_CMCFG0_ADDSIZE(addsize));

	regmap_update_bits(xspi->regmap, XSPI_CMCFG2CS0,
			XSPI_CMCFG2_WRCMD_UPPER(0xff) | XSPI_CMCFG2_WRLATE(0x1f),
			XSPI_CMCFG2_WRCMD_UPPER(xspi->command) |
			XSPI_CMCFG2_WRLATE(xspi->dummy));

	regmap_update_bits(xspi->regmap, XSPI_BMCTL0,
			XSPI_BMCTL0_CS0ACC(0xff), XSPI_BMCTL0_CS0ACC(0x03));

	regmap_update_bits(xspi->regmap, XSPI_BMCFG,
			XSPI_BMCFG_WRMD | XSPI_BMCFG_MWRCOMB |
			XSPI_BMCFG_MWRSIZE(0xff) | XSPI_BMCFG_PREEN,
			0 | XSPI_BMCFG_MWRCOMB | XSPI_BMCFG_MWRSIZE(0x0f) |
			XSPI_BMCFG_PREEN);

	regmap_update_bits(xspi->regmap, XSPI_LIOCFGCS0, XSPI_LIOCFG_PRTMD(0x3ff),
			XSPI_LIOCFG_PRTMD(xspi->proto));

	memcpy_toio(xspi->dirmap + from, buf, writebytes);

	/* Request to push the pending data */
	if (writebytes < MWRSIZE_MAX)
		regmap_update_bits(xspi->regmap, XSPI_BMCTL1,
				XSPI_BMCTL1_MWRPUSH, XSPI_BMCTL1_MWRPUSH);

	pm_runtime_put(xspi->dev);

	return writebytes;
}
EXPORT_SYMBOL(xspi_dirmap_write);

ssize_t xspi_dirmap_read(struct rpcif *xspi, u64 offs, size_t len, void *buf)
{
	loff_t from = offs & (xspi->size - 1);
	size_t size = xspi->size - from;
	u8 addsize = xspi->addr_nbytes - 1;

	if (len > size)
		len = size;

	pm_runtime_get_sync(xspi->dev);

	regmap_update_bits(xspi->regmap, XSPI_CMCFG0CS0,
			XSPI_CMCFG0_FFMT(0x3) | XSPI_CMCFG0_ADDSIZE(0x3),
			XSPI_CMCFG0_FFMT(0) | XSPI_CMCFG0_ADDSIZE(addsize));

	regmap_update_bits(xspi->regmap, XSPI_CMCFG1CS0,
			XSPI_CMCFG1_RDCMD(0xffff) | XSPI_CMCFG1_RDLATE(0x1f),
			XSPI_CMCFG1_RDCMD_UPPER_BYTE(xspi->command) |
			XSPI_CMCFG1_RDLATE(xspi->dummy));

	regmap_update_bits(xspi->regmap, XSPI_BMCTL0,
			XSPI_BMCTL0_CS0ACC(0xff), XSPI_BMCTL0_CS0ACC(0x01));

	regmap_update_bits(xspi->regmap, XSPI_BMCFG,
			XSPI_BMCFG_WRMD | XSPI_BMCFG_MWRCOMB |
			XSPI_BMCFG_MWRSIZE(0xff) | XSPI_BMCFG_PREEN,
			0 | XSPI_BMCFG_MWRCOMB | XSPI_BMCFG_MWRSIZE(0x0f) |
			XSPI_BMCFG_PREEN);

	regmap_update_bits(xspi->regmap, XSPI_LIOCFGCS0, XSPI_LIOCFG_PRTMD(0x3ff),
			XSPI_LIOCFG_PRTMD(xspi->proto));

	memcpy_fromio(buf, xspi->dirmap + from, len);

	pm_runtime_put(xspi->dev);

	return len;
}
EXPORT_SYMBOL(xspi_dirmap_read);

static int xspi_probe(struct platform_device *pdev)
{
	struct platform_device *vdev;
	struct device_node *flash;
	const char *name;
	int ret;

	flash = of_get_next_child(pdev->dev.of_node, NULL);
	if (!flash) {
		dev_warn(&pdev->dev, "no flash node found\n");
		return -ENODEV;
	}

	if (of_device_is_compatible(flash, "jedec,spi-nor")) {
		name = "rpc-if-spi";
	} else	{
		of_node_put(flash);
		dev_warn(&pdev->dev, "unknown flash type\n");
		return -ENODEV;
	}
	of_node_put(flash);

	vdev = platform_device_alloc(name, pdev->id);
	if (!vdev)
		return -ENOMEM;
	vdev->dev.parent = &pdev->dev;
	platform_set_drvdata(pdev, vdev);

	ret = platform_device_add(vdev);
	if (ret) {
		platform_device_put(vdev);
		return ret;
	}

	return 0;
}

static int xspi_remove(struct platform_device *pdev)
{
	struct platform_device *vdev = platform_get_drvdata(pdev);

	platform_device_unregister(vdev);

	return 0;
}

static const struct of_device_id xspi_of_match[] = {
	{ .compatible = "renesas,g3s-xspi-if", .data = (void *)XSPI_RZ_G3S },
	{},
};
MODULE_DEVICE_TABLE(of, xspi_of_match);

static struct platform_driver xspi_driver = {
	.probe	= xspi_probe,
	.remove	= xspi_remove,
	.driver = {
		.name =	"xspi-if",
		.of_match_table = xspi_of_match,
	},
};
module_platform_driver(xspi_driver);

MODULE_DESCRIPTION("Renesas RPC-IF core driver");
MODULE_LICENSE("GPL v2");
