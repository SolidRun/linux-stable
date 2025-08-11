// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RSCI I2C driver
 *
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#define RDR_MASK		GENMASK(7, 0)
#define TDR_MASK		GENMASK(7, 0)

#define CCR0_RE			BIT(0)
#define CCR0_TE			BIT(4)
#define CCR0_RIE		BIT(16)
#define CCR0_TIE		BIT(20)
#define CCR0_TEIE		BIT(21)

#define CCR1_NFCS(x)		((x) << 24)
#define CCR1_NFEN		BIT(28)

#define CCR2_BCP_32		BIT(2)
#define CCR2_BRR(x)		((x) << 8)
#define CCR2_BRME		BIT(16)
#define CCR2_CKS(x)		((x) << 20)
#define CCR2_MDDR(x)		((x) << 24)	/* Should be 1 on writes */

#define CCR3_CHR_9BIT		BIT(8)
#define CCR3_CHR_8BIT		BIT(9)
#define CCR3_CHR_7BIT		(BIT(9) | BIT(8))
#define CCR3_LSBF		BIT(12)
#define CCR3_MOD_I2C		BIT(18)

#define ICR_IICDL(x)		(x)		/* with x from 0 to 31 cycles delay */
#define ICR_IICINTM		BIT(8)
#define ICR_IICCSC		BIT(9)
#define ICR_IICACKT_ACK		(0 << 13)
#define ICR_IICACKT_NACK	BIT(13)
#define ICR_IICSTAREQ		BIT(16)
#define ICR_IICRSTAREQ		BIT(17)
#define ICR_IICSTPREQ		BIT(18)
#define ICR_IICSDAS_SERIAL	(0 << 20)
#define ICR_IICSDAS_COND	BIT(20)
#define ICR_IICSDAS_LOW		BIT(21)
#define ICR_IICSDAS_HIGHZ	(BIT(21) | BIT(20))
#define ICR_IICSCLS_SERIAL	(0 << 22)
#define ICR_IICSCLS_COND	BIT(22)
#define ICR_IICSCLS_LOW		BIT(23)
#define ICR_IICSCLS_HIGHZ	(BIT(23) | BIT(22))

#define ISR_IICACKR		BIT(0)

#define CFCLR_ERSC		BIT(4)
#define CFCLR_DCMFC		BIT(16)
#define CFCLR_DPERC		BIT(17)
#define CFCLR_DFERC		BIT(18)
#define CFCLR_ORERC		BIT(24)
#define CFCLR_MFFC		BIT(26)
#define CFCLR_PERC		BIT(27)
#define CFCLR_FERC		BIT(28)
#define CFCLR_RDRFC		BIT(31)

#define ICFCLR_IICSTIFC		BIT(3)

#define RIIC_INIT_MSG	-1

enum rsci_reg_list {
	RSCI_RDR,				/* Receive Data Register */
	RSCI_TDR,				/* Transmit Data Register */
	RSCI_CCR0,				/* Common Control Register 0 */
	RSCI_CCR1,				/* Common Control Register 1 */
	RSCI_CCR2,				/* Common Control Register 2 */
	RSCI_CCR3,				/* Common Control Register 3 */
	RSCI_CCR4,				/* Common Control Register 4 */
	RSCI_ICR,				/* Simple I2C Control Register */
	RSCI_FCR,				/* FIFO Control Register */
	RSCI_DCR,				/* Driver Control Register */
	RSCI_CSR,				/* Common Status Register */
	RSCI_ISR,				/* Simple I2C Status Register */
	RSCI_FRSR,				/* FIFO Receive Status Register */
	RSCI_FTSR,				/* FIFO Transmit Status Register */
	RSCI_CFCLR,				/* Common Flag CLear Register */
	RSCI_ICFCLR,				/* Simple I2C Flag Clear Register */
	RSCI_FFCLR,				/* FIFO Flag CLear Register */
	RSCI_NR_REG
};

struct rsci_of_data {
	const u8 *regs;
};

struct rsci_dev {
	void __iomem *base;
	u8 *buf;
	struct i2c_msg *msg;
	int bytes_left;
	int err;
	const struct rsci_of_data *info;
	struct completion msg_done;
	struct i2c_adapter adapter;
	struct clk *t_clk;
	struct device *dev;
	struct i2c_timings i2c_t;
	struct reset_control *rstc;
};

struct rsci_irq_desc {
	int res_num;
	irq_handler_t isr;
	char *name;
};

static inline u32 rsci_read_reg(struct rsci_dev *riic, int offset)
{
	return readl(riic->base + riic->info->regs[offset]);
}

static inline void rsci_write_reg(u32 val, struct rsci_dev *riic, int offset)
{
	writel(val, riic->base + riic->info->regs[offset]);
}

static inline void rsci_clear_set_bit(struct rsci_dev *riic, u32 clear,
				      u32 set, u8 reg)
{
	writel((readl(riic->base + riic->info->regs[reg]) & ~clear) | set,
	       riic->base + riic->info->regs[reg]);
}

static int rsci_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			 int num)
{
	struct rsci_dev *riic = i2c_get_adapdata(adap);
	struct device *dev = adap->dev.parent;
	unsigned long time_left;
	int i, ret;
	u32 start_bit;
	u32 icr, ccr0;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	reinit_completion(&riic->msg_done);
	riic->err = 0;

	for (i = 0, start_bit = ICR_IICSTAREQ; i < num; i++) {
		riic->bytes_left = RIIC_INIT_MSG;
		riic->buf = msgs[i].buf;
		riic->msg = &msgs[i];

		/* Enable TE, RE, TXI and TEIE. */
		ccr0 = CCR0_RE | CCR0_TE | CCR0_TIE | CCR0_TEIE;
		rsci_write_reg(ccr0, riic, RSCI_CCR0);

		/* Initiate a start condition.
		 * - The IICSTARREQ, IICSDAS, IICSCLS bits must be set simultaneously.
		 * - IICDL, IICINTM, IICCSC, and IICACKT settings must be preserved.
		 */
		icr = ICR_IICSDAS_COND | ICR_IICSCLS_COND | start_bit;
		rsci_clear_set_bit(riic, ICR_IICSDAS_HIGHZ | ICR_IICSCLS_HIGHZ
				   | start_bit, icr, RSCI_ICR);

		time_left = wait_for_completion_timeout(&riic->msg_done,
							riic->adapter.timeout);
		if (time_left == 0)
			riic->err = -ETIMEDOUT;

		if (riic->err)
			break;

		start_bit = ICR_IICRSTAREQ;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return riic->err ?: num;
}

static inline void rsci_i2c_stop_transfer(struct rsci_dev *riic)
{
	u32 icr;

	icr = ICR_IICSDAS_COND | ICR_IICSCLS_COND | ICR_IICSTPREQ;
	rsci_clear_set_bit(riic, ICR_IICSDAS_HIGHZ | ICR_IICSCLS_HIGHZ |
			   ICR_IICSTPREQ, icr, RSCI_ICR);
}

static irqreturn_t rsci_tdre_isr(int irq, void *data)
{
	struct rsci_dev *riic = data;
	u8 val;

	if (!riic->bytes_left) {
		rsci_i2c_stop_transfer(riic);
		return IRQ_NONE;
	}

	if (rsci_read_reg(riic, RSCI_ISR) & ISR_IICACKR) {
		riic->err = -ENXIO;
		riic->bytes_left = 0;
		rsci_i2c_stop_transfer(riic);
		return IRQ_HANDLED;
	}

	if (riic->bytes_left == RIIC_INIT_MSG) {
		riic->bytes_left = riic->msg->len;
		if (!riic->bytes_left) {
			rsci_i2c_stop_transfer(riic);
			return IRQ_HANDLED;
		}
	}

	if (riic->msg->flags & I2C_M_RD) {
		rsci_clear_set_bit(riic, ICR_IICACKT_NACK, ICR_IICACKT_ACK,
				   RSCI_ICR);
		rsci_clear_set_bit(riic, CCR0_RIE, CCR0_RIE, RSCI_CCR0);
		if (riic->bytes_left == 1)
			rsci_clear_set_bit(riic, ICR_IICACKT_NACK,
					   ICR_IICACKT_NACK, RSCI_ICR);

		rsci_clear_set_bit(riic, TDR_MASK, TDR_MASK, RSCI_TDR);
	} else {
		val = *riic->buf++;
		riic->bytes_left--;
		rsci_clear_set_bit(riic, TDR_MASK, val, RSCI_TDR);
	}

	return IRQ_HANDLED;
}

static irqreturn_t rsci_tend_isr(int irq, void *data)
{
	struct rsci_dev *riic = data;
	u8 val;
	u32 icr;

	if (!riic->bytes_left) {
		icr = ICR_IICSDAS_HIGHZ | ICR_IICSCLS_HIGHZ;
		rsci_clear_set_bit(riic, ICR_IICSDAS_HIGHZ | ICR_IICSCLS_HIGHZ,
				   icr, RSCI_ICR);
		rsci_write_reg(ICFCLR_IICSTIFC, riic, RSCI_ICFCLR);
		complete(&riic->msg_done);
	} else if (riic->bytes_left == RIIC_INIT_MSG) {
		val = i2c_8bit_addr_from_msg(riic->msg);
		rsci_write_reg(ICFCLR_IICSTIFC, riic, RSCI_ICFCLR);
		icr = ICR_IICSDAS_SERIAL | ICR_IICSCLS_SERIAL;
		rsci_clear_set_bit(riic, ICR_IICSDAS_HIGHZ | ICR_IICSCLS_HIGHZ,
				   icr, RSCI_ICR);
		rsci_clear_set_bit(riic, TDR_MASK, val, RSCI_TDR);
	}

	return IRQ_HANDLED;
}

static irqreturn_t rsci_rdrf_isr(int irq, void *data)
{
	struct rsci_dev *riic = data;

	*riic->buf = rsci_read_reg(riic, RSCI_RDR) & RDR_MASK;
	riic->buf++;
	riic->bytes_left--;

	return IRQ_HANDLED;
}

static u32 rsci_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm riic_algo = {
	.master_xfer	= rsci_i2c_xfer,
	.functionality	= rsci_i2c_func,
};

static int rsci_i2c_init_hw(struct rsci_dev *riic)
{
	struct i2c_timings *t = &riic->i2c_t;
	int ret;
	unsigned long rate;
	unsigned int cks = 0, c = 0, scrate, prediv, sr = 16, brr;
	struct device *dev = riic->adapter.dev.parent;
	u32 ccr2, icr, cfclr;

	if (t->bus_freq_hz > I2C_MAX_FAST_MODE_FREQ) {
		dev_err(&riic->adapter.dev,
			"unsupported bus speed (%dHz). %d max\n",
			t->bus_freq_hz, I2C_MAX_FAST_MODE_FREQ);
		return -EINVAL;
	}

	rate = clk_get_rate(riic->t_clk);
	if (!rate) {
		dev_err(riic->dev, "input clock rate should not be zero\n");
		return -EINVAL;
	}

	/* Find bitrate setting and clock select to match freq */
	do {
		prediv = sr << (2 * c + 1);
		scrate = prediv * t->bus_freq_hz;
		brr = DIV_ROUND_CLOSEST(rate, scrate) - 1;
		c++;
	} while (brr > 255 && c < 4);

	cks = c - 1;
	brr = clamp(brr, 0U, 255U);

	pr_debug("rsci-i2c: freq=%uHz, cks=%u, brr=%u, rate=%luHZ\n",
		 t->bus_freq_hz, cks, brr, rate);

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	/* Set CCR0 register initial settings */
	rsci_write_reg(0, riic, RSCI_CCR0);

	/* Set ICR register initial setting */
	icr = ICR_IICSDAS_HIGHZ | ICR_IICSCLS_HIGHZ | ICR_IICDL(2) | ICR_IICINTM
				| ICR_IICACKT_NACK | ICR_IICCSC;
	rsci_write_reg(icr, riic, RSCI_ICR);

	/* Set the transmission/reception format as the I2C mode */
	rsci_write_reg(CCR3_MOD_I2C & (~CCR3_LSBF), riic, RSCI_CCR3);

	/* Set the clock selection, and the bit rate */
	ccr2 = (CCR2_MDDR(0xFF) | CCR2_CKS(cks) | CCR2_BRR(brr) | CCR2_BCP_32)
								& ~CCR2_BRME;
	rsci_write_reg(ccr2, riic, RSCI_CCR2);

	/* Set up the Noise-filter function */
	rsci_write_reg(CCR1_NFCS(1) | CCR1_NFEN, riic, RSCI_CCR1);

	/* Clear the corresponding flag */
	cfclr = CFCLR_RDRFC | CFCLR_FERC | CFCLR_PERC | CFCLR_MFFC |
			      CFCLR_ORERC | CFCLR_DFERC | CFCLR_DPERC
					  | CFCLR_DCMFC | CFCLR_ERSC;
	rsci_write_reg(cfclr, riic, RSCI_CFCLR);
	rsci_write_reg(ICFCLR_IICSTIFC, riic, RSCI_ICFCLR);

	/* Enable transmit and receive pin */
	rsci_write_reg(CCR0_RE | CCR0_TE, riic, RSCI_CCR0);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
}

static struct rsci_irq_desc rsci_irqs[] = {
	{ .res_num = 1, .isr = rsci_rdrf_isr, .name = "rsci-rdrf" },
	{ .res_num = 2, .isr = rsci_tdre_isr, .name = "rsci-tdre" },
	{ .res_num = 3, .isr = rsci_tend_isr, .name = "rsci-tend" },
};

static void rsci_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static int rsci_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rsci_dev *riic;
	struct i2c_adapter *adap;
	int i, ret;

	riic = devm_kzalloc(dev, sizeof(*riic), GFP_KERNEL);
	if (!riic)
		return -ENOMEM;

	riic->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(riic->base))
		return PTR_ERR(riic->base);

	riic->t_clk = devm_clk_get(dev, "tclk");
	if (IS_ERR(riic->t_clk)) {
		dev_err(dev, "missing controller tck clock");
		return PTR_ERR(riic->t_clk);
	}

	riic->dev = dev;

	riic->rstc = devm_reset_control_array_get(dev, false, false);
	if (IS_ERR(riic->rstc))
		return dev_err_probe(dev, PTR_ERR(riic->rstc),
				     "Error: failed to get reset ctrl\n");

	ret = reset_control_deassert(riic->rstc);
	if (ret) {
		dev_err(dev, "failed to deassert reset %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, rsci_reset_control_assert,
				       riic->rstc);
	if (ret) {
		dev_err(dev,
			"failed to register assert devm action, %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(rsci_irqs); i++) {
		ret = platform_get_irq(pdev, rsci_irqs[i].res_num);
		if (ret < 0)
			return ret;

		ret = devm_request_irq(dev, ret, rsci_irqs[i].isr,
				       0, rsci_irqs[i].name, riic);
		if (ret) {
			dev_err(dev, "failed to request irq %s\n",
				rsci_irqs[i].name);
			return ret;
		}
	}

	riic->info = of_device_get_match_data(dev);

	adap = &riic->adapter;
	i2c_set_adapdata(adap, riic);
	strscpy(adap->name, "Renesas RSCI I2C adapter", sizeof(adap->name));
	adap->owner = THIS_MODULE;
	adap->algo = &riic_algo;
	adap->dev.parent = dev;
	adap->dev.of_node = dev->of_node;

	init_completion(&riic->msg_done);

	i2c_parse_fw_timings(dev, &riic->i2c_t, true);

	/* Default 0 to save power */
	pm_runtime_set_autosuspend_delay(dev, 0);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = rsci_i2c_init_hw(riic);
	if (ret)
		goto out;

	ret = i2c_add_adapter(adap);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, riic);

	dev_info(dev, "registered with %dHz bus speed\n",
		 riic->i2c_t.bus_freq_hz);
	return 0;

out:
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);
	return ret;
}

static int rsci_i2c_remove(struct platform_device *pdev)
{
	struct rsci_dev *riic = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (!ret) {
		rsci_write_reg(0, riic, RSCI_CCR0);
		pm_runtime_put(&pdev->dev);
	}
	i2c_del_adapter(&riic->adapter);
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);

	return 0;
}

static const u8 rsci_common_regs[RSCI_NR_REG] = {
	[RSCI_RDR]	=  0x00,
	[RSCI_TDR]	=  0x04,
	[RSCI_CCR0]	=  0x08,
	[RSCI_CCR1]	=  0x0C,
	[RSCI_CCR2]	=  0x10,
	[RSCI_CCR3]	=  0x14,
	[RSCI_CCR4]	=  0x18,
	[RSCI_ICR]	=  0x20,
	[RSCI_FCR]	=  0x24,
	[RSCI_DCR]	=  0x30,
	[RSCI_CSR]	=  0x48,
	[RSCI_ISR]	=  0x4C,
	[RSCI_FRSR]	=  0x50,
	[RSCI_FTSR]	=  0x54,
	[RSCI_CFCLR]	=  0x68,
	[RSCI_ICFCLR]	=  0x6C,
	[RSCI_FFCLR]	=  0x70,
};

static const struct rsci_of_data rsci_common_info = {
	.regs = rsci_common_regs,
};

static int rsci_i2c_suspend(struct device *dev)
{
	struct rsci_dev *riic = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	i2c_mark_adapter_suspended(&riic->adapter);

	/* Disable output on transmit/receive pins. */
	rsci_write_reg(0, riic, RSCI_CCR0);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_sync(dev);

	return reset_control_assert(riic->rstc);
}

static int rsci_i2c_resume(struct device *dev)
{
	struct rsci_dev *riic = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(riic->rstc);
	if (ret)
		return ret;

	ret = rsci_i2c_init_hw(riic);
	if (ret) {
		reset_control_assert(riic->rstc);
		return ret;
	}

	i2c_mark_adapter_resumed(&riic->adapter);

	return 0;
}

static const struct dev_pm_ops rsci_i2c_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(rsci_i2c_suspend, rsci_i2c_resume)
};

static const struct of_device_id rsci_i2c_dt_ids[] = {
	{ .compatible = "renesas,rsci-i2c", .data = &rsci_common_info },
	{ /* Sentinel */ },
};

static struct platform_driver rsci_i2c_driver = {
	.probe		= rsci_i2c_probe,
	.remove		= rsci_i2c_remove,
	.driver		= {
		.name	= "rz-rsci-i2c",
		.of_match_table = rsci_i2c_dt_ids,
		.pm     = pm_ptr(&rsci_i2c_pm_ops),
	},
};

module_platform_driver(rsci_i2c_driver);

MODULE_DESCRIPTION("Renesas RSCI I2C adapter");
MODULE_AUTHOR("Tranh Ha <tranh.ha.xb@renesas.com>");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, rsci_i2c_dt_ids);
