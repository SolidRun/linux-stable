// SPDX-License-Identifier: GPL-2.0
/*
 * Vbattb driver support G3S
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/clk.h>

/* Register Offset and Bits */
#define BKPSR		0x00	/* Backup Domain Power Status Register	*/
#define TAMPSR		0x04	/* Tamper Status Register		*/
#define TAMPCR		0x08	/* Tamper Control Register		*/
#define TCECR		0x0C	/* Time Capture Event Control Register	*/
#define TAMPICR1	0x10	/* Tamper Input Control Register 1	*/
#define TAMPICR2	0x14	/* Tamper Input Control Register 1	*/
#define BKSCCR		0x1C	/* Backup Domain 32kHz Control Register	*/
#define SOSCCR2		0x24	/* 32kHz Oscillator Control Register	*/
#define ISOEN		0x28	/* Isolation Enable Control Register	*/
#define ISOENPROT	0x2C
#define XOSCCR		0x30	/* Oscillator Control Register		*/

#define BKSCCR_SOSEL	0x40	/* 32kHz oscillator XBYP output select	*/
#define SOSCCR2_SOSTP2	0x01	/* 32kHz oscillator is stopped		*/
#define XOSCCR_OUTEN	0x10003	/* Enable 32kHz clock output		*/

#define TCECR_TCE0S	0x01	/* Time capture event source sellect	*/
#define TAMPICR1_CH0EN	0x01	/* Channel 0 input enable		*/
#define TAMPICR2_CH0NFE 0x01	/* Channel 0 noisefilter		*/
#define TAMPICR2_CH0TRG 0x10	/* Edge of the input tampin		*/

struct rzg3s_vbattb_priv {
	struct device		*dev;
	void __iomem		*regbase;
	unsigned long		regsize;
	struct reset_control	*rstc;
	struct clk		*clk;
};

static int vbattb_rzg3s_probe(struct platform_device *pdev)
{
	struct rzg3s_vbattb_priv *vbattb;
	u32 tmp;
	int ret;

	vbattb = devm_kzalloc(&pdev->dev, sizeof(*vbattb), GFP_KERNEL);
	if (!vbattb)
		return -ENOMEM;

	vbattb->regbase = devm_platform_ioremap_resource(pdev, 0);
	if (!vbattb->regbase)
		return -EINVAL;

	vbattb->clk = devm_clk_get(&pdev->dev, "bclk");
	if (IS_ERR(vbattb->clk))
		return dev_err_probe(&pdev->dev, -EINVAL, "bclk rate is 0");

	clk_enable(vbattb->clk);

	vbattb->rstc = devm_reset_control_array_get_exclusive(&pdev->dev);
	if (IS_ERR(vbattb->rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(vbattb->rstc),
					"failed to get reset\n");
	ret = reset_control_deassert(vbattb->rstc);
	if (ret)
		return ret;

	/* Set 32kHz oscillator to active	*/
	tmp = readl(vbattb->regbase + SOSCCR2);
	tmp &= ~SOSCCR2_SOSTP2;
	writel(tmp, vbattb->regbase + SOSCCR2);
	/* Enable 32kHz clock output		*/
	tmp = readl(vbattb->regbase + XOSCCR);
	tmp |= XOSCCR_OUTEN;
	writel(tmp, vbattb->regbase + XOSCCR);

	return 0;
}

static int vbattb_rzg3s_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id vbattb_rzg3s_of_match[] = {
	{ .compatible = "renesas,vbattb-rzg3s" },
	{ }
};

MODULE_DEVICE_TABLE(of, vbattb_rzg3s_of_match);

static struct platform_driver vbattb_rzg3s_driver = {
	.driver = {
		.name = "vbat-backup-function",
		.of_match_table = vbattb_rzg3s_of_match,
	},
	.probe = vbattb_rzg3s_probe,
	.remove = vbattb_rzg3s_remove,
};

module_platform_driver(vbattb_rzg3s_driver);

MODULE_DESCRIPTION("RZ/G3S simple VBATTB driver");
MODULE_AUTHOR("Nghia Nguyen <nghia.nguyen.xm@renesas.com>");
MODULE_LICENSE("GPL v2");
