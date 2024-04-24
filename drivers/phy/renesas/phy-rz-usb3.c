// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ for USB3.0 PHY driver
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

/* USB2TEST Registers */
#define USB2TEST_RESET				0x0
#define USB2TEST_OTGR				0x600
#define USB2TEST_UTMICTRL2			0xb04
#define USB2TEST_PRMCTRL5			0xc0c

/* USB3TEST Registers */
#define USB3TEST_RSTCTRL			0x1000
#define USB3TEST_CLKCTRL			0x1004
#define USB3TEST_RAMCTRL			0x100c
#define USB3TEST_RAMCTRL_SRAM_INIT_DONE		BIT(2)
#define USB3TEST_CREGCTRL			0x1010
#define USB3TEST_CREGCTRL_PARA_SEL		BIT(8)
#define USB3TEST_LANECONFIG0			0x1030

struct rz_usb3 {
	void __iomem *base;
	struct reset_control *rstc;
};

void usb2test_phy_init(void __iomem *usbtest)
{
	writel(0x00000303, usbtest + USB2TEST_UTMICTRL2);
	writel(0x0000020E, usbtest + USB2TEST_RESET);
	fsleep(10);

	writel(0x00000206, usbtest + USB2TEST_RESET);
	writel(0x00000140, usbtest + USB2TEST_PRMCTRL5);
	fsleep(10);

	writel(0x00000202, usbtest + USB2TEST_RESET);
	fsleep(10);

	writel(0x00000200, usbtest + USB2TEST_RESET);
	writel(0x00000300, usbtest + USB2TEST_UTMICTRL2);
	fsleep(10);
	writel(0x00000000, usbtest + USB2TEST_UTMICTRL2);

	writel(0x00000000, usbtest + USB2TEST_RESET);
	writel(0x00000101, usbtest + USB2TEST_OTGR);
}

void usb3test_phy_init(void __iomem *usbtest)
{
	writel(USB3TEST_CREGCTRL_PARA_SEL, usbtest + USB3TEST_CREGCTRL);
	writel(0x00000303, usbtest +  USB3TEST_RSTCTRL);

	fsleep(20);

	writel(0x00000004, usbtest + USB3TEST_CLKCTRL);
	writel(0x0000000D, usbtest + USB3TEST_LANECONFIG0);
	writel(0x00000301, usbtest + USB3TEST_RSTCTRL);

	while (1) {
		if (readl(usbtest + USB3TEST_RAMCTRL) &
		    USB3TEST_RAMCTRL_SRAM_INIT_DONE)
			break;

		cpu_relax();
	};

	writel(0x00000300, usbtest +  USB3TEST_RSTCTRL);
	writel(0x00000001, usbtest +  USB3TEST_RAMCTRL);
	writel(0x00000000, usbtest +  USB3TEST_RSTCTRL);
}

static int rz_phy_usb3_init(struct phy *p)
{
	struct rz_usb3 *r = phy_get_drvdata(p);

	usb2test_phy_init(r->base);
	usb3test_phy_init(r->base);

	return 0;
}

static const struct phy_ops rz_phy_usb3_ops = {
	.init		= rz_phy_usb3_init,
	.owner		= THIS_MODULE,
};

static const struct of_device_id rz_phy_usb3_match_table[] = {
	{ .compatible = "renesas,rz-usb3-phy" },
	{ }
};

MODULE_DEVICE_TABLE(of, rz_phy_usb3_match_table);

static int rz_phy_usb3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct rz_usb3 *r;
	struct phy *phy;
	struct clk *clk;
	int ret;

	r = devm_kzalloc(dev, sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	r->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(r->base))
		return PTR_ERR(r->base);

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return -EINVAL;

	r->rstc = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(r->rstc))
		return dev_err_probe(dev, PTR_ERR(r->rstc), "failed to get reset\n");

	ret = reset_control_deassert(r->rstc);
	if (ret)
		return ret;

	/*
	 * devm_phy_create() will call pm_runtime_enable(&phy->dev);
	 * And then, phy-core will manage runtime pm for this device.
	 */
	pm_runtime_enable(dev);

	phy = devm_phy_create(dev, NULL, &rz_phy_usb3_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "Failed to create USB3 PHY\n");
		ret = PTR_ERR(phy);
		goto error;
	}

	platform_set_drvdata(pdev, r);
	phy_set_drvdata(phy, r);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "Failed to register PHY provider\n");
		ret = PTR_ERR(provider);
		goto error;
	}

	return 0;

error:
	pm_runtime_disable(dev);

	return ret;
}

static int rz_phy_usb3_remove(struct platform_device *pdev)
{
	struct rz_usb3 *r = dev_get_drvdata(&pdev->dev);

	reset_control_assert(r->rstc);
	pm_runtime_disable(&pdev->dev);

	return 0;
};

static struct platform_driver rz_phy_usb3_driver = {
	.driver = {
		.name = "phy_rz_usb3",
		.of_match_table = rz_phy_usb3_match_table,
	},
	.probe	= rz_phy_usb3_probe,
	.remove = rz_phy_usb3_remove,
};
module_platform_driver(rz_phy_usb3_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas RZ USB 3.0 PHY");
MODULE_AUTHOR("biju.das.jz@bp.renesas.com>");
