// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/V2H USBPHY control driver
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>
#include <linux/delay.h>

#define RESET			0x000

#define UTMICTRL1		0xB00
#define UTMICTRL2		0xB04
#define OTG			0x600

#define NUM_PORTS		2
#define SLEEPM0_ODEN		BIT(9)
#define SUSPENDM0_ODEN		BIT(8)
#define SLEEPM0_REG		BIT(1)
#define SUSPENDM0_REG		BIT(0)
#define PORT_RESET0_REG		BIT(1)
#define PORTRESET0_ODEN		BIT(9)
#define PHY_RESET_REG		BIT(2)
#define SIDDQ_REG		BIT(3)
#define DRVVBUS0_ODEN		BIT(8)
#define DRVVBUS0_REG		BIT(0)
#define IDPULLUP0_ODEN		BIT(11)
#define IDPULLUP0_REG		BIT(3)

struct rzv2h_usbphy_ctrl_priv {
	struct reset_controller_dev rcdev;
	struct reset_control *rstc;
	void __iomem *base;

	spinlock_t lock;
};

#define rcdev_to_priv(x)	container_of(x, struct rzv2h_usbphy_ctrl_priv, rcdev)

void phyctrl_modify(void __iomem *base, u32 reg, u32 clear,
			u32 set)
{
	u32 data;

	data = (readl(base + reg) & ~clear) | set;
	writel(data, base + reg);
}

static int rzv2h_usbphy_ctrl_assert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct rzv2h_usbphy_ctrl_priv *priv = rcdev_to_priv(rcdev);
	unsigned long flags;
	void __iomem *base = priv->base;

	spin_lock_irqsave(&priv->lock, flags);

	phyctrl_modify(base, OTG, DRVVBUS0_ODEN | DRVVBUS0_REG, 0);
	phyctrl_modify(base, RESET, 0, SIDDQ_REG | PORTRESET0_ODEN | PHY_RESET_REG);
	phyctrl_modify(base, UTMICTRL2, SLEEPM0_REG | SUSPENDM0_REG, 0);

	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

static int rzv2h_usbphy_ctrl_deassert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	struct rzv2h_usbphy_ctrl_priv *priv = rcdev_to_priv(rcdev);
	unsigned long flags;
	void __iomem *base = priv->base;

	spin_lock_irqsave(&priv->lock, flags);

	writel(PORTRESET0_ODEN, base + RESET);
	udelay(10);
	writel(SLEEPM0_REG | SUSPENDM0_REG, base + UTMICTRL2);
	writel(0, base + RESET);
	writel(IDPULLUP0_ODEN | DRVVBUS0_ODEN | DRVVBUS0_REG | IDPULLUP0_REG, base + OTG);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}


static int rzv2h_usbphy_ctrl_status(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct rzv2h_usbphy_ctrl_priv *priv = rcdev_to_priv(rcdev);
	u32 port_mask;
	void __iomem *base = priv->base;

	port_mask = PORTRESET0_ODEN | PORT_RESET0_REG;

	return !!(readl(base + RESET) & port_mask);
}

static const struct of_device_id rzv2h_usbphy_ctrl_match_table[] = {
	{ .compatible = "renesas,rzv2h-usbphy-ctrl" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzv2h_usbphy_ctrl_match_table);

static const struct reset_control_ops rzv2h_usbphy_ctrl_reset_ops = {
	.assert = rzv2h_usbphy_ctrl_assert,
	.deassert = rzv2h_usbphy_ctrl_deassert,
	.status = rzv2h_usbphy_ctrl_status,
};

static int rzv2h_usbphy_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzv2h_usbphy_ctrl_priv *priv;
	int error;
	unsigned long flags;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->rstc = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(priv->rstc))
		return dev_err_probe(dev, PTR_ERR(priv->rstc),
				     "failed to get reset\n");

	error = reset_control_deassert(priv->rstc);
	if (error)
		return error;

	priv->rcdev.ops = &rzv2h_usbphy_ctrl_reset_ops;
	priv->rcdev.of_reset_n_cells = 1;
	priv->rcdev.nr_resets = NUM_PORTS;
	priv->rcdev.of_node = dev->of_node;
	priv->rcdev.dev = dev;

	error = devm_reset_controller_register(dev, &priv->rcdev);
	if (error)
		return error;

	spin_lock_init(&priv->lock);
	dev_set_drvdata(dev, priv);

	pm_runtime_enable(&pdev->dev);
	error = pm_runtime_resume_and_get(&pdev->dev);
	if (error < 0) {
		pm_runtime_disable(&pdev->dev);
		reset_control_assert(priv->rstc);
		return dev_err_probe(&pdev->dev, error, "pm_runtime_resume_and_get failed");
	}

	spin_lock_irqsave(&priv->lock, flags);

	writel(SLEEPM0_ODEN | SUSPENDM0_ODEN | SLEEPM0_REG | SUSPENDM0_REG, priv->base + UTMICTRL2);
	phyctrl_modify(priv->base, OTG, DRVVBUS0_ODEN | DRVVBUS0_REG, 0);
	phyctrl_modify(priv->base, RESET, 0, SIDDQ_REG | PORTRESET0_ODEN | PHY_RESET_REG);
	phyctrl_modify(priv->base, UTMICTRL2, SLEEPM0_REG | SUSPENDM0_REG, 0);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int __maybe_unused rzv2h_usbphy_ctrl_suspend(struct device *dev)
{
	struct rzv2h_usbphy_ctrl_priv *priv = dev_get_drvdata(dev);

	rzv2h_usbphy_ctrl_assert(&priv->rcdev, 0);

	pm_runtime_put(dev);
	reset_control_assert(priv->rstc);

	return 0;
}

static int __maybe_unused rzv2h_usbphy_ctrl_resume(struct device *dev)
{
	struct rzv2h_usbphy_ctrl_priv *priv = dev_get_drvdata(dev);

	reset_control_deassert(priv->rstc);
	pm_runtime_get_sync(dev);

	rzv2h_usbphy_ctrl_deassert(&priv->rcdev, 0);
	return 0;
}

static const struct dev_pm_ops rzv2h_usbphy_ctrl_pm = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(rzv2h_usbphy_ctrl_suspend,
					rzv2h_usbphy_ctrl_resume)
};

static int rzv2h_usbphy_ctrl_remove(struct platform_device *pdev)
{
	struct rzv2h_usbphy_ctrl_priv *priv = dev_get_drvdata(&pdev->dev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	reset_control_assert(priv->rstc);

	return 0;
}

static struct platform_driver rzv2h_usbphy_ctrl_driver = {
	.driver = {
		.name		= "rzv2h_usbphy_ctrl",
		.of_match_table	= rzv2h_usbphy_ctrl_match_table,
		.pm		= &rzv2h_usbphy_ctrl_pm,
	},
	.probe	= rzv2h_usbphy_ctrl_probe,
	.remove	= rzv2h_usbphy_ctrl_remove,
};
module_platform_driver(rzv2h_usbphy_ctrl_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Renesas RZ/V2H USBPHY Control");
