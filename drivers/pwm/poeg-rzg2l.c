// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G2L Port Output Enable for PWM (POEG) driver
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#define POEGG		0x0000

#define PIDE		(1 << 4)
#define EN_NFEN		(1 << 29)
#define SSF		(1 << 3)
#define PIDF		(1 << 0)
#define ST		(1 << 16)
#define IOCE		(1 << 5)
#define IOCF		(1 << 1)

struct rzg2l_poeg_chip {
	struct	clk *clk;
	void	__iomem *mmio_base;
	struct reset_control *rstc;
	struct mutex mutex;
};

void rzg2l_poeg_clear_bit_export(struct platform_device *poeg_dev, u32 data,
				unsigned int offset)
{
	struct rzg2l_poeg_chip *poeg_chip = platform_get_drvdata(poeg_dev);
	u32 read_data = 0;

	read_data = ioread32(poeg_chip->mmio_base + offset);
	iowrite32(read_data & (~data), poeg_chip->mmio_base + offset);
}
EXPORT_SYMBOL(rzg2l_poeg_clear_bit_export);

static inline void rzg2l_poeg_set_bit(struct rzg2l_poeg_chip *poeg_chip,
		u32 data, unsigned int offset)
{
	u32 read_data = 0;

	read_data = ioread32(poeg_chip->mmio_base + offset);
	iowrite32(data|read_data, poeg_chip->mmio_base + offset);
}

static inline void rzg2l_poeg_clear_bit(struct rzg2l_poeg_chip *poeg_chip,
		u32 data, unsigned int offset)
{
	u32 read_data = 0;

	read_data = ioread32(poeg_chip->mmio_base + offset);
	iowrite32(read_data & (~data), poeg_chip->mmio_base + offset);
}

static inline u32 rzg2l_poeg_read(struct rzg2l_poeg_chip *poeg_chip,
				unsigned int offset)
{
	return ioread32(poeg_chip->mmio_base + offset);
}

static void enable_poeg_function(struct rzg2l_poeg_chip *poeg_chip)
{
	/* Port Input Detection and GPT disable Enable */
	rzg2l_poeg_set_bit(poeg_chip, PIDE|IOCE, POEGG);
	/* Enable noise filter */
	rzg2l_poeg_set_bit(poeg_chip, EN_NFEN, POEGG);
}

static ssize_t output_disable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_poeg_chip *poeg_chip = platform_get_drvdata(pdev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&poeg_chip->mutex);

	if (!val)
		rzg2l_poeg_clear_bit(poeg_chip, SSF, POEGG);
	else
		rzg2l_poeg_set_bit(poeg_chip, SSF, POEGG);

	mutex_unlock(&poeg_chip->mutex);

	return ret ? : count;
}

static ssize_t output_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rzg2l_poeg_chip *poeg_chip = platform_get_drvdata(pdev);
	u32 read_data = 0;

	read_data = rzg2l_poeg_read(poeg_chip, POEGG);

	if (read_data & SSF)
		read_data = 1;
	else
		read_data = 0;

	return sprintf(buf, "%u\n", read_data);
}

static DEVICE_ATTR_RW(output_disable);

static int rzg2l_poeg_probe(struct platform_device *pdev)
{
	struct rzg2l_poeg_chip *poeg_chip;
	struct resource *res;
	int ret;

	poeg_chip = devm_kzalloc(&pdev->dev, sizeof(*poeg_chip), GFP_KERNEL);
	if (poeg_chip == NULL)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No memory resource defined\n");
		return -ENODEV;
	}

	poeg_chip->mmio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(poeg_chip->mmio_base))
		return PTR_ERR(poeg_chip->mmio_base);

	poeg_chip->rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(poeg_chip->rstc)) {
		dev_err(&pdev->dev, "failed to get cpg reset\n");
		return PTR_ERR(poeg_chip->rstc);
	}

	reset_control_deassert(poeg_chip->rstc);

	poeg_chip->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(poeg_chip->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(poeg_chip->clk);
	}

	ret = device_create_file(&pdev->dev, &dev_attr_output_disable);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to create poeg sysfs: %d\n", ret);
		return ret;
	}

	/* Power up the device */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	enable_poeg_function(poeg_chip);

	platform_set_drvdata(pdev, poeg_chip);

	dev_info(&pdev->dev, "RZ/G2L POEG Driver probed\n");

	return 0;
}

static int rzg2l_poeg_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_output_disable);
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id rzg2l_poeg_of_table[] = {
	{ .compatible = "renesas,poeg-r9a07g044", },
	{ },
};

MODULE_DEVICE_TABLE(of, rzg2l_poeg_of_table);

static int rzg2l_poeg_suspend(struct device *dev)
{
	pm_runtime_put(dev);

	return 0;
}

static int rzg2l_poeg_resume(struct device *dev)
{
	struct rzg2l_poeg_chip *poeg_chip = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);
	enable_poeg_function(poeg_chip);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rzg2l_poeg_pm_ops, rzg2l_poeg_suspend,
		rzg2l_poeg_resume);

static struct platform_driver rzg2l_poeg_driver = {
	.probe = rzg2l_poeg_probe,
	.remove = rzg2l_poeg_remove,
	.driver = {
		.name = "poeg-rzg2l",
		.owner = THIS_MODULE,
		.pm	= &rzg2l_poeg_pm_ops,
		.of_match_table = of_match_ptr(rzg2l_poeg_of_table),
	},
};

module_platform_driver(rzg2l_poeg_driver);

MODULE_DESCRIPTION("Renesas RZG2L POEG Driver");
MODULE_LICENSE("GPL v2");
