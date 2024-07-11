// SPDX-License-Identifier: GPL-2.0
/*
 * Dummy GPIO Driver
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define GPIO_DUMMY_NUM_PINS 32

static const char *gpio_dummy_pin_names[GPIO_DUMMY_NUM_PINS];

static int gpio_dummy_get(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static void gpio_dummy_set(struct gpio_chip *chip, unsigned int offset, int value)
{
}

static int gpio_dummy_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static int gpio_dummy_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	return 0;
}

static struct gpio_chip gpio_dummy_chip = {
	.label = "gpio_dummy",
	.owner = THIS_MODULE,
	.get = gpio_dummy_get,
	.set = gpio_dummy_set,
	.direction_input = gpio_dummy_direction_input,
	.direction_output = gpio_dummy_direction_output,
	.base = 0,
	.ngpio = GPIO_DUMMY_NUM_PINS,
	.names = gpio_dummy_pin_names,
};

static int gpio_dummy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret, i;

	for (i = 0; i < GPIO_DUMMY_NUM_PINS; i++) {
		gpio_dummy_pin_names[i] = devm_kasprintf(dev, GFP_KERNEL, "GP_DUMMY_%d", i + 1);
		if (!gpio_dummy_pin_names[i]) {
			dev_err(dev, "Failed to allocate memory for GPIO name %d\n", i + 1);
			return -ENOMEM;
		}
	}

	gpio_dummy_chip.parent = dev;

	ret = gpiochip_add_data(&gpio_dummy_chip, NULL);
	if (ret) {
		dev_err(dev, "Failed to add GPIO chip\n");
		return ret;
	}

	return 0;
}

static int gpio_dummy_remove(struct platform_device *pdev)
{
	gpiochip_remove(&gpio_dummy_chip);
	return 0;
}

static const struct of_device_id gpio_dummy_of_match[] = {
	{ .compatible = "gpio-dummy", },
	{},
};

MODULE_DEVICE_TABLE(of, gpio_dummy_of_match);

static struct platform_driver gpio_dummy_driver = {
	.driver = {
		.name = "gpio-dummy",
		.of_match_table = gpio_dummy_of_match,
	},
	.probe = gpio_dummy_probe,
	.remove = gpio_dummy_remove,
};

module_platform_driver(gpio_dummy_driver);

MODULE_AUTHOR("Nghia Vo <nghia.vo.zn@renesas.com>");
MODULE_DESCRIPTION("Dummy GPIO driver");
MODULE_LICENSE("GPL");
