// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas Port Output Enable 3 (POE3) driver
 *
 * This program is free software; you can redistribute and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/reset.h>

#define ICSR1	0x00 /* Input level control/status register 1 */
#define OCSR1	0x02 /* Output level control/status register 1 */
#define ICSR2	0x04 /* Input level control/status register 2 */
#define OCSR2	0x06 /* Output level control/status register 1 */
#define ICSR3	0x08 /* Input level control/status register 3 */
#define SPOER	0x0A /* Software port output enable register */
#define POECR1	0x0B /* Port output enable control register 1 */
#define POECR2	0x0C /* Port output enable control register 2 */
#define POECR4	0x10 /* Port output enable control register 4 */
#define POECR5	0x12 /* Port output enable control register 5 */
#define ICSR4	0x16 /* Input level control/status register 4 */

#define OCSR_OCE	BIT(9) /* Output Short High-Impedance Enable */
#define OCSR_OIE	BIT(8) /* Output Short Interrupt */
#define OCSR_OSF	BIT(15) /* Output Short Flag */
#define ICSR_PIE	BIT(8) /* Port Interrupt Enable */
#define ICSR_POEF	BIT(12) /* High-impedance request by input */
#define ICSR34_POEE	BIT(9) /* High-impedance Enable of ICSR3 and ICSR4 */
#define SPOER_MTUCH0HIZ		BIT(2) /* Place MTU0 output in High-Z state */
#define SPOER_MTUCH67HIZ	BIT(1) /* Place MTU67 output in High-Z state */
#define SPOER_MTUCH34HIZ	BIT(0) /* Place MTU34 output in High-Z state */
#define POECR1_MTU0AZE		0x0001 /* MTIOC0A High-impedance Enable */
#define POECR2_MTU3BDZE		0x0400 /* MTIOC3B/D High-impedance Enable */
#define POECR2_MTU6BDZE		0x0004 /* MTIOC6B/D High-impedance Enable */
#define POECR4_IC2ADDMT34	BIT(2) /* MTU34 High-impedance add POE4F */
#define POECR4_IC3ADDMT34	BIT(3) /* MTU34 High-impedance add POE8F */
#define POECR4_IC4ADDMT34	BIT(4) /* MTU34 High-impedance add POE10F */
#define POECR4_IC1ADDMT67	BIT(9) /* MTU67 High-impedance add POE0F */
#define POECR4_IC3ADDMT67	BIT(11) /* MTU67 High-impedance add POE8F */
#define POECR4_IC4ADDMT67	BIT(12) /* MTU67 High-impedance add POE10F */
#define POECR5_IC1ADDMT0	BIT(1) /* MTU0 High-impedance add POE0F */
#define POECR5_IC2ADDMT0	BIT(2) /* MTU0 High-impedance add POE4F */
#define POECR5_IC4ADDMT0	BIT(4) /* MTU0 High-impedance add POE10F */

enum mtu3_channel_port_output {
	MTU3_CHANNEL_0,
	MTU3_CHANNEL_34,
	MTU3_CHANNEL_67,
};

struct renesas_poe3 {
	struct platform_device *pdev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rstc;
	struct mutex mutex;
	int dev_base;
};

static inline unsigned int renesas_poe3_read(struct renesas_poe3 *poe3,
					     unsigned int reg_nr)
{
	if (reg_nr == SPOER || reg_nr == POECR1)
		return ioread8(poe3->base + reg_nr);
	return ioread16(poe3->base + reg_nr);
}

static inline void renesas_poe3_write(struct renesas_poe3 *poe3,
				      unsigned int reg_nr, u16 value)
{
	if (reg_nr == SPOER || reg_nr == POECR1)
		iowrite8((u8)value, poe3->base + reg_nr);
	else
		iowrite16((u16)value, poe3->base + reg_nr);
}

static int renesas_poe3_clear_Hi_z_state(struct renesas_poe3 *poe3,
					 enum mtu3_channel_port_output ch)
{
	unsigned int poecr_reg, poecr_offset, ocsr_reg, icsr_reg;
	int i, val;

	val = renesas_poe3_read(poe3, SPOER);

	switch (ch) {
	case MTU3_CHANNEL_0:
		if ((val & SPOER_MTUCH0HIZ) != 0)
			renesas_poe3_write(poe3, SPOER,
				(val & ~SPOER_MTUCH0HIZ));
		poecr_reg = POECR5;
		poecr_offset = 0x0001;
		break;
	case MTU3_CHANNEL_34:
		if ((val & SPOER_MTUCH34HIZ) != 0)
			renesas_poe3_write(poe3, SPOER,
				(val & ~SPOER_MTUCH34HIZ));
		poecr_reg = POECR4;
		poecr_offset = 0x0001;
		ocsr_reg = OCSR1;
		break;
	case MTU3_CHANNEL_67:
		if ((val & SPOER_MTUCH67HIZ) != 0)
			renesas_poe3_write(poe3, SPOER,
				(val & ~SPOER_MTUCH67HIZ));
		poecr_reg = POECR4;
		poecr_offset = 0x0100;
		ocsr_reg = OCSR2;
		break;
	default:
		goto error_ports;
	}

	/* Clear POExF bits */
	for (i = 1; i <= 4; i++) {
		val = renesas_poe3_read(poe3, poecr_reg);
		if ((val && (poecr_offset << i)) != 0) {
			switch (i) {
			case 1:
				icsr_reg = ICSR1;
				break;
			case 2:
				icsr_reg =  ICSR2;
				break;
			case 3:
				icsr_reg =  ICSR3;
				break;
			case 4:
				icsr_reg =  ICSR4;
				break;
			}

			val = renesas_poe3_read(poe3, icsr_reg);
			if ((val & ICSR_POEF) != 0)
				renesas_poe3_write(poe3,
					icsr_reg, (val & 0x0FFF));
		}
	}

	/* Clear OSF bits */
	if ((ch == MTU3_CHANNEL_34) || (ch == MTU3_CHANNEL_67)) {
		val = renesas_poe3_read(poe3, ocsr_reg);
		if ((val & OCSR_OSF) != 0)
			renesas_poe3_write(poe3, ocsr_reg,
					   (val & 0x0FFF));
	}

	return 0;

error_ports:
	return -EINVAL;
}

static ssize_t mtu0_output_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct renesas_poe3 *poe3 = platform_get_drvdata(pdev);
	unsigned int val1, val2;
	int ret;

	ret = kstrtouint(buf, 0, &val1);
	if (ret == 1)
		return ret;

	mutex_lock(&poe3->mutex);

	val2 = renesas_poe3_read(poe3, SPOER);
	if (val1 == 0)
		renesas_poe3_write(poe3, SPOER,
				  (val2 | SPOER_MTUCH0HIZ));
	else if (val1 == 1)
		ret = renesas_poe3_clear_Hi_z_state(poe3, MTU3_CHANNEL_0);

	mutex_unlock(&poe3->mutex);

	return ret ? : size;
}

static DEVICE_ATTR_WO(mtu0_output_enable);

static ssize_t mtu34_output_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct renesas_poe3 *poe3 = platform_get_drvdata(pdev);
	unsigned int val1, val2;
	int ret;

	ret = kstrtouint(buf, 0, &val1);
	if (ret == 1)
		return ret;

	mutex_lock(&poe3->mutex);

	val2 = renesas_poe3_read(poe3, SPOER);
	if (val1 == 0)
		renesas_poe3_write(poe3, SPOER,
				  (val2 | SPOER_MTUCH34HIZ));
	else if (val1 == 1)
		ret = renesas_poe3_clear_Hi_z_state(poe3, MTU3_CHANNEL_34);

	mutex_unlock(&poe3->mutex);

	return ret ? : size;
}

static DEVICE_ATTR_WO(mtu34_output_enable);

static ssize_t mtu67_output_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct renesas_poe3 *poe3 = platform_get_drvdata(pdev);
	unsigned int val1, val2;
	int ret;

	ret = kstrtouint(buf, 0, &val1);
	if (ret == 1)
		return ret;

	mutex_lock(&poe3->mutex);

	val2 = renesas_poe3_read(poe3, SPOER);
	if (val1 == 0)
		renesas_poe3_write(poe3, SPOER,
				  (val2 | SPOER_MTUCH67HIZ));
	else if (val1 == 1)
		ret = renesas_poe3_clear_Hi_z_state(poe3, MTU3_CHANNEL_67);

	mutex_unlock(&poe3->mutex);

	return ret ? : size;
}

static DEVICE_ATTR_WO(mtu67_output_enable);

static void renesas_poe3_setup(struct renesas_poe3 *poe3)
{
	struct device *dev = &poe3->pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	unsigned int poecr1_val, poecr2_val, poecr4_val, poecr5_val, val;
	u32 tmp, num, ch;
	int i, ret;

	if (!of_get_property(np, "poe3_pins_mode", &tmp))
		goto poe3_assign;

	num = tmp/(sizeof(u32)*2);

	for (i = 0; i < num; i++) {
		of_property_read_u32_index(np, "poe3_pins_mode",
					   i*2, &ch);
		of_property_read_u32_index(np, "poe3_pins_mode",
					   i*2 + 1, &tmp);
		if (tmp < 4) {
			switch (ch) {
			case 0:
				val = renesas_poe3_read(poe3, ICSR1);
				val = (val | (ICSR_PIE | tmp)) & 0x0FFF;
				renesas_poe3_write(poe3, ICSR1, val);
				break;
			case 4:
				val = renesas_poe3_read(poe3, ICSR2);
				val = (val | (ICSR_PIE | tmp)) & 0x0FFF;
				renesas_poe3_write(poe3, ICSR2, val);
				break;
			case 8:
				val = renesas_poe3_read(poe3, ICSR3);
				val = (val | (ICSR_PIE | ICSR34_POEE | tmp)) &
					0x0FFF;
				renesas_poe3_write(poe3, ICSR3, val);
				break;
			case 10:
				val = renesas_poe3_read(poe3, ICSR4);
				val = (val | (ICSR_PIE | ICSR34_POEE | tmp)) &
					0x0FFF;
				renesas_poe3_write(poe3, ICSR4, val);
				break;
			default:
				dev_err(&poe3->pdev->dev, "Invalid pin POE%d: %d\n",
					ch, -EINVAL);
				break;
			}
		} else
			dev_err(&poe3->pdev->dev,
				"Invalid mode %d, must be smaller than 4: %d\n",
				tmp, -EINVAL);
	}

poe3_assign:
	poecr1_val = 0;
	poecr2_val = 0;
	poecr4_val = 0;
	poecr5_val = 0;

	for_each_child_of_node(np, child) {
		if (of_get_property(child, "mtu3_outputs", &tmp)) {
			num = tmp/sizeof(u32);
			for (i = 0; i < num; i++) {
				of_property_read_u32_index(child,
					"mtu3_outputs", i, &tmp);
				if (!strcmp(child->name, "mtu3_ch0") &&
				    (tmp < 4))
					poecr1_val |= (POECR1_MTU0AZE << tmp);
				else if (!strcmp(child->name, "mtu3_ch34") &&
					(tmp < 3))
					poecr2_val |= (POECR2_MTU3BDZE >> tmp);
				else if (!strcmp(child->name, "mtu3_ch67") &&
					(tmp < 3))
					poecr2_val |= (POECR2_MTU6BDZE >>  tmp);
			}

			of_get_property(child, "addition_poe3_inputs", &tmp);
			num = tmp/sizeof(u32);
			for (i = 0; i < num; i++) {
				of_property_read_u32_index(child,
					"addition_poe3_inputs", i, &tmp);
				if (!strcmp(child->name, "mtu3_ch0")) {
					switch (tmp) {
					case 0:
						poecr5_val |= POECR5_IC1ADDMT0;
						break;
					case 4:
						poecr5_val |= POECR5_IC2ADDMT0;
						break;
					case 10:
						poecr5_val |= POECR5_IC4ADDMT0;
						break;
					}
				} else if (!strcmp(child->name, "mtu3_ch34")) {
					switch (tmp) {
					case 4:
						poecr4_val |= POECR4_IC2ADDMT34;
						break;
					case 8:
						poecr4_val |= POECR4_IC3ADDMT34;
						break;
					case 10:
						poecr4_val |= POECR4_IC4ADDMT34;
						break;
					}
				} else if (!strcmp(child->name, "mtu3_ch67")) {
					switch (tmp) {
					case 0:
						poecr4_val |= POECR4_IC1ADDMT67;
						break;
					case 8:
						poecr4_val |= POECR4_IC3ADDMT67;
						break;
					case 10:
						poecr4_val |= POECR4_IC4ADDMT67;
						break;
					}
				}
			}
		}

		if (!strcmp(child->name, "mtu3_ch0"))
			ret = device_create_file(&poe3->pdev->dev,
					&dev_attr_mtu0_output_enable);
		else if (!strcmp(child->name, "mtu3_ch34")) {
			renesas_poe3_write(poe3, OCSR1, OCSR_OCE | OCSR_OIE);
			ret = device_create_file(&poe3->pdev->dev,
					&dev_attr_mtu34_output_enable);
		} else if (!strcmp(child->name, "mtu3_ch67")) {
			renesas_poe3_write(poe3, OCSR2, OCSR_OCE | OCSR_OIE);
			ret = device_create_file(&poe3->pdev->dev,
					&dev_attr_mtu67_output_enable);
		} else
			ret = 0;

		if (ret < 0)
			dev_err(&poe3->pdev->dev, "Failed to create poe3 sysfs for %s\n",
				child->name);
	}

	renesas_poe3_write(poe3, POECR1, poecr1_val);
	renesas_poe3_write(poe3, POECR2, poecr2_val);
	renesas_poe3_write(poe3, POECR4, poecr4_val);
	renesas_poe3_write(poe3, POECR5, poecr5_val);
}

static int renesas_poe3_probe(struct platform_device *pdev)
{
	struct renesas_poe3 *poe3;
	struct resource *res;
	int ret;

	poe3 = devm_kzalloc(&pdev->dev, sizeof(*poe3), GFP_KERNEL);
	if (poe3 == NULL)
		return -ENOMEM;

	poe3->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	poe3->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(poe3->base))
		return PTR_ERR(poe3->base);

	poe3->rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(poe3->rstc)) {
		dev_err(&pdev->dev, "failed to get reset control\n");
		return PTR_ERR(poe3->rstc);
	}
	reset_control_deassert(poe3->rstc);

	poe3->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(poe3->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(poe3->clk);
	}

	ret = clk_prepare_enable(poe3->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		return ret;
	}

	renesas_poe3_setup(poe3);

	platform_set_drvdata(pdev, poe3);
	dev_info(&pdev->dev, "Renesas POE3 driver probed\n");
	return 0;
}

static int renesas_poe3_remove(struct platform_device *pdev)
{
	struct renesas_poe3 *poe = platform_get_drvdata(pdev);

	clk_disable_unprepare(poe->clk);
	return 0;
}

static const struct of_device_id renesas_poe3_of_table[] = {
	{ .compatible = "renesas,poe3", },
	{ .compatible = "renesas,rz-poe3", },
	{ },
};

MODULE_DEVICE_TABLE(of, poe3_of_table);

static struct platform_driver renesas_poe3_device_driver = {
	.probe		= renesas_poe3_probe,
	.remove		= renesas_poe3_remove,
	.driver		= {
		.name	= "renesas_poe3",
		.of_match_table = of_match_ptr(renesas_poe3_of_table),
	}
};

module_platform_driver(renesas_poe3_device_driver);

MODULE_DESCRIPTION("Renesas POE3 Driver");
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("GPL v2");
