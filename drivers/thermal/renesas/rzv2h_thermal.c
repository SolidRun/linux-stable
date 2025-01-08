// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZV2H TSU Temperature Sensor Unit
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/units.h>
#include <linux/interrupt.h>

#include "../thermal_hwmon.h"

#define SYS_BASE			0x10430000
#define SYS_TSU0_TRMVAL0_BASE		0x0320
#define SYS_TSU0_TRMVAL1_BASE		0x0324
#define SYS_TSU1_TRMVAL0_BASE		0x0330
#define SYS_TSU1_TRMVAL1_BASE		0x0334

#define CTEMP_MASK			GENMASK(11, 0)

/* Register offsets */
#define TSU_SSUSR	0x00
#define TSU_STRGR	0x04
#define	TSU_SOSR1	0x08
#define TSU_SCRR	0x10
#define TSU_SSR		0x14
#define TSU_CMSR	0x18
#define TSU_SISR	0x30
#define TSU_SIER	0x34
#define TSU_SICR	0x38

#define TSU_SSUSR_EN_TS			BIT(0)
#define TSU_SSUSR_ADC_PD_TS		BIT(1)
#define TSU_SSUSR_SOC_TS_EN		BIT(2)

#define TSU_SOSR1_ADCT_1		0x00
#define TSU_SOSR1_ADCT_2		0x01
#define TSU_SOSR1_ADCT_4		0x02
#define TSU_SOSR1_ADCT_8		0x03
#define TSU_SOSR1_TRGE			BIT(3)
#define TSU_SOSR1_ADCS			BIT(4)
#define TSU_SOSR1_OUTSEL_AVERAGE	BIT(9)
#define TSU_SOSR1_OUTSEL_LATEST		(0 << 9)

#define TSU_CMSR_CMPEN			BIT(0)
#define TSU_CMSR_CMPCOND		BIT(1)

#define TSU_SISR_ADF			BIT(0)

#define TSU_SIER_ADIE			BIT(0)
#define TSU_SIER_CMPIE			BIT(1)

#define TSU_STRGR_ADST			BIT(0)

#define TSU_SICR_ADCLR			BIT(0)
#define TSU_SICR_CMPCLR			BIT(1)

#define TSU_SSR_CONV_RUNNING		BIT(0)

#define TSU_D				41
#define TSU_E				126

#define TSU_TRMVAL_11_0_MASK		GENMASK(11, 0)

#define MCELSIUS(temp)			((temp) * MILLIDEGREE_PER_DEGREE)

#define RZV2H_TSU_SS_TIMEOUT_US		10000

struct rzv2h_thermal_priv {
	struct device *dev;
	void __iomem *base;
	struct thermal_zone_device *zone;
	struct reset_control *rstc;
	u32 trmval0_11_0, trmval1_11_0;
};

static int rzv2h_thermal_get_temp(struct thermal_zone_device *zone, int *temp)
{
	struct rzv2h_thermal_priv *priv = zone->devdata;
	u32 result, reg;
	s64 val, val1, val2;
	int ret;

	/* Check the conversion status */
	ret = readl_poll_timeout(priv->base + TSU_SSR, reg, !reg,
					50, RZV2H_TSU_SS_TIMEOUT_US);
	if (ret)
		return ret;

	/* Start conversion */
	writel(TSU_STRGR_ADST, priv->base + TSU_STRGR);

	/* The conversion complete interrupt is generated.
	 * Check the interrupt status.
	 */
	ret = readl_poll_timeout(priv->base + TSU_SISR, reg,
					reg == TSU_SISR_ADF, 50,
					RZV2H_TSU_SS_TIMEOUT_US);
	if (ret)
		return ret;

	/* Check the conversion status */
	ret = readl_poll_timeout(priv->base + TSU_SSR, reg, !reg,
					50, RZV2H_TSU_SS_TIMEOUT_US);
	if (ret)
		return ret;

	/* Read the converted code */
	result = readl(priv->base + TSU_SCRR) & CTEMP_MASK;

	/* Compensation formula */
	val = (TSU_E + TSU_D) * (result - priv->trmval0_11_0);

	val1 = (priv->trmval1_11_0 - priv->trmval0_11_0);

	val2 = (val / val1) - TSU_D;

	/* Clear the conversion complete status flag */
	writel(TSU_SICR_ADCLR, priv->base + TSU_SICR);

	/* Check that the conversion complete status flag was cleared */
	ret = readl_poll_timeout(priv->base + TSU_SISR, reg,
					reg == !TSU_SISR_ADF, 50,
					RZV2H_TSU_SS_TIMEOUT_US);
	if (ret)
		return ret;

	*temp = MCELSIUS(val2);

	return 0;
}

static const struct thermal_zone_device_ops rzv2h_tz_of_ops = {
	.get_temp = rzv2h_thermal_get_temp,
};

static void rzv2h_thermal_init(struct rzv2h_thermal_priv *priv)
{
	u32 reg;

	/* Release the temperature sensor IP from the power-down state */
	reg = TSU_SSUSR_SOC_TS_EN | TSU_SSUSR_EN_TS;
	writel(reg, priv->base + TSU_SSUSR);

	/* Set the operating mode
	 * - SOSR1: Average data output, Single scan mode, 8 samples.
	 * - CMSR: Default setting, Disable the comparison function.
	 * - SIER: Enable the conversion complete interrupt.
	 */
	reg = TSU_SOSR1_OUTSEL_AVERAGE | TSU_SOSR1_ADCT_8;
	writel(reg, priv->base + TSU_SOSR1);

	writel(0, priv->base + TSU_CMSR);

	reg = TSU_SIER_ADIE;
	writel(reg, priv->base + TSU_SIER);
}

static int rzv2h_thermal_get_value_trimming_registers(struct rzv2h_thermal_priv *priv,
							struct device_node *np)
{
	const char *propname = "tsu,channel";
	int ret;
	u32 id;

	ret = of_property_read_u32_index(np, propname, 0, &id);
	if (ret < 0) {
		dev_err(priv->dev, "not found TSU channel id");
		return -EINVAL;
	}
	{
		/* FIXME: Hard code to access to SYS registers until can find
		 * another way.
		 */
		void __iomem *sys_base = ioremap(SYS_BASE, 0x1000);

		if (id) {
			/* Get TSU Trimming values from SYS registers */
			priv->trmval0_11_0 = ioread32(sys_base + SYS_TSU1_TRMVAL0_BASE)
						& TSU_TRMVAL_11_0_MASK;

			priv->trmval1_11_0 = ioread32(sys_base + SYS_TSU1_TRMVAL1_BASE)
						& TSU_TRMVAL_11_0_MASK;

		} else {
			/* Get TSU Trimming values from SYS registers */
			priv->trmval0_11_0 = ioread32(sys_base + SYS_TSU0_TRMVAL0_BASE)
						& TSU_TRMVAL_11_0_MASK;

			priv->trmval1_11_0 = ioread32(sys_base + SYS_TSU0_TRMVAL1_BASE)
						& TSU_TRMVAL_11_0_MASK;
		}

		iounmap(sys_base);
	}

	if (priv->trmval0_11_0 == 0 || priv->trmval1_11_0 == 0) {
		dev_err(priv->dev, "not found OTP trimming value");
		return -EINVAL;
	}

	return 0;
}

static void rzv2h_thermal_reset_assert_pm_disable_put(struct platform_device *pdev)
{
	struct rzv2h_thermal_priv *priv = dev_get_drvdata(&pdev->dev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	reset_control_assert(priv->rstc);
}

static int rzv2h_thermal_remove(struct platform_device *pdev)
{
	struct rzv2h_thermal_priv *priv = dev_get_drvdata(&pdev->dev);

	thermal_remove_hwmon_sysfs(priv->zone);
	rzv2h_thermal_reset_assert_pm_disable_put(pdev);

	return 0;
}

static int rzv2h_thermal_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *zone;
	struct rzv2h_thermal_priv *priv;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->dev = dev;
	priv->rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->rstc))
		return dev_err_probe(dev, PTR_ERR(priv->rstc),
				     "failed to get cpg reset");

	ret = reset_control_deassert(priv->rstc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to deassert");

	ret = rzv2h_thermal_get_value_trimming_registers(priv, np);
	if (ret < 0)
		return ret;

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	platform_set_drvdata(pdev, priv);
	rzv2h_thermal_init(priv);

	zone = devm_thermal_of_zone_register(dev, 0, priv,
						    &rzv2h_tz_of_ops);
	if (IS_ERR(zone)) {
		dev_err(dev, "Can't register thermal zone");
		ret = PTR_ERR(zone);
		goto err;
	}

	priv->zone = zone;
	priv->zone->tzp->no_hwmon = false;
	ret = thermal_add_hwmon_sysfs(priv->zone);
	if (ret)
		goto err;

	return 0;

err:
	rzv2h_thermal_reset_assert_pm_disable_put(pdev);
	return ret;
}

static int __maybe_unused rzv2h_thermal_suspend(struct device *dev)
{
	struct rzv2h_thermal_priv *priv = dev_get_drvdata(dev);

	pm_runtime_put(dev);
	reset_control_assert(priv->rstc);

	return 0;
}

static int __maybe_unused rzv2h_thermal_resume(struct device *dev)
{
	struct rzv2h_thermal_priv *priv = dev_get_drvdata(dev);

	reset_control_deassert(priv->rstc);
	pm_runtime_get_sync(dev);

	rzv2h_thermal_init(priv);

	return 0;
}

static const struct of_device_id rzv2h_thermal_dt_ids[] = {
	{ .compatible = "renesas,tsu-r9a09g057", },
	{ .compatible = "renesas,tsu-r9a09g047", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzv2h_thermal_dt_ids);

static const struct dev_pm_ops rzv2h_thermal_pm = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(rzv2h_thermal_suspend,
					rzv2h_thermal_resume)
};

static struct platform_driver rzv2h_thermal_driver = {
	.driver = {
		.name = "rzv2h_thermal",
		.of_match_table = rzv2h_thermal_dt_ids,
		.pm		= &rzv2h_thermal_pm,
	},
	.probe = rzv2h_thermal_probe,
	.remove = rzv2h_thermal_remove,
};
module_platform_driver(rzv2h_thermal_driver);

MODULE_DESCRIPTION("Renesas RZV2H TSU Thermal Sensor Driver");
MODULE_AUTHOR("Huy Nguyen <huy.nguyen.wh@renesas.com>");
MODULE_LICENSE("GPL v2");
