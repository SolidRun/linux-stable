// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/G3S TSU Thermal Sensor Driver
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
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

#include <linux/iio/adc/rzg2l_adc.h>
#include "thermal_hwmon.h"

#define CTEMP_MASK	0xFFF

/* default calibration values, if FUSE values are missing */
#define SW_CALIB0_VAL	1297
#define SW_CALIB1_VAL	751

/* Register offsets */
#define TSU_SM		0x00

#define OTPTSUTRIM_REG(n)	(0x18 + ((n) * 0x4))
#define OTPTSUTRIM_EN_MASK	BIT(31)
#define OTPTSUTRIM_MASK		GENMASK(11, 0)

/* Sensor Mode Register (TSU_SM) */
#define TSU_SM_EN		BIT(0)
#define TSU_SM_OE		BIT(1)

#define TS_CODE_AVE_SCALE(x)	((x) * 1000000)
#define MCELSIUS(temp)		((temp) * MILLIDEGREE_PER_DEGREE)
#define TS_CODE_CAP_TIMES	8	/* Total number of ADC data samples */

#define RZG2L_THERMAL_GRAN	500	/* milli Celsius */

#define CURVATURE_CORRECTION_CONST	13

struct rzg3s_thermal_priv {
	struct device *dev;
	void __iomem *base;
	struct thermal_zone_device *zone;
	struct reset_control *rstc;
	u32 calib0, calib1;
	struct platform_device *adc_pdev;
};

static inline u32 rzg3s_thermal_read(struct rzg3s_thermal_priv *priv, u32 reg)
{
	return ioread32(priv->base + reg);
}

static inline void rzg3s_thermal_write(struct rzg3s_thermal_priv *priv, u32 reg,
				       u32 data)
{
	iowrite32(data, priv->base + reg);
}

static int rzg3s_thermal_get_temp(void *devdata, int *temp)
{
	struct rzg3s_thermal_priv *priv = devdata;
	u32 result;
	int ts_code_ave = 0;
	int val, i, ret;

	for (i = 0; i < TS_CODE_CAP_TIMES ; i++) {
		/*
		 * TSU repeats measurement at 20 microseconds intervals and
		 * automatically updates the results of measurement. As per
		 * the HW manual for measuring temperature we need to read 8
		 * values consecutively and then take the average.
		 * ts_code_ave = (ts_code[0] + ⋯ + ts_code[7]) / 8
		 */
		ret = rzg2l_adc_read_tsu(&priv->adc_pdev->dev, &result);
		if (ret)
			return ret;

		ts_code_ave += result;
		usleep_range(20, 30);
	}

	ts_code_ave /= TS_CODE_CAP_TIMES;

	/*
	 * The temperature Tj is calculated by the formula
	 * Tj = (ts_code_ave − calib1) * 165/ (calib0 − calib1) − 40
	 * where calib0 and calib1 are the calibration values.
	 */
	val = ((ts_code_ave - priv->calib1) * (MCELSIUS(165) /
		(priv->calib0 - priv->calib1))) - MCELSIUS(40);

	*temp = roundup(val, RZG2L_THERMAL_GRAN);

	return 0;
}

static const struct thermal_zone_of_device_ops rzg3s_tz_of_ops = {
	.get_temp = rzg3s_thermal_get_temp,
};

static int rzg3s_thermal_init(struct rzg3s_thermal_priv *priv)
{
	u32 reg_val;

	/* Enable the thermal sensor */
	rzg3s_thermal_write(priv, TSU_SM, TSU_SM_EN);
	usleep_range(30, 50);

	if (!rzg3s_thermal_read(priv, TSU_SM))
		return -EINVAL;

	/* Enable output signal */
	reg_val = rzg3s_thermal_read(priv, TSU_SM);
	reg_val |= TSU_SM_OE;
	rzg3s_thermal_write(priv, TSU_SM, reg_val);
	usleep_range(1000, 1020);

	if (!(rzg3s_thermal_read(priv, TSU_SM) & TSU_SM_OE))
		return -EINVAL;

	return 0;
}

static void rzg3s_thermal_reset_assert_pm_disable_put(struct platform_device *pdev)
{
	struct rzg3s_thermal_priv *priv = dev_get_drvdata(&pdev->dev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	reset_control_assert(priv->rstc);
}

static int rzg3s_thermal_remove(struct platform_device *pdev)
{
	struct rzg3s_thermal_priv *priv = dev_get_drvdata(&pdev->dev);

	thermal_remove_hwmon_sysfs(priv->zone);
	rzg3s_thermal_reset_assert_pm_disable_put(pdev);

	return 0;
}

static int rzg3s_get_adc_device(struct rzg3s_thermal_priv *priv)
{
	struct device_node *adc_node;
	struct platform_device *adc_pdev;

	adc_node = of_parse_phandle(priv->dev->of_node, "adc-handle", 0);
	if (!adc_node) {
		dev_err(priv->dev, "No adc-handle property found \n");
		return -EINVAL;
	}

	adc_pdev = of_find_device_by_node(adc_node);
	if (!adc_pdev) {
		dev_err(priv->dev, "Failed to get ADC \n");
		return -EINVAL;
	}

	priv->adc_pdev = adc_pdev;

	return 0;
}

static int rzg3s_thermal_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *zone;
	struct rzg3s_thermal_priv *priv;
	struct device *dev = &pdev->dev;
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

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	priv->calib0 = rzg3s_thermal_read(priv, OTPTSUTRIM_REG(0));
	if (priv->calib0 & OTPTSUTRIM_EN_MASK)
		priv->calib0 &= OTPTSUTRIM_MASK;
	else
		priv->calib0 = SW_CALIB0_VAL;

	priv->calib1 = rzg3s_thermal_read(priv, OTPTSUTRIM_REG(1));
	if (priv->calib1 & OTPTSUTRIM_EN_MASK)
		priv->calib1 &= OTPTSUTRIM_MASK;
	else
		priv->calib1 = SW_CALIB1_VAL;

	platform_set_drvdata(pdev, priv);

	/* Get device of ADC driver */
	ret = rzg3s_get_adc_device(priv);
	if (ret) {
		dev_err(dev, "Failed to get ADC device");
		goto err;
	}

	ret = rzg3s_thermal_init(priv);
	if (ret) {
		dev_err(dev, "Failed to start TSU");
		goto err;
	}

	zone = devm_thermal_zone_of_sensor_register(dev, 0, priv,
						    &rzg3s_tz_of_ops);
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

	dev_dbg(dev, "TSU probed with %s calibration values",
		rzg3s_thermal_read(priv, OTPTSUTRIM_REG(0)) ?  "hw" : "sw");

	return 0;

err:
	rzg3s_thermal_reset_assert_pm_disable_put(pdev);
	return ret;
}

static const struct of_device_id rzg3s_thermal_dt_ids[] = {
	{ .compatible = "renesas,rzg3s-tsu", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzg3s_thermal_dt_ids);

static int rzg3s_thermal_power_off(struct rzg3s_thermal_priv *priv)
{
	u32 value;

	/* Disabled the thermal sensor */
	value = rzg3s_thermal_read(priv, TSU_SM);
	value &= ~TSU_SM_EN;
	rzg3s_thermal_write(priv, TSU_SM, value);
	usleep_range(30, 50);

	/* Disable output signal */
	value = rzg3s_thermal_read(priv, TSU_SM);
	value &= ~TSU_SM_OE;
	rzg3s_thermal_write(priv, TSU_SM, value);

	usleep_range(1000, 1020);

	return 0;
}

static int rzg3s_toggle_thermal_zone(struct rzg3s_thermal_priv *priv, bool on)
{
	struct thermal_zone_device *zone;
	int ret;

	zone = priv->zone;
	if (on) {
		dev_dbg(priv->dev, "Enable thermal zone device\n");
		ret = thermal_zone_device_enable(zone);
	} else {
		dev_dbg(priv->dev, "Disable thermal zone device\n");
		ret = thermal_zone_device_disable(zone);
	}

	return ret;
}

static int __maybe_unused rzg3s_thermal_suspend(struct device *dev)
{
	struct rzg3s_thermal_priv *priv = dev_get_drvdata(dev);
	int ret;

	/* Disable registers thermal */
	ret = rzg3s_thermal_power_off(priv);
	if (ret) {
		dev_err(priv->dev, "FAIL to turn off thermal, err %d\n", ret);
		return ret;
	}

	/* Disable reset */
	ret = reset_control_assert(priv->rstc);
	if (ret) {
		dev_err(priv->dev, "FAIL to assert reset, err %d\n", ret);
		return ret;
	}

	/* Disable thermal zone */
	ret = rzg3s_toggle_thermal_zone(priv, false);
	if (ret) {
		dev_err(priv->dev,
			"FAIL to disable thermal zone, err %d\n", ret);
		return ret;
	}

	return 0;
}

static int __maybe_unused rzg3s_thermal_resume(struct device *dev)
{
	struct rzg3s_thermal_priv *priv = dev_get_drvdata(dev);
	int ret;

	/* Dessert reset thermal */
	ret = reset_control_deassert(priv->rstc);
	if (ret) {
		dev_err(priv->dev, "FAILED to deassert reset, err %d\n", ret);
		return ret;
	}

	/* Init registers thermal */
	ret = rzg3s_thermal_init(priv);
	if (ret) {
		dev_err(priv->dev, "FAILED to init TSU, err %d\n", ret);
		return ret;
	}

	/* Enable thermal zone */
	ret = rzg3s_toggle_thermal_zone(priv, true);
	if (ret) {
		dev_err(priv->dev,
			"FAILED to enable thermal zone ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(rzg3s_thermal_pm_ops, rzg3s_thermal_suspend,
			 rzg3s_thermal_resume);

static struct platform_driver rzg3s_thermal_driver = {
	.driver = {
		.name = "rzg3s_thermal",
		.pm = &rzg3s_thermal_pm_ops,
		.of_match_table = rzg3s_thermal_dt_ids,
	},
	.probe = rzg3s_thermal_probe,
	.remove = rzg3s_thermal_remove,
};

static __init int rzg3s_thermal_initcall(void)
{
	platform_driver_register(&rzg3s_thermal_driver);
	return 0;
}

static __exit void rzg3s_thermal_exitcall(void)
{
	platform_driver_unregister(&rzg3s_thermal_driver);
}
late_initcall_sync(rzg3s_thermal_initcall);
module_exit(rzg3s_thermal_exitcall);

MODULE_DESCRIPTION("Renesas RZ/G3S TSU Thermal Sensor Driver");
MODULE_AUTHOR("Duy Dang <duy.dang.yb@renesas.com>");
MODULE_LICENSE("GPL v2");
