// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/V2H(P) USB VBENCTL VBUS_SEL mux driver
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 */

#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset/reset_rzv2h_usb2phy.h>

#define RZV2H_VBENCTL		0xf0c

struct mux_rzv2h_usb_vbenctl_priv {
	struct regmap_field *field;
	int cached_reg;
};

static int mux_rzv2h_usb_vbenctl_set(struct mux_control *mux, int state)
{
	struct mux_rzv2h_usb_vbenctl_priv *priv = mux_chip_priv(mux->chip);

	return regmap_field_write(priv->field, state);
}

static const struct mux_control_ops mux_rzv2h_usb_vbenctl_ops = {
	.set = mux_rzv2h_usb_vbenctl_set,
};

static const struct regmap_config rzv2h_usb_vbenctl_regconf = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = RZV2H_VBENCTL,
};

static int mux_rzv2h_usb_vbenctl_probe(struct auxiliary_device *adev,
				    const struct auxiliary_device_id *id)
{
	struct reset_rzv2h_usb2phy_adev *rdev = adev->dev.platform_data;
	struct mux_rzv2h_usb_vbenctl_priv *priv;
	struct device *dev = &adev->dev;
	struct mux_chip *mux_chip;
	struct regmap *regmap;
	struct reg_field reg_field = {
		.reg = RZV2H_VBENCTL,
		.lsb = 0,
		.msb = 0,
	};
	int ret;

	regmap = devm_regmap_init_mmio(dev, rdev->base, &rzv2h_usb_vbenctl_regconf);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	mux_chip = devm_mux_chip_alloc(dev, 1, sizeof(*priv));
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	priv = mux_chip_priv(mux_chip);

	priv->field = devm_regmap_field_alloc(dev, regmap, reg_field);
	if (IS_ERR(priv->field))
		return PTR_ERR(priv->field);

	mux_chip->ops = &mux_rzv2h_usb_vbenctl_ops;
	mux_chip->mux[0].states = 2;
	mux_chip->mux[0].idle_state = MUX_IDLE_AS_IS;

	ret = devm_mux_chip_register(dev, mux_chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to register mux chip\n");

	dev_set_drvdata(dev, priv);

	return 0;
}

static int mux_rzv2h_usb_vbenctl_suspend(struct device *dev)
{
	struct mux_rzv2h_usb_vbenctl_priv *priv = dev_get_drvdata(dev);

	regmap_field_read(priv->field, &priv->cached_reg);

	return 0;
}

static int mux_rzv2h_usb_vbenctl_resume(struct device *dev)
{
	struct mux_rzv2h_usb_vbenctl_priv *priv = dev_get_drvdata(dev);

	regmap_field_write(priv->field, priv->cached_reg);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mux_rzv2h_usb_vbenctl_pm_ops,
	mux_rzv2h_usb_vbenctl_suspend, mux_rzv2h_usb_vbenctl_resume);

static const struct auxiliary_device_id mux_rzv2h_usb_vbenctl_ids[] = {
	{ .name = "rzv2h_usb2phy_reset.vbenctl" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(auxiliary, mux_rzv2h_usb_vbenctl_ids);

static struct auxiliary_driver mux_rzv2h_usb_vbenctl_driver = {
	.name		= "vbenctl",
	.probe		= mux_rzv2h_usb_vbenctl_probe,
	.id_table	= mux_rzv2h_usb_vbenctl_ids,
	.driver.pm	= &mux_rzv2h_usb_vbenctl_pm_ops,
};
module_auxiliary_driver(mux_rzv2h_usb_vbenctl_driver);

MODULE_DESCRIPTION("RZ/V2H USB VBENCTL VBUS_SEL mux driver");
MODULE_AUTHOR("Tommaso Merciai <tommaso.merciai.xr@bp.renesas.com>");
MODULE_LICENSE("GPL");
