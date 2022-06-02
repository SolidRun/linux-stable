// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RAA215300 PMIC driver
 *
 * Copyright (C) 2021-2022 Renesas Electronics Corporation
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>

#define RAA215300_REG_BLOCK_EN		0x6C
#define RAA215300_RTC_EN		BIT(6)

struct raa215300 {
	struct regmap		*regmap;
	struct i2c_client	*client;
	bool			rtc_enabled;
};

static bool raa215300_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config raa215300_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
	.volatile_reg = raa215300_is_volatile_reg,
	.cache_type = REGCACHE_FLAT,
};

static int raa215300_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct raa215300 *pmic;
	int ret;

	pmic = devm_kzalloc(&client->dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	pmic->regmap = devm_regmap_init_i2c(client, &raa215300_regmap_config);
	if (IS_ERR(pmic->regmap)) {
		ret = PTR_ERR(pmic->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	pmic->client = client;
	i2c_set_clientdata(client, pmic);

	pmic->rtc_enabled = of_property_read_bool(client->dev.of_node,
						  "rtc-enable");

	if (pmic->rtc_enabled) {
		regmap_update_bits(pmic->regmap, RAA215300_REG_BLOCK_EN,
				   RAA215300_RTC_EN, 0x40);
		dev_info(&client->dev, "RTC enabled\n");
	}

	dev_info(&client->dev, "RAA215300 initialized");

	return 0;
}

static int raa215300_i2c_remove(struct i2c_client *i2c)
{
	return 0;
}


static const struct of_device_id raa215300_dt_match[] = {
	{ .compatible = "renesas,raa215300" },
	{},
};
MODULE_DEVICE_TABLE(of, raa215300_dt_match);

static struct i2c_driver raa215300_i2c_driver = {
	.driver = {
		   .name = "raa215300",
		   .of_match_table = of_match_ptr(raa215300_dt_match),
	},
	.probe = raa215300_i2c_probe,
	.remove = raa215300_i2c_remove,
};

static int __init raa215300_i2c_init(void)
{
	return i2c_add_driver(&raa215300_i2c_driver);
}
subsys_initcall(raa215300_i2c_init);

static void __exit raa215300_i2c_exit(void)
{
	i2c_del_driver(&raa215300_i2c_driver);
}
module_exit(raa215300_i2c_exit);

MODULE_DESCRIPTION("Renesas RAA215300 PMIC driver");
MODULE_AUTHOR("Fabrizio Castro <fabrizio.castro.jz@renesas.com>");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("post: rtc_isl1208");
