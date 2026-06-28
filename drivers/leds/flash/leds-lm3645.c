// SPDX-License-Identifier: GPL-2.0-only
/*
 * leds-lm3645.c - TI LM3645 synchronous-boost quad LED flash driver
 *
 * The LM3645 drives four LED current sources (D1..D4), each selectable as
 * off / IR / torch / flash. On the SolidRun SolidSense AIOT Flash Card the
 * four outputs drive an IR illuminator (4x SFH4722). This driver registers a
 * single LED-flash-class device that programs all four outputs identically.
 *
 * The flash is triggered over I2C (software strobe) by default. If the
 * optional "strobe" (STR1) and/or "torch" (TOR/TX) GPIOs are described, the
 * driver instead gates those hardware pins for low-latency external control.
 * The IR/voltage/NTC modes are not used here.
 *
 * Register map and currents from the LM3645 datasheet (SNVSCV4, Sept 2024).
 *
 * Copyright (C) 2026 SolidRun Ltd.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include <media/v4l2-flash-led-class.h>

/* Register map (MainReg) */
#define LM3645_REG_CTRL1	0x00
#define LM3645_REG_CTRL2	0x01
#define LM3645_CTRL2_DX_EN_MASK	0x0f	/* D1..D4 output enable */
#define LM3645_CTRL2_STR1_EN	0x10	/* STR1 pin enable */
#define LM3645_CTRL2_STR2_EN	0x20	/* STR2 pin enable */
#define LM3645_CTRL2_TORTX_EN	0x40	/* TOR/TX pin enable */
#define LM3645_CTRL2_TORTX_TORCH 0x80	/* TOR/TX pin acts as torch enable (else Tx) */
#define LM3645_REG_DMODE	0x02	/* 2 bits/output: 0 off,1 IR,2 torch,3 flash */
#define LM3645_DMODE_ALL_OFF	0x00
#define LM3645_DMODE_ALL_TORCH	0xaa
#define LM3645_DMODE_ALL_FLASH	0xff
#define LM3645_REG_STR_CTRL	0x03	/* strobe-pin output assignment */
#define LM3645_STR_CTRL_DX_STR1	0x0f	/* assign D1..D4 to STR1 (bits[3:0]) */
#define LM3645_REG_STR_TIME	0x04
#define LM3645_STR_TIME_FTO_MASK	0x0f
#define LM3645_REG_D1_FLASH	0x05	/* 0x05..0x08: D1..D4 flash/IR current */
#define LM3645_REG_D1_TORCH	0x09	/* 0x09..0x0c: D1..D4 torch current */
#define LM3645_REG_FLAG		0x14
#define LM3645_REG_DEVINFO	0x1b
#define LM3645_REG_MAX		0x1b

/* Flash/IR current: ILED = 7.8mA * code + 7.325mA (datasheet 6.6.1.6) */
#define LM3645_FLASH_MIN_UA	7325
#define LM3645_FLASH_STEP_UA	7800
#define LM3645_FLASH_MAX_UA	1996325

/* Torch current: 0.525mA .. 360mA over 256 steps (~1.4mA/step) */
#define LM3645_TORCH_MIN_UA	525
#define LM3645_TORCH_STEP_UA	1400

/* Flash time-out durations (FTO_DUR codes 0..15), in microseconds */
static const u32 lm3645_timeout_us[16] = {
	 10000,  20000,  30000,  40000,  50000,  60000,  70000,  80000,
	 90000, 100000, 150000, 200000, 250000, 300000, 350000, 400000,
};

struct lm3645 {
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *strobe_gpio;	/* STR1: hardware flash strobe */
	struct gpio_desc *torch_gpio;	/* TOR/TX: hardware torch enable */
	struct led_classdev_flash fled;
	struct v4l2_flash *v4l2_flash;
	struct fwnode_handle *led_node;
	u32 torch_max_brightness;
};

static struct lm3645 *fled_to_lm3645(struct led_classdev_flash *fled)
{
	return container_of(fled, struct lm3645, fled);
}

static u8 lm3645_clamp_code(u32 val, u32 min, u32 step)
{
	if (val < min)
		return 0;
	val = (val - min) / step;
	return min_t(u32, val, 255);
}

/* Program the same current code into all four Dx registers. */
static int lm3645_set_all(struct lm3645 *chip, u8 base_reg, u8 code)
{
	int i, ret;

	for (i = 0; i < 4; i++) {
		ret = regmap_write(chip->regmap, base_reg + i, code);
		if (ret)
			return ret;
	}
	return 0;
}

/* LED-class brightness == torch level (0 turns the outputs off). */
static int lm3645_torch_brightness_set(struct led_classdev *cdev,
				       enum led_brightness brightness)
{
	struct led_classdev_flash *fled = lcdev_to_flcdev(cdev);
	struct lm3645 *chip = fled_to_lm3645(fled);
	int ret;

	if (!brightness) {
		if (chip->torch_gpio)
			gpiod_set_value_cansleep(chip->torch_gpio, 0);
		return regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
					  LM3645_CTRL2_DX_EN_MASK, 0);
	}

	/* brightness level maps directly to the torch current code */
	ret = lm3645_set_all(chip, LM3645_REG_D1_TORCH, brightness);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, LM3645_REG_DMODE,
			   LM3645_DMODE_ALL_TORCH);
	if (ret)
		return ret;

	/*
	 * Hardware torch: configure TOR/TX as a torch enable and arm the
	 * outputs while the pin is still low, then drive the pin high to turn
	 * the torch on (datasheet 6.5.3, Table 6-4).
	 */
	if (chip->torch_gpio) {
		ret = regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
					 LM3645_CTRL2_TORTX_TORCH |
					 LM3645_CTRL2_TORTX_EN |
					 LM3645_CTRL2_DX_EN_MASK,
					 LM3645_CTRL2_TORTX_TORCH |
					 LM3645_CTRL2_TORTX_EN |
					 LM3645_CTRL2_DX_EN_MASK);
		if (ret)
			return ret;

		gpiod_set_value_cansleep(chip->torch_gpio, 1);
		return 0;
	}

	return regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
				  LM3645_CTRL2_DX_EN_MASK,
				  LM3645_CTRL2_DX_EN_MASK);
}

static int lm3645_flash_brightness_set(struct led_classdev_flash *fled,
				       u32 brightness_ua)
{
	struct lm3645 *chip = fled_to_lm3645(fled);
	u8 code = lm3645_clamp_code(brightness_ua, LM3645_FLASH_MIN_UA,
				    LM3645_FLASH_STEP_UA);

	return lm3645_set_all(chip, LM3645_REG_D1_FLASH, code);
}

static int lm3645_flash_timeout_set(struct led_classdev_flash *fled,
				    u32 timeout_us)
{
	struct lm3645 *chip = fled_to_lm3645(fled);
	int code;

	/* pick the largest tabulated duration that does not exceed the request */
	for (code = ARRAY_SIZE(lm3645_timeout_us) - 1; code > 0; code--)
		if (lm3645_timeout_us[code] <= timeout_us)
			break;

	return regmap_update_bits(chip->regmap, LM3645_REG_STR_TIME,
				  LM3645_STR_TIME_FTO_MASK, code);
}

static int lm3645_strobe_set(struct led_classdev_flash *fled, bool state)
{
	struct lm3645 *chip = fled_to_lm3645(fled);
	int ret;

	/*
	 * Hardware strobe: the STR1 pin gates the flash. Arm the outputs in
	 * flash mode and assign them to STR1 while the pin is still low, then
	 * drive the pin high to fire. STR1 defaults to level-triggered
	 * (STR_TIME.STR1_LE = 0), so the pulse is bounded by the programmed
	 * flash time-out (datasheet 6.5.2, Table 6-3).
	 */
	if (chip->strobe_gpio) {
		if (!state) {
			gpiod_set_value_cansleep(chip->strobe_gpio, 0);
			return regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
						  LM3645_CTRL2_STR1_EN |
						  LM3645_CTRL2_DX_EN_MASK, 0);
		}

		ret = regmap_write(chip->regmap, LM3645_REG_DMODE,
				   LM3645_DMODE_ALL_FLASH);
		if (ret)
			return ret;
		ret = regmap_write(chip->regmap, LM3645_REG_STR_CTRL,
				   LM3645_STR_CTRL_DX_STR1);
		if (ret)
			return ret;
		ret = regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
					 LM3645_CTRL2_STR1_EN |
					 LM3645_CTRL2_DX_EN_MASK,
					 LM3645_CTRL2_STR1_EN |
					 LM3645_CTRL2_DX_EN_MASK);
		if (ret)
			return ret;

		gpiod_set_value_cansleep(chip->strobe_gpio, 1);
		return 0;
	}

	if (!state)
		return regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
					  LM3645_CTRL2_DX_EN_MASK, 0);

	/*
	 * Software (I2C) strobe: set all outputs to flash mode and enable
	 * them. With no STRx pin assigned the device fires an I2C-triggered
	 * flash that self-terminates after the programmed time-out.
	 */
	ret = regmap_write(chip->regmap, LM3645_REG_DMODE,
			   LM3645_DMODE_ALL_FLASH);
	if (ret)
		return ret;

	return regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
				  LM3645_CTRL2_DX_EN_MASK,
				  LM3645_CTRL2_DX_EN_MASK);
}

static const struct led_flash_ops lm3645_flash_ops = {
	.flash_brightness_set	= lm3645_flash_brightness_set,
	.strobe_set		= lm3645_strobe_set,
	.timeout_set		= lm3645_flash_timeout_set,
};

static const struct regmap_config lm3645_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LM3645_REG_MAX,
};

static int lm3645_init_hw(struct lm3645 *chip)
{
	int ret;

	/* Start from a known-quiet state: all outputs off, no strobe pins. */
	ret = regmap_write(chip->regmap, LM3645_REG_CTRL2, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(chip->regmap, LM3645_REG_DMODE, LM3645_DMODE_ALL_OFF);
	if (ret)
		return ret;
	ret = regmap_write(chip->regmap, LM3645_REG_STR_CTRL, 0x00);
	if (ret)
		return ret;
	ret = lm3645_set_all(chip, LM3645_REG_D1_FLASH, 0x00);
	if (ret)
		return ret;
	return lm3645_set_all(chip, LM3645_REG_D1_TORCH, 0x00);
}

static int lm3645_parse_node(struct lm3645 *chip)
{
	struct device *dev = &chip->client->dev;
	struct led_flash_setting *s;
	u32 flash_max_ua = LM3645_FLASH_MAX_UA;
	u32 flash_timeout_us = lm3645_timeout_us[15];
	u32 torch_max_ua = 360000;

	chip->led_node = device_get_next_child_node(dev, NULL);
	if (!chip->led_node) {
		dev_err(dev, "no LED child node\n");
		return -ENODEV;
	}

	fwnode_property_read_u32(chip->led_node, "flash-max-microamp",
				 &flash_max_ua);
	fwnode_property_read_u32(chip->led_node, "flash-max-timeout-us",
				 &flash_timeout_us);
	fwnode_property_read_u32(chip->led_node, "led-max-microamp",
				 &torch_max_ua);

	chip->torch_max_brightness =
		lm3645_clamp_code(torch_max_ua, LM3645_TORCH_MIN_UA,
				  LM3645_TORCH_STEP_UA);

	s = &chip->fled.brightness;
	s->min = LM3645_FLASH_MIN_UA;
	s->max = min(flash_max_ua, (u32)LM3645_FLASH_MAX_UA);
	s->step = LM3645_FLASH_STEP_UA;
	s->val = s->max;

	s = &chip->fled.timeout;
	s->min = lm3645_timeout_us[0];
	s->max = min(flash_timeout_us, lm3645_timeout_us[15]);
	s->step = 10000;
	s->val = s->max;

	return 0;
}

/*
 * Register a V4L2 flash sub-device wrapping the LED flash class device, so
 * the camera framework (media controller / libcamera) can drive the IR
 * illuminator via V4L2 flash controls. No-op when CONFIG_V4L2_FLASH_LED_CLASS
 * is disabled (v4l2_flash_init() returns NULL).
 */
static int lm3645_v4l2_setup(struct lm3645 *chip)
{
	struct led_classdev *led = &chip->fled.led_cdev;
	u32 torch_max_ua = chip->torch_max_brightness * LM3645_TORCH_STEP_UA +
			   LM3645_TORCH_MIN_UA;
	struct v4l2_flash_config cfg = {
		.intensity = {
			.min = LM3645_TORCH_MIN_UA,
			.max = torch_max_ua,
			.step = LM3645_TORCH_STEP_UA,
			.val = torch_max_ua,
		},
	};

	strscpy(cfg.dev_name, led->dev->kobj.name, sizeof(cfg.dev_name));

	chip->v4l2_flash = v4l2_flash_init(&chip->client->dev, chip->led_node,
					   &chip->fled, NULL, &cfg);
	return PTR_ERR_OR_ZERO(chip->v4l2_flash);
}

static int lm3645_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct led_classdev *cdev;
	struct led_init_data init_data = {};
	struct lm3645 *chip;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &lm3645_regmap);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	/* HWEN: bring the device out of hardware shutdown. */
	chip->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(chip->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->enable_gpio),
				     "failed to get enable GPIO\n");
	if (chip->enable_gpio)
		usleep_range(1000, 2000);

	/*
	 * Optional hardware control pins. When present the driver switches from
	 * software (I2C) strobing to gating the LM3645 STR1 / TOR-TX pins. Both
	 * start low (held inactive until the chip is armed over I2C).
	 */
	chip->strobe_gpio = devm_gpiod_get_optional(dev, "strobe", GPIOD_OUT_LOW);
	if (IS_ERR(chip->strobe_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->strobe_gpio),
				     "failed to get strobe GPIO\n");

	chip->torch_gpio = devm_gpiod_get_optional(dev, "torch", GPIOD_OUT_LOW);
	if (IS_ERR(chip->torch_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->torch_gpio),
				     "failed to get torch GPIO\n");

	ret = lm3645_parse_node(chip);
	if (ret)
		return ret;

	ret = lm3645_init_hw(chip);
	if (ret) {
		dev_err(dev, "device not responding on I2C\n");
		goto err_put_node;
	}

	chip->fled.ops = &lm3645_flash_ops;

	cdev = &chip->fled.led_cdev;
	cdev->brightness_set_blocking = lm3645_torch_brightness_set;
	cdev->max_brightness = chip->torch_max_brightness;
	cdev->flags = LED_DEV_CAP_FLASH | LED_CORE_SUSPENDRESUME;

	init_data.fwnode = chip->led_node;
	init_data.devicename = "lm3645";
	init_data.default_label = "flash";

	ret = devm_led_classdev_flash_register_ext(dev, &chip->fled,
						   &init_data);
	if (ret) {
		dev_err(dev, "failed to register flash LED: %d\n", ret);
		goto err_put_node;
	}

	ret = lm3645_v4l2_setup(chip);
	if (ret) {
		dev_err(dev, "failed to init V4L2 flash: %d\n", ret);
		goto err_put_node;
	}

	return 0;

err_put_node:
	fwnode_handle_put(chip->led_node);
	return ret;
}

static void lm3645_remove(struct i2c_client *client)
{
	struct lm3645 *chip = i2c_get_clientdata(client);

	/* release V4L2 wrapper before the devm-managed LED classdev goes away */
	v4l2_flash_release(chip->v4l2_flash);

	/* outputs off */
	regmap_update_bits(chip->regmap, LM3645_REG_CTRL2,
			   LM3645_CTRL2_DX_EN_MASK, 0);
	fwnode_handle_put(chip->led_node);
}

static const struct i2c_device_id lm3645_id[] = {
	{ "lm3645", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3645_id);

static const struct of_device_id lm3645_of_match[] = {
	{ .compatible = "ti,lm3645" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3645_of_match);

static struct i2c_driver lm3645_i2c_driver = {
	.driver = {
		.name = "lm3645",
		.of_match_table = lm3645_of_match,
	},
	.probe = lm3645_probe,
	.remove = lm3645_remove,
	.id_table = lm3645_id,
};
module_i2c_driver(lm3645_i2c_driver);

MODULE_DESCRIPTION("TI LM3645 LED flash driver");
MODULE_AUTHOR("SolidRun Ltd.");
MODULE_LICENSE("GPL");
