// SPDX-License-Identifier: GPL-2.0+
/*
 * GPIO Driver for Renesas GreenPAK SLG46826 Programmable Mixed-Signal Matrix.
 *
 * Matrix Inputs are used for reading levels on physical IO pins.
 * Matrix inputs "VDD" and "GND" provide high and low levels.
 * Interrupts / shange detection is not supported, users should poll for updates.
 *
 * This driver operates in SLG46826 ram, modifying only requested IOs.
 */

#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/module.h>

/* matrix inputs for use in connection matrix */
enum SLG46826_MATRIX_INPUT {
	SLG46826_I__GND = 0,
	SLG46826_I__IO00_DIGITAL_IN,
	SLG46826_I__IO01_DIGITAL_IN,
	SLG46826_I__IO02_DIGITAL_IN,
	SLG46826_I__IO03_DIGITAL_IN,
	SLG46826_I__IO04_DIGITAL_IN,
	SLG46826_I__IO05_DIGITAL_IN,
	SLG46826_I__IO08_DIGITAL_IN,
	SLG46826_I__IO09_DIGITAL_IN,
	SLG46826_I__IO10_DIGITAL_IN,
	SLG46826_I__IO11_DIGITAL_IN,
	SLG46826_I__IO12_DIGITAL_IN,
	SLG46826_I__IO13_DIGITAL_IN,
	SLG46826_I__IO14_DIGITAL_IN,
	SLG46826_I__LUT2_0_DFF0_OUT,
	SLG46826_I__LUT2_1_DFF1_OUT,
	SLG46826_I__LUT2_2_DFF2_OUT,
	SLG46826_I__LUT2_3_PGEN_OUT,
	SLG46826_I__LUT3_0_DFF3_OUT,
	SLG46826_I__LUT3_1_DFF4_OUT,
	SLG46826_I__LUT3_2_DFF5_OUT,
	SLG46826_I__LUT3_3_DFF6_OUT,
	SLG46826_I__LUT3_4_DFF7_OUT,
	SLG46826_I__LUT3_5_DFF8_OUT,
	SLG46826_I__LUT3_6_PIPEDLY_RIPP_CNT_OUT0,
	SLG46826_I__PIPEDLY_RIPP_CNT_OUT1,
	SLG46826_I__RIPP_CNT_OUT2,
	SLG46826_I__EDET_FILTER_OUT,
	SLG46826_I__PROG_DLY_EDET_OUT,
	SLG46826_I__MULTFUNC_8BIT_1__DLY_CNT_OUT,
	SLG46826_I__CKOSC1_MATRIX_IN,
	SLG46826_I__CKOSC0_MATRIX_IN,
	SLG46826_I__CKOSC2_MATRIX_IN,
	SLG46826_I__MULTFUNC_8BIT_2__DLY_CNT_OUT,
	SLG46826_I__MULTFUNC_8BIT_3__DLY_CNT_OUT,
	SLG46826_I__MULTFUNC_8BIT_4__DLY_CNT_OUT,
	SLG46826_I__MULTFUNC_8BIT_5__DLY_CNT_OUT,
	SLG46826_I__MULTFUNC_8BIT_6__DLY_CNT_OUT,
	SLG46826_I__MULTFUNC_8BIT_7__DLY_CNT_OUT,
	SLG46826_I__MULTFUNC_16BIT_0__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_8BIT_1__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_8BIT_2__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_8BIT_3__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_8BIT_4__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_8BIT_5__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_8BIT_6__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_8BIT_7__LUT_DFF_OUT,
	SLG46826_I__MULTFUNC_16BIT_0__DLY_CNT_OUT,
	SLG46826_I__I2C_VIRTUAL_7_IN,
	SLG46826_I__I2C_VIRTUAL_6_IN,
	SLG46826_I__I2C_VIRTUAL_5_IN,
	SLG46826_I__I2C_VIRTUAL_4_IN,
	SLG46826_I__I2C_VIRTUAL_3_IN,
	SLG46826_I__I2C_VIRTUAL_2_IN,
	SLG46826_I__I2C_VIRTUAL_1_IN,
	SLG46826_I__I2C_VIRTUAL_0_IN,
	SLG46826_I__ACMP0H_OUT,
	SLG46826_I__ACMP1H_OUT,
	SLG46826_I__ACMP2L_OUT,
	SLG46826_I__ACMP3L_OUT,
	SLG46826_I__CKOSC1_MATRIX_OUT,
	SLG46826_I__CKOSC0_MATRIX_OUT,
	SLG46826_I__POR_OUT,
	SLG46826_I__VDD,
	SLG46826_I__MAX,
};

/* translate io number to matrix input */
static enum SLG46826_MATRIX_INPUT slg46826_i_io_map[] = {
	SLG46826_I__IO00_DIGITAL_IN,
	SLG46826_I__IO01_DIGITAL_IN,
	SLG46826_I__IO02_DIGITAL_IN,
	SLG46826_I__IO03_DIGITAL_IN,
	SLG46826_I__IO04_DIGITAL_IN,
	SLG46826_I__IO05_DIGITAL_IN,
	SLG46826_I__MAX,
	SLG46826_I__MAX,
	SLG46826_I__IO08_DIGITAL_IN,
	SLG46826_I__IO09_DIGITAL_IN,
	SLG46826_I__IO10_DIGITAL_IN,
	SLG46826_I__IO11_DIGITAL_IN,
	SLG46826_I__IO12_DIGITAL_IN,
	SLG46826_I__IO13_DIGITAL_IN,
	SLG46826_I__IO14_DIGITAL_IN,
};

/* map numerical IO offset to matrix input */
static inline enum SLG46826_MATRIX_INPUT SLG46826_I__IO(unsigned int offset)
{
	if (offset >= ARRAY_SIZE(slg46826_i_io_map))
		return SLG46826_I__MAX;

	return slg46826_i_io_map[offset];
}

/* matrix outputs that can be connected to inputs */
enum SLG46826_MATRIX_OUTPUT {
	SLG46826_O__IO00_DIGITAL_OUT = 67,
	SLG46826_O__IO01_DIGITAL_OUT,
	SLG46826_O__IO01_DIGITAL_OUT_EN,
	SLG46826_O__IO02_DIGITAL_OUT,
	SLG46826_O__IO03_DIGITAL_OUT,
	SLG46826_O__IO04_DIGITAL_OUT,
	SLG46826_O__IO04_DIGITAL_OUT_EN,
	SLG46826_O__IO05_DIGITAL_OUT,
	SLG46826_O__IO05_DIGITAL_OUT_EN,
	SLG46826_O__IO06_DIGITAL_OUT,
	SLG46826_O__IO07_DIGITAL_OUT,
	SLG46826_O__IO08_DIGITAL_OUT,
	SLG46826_O__IO08_DIGITAL_OUT_EN,
	SLG46826_O__IO09_DIGITAL_OUT,
	SLG46826_O__IO09_DIGITAL_OUT_EN,
	SLG46826_O__IO10_DIGITAL_OUT,
	SLG46826_O__IO10_DIGITAL_OUT_EN,
	SLG46826_O__IO11_DIGITAL_OUT,
	SLG46826_O__IO11_DIGITAL_OUT_EN,
	SLG46826_O__IO12_DIGITAL_OUT,
	SLG46826_O__IO12_DIGITAL_OUT_EN,
	SLG46826_O__IO13_DIGITAL_OUT,
	SLG46826_O__IO13_DIGITAL_OUT_EN,
	SLG46826_O__IO14_DIGITAL_OUT,
	SLG46826_O__IO14_DIGITAL_OUT_EN,
	SLG46826_O_MAX = 96,
};

/* translate io number to matrix output */
static enum SLG46826_MATRIX_OUTPUT slg46826_o_io_map[] = {
	SLG46826_O__IO00_DIGITAL_OUT,
	SLG46826_O__IO01_DIGITAL_OUT,
	SLG46826_O__IO02_DIGITAL_OUT,
	SLG46826_O__IO03_DIGITAL_OUT,
	SLG46826_O__IO04_DIGITAL_OUT,
	SLG46826_O__IO05_DIGITAL_OUT,
	SLG46826_O__IO06_DIGITAL_OUT,
	SLG46826_O__IO07_DIGITAL_OUT,
	SLG46826_O__IO08_DIGITAL_OUT,
	SLG46826_O__IO09_DIGITAL_OUT,
	SLG46826_O__IO10_DIGITAL_OUT,
	SLG46826_O__IO11_DIGITAL_OUT,
	SLG46826_O__IO12_DIGITAL_OUT,
	SLG46826_O__IO13_DIGITAL_OUT,
	SLG46826_O__IO14_DIGITAL_OUT,
};

/* translate io number to matrix output */
static inline enum SLG46826_MATRIX_OUTPUT SLG46826_O__IO(unsigned int offset)
{
	if (offset >= ARRAY_SIZE(slg46826_o_io_map))
		return SLG46826_O_MAX;

	return slg46826_o_io_map[offset];
}

/* translate io number to output-enable matrix output */
static enum SLG46826_MATRIX_OUTPUT slg46826_o_io_oe_map[] = {
	SLG46826_O_MAX,
	SLG46826_O__IO01_DIGITAL_OUT_EN,
	SLG46826_O_MAX,
	SLG46826_O_MAX,
	SLG46826_O__IO04_DIGITAL_OUT_EN,
	SLG46826_O__IO05_DIGITAL_OUT_EN,
	SLG46826_O_MAX,
	SLG46826_O_MAX,
	SLG46826_O__IO08_DIGITAL_OUT_EN,
	SLG46826_O__IO09_DIGITAL_OUT_EN,
	SLG46826_O__IO10_DIGITAL_OUT_EN,
	SLG46826_O__IO11_DIGITAL_OUT_EN,
	SLG46826_O__IO12_DIGITAL_OUT_EN,
	SLG46826_O__IO13_DIGITAL_OUT_EN,
	SLG46826_O__IO14_DIGITAL_OUT_EN,
};

/* translate io number to output-enable matrix output */
static inline enum SLG46826_MATRIX_OUTPUT SLG46826_O__IO_OE(unsigned int offset)
{
	if (offset >= ARRAY_SIZE(slg46826_o_io_oe_map))
		return SLG46826_O_MAX;

	return slg46826_o_io_oe_map[offset];
}

/*
 * Register Definitions
 */
/* connection matrix register per output */
#define SLG46826_O_REG(i) (0x00 + (6 * i) / 8)
#define SLG46826_O_SHIFT(i) ((6 * i) % 8)
#define SLG46826_O_MASK(i) (0x3f << SLG46826_O_SHIFT(i))

/* matrix registers per input */
#define SLG46826_I__REG(i) (0x74 + i / 8)
#define SLG46826_I__SHIFT(i) (i % 8)
#define SLG46826_I__MASK(i) (1 << SLG46826_I__SHIFT(i))

#define SLG46826_IO00_CONFIG_REG 0x61
#define SLG46826_IO01_CONFIG_REG 0x62
#define SLG46826_IO02_CONFIG_REG 0x64
#define SLG46826_IO03_CONFIG_REG 0x65
#define SLG46826_IO04_CONFIG_REG 0x66
#define SLG46826_IO05_CONFIG_REG 0x67
#define SLG46826_SCL_CONFIG_REG 0x68
#define SLG46826_SDA_CONFIG_REG 0x69
#define SLG46826_IO06_CONFIG_REG 0x6a
#define SLG46826_IO07_CONFIG_REG 0x6b
#define SLG46826_IO08_CONFIG_REG 0x6c
#define SLG46826_IO09_CONFIG_REG 0x6e
#define SLG46826_IO10_CONFIG_REG 0x6f
#define SLG46826_IO11_CONFIG_REG 0x70
#define SLG46826_IO12_CONFIG_REG 0x71
#define SLG46826_IO13_CONFIG_REG 0x72
#define SLG46826_IO14_CONFIG_REG 0x73

/* translate io number to register number */
static int slg46826_io_config_reg_map[] = {
	SLG46826_IO00_CONFIG_REG,
	SLG46826_IO01_CONFIG_REG,
	SLG46826_IO02_CONFIG_REG,
	SLG46826_IO03_CONFIG_REG,
	SLG46826_IO04_CONFIG_REG,
	SLG46826_IO05_CONFIG_REG,
	SLG46826_IO06_CONFIG_REG,
	SLG46826_IO07_CONFIG_REG,
	SLG46826_IO08_CONFIG_REG,
	SLG46826_IO09_CONFIG_REG,
	SLG46826_IO10_CONFIG_REG,
	SLG46826_IO11_CONFIG_REG,
	SLG46826_IO12_CONFIG_REG,
	SLG46826_IO13_CONFIG_REG,
	SLG46826_IO14_CONFIG_REG,
};

/* translate io number to register number */
static inline int SLG46826_IO_CONFIG_REG(unsigned int offset)
{
	if (offset >= ARRAY_SIZE(slg46826_io_config_reg_map))
		return -EINVAL;

	return slg46826_io_config_reg_map[offset];
}

#define SLG46826_CONFIG_DRIVE_STRENGTH BIT(2) /* 0 = weak, 1 = strong */
#define SLG46826_CONFIG_OPENDRAIN BIT(3) /* 0 = push-pull, 1 = open-drain */
#define SLG46826_CONFIG_PULL_MASK GENMASK(5,4)
#define SLG46826_CONFIG_PULL_NONE 0x00
#define SLG46826_CONFIG_PULL_10K 0x10
#define SLG46826_CONFIG_PULL_100K 0x20
#define SLG46826_CONFIG_PULL_1M 0x30
#define SLG46826_CONFIG_PULLUP BIT(6) /* 0 = down, 1 = up */
#define SLG46826_CONFIG_OUTPUTENABLE BIT(7)

/* total number IOs */
#define SLG46826_IO_MAX 15

struct slg46826 {
	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	struct mutex i2c_lock;
};

/* get connected input for given matrix output */
static int slg46826_get_matrix_output(struct slg46826 *priv, enum SLG46826_MATRIX_OUTPUT output)
{
	s32 val;

	if (output >= SLG46826_O_MAX)
		return -EINVAL;

	/* read matrix output (2 bytes at a time as value can overlap) */
	val = i2c_smbus_read_word_data(priv->client, SLG46826_O_REG(output));
	if (val < 0)
		return val;

	/* shift value */
	val &= SLG46826_O_MASK(output);
	val >>= SLG46826_O_SHIFT(output);

	/* deliver result */
	return val;
}

/* set connected input for given matrix output */
static int slg46826_set_matrix_output(struct slg46826 *priv, enum SLG46826_MATRIX_OUTPUT output, enum SLG46826_MATRIX_INPUT input)
{
	s32 ret, val;

	if (output >= SLG46826_O_MAX || input >= SLG46826_I__MAX)
		return -EINVAL;

	/* protect read-modify-write cycle */
	mutex_lock(&priv->i2c_lock);

	/* read matrix output (2 bytes at a time as value can overlap) */
	val = i2c_smbus_read_word_data(priv->client, SLG46826_O_REG(output));
	if (val < 0) {
		mutex_unlock(&priv->i2c_lock);
		return val;
	}

	/* update value */
	val &= ~SLG46826_O_MASK(output);
	val |= input << SLG46826_O_SHIFT(output);

	/* write matrix output */
	ret = i2c_smbus_write_word_data(priv->client, SLG46826_O_REG(output), val);

	mutex_unlock(&priv->i2c_lock);
	return ret;
}

static int slg46826_get_matrix_input(struct slg46826 *priv, enum SLG46826_MATRIX_INPUT input)
{
	int val;

	if (input >= SLG46826_I__MAX)
		return -EINVAL;

	/* read matrix input */
	val = i2c_smbus_read_byte_data(priv->client, SLG46826_I__REG(input));
	if (val < 0)
		return val;

	/* apply mask to select single input */
	val &= SLG46826_I__MASK(input);

	/* no need to shif, result is single bit */
	return !!val;
}

static int slg46826_get_io_config_reg(struct slg46826 *priv, unsigned int offset)
{
	int reg;

	reg = SLG46826_IO_CONFIG_REG(offset);
	if (reg < 0)
		return reg;

	/* read io config register */
	return i2c_smbus_read_byte_data(priv->client, reg);
}

static int slg46826_set_io_config_reg(struct slg46826 *priv, unsigned int offset, u8 value)
{
	int reg;

	reg = SLG46826_IO_CONFIG_REG(offset);
	if (reg < 0)
		return reg;

	/* write io config register */
	return i2c_smbus_write_byte_data(priv->client, reg, value);
}

/* indicate whether the ioconfig register supports output-enable bit */
inline bool SLG46826_IO_CONFIG_REG_SUPPORTS_OE(unsigned int offset)
{
	/* config register and matrix OE are mutually exclusive */
	return SLG46826_O__IO_OE(offset) == SLG46826_O_MAX;
}

static int slg46826_gpio_get_direction(struct gpio_chip *gc,
				       unsigned int offset)
{
	struct slg46826 *priv = gpiochip_get_data(gc);
	int val;

	if (SLG46826_IO_CONFIG_REG_SUPPORTS_OE(offset)) {
		/* some IOs use config register for output-enable */
		val = slg46826_get_io_config_reg(priv, offset);
		if (val < 0)
			return val;

		/* test output-enable bit */
		if (val & SLG46826_CONFIG_OUTPUTENABLE)
			return GPIO_LINE_DIRECTION_OUT;
		else
			return GPIO_LINE_DIRECTION_IN;
	} else {
		/* others use matrix output */
		val = slg46826_get_matrix_output(priv, SLG46826_O__IO_OE(offset));
		if (val < 0)
			return val;

		/* evaluate value */
		switch(val) {
		case SLG46826_I__GND:
			/* shortcut for constant 0 */
			return GPIO_LINE_DIRECTION_IN;
		case SLG46826_I__VDD:
			/* shortcut for constant 1 */
			return GPIO_LINE_DIRECTION_OUT;
		default:
			/* look-up value from matrix */
			val = slg46826_get_matrix_input(priv, val);
			if (val < 0)
				return val;
			else if (val)
				return GPIO_LINE_DIRECTION_OUT;
			else
				return GPIO_LINE_DIRECTION_IN;
		};
	}
}

static int slg46826_gpio_get_value(struct gpio_chip *gc, unsigned offset)
{
	struct slg46826 *priv = gpiochip_get_data(gc);
	int dir, val;

	dir = slg46826_gpio_get_direction(gc, offset);
	if (dir < 0)
		return dir;

	if (dir == GPIO_LINE_DIRECTION_IN) {
		/* read matrix-input */
		return slg46826_get_matrix_input(priv, SLG46826_I__IO(offset));
	} else {
		/* read matrix output */
		val = slg46826_get_matrix_output(priv, SLG46826_O__IO(offset));
		if (val < 0)
			return val;

		/* evaluate value */
		switch(val) {
		case SLG46826_I__GND:
			/* shortcut for constant 0 */
			return 0;
		case SLG46826_I__VDD:
			/* shortcut for constant 1 */
			return 1;
		default:
			/* look-up value from matrix */
			return slg46826_get_matrix_input(priv, val);
		};
	}
}

static void slg46826_gpio_set_value(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct slg46826 *priv = gpiochip_get_data(gc);
	int ret;

	ret = slg46826_set_matrix_output(priv, SLG46826_O__IO(offset),
					 value ? SLG46826_I__VDD : SLG46826_I__GND);
	if (ret)
		dev_err(&priv->client->dev, "failed to set matrix output: %d\n", ret);
}

static int slg46826_set_gpio_direction(struct gpio_chip *gc, unsigned int offset, bool output)
{
	struct slg46826 *priv = gpiochip_get_data(gc);
	int ret, val;

	if (SLG46826_IO_CONFIG_REG_SUPPORTS_OE(offset)) {
		/* some IOs use config register for output-enable */
		val = slg46826_get_io_config_reg(priv, offset);
		if (val < 0)
			return val;

		val &= ~SLG46826_CONFIG_OUTPUTENABLE;
		if (output)
			val |= SLG46826_CONFIG_OUTPUTENABLE;

		ret = slg46826_set_io_config_reg(priv, offset, val);
	} else {
		/* others use matrix output */
		ret = slg46826_set_matrix_output(priv, SLG46826_O__IO_OE(offset),
						 output ? SLG46826_I__VDD : SLG46826_I__GND);
	}

	return ret;
}

static int slg46826_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	return slg46826_set_gpio_direction(gc, offset, false);
}

static int slg46826_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	/* first set value */
	slg46826_gpio_set_value(gc, offset, value);

	/* then set direction */
	return slg46826_set_gpio_direction(gc, offset, true);
}

static int slg46826_gpio_set_config(struct gpio_chip *gc,
				    unsigned int offset, unsigned long config)
{
	struct slg46826 *priv = gpiochip_get_data(gc);
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	int ret, val;

	val = slg46826_get_io_config_reg(priv, offset);
	if (val)
		return val;

	switch(param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		/* set open-drain bit */
		val |= SLG46826_CONFIG_OPENDRAIN;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		/* clear open-drain bit */
		val &= ~SLG46826_CONFIG_OPENDRAIN;
		break;
	case PIN_CONFIG_OUTPUT:
		/* set value */
		slg46826_gpio_set_value(gc, offset, arg);

		fallthrough;
	case PIN_CONFIG_OUTPUT_ENABLE:
		/* set direction to output */
		ret = slg46826_set_gpio_direction(gc, offset, 1);
		if (ret)
			return ret;
		break;
	default:
		dev_warn(&priv->client->dev, "set config param %#0x = %#0x not supported\n", param, arg);
		return -ENOTSUPP;
	}

	return slg46826_set_io_config_reg(priv, offset, val);
}

static int slg46826_probe(struct i2c_client *client,
			 const struct i2c_device_id *i2c_id)
{
	struct slg46826 *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->client = client;

	priv->gpio_chip.can_sleep = true;
	priv->gpio_chip.set_config = slg46826_gpio_set_config;
	priv->gpio_chip.get_direction = slg46826_gpio_get_direction;
	priv->gpio_chip.get = slg46826_gpio_get_value;
	priv->gpio_chip.set = slg46826_gpio_set_value;
	priv->gpio_chip.direction_input = slg46826_gpio_direction_input;
	priv->gpio_chip.direction_output = slg46826_gpio_direction_output;
	priv->gpio_chip.base = -1;
	priv->gpio_chip.ngpio = 15;
	priv->gpio_chip.label = dev_name(&client->dev);
	priv->gpio_chip.parent = &client->dev;
	priv->gpio_chip.owner = THIS_MODULE;

	ret = devm_gpiochip_add_data(&client->dev, &priv->gpio_chip, priv);

	mutex_init(&priv->i2c_lock);

	return ret;
}

static int slg46826_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id slg46826_id[] = {
	{ .name = "slg46826-gpio", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, slg46826_id);

static const struct of_device_id slg46826_dt_ids[] = {
	{ .compatible = "renesas,slg46826-gpio", },
	{ }
};

MODULE_DEVICE_TABLE(of, slg46826_dt_ids);

static struct i2c_driver slg46826_driver = {
	.driver = {
		.name	= "slg46826-gpio",
		.of_match_table = slg46826_dt_ids,
	},
	.probe		= slg46826_probe,
	.remove		= slg46826_remove,
	.id_table	= slg46826_id,
};

static int __init slg46826_init(void)
{
	return i2c_add_driver(&slg46826_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(slg46826_init);

static void __exit slg46826_exit(void)
{
	i2c_del_driver(&slg46826_driver);
}
module_exit(slg46826_exit);

MODULE_AUTHOR("Josua Mayer <josua@solid-run.com>");
MODULE_DESCRIPTION("GPIO expander driver for SLG46826");
MODULE_LICENSE("GPL v2");
