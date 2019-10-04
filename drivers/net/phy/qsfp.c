// SPDX-License-Identifier: GPL-2.0
#define DEBUG
#include <linux/acpi.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "mdio-i2c.h"
#include "sff.h"
#include "sfp.h"
#include "swphy.h"

enum {
	GPIO_MODPRS,
	GPIO_INTL,
	GPIO_MAX,

	SFP_F_PRESENT = BIT(GPIO_MODPRS),
	SFP_F_INTL = BIT(GPIO_INTL),
	SFP_F_LOS = BIT(GPIO_MAX + 0),
	SFP_F_TX_FAULT = BIT(GPIO_MAX + 1),
	SFP_F_TX_DISABLE = BIT(GPIO_MAX + 2),

	SFP_E_ATTACH = 0,
	SFP_E_DETACH,
	SFP_E_INSERT,
	SFP_E_REMOVE,
	SFP_E_DEV_DOWN,
	SFP_E_DEV_UP,
	SFP_E_TX_FAULT,
	SFP_E_TX_CLEAR,
	SFP_E_LOS_HIGH,
	SFP_E_LOS_LOW,
	SFP_E_TIMEOUT,

	SFP_MOD_EMPTY = 0,
	SFP_MOD_ERROR,
	SFP_MOD_PROBE,
	SFP_MOD_WATTACH,
	SFP_MOD_PRESENT,

	SFP_DEV_DETACHED = 0,
	SFP_DEV_DOWN,
	SFP_DEV_UP,

	SFP_S_DOWN = 0,
	SFP_S_ERROR,
	SFP_S_WPOWER,
	SFP_S_WTXEN,
	SFP_S_INIT,
	SFP_S_WAIT_LOS,
	SFP_S_LINK_UP,
	SFP_S_TX_FAULT,
	SFP_S_REINIT,
	SFP_S_TX_DISABLE,
};

static const char  * const mod_state_strings[] = {
	[SFP_MOD_EMPTY] = "empty",
	[SFP_MOD_ERROR] = "error",
	[SFP_MOD_PROBE] = "probe",
	[SFP_MOD_WATTACH] = "wattach",
	[SFP_MOD_PRESENT] = "present",
};

static const char *mod_state_to_str(unsigned short mod_state)
{
	if (mod_state >= ARRAY_SIZE(mod_state_strings))
		return "Unknown module state";
	return mod_state_strings[mod_state];
}

static const char * const dev_state_strings[] = {
	[SFP_DEV_DETACHED] = "unattached",
	[SFP_DEV_DOWN] = "down",
	[SFP_DEV_UP] = "up",
};

static const char *dev_state_to_str(unsigned short dev_state)
{
	if (dev_state >= ARRAY_SIZE(dev_state_strings))
		return "Unknown device state";
	return dev_state_strings[dev_state];
}

static const char * const event_strings[] = {
	[SFP_E_ATTACH] = "attach",
	[SFP_E_DETACH] = "detach",
	[SFP_E_INSERT] = "insert",
	[SFP_E_REMOVE] = "remove",
	[SFP_E_DEV_DOWN] = "dev_down",
	[SFP_E_DEV_UP] = "dev_up",
	[SFP_E_TX_FAULT] = "tx_fault",
	[SFP_E_TX_CLEAR] = "tx_clear",
	[SFP_E_LOS_HIGH] = "los_high",
	[SFP_E_LOS_LOW] = "los_low",
	[SFP_E_TIMEOUT] = "timeout",
};

static const char *event_to_str(unsigned short event)
{
	if (event >= ARRAY_SIZE(event_strings))
		return "Unknown event";
	return event_strings[event];
}

static const char * const sm_state_strings[] = {
	[SFP_S_DOWN] = "down",
	[SFP_S_ERROR] = "error",
	[SFP_S_WPOWER] = "wpower",
	[SFP_S_WTXEN] = "wtxen",
	[SFP_S_INIT] = "init",
	[SFP_S_WAIT_LOS] = "wait_los",
	[SFP_S_LINK_UP] = "link_up",
	[SFP_S_TX_FAULT] = "tx_fault",
	[SFP_S_REINIT] = "reinit",
	[SFP_S_TX_DISABLE] = "rx_disable",
};

static const char *sm_state_to_str(unsigned short sm_state)
{
	if (sm_state >= ARRAY_SIZE(sm_state_strings))
		return "Unknown state";
	return sm_state_strings[sm_state];
}

static const char *gpio_of_names[] = {
	"mod-prs",
	"intl",
};

static const enum gpiod_flags gpio_flags[] = {
	GPIOD_IN,
	GPIOD_IN,
};

#define T_INIT_JIFFIES	msecs_to_jiffies(300)
#define T_RESET_US	10
#define T_FAULT_RECOVER	msecs_to_jiffies(1000)

/*
 * SFF8436 defines the time from power on until the module responds to data
 * transmission over the serial bus as t_serial, 2 seconds.
 */
#define T_SERIAL	msecs_to_jiffies(2000)
#define T_OFF_PDOWN	msecs_to_jiffies(300)
#define T_OFF_TXDIS	msecs_to_jiffies(400)
#define T_PROBE_RETRY	msecs_to_jiffies(100)

/* SFP modules appear to always have their PHY configured for bus address
 * 0x56 (which with mdio-i2c, translates to a PHY address of 22).
 */
#define SFP_PHY_ADDR	22

struct sff_data {
	unsigned int gpios;
	bool (*module_supported)(const struct sfp_eeprom_id *id);
};

struct qsfp {
	struct device *dev;
	struct i2c_adapter *i2c;
	struct mii_bus *i2c_mii;
//	struct sfp_bus *sfp_bus;
	const struct sff_data *type;
	u32 max_power_mW;

	int (*read)(struct qsfp *, u8, u8, void *, size_t);
	int (*write)(struct qsfp *, u8, u8, void *, size_t);

	struct gpio_desc *gpio[GPIO_MAX];
	int gpio_irq[GPIO_MAX];

	bool need_poll;

	struct mutex st_mutex;			/* Protects state */
	unsigned int state;
	struct delayed_work poll;
	struct delayed_work timeout;
	struct mutex sm_mutex;			/* Protects state machine */
	unsigned char sm_mod_state;
	unsigned char sm_dev_state;
	unsigned short sm_state;
	unsigned int sm_retries;

	// Module data
	struct qsfp_sff8x36_id id;
	unsigned int module_power_mW;
	bool module_flat_mem;
	u8 module_power_class;
	u8 module_revision;
	u8 module_irq_flags[19];
	u8 request_tx_disable;
	u8 current_tx_disable;

#if IS_ENABLED(CONFIG_HWMON)
	struct sfp_diag diag;
	struct device *hwmon_dev;
	char *hwmon_name;
	__be16 thresholds[48];
#endif

};

static const struct of_device_id qsfp_of_match[] = {
	{ .compatible = "sff,qsfp", },
	{ .compatible = "sff,qsfp+", },
	{ .compatible = "sff,qsfp28", },
	{ },
};
MODULE_DEVICE_TABLE(of, qsfp_of_match);

static unsigned long poll_jiffies;

static int qsfp_i2c_read(struct qsfp *qsfp, u8 page, u8 dev_addr, void *buf,
			 size_t len)
{
	struct i2c_adapter *i2c = qsfp->i2c;
	struct i2c_msg msgs[2];
	size_t this_len;
	u8 page_buf[2];
	int ret;

	page_buf[0] = 0x7f;
	page_buf[1] = page;

	msgs[0].addr = 0x50;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(page_buf);
	msgs[0].buf = page_buf;
	msgs[1].addr = 0x50;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	i2c_lock_bus(i2c, I2C_LOCK_SEGMENT);

	if (dev_addr >= 128 && !qsfp->module_flat_mem)
		ret = __i2c_transfer(i2c, msgs, 1);
	else
		ret = 1;
	if (ret == 1) {
		msgs[0].len = 1;
		msgs[0].buf = &dev_addr;

		while (len) {
			this_len = len;
			if (this_len > 16)
				this_len = 16;

			msgs[1].len = this_len;

			ret = __i2c_transfer(qsfp->i2c, msgs, ARRAY_SIZE(msgs));

			if (ret != ARRAY_SIZE(msgs))
				break;

			msgs[1].buf += this_len;
			dev_addr += this_len;
			len -= this_len;
		}
	}
	i2c_unlock_bus(i2c, I2C_LOCK_SEGMENT);

	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static int qsfp_i2c_write(struct qsfp *qsfp, u8 page, u8 dev_addr, void *buf,
			  size_t len)
{
	struct i2c_adapter *i2c = qsfp->i2c;
	struct i2c_msg msgs[1];
	u8 page_buf[2];
	u8 *p;
	int ret;

	p = kmalloc(1 + len, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p[0] = dev_addr;
	memcpy(&p[1], buf, len);

	page_buf[0] = 0x7f;
	page_buf[1] = page;

	msgs[0].addr = 0x50;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(page_buf);
	msgs[0].buf = page_buf;

	i2c_lock_bus(i2c, I2C_LOCK_SEGMENT);
	if (dev_addr >= 128 && !qsfp->module_flat_mem)
		ret = __i2c_transfer(i2c, msgs, 1);
	else
		ret = 1;
	if (ret == 1) {
		msgs[0].addr = 0x50;
		msgs[0].flags = 0;
		msgs[0].len = 1 + len;
		msgs[0].buf = p;

		ret = __i2c_transfer(qsfp->i2c, msgs, ARRAY_SIZE(msgs));
	}
	i2c_unlock_bus(i2c, I2C_LOCK_SEGMENT);

	kfree(p);

	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static int qsfp_i2c_configure(struct qsfp *qsfp, struct i2c_adapter *i2c)
{
	struct mii_bus *i2c_mii;
	int ret;

	if (!i2c_check_functionality(i2c, I2C_FUNC_I2C))
		return -EINVAL;

	qsfp->i2c = i2c;
	qsfp->read = qsfp_i2c_read;
	qsfp->write = qsfp_i2c_write;

	i2c_mii = mdio_i2c_alloc(qsfp->dev, i2c);
	if (IS_ERR(i2c_mii))
		return PTR_ERR(i2c_mii);

	i2c_mii->name = "QSFP I2C Bus";
	i2c_mii->phy_mask = ~0;

	ret = mdiobus_register(i2c_mii);
	if (ret < 0) {
		mdiobus_free(i2c_mii);
		return ret;
	}

	qsfp->i2c_mii = i2c_mii;

	return 0;
}

/* Interface */
static int qsfp_read(struct qsfp *qsfp, u16 addr, void *buf, size_t len)
{
	return qsfp->read(qsfp, addr >> 8, addr, buf, len);
}

static int qsfp_write(struct qsfp *qsfp, u16 addr, void *buf, size_t len)
{
	return qsfp->write(qsfp, addr >> 8, addr, buf, len);
}

static int qsfp_modb(struct qsfp *qsfp, u16 addr, u8 mask, u8 set)
{
	int ret;
	u8 val;

	ret = qsfp_read(qsfp, addr, &val, sizeof(val));
	if (ret < 0)
		return ret;

	val &= ~mask;
	val |= set & mask;

	return qsfp_write(qsfp, addr, &val, sizeof(val));
}

static unsigned int sfp_check(void *buf, size_t len)
{
	u8 *p, check;

	for (p = buf, check = 0; len; p++, len--)
		check += *p;

	return check;
}

#if IS_ENABLED(CONFIG_HWMON)
static umode_t qsfp_hwmon_is_visible(const void *data,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel)
{
	const struct qsfp *qsfp = data;

	switch (type) {
	case hwmon_temp:
		/* Temperature monitoring is optional, but this bit only
		 * exists in SFF8636 revision 2.8+ */
		if (qsfp->module_revision >= SFF8X36_REV_8636_2_8 &&
		    !(qsfp->id.ext.sff8636.diagmon & SFF8636_DIAGMON_TEMP))
			break;
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		case hwmon_temp_lcrit:
		case hwmon_temp_min:
		case hwmon_temp_max:
		case hwmon_temp_crit:
			return !qsfp->module_flat_mem ? 0444 : 0;
		}
		break;
	case hwmon_in:
		/* Supply monitoring is optional, but this bit only
		 * exists in SFF8636 revision 2.8+ */
		if (qsfp->module_revision >= SFF8X36_REV_8636_2_8 &&
		    !(qsfp->id.ext.sff8636.diagmon & SFF8636_DIAGMON_VCC))
			break;
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		case hwmon_in_lcrit:
		case hwmon_in_min:
		case hwmon_in_max:
		case hwmon_in_crit:
			return !qsfp->module_flat_mem ? 0444 : 0;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
		case hwmon_curr_label:
			return 0444;
		case hwmon_curr_lcrit:
		case hwmon_curr_min:
		case hwmon_curr_max:
		case hwmon_curr_crit:
			return !qsfp->module_flat_mem ? 0444 : 0;
		}
		break;
	case hwmon_power:
		/* TX power monitoring is optional, but this bit only
		 * exists in SFF8636 revision 1.9+ - we only know 2.0+ */
		if (channel >= 4 &&
		    qsfp->module_revision >= SFF8X36_REV_8636_2_0 &&
		    !(qsfp->id.ext.sff8636.diagmon & SFF8636_DIAGMON_TXPWR))
			break;
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_label:
			return 0444;
		case hwmon_power_lcrit:
		case hwmon_power_min:
		case hwmon_power_max:
		case hwmon_power_crit:
			return !qsfp->module_flat_mem ? 0444 : 0;
		}
		break;
	default:
		break;
	}
	return 0;
}

static int qsfp_hwmon_temp(struct qsfp *qsfp, u32 attr, long *value)
{
	__be16 temp;
	int ret;

	switch (attr) {
	case hwmon_temp_input:
		ret = qsfp_read(qsfp, SFF8X36_TEMPERATURE, &temp, sizeof(temp));
		if (ret < 0)
			return ret;
		break;
	case hwmon_temp_lcrit:
		temp = qsfp->thresholds[1];
		break;
	case hwmon_temp_min:
		temp = qsfp->thresholds[3];
		break;
	case hwmon_temp_max:
		temp = qsfp->thresholds[2];
		break;
	case hwmon_temp_crit:
		temp = qsfp->thresholds[0];
		break;
	default:
		return -EOPNOTSUPP;
	}

	*value = DIV_ROUND_CLOSEST(((s16)be16_to_cpu(temp)) * 1000, 256);

	return 0;
}

static int qsfp_hwmon_vcc(struct qsfp *qsfp, u32 attr, long *value)
{
	__be16 volt;
	int ret;

	switch (attr) {
	case hwmon_in_input:
		ret = qsfp_read(qsfp, SFF8X36_SUPPLY_VOLTAGE,
				&volt, sizeof(volt));
		if (ret < 0)
			return ret < 0;
		break;
	case hwmon_in_lcrit:
		volt = qsfp->thresholds[9];
		break;
	case hwmon_in_min:
		volt = qsfp->thresholds[11];
		break;
	case hwmon_in_max:
		volt = qsfp->thresholds[10];
		break;
	case hwmon_in_crit:
		volt = qsfp->thresholds[8];
		break;
	default:
		return -EOPNOTSUPP;
	}

	*value = DIV_ROUND_CLOSEST(be16_to_cpu(volt), 10);

	return 0;
}

static int qsfp_hwmon_bias(struct qsfp *qsfp, u32 attr, int chan, long *value)
{
	__be16 bias;
	u8 addr;
	int ret;

	switch (attr) {
	case hwmon_curr_input:
		addr = SFF8X36_TX_BIAS + 2 * chan;

		ret = qsfp_read(qsfp, addr, &bias, sizeof(bias));
		if (ret < 0)
			return ret < 0;
		break;
	case hwmon_curr_lcrit:
		bias = qsfp->thresholds[29];
		break;
	case hwmon_curr_min:
		bias = qsfp->thresholds[31];
		break;
	case hwmon_curr_max:
		bias = qsfp->thresholds[30];
		break;
	case hwmon_curr_crit:
		bias = qsfp->thresholds[28];
		break;
	default:
		return -EOPNOTSUPP;
	}

	*value = DIV_ROUND_CLOSEST(be16_to_cpu(bias), 10);

	return 0;
}

static int qsfp_hwmon_power(struct qsfp *qsfp, u32 attr, int chan, long *value)
{
	__be16 pwr;
	u8 addr;
	int ret;

	switch (attr) {
	case hwmon_power_input:
		addr = 2 * (chan & 3) +
		       (chan & 4 ? SFF8X36_TX_POWER : SFF8X36_RX_POWER);

		ret = qsfp_read(qsfp, addr, &pwr, sizeof(pwr));
		if (ret < 0)
			return ret;
		break;
	case hwmon_power_lcrit:
		pwr = qsfp->thresholds[chan & 4 ? 33 : 25];
		break;
	case hwmon_power_min:
		pwr = qsfp->thresholds[chan & 4 ? 35 : 27];
		break;
	case hwmon_power_max:
		pwr = qsfp->thresholds[chan & 4 ? 34 : 26];
		break;
	case hwmon_power_crit:
		pwr = qsfp->thresholds[chan & 4 ? 32 : 24];
		break;
	default:
		return -EOPNOTSUPP;
	}

	*value = DIV_ROUND_CLOSEST(be16_to_cpu(pwr), 10);

	return 0;
}

static int qsfp_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long *value)
{
	struct qsfp *qsfp = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		return qsfp_hwmon_temp(qsfp, attr, value);
	case hwmon_in:
		return qsfp_hwmon_vcc(qsfp, attr, value);
	case hwmon_curr:
		return qsfp_hwmon_bias(qsfp, attr, channel, value);
	case hwmon_power:
		return qsfp_hwmon_power(qsfp, attr, channel, value);
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static const char *qsfp_hwmon_power_labels[] = {
	"power_rx_1",
	"power_rx_2",
	"power_rx_3",
	"power_rx_4",
	"power_tx_1",
	"power_tx_2",
	"power_tx_3",
	"power_tx_4",
};

static const char *qsfp_hwmon_curr_labels[] = {
	"bias_tx_1",
	"bias_tx_2",
	"bias_tx_3",
	"bias_tx_4",
};

static int qsfp_hwmon_read_str(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_curr:
		if (attr != hwmon_curr_label || channel < 0 ||
		    channel > ARRAY_SIZE(qsfp_hwmon_curr_labels))
			break;
		*str = qsfp_hwmon_curr_labels[channel];
		return 0;
	case hwmon_power:
		if (attr != hwmon_power_label || channel < 0 ||
		    channel > ARRAY_SIZE(qsfp_hwmon_power_labels))
			break;
		*str = qsfp_hwmon_power_labels[channel];
		return 0;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static const struct hwmon_ops qsfp_hwmon_ops = {
	.is_visible = qsfp_hwmon_is_visible,
	.read = qsfp_hwmon_read,
	.read_string = qsfp_hwmon_read_str,
};

static const u32 qsfp_hwmon_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0,
};

static const struct hwmon_channel_info qsfp_hwmon_chip = {
	.type = hwmon_chip,
	.config = qsfp_hwmon_chip_config,
};

static const u32 qsfp_hwmon_temp_config[] = {
	HWMON_T_INPUT |
	HWMON_T_LCRIT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_CRIT,
	0,
};

static const struct hwmon_channel_info qsfp_hwmon_temp_channel_info = {
	.type = hwmon_temp,
	.config = qsfp_hwmon_temp_config,
};

static const u32 qsfp_hwmon_vcc_config[] = {
	HWMON_I_INPUT |
	HWMON_I_LCRIT | HWMON_I_MIN | HWMON_I_MAX | HWMON_I_CRIT,
	0,
};

static const struct hwmon_channel_info qsfp_hwmon_vcc_channel_info = {
	.type = hwmon_in,
	.config = qsfp_hwmon_vcc_config,
};

static u32 qsfp_hwmon_bias_config[] = {
	HWMON_C_INPUT | HWMON_C_LABEL |
	HWMON_C_LCRIT | HWMON_C_MIN | HWMON_C_MAX | HWMON_C_CRIT,
	HWMON_C_INPUT | HWMON_C_LABEL |
	HWMON_C_LCRIT | HWMON_C_MIN | HWMON_C_MAX | HWMON_C_CRIT,
	HWMON_C_INPUT | HWMON_C_LABEL |
	HWMON_C_LCRIT | HWMON_C_MIN | HWMON_C_MAX | HWMON_C_CRIT,
	HWMON_C_INPUT | HWMON_C_LABEL |
	HWMON_C_LCRIT | HWMON_C_MIN | HWMON_C_MAX | HWMON_C_CRIT,
	0,
};

static const struct hwmon_channel_info qsfp_hwmon_bias_channel_info = {
	.type = hwmon_curr,
	.config = qsfp_hwmon_bias_config,
};

static const u32 qsfp_hwmon_power_config[] = {
	/* Receive power */
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	/* Transmit power */
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	HWMON_P_INPUT | HWMON_P_LABEL |
	HWMON_P_LCRIT | HWMON_P_MIN | HWMON_P_MAX | HWMON_P_CRIT,
	0,
};

static const struct hwmon_channel_info qsfp_hwmon_power_channel_info = {
	.type = hwmon_power,
	.config = qsfp_hwmon_power_config,
};

static const struct hwmon_channel_info *qsfp_hwmon_info[] = {
	&qsfp_hwmon_chip,
	&qsfp_hwmon_vcc_channel_info,
	&qsfp_hwmon_temp_channel_info,
	&qsfp_hwmon_bias_channel_info,
	&qsfp_hwmon_power_channel_info,
	NULL,
};

static const struct hwmon_chip_info qsfp_hwmon_chip_info = {
	.ops = &qsfp_hwmon_ops,
	.info = qsfp_hwmon_info,
};

static int qsfp_hwmon_insert(struct qsfp *qsfp)
{
	int i, ret;

	if (!qsfp->module_flat_mem) {
		ret = qsfp_read(qsfp, SFF8X36_THRESHOLDS, qsfp->thresholds,
				sizeof(qsfp->thresholds));
		if (ret < 0)
			return ret < 0;
	}

	qsfp->hwmon_name = kstrdup(dev_name(qsfp->dev), GFP_KERNEL);
	if (!qsfp->hwmon_name)
		return -ENOMEM;

	for (i = 0; qsfp->hwmon_name[i]; i++)
		if (hwmon_is_bad_char(qsfp->hwmon_name[i]))
			qsfp->hwmon_name[i] = '_';

	qsfp->hwmon_dev = hwmon_device_register_with_info(qsfp->dev,
							  qsfp->hwmon_name,
							  qsfp,
							  &qsfp_hwmon_chip_info,
							  NULL);

	return PTR_ERR_OR_ZERO(qsfp->hwmon_dev);
}

static void qsfp_hwmon_remove(struct qsfp *qsfp)
{
	if (!IS_ERR_OR_NULL(qsfp->hwmon_dev)) {
		hwmon_device_unregister(qsfp->hwmon_dev);
		qsfp->hwmon_dev = NULL;
		kfree(qsfp->hwmon_name);
	}
}
#else
static int qsfp_hwmon_insert(struct qsfp *qsfp)
{
	return 0;
}

static void qsfp_hwmon_remove(struct qsfp *qsfp)
{
}
#endif

static const char *sff8436_encoding(unsigned int encoding)
{
	switch (encoding) {
	case SFF8024_ENCODING_UNSPEC:
		return "unspecified";
	case SFF8024_ENCODING_8436_64B66B:
		return "64b66b";
	case SFF8024_ENCODING_8B10B:
		return "8b10b";
	case SFF8024_ENCODING_4B5B:
		return "4b5b";
	case SFF8024_ENCODING_NRZ:
		return "NRZ";
	case SFF8024_ENCODING_8436_MANCHESTER:
		return "MANCHESTER";
	default:
		return "unknown";
	}
}

/* SFP state machine */
static void qsfp_sm_set_timer(struct qsfp *sfp, unsigned int timeout)
{
	if (timeout)
		mod_delayed_work(system_power_efficient_wq, &sfp->timeout,
				 timeout);
	else
		cancel_delayed_work(&sfp->timeout);
}

static void qsfp_sm_next(struct qsfp *sfp, unsigned int state,
			unsigned int timeout)
{
	sfp->sm_state = state;
	qsfp_sm_set_timer(sfp, timeout);
}

static void qsfp_sm_ins_next(struct qsfp *sfp, unsigned int state,
			    unsigned int timeout)
{
	sfp->sm_mod_state = state;
	qsfp_sm_set_timer(sfp, timeout);
}

static void sfp_sm_link_up(struct qsfp *sfp)
{
//	sfp_link_up(sfp->sfp_bus);
	qsfp_sm_next(sfp, SFP_S_LINK_UP, 0);
}

static void sfp_sm_link_down(struct qsfp *sfp)
{
//	sfp_link_down(sfp->sfp_bus);
}

static void sfp_sm_link_check_los(struct qsfp *sfp)
{
	sfp_sm_link_up(sfp);
}

static bool sfp_los_event_active(struct qsfp *sfp, unsigned int event)
{
	return 0;
}

static bool sfp_los_event_inactive(struct qsfp *sfp, unsigned int event)
{
	return 0;
}

static int qsfp_set_tx_disable(struct qsfp *qsfp, unsigned int val)
{
	return qsfp_modb(qsfp, SFF8X36_TX_DISABLE,
			 SFF8X36_TX_DISABLE_TX4 | SFF8X36_TX_DISABLE_TX3 |
			 SFF8X36_TX_DISABLE_TX2 | SFF8X36_TX_DISABLE_TX1, val);
}

static int qsfp_set_power(struct qsfp *qsfp, unsigned int val)
{
	return qsfp_modb(qsfp, SFF8X36_CTRL_93,
			 SFF8X36_CTRL_93_POWER_ORIDE |
			 SFF8X36_CTRL_93_POWER_SET |
			 SFF8636_CTRL_93_POWER_CLS5_7 |
			 SFF8636_CTRL_93_POWER_CLS8, val);
}

static void sfp_sm_fault(struct qsfp *sfp, bool warn)
{
	if (sfp->sm_retries && !--sfp->sm_retries) {
		dev_err(sfp->dev,
			"module persistently indicates fault, disabling\n");
		qsfp_sm_next(sfp, SFP_S_TX_DISABLE, 0);
	} else {
		if (warn)
			dev_err(sfp->dev, "module transmit fault indicated\n");

		qsfp_sm_next(sfp, SFP_S_TX_FAULT, T_FAULT_RECOVER);
	}
}

/* Device/upstream state machine - tracks whether we are attached to an
 * upstream, and whether the upstream is ready to pass data (i.o.w. up).
 */
static void qsfp_sm_device(struct qsfp *qsfp, unsigned int event)
{
	if (event == SFP_E_DETACH)
		qsfp->sm_dev_state = SFP_DEV_DETACHED;

	switch (qsfp->sm_dev_state) {
	default:
		if (event == SFP_E_ATTACH)
			qsfp->sm_dev_state = SFP_DEV_DOWN;
		break;

	case SFP_DEV_DOWN:
		if (event == SFP_E_DEV_UP)
			qsfp->sm_dev_state = SFP_DEV_UP;
		break;

	case SFP_DEV_UP:
		if (event == SFP_E_DEV_DOWN)
			qsfp->sm_dev_state = SFP_DEV_DOWN;
		break;
	}
}

static int qsfp_parse_power(struct qsfp *qsfp)
{
	unsigned int power_mW, power_class;
	u8 pwr, mask;
	int ret;

	if (qsfp->module_revision >= SFF8X36_REV_8636_2_8 &&
            qsfp->id.base.ext_identifier & BIT(5)) {
		ret = qsfp_read(qsfp, SFF8X36_CLS8_MAX_POWER, &pwr,
				sizeof(pwr));
		if (ret < 0)
			return ret;

		power_class = 8;
		power_mW = pwr * 100;
	} else {
		if (qsfp->id.base.identifier == SFF8024_ID_QSFP_8438)
			mask = 0xc0;
		else
			mask = 0xc3;

		switch (qsfp->id.base.ext_identifier & mask) {
		default:
			power_mW = 1500;
			power_class = 1;
			break;
		case 0x40:
			power_mW = 2000;
			power_class = 2;
			break;
		case 0x80:
			power_mW = 2500;
			power_class = 3;
			break;
		case 0xc0:
			power_mW = 3500;
			power_class = 4;
			break;
		case 0xc1:
			power_mW = 4000;
			power_class = 5;
			break;
		case 0xc2:
			power_mW = 4500;
			power_class = 6;
			break;
		case 0xc3:
			power_mW = 5000;
			power_class = 7;
			break;
		}
	}

	if (power_mW > qsfp->max_power_mW) {
		if (power_mW > 3500u) {
			dev_warn(qsfp->dev,
				 "Host does not support %u.%uW modules, limited to 3.5W\n",
				 power_mW / 1000, (power_mW / 100) % 10);
			return 0;
		} else {
			dev_err(qsfp->dev,
				"Host does not support %u.%uW modules\n",
				power_mW / 1000, (power_mW / 100) % 10);
			return -EINVAL;
		}
	}

	qsfp->module_power_mW = power_mW;
	qsfp->module_power_class = power_class;

	dev_info(qsfp->dev, "module power %u.%uW\n",
		 qsfp->module_power_mW / 1000,
		 (qsfp->module_power_mW / 100) % 10);

	return 0;
}

static void qsfp_mod_clear(struct qsfp *qsfp)
{
	qsfp->module_power_class = 0;
	qsfp->module_power_mW = 0;
	qsfp->module_flat_mem = false;
	qsfp->module_revision = 0;
	memset(&qsfp->id, 0, sizeof(qsfp->id));
	memset(&qsfp->module_irq_flags, 0, sizeof(qsfp->module_irq_flags));
}

static void qsfp_sm_mod_print(struct qsfp *qsfp)
{
	const struct qsfp_sff8x36_id *id = &qsfp->id;
	unsigned int br_nom, wavelen;
	bool copper;
	char length[16];
	char date[9];

	date[0] = id->ext.sff8436.datecode[4];
	date[1] = id->ext.sff8436.datecode[5];
	date[2] = '-';
	date[3] = id->ext.sff8436.datecode[2];
	date[4] = id->ext.sff8436.datecode[3];
	date[5] = '-';
	date[6] = id->ext.sff8436.datecode[0];
	date[7] = id->ext.sff8436.datecode[1];
	date[8] = '\0';

	if (id->base.br_nominal == 0) {
		br_nom = 0;
	} else if (id->base.br_nominal == 255 &&
		   qsfp->module_revision >= SFF8X36_REV_8636_1_3) {
		br_nom = 250 * id->ext.sff8636.baud_rate_nominal;
	} else {
		br_nom = 100 * id->base.br_nominal;
	}

	dev_info(qsfp->dev, "module %.*s %.*s rev %.*s sn %.*s dc %s\n",
		 (int)sizeof(id->base.vendor_name), id->base.vendor_name,
		 (int)sizeof(id->base.vendor_pn), id->base.vendor_pn,
		 (int)sizeof(id->base.vendor_rev), id->base.vendor_rev,
		 (int)sizeof(id->ext.sff8436.vendor_sn), id->ext.sff8436.vendor_sn,
		 date);

	dev_info(qsfp->dev, "  0x%02x %s, 0x%02x %s\n",
		 id->base.identifier,
		 id->base.identifier == 0x0c ? "QSFP (INF-8438)" :
		 id->base.identifier == 0x0d ? "QSFP+ (SFF-8636 or SFF-8436)" :
		 id->base.identifier == 0x11 ? "QSFP28 (SFF-8636)" : "unknown",
		 qsfp->module_revision,
		 qsfp->module_revision == 0x00 ? "unspecified" :
		 qsfp->module_revision == 0x01 ? "SFF-8436 4.8 or earlier" :
		 qsfp->module_revision == 0x02 ? "SFF-8436 4.8 or earlier+" :
		 qsfp->module_revision == 0x03 ? "SFF-8636 1.3 or earlier" :
		 qsfp->module_revision == 0x04 ? "SFF-8636 1.4" :
		 qsfp->module_revision == 0x05 ? "SFF-8636 1.5" :
		 qsfp->module_revision == 0x06 ? "SFF-8636 2.0" :
		 qsfp->module_revision == 0x07 ? "SFF-8636 2.5..2.7" :
		 qsfp->module_revision == 0x08 ? "SFF-8636 2.8..2.10" : "unknown");

	dev_info(qsfp->dev, "  %s connector, encoding %s, bitrate %u.%03u GBd\n",
		 sff_connector(id->base.connector),
		 sff8436_encoding(id->base.encoding),
		 br_nom / 1000, br_nom % 1000);

	dev_info(qsfp->dev, "  Compliance bytes: %*ph  %02x\n",
		 (int)sizeof(id->base.compliance), id->base.compliance,
		 id->ext.sff8636.link_codes);

	copper = id->base.compliance[0] & BIT(0) ||
		 id->base.compliance[3] & (BIT(2) | BIT(3));

	if (copper) {
		dev_info(qsfp->dev, "  Copper Attenuation @ 2.5 GHz: %udB\n",
			 id->base.copper_atten[0]);
		dev_info(qsfp->dev, "  Copper Attenuation @ 5 GHz  : %udB\n",
			 id->base.copper_atten[1]);
		dev_info(qsfp->dev, "  Copper length               : %s\n",
			 sff_link_len(length, sizeof(length),
				      id->base.length[4], 1));
	} else {
		wavelen = be16_to_cpup(&id->base.wavelength) * 5;
		dev_info(qsfp->dev, "  Wavelength %u.%02unm, fiber lengths:\n",
			 wavelen / 100, wavelen % 100);
		dev_info(qsfp->dev, "     9µm SM    : %s\n",
			 sff_link_len(length, sizeof(length),
				      id->base.length[0], 1));
		dev_info(qsfp->dev, "  62.5µm MM OM1: %s\n",
			 sff_link_len(length, sizeof(length),
				      id->base.length[3], 1));
		dev_info(qsfp->dev, "    50µm MM OM2: %s\n",
			 sff_link_len(length, sizeof(length),
				      id->base.length[2], 1));
		dev_info(qsfp->dev, "    50µm MM OM3: %s\n",
			 sff_link_len(length, sizeof(length),
				      id->base.length[1], 2));
		dev_info(qsfp->dev, "    50µm MM OM4: %s\n",
			 sff_link_len(length, sizeof(length),
				      id->base.length[4], 2));
	}
}

static int qsfp_sm_mod_probe(struct qsfp *sfp)
{
	struct qsfp_sff8x36_id_stat id_stat;
	struct qsfp_sff8x36_id id;
	//char options[80];
	u8 check;
	int ret;

	ret = qsfp_read(sfp, 0, &id_stat, sizeof(id_stat));
	if (ret < 0) {
		dev_err(sfp->dev, "failed to read EEPROM: %d\n", ret);
		return -EAGAIN;
	}

	// FIXME: check id_stat.identifier values

	// Wait for the module to report ready
	if (id_stat.status & SFF8X36_STAT_DATA_NOT_READY)
		return -EAGAIN;

	// Early setup - we need to know if this module has a page register
	sfp->module_flat_mem = id_stat.status & SFF8X36_STAT_FLAT_MEM;
	sfp->module_revision = id_stat.rev_compliance;

	ret = qsfp_read(sfp, SFF8X36_ID, &id, sizeof(id));
	if (ret < 0) {
		qsfp_mod_clear(sfp);
		dev_err(sfp->dev, "failed to read EEPROM: %d\n", ret);
		return -EAGAIN;
	}

	if (id.base.identifier != id_stat.identifier) {
		qsfp_mod_clear(sfp);
		dev_err(sfp->dev, "QSFP identifier mismatch: 0x%02x != 0x%02x\n",
			id_stat.identifier, id.base.identifier);
		return -EINVAL;
	}

	// Validate the checksum over the base structure
	check = sfp_check(&id.base, sizeof(id.base) - 1);
	if (check != id.base.cc_base) {
		qsfp_mod_clear(sfp);
		dev_err(sfp->dev,
			"EEPROM base structure checksum failure: 0x%02x != 0x%02x\n",
			check, id.base.cc_base);
		print_hex_dump(KERN_ERR, "sfp EE: ", DUMP_PREFIX_OFFSET,
			       16, 1, &id, sizeof(id), true);
		return -EINVAL;
	}

	// Validate the checksum over the extended structure
	check = sfp_check(&id.ext.sff8436, sizeof(id.ext.sff8436) - 1);
	if (check != id.ext.sff8436.cc_ext) {
		dev_err(sfp->dev,
			"EEPROM extended structure checksum failure: 0x%02x != 0x%02x\n",
			check, id.ext.sff8436.cc_ext);
		print_hex_dump(KERN_ERR, "sfp EE: ", DUMP_PREFIX_OFFSET,
			       16, 1, &id, sizeof(id), true);
		memset(&id.ext, 0, sizeof(id.ext));
	}

	sfp->id = id;

	if (id.ext.sff8436.options[3] & SFF8X36_OPTIONS195_TX_DISABLE) {
		// Active tx disable
		sfp->request_tx_disable = SFF8X36_TX_DISABLE_TX4 |
					  SFF8X36_TX_DISABLE_TX3 |
					  SFF8X36_TX_DISABLE_TX2 |
					  SFF8X36_TX_DISABLE_TX1;
		sfp->current_tx_disable = sfp->request_tx_disable;
		ret = qsfp_set_tx_disable(sfp, sfp->current_tx_disable);
		if (ret < 0) {
			qsfp_mod_clear(sfp);
			return ret;
		}
	}

	// Parse the module power requirements
	ret = qsfp_parse_power(sfp);
	if (ret) {
		qsfp_mod_clear(sfp);
		return ret;
	}

	qsfp_sm_mod_print(sfp);

	/* Check whether we support this module */
//	if (!sfp->type->module_supported(&id)) {
//		dev_err(sfp->dev,
//			"module is not supported - phys id 0x%02x 0x%02x\n",
//			id.base.phys_id, id.base.phys_ext_id);
//		return -EINVAL;
//	}

	return qsfp_hwmon_insert(sfp);
}

static void qsfp_sm_mod_present(struct qsfp *qsfp)
{
//	int ret;

	if (qsfp->sm_dev_state == SFP_DEV_DETACHED) {
		qsfp_sm_ins_next(qsfp, SFP_MOD_WATTACH, 0);
		return;
	}

	// Start the poller if there is no interrupt support if not running
	if (!qsfp->gpio_irq[GPIO_INTL])
		queue_delayed_work(system_wq, &qsfp->poll, poll_jiffies);

//	ret = sfp_module_insert(qsfp->sfp_bus, &id);
//	if (ret < 0)
//		qsfp_sm_ins_next(qsfp, SFP_MOD_ERROR, 0);
//	else
		qsfp_sm_ins_next(qsfp, SFP_MOD_PRESENT, 0);
}

static void qsfp_sm_mod_remove(struct qsfp *qsfp)
{
//	if (qsfp->sm_mod_state >= SFP_MOD_PRESENT)
//		sfp_module_remove(qsfp->sfp_bus);

	qsfp_hwmon_remove(qsfp);
	qsfp_mod_clear(qsfp);

	dev_info(qsfp->dev, "module removed\n");
}

/* Module state machine - this tracks the insertion state of the module,
 * probes the EEPROM to identify the type of module and the power level.
 *
 * We handle remove events outside of the main switch() as what
 * is required is fairly universal between the various states.
 */
static void qsfp_sm_module(struct qsfp *qsfp, unsigned int event)
{
	int ret;

	if (event == SFP_E_REMOVE && qsfp->sm_mod_state != SFP_MOD_EMPTY) {
		if (qsfp->sm_mod_state >= SFP_MOD_WATTACH)
			qsfp_sm_mod_remove(qsfp);
		qsfp_sm_ins_next(qsfp, SFP_MOD_EMPTY, 0);
	}

	switch (qsfp->sm_mod_state) {
	default:
		if (event == SFP_E_INSERT) {
			qsfp_sm_ins_next(qsfp, SFP_MOD_PROBE, T_SERIAL);
			qsfp->sm_retries = 10;
		}
		break;

	case SFP_MOD_PROBE:
		if (event == SFP_E_TIMEOUT) {
			ret = qsfp_sm_mod_probe(qsfp);
			if (ret == 0)
				qsfp_sm_mod_present(qsfp);
			else if (ret != -EAGAIN)
				qsfp_sm_ins_next(qsfp, SFP_MOD_ERROR, 0);
			else if (--qsfp->sm_retries)
				qsfp_sm_set_timer(qsfp, T_PROBE_RETRY);
			else
				qsfp_sm_mod_present(qsfp);
		}
		break;

	case SFP_MOD_WATTACH:
		if (event == SFP_E_ATTACH)
			qsfp_sm_mod_present(qsfp);
		break;

	case SFP_MOD_PRESENT:
		if (event == SFP_E_DETACH)
			qsfp_sm_ins_next(qsfp, SFP_MOD_WATTACH, 0);
		break;
	}
}

static int qsfp_sm_power_up(struct qsfp *qsfp)
{
	int ret;
	u8 val;

	val = SFF8X36_CTRL_93_POWER_ORIDE;
	if (qsfp->module_power_class >= 8)
		val |= SFF8636_CTRL_93_POWER_CLS8;
	if (qsfp->module_power_mW >= 1500u)
		val |= SFF8636_CTRL_93_POWER_CLS5_7;

	ret = qsfp_set_power(qsfp, val);
	if (ret < 0)
		qsfp_sm_next(qsfp, SFP_S_ERROR, 0);
	else
		qsfp_sm_next(qsfp, SFP_S_WPOWER, T_OFF_PDOWN);

	return 1;
}

static void qsfp_sm_power_down(struct qsfp *qsfp)
{
	qsfp_set_power(qsfp, SFF8X36_CTRL_93_POWER_ORIDE |
		       SFF8X36_CTRL_93_POWER_SET);
}

static int sfp_sm_tx_enable(struct qsfp *qsfp)
{
	if (qsfp->id.ext.sff8436.options[3] & SFF8X36_OPTIONS195_TX_DISABLE) {
		if (qsfp_set_tx_disable(qsfp, 0) < 0)
			qsfp_sm_next(qsfp, SFP_S_ERROR, 0);
		else
			qsfp_sm_next(qsfp, SFP_S_WTXEN, T_OFF_TXDIS);
		return 1;
	}
	return 0;
}

static void sfp_sm_init(struct qsfp *qsfp)
{
	qsfp_sm_next(qsfp, SFP_S_INIT, T_OFF_PDOWN);
}

static void qsfp_sm_main(struct qsfp *qsfp, unsigned int event)
{
	if (qsfp->sm_mod_state != SFP_MOD_PRESENT ||
	    qsfp->sm_dev_state != SFP_DEV_UP) {
		if (qsfp->sm_state != SFP_S_DOWN) {
			/* If we reported link up, and the upstream is
			 * still up, report link down.
			 */
			if (qsfp->sm_state == SFP_S_LINK_UP &&
			    qsfp->sm_dev_state == SFP_DEV_UP)
				sfp_sm_link_down(qsfp);
			/* If the module is still present, and we've
			 * powered it up, power it back down.
			 */
			if (qsfp->sm_mod_state == SFP_MOD_PRESENT &&
			    qsfp->sm_state >= SFP_S_WPOWER)
				qsfp_sm_power_down(qsfp);
			qsfp_sm_next(qsfp, SFP_S_DOWN, 0);
		}
		return;
	}

	/* The main state machine - only entered when the upstream is up
	 * and the module is present.
	 */
	switch (qsfp->sm_state) {
	case SFP_S_ERROR:
		break;

	case SFP_S_DOWN:
		if (qsfp_sm_power_up(qsfp))
			break;
		/* Fall through */
	case SFP_S_WPOWER:
		if (qsfp->sm_state != SFP_S_WPOWER || event == SFP_E_TIMEOUT)
			if (sfp_sm_tx_enable(qsfp))
				break;
		/* Fall through */
	case SFP_S_WTXEN:
		if (qsfp->sm_state != SFP_S_WTXEN || event == SFP_E_TIMEOUT)
			sfp_sm_init(qsfp);
		break;

	case SFP_S_INIT:
		if (event == SFP_E_TIMEOUT && qsfp->state & SFP_F_TX_FAULT)
			sfp_sm_fault(qsfp, true);
		else if (event == SFP_E_TIMEOUT || event == SFP_E_TX_CLEAR)
			sfp_sm_link_check_los(qsfp);
		break;

	case SFP_S_WAIT_LOS:
		if (event == SFP_E_TX_FAULT)
			sfp_sm_fault(qsfp, true);
		else if (sfp_los_event_inactive(qsfp, event))
			sfp_sm_link_up(qsfp);
		break;

	case SFP_S_LINK_UP:
		if (event == SFP_E_TX_FAULT) {
			sfp_sm_link_down(qsfp);
			sfp_sm_fault(qsfp, true);
		} else if (sfp_los_event_active(qsfp, event)) {
			sfp_sm_link_down(qsfp);
			qsfp_sm_next(qsfp, SFP_S_WAIT_LOS, 0);
		}
		break;

	case SFP_S_TX_FAULT:
		if (event == SFP_E_TIMEOUT) {
			qsfp_sm_next(qsfp, SFP_S_REINIT, T_INIT_JIFFIES);
		}
		break;

	case SFP_S_REINIT:
		if (event == SFP_E_TIMEOUT && qsfp->state & SFP_F_TX_FAULT) {
			sfp_sm_fault(qsfp, false);
		} else if (event == SFP_E_TIMEOUT || event == SFP_E_TX_CLEAR) {
			dev_info(qsfp->dev, "module transmit fault recovered\n");
			sfp_sm_link_check_los(qsfp);
		}
		break;

	case SFP_S_TX_DISABLE:
		break;
	}

	if (qsfp->current_tx_disable != qsfp->request_tx_disable &&
	    qsfp->id.ext.sff8436.options[3] & SFF8X36_OPTIONS195_TX_DISABLE) {
		qsfp->current_tx_disable = qsfp->request_tx_disable;

		if (qsfp_set_tx_disable(qsfp, qsfp->current_tx_disable) < 0)
			qsfp_sm_next(qsfp, SFP_S_ERROR, 0);
	}
}

static void qsfp_sm_event(struct qsfp *qsfp, unsigned int event)
{
	mutex_lock(&qsfp->sm_mutex);

	dev_dbg(qsfp->dev, "SM: enter %s:%s:%s event %s\n",
		mod_state_to_str(qsfp->sm_mod_state),
		dev_state_to_str(qsfp->sm_dev_state),
		sm_state_to_str(qsfp->sm_state),
		event_to_str(event));

	qsfp_sm_device(qsfp, event);
	qsfp_sm_module(qsfp, event);
	qsfp_sm_main(qsfp, event);

	dev_dbg(qsfp->dev, "SM: exit %s:%s:%s\n",
		mod_state_to_str(qsfp->sm_mod_state),
		dev_state_to_str(qsfp->sm_dev_state),
		sm_state_to_str(qsfp->sm_state));

	mutex_unlock(&qsfp->sm_mutex);
}

static void sfp_attach(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, SFP_E_ATTACH);
}

static void sfp_detach(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, SFP_E_DETACH);
}

static void sfp_start(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, SFP_E_DEV_UP);
}

static void sfp_stop(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, SFP_E_DEV_DOWN);
}

#if 0
static int sfp_module_info(struct qsfp *sfp, struct ethtool_modinfo *modinfo)
{
	/* locking... and check module is present */

	if (sfp->id.ext.sff8472_compliance &&
	    !(sfp->id.ext.diagmon & SFP_DIAGMON_ADDRMODE)) {
		modinfo->type = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
	} else {
		modinfo->type = ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = ETH_MODULE_SFF_8079_LEN;
	}
	return 0;
}

static int sfp_module_eeprom(struct qsfp *sfp, struct ethtool_eeprom *ee,
			     u8 *data)
{
	unsigned int first, last, len;
	int ret;

	if (ee->len == 0)
		return -EINVAL;

	first = ee->offset;
	last = ee->offset + ee->len;
	if (first < ETH_MODULE_SFF_8079_LEN) {
		len = min_t(unsigned int, last, ETH_MODULE_SFF_8079_LEN);
		len -= first;

		ret = qsfp_read(sfp, false, first, data, len);
		if (ret < 0)
			return ret;

		first += len;
		data += len;
	}
	if (first < ETH_MODULE_SFF_8472_LEN && last > ETH_MODULE_SFF_8079_LEN) {
		len = min_t(unsigned int, last, ETH_MODULE_SFF_8472_LEN);
		len -= first;
		first -= ETH_MODULE_SFF_8079_LEN;

		ret = qsfp_read(sfp, true, first, data, len);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static const struct sfp_socket_ops sfp_module_ops = {
	.attach = sfp_attach,
	.detach = sfp_detach,
	.start = sfp_start,
	.stop = sfp_stop,
	.module_info = sfp_module_info,
	.module_eeprom = sfp_module_eeprom,
};
#endif

static void sfp_timeout(struct work_struct *work)
{
	struct qsfp *sfp = container_of(work, struct qsfp, timeout.work);

	rtnl_lock();
	qsfp_sm_event(sfp, SFP_E_TIMEOUT);
	rtnl_unlock();
}

static void qsfp_check_modprs(struct qsfp *qsfp)
{
	unsigned int v, state = 0;

	v = gpiod_get_value_cansleep(qsfp->gpio[GPIO_MODPRS]);
	if (v)
		state |= SFP_F_PRESENT;

	if (state != qsfp->state) {
		rtnl_lock();
		qsfp_sm_event(qsfp, state & SFP_F_PRESENT ?
			     SFP_E_INSERT : SFP_E_REMOVE);
		rtnl_unlock();
		qsfp->state = state;
	}
}

static void qsfp_check_intl(struct qsfp *qsfp, bool poll)
{
	u8 irqs[19];
	int ret;

	// Read interrupt flags
	ret = qsfp_read(qsfp, SFF8X36_IRQ_FLAGS, irqs, sizeof(irqs));
	if (ret < 0)
		return;

	if (!memcmp(qsfp->module_irq_flags, irqs, sizeof(irqs)))
		return;

	dev_info(qsfp->dev, "irqs: %*ph\n", (int)sizeof(irqs), irqs);

	memcpy(qsfp->module_irq_flags, irqs, sizeof(irqs));
}

static irqreturn_t qsfp_irq_modprs(int irq, void *data)
{
	struct qsfp *qsfp = data;

	mutex_lock(&qsfp->st_mutex);
	qsfp_check_modprs(qsfp);
	mutex_unlock(&qsfp->st_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t qsfp_irq_intl(int irq, void *data)
{
	struct qsfp *qsfp = data;

	mutex_lock(&qsfp->st_mutex);
	qsfp_check_intl(qsfp, false);
	mutex_unlock(&qsfp->st_mutex);

	return IRQ_HANDLED;
}

static void qsfp_poll(struct work_struct *work)
{
	struct qsfp *qsfp = container_of(work, struct qsfp, poll.work);
	bool need_poll = false;

	mutex_lock(&qsfp->st_mutex);
	if (!qsfp->gpio_irq[GPIO_MODPRS]) {
		qsfp_check_modprs(qsfp);
		need_poll = true;
	}
	if (!qsfp->gpio_irq[GPIO_INTL] && qsfp->sm_mod_state > SFP_MOD_PROBE) {
		qsfp_check_intl(qsfp, true);
		need_poll = true;
	}
	mutex_unlock(&qsfp->st_mutex);

	if (need_poll)
		mod_delayed_work(system_wq, &qsfp->poll, poll_jiffies);
}

static struct qsfp *sfp_alloc(struct device *dev)
{
	struct qsfp *qsfp;

	qsfp = kzalloc(sizeof(*qsfp), GFP_KERNEL);
	if (!qsfp)
		return ERR_PTR(-ENOMEM);

	qsfp->dev = dev;

	mutex_init(&qsfp->sm_mutex);
	mutex_init(&qsfp->st_mutex);
	INIT_DELAYED_WORK(&qsfp->poll, qsfp_poll);
	INIT_DELAYED_WORK(&qsfp->timeout, sfp_timeout);

	return qsfp;
}

static void sfp_cleanup(void *data)
{
	struct qsfp *qsfp = data;

	cancel_delayed_work_sync(&qsfp->poll);
	cancel_delayed_work_sync(&qsfp->timeout);
	if (qsfp->i2c_mii) {
		mdiobus_unregister(qsfp->i2c_mii);
		mdiobus_free(qsfp->i2c_mii);
	}
	if (qsfp->i2c)
		i2c_put_adapter(qsfp->i2c);
	kfree(qsfp);
}

static int qsfp_probe(struct platform_device *pdev)
{
	struct i2c_adapter *i2c;
	struct qsfp *qsfp;
	int err, i;

	qsfp = sfp_alloc(&pdev->dev);
	if (IS_ERR(qsfp))
		return PTR_ERR(qsfp);

	platform_set_drvdata(pdev, qsfp);

	err = devm_add_action(qsfp->dev, sfp_cleanup, qsfp);
	if (err < 0)
		return err;

	if (pdev->dev.of_node) {
		struct device_node *node = pdev->dev.of_node;
		const struct of_device_id *id;
		struct device_node *np;

		id = of_match_node(qsfp_of_match, node);
		if (WARN_ON(!id))
			return -EINVAL;

		np = of_parse_phandle(node, "i2c-bus", 0);
		if (!np) {
			dev_err(qsfp->dev, "missing 'i2c-bus' property\n");
			return -ENODEV;
		}

		i2c = of_find_i2c_adapter_by_node(np);
		of_node_put(np);
	} else if (has_acpi_companion(&pdev->dev)) {
		struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);
		struct fwnode_handle *fw = acpi_fwnode_handle(adev);
		struct fwnode_reference_args args;
		struct acpi_handle *acpi_handle;
		int ret;

		ret = acpi_node_get_property_reference(fw, "i2c-bus", 0, &args);
		if (ret || !is_acpi_device_node(args.fwnode)) {
			dev_err(&pdev->dev, "missing 'i2c-bus' property\n");
			return -ENODEV;
		}

		acpi_handle = ACPI_HANDLE_FWNODE(args.fwnode);
		i2c = i2c_acpi_find_adapter_by_handle(acpi_handle);
	} else {
		return -EINVAL;
	}

	if (!i2c)
		return -EPROBE_DEFER;

	err = qsfp_i2c_configure(qsfp, i2c);
	if (err < 0) {
		i2c_put_adapter(i2c);
		return err;
	}

	for (i = 0; i < GPIO_MAX; i++) {
		qsfp->gpio[i] = devm_gpiod_get_optional(qsfp->dev,
				   gpio_of_names[i], gpio_flags[i]);
		if (IS_ERR(qsfp->gpio[i]))
			return PTR_ERR(qsfp->gpio[i]);
	}

	device_property_read_u32(&pdev->dev, "maximum-power-milliwatt",
				 &qsfp->max_power_mW);
	if (!qsfp->max_power_mW)
		qsfp->max_power_mW = 1500;

	dev_info(qsfp->dev, "Host maximum power %u.%uW\n",
		 qsfp->max_power_mW / 1000, (qsfp->max_power_mW / 100) % 10);

	for (i = 0; i < GPIO_MAX; i++) {
		unsigned long irq_flags;
		irq_handler_t irq_handler;

		if (gpio_flags[i] != GPIOD_IN || !qsfp->gpio[i])
			continue;

		qsfp->gpio_irq[i] = gpiod_to_irq(qsfp->gpio[i]);
		if (!qsfp->gpio_irq[i]) {
			qsfp->need_poll = true;
			continue;
		}

		irq_flags = IRQF_ONESHOT;
		if (i == GPIO_MODPRS) {
			irq_flags |= IRQF_TRIGGER_RISING |
				     IRQF_TRIGGER_FALLING;
			irq_handler = qsfp_irq_modprs;
		} else {
			irq_handler = qsfp_irq_intl;
		}

		err = devm_request_threaded_irq(qsfp->dev, qsfp->gpio_irq[i],
						NULL, irq_handler, irq_flags,
						dev_name(qsfp->dev), qsfp);
		if (err) {
			qsfp->gpio_irq[i] = 0;
			qsfp->need_poll = true;
		}
	}

	if (qsfp->need_poll)
		mod_delayed_work(system_wq, &qsfp->poll, poll_jiffies);

	// Get the initial state
	mutex_lock(&qsfp->st_mutex);
	qsfp_check_modprs(qsfp);
	mutex_unlock(&qsfp->st_mutex);

//	qsfp->sfp_bus = sfp_register_socket(qsfp->dev, qsfp, &sfp_module_ops);
//	if (!qsfp->sfp_bus)
//		return -ENOMEM;

	rtnl_lock();
	sfp_attach(qsfp);
	sfp_start(qsfp);
	rtnl_unlock();

	return 0;
}

static int qsfp_remove(struct platform_device *pdev)
{
	struct qsfp *qsfp = platform_get_drvdata(pdev);

//	sfp_unregister_socket(sfp->sfp_bus);
	rtnl_lock();
	sfp_stop(qsfp);
	sfp_detach(qsfp);
	qsfp_sm_event(qsfp, SFP_E_REMOVE);
	rtnl_unlock();

	return 0;
}

static void qsfp_shutdown(struct platform_device *pdev)
{
	struct qsfp *qsfp = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < GPIO_MAX; i++) {
		if (!qsfp->gpio_irq[i])
			continue;

		devm_free_irq(qsfp->dev, qsfp->gpio_irq[i], qsfp);
	}

	cancel_delayed_work_sync(&qsfp->poll);
	cancel_delayed_work_sync(&qsfp->timeout);
}

static struct platform_driver qsfp_driver = {
	.probe = qsfp_probe,
	.remove = qsfp_remove,
	.shutdown = qsfp_shutdown,
	.driver = {
		.name = "qsfp",
		.of_match_table = qsfp_of_match,
	},
};

static int qsfp_init(void)
{
	poll_jiffies = msecs_to_jiffies(100);

	return platform_driver_register(&qsfp_driver);
}
module_init(qsfp_init);

static void qsfp_exit(void)
{
	platform_driver_unregister(&qsfp_driver);
}
module_exit(qsfp_exit);

MODULE_ALIAS("platform:qsfp");
MODULE_AUTHOR("Russell King");
MODULE_LICENSE("GPL v2");
