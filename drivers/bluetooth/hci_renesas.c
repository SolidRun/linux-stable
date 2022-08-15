// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Renesas Bluetooth Low Energy HCI UART driver
 *
 * Copyright (C) 2022  Renesas Electroics Corp.
 *
 * Alvin Park <alvin.park.pv@renesas.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/gpio/consumer.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

enum {
	STATE_FW_STX_PENDING,
	STATE_FW_ACK_PENDING,
	STATE_FW_CRC_PENDING
};

enum {
	STATE_FW_INIT,
	STATE_FW_SIZE,
	STATE_FW_PROG,
	STATE_FW_DONE,
	STATE_FW_BOOTED
};

#define VERSION "0.1"

#define FIRMWARE_DA14531	"renesas/hci_531.bin"

#define STX		0x02
#define SOH		0x01
#define ACK		0x06
#define NACK		0x15
#define CRC_INIT	0x00

struct renesas_serdev {
	struct hci_uart hu;

	struct gpio_desc *reset;
};

struct renesas_data {
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	struct sk_buff_head rawq;
	unsigned long flags;
	unsigned int state;
	u8 id, rev, fw_crc;
};

static const struct h4_recv_pkt renesas_recv_pkts[] = {
	{ H4_RECV_ACL,       .recv = hci_recv_frame     },
	{ H4_RECV_SCO,       .recv = hci_recv_frame     },
	{ H4_RECV_EVENT,     .recv = hci_recv_frame     },
};

static int renesas_open(struct hci_uart *hu)
{
	struct renesas_data *rdata;
	int ret;

	BT_DBG("hu %p", hu);

	if (!hci_uart_has_flow_control(hu))
		return -EOPNOTSUPP;

	rdata = kzalloc(sizeof(*rdata), GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	skb_queue_head_init(&rdata->txq);
	skb_queue_head_init(&rdata->rawq);

	hu->priv = rdata;

	if (hu->serdev) {
		ret = serdev_device_open(hu->serdev);
		if (ret) {
			kfree(rdata);
			return ret;
		}
	}

	return 0;
}

static int renesas_close(struct hci_uart *hu)
{
	struct renesas_data *rdata = hu->priv;

	BT_DBG("hu %p", hu);

	if (hu->serdev)
		serdev_device_close(hu->serdev);

	skb_queue_purge(&rdata->txq);
	skb_queue_purge(&rdata->rawq);
	kfree_skb(rdata->rx_skb);
	kfree(rdata);

	hu->priv = NULL;
	return 0;
}

static int renesas_flush(struct hci_uart *hu)
{
	struct renesas_data *rdata = hu->priv;

	BT_DBG("hu %p", hu);

	skb_queue_purge(&rdata->txq);
	skb_queue_purge(&rdata->rawq);

	return 0;
}

static int renesas_send_ack(struct hci_uart *hu, unsigned char type)
{
	struct renesas_data *rdata = hu->priv;
	struct sk_buff *skb;

	skb = alloc_skb(1, GFP_ATOMIC);
	if (!skb) {
		bt_dev_err(hu->hdev, "Unable to alloc ack/nak packet");
		return -ENOMEM;
	}
	skb_put_u8(skb, type);

	skb_queue_tail(&rdata->rawq, skb);
	hci_uart_tx_wakeup(hu);

	return 0;
}

static int renesas_send_fw_size(struct hci_uart *hu, u16 length)
{
	struct renesas_data *rdata = hu->priv;
	struct sk_buff *skb;

	skb = alloc_skb(3, GFP_ATOMIC);
	if (!skb) {
		bt_dev_err(hu->hdev, "Failed to alloc mem for FW size packet");
		return -ENOMEM;
	}

	skb_put_u8(skb, SOH);
	skb_put_u8(skb, length);
	skb_put_u8(skb, length>>8);

	skb_queue_tail(&rdata->rawq, skb);
	hci_uart_tx_wakeup(hu);

	return 0;
}

static void reset_device(struct gpio_desc *gpio)
{
	if (gpiod_get_value(gpio) == 0) {
		gpiod_set_value_cansleep(gpio, 1);
		usleep_range(1000, 2000);
	}
	gpiod_set_value_cansleep(gpio, 0);
	usleep_range(5000, 10000);
}

static void renesas_reset(struct hci_uart *hu)
{
	struct serdev_device *serdev = hu->serdev;
	if (serdev) {
		struct renesas_serdev *rdatadev = serdev_device_get_drvdata(serdev);
		if (rdatadev && rdatadev->reset) {
			reset_device(rdatadev->reset);
		} else {
			bt_dev_warn(hu->hdev, "Reset pin is not available!");
		}
	} else {
		bt_dev_warn(hu->hdev, "Reset is required!");
	}
}

static int renesas_load_firmware(struct hci_dev *hdev, const char *name)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct renesas_data *rdata = hu->priv;
	const struct firmware *fw = NULL;
	const u8 *fw_ptr, *fw_max;
	u16 fw_size = 0;
	int err, i;

	err = request_firmware(&fw, name, &hdev->dev);
	if (err < 0) {
		bt_dev_err(hdev, "Failed to load firmware file %s", name);
		return err;
	}

	fw_ptr = fw->data;
	fw_max = fw->data + fw->size;
	fw_size = fw->size;

	bt_dev_info(hdev, "Loading %s", name);

	/* update crc */
	for (i = 0; i < fw->size; i++)
		rdata->fw_crc ^= fw_ptr[i];

	rdata->state = STATE_FW_INIT;
	set_bit(STATE_FW_STX_PENDING, &rdata->flags);

	/* reset */
	renesas_reset(hu);

	while (rdata->state != STATE_FW_DONE) {
		struct sk_buff *skb;

		/* Controller drives the firmware load by sending firmware
		 * request packets containing the expected fragment size.
		 */
		err = wait_on_bit_timeout(&rdata->flags, STATE_FW_STX_PENDING,
					  TASK_INTERRUPTIBLE,
					  msecs_to_jiffies(5000));
		if (err == 1) {
			bt_dev_err(hdev, "Firmware load interrupted");
			err = -EINTR;
			break;
		} else if (err) {
			bt_dev_err(hdev, "Firmware request timeout");
			err = -ETIMEDOUT;
			break;
		}

		switch(rdata->state) {
			case STATE_FW_INIT:
				bt_dev_dbg(hdev, "Firmware request, expecting %lu bytes", 
					fw->size);

				set_bit(STATE_FW_ACK_PENDING, &rdata->flags);
				err = renesas_send_fw_size(hu, fw_size);
				if (err) {
					break;
				}

				rdata->state = STATE_FW_SIZE;
				break;

			case STATE_FW_SIZE:
				err = wait_on_bit_timeout(&rdata->flags, STATE_FW_ACK_PENDING,
						TASK_INTERRUPTIBLE,
						msecs_to_jiffies(2000));
				if (err == 1) {
					bt_dev_err(hdev, "Firmware load interrupted");
					err = -EINTR;
					break;
				} else if (err) {
					bt_dev_err(hdev, "Firmware request timeout");
					err = -ETIMEDOUT;
					break;
				}

				set_bit(STATE_FW_CRC_PENDING, &rdata->flags);

				skb = alloc_skb(fw_size, GFP_KERNEL);
				if (!skb) {
					bt_dev_err(hdev, "Failed to alloc mem for FW packet");
					err = -ENOMEM;
					break;
				}
				skb_put_data(skb, fw_ptr, fw_size);
				skb_queue_tail(&rdata->rawq, skb);
				hci_uart_tx_wakeup(hu);

				rdata->state = STATE_FW_PROG;
				break;

			case STATE_FW_PROG:
				err = wait_on_bit_timeout(&rdata->flags, STATE_FW_CRC_PENDING,
						TASK_INTERRUPTIBLE,
						msecs_to_jiffies(2000));
				if (err == 1) {
					bt_dev_err(hdev, "Firmware load interrupted");
					err = -EINTR;
					break;
				} else if (err) {
					bt_dev_err(hdev, "Firmware request timeout");
					err = -ETIMEDOUT;
					break;
				}

				err = renesas_send_ack(hu, ACK);
				if (err) {
					break;
				}

				bt_dev_dbg(hdev, "Firmware has been loaded");

				set_bit(STATE_FW_STX_PENDING, &rdata->flags);
				rdata->state = STATE_FW_DONE;
				break;

			default:
				break;
		}

		if (err) {
			break;
		}
	}

	release_firmware(fw);
	return err;
}

static int renesas_setup(struct hci_uart *hu)
{
	struct renesas_data *rdata = hu->priv;
	int err;

	hci_uart_set_flow_control(hu, true);

	err = renesas_load_firmware(hu->hdev, FIRMWARE_DA14531);
	if (err)
		return err;

	/* wait for HCI application to start */
	usleep_range(8000, 10000);

	rdata->state = STATE_FW_BOOTED;

	hci_uart_set_flow_control(hu, false);

	return 0;
}

static int renesas_recv(struct hci_uart *hu, const void *data, int count)
{
	struct renesas_data *rdata = hu->priv;
	u8 *data_ptr = (u8 *)data;

	switch(rdata->state) {
		case STATE_FW_INIT:
			if (*data_ptr == STX) {
				if (!test_bit(STATE_FW_STX_PENDING, &rdata->flags)) {
					bt_dev_err(hu->hdev, "Received unexpected STX");
					return -EINVAL;
				}
				clear_bit(STATE_FW_STX_PENDING, &rdata->flags);
				wake_up_bit(&rdata->flags, STATE_FW_STX_PENDING);
			}
			break;

		case STATE_FW_SIZE:
			if (*data_ptr == ACK) {
				if (!test_bit(STATE_FW_ACK_PENDING, &rdata->flags)) {
					bt_dev_err(hu->hdev, "Received unexpected ACK");
					return -EINVAL;
				}
				clear_bit(STATE_FW_ACK_PENDING, &rdata->flags);
				wake_up_bit(&rdata->flags, STATE_FW_ACK_PENDING);
			} else {
				bt_dev_err(hu->hdev, "Received unexpected data (%x)", *data_ptr);
				return -EINVAL;
			}
			break;

		case STATE_FW_PROG:
			if (!test_bit(STATE_FW_CRC_PENDING, &rdata->flags)) {
				bt_dev_err(hu->hdev, "Received unexpected CRC");
				return -EINVAL;
			}

			if (rdata->fw_crc != *data_ptr) {
				bt_dev_err(hu->hdev, "Received CRC %02x, "
					"which does not match "
					"computed CRC %02x.\n",
					*data_ptr, rdata->fw_crc);
				return -EINVAL;
			}

			clear_bit(STATE_FW_CRC_PENDING, &rdata->flags);
			wake_up_bit(&rdata->flags, STATE_FW_CRC_PENDING);
			break;

		case STATE_FW_DONE:
			/* Do nothing */
			break;

		case STATE_FW_BOOTED:
			rdata->rx_skb = h4_recv_buf(hu->hdev, rdata->rx_skb, data, count,
								renesas_recv_pkts,
								ARRAY_SIZE(renesas_recv_pkts));
			if (IS_ERR(rdata->rx_skb)) {
				int err = PTR_ERR(rdata->rx_skb);
				bt_dev_err(hu->hdev, "Frame reassembly failed (%d)", err);
				rdata->rx_skb = NULL;
				return err;
			}
			break;

		default:
			bt_dev_err(hu->hdev, "Unknown state (%d)", rdata->state);
			break;
	}

	return count;
}

static int renesas_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct renesas_data *rdata = hu->priv;

	skb_queue_tail(&rdata->txq, skb);
	return 0;
}

static struct sk_buff *renesas_dequeue(struct hci_uart *hu)
{
	struct renesas_data *rdata = hu->priv;
	struct sk_buff *skb;

	skb = skb_dequeue(&rdata->txq);
	if (!skb) {
		/* Any raw data ? */
		skb = skb_dequeue(&rdata->rawq);
	} else {
		/* Prepend skb with frame type */
		memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);
	}

	return skb;
}

static const struct hci_uart_proto renesas_proto = {
	.id		= HCI_UART_RENESAS,
	.name		= "Renesas",
	.init_speed	= 115200,
	.open		= renesas_open,
	.close		= renesas_close,
	.flush		= renesas_flush,
	.setup		= renesas_setup,
	.recv		= renesas_recv,
	.enqueue	= renesas_enqueue,
	.dequeue	= renesas_dequeue,
};

static int renesas_serdev_probe(struct serdev_device *serdev)
{
	struct renesas_serdev *rdatadev;

	rdatadev = devm_kzalloc(&serdev->dev, sizeof(*rdatadev), GFP_KERNEL);
	if (!rdatadev)
		return -ENOMEM;

	rdatadev->hu.serdev = serdev;
	serdev_device_set_drvdata(serdev, rdatadev);

	rdatadev->reset = devm_gpiod_get(&serdev->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rdatadev->reset)) {
		int err = 0;
		err = PTR_ERR(rdatadev->reset);
		dev_warn(&serdev->dev, "could not get reset gpio: %d", err);
	}

	return hci_uart_register_device(&rdatadev->hu, &renesas_proto);
}

static void renesas_serdev_remove(struct serdev_device *serdev)
{
	struct renesas_serdev *rdatadev = serdev_device_get_drvdata(serdev);

	hci_uart_unregister_device(&rdatadev->hu);
}

#ifdef CONFIG_OF
static const struct of_device_id renesas_bluetooth_of_match[] = {
	{ .compatible = "renesas,DA14531" },
	{ },
};
MODULE_DEVICE_TABLE(of, renesas_bluetooth_of_match);
#endif

static struct serdev_device_driver renesas_serdev_driver = {
	.probe = renesas_serdev_probe,
	.remove = renesas_serdev_remove,
	.driver = {
		.name = "hci_uart_renesas",
		.of_match_table = of_match_ptr(renesas_bluetooth_of_match),
	},
};

int __init renesas_init(void)
{
	serdev_device_driver_register(&renesas_serdev_driver);

	return hci_uart_register_proto(&renesas_proto);
}

int __exit renesas_deinit(void)
{
	serdev_device_driver_unregister(&renesas_serdev_driver);

	return hci_uart_unregister_proto(&renesas_proto);
}

MODULE_AUTHOR("Alvin Park <alvin.park.pv@renesas.com>");
MODULE_DESCRIPTION("Renesas Bluetooth Serial driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(FIRMWARE_DA14531);
