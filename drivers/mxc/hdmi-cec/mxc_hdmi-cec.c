/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_hdmi-cec.c
 *
 * @brief HDMI CEC system initialization and file operation implementation
 *
 * @ingroup HDMI
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/sizes.h>

#include <linux/console.h>
#include <linux/types.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/pinctrl/consumer.h>

#include <video/mxc_hdmi.h>

#include "mxc_hdmi-cec.h"


#define MAX_MESSAGE_LEN			17

#define MESSAGE_TYPE_RECEIVE_SUCCESS	1
#define MESSAGE_TYPE_NOACK		2
#define MESSAGE_TYPE_DISCONNECTED	3
#define MESSAGE_TYPE_CONNECTED		4
#define MESSAGE_TYPE_SEND_SUCCESS	5

#define CEC_TX_INPROGRESS		-1
#define CEC_TX_AVAIL			0

/* These flags must not collide with HDMI_IH_CEC_STAT0_xxxx */
#define	CEC_STAT0_EX_CONNECTED		0x0100
#define	CEC_STAT0_EX_DISCONNECTED	0x0200

#define	CEC_STAT0_MASK_ALL	(HDMI_IH_CEC_STAT0_WAKEUP | \
				 HDMI_IH_CEC_STAT0_ERROR_FOLL | \
				 HDMI_IH_CEC_STAT0_ARB_LOST | \
				 HDMI_IH_CEC_STAT0_ERROR_INIT | \
				 HDMI_IH_CEC_STAT0_NACK | \
				 HDMI_IH_CEC_STAT0_EOM | \
				 HDMI_IH_CEC_STAT0_DONE)

#define	CEC_STAT0_MASK_DEFAULT	(HDMI_IH_CEC_STAT0_WAKEUP | \
				 HDMI_IH_CEC_STAT0_ERROR_FOLL | \
				 HDMI_IH_CEC_STAT0_ARB_LOST)

struct hdmi_cec_priv {
	int  receive_error;
	int  send_error;
	u8 Logical_address;
	u8 cec_state;
	int tx_answer;
	u32 cec_stat0;
	u8 link_status;
	spinlock_t irq_lock;
	struct work_struct hdmi_cec_work;
	struct mutex lock;
};

struct hdmi_cec_event {
	int event_type;
	int msg_len;
	u8 msg[MAX_MESSAGE_LEN];
	struct list_head list;
};


static LIST_HEAD(head);

static int hdmi_cec_irq;
static int hdmi_cec_major;
static struct class *hdmi_cec_class;
static struct hdmi_cec_priv hdmi_cec_data;
static u8 open_count;
static u8 want_start;
static u8 link_status;
static u8 is_initialized;

static wait_queue_head_t hdmi_cec_queue;
static wait_queue_head_t tx_cec_queue;

static u32 get_hpd_stat(struct hdmi_cec_priv *hdmi_cec)
{
	u32 cec_stat0 = 0;

	if (hdmi_cec->link_status ^ link_status) {
		hdmi_cec->link_status = link_status;
		if (hdmi_cec->link_status)
			cec_stat0 = CEC_STAT0_EX_CONNECTED;
		else
			cec_stat0 = CEC_STAT0_EX_DISCONNECTED;
	}

	return cec_stat0;
}

static irqreturn_t mxc_hdmi_cec_isr(int irq, void *data)
{
	struct hdmi_cec_priv *hdmi_cec = data;
	unsigned long flags;
	u8 cec_stat;

	spin_lock_irqsave(&hdmi_cec->irq_lock, flags);

	hdmi_writeb(CEC_STAT0_MASK_ALL, HDMI_IH_MUTE_CEC_STAT0);

	cec_stat = hdmi_readb(HDMI_IH_CEC_STAT0) & CEC_STAT0_MASK_ALL;
	hdmi_writeb(cec_stat, HDMI_IH_CEC_STAT0);

	if ((cec_stat & ~CEC_STAT0_MASK_DEFAULT) == 0) {
		if (hdmi_cec->cec_state)
			hdmi_writeb(CEC_STAT0_MASK_DEFAULT, HDMI_IH_MUTE_CEC_STAT0);
		spin_unlock_irqrestore(&hdmi_cec->irq_lock, flags);
		return IRQ_NONE;
	}

	pr_debug("HDMI-CEC: interrupt received\n");

	hdmi_cec->cec_stat0 = cec_stat | get_hpd_stat(hdmi_cec);
	schedule_work(&hdmi_cec->hdmi_cec_work);

	spin_unlock_irqrestore(&hdmi_cec->irq_lock, flags);

	return IRQ_HANDLED;
}

static void mxc_hdmi_cec_handle(u32 cec_stat)
{
	u8 i = 0;
	struct hdmi_cec_event *event = NULL;

	if (!open_count)
		return;

	/* The current transmission is successful (for initiator only). */
	if (cec_stat & HDMI_IH_CEC_STAT0_DONE) {
		hdmi_cec_data.tx_answer = cec_stat;
		wake_up(&tx_cec_queue);
	}

	/*EOM is detected so that the received data is ready in the receiver data buffer*/
	if (cec_stat & HDMI_IH_CEC_STAT0_EOM) {
		event = vmalloc(sizeof(struct hdmi_cec_event));
		if (NULL == event) {
			pr_err("%s: Not enough memory!\n", __func__);
			return;
		}
		memset(event, 0, sizeof(struct hdmi_cec_event));
		event->msg_len = hdmi_readb(HDMI_CEC_RX_CNT);
		if (!event->msg_len) {
			pr_err("%s: Invalid CEC message length!\n", __func__);
			vfree(event);
			return;
		}
		event->event_type = MESSAGE_TYPE_RECEIVE_SUCCESS;
		for (i = 0; i < event->msg_len; i++)
			event->msg[i] = hdmi_readb(HDMI_CEC_RX_DATA0+i);
		hdmi_writeb(0x0, HDMI_CEC_LOCK);
		mutex_lock(&hdmi_cec_data.lock);
		list_add_tail(&event->list, &head);
		mutex_unlock(&hdmi_cec_data.lock);
		wake_up(&hdmi_cec_queue);
	}

	/* An error is detected on cec line (for initiator only). */
	if (cec_stat & HDMI_IH_CEC_STAT0_ERROR_INIT) {
		hdmi_cec_data.tx_answer = cec_stat;
		wake_up(&tx_cec_queue);
		return;
	}
	/* A frame is not acknowledged in a directly addressed message.
	 * Or a frame is negatively acknowledged in
	 * a broadcast message (for initiator only).
	 */
	if (cec_stat & HDMI_IH_CEC_STAT0_NACK) {
		hdmi_cec_data.send_error++;
		hdmi_cec_data.tx_answer = cec_stat;
		wake_up(&tx_cec_queue);
	}

	/* An error is notified by a follower.
	 * Abnormal logic data bit error (for follower).
	 */
	if (cec_stat & HDMI_IH_CEC_STAT0_ERROR_FOLL) {
		hdmi_cec_data.receive_error++;
	}

	/* HDMI cable connected */
	if (cec_stat & CEC_STAT0_EX_CONNECTED) {
		pr_info("HDMI-CEC: link connected\n");
		pr_info("HDMI link connected\n");
		event = vmalloc(sizeof(struct hdmi_cec_event));
		if (NULL == event) {
			pr_err("%s: Not enough memory\n", __func__);
			return;
		}
		memset(event, 0, sizeof(struct hdmi_cec_event));
		event->event_type = MESSAGE_TYPE_CONNECTED;
		mutex_lock(&hdmi_cec_data.lock);
		list_add_tail(&event->list, &head);
		mutex_unlock(&hdmi_cec_data.lock);
		wake_up(&hdmi_cec_queue);
	}
	/*HDMI cable disconnected*/
	if (cec_stat & CEC_STAT0_EX_DISCONNECTED) {
		pr_info("HDMI-CEC: link disconnected\n");
		event = vmalloc(sizeof(struct hdmi_cec_event));
		if (NULL == event) {
			pr_err("%s: Not enough memory!\n", __func__);
			return;
		}
		memset(event, 0, sizeof(struct hdmi_cec_event));
		event->event_type = MESSAGE_TYPE_DISCONNECTED;
		mutex_lock(&hdmi_cec_data.lock);
		list_add_tail(&event->list, &head);
		mutex_unlock(&hdmi_cec_data.lock);
		wake_up(&hdmi_cec_queue);
	}
}

static void mxc_hdmi_cec_worker(struct work_struct *work)
{
	unsigned long flags;

	mxc_hdmi_cec_handle(hdmi_cec_data.cec_stat0);

	spin_lock_irqsave(&hdmi_cec_data.irq_lock, flags);
	hdmi_cec_data.cec_stat0 = 0;
	if (hdmi_cec_data.cec_state)
		hdmi_writeb(CEC_STAT0_MASK_DEFAULT, HDMI_IH_MUTE_CEC_STAT0);
	spin_unlock_irqrestore(&hdmi_cec_data.irq_lock, flags);
}

static int hdmi_cec_open(struct inode *inode, struct file *filp)
{
	mutex_lock(&hdmi_cec_data.lock);
	if (open_count) {
		mutex_unlock(&hdmi_cec_data.lock);
		return -EBUSY;
	}
	open_count = 1;
	filp->private_data = (void *)(&hdmi_cec_data);
	hdmi_cec_data.Logical_address = 15;
	hdmi_cec_data.cec_state = false;
	mutex_unlock(&hdmi_cec_data.lock);
	return 0;
}

static ssize_t hdmi_cec_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	struct hdmi_cec_event *event = NULL;

	pr_debug("function : %s\n", __func__);

	mutex_lock(&hdmi_cec_data.lock);
	if (!hdmi_cec_data.cec_state) {
		mutex_unlock(&hdmi_cec_data.lock);
		return -EACCES;
	}

	if (list_empty(&head)) {
		if (file->f_flags & O_NONBLOCK) {
			mutex_unlock(&hdmi_cec_data.lock);
			return -EAGAIN;
		} else {
			do {
				mutex_unlock(&hdmi_cec_data.lock);
				if (wait_event_interruptible(hdmi_cec_queue, (!list_empty(&head))))
					return -ERESTARTSYS;
				mutex_lock(&hdmi_cec_data.lock);
			} while (list_empty(&head));
		}
	}

	event = list_first_entry(&head, struct hdmi_cec_event, list);
	list_del(&event->list);
	mutex_unlock(&hdmi_cec_data.lock);
	if (copy_to_user(buf, event,
			 sizeof(struct hdmi_cec_event) - sizeof(struct list_head))) {
		vfree(event);
		return -EFAULT;
	}
	vfree(event);
	return (sizeof(struct hdmi_cec_event) - sizeof(struct list_head));
}

static ssize_t hdmi_cec_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	int ret = 0;
	u8 i, msg_len, val;
	u8 msg[MAX_MESSAGE_LEN] = { 0 };

	pr_debug("function : %s\n", __func__);

	mutex_lock(&hdmi_cec_data.lock);

	if (!hdmi_cec_data.cec_state)
		ret = -EACCES;
	else if (hdmi_cec_data.tx_answer != CEC_TX_AVAIL)
		ret = -EBUSY;
	else if (count > MAX_MESSAGE_LEN)
		ret = -EINVAL;
	else if (copy_from_user(&msg, buf, count))
		ret = -EACCES;

	if (ret) {
		mutex_unlock(&hdmi_cec_data.lock);
		return ret;
	}

	hdmi_cec_data.send_error = 0;
	hdmi_cec_data.tx_answer = CEC_TX_INPROGRESS;

	msg_len = count;
	hdmi_writeb(msg_len, HDMI_CEC_TX_CNT);
	for (i = 0; i < msg_len; i++)
		hdmi_writeb(msg[i], HDMI_CEC_TX_DATA0+i);
	val = hdmi_readb(HDMI_CEC_CTRL);
	val |= 0x01;
	hdmi_writeb(val, HDMI_CEC_CTRL);

	mutex_unlock(&hdmi_cec_data.lock);

	ret = wait_event_interruptible_timeout(
		tx_cec_queue, hdmi_cec_data.tx_answer != CEC_TX_INPROGRESS, HZ);

	if (ret < 0) {
		ret = -ERESTARTSYS;
		goto tx_out;
	}

	if (hdmi_cec_data.tx_answer & HDMI_IH_CEC_STAT0_DONE)
		/* msg correctly sent */
		ret = msg_len;
	else
		ret = -EIO;

tx_out:
	hdmi_cec_data.tx_answer = CEC_TX_AVAIL;
	return ret;
}

void hdmi_cec_hpd_changed(unsigned int state)
{
	unsigned long flags;
	u32           cec_stat0;

	pr_debug("function : %s (%d)\n", __func__, state);

	link_status = state & 1;

	if (is_initialized) {
		spin_lock_irqsave(&hdmi_cec_data.irq_lock, flags);
		cec_stat0 = get_hpd_stat(&hdmi_cec_data);
		spin_unlock_irqrestore(&hdmi_cec_data.irq_lock, flags);

		if (cec_stat0)
			mxc_hdmi_cec_handle(cec_stat0);
	}
}
EXPORT_SYMBOL(hdmi_cec_hpd_changed);

static void hdmi_cec_start_device(void)
{
	u8 val;
	unsigned long flags;

	if (!is_initialized) {
		want_start = 1;
		return;
	}

	spin_lock_irqsave(&hdmi_cec_data.irq_lock, flags);

	val = hdmi_readb(HDMI_MC_CLKDIS);
	val &= ~HDMI_MC_CLKDIS_CECCLK_DISABLE;
	hdmi_writeb(val, HDMI_MC_CLKDIS);
	hdmi_writeb(0x02, HDMI_CEC_CTRL);
	/* Force read unlock */
	hdmi_writeb(0x0, HDMI_CEC_LOCK);

	val = HDMI_IH_CEC_STAT0_ERROR_INIT | HDMI_IH_CEC_STAT0_NACK |
	      HDMI_IH_CEC_STAT0_EOM | HDMI_IH_CEC_STAT0_DONE;
	hdmi_writeb(val, HDMI_CEC_POLARITY);

	val = CEC_STAT0_MASK_DEFAULT;
	hdmi_writeb(val, HDMI_CEC_MASK);
	hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);
	hdmi_cec_data.link_status = link_status;
	hdmi_cec_data.cec_state = true;

	spin_unlock_irqrestore(&hdmi_cec_data.irq_lock, flags);
}

static void hdmi_cec_stop_device(void)
{
	u8 val;
	unsigned long flags;

	if (!is_initialized) {
		want_start = 0;
		return;
	}

	spin_lock_irqsave(&hdmi_cec_data.irq_lock, flags);

	hdmi_cec_data.cec_state = false;
	hdmi_writeb(0x10, HDMI_CEC_CTRL);
	val = CEC_STAT0_MASK_ALL;
	hdmi_writeb(val, HDMI_CEC_MASK);
	hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);

	hdmi_writeb(0x0, HDMI_CEC_POLARITY);
	val = hdmi_readb(HDMI_MC_CLKDIS);
	val |= HDMI_MC_CLKDIS_CECCLK_DISABLE;
	hdmi_writeb(val, HDMI_MC_CLKDIS);

	spin_unlock_irqrestore(&hdmi_cec_data.irq_lock, flags);
}

static long hdmi_cec_ioctl(struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret = 0, status = 0;
	u8 val = 0;
	struct mxc_edid_cfg hdmi_edid_cfg;

	pr_debug("function : %s\n", __func__);

	switch (cmd) {
	case HDMICEC_IOC_SETLOGICALADDRESS:
		mutex_lock(&hdmi_cec_data.lock);
		if (!hdmi_cec_data.cec_state) {
			mutex_unlock(&hdmi_cec_data.lock);
			pr_err("Trying to set logical address while not started\n");
			return -EACCES;
		}
		hdmi_cec_data.Logical_address = (u8)arg;
		if (hdmi_cec_data.Logical_address <= 7) {
			val = 1 << hdmi_cec_data.Logical_address;
			hdmi_writeb(val, HDMI_CEC_ADDR_L);
			hdmi_writeb(0, HDMI_CEC_ADDR_H);
		} else if (hdmi_cec_data.Logical_address > 7 && hdmi_cec_data.Logical_address <= 15) {
			val = 1 << (hdmi_cec_data.Logical_address - 8);
			hdmi_writeb(val, HDMI_CEC_ADDR_H);
			hdmi_writeb(0, HDMI_CEC_ADDR_L);
		} else
			ret = -EINVAL;
		mutex_unlock(&hdmi_cec_data.lock);
		break;
	case HDMICEC_IOC_STARTDEVICE:
		hdmi_cec_start_device();
		break;
	case HDMICEC_IOC_STOPDEVICE:
		hdmi_cec_stop_device();
		break;
	case HDMICEC_IOC_GETPHYADDRESS:
		hdmi_get_edid_cfg(&hdmi_edid_cfg);
		status = copy_to_user((void __user *)arg,
					 &hdmi_edid_cfg.physical_address,
					 4*sizeof(u8));
		if (status)
			ret = -EFAULT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
    return ret;
}

static int hdmi_cec_release(struct inode *inode, struct file *filp)
{
	struct hdmi_cec_event *event, *tmp_event;

	pr_debug("function : %s\n", __func__);

	mutex_lock(&hdmi_cec_data.lock);
	if (open_count) {
		open_count = 0;
		hdmi_cec_data.cec_state = false;
		hdmi_cec_data.Logical_address = 15;

		/* Flush eventual events which have not been read by user space */
		list_for_each_entry_safe(event, tmp_event, &head, list) {
			list_del(&event->list);
			vfree(event);
		}
	}
	mutex_unlock(&hdmi_cec_data.lock);

	return 0;
}

static unsigned int hdmi_cec_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	pr_debug("function : %s\n", __func__);

	poll_wait(file, &hdmi_cec_queue, wait);

	mutex_lock(&hdmi_cec_data.lock);
	if (hdmi_cec_data.tx_answer == CEC_TX_AVAIL)
		mask =  (POLLOUT | POLLWRNORM);
	if (!list_empty(&head))
		mask |= (POLLIN | POLLRDNORM);
	mutex_unlock(&hdmi_cec_data.lock);
	return mask;
}


const struct file_operations hdmi_cec_fops = {
	.owner = THIS_MODULE,
	.read = hdmi_cec_read,
	.write = hdmi_cec_write,
	.open = hdmi_cec_open,
	.unlocked_ioctl = hdmi_cec_ioctl,
	.release = hdmi_cec_release,
	.poll = hdmi_cec_poll,
};

static int hdmi_cec_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct pinctrl *pinctrl;
	struct device *temp_class;

	hdmi_cec_irq = platform_get_irq(pdev, 0);
	if (hdmi_cec_irq < 0) {
		dev_err(&pdev->dev, "No HDMI irq line provided\n");
		err = -ENXIO;
		goto err_out;
	}

	hdmi_cec_major = register_chrdev(hdmi_cec_major, "mxc_hdmi_cec", &hdmi_cec_fops);
	if (hdmi_cec_major < 0) {
		dev_err(&pdev->dev, "Unable to get a major for HDMI CEC\n");
		err = -EBUSY;
		goto err_out;
	}

	hdmi_cec_class = class_create(THIS_MODULE, "mxc_hdmi_cec");
	if (IS_ERR(hdmi_cec_class)) {
		err = PTR_ERR(hdmi_cec_class);
		goto err_out_chrdev;
	}

	temp_class = device_create(hdmi_cec_class, NULL,
				   MKDEV(hdmi_cec_major, 0), NULL, "mxc_hdmi_cec");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "Can't get/select CEC pinctrl\n");
		err = PTR_ERR(pinctrl);
		goto err_out_class;
	}

	INIT_LIST_HEAD(&head);

	init_waitqueue_head(&hdmi_cec_queue);
	init_waitqueue_head(&tx_cec_queue);

	mutex_init(&hdmi_cec_data.lock);
	spin_lock_init(&hdmi_cec_data.irq_lock);

	hdmi_cec_data.Logical_address = 15;
	hdmi_cec_data.tx_answer = CEC_TX_AVAIL;
	INIT_WORK(&hdmi_cec_data.hdmi_cec_work, mxc_hdmi_cec_worker);

	platform_set_drvdata(pdev, &hdmi_cec_data);

	err = devm_request_irq(&pdev->dev, hdmi_cec_irq, mxc_hdmi_cec_isr,
				IRQF_SHARED, dev_name(&pdev->dev), &hdmi_cec_data);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to request irq%d: %d\n", hdmi_cec_irq, err);
		goto err_out_class;
	}

	is_initialized = 1;
	if (want_start)
	      hdmi_cec_start_device();

	pr_info("HDMI-CEC initialized\n");
	return 0;

err_out_class:
	device_destroy(hdmi_cec_class, MKDEV(hdmi_cec_major, 0));
	class_destroy(hdmi_cec_class);
err_out_chrdev:
	unregister_chrdev(hdmi_cec_major, "mxc_hdmi_cec");
err_out:
	return err;
}

static int hdmi_cec_dev_remove(struct platform_device *pdev)
{
	hdmi_cec_stop_device();

	is_initialized = 0;
	devm_free_irq(&pdev->dev, hdmi_cec_irq, &hdmi_cec_data);

	device_destroy(hdmi_cec_class, MKDEV(hdmi_cec_major, 0));
	class_destroy(hdmi_cec_class);
	unregister_chrdev(hdmi_cec_major, "mxc_hdmi_cec");
	hdmi_cec_major = 0;

	return 0;
}

static const struct of_device_id imx_hdmi_cec_match[] = {
	{ .compatible = "fsl,imx6q-hdmi-cec", },
	{ .compatible = "fsl,imx6dl-hdmi-cec", },
	{ /* sentinel */ }
};

static struct platform_driver mxc_hdmi_cec_driver = {
	.probe = hdmi_cec_dev_probe,
	.remove = hdmi_cec_dev_remove,
	.driver = {
		.name = "mxc_hdmi_cec",
		.of_match_table = imx_hdmi_cec_match,
	},
};

module_platform_driver(mxc_hdmi_cec_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux HDMI CEC driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxc_hdmi_cec");
