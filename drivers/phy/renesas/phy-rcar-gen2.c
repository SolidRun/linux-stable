// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas R-Car Gen2 PHY driver
 *
 * Copyright (C) 2014 Renesas Solutions Corp.
 * Copyright (C) 2014 Cogent Embedded, Inc.
 * Copyright (C) 2019 Renesas Electronics Corp.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/of_device.h>

#define USBHS_LPSTS			0x02
#define USBHS_UGCTRL			0x80
#define USBHS_UGCTRL2			0x84
#define USBHS_UGSTS			0x88	/* From technical update */

/* Low Power Status register (LPSTS) */
#define USBHS_LPSTS_SUSPM		0x4000

/* USB General control register (UGCTRL) */
#define USBHS_UGCTRL_CONNECT		0x00000004
#define USBHS_UGCTRL_PLLRESET		0x00000001

/* USB General control register 2 (UGCTRL2) */
#define USBHS_UGCTRL2_USB2SEL		0x80000000
#define USBHS_UGCTRL2_USB2SEL_PCI	0x00000000
#define USBHS_UGCTRL2_USB2SEL_USB30	0x80000000
#define USBHS_UGCTRL2_USB0SEL		0x00000030
#define USBHS_UGCTRL2_USB0SEL_PCI	0x00000010
#define USBHS_UGCTRL2_USB0SEL_HS_USB	0x00000030
#define USBHS_UGCTRL2_USB0SEL_USB20	0x00000010
#define USBHS_UGCTRL2_USB0SEL_HS_USB20	0x00000020

/* USB General status register (UGSTS) */
#define USBHS_UGSTS_LOCK		0x00000100 /* From technical update */

#define PHYS_PER_CHANNEL	2

struct rcar_gen2_phy {
	struct phy *phy;
	struct rcar_gen2_channel *channel;
	int number;
	u32 select_value;
};

struct rcar_gen2_channel {
	struct device_node *of_node;
	struct rcar_gen2_phy_driver *drv;
	struct rcar_gen2_phy phys[PHYS_PER_CHANNEL];
	int selected_phy;
	u32 select_mask;
	/* external power enable pin */
	int gpio_pwr;

	/* Host/Function switching */
	struct delayed_work	work;
	int use_otg;
	int gpio_vbus;
	int gpio_id;
	int gpio_vbus_pwr;
	int irq_id;
	int irq_vbus;
	struct delayed_work     work_vbus;
	struct delayed_work     work_id;
	struct workqueue_struct *work_queue;
	struct usb_phy usbphy;
	struct usb_otg *otg;
	struct platform_device *pdev;
};

struct rcar_gen2_phy_driver {
	void __iomem *base;
	struct clk *clk;
	spinlock_t lock;
	int num_channels;
	struct rcar_gen2_channel *channels;
};

struct rcar_gen2_phy_data {
	const struct phy_ops *gen2_phy_ops;
	const u32 (*select_value)[PHYS_PER_CHANNEL];
	const u32 num_channels;
};

static void rcar_gen2_phy_switch(struct rcar_gen2_channel *channel,
	u32 select_value)
{
	struct rcar_gen2_phy_driver *drv = channel->drv;
	unsigned long flags;
	u32 ugctrl2;

	spin_lock_irqsave(&drv->lock, flags);
	ugctrl2 = readl(drv->base + USBHS_UGCTRL2);
	ugctrl2 &= ~channel->select_mask;
	ugctrl2 |= select_value;
	writel(ugctrl2, drv->base + USBHS_UGCTRL2);
	spin_unlock_irqrestore(&drv->lock, flags);
	return 0;
}

static int rcar_gen2_phy_init(struct phy *p)
{
	struct rcar_gen2_phy *phy = phy_get_drvdata(p);
	struct rcar_gen2_channel *channel = phy->channel;
	struct rcar_gen2_phy_driver *drv = channel->drv;
#ifdef CONFIG_USB_OTG
	if (!channel->use_otg) {
		/*
		 * Static Host/Function role.
		 * Try to acquire exclusive access to PHY. The first driver
		 * calling phy_init() on a given channel wins, and all attempts
		 * to use another PHY on this channel will fail until
		 * phy_exit() is called by the first driver. Achieving this
		 * with cmpxcgh() should be SMP-safe.
		 */
		if (cmpxchg(&channel->selected_phy, -1, phy->number) != -1)
			return -EBUSY;

		clk_prepare_enable(drv->clk);
		rcar_gen2_phy_switch(channel, phy->select_value);
	} else {
		/*
		 * Using Host/Function switching, so schedule work to determine
		 * which role to use.
		 */
		clk_prepare_enable(drv->clk);
		schedule_delayed_work(&channel->work_vbus, 100);
	}
#else
	if (cmpxchg(&channel->selected_phy, -1, phy->number) != -1)
		return -EBUSY;

	clk_prepare_enable(drv->clk);
	rcar_gen2_phy_switch(channel, phy->select_value);
#endif
}

static int rcar_gen2_phy_exit(struct phy *p)
{
	struct rcar_gen2_phy *phy = phy_get_drvdata(p);
	struct rcar_gen2_channel *channel = phy->channel;

	clk_disable_unprepare(channel->drv->clk);

	channel->selected_phy = -1;

	return 0;
}

static int rcar_gen2_phy_power_on(struct phy *p)
{
	struct rcar_gen2_phy *phy = phy_get_drvdata(p);
	struct rcar_gen2_phy_driver *drv = phy->channel->drv;
	void __iomem *base = drv->base;
	unsigned long flags;
	u32 value;
	int err = 0, i;

#ifdef CONFIG_USB_OTG
	/* Optional external power pin */
	if (gpio_is_valid(phy->channel->gpio_pwr))
		gpio_direction_output(phy->channel->gpio_pwr, 1);
#endif

	spin_lock_irqsave(&drv->lock, flags);

	/* Power on USBHS PHY */
	value = readl(base + USBHS_UGCTRL);
	value &= ~USBHS_UGCTRL_PLLRESET;
	writel(value, base + USBHS_UGCTRL);

	value = readw(base + USBHS_LPSTS);
	value |= USBHS_LPSTS_SUSPM;
	writew(value, base + USBHS_LPSTS);

	for (i = 0; i < 20; i++) {
		value = readl(base + USBHS_UGSTS);
		if ((value & USBHS_UGSTS_LOCK) == USBHS_UGSTS_LOCK) {
			value = readl(base + USBHS_UGCTRL);
			value |= USBHS_UGCTRL_CONNECT;
			writel(value, base + USBHS_UGCTRL);
			goto out;
		}
		udelay(1);
	}

	/* Timed out waiting for the PLL lock */
	err = -ETIMEDOUT;

out:
	spin_unlock_irqrestore(&drv->lock, flags);

	return err;
}

static int rcar_gen2_phy_power_off(struct phy *p)
{
	struct rcar_gen2_phy *phy = phy_get_drvdata(p);
	struct rcar_gen2_phy_driver *drv = phy->channel->drv;
	void __iomem *base = drv->base;
	unsigned long flags;
	u32 value;

#ifdef CONFIG_USB_OTG
	/* External power pin */
	if (gpio_is_valid(phy->channel->gpio_pwr))
		gpio_direction_output(phy->channel->gpio_pwr, 0);
#endif

	spin_lock_irqsave(&drv->lock, flags);

	/* Power off USBHS PHY */
	value = readl(base + USBHS_UGCTRL);
	value &= ~USBHS_UGCTRL_CONNECT;
	writel(value, base + USBHS_UGCTRL);

	value = readw(base + USBHS_LPSTS);
	value &= ~USBHS_LPSTS_SUSPM;
	writew(value, base + USBHS_LPSTS);

	value = readl(base + USBHS_UGCTRL);
	value |= USBHS_UGCTRL_PLLRESET;
	writel(value, base + USBHS_UGCTRL);

	spin_unlock_irqrestore(&drv->lock, flags);

	return 0;
}

static int rz_g1c_phy_power_on(struct phy *p)
{
	struct rcar_gen2_phy *phy = phy_get_drvdata(p);
	struct rcar_gen2_phy_driver *drv = phy->channel->drv;
	void __iomem *base = drv->base;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&drv->lock, flags);

	/* Power on USBHS PHY */
	value = readl(base + USBHS_UGCTRL);
	value &= ~USBHS_UGCTRL_PLLRESET;
	writel(value, base + USBHS_UGCTRL);

	/* As per the data sheet wait 340 micro sec for power stable */
	udelay(340);

	if (phy->select_value == USBHS_UGCTRL2_USB0SEL_HS_USB20) {
		value = readw(base + USBHS_LPSTS);
		value |= USBHS_LPSTS_SUSPM;
		writew(value, base + USBHS_LPSTS);
	}

	spin_unlock_irqrestore(&drv->lock, flags);

	return 0;
}

static int rz_g1c_phy_power_off(struct phy *p)
{
	struct rcar_gen2_phy *phy = phy_get_drvdata(p);
	struct rcar_gen2_phy_driver *drv = phy->channel->drv;
	void __iomem *base = drv->base;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&drv->lock, flags);
	/* Power off USBHS PHY */
	if (phy->select_value == USBHS_UGCTRL2_USB0SEL_HS_USB20) {
		value = readw(base + USBHS_LPSTS);
		value &= ~USBHS_LPSTS_SUSPM;
		writew(value, base + USBHS_LPSTS);
	}

	value = readl(base + USBHS_UGCTRL);
	value |= USBHS_UGCTRL_PLLRESET;
	writel(value, base + USBHS_UGCTRL);

	spin_unlock_irqrestore(&drv->lock, flags);

	return 0;
}

static const struct phy_ops rcar_gen2_phy_ops = {
	.init		= rcar_gen2_phy_init,
	.exit		= rcar_gen2_phy_exit,
	.power_on	= rcar_gen2_phy_power_on,
	.power_off	= rcar_gen2_phy_power_off,
	.owner		= THIS_MODULE,
};

static const struct phy_ops rz_g1c_phy_ops = {
	.init		= rcar_gen2_phy_init,
	.exit		= rcar_gen2_phy_exit,
	.power_on	= rz_g1c_phy_power_on,
	.power_off	= rz_g1c_phy_power_off,
	.owner		= THIS_MODULE,
};

static const u32 pci_select_value[][PHYS_PER_CHANNEL] = {
	[0]	= { USBHS_UGCTRL2_USB0SEL_PCI, USBHS_UGCTRL2_USB0SEL_HS_USB },
	[2]	= { USBHS_UGCTRL2_USB2SEL_PCI, USBHS_UGCTRL2_USB2SEL_USB30 },
};

#ifdef CONFIG_USB_OTG
#define VBUS_IRQ_FLAGS \
	(IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)

static void gpio_id_work(struct work_struct *work)
{
	struct rcar_gen2_channel *channel = container_of(work,
					struct rcar_gen2_channel, work_id.work);
	struct usb_phy *usbphy = &channel->usbphy;
	int status, vbus, id;

	id = !!gpio_get_value(channel->gpio_id);
	/* Switch the PHY over */
	if (!id)
		rcar_gen2_phy_switch(channel, USBHS_UGCTRL2_USB0SEL_PCI | USBHS_UGCTRL2_USB2SEL_USB30);

	/* If VBUS is powered and we are not the initial Host, turn off VBUS */
	if (gpio_is_valid(channel->gpio_vbus_pwr))
		gpio_direction_output(channel->gpio_vbus_pwr, !(id));
}

static void gpio_vbus_work(struct work_struct *work)
{
	struct rcar_gen2_channel *channel = container_of(work,
							struct rcar_gen2_channel, work_vbus.work);
	struct usb_phy *usbphy = &channel->usbphy;
	int status, vbus, id;

	vbus = !!gpio_get_value(channel->gpio_vbus);
	id = !!gpio_get_value(channel->gpio_id);

	if (gpio_is_valid(channel->gpio_vbus_pwr))
		gpio_request_one(channel->gpio_vbus_pwr, GPIOF_DIR_OUT, NULL);
	/* Switch the PHY over */
	if (id)
		rcar_gen2_phy_switch(channel, USBHS_UGCTRL2_USB0SEL_HS_USB | USBHS_UGCTRL2_USB2SEL_USB30);
	else
		rcar_gen2_phy_switch(channel, USBHS_UGCTRL2_USB0SEL_PCI | USBHS_UGCTRL2_USB2SEL_USB30);

	if (!channel->otg->gadget)
		return;

	/* Function handling: vbus=1 when initially plugged into a Host */
	if (vbus) {
		status = USB_EVENT_VBUS;
		usbphy->otg->state = OTG_STATE_B_PERIPHERAL;
		usbphy->last_event = status;
		usb_gadget_vbus_connect(usbphy->otg->gadget);

		atomic_notifier_call_chain(&usbphy->notifier,
							status, usbphy->otg->gadget);
	} else {
		usb_gadget_vbus_disconnect(usbphy->otg->gadget);
		status = USB_EVENT_NONE;
		usbphy->otg->state = OTG_STATE_B_IDLE;
		usbphy->last_event = status;

		atomic_notifier_call_chain(&usbphy->notifier,
						status, usbphy->otg->gadget);
	}
}

/* VBUS change IRQ handler */
static irqreturn_t gpio_vbus_irq(int irq, void *data)
{
	struct rcar_gen2_channel *channel = data;
	struct usb_otg *otg = channel->otg;
	int id;

	id = gpio_get_value(channel->gpio_id);
	if (irq == channel->irq_vbus) {
		if (otg->gadget) {
			queue_delayed_work(channel->work_queue,
			&channel->work_vbus, msecs_to_jiffies(100));
			}
		} else {
		queue_delayed_work(channel->work_queue,
		&channel->work_id, msecs_to_jiffies(100));
		}
	return IRQ_HANDLED;
}

static int probe_otg(struct platform_device *pdev,
	struct rcar_gen2_phy_driver *drv)
{
	struct device *dev = &pdev->dev;
	struct rcar_gen2_channel *ch = drv->channels;
	int irq;
	int ret;
	/* GPIOs for Host/Fn switching */
	ch->gpio_id = of_get_named_gpio_flags(dev->of_node,
				"renesas,id-gpio", 0, NULL);
	ch->gpio_vbus = of_get_named_gpio_flags(dev->of_node,
				"renesas,vbus-gpio", 0, NULL);
	/* Need valid ID and VBUS gpios for Host/Fn switching */
	if (gpio_is_valid(ch->gpio_id) && gpio_is_valid(ch->gpio_vbus)) {
		ch->use_otg = 1;
		/* GPIO for ID input */
		ret = devm_gpio_request_one(dev, ch->gpio_id, GPIOF_IN, "id");
		if (ret)
			return ret;
		/* GPIO for VBUS input */
		ret = devm_gpio_request_one(dev, ch->gpio_vbus, GPIOF_IN, "vbus");
		if (ret)
			return ret;

		/* Optional GPIO for VBUS power */
		ch->gpio_vbus_pwr = of_get_named_gpio_flags(dev->of_node,
						"renesas,vbus-pwr-gpio", 0, NULL);

		if (gpio_is_valid(ch->gpio_vbus_pwr)) {
			ret = devm_gpio_request_one(dev, ch->gpio_vbus_pwr,
					GPIOF_OUT_INIT_LOW, "vbus-pwr");
			if (ret)
				return ret;
		}
		return 0;
	} else if (gpio_is_valid(ch->gpio_id)) {
		dev_err(dev, "Failed to get VBUS gpio\n");
		return ch->gpio_vbus;
	} else if (gpio_is_valid(ch->gpio_vbus)) {
		dev_err(dev, "Failed to get ID gpio\n");
		return ch->gpio_id;
	}

	return -EPROBE_DEFER;
}

/* bind/unbind the peripheral controller */
static int rcar_gen2_usb_set_peripheral(struct usb_otg *otg,
					struct usb_gadget *gadget)
{
	struct rcar_gen2_channel *channel;

	channel = container_of(otg->usb_phy, struct rcar_gen2_channel, usbphy);
	if (!gadget) {
		usb_gadget_vbus_disconnect(otg->gadget);
		otg->state = OTG_STATE_UNDEFINED;
		return -EPROBE_DEFER;
	}
	otg->gadget = gadget;

	/* initialize connection state */
	gpio_vbus_irq(channel->irq_vbus, channel);

	return 0;
}
#endif

static const u32 usb20_select_value[][PHYS_PER_CHANNEL] = {
	{ USBHS_UGCTRL2_USB0SEL_USB20, USBHS_UGCTRL2_USB0SEL_HS_USB20 },
};

static const struct rcar_gen2_phy_data rcar_gen2_usb_phy_data = {
	.gen2_phy_ops = &rcar_gen2_phy_ops,
	.select_value = pci_select_value,
	.num_channels = ARRAY_SIZE(pci_select_value),
};

static const struct rcar_gen2_phy_data rz_g1c_usb_phy_data = {
	.gen2_phy_ops = &rz_g1c_phy_ops,
	.select_value = usb20_select_value,
	.num_channels = ARRAY_SIZE(usb20_select_value),
};

static const struct of_device_id rcar_gen2_phy_match_table[] = {
	{
		.compatible = "renesas,usb-phy-r8a77470",
		.data = &rz_g1c_usb_phy_data,
	},
	{
		.compatible = "renesas,usb-phy-r8a7790",
		.data = &rcar_gen2_usb_phy_data,
	},
	{
		.compatible = "renesas,usb-phy-r8a7791",
		.data = &rcar_gen2_usb_phy_data,
	},
	{
		.compatible = "renesas,usb-phy-r8a7794",
		.data = &rcar_gen2_usb_phy_data,
	},
	{
		.compatible = "renesas,rcar-gen2-usb-phy",
		.data = &rcar_gen2_usb_phy_data,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rcar_gen2_phy_match_table);

static struct phy *rcar_gen2_phy_xlate(struct device *dev,
				       struct of_phandle_args *args)
{
	struct rcar_gen2_phy_driver *drv;
	struct device_node *np = args->np;
	int i;

	drv = dev_get_drvdata(dev);
	if (!drv)
		return ERR_PTR(-EINVAL);

	for (i = 0; i < drv->num_channels; i++) {
		if (np == drv->channels[i].of_node)
			break;
	}

	if (i >= drv->num_channels || args->args[0] >= 2)
		return ERR_PTR(-ENODEV);

	return drv->channels[i].phys[args->args[0]].phy;
}

static const u32 select_mask[] = {
	[0]	= USBHS_UGCTRL2_USB0SEL,
	[2]	= USBHS_UGCTRL2_USB2SEL,
};

static int rcar_gen2_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rcar_gen2_phy_driver *drv;
	struct phy_provider *provider;
	struct device_node *np;
	void __iomem *base;
	struct clk *clk;
	const struct rcar_gen2_phy_data *data;
	int i = 0;

#ifdef CONFIG_USB_OTG
	struct usb_otg *otg;
	struct workqueue_struct *work_queue;
	int irq;
	int err;
	int retval;
#endif

	if (!dev->of_node) {
		dev_err(dev,
			"This driver is required to be instantiated from device tree\n");
		return -EINVAL;
	}

	clk = devm_clk_get(dev, "usbhs");
	if (IS_ERR(clk)) {
		dev_err(dev, "Can't get USBHS clock\n");
		return PTR_ERR(clk);
	}

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	spin_lock_init(&drv->lock);

	drv->clk = clk;
	drv->base = base;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	drv->num_channels = of_get_child_count(dev->of_node);
	drv->channels = devm_kcalloc(dev, drv->num_channels,
				     sizeof(struct rcar_gen2_channel),
				     GFP_KERNEL);
	if (!drv->channels)
		return -ENOMEM;

#ifdef CONFIG_USB_OTG
	drv->channels->pdev = pdev;

	/* USB0 optional GPIO power pin for external devices */
	drv->channels->gpio_pwr = of_get_named_gpio_flags(dev->of_node,
						"renesas,pwr-gpio", 0, NULL);
	if (drv->channels->gpio_pwr == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (gpio_is_valid(drv->channels->gpio_pwr)) {
		err = devm_gpio_request(dev, drv->channels->gpio_pwr, "pwr");
		if (err)
			return err;
	}

	/*
	* The PHY connected to channel 0 can be used to steer signals to the
	* USB Host IP that stils behind a PCI bridge (pci0), or the USB Func
	* IP (hsusb). We can dynamically switch this based on VBUS and ID
	* signals connected to gpios, to get something approaching OTG.
	*/

	otg = devm_kzalloc(dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
			return -ENOMEM;

	drv->channels->usbphy.dev    	= dev;
	drv->channels->usbphy.otg    	= otg;
	drv->channels->usbphy.type	= USB_PHY_TYPE_UNDEFINED;

	otg->usb_phy            = &drv->channels->usbphy;
	otg->state              = OTG_STATE_UNDEFINED;
	otg->set_peripheral     = rcar_gen2_usb_set_peripheral;

	drv->channels->otg      = otg;

	/* USB0 Host/Function switching info */
	err = probe_otg(pdev, drv);
	if (err)
		return err;

	if (drv->channels->use_otg) {
		if (gpio_is_valid(drv->channels->gpio_id)) {
			irq = gpio_to_irq(drv->channels->gpio_id);
			if (irq < 0) {
				dev_err(dev,
					"Unable to get irq number for GPIO ID %d, error %d\n",
					drv->channels->gpio_id, irq);
				return irq;
			}
			drv->channels->irq_id = irq;

			INIT_DELAYED_WORK(&drv->channels->work_id, gpio_id_work);
			work_queue = create_singlethread_workqueue(dev_name(&pdev->dev));
			if (!work_queue) {
				dev_err(dev, "failed to create workqueue\n");
				return -ENOMEM;
			}
			drv->channels->work_queue = work_queue;

			retval = devm_request_irq(dev, irq,
						gpio_vbus_irq,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						pdev->name, drv->channels);

			if (retval) {
				dev_err(dev, "can't request irq %i, err: %d\n",
					irq, retval);
				goto err_irq;
			}

			if (of_property_read_bool(np, "renesas,wakeup"))
				device_init_wakeup(&pdev->dev, 1);
			else
				device_init_wakeup(&pdev->dev, 0);
		}

		if (gpio_is_valid(drv->channels->gpio_vbus)) {
			irq = gpio_to_irq(drv->channels->gpio_vbus);
			if (irq < 0) {
				dev_err(dev,
					"Unable to get irq number for GPIO ID %d, error %d\n",
						drv->channels->gpio_vbus, irq);
				return irq;
			}
			drv->channels->irq_vbus = irq;

			INIT_DELAYED_WORK(&drv->channels->work_vbus, gpio_vbus_work);
			work_queue = create_singlethread_workqueue(dev_name(&pdev->dev));
			if (!work_queue) {
				dev_err(dev, "failed to create workqueue\n");
				return -ENOMEM;
			}
			drv->channels->work_queue = work_queue;

			retval = devm_request_irq(dev, irq,
							gpio_vbus_irq,
							IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING,
							pdev->name, drv->channels);

			if (retval) {
				dev_err(dev, "can't request irq %i, err: %d\n",
					irq, retval);
				goto err_irq;
			}

			if (of_property_read_bool(np, "renesas,wakeup"))
				device_init_wakeup(&pdev->dev, 1);
			else
				device_init_wakeup(&pdev->dev, 0);
		}
	}

	retval = usb_add_phy_dev(&drv->channels->usbphy);
	if (retval < 0) {
		dev_err(dev, "Failed to add USB phy\n");
		goto err_otg;
	}
	platform_set_drvdata(pdev, drv->channels);
#endif

	for_each_child_of_node(dev->of_node, np) {
		struct rcar_gen2_channel *channel = drv->channels + i;
		u32 channel_num;
		int error, n;

		channel->of_node = np;
		channel->drv = drv;
		channel->selected_phy = -1;

#ifdef CONFIG_USB_OTG
		if (i != 0)
			channel->gpio_pwr = -ENOENT;
#endif

		error = of_property_read_u32(np, "reg", &channel_num);
		if (error || channel_num >= data->num_channels) {
			dev_err(dev, "Invalid \"reg\" property\n");
			of_node_put(np);
			return error;
		}
		channel->select_mask = select_mask[channel_num];

		for (n = 0; n < PHYS_PER_CHANNEL; n++) {
			struct rcar_gen2_phy *phy = &channel->phys[n];

			phy->channel = channel;
			phy->number = n;
			phy->select_value = data->select_value[channel_num][n];

			phy->phy = devm_phy_create(dev, NULL,
						   data->gen2_phy_ops);
			if (IS_ERR(phy->phy)) {
				dev_err(dev, "Failed to create PHY\n");
				of_node_put(np);
				return PTR_ERR(phy->phy);
			}
			phy_set_drvdata(phy->phy, phy);
		}

		i++;
	}

	provider = devm_of_phy_provider_register(dev, rcar_gen2_phy_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "Failed to register PHY provider\n");
		return PTR_ERR(provider);
	}

	dev_set_drvdata(dev, drv);

#ifdef CONFIG_USB_OTG
	/*
	* If we already have something plugged into USB0, we won't get an edge
	* on VBUS, so we have to manually schedule work to look at the VBUS
	* and ID signals.
	*/
	if (drv->channels->use_otg) {
		schedule_delayed_work(&drv->channels->work_vbus, msecs_to_jiffies(100));
		schedule_delayed_work(&drv->channels->work_id, msecs_to_jiffies(100));
	}
#endif

	return 0;

#ifdef CONFIG_USB_OTG
err_otg:
	if (gpio_is_valid(drv->channels->gpio_id) || gpio_is_valid(drv->channels->gpio_vbus))
		device_init_wakeup(&pdev->dev, 0);
err_irq:
	if (gpio_is_valid(drv->channels->gpio_id) || gpio_is_valid(drv->channels->gpio_vbus)) {
		if (gpio_is_valid(drv->channels->gpio_id))
			cancel_delayed_work_sync(&drv->channels->work_id);

		if (gpio_is_valid(drv->channels->gpio_vbus))
			cancel_delayed_work_sync(&drv->channels->work_vbus);

		destroy_workqueue(drv->channels->work_queue);
	}

	return retval;
#endif

}

static struct platform_driver rcar_gen2_phy_driver = {
	.driver = {
		.name		= "phy_rcar_gen2",
		.of_match_table	= rcar_gen2_phy_match_table,
	},
	.probe	= rcar_gen2_phy_probe,
};

module_platform_driver(rcar_gen2_phy_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car Gen2 PHY");
MODULE_AUTHOR("Sergei Shtylyov <sergei.shtylyov@cogentembedded.com>");
