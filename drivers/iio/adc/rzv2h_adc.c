// SPDX-License-Identifier: GPL-2.0
/*
 * RZV2H A/D Converter driver
 *
 *  Copyright (c) 2021 Renesas Electronics Europe GmbH
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/property.h>

#define DRIVER_NAME		"rzv2h-adc"

#define RZV2H_ADCSR			0x0
#define RZV2H_ADCSR_GBADIE		BIT(6)
#define RZV2H_ADCSR_DBLE		BIT(7)
#define RZV2H_ADCSR_EXTRG		BIT(8)
#define RZV2H_ADCSR_TRGE		BIT(9)
#define RZV2H_ADCSR_ADIE		BIT(12)
#define RZV2H_ADCSR_ADCS_MASK		GENMASK(14, 13)
#define RZV2H_ADCSR_ADST		BIT(15)

#define RZV2H_ADREF			0x02

#define RZV2H_ADANSA0			0x04
#define RZV2H_ADANSA0_ANSA0_MASK	GENMASK(7, 0)

#define RZV2H_ADDR(n)			(0x20 + (n) * 0x2)
#define RZV2H_ADDR_DR_MASK		GENMASK(11, 0)

#define RZV2H_ADC_MAX_CHANNELS		8
#define RZV2H_ADC_CHN_MASK		0x7
#define RZV2H_ADC_TIMEOUT		usecs_to_jiffies(1 * 4)

struct rzv2h_adc_data {
	const struct iio_chan_spec *channels;
	u8 num_channels;
};

struct rzv2h_adc {
	void __iomem *base;
	struct clk *pclk;
	struct clk *adclk;
	struct reset_control *adrstn;
	struct completion completion;
	const struct rzv2h_adc_data *data;
	struct mutex lock;
	u16 last_val[RZV2H_ADC_MAX_CHANNELS];
};

static const char * const rzv2h_adc_channel_name[] = {
	"adc0",
	"adc1",
	"adc2",
	"adc3",
	"adc4",
	"adc5",
	"adc6",
	"adc7",
};

static void rzv2h_adc_start_stop(struct rzv2h_adc *adc, bool start)
{
	u16 reg;
	reg = readw(adc->base + RZV2H_ADCSR);
	if (start)
		reg |= RZV2H_ADCSR_ADST;
	else
		reg &= ~RZV2H_ADCSR_ADST;
	writew(reg, adc->base + RZV2H_ADCSR);
}

static int rzv2h_adc_conversion_setup(struct rzv2h_adc *adc, u8 ch)
{
	u16 reg;

	/* Select analog input channel subjected to conversion. */
	reg = readw(adc->base + RZV2H_ADANSA0);
	reg &= ~RZV2H_ADANSA0_ANSA0_MASK;
		reg |= BIT(ch);

	writew(reg, adc->base + RZV2H_ADANSA0);

	return 0;
}

static int rzv2h_adc_set_power(struct iio_dev *indio_dev, bool on)
{
	struct device *dev = indio_dev->dev.parent;

	if (on)
		return pm_runtime_resume_and_get(dev);

	return pm_runtime_put_sync(dev);
}

static int rzv2h_adc_conversion(struct iio_dev *indio_dev, struct rzv2h_adc *adc, u8 ch)
{
	int ret;
	unsigned long intst;
	u16 reg;
	ret = rzv2h_adc_set_power(indio_dev, true);
	if (ret)
		return ret;
	ret = rzv2h_adc_conversion_setup(adc, ch);
	if (ret) {
		rzv2h_adc_set_power(indio_dev, false);
		return ret;
	}

	rzv2h_adc_start_stop(adc, true);
	reg = readw(adc->base + RZV2H_ADANSA0);
	intst = reg & RZV2H_ADANSA0_ANSA0_MASK;

	udelay(100);

	for_each_set_bit(ch, &intst, RZV2H_ADC_MAX_CHANNELS)
		adc->last_val[ch] = readw(adc->base + RZV2H_ADDR(ch))
						& RZV2H_ADDR_DR_MASK;

	reinit_completion(&adc->completion);

	return rzv2h_adc_set_power(indio_dev, false);
}

static ssize_t rzv2h_adc_get_value(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				char *buf)
{
	struct rzv2h_adc *adc = iio_priv(indio_dev);
	int tmp;
	u8 ch;

	clk_prepare_enable(adc->pclk);
	clk_prepare_enable(adc->adclk);

	tmp = readw(adc->base + RZV2H_ADCSR);

	if (tmp >= 0) {
		ch = chan->channel & RZV2H_ADC_CHN_MASK;
		tmp = adc->last_val[ch];
	} else
		tmp = -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%u\n", tmp);
}

static const struct iio_chan_spec_ext_info rzv2h_adc_iio_ext_info[] = {
	{
		.name = "get_value",
		.shared = IIO_SEPARATE,
		.read = rzv2h_adc_get_value,
	},
	{},
};

static int rzv2h_adc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct rzv2h_adc *adc = iio_priv(indio_dev);
	int ret;
	u8 ch;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type != IIO_VOLTAGE)
			return -EINVAL;

		mutex_lock(&adc->lock);
		ch = chan->channel & RZV2H_ADC_CHN_MASK;
		ret = rzv2h_adc_conversion(indio_dev, adc, ch);
		if (ret) {
			mutex_unlock(&adc->lock);
			return ret;
		}

		*val = adc->last_val[ch];
		mutex_unlock(&adc->lock);

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info rzv2h_adc_iio_info = {
	.read_raw = rzv2h_adc_read_raw,
};

static int rzv2h_adc_parse_properties(struct platform_device *pdev, struct rzv2h_adc *adc)
{
	struct iio_chan_spec *chan_array;
	struct fwnode_handle *fwnode;
	struct rzv2h_adc_data *data;
	unsigned int channel;
	int num_channels;
	int ret;
	u8 i;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	num_channels = device_get_child_node_count(&pdev->dev);
	if (!num_channels) {
		dev_err(&pdev->dev, "no channel children\n");
		return -ENODEV;
	}

	if (num_channels > RZV2H_ADC_MAX_CHANNELS) {
		dev_err(&pdev->dev, "num of channel children out of range\n");
		return -EINVAL;
	}

	chan_array = devm_kcalloc(&pdev->dev, num_channels, sizeof(*chan_array),
				  GFP_KERNEL);
	if (!chan_array)
		return -ENOMEM;

	i = 0;
	device_for_each_child_node(&pdev->dev, fwnode) {
		ret = fwnode_property_read_u32(fwnode, "reg", &channel);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}

		if (channel >= RZV2H_ADC_MAX_CHANNELS) {
			fwnode_handle_put(fwnode);
			return -EINVAL;
		}

		chan_array[i].type = IIO_VOLTAGE;
		chan_array[i].indexed = 1;
		chan_array[i].channel = channel;
		chan_array[i].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		chan_array[i].datasheet_name = rzv2h_adc_channel_name[channel];
		chan_array[i].ext_info = rzv2h_adc_iio_ext_info;
		i++;
	}

	data->num_channels = num_channels;
	data->channels = chan_array;
	adc->data = data;

	return 0;
}

static void rzv2h_adc_pm_runtime_disable(void *data)
{
	struct device *dev = data;

	pm_runtime_disable(dev->parent);
}

static void rzv2h_adc_pm_runtime_set_suspended(void *data)
{
	struct device *dev = data;

	pm_runtime_set_suspended(dev->parent);
}

static void rzv2h_adc_reset_assert(void *data)
{
	reset_control_assert(data);
}

static int rzv2h_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct rzv2h_adc *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);

	ret = rzv2h_adc_parse_properties(pdev, adc);
	if (ret)
		return ret;

	mutex_init(&adc->lock);

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);

	adc->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(adc->pclk)) {
		dev_err(dev, "Failed to get pclk");
		return PTR_ERR(adc->pclk);
	}

	adc->adclk = devm_clk_get(dev, "adclk");
	if (IS_ERR(adc->adclk)) {
		dev_err(dev, "Failed to get adclk");
		return PTR_ERR(adc->adclk);
	}

	adc->adrstn = devm_reset_control_get_exclusive(dev, "adrst-n");
	if (IS_ERR(adc->adrstn)) {
		dev_err(dev, "failed to get adrstn\n");
		return PTR_ERR(adc->adrstn);
	}

	ret = reset_control_deassert(adc->adrstn);
	if (ret) {
		dev_err(&pdev->dev, "failed to deassert adrstn pin, %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(&pdev->dev,
				       rzv2h_adc_reset_assert, adc->adrstn);
	if (ret) {
		dev_err(&pdev->dev, "failed to register adrstn assert devm action, %d\n",
			ret);
		return ret;
	}

	init_completion(&adc->completion);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = DRIVER_NAME;
	indio_dev->info = &rzv2h_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adc->data->channels;
	indio_dev->num_channels = adc->data->num_channels;

	pm_runtime_set_suspended(dev);
	ret = devm_add_action_or_reset(&pdev->dev,
				       rzv2h_adc_pm_runtime_set_suspended, &indio_dev->dev);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(&pdev->dev,
				       rzv2h_adc_pm_runtime_disable, &indio_dev->dev);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id rzv2h_adc_match[] = {
	{ .compatible = "renesas,rzv2h-adc",},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzv2h_adc_match);

static int __maybe_unused rzv2h_adc_pm_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct rzv2h_adc *adc = iio_priv(indio_dev);

	clk_disable_unprepare(adc->adclk);
	clk_disable_unprepare(adc->pclk);

	return 0;
}

static int __maybe_unused rzv2h_adc_pm_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct rzv2h_adc *adc = iio_priv(indio_dev);
	int ret;

	ret = clk_prepare_enable(adc->pclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(adc->adclk);
	if (ret) {
		clk_disable_unprepare(adc->pclk);
		return ret;
	}

	return 0;
}

static const struct dev_pm_ops rzv2h_adc_pm_ops = {
	SET_RUNTIME_PM_OPS(rzv2h_adc_pm_runtime_suspend,
			   rzv2h_adc_pm_runtime_resume,
			   NULL)
};

static struct platform_driver rzv2h_adc_driver = {
	.probe		= rzv2h_adc_probe,
	.driver		= {
		.name		= DRIVER_NAME,
		.of_match_table = rzv2h_adc_match,
		.pm		= &rzv2h_adc_pm_ops,
	},
};

module_platform_driver(rzv2h_adc_driver);

MODULE_AUTHOR("Huy Nguyen <huy.nguyen.wh@renesas.com>");
MODULE_DESCRIPTION("Renesas RZV2H ADC driver");
MODULE_LICENSE("GPL v2");
