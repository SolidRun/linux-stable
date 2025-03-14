// SPDX-License-Identifier: GPL-2.0
/*
 * RZ System controller driver
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 */

#include <linux/dcache.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/refcount.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/sys_soc.h>

#include "rz-sysc.h"

#define field_get(_mask, _reg) (((_reg) & (_mask)) >> (ffs(_mask) - 1))

/**
 * struct rz_sysc - RZ SYSC private data structure
 * @base: SYSC base address
 * @dev: SYSC device pointer
 * @signals: SYSC signals
 * @num_signals: number of SYSC signals
 */
struct rz_sysc {
	void __iomem *base;
	struct device *dev;
	struct rz_sysc_signal *signals;
	u8 num_signals;
};

static int rz_sysc_reg_read(void *context, unsigned int off, unsigned int *val)
{
	struct rz_sysc *sysc = context;

	*val = readl(sysc->base + off);

	return 0;
}

static struct rz_sysc_signal *rz_sysc_off_to_signal(struct rz_sysc *sysc, unsigned int offset,
						    unsigned int mask)
{
	struct rz_sysc_signal *signals = sysc->signals;

	for (u32 i = 0; i < sysc->num_signals; i++) {
		if (signals[i].init_data->offset != offset)
			continue;

		/*
		 * In case mask == 0 we just return the signal data w/o checking the mask.
		 * This is useful when calling through rz_sysc_reg_write() to check
		 * if the requested setting is for a mapped signal or not.
		 */
		if (mask) {
			if (signals[i].init_data->mask == mask)
				return &signals[i];
		} else {
			return &signals[i];
		}
	}

	return NULL;
}

static int rz_sysc_reg_update_bits(void *context, unsigned int off,
				   unsigned int mask, unsigned int val)
{
	struct rz_sysc *sysc = context;
	struct rz_sysc_signal *signal;
	bool update = false;

	signal = rz_sysc_off_to_signal(sysc, off, mask);
	if (signal) {
		if (signal->init_data->refcnt_incr_val == val) {
			if (!refcount_read(&signal->refcnt)) {
				refcount_set(&signal->refcnt, 1);
				update = true;
			} else {
				refcount_inc(&signal->refcnt);
			}
		} else {
			update = refcount_dec_and_test(&signal->refcnt);
		}
	} else {
		update = true;
	}

	if (update) {
		u32 tmp;

		tmp = readl(sysc->base + off);
		tmp &= ~mask;
		tmp |= val & mask;
		writel(tmp, sysc->base + off);
	}

	return 0;
}

static int rz_sysc_reg_write(void *context, unsigned int off, unsigned int val)
{
	struct rz_sysc *sysc = context;
	struct rz_sysc_signal *signal;

	/*
	 * Force using regmap_update_bits() for signals to have reference counter
	 * per individual signal in case there are multiple signals controlled
	 * through the same register.
	 */
	signal = rz_sysc_off_to_signal(sysc, off, 0);
	if (signal) {
		dev_err(sysc->dev,
			"regmap_write() not allowed on register controlling a signal. Use regmap_update_bits()!");
		return -EOPNOTSUPP;
	}

	writel(val, sysc->base + off);

	return 0;
}

static bool rz_sysc_writeable_reg(struct device *dev, unsigned int off)
{
	struct rz_sysc *sysc = dev_get_drvdata(dev);
	struct rz_sysc_signal *signal;

	/* Any register containing a signal is writeable. */
	signal = rz_sysc_off_to_signal(sysc, off, 0);
	if (signal)
		return true;

	return false;
}

static bool rz_sysc_readable_reg(struct device *dev, unsigned int off)
{
	struct rz_sysc *sysc = dev_get_drvdata(dev);
	struct rz_sysc_signal *signal;

	/* Any register containing a signal is readable. */
	signal = rz_sysc_off_to_signal(sysc, off, 0);
	if (signal)
		return true;

	return false;
}

static int rz_sysc_signals_show(struct seq_file *s, void *what)
{
	struct rz_sysc *sysc = s->private;

	seq_printf(s, "%-20s Enable count\n", "Signal");
	seq_printf(s, "%-20s ------------\n", "--------------------");

	for (u8 i = 0; i < sysc->num_signals; i++) {
		seq_printf(s, "%-20s %d\n", sysc->signals[i].init_data->name,
			   refcount_read(&sysc->signals[i].refcnt));
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rz_sysc_signals);

static void rz_sysc_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int rz_sysc_signals_init(struct rz_sysc *sysc,
				const struct rz_sysc_signal_init_data *init_data,
				u32 num_signals)
{
	struct dentry *root;
	int ret;

	sysc->signals = devm_kcalloc(sysc->dev, num_signals, sizeof(*sysc->signals),
				     GFP_KERNEL);
	if (!sysc->signals)
		return -ENOMEM;

	for (u32 i = 0; i < num_signals; i++) {
		struct rz_sysc_signal_init_data *id;

		id = devm_kzalloc(sysc->dev, sizeof(*id), GFP_KERNEL);
		if (!id)
			return -ENOMEM;

		id->name = devm_kstrdup(sysc->dev, init_data->name, GFP_KERNEL);
		if (!id->name)
			return -ENOMEM;

		id->offset = init_data->offset;
		id->mask = init_data->mask;
		id->refcnt_incr_val = init_data->refcnt_incr_val;

		sysc->signals[i].init_data = id;
		refcount_set(&sysc->signals[i].refcnt, 0);
	}

	sysc->num_signals = num_signals;

	root = debugfs_create_dir("renesas-rz-sysc", NULL);
	ret = devm_add_action_or_reset(sysc->dev, rz_sysc_debugfs_remove, root);
	if (ret)
		return ret;
	debugfs_create_file("signals", 0444, root, sysc, &rz_sysc_signals_fops);

	return 0;
}

static int rz_sysc_soc_init(struct rz_sysc *sysc, const struct of_device_id *match)
{
	const struct rz_sysc_init_data *sysc_data = match->data;
	const struct rz_sysc_soc_id_init_data *soc_data = sysc_data->soc_id_init_data;
	struct soc_device_attribute *soc_dev_attr;
	const char *soc_id_start, *soc_id_end;
	u32 val, revision, specific_id;
	struct soc_device *soc_dev;
	char soc_id[32] = {0};
	u8 size;

	if (!soc_data || !soc_data->family || !soc_data->offset ||
	    !soc_data->revision_mask)
		return -EINVAL;

	soc_id_start = strchr(match->compatible, ',') + 1;
	soc_id_end = strchr(match->compatible, '-');
	size = soc_id_end - soc_id_start;
	if (size > 32)
		size = 32;
	strscpy(soc_id, soc_id_start, size);

	soc_dev_attr = devm_kzalloc(sysc->dev, sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENOMEM;

	soc_dev_attr->family = soc_data->family;
	soc_dev_attr->soc_id = devm_kstrdup(sysc->dev, soc_id, GFP_KERNEL);
	if (!soc_dev_attr->soc_id)
		return -ENOMEM;

	val = readl(sysc->base + soc_data->offset);
	revision = field_get(soc_data->revision_mask, val);
	specific_id = field_get(soc_data->specific_id_mask, val);
	soc_dev_attr->revision = devm_kasprintf(sysc->dev, GFP_KERNEL, "%u", revision);
	if (!soc_dev_attr->revision)
		return -ENOMEM;

	if (soc_data->id && specific_id != soc_data->id) {
		dev_warn(sysc->dev, "SoC mismatch (product = 0x%x)\n", specific_id);
		return -ENODEV;
	}

	dev_info(sysc->dev, "Detected Renesas %s %s Rev %s\n", soc_dev_attr->family,
		 soc_dev_attr->soc_id, soc_dev_attr->revision);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev))
		return PTR_ERR(soc_dev);

	return 0;
}

static struct regmap_config rz_sysc_regmap = {
	.name = "rz_sysc_regs",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
	.reg_read = rz_sysc_reg_read,
	.reg_write = rz_sysc_reg_write,
	.reg_update_bits = rz_sysc_reg_update_bits,
	.writeable_reg = rz_sysc_writeable_reg,
	.readable_reg = rz_sysc_readable_reg,
};

static const struct of_device_id rz_sysc_match[] = {
#ifdef CONFIG_SYSC_R9A08G045
	{ .compatible = "renesas,r9a08g045-sysc", .data = &rzg3s_sysc_init_data },
#endif
#ifdef CONFIG_SYS_R9A09G047
	{ .compatible = "renesas,r9a09g047-sys", .data = &rzg3e_sys_init_data },
#endif
#ifdef CONFIG_SYS_R9A09G057
	{ .compatible = "renesas,r9a09g057-sys", .data = &rzv2h_sys_init_data },
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, rz_sysc_match);

static int rz_sysc_probe(struct platform_device *pdev)
{
	const struct rz_sysc_init_data *data;
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	struct rz_sysc *sysc;
	int ret;

	match = of_match_node(rz_sysc_match, dev->of_node);
	if (!match || !match->data)
		return -ENODEV;

	sysc = devm_kzalloc(dev, sizeof(*sysc), GFP_KERNEL);
	if (!sysc)
		return -ENOMEM;

	sysc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sysc->base))
		return PTR_ERR(sysc->base);

	sysc->dev = dev;
	ret = rz_sysc_soc_init(sysc, match);
	if (ret)
		return ret;

	data = match->data;
	if (!data->max_register_offset)
		return -EINVAL;

	ret = rz_sysc_signals_init(sysc, data->signals_init_data, data->num_signals);
	if (ret)
		return ret;

	dev_set_drvdata(dev, sysc);
	rz_sysc_regmap.max_register = data->max_register_offset;
	regmap = devm_regmap_init(dev, NULL, sysc, &rz_sysc_regmap);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return of_syscon_register_regmap(dev->of_node, regmap);
}

static struct platform_driver rz_sysc_driver = {
	.driver = {
		.name = "renesas-rz-sysc",
		.of_match_table = rz_sysc_match
	},
	.probe = rz_sysc_probe
};

static int __init rz_sysc_init(void)
{
	return platform_driver_register(&rz_sysc_driver);
}
subsys_initcall(rz_sysc_init);

MODULE_DESCRIPTION("Renesas RZ System Controller Driver");
MODULE_AUTHOR("Claudiu Beznea <claudiu.beznea.uj@bp.renesas.com>");
MODULE_LICENSE("GPL");
