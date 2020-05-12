// SPDX-License-Identifier: GPL-2.0
/* MFIS ECC driver
 *
 * Copyright (C) 2019 Renesas Electronics Corporation
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/sched/signal.h>
#include <linux/pid.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/rcupdate.h>

#define DEV_NAME "mfis_ecc"
/*MFIS Error Status Register*/
#define MFIERRSTSR10	0x0814
#define MFIERRSTSR11	0x0818
/*MFIS Error Control Register*/
#define MFIERRCTLR10	0x0808
#define MFIERRCTLR11	0x080C
/*MFIS Error Target Register*/
#define MFIERRTGTR10	0x0820
#define MFIERRTGTR11	0x0824
/*MFIS Error Count Register*/
#define MFIERRCNTR11	0x0860

/*Module Stop Base Address*/
#define MSTP_BASE	0xE6150000
#define MSTP_SIZE	0x200
/*Module Stop Status Register*/
#define MSTPSR2	0x40
/*System Module Stop Control Register*/
#define SMSTPCR2	0x138

#define MFIS_CLK_BIT		(1<<13)
#define DRAM_ECC_MBIT		(1<<24)
#define DRAM_ECC_SBIT		(1<<23)
#define DRAM_ECC_MBIT_CNT	(1<<11)
#define DRAM_ECC_SBIT_CNT	(1<<11)

#define MBIT_CNT_SETTING(x)	((x)<<8)
#define SBIT_CNT_SETTING(x)	((x)<<24)

#define MBIT_CNT_MASK		(0x7)
#define SBIT_CNT_MASK		(0x1f<<16)
#define MBIT_CNT_MAX		(0x7)
#define SBIT_CNT_MAX		(0x1f)

#define SIG_ERROR		(SIGRTMIN + 2)
#define MAX_BUF			(10)

static unsigned int pid;
static unsigned int sbit_err_count = 1;
static unsigned int mbit_err_count = 1;
static unsigned int signal_id;

static int irq10;
static int irq11;
static struct dentry *dir;
static struct dentry *pid_file;
static struct dentry *sbit_file;
static struct dentry *mbit_file;
static struct dentry *sig_file;
static struct task_struct *t;
static struct siginfo info;
static void __iomem *base_addr;

static ssize_t read_pid(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;

	if (count > MAX_BUF)
		return -EINVAL;

	sprintf(mybuf, "%d\n", pid);
	ret = copy_to_user(buf, mybuf, count);
	if (ret)
		return -EINVAL;

	return count;
}

static ssize_t write_pid(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;

	if (count > MAX_BUF)
		return -EINVAL;

	ret = copy_from_user(mybuf, buf, count);
	if (ret)
		return -EINVAL;

	ret = kstrtou32((char *)mybuf, 10, &pid);
	if (ret < 0)
		return -EINVAL;

	/* send the signal */
	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = signal_id;
	/* FIXME: SI_QUEUE is normally used by sigqueue from user space,
	 * and kernel space should use SI_KERNEL. But if SI_KERNEL is used the
	 * real_time data is not delivered to the user space signal handler
	 *function.
	 */
	info.si_code = SI_QUEUE;

	rcu_read_lock();

	/*find the task_struct associated with this pid*/
	t = pid_task(find_vpid(pid), PIDTYPE_PID);
	if (t == NULL) {
		pr_info("no such pid\n");
		rcu_read_unlock();
		return -ENODEV;
	}

	rcu_read_unlock();
	return count;
}

static const struct file_operations pid_fops = {
	.read = read_pid,
	.write = write_pid,
};

static ssize_t read_sbit_err_count(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;

	if (count > MAX_BUF)
		return -EINVAL;

	sprintf(mybuf, "%d\n", sbit_err_count);
	ret = copy_to_user(buf, mybuf, count);
	if (ret)
		return -EINVAL;

	return count;
}

static ssize_t write_sbit_err_count(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;
	uint32_t mfierrcntr11;

	if (count > MAX_BUF)
		return -EINVAL;

	ret = copy_from_user(mybuf, buf, count);
	if (ret)
		return -EINVAL;

	ret = kstrtou32((char *)mybuf, 10, &sbit_err_count);
	if (ret < 0)
		return -EINVAL;

	if (sbit_err_count > SBIT_CNT_MAX) {
		pr_info("Exceed single bit error max count: %d\n",
			SBIT_CNT_MAX);
		sbit_err_count = SBIT_CNT_MAX;
	}
	mfierrcntr11 = readl(base_addr + MFIERRCNTR11);
	mfierrcntr11 &= (~SBIT_CNT_SETTING(SBIT_CNT_MAX));
	writel(mfierrcntr11 | SBIT_CNT_SETTING(sbit_err_count),
			base_addr + MFIERRCNTR11);

	return count;
}

static const struct file_operations sbit_err_fops = {
	.read = read_sbit_err_count,
	.write = write_sbit_err_count,
};

static ssize_t read_mbit_err_count(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;

	if (count > MAX_BUF)
		return -EINVAL;

	sprintf(mybuf, "%d\n", mbit_err_count);
	ret = copy_to_user(buf, mybuf, count);
	if (ret)
		return -EINVAL;

	return count;
}

static ssize_t write_mbit_err_count(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;
	uint32_t mfierrcntr11;

	if (count > MAX_BUF)
		return -EINVAL;

	ret = copy_from_user(mybuf, buf, count);
	if (ret)
		return -EINVAL;

	ret = kstrtou32((char *)mybuf, 10, &mbit_err_count);
	if (ret < 0)
		return -EINVAL;

	if (mbit_err_count > MBIT_CNT_MAX) {
		pr_info("Exceed multi-bits error max count: %d\n",
			MBIT_CNT_MAX);
		mbit_err_count = MBIT_CNT_MAX;
	}

	mfierrcntr11 = readl(base_addr + MFIERRCNTR11);
	mfierrcntr11 &= (~MBIT_CNT_SETTING(MBIT_CNT_MAX));
	writel(mfierrcntr11 | MBIT_CNT_SETTING(mbit_err_count),
			base_addr + MFIERRCNTR11);

	return count;
}

static const struct file_operations mbit_err_fops = {
	.read = read_mbit_err_count,
	.write = write_mbit_err_count,
};

static ssize_t write_signal_id(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;

	if (count > MAX_BUF)
		return -EINVAL;

	ret = copy_from_user(mybuf, buf, count);
	if (ret)
		return -EINVAL;

	ret = kstrtou32((char *)mybuf, 10, &signal_id);
	if (ret < 0)
		return -EINVAL;

	info.si_signo = signal_id;

	return count;
}

static ssize_t read_signal_id(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	char mybuf[MAX_BUF];
	int ret;

	if (count > MAX_BUF)
		return -EINVAL;

	sprintf(mybuf, "%d\n", signal_id);
	ret = copy_to_user(buf, mybuf, count);
	if (ret)
		return -EINVAL;

	return count;
}

static const struct file_operations signal_id_fops = {
	.read = read_signal_id,
	.write = write_signal_id,
};

static irqreturn_t mfis_error10_interrupt(int irq, void *pdev)
{
	uint32_t mfiserrstsr10;
	/* single bit error */
	info.si_int = 0;

	if (t != NULL)
		send_sig_info(signal_id, &info, t);

	mfiserrstsr10 = readl(base_addr + MFIERRSTSR10);
	writel(mfiserrstsr10, base_addr + MFIERRSTSR10);

	return IRQ_HANDLED;
}

static irqreturn_t mfis_error11_interrupt(int irq, void *pdev)
{
	uint32_t mfiserrstsr11;
	/* multi bits error */
	info.si_int = 1;

	if (t != NULL)
		send_sig_info(signal_id, &info, t);

	mfiserrstsr11 = readl(base_addr + MFIERRSTSR11);
	writel(mfiserrstsr11, base_addr + MFIERRSTSR11);

	return IRQ_HANDLED;
}

static int mfis_ecc_probe(struct platform_device *pdev)
{
	int err;
	struct resource *mem;
	void __iomem *mstp_base_addr;
	uint32_t mfierrctrl10;
	uint32_t mfierrctrl11;
	uint32_t mfierrtgtr10;
	uint32_t mfierrtgtr11;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base_addr = ioremap_nocache(mem->start, resource_size(mem));
	if (IS_ERR(base_addr)) {
		err = PTR_ERR(base_addr);
		return err;
	}
	platform_set_drvdata(pdev, base_addr);

	irq10 = platform_get_irq(pdev, 10);
	if (irq10 < 0) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		return irq10;
	}

	err = request_irq(irq10, mfis_error10_interrupt, IRQF_SHARED,
			DEV_NAME, pdev);
	if (err < 0) {
		dev_err(&pdev->dev, "request_irq (%d)failed, error %d\n",
			irq10, err);
		return err;
	}

	irq11 = platform_get_irq(pdev, 11);
	if (irq11 < 0) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		return irq11;
	}

	err = request_irq(irq11, mfis_error11_interrupt, IRQF_SHARED,
			DEV_NAME, pdev);
	if (err < 0) {
		dev_err(&pdev->dev, "request_irq (%d)failed, error %d\n",
			irq11, err);
		return err;
	}

	mstp_base_addr = ioremap_nocache(MSTP_BASE, MSTP_SIZE);
	if (IS_ERR(mstp_base_addr)) {
		err = PTR_ERR(mstp_base_addr);
		return err;
	}

	if (readl(mstp_base_addr + MSTPSR2) & MFIS_CLK_BIT) {
		writel(readl(mstp_base_addr + SMSTPCR2) & (~MFIS_CLK_BIT),
		mstp_base_addr + SMSTPCR2);
		if (readl(mstp_base_addr + MSTPSR2) & MFIS_CLK_BIT) {
			dev_err(&pdev->dev, "Failed to turn on clk for MFIS\n");
			return -EPERM;
		}
	}

	iounmap(mstp_base_addr);

	mfierrctrl10 = readl(base_addr + MFIERRCTLR10);
	mfierrctrl11 = readl(base_addr + MFIERRCTLR11);
	mfierrtgtr10 = readl(base_addr + MFIERRTGTR10);
	mfierrtgtr11 = readl(base_addr + MFIERRTGTR11);

	mfierrctrl10 |= DRAM_ECC_SBIT_CNT;
	mfierrctrl11 |= DRAM_ECC_MBIT_CNT;
	mfierrtgtr10 |= DRAM_ECC_SBIT_CNT;
	mfierrtgtr11 |= DRAM_ECC_MBIT_CNT;

	writel(mfierrctrl10, base_addr + MFIERRCTLR10);
	writel(mfierrctrl11, base_addr + MFIERRCTLR11);
	writel(mfierrtgtr10, base_addr + MFIERRTGTR10);
	writel(mfierrtgtr11, base_addr + MFIERRTGTR11);

	signal_id = SIG_ERROR;

	dir = debugfs_create_dir("mfis_ecc", NULL);
	if (dir == NULL) {
		dev_err(&pdev->dev, "Failed to create mfis_ecc directory\n");
		return -EPERM;
	}

	pid_file = debugfs_create_file("pid", 0600, dir, NULL,
			&pid_fops);
	sbit_file = debugfs_create_file("sbit_count", 0600, dir, NULL,
			&sbit_err_fops);
	mbit_file = debugfs_create_file("mbit_count", 0600, dir, NULL,
			&mbit_err_fops);
	sig_file = debugfs_create_file("sig_id", 0600, dir, NULL,
			&signal_id_fops);

	if (pid_file == NULL || sbit_file == NULL
				|| mbit_file == NULL
				|| sig_file == NULL) {
		dev_err(&pdev->dev, "Failed to create debug files for mfis_ecc\n");
		return -EPERM;
	}

	return 0;
}

static int mfis_ecc_remove(struct platform_device *pdev)
{
	iounmap(base_addr);
	free_irq(irq10, pdev);
	free_irq(irq11, pdev);
	debugfs_remove(pid_file);
	debugfs_remove(sbit_file);
	debugfs_remove(mbit_file);
	debugfs_remove(sig_file);

	return 0;
}

static const struct of_device_id mfis_ecc_of_table[] = {
	{ .compatible = "renesas,mfis-ecc-r8a774c0" },
	{ .compatible = "renesas,mfis-ecc-r8a774b1" },
	{ .compatible = "renesas,mfis-ecc-r8a774e1" },
	{ }
};

static struct platform_driver mfis_ecc_drv = {
	.probe = mfis_ecc_probe,
	.remove = mfis_ecc_remove,
	.driver = {
		.name  = DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mfis_ecc_of_table,
	},
};

module_platform_driver(mfis_ecc_drv);
MODULE_LICENSE("GPL v2");
