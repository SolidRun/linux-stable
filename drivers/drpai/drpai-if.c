/*
 * Driver for the Renesas RZ/V2M RZ/V2MA RZ/V2L DRP-AI unit
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/cacheflush.h>
#include <asm/current.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/drpai.h>    /* Header file for DRP-AI Driver */
#include "drpai-core.h"     /* Header file for DRP-AI Core */

// #define DRPAI_DRV_DEBUG_WAIT
#ifdef DRPAI_DRV_DEBUG_WAIT
#define DRPAI_DEBUG_WAIT(...) msleep(1100);
#else
#define DRPAI_DEBUG_WAIT(...)
#endif

#ifdef DRPAI_DRV_DEBUG
#define DRPAI_DRV_DEBUG_MODE        " (Debug Mode ON)"
#else
#define DRPAI_DRV_DEBUG_MODE        ""
#endif

#ifdef DRPAI_DRV_DEBUG_WAIT
#define DRPAI_DRV_DEBUG_WAIT_MODE   " (Debug Wait Mode ON)"
#else
#define DRPAI_DRV_DEBUG_WAIT_MODE   ""
#endif

/*Macro definitions*/
#define DRPAI_DRIVER_VERSION        "2.10 rel.1"
#define DRPAI_DEV_NUM               (1)
#define DRPAI_DRIVER_NAME           "drpai"     /* Device name */
#define DRPAI_64BYTE_ALIGN          (0x3F)      /* Check 64-byte alignment */

#define DRPAI_STATUS_IDLE_RW        (10)
#define DRPAI_STATUS_ASSIGN         (11)
#define DRPAI_STATUS_DUMP_REG       (12)
#define DRPAI_STATUS_READ_MEM       (13)
#define DRPAI_STATUS_READ_REG       (14)
#define DRPAI_STATUS_WRITE          (15)
#define DRPAI_STATUS_ASSIGN_PARAM   (16)
#define DRPAI_STATUS_WRITE_PARAM    (17)
#define DRPAI_STATUS_ASSIGN_DYNAMIC (18)
#define DRPAI_STATUS_WRITE_DYNAMIC  (19)

#define MAX_SEM_TIMEOUT             (msecs_to_jiffies(1000))
#define HEAD_SENTINEL               (UINT_MAX)

#define DRP_DESC_ENTRY_SIZE         (16)
#define DRP_DESC_TYPE_INDEX         (3)
#define DRP_DESC_TYPE_RSHIFT        (3)
#define DRP_DESC_TYPE_MASK          (0x01)
#define DRP_DESC_CF_INDEX           (3)
#define DRP_DESC_CF_RSHIFT          (4)
#define DRP_DESC_CF_MASK            (0x0F)
#define DRP_DESC_MOD_INDEX          (1)
#define DRP_DESC_MOD_RSHIFT         (6)
#define DRP_DESC_MOD_MASK           (0x03)
#define DRP_DESC_FUNC_INDEX         (1)
#define DRP_DESC_FUNC_RSHIFT        (2)
#define DRP_DESC_FUNC_MASK          (0x01)
#define DRP_DESC_CF_DRP_DMA         (0)
#define DRP_DESC_CF_AIMAC_DMA0      (8)
#define DRP_DESC_CF_AIMAC_DMA1      (9)
#define DRP_DESC_CF_AIMAC_DMA2      (12)
#define DRP_DESC_CF_AIMAC_DMA3      (13)
#define DRP_DESC_CF_AIMAC_PARAM     (5)
#define DRP_DESC_CF_CFG_LOAD        (1)
#define DRP_DESC_ADDR_INDEX         (4)

#define DRP_PARAM_raddr             (0)
#define DRP_PARAM_waddr             (4)
#define DRP_PARAM_IMG_IWIDTH        (8)
#define DRP_PARAM_IMG_IHEIGHT       (10)
#define DRP_PARAM_IMG_OWIDTH        (16)
#define DRP_PARAM_IMG_OHEIGHT       (18)
#define DRP_PARAM_CROP_POS_X        (48)
#define DRP_PARAM_CROP_POS_Y        (50)
#define DRP_LIB_NAME_CROP           (",drp_lib:crop,")
#define DRP_PARAM_ATTR_OFFSET_ADD   ("OFFSET_ADD:")
#define DRP_PARAM_ATTR_PROP_INPUT   (",prop:input,")
#define DRP_PARAM_ATTR_PROP_OUTPUT  (",prop:output,")
#define DRP_PARAM_ATTR_PARAM        ("Param:")
#define DRP_PARAM_ATTR_OFFSET       ("offset:")
#define DRP_PARAM_NAME_RADDR        ("raddr")
#define DRP_PARAM_NAME_WADDR        ("waddr")
#define DRP_PARAM_NAME_ADDR         ("_ADDR")

#define SYS_SIZE                    (1024)
#define SYS_DRP_BANK                (0x38)
#define SYS_MASK_DRP                (0x000000C0)
#define SYS_SHIFT                   (26)

#define IRQ_CHECK_ENABLE            (1)
#define IRQ_CHECK_DISABLE           (0)

#define ISP_FINISH_SUCCESS          (0)

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
#define DRPAI_SGL_DRP_DESC_SIZE     (80)
#define DRPAI_DESC_CMD_SIZE         (16)
#define DRPAI_CMA_SIZE              ((DRPAI_SGL_DRP_DESC_SIZE * DRPAI_SEQ_NUM) + DRPAI_DESC_CMD_SIZE + 64)
#endif /* CONFIG_ARCH_R9A07G054  */

/* A function called from the kernel */
static int drpai_probe(struct platform_device *pdev);
static int drpai_remove(struct platform_device *pdev);
static int drpai_open(struct inode *inode, struct file *file);
static int drpai_close(struct inode *inode, struct file *file);
static int drpai_flush(struct file *file, fl_owner_t id);
static ssize_t  drpai_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static ssize_t  drpai_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static long drpai_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static unsigned int drpai_poll( struct file* filp, poll_table* wait );
static irqreturn_t irq_drp_errint(int irq, void *dev);
static irqreturn_t irq_mac_nmlint(int irq, void *dev);
static irqreturn_t irq_mac_errint(int irq, void *dev);
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
static irqreturn_t irq_drp_nmlint(int irq, void *dev);
#endif /* CONFIG_ARCH_R9A07G054 */

/* Internal function */
static int drpai_regist_driver(void);
static int drpai_regist_device(struct platform_device *pdev);
static void drpai_unregist_driver(void);
static void drpai_unregist_device(void);
static void drpai_init_device(uint32_t ch);
static int8_t drpai_reset_device(uint32_t ch);
static ssize_t drpai_write_dynamic(struct file *filp, uint8_t *pvirt);
static ssize_t drpai_write_dynamic_desc(struct file *filp, uint8_t *pvirt);
static ssize_t drpai_write_dynamic_param(struct file *filp, uint8_t *pvirt);
static void drpai_rewrite_address(uint8_t *entry, uint32_t addr);
static struct param_info_list* param_info_list_push(struct param_info_list **pptop, struct param_info_list *pcur, uint32_t offset);
static void param_info_list_clear(struct param_info_list *ptop);
static long drpai_ioctl_assign(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_start(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_get_status(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_reset(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_reg_dump(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_assign_param(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_prepost_crop(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_prepost_inaddr(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_assign_dynamic(struct file *filp, unsigned int cmd, unsigned long arg);
static long drpai_ioctl_get_drpai_area(struct file *filp, unsigned int cmd, unsigned long arg);
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
static long drpai_ioctl_set_seq(struct file *filp, unsigned int cmd, unsigned long arg);
#endif /* CONFIG_ARCH_R9A07G054 */
static char *get_param_attr(char *line, char *attr, unsigned long *rvalue, char **str);
static int8_t drp_param_change16(uint64_t base, uint64_t offset, uint16_t value);
static int8_t drp_param_change32(uint64_t base, uint64_t offset, uint32_t value);
static int8_t drpai_flush_dcache_input_area(uint64_t addr, uint64_t size);
static int drpai_drp_cpg_init(void);
static int8_t drpai_get_sys_bank(uint64_t *bank);
static int drpai_open_process(void);
static int drpai_close_process(void);

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
/* DRP init function for V2L simple ISP */
static int drpai_drp_config_init(void);
static void drpai_drp_config_uninit(void);

/* Function called from the kernel for V2L simple ISP */
int drpai_open_k(void);
int drpai_close_k(void);
int drpai_start_k(drpai_data_t *arg, void (*isp_finish)(int result));
#endif /* CONFIG_ARCH_R9A07G054 */

/* Linux device driver initialization */
static const unsigned int MINOR_BASE = 0;
static const unsigned int MINOR_NUM  = DRPAI_DEV_NUM;       /* Minor number */
static unsigned int drpai_major;                    /* Major number (decided dinamically) */
static struct cdev drpai_cdev;                      /* Character device object */
static struct class *drpai_class = NULL;            /* class object */
struct device *drpai_device_array[DRPAI_DEV_NUM];

/* Type definitions */
struct drpai_priv 
{
    struct platform_device *pdev;
    const char *dev_name;
    uint8_t dev_tag;
    drpai_status_t drpai_status;
    spinlock_t lock;
    struct semaphore sem;
    refcount_t count;
    void __iomem *drp_base;
    void __iomem *aimac_base;
    uint64_t bank;
    struct reset_control *rstc;
    uint32_t irq_flag;

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
/* V2L ISP */
    struct clk *clk_int;
    struct clk *clk_aclk_drp;
    struct clk *clk_mclk;
    struct clk *clk_dclkin;
    struct clk *clk_aclk;
    void (*isp_finish_loc)(int);
#endif /* CONFIG_ARCH_R9A07G054 */
};

static DECLARE_WAIT_QUEUE_HEAD(drpai_waitq);

static struct drpai_priv *drpai_priv;

struct drpai_rw_status 
{
    uint32_t rw_status;
    uint32_t read_count;
    uint32_t write_count;
    uint32_t drp_reg_offset_count;
    uint32_t aimac_reg_offset_count;
    drpai_data_t drpai_data;
    drpai_data_dynamic_t drpai_data_dynamic;
    struct list_head list;
    drpai_assign_param_t assign_param;
    char *param_info;
    atomic_t inout_flag;
};

struct param_info_list
{
    uint32_t offset;
    struct param_info_list *next;
};

static DEFINE_SEMAPHORE(rw_sem);
static struct drpai_rw_status *drpai_rw_sentinel;

/* Virtual base address of register */
static void __iomem *drp_base_addr[DRP_CH_NUM];
static void __iomem *aimac_base_address[AIMAC_CH_NUM];
static resource_size_t drp_size;
static resource_size_t aimac_size;
static resource_size_t drpai_region_base_addr;
static resource_size_t drpai_region_size;
static resource_size_t drpai_linux_mem_start;
static resource_size_t drpai_linux_mem_size;
static resource_size_t sysctrl_region_base_addr;

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
/* ISP */
static char* p_dmabuf_vaddr;
static dma_addr_t p_dmabuf_phyaddr;
static unsigned char drp_single_desc_bin[] =
{
  0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x00, 0x01, 0x00, 0x91, 0x81, 0x50, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x00, 0x01, 0x00, 0x91, 0x81, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static drpai_seq_t seq;
drpai_odif_intcnto_t odif_intcnto;
static uint32_t exe_mode;
/* ISP */
#endif /* CONFIG_ARCH_R9A07G054 */

/* handler table */
static struct file_operations s_mydevice_fops = 
{
    .open           = drpai_open,
    .release        = drpai_close,
    .write          = drpai_write,
    .read           = drpai_read,
    .unlocked_ioctl = drpai_ioctl,
    .compat_ioctl   = drpai_ioctl, /* for 32-bit App */
    .poll           = drpai_poll,
    .flush          = drpai_flush,
};

static const struct of_device_id drpai_match[] = 
{
    { .compatible = "renesas,rzv2ma-drpai",},
    { .compatible = "renesas,rzv2m-drpai", },
    { .compatible = "renesas,rzv2l-drpai", },
    { /* sentinel */ }
};

static struct platform_driver drpai_platform_driver = 
{
    .driver = {
        .name   = "drpai-rz",
        .of_match_table = drpai_match,
    },
    .probe      = drpai_probe,
    .remove     = drpai_remove,
};

static int drpai_probe(struct platform_device *pdev)
{
    DRPAI_DEBUG_PRINT("start.\n");

    drpai_regist_driver();
    drpai_regist_device(pdev);

    DRPAI_DEBUG_PRINT("end.\n");

    return 0;
}

static int drpai_remove(struct platform_device *pdev)
{
    DRPAI_DEBUG_PRINT("start.\n");

    drpai_unregist_driver();
    drpai_unregist_device();

    DRPAI_DEBUG_PRINT("end.\n");

    return 0;
}

static int drpai_open(struct inode *inode, struct file *file)
{
    int result = 0;
    struct drpai_priv *priv = drpai_priv;
    struct drpai_rw_status *drpai_rw_status = 0;

    DRPAI_DEBUG_PRINT("start.\n");
    DRPAI_DEBUG_PRINT("major %d minor %d\n", imajor(inode), iminor(inode));

    /* Allocate drpai_rw_status to each file descriptor */
    drpai_rw_status = kzalloc(sizeof(struct drpai_rw_status), GFP_KERNEL);
    if (!drpai_rw_status) 
    {
        result = -ENOMEM;
        goto end;
    }
    /* Initialization flag */
    drpai_rw_status->rw_status  = DRPAI_STATUS_IDLE_RW;
    drpai_rw_status->param_info = NULL;
    INIT_LIST_HEAD(&drpai_rw_status->list);
    atomic_set(&drpai_rw_status->inout_flag, 0);
    DRPAI_DEBUG_PRINT("Generated list %px rw_status %d prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->rw_status, drpai_rw_status->list.prev, drpai_rw_status->list.next);
    file->private_data = drpai_rw_status;
    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT)))
    {
        result = -ETIMEDOUT;
        goto end;
    }

    if(1 == refcount_read(&priv->count))
    {
        result = drpai_open_process();
        if(R_DRPAI_SUCCESS != result)
        {
            goto end;
        }
    }
    /* Increment reference count */
    refcount_inc(&priv->count);

    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);

    result = 0;

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if((-ENOMEM != result) || (-ETIMEDOUT != result))
    {
        up(&priv->sem);
    }
    if((0 != drpai_rw_status) && (0 != result))
    {
        kfree(file->private_data);
        file->private_data = NULL;
    }

    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static int drpai_close(struct inode *inode, struct file *file)
{
    int result = 0;
    struct drpai_priv *priv = drpai_priv;
    struct drpai_rw_status *drpai_rw_status = file->private_data;

    DRPAI_DEBUG_PRINT("start.\n");
    DRPAI_DEBUG_PRINT("major %d minor %d\n", imajor(inode), iminor(inode));

    if(unlikely(down_interruptible(&rw_sem))) 
    {
        /* Note: this errno won't be returned to user*/
        result = -ERESTART;
        DRPAI_DEBUG_PRINT("R/W semaphore obtained failed\n");
        goto end;
    }

    DRPAI_DEBUG_PRINT("HEAD  list %px rw_status %d prev %px next %px\n", &drpai_rw_sentinel->list, drpai_rw_sentinel->rw_status, drpai_rw_sentinel->list.prev, drpai_rw_sentinel->list.next);
    if(!list_empty(&drpai_rw_sentinel->list))
    {
        if((DRPAI_STATUS_ASSIGN         == drpai_rw_status->rw_status) ||
           (DRPAI_STATUS_ASSIGN_DYNAMIC == drpai_rw_status->rw_status) ||
           (DRPAI_STATUS_READ_MEM       == drpai_rw_status->rw_status) ||
           (DRPAI_STATUS_WRITE          == drpai_rw_status->rw_status) ||
           (DRPAI_STATUS_WRITE_DYNAMIC  == drpai_rw_status->rw_status))
           {
                DRPAI_DEBUG_PRINT("Deleted list %px rw_status %d prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->rw_status, drpai_rw_status->list.prev, drpai_rw_status->list.next);
                list_del(&drpai_rw_status->list);
           }
    }
    up(&rw_sem);

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
    {
        /* Note: this errno won't be returned to user*/
        result = -ETIMEDOUT;
        DRPAI_DEBUG_PRINT("API semaphore obtained failed\n");
        goto end;
    }

    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    if(2 == refcount_read(&priv->count))
    {
        result = drpai_close_process();
        if(R_DRPAI_SUCCESS != result)
        {
            DRPAI_DEBUG_PRINT("Reset failed\n");
        }
    }
    /* Decrement referenece count */
    refcount_dec(&priv->count);

    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if((-ERESTART != result) || (-ETIMEDOUT != result))
    {
        up(&priv->sem);
    }
    /* Free memory */
    if(NULL != drpai_rw_status->param_info)
    {
        DRPAI_DEBUG_PRINT("vfree is called\n");
        vfree(drpai_rw_status->param_info);
        drpai_rw_status->param_info = NULL;
    }
    if(file->private_data) 
    {
        DRPAI_DEBUG_PRINT("kfree is called\n");
        kfree(file->private_data);
        file->private_data = NULL;
    }
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static int drpai_flush(struct file *file, fl_owner_t id)
{
    DRPAI_DEBUG_PRINT("start.\n");

    DRPAI_DEBUG_PRINT("end.\n");
    return 0;
}

static ssize_t  drpai_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t result = 0;
    volatile void *p_drpai_cma = 0;
    struct drpai_priv *priv = drpai_priv;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    uint64_t addr;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_trylock(&rw_sem)))
    {
        result = -ERESTART;
        goto end;
    }

    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    /* Check status */
    if (!((DRPAI_STATUS_ASSIGN         == drpai_rw_status->rw_status) ||
          (DRPAI_STATUS_ASSIGN_DYNAMIC == drpai_rw_status->rw_status) ||
          (DRPAI_STATUS_WRITE          == drpai_rw_status->rw_status) ||
          (DRPAI_STATUS_WRITE_DYNAMIC  == drpai_rw_status->rw_status) ||
          (DRPAI_STATUS_ASSIGN_PARAM   == drpai_rw_status->rw_status) ||
          (DRPAI_STATUS_WRITE_PARAM    == drpai_rw_status->rw_status)))
    {
        result = -EACCES;
        goto end;
    }

    /* Check Argument */
    if (NULL == buf)
    {
        result = -EFAULT;
        goto end;
    }
    if (0 == count)
    {
        result = -EINVAL;
        goto end;
    }

    switch(drpai_rw_status->rw_status)
    {
        case DRPAI_STATUS_ASSIGN:
            /* DRPAI_STATUS_ASSIGN -> DRPAI_STATUS_WRITE */
            drpai_rw_status->rw_status = DRPAI_STATUS_WRITE;
            break;
        case DRPAI_STATUS_ASSIGN_DYNAMIC:
            /* DRPAI_STATUS_ASSIGN_DYNAMIC -> DRPAI_STATUS_WRITE_DYNAMIC */
            drpai_rw_status->rw_status = DRPAI_STATUS_WRITE_DYNAMIC;
            break;
        case DRPAI_STATUS_ASSIGN_PARAM:
            /* DRPAI_STATUS_ASSIGN_PARAM -> DRPAI_STATUS_WRITE_PARAM */
            drpai_rw_status->rw_status = DRPAI_STATUS_WRITE_PARAM;
            break;
        default:
            ; /* Do nothing */
            break;
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);

    switch(drpai_rw_status->rw_status)
    {
        case DRPAI_STATUS_WRITE:
            /* Expand to DRP for CMA */
            addr = priv->bank | (uint64_t)drpai_rw_status->drpai_data.address;
            p_drpai_cma = phys_to_virt(addr + (uint64_t)drpai_rw_status->write_count);
            if (p_drpai_cma == 0)
            {
                result = -EFAULT;
                goto end;
            }
            if (drpai_rw_status->drpai_data.size < (drpai_rw_status->write_count + count))
            {
                count = drpai_rw_status->drpai_data.size - drpai_rw_status->write_count;
            }
            if (copy_from_user((void *)p_drpai_cma, buf, count))
            {
                result = -EFAULT;
                goto end;
            }
            drpai_rw_status->write_count = drpai_rw_status->write_count + count;

            /* DRPAI_STATUS_WRITE -> DRPAI_STATUS_IDLE_RW */
            if (drpai_rw_status->drpai_data.size <= drpai_rw_status->write_count)
            {
                p_drpai_cma = phys_to_virt(addr);
                if (p_drpai_cma == 0)
                {
                    result = -EFAULT;
                    goto end;
                }
                __flush_dcache_area((void *)p_drpai_cma, drpai_rw_status->drpai_data.size);
                drpai_rw_status->rw_status = DRPAI_STATUS_IDLE_RW;
                DRPAI_DEBUG_PRINT("Deleted list %px rw_status %d prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->rw_status, drpai_rw_status->list.prev, drpai_rw_status->list.next);
                list_del(&drpai_rw_status->list);
                drpai_rw_status->drpai_data.address = 0x0;
                drpai_rw_status->drpai_data.size    = 0x0;
            }
            result = count;
            break;
        case DRPAI_STATUS_WRITE_DYNAMIC:
            /* Expand to DRP for CMA */
            addr = priv->bank | ((uint64_t)drpai_rw_status->drpai_data_dynamic.start_address + (uint64_t)drpai_rw_status->drpai_data_dynamic.offset);
            p_drpai_cma = phys_to_virt(addr + (uint64_t)drpai_rw_status->write_count);
            if (p_drpai_cma == 0)
            {
                result = -EFAULT;
                goto end;
            }
            if (drpai_rw_status->drpai_data_dynamic.size < (drpai_rw_status->write_count + count))
            {
                count = drpai_rw_status->drpai_data_dynamic.size - drpai_rw_status->write_count;
            }

            if (copy_from_user((void *)p_drpai_cma, buf, count))
            {
                result = -EFAULT;
                goto end;
            }
            drpai_rw_status->write_count = drpai_rw_status->write_count + count;

            /* DRPAI_STATUS_WRITE_DYNAMIC -> DRPAI_STATUS_IDLE_RW */
            if (drpai_rw_status->drpai_data_dynamic.size <= drpai_rw_status->write_count)
            {
                p_drpai_cma = phys_to_virt(addr);
                if (p_drpai_cma == 0)
                {
                    result = -EFAULT;
                    goto end;
                }

                result = drpai_write_dynamic(filp, (uint8_t *)p_drpai_cma);
                if (result < 0)
                {
                    goto end;
                }

                __flush_dcache_area((void *)p_drpai_cma, drpai_rw_status->drpai_data.size);
                drpai_rw_status->rw_status = DRPAI_STATUS_IDLE_RW;
                DRPAI_DEBUG_PRINT("Deleted list %px rw_status %d prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->rw_status, drpai_rw_status->list.prev, drpai_rw_status->list.next);
                list_del(&drpai_rw_status->list);
                drpai_rw_status->drpai_data_dynamic.start_address = 0x0;
                drpai_rw_status->drpai_data_dynamic.offset        = 0x0;
                drpai_rw_status->drpai_data_dynamic.size          = 0x0;
                drpai_rw_status->drpai_data_dynamic.file_type     = 0x0;
            }
            result = count;
            break;
        case DRPAI_STATUS_WRITE_PARAM:
            if (drpai_rw_status->assign_param.info_size < (drpai_rw_status->write_count + count))
            {
                count = drpai_rw_status->assign_param.info_size - drpai_rw_status->write_count;
            }
            /* Copy arguments from user space to kernel space */
            if (copy_from_user(&drpai_rw_status->param_info[drpai_rw_status->write_count], buf, count))
            {
                result = -EFAULT;
                goto end;
            }
            drpai_rw_status->write_count = drpai_rw_status->write_count + count;
            /* DRPAI_STATUS_WRITE_PARAM -> DRPAI_STATUS_IDLE_RW */
            if (drpai_rw_status->assign_param.info_size <= drpai_rw_status->write_count)
            {
                DRPAI_DEBUG_PRINT("Status is changed \n");
                drpai_rw_status->rw_status = DRPAI_STATUS_IDLE_RW;
            }
            result = count;
            break;
        default:
            ; /* Do nothing */
            break;
    }

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ERESTART != result)
    {
        up(&rw_sem);
    }
    DRPAI_DEBUG_PRINT("status_rw3:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static ssize_t drpai_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t result = 0;
    uint8_t *kbuf = NULL;
    volatile void *p_drpai_cma = 0;
    uint32_t reg_val;
    uint32_t i;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    struct drpai_priv *priv = drpai_priv;
    uint64_t addr;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_trylock(&rw_sem)))
    {
        result = -ERESTART;
        goto end;
    }

    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    /* Check status */
    if (!((DRPAI_STATUS_ASSIGN  == drpai_rw_status->rw_status) ||
        (DRPAI_STATUS_DUMP_REG  == drpai_rw_status->rw_status) ||
        (DRPAI_STATUS_READ_MEM  == drpai_rw_status->rw_status) ||
        (DRPAI_STATUS_READ_REG  == drpai_rw_status->rw_status)))
    {
        result = -EACCES;
        goto end;
    }

    /* Check Argument */
    if (NULL == buf)
    {
        result = -EFAULT;
        goto end;
    }
    if (0 == count)
    {
        result = -EINVAL;
        goto end;
    }

    switch(drpai_rw_status->rw_status)
    {
        case DRPAI_STATUS_ASSIGN:
            /* DRPAI_STATUS_ASSIGN -> DRPAI_STATUS_READ_MEM */
            drpai_rw_status->rw_status = DRPAI_STATUS_READ_MEM;
            break;
        case DRPAI_STATUS_DUMP_REG:
            /* DRPAI_STATUS_DUMP_REG -> DRPAI_STATUS_READ_REG */
            drpai_rw_status->rw_status = DRPAI_STATUS_READ_REG;
            break;
        default:
            ; /* Do nothing */
            break;
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);

    switch(drpai_rw_status->rw_status)
    {
        case DRPAI_STATUS_READ_MEM:
            /* Read DRP-AI memory */
            addr = priv->bank | (uint64_t)drpai_rw_status->drpai_data.address;
            p_drpai_cma = phys_to_virt(addr + drpai_rw_status->read_count);
            if (p_drpai_cma == 0)
            {
                result = -EFAULT;
                goto end;
            }
            if ( !( drpai_rw_status->drpai_data.size >= (drpai_rw_status->read_count + count) ) )
            {
                count = drpai_rw_status->drpai_data.size - drpai_rw_status->read_count;
            }
            /* Copy arguments from kernel space to user space */
            if (copy_to_user(buf, (void *)p_drpai_cma, count))
            {
                result = -EFAULT;
                goto end;
            }
            drpai_rw_status->read_count = drpai_rw_status->read_count + count;

            /* DRPAI_STATUS_READ -> DRPAI_STATUS_IDLE_RW */
            if (drpai_rw_status->drpai_data.size <= drpai_rw_status->read_count)
            {
                drpai_rw_status->rw_status = DRPAI_STATUS_IDLE_RW;
                DRPAI_DEBUG_PRINT("Deleted list %px rw_status %d prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->rw_status, drpai_rw_status->list.prev, drpai_rw_status->list.next);
                list_del(&drpai_rw_status->list);
                drpai_rw_status->drpai_data.address = 0x0;
                drpai_rw_status->drpai_data.size    = 0x0;
            }
            i = count;
            break;
        case DRPAI_STATUS_READ_REG:
            /* Read DRP-AI register */
            if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
            {
                result = -ETIMEDOUT;
                goto end;
            }
            /* Secure Kbuf area */
            kbuf = vmalloc(count);
            if (NULL == kbuf)
            {
                result = -ENOMEM;
                goto end;
            }
            for (i = 0; i < count; i+=4)
            {
                if (drp_size > drpai_rw_status->read_count)
                {
                    reg_val = ioread32(drp_base_addr[0] + drpai_rw_status->drp_reg_offset_count);
                    *(kbuf + i)     = (uint8_t)reg_val;
                    *(kbuf + i + 1) = (uint8_t)(reg_val >> 8);
                    *(kbuf + i + 2) = (uint8_t)(reg_val >> 16);
                    *(kbuf + i + 3) = (uint8_t)(reg_val >> 24);
                    drpai_rw_status->drp_reg_offset_count+=4;
                }
                else
                {
                    reg_val = ioread32(aimac_base_address[0] + drpai_rw_status->aimac_reg_offset_count);
                    *(kbuf + i)     = (uint8_t)reg_val;
                    *(kbuf + i + 1) = (uint8_t)(reg_val >> 8);
                    *(kbuf + i + 2) = (uint8_t)(reg_val >> 16);
                    *(kbuf + i + 3) = (uint8_t)(reg_val >> 24);
                    drpai_rw_status->aimac_reg_offset_count+=4;
                }
                drpai_rw_status->read_count+=4;

                /* DRPAI_STATUS_READ_REG -> DRPAI_STATUS_IDLE_RW */
                if ((drp_size + aimac_size) <= drpai_rw_status->read_count)
                {
                    drpai_rw_status->rw_status = DRPAI_STATUS_IDLE_RW;
                    i+=4;
                    break;
                }
            }
            up(&priv->sem);
            /* Copy arguments from kernel space to user space */
            if (copy_to_user(buf, kbuf, count))
            {
                result = -EFAULT;
                goto end;
            }
            break;
        default:
            ; /* Do nothing */
            break;
    }

    result = i;

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ERESTART != result)
    {
        up(&rw_sem);
    }
    if (NULL != kbuf)
    {
        /* Free kbuf */
        vfree(kbuf);
    }
    DRPAI_DEBUG_PRINT("status_rw3:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;

    switch (cmd) {
    case DRPAI_ASSIGN:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_ASSIGN)]\n");
        result = drpai_ioctl_assign(filp, cmd, arg);
        break;
    case DRPAI_START:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_START)]\n");
        result = drpai_ioctl_start(filp, cmd, arg);
        break;
    case DRPAI_RESET:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_RESET)]\n");
        result = drpai_ioctl_reset(filp, cmd, arg);
        break;
    case DRPAI_GET_STATUS:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_GET_STATUS)]\n");
        result = drpai_ioctl_get_status(filp, cmd, arg);
        break;
    case DRPAI_REG_DUMP:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_REG_DUMP)]\n");
        result = drpai_ioctl_reg_dump(filp, cmd, arg);
        break;
    case DRPAI_ASSIGN_PARAM:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_ASSIGN_PARAM)]\n");
        result = drpai_ioctl_assign_param(filp, cmd, arg);
        break;
    case DRPAI_PREPOST_CROP:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_PREPOST_CROP)]\n");
        result = drpai_ioctl_prepost_crop(filp, cmd, arg);
        break;
    case DRPAI_PREPOST_INADDR:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_PREPOST_INADDR)]\n");
        result = drpai_ioctl_prepost_inaddr(filp, cmd, arg);
        break;
    case DRPAI_ASSIGN_DYNAMIC:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_ASSIGN_DYNAMIC)]\n");
        result = drpai_ioctl_assign_dynamic(filp, cmd, arg);
        break;
    case DRPAI_GET_DRPAI_AREA:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_GET_DRPAI_AREA)]\n");
        result = drpai_ioctl_get_drpai_area(filp, cmd, arg);
        break;
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    case DRPAI_SET_SEQ:
        DRPAI_DEBUG_PRINT("[ioctl(DRPAI_SET_SEQ)]\n");
        result = drpai_ioctl_set_seq(filp, cmd, arg);
        break;
#endif /* CONFIG_ARCH_R9A07G054 */
    default:
        DRPAI_DEBUG_PRINT(KERN_WARNING "unsupported command %d\n", cmd);
        result = -EFAULT;
        break;
    }
    goto end;

end:
    return result;
}

static unsigned int drpai_poll( struct file* filp, poll_table* wait )
{
    unsigned int retmask = 0;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;

    DRPAI_DEBUG_PRINT("start.\n");

    spin_lock_irqsave(&priv->lock, flags);

    poll_wait( filp, &drpai_waitq,  wait );

    if (IRQ_CHECK_DISABLE == priv->irq_flag)
    {
        retmask |= ( POLLIN  | POLLRDNORM );
    }

    spin_unlock_irqrestore(&priv->lock, flags);
    DRPAI_DEBUG_PRINT("end.\n");

    return retmask;
}

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
static irqreturn_t irq_drp_nmlint(int irq, void *dev)
{
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;
    drpai_odif_intcnto_t local_odif_intcnto;
    void (*finish_callback)(int);

    DRPAI_DEBUG_PRINT("start.\n");

    spin_lock_irqsave(&priv->lock, flags);
    /* For success ISP call back*/
    finish_callback = priv->isp_finish_loc;

    DRPAI_DEBUG_PRINT("\n");
    DRPAI_DEBUG_PRINT("status1:%d\n", priv->drpai_status.status);

    /* DRP normal interrupt processing */
    R_DRPAI_DRP_Nmlint(drp_base_addr[0], 0, &local_odif_intcnto);

    odif_intcnto.ch0 += local_odif_intcnto.ch0;
    odif_intcnto.ch1 += local_odif_intcnto.ch1;
    odif_intcnto.ch2 += local_odif_intcnto.ch2;
    odif_intcnto.ch3 += local_odif_intcnto.ch3;

    DRPAI_DEBUG_PRINT("ODIF_INTCNTO0 : 0x%08X\n", odif_intcnto.ch0);
    DRPAI_DEBUG_PRINT("ODIF_INTCNTO1 : 0x%08X\n", odif_intcnto.ch1);
    DRPAI_DEBUG_PRINT("ODIF_INTCNTO2 : 0x%08X\n", odif_intcnto.ch2);
    DRPAI_DEBUG_PRINT("ODIF_INTCNTO3 : 0x%08X\n", odif_intcnto.ch3);

    DRPAI_DEBUG_PRINT("local_ODIF_INTCNTO0 : 0x%08X\n", local_odif_intcnto.ch0);
    DRPAI_DEBUG_PRINT("local_ODIF_INTCNTO1 : 0x%08X\n", local_odif_intcnto.ch1);
    DRPAI_DEBUG_PRINT("local_ODIF_INTCNTO2 : 0x%08X\n", local_odif_intcnto.ch2);
    DRPAI_DEBUG_PRINT("local_ODIF_INTCNTO3 : 0x%08X\n", local_odif_intcnto.ch3);

/* DRP single operation */
    if(DRPAI_EXE_AI == exe_mode)
    {
        if  ((1 == odif_intcnto.ch0) &&
            (1 == odif_intcnto.ch1) &&
            (1 == odif_intcnto.ch2) &&
            (1 == odif_intcnto.ch3))
        {
            /* Internal state update */
            priv->drpai_status.status = DRPAI_STATUS_IDLE;
            DRPAI_DEBUG_PRINT("status2:%d\n", priv->drpai_status.status);
            /* For success ISP call back*/
            if(NULL != finish_callback)
            {
                (*finish_callback)(ISP_FINISH_SUCCESS);
            }
        }
    }
    else if (DRPAI_EXE_DRP == exe_mode)
    {
        if ((seq.num == odif_intcnto.ch0) &&
            (seq.num == odif_intcnto.ch1) &&
            (seq.num == odif_intcnto.ch2) &&
            (seq.num == odif_intcnto.ch3))
        {
            /* Internal state update */
            priv->drpai_status.status = DRPAI_STATUS_IDLE;
            DRPAI_DEBUG_PRINT("status2:%d\n", priv->drpai_status.status);
            priv->irq_flag = IRQ_CHECK_DISABLE;
            /* Wake up the process */
            wake_up_interruptible( &drpai_waitq );
        }
    }
    else
    {
        // do nothing
    }
/* DRP single operation */

    priv->isp_finish_loc = NULL;
    spin_unlock_irqrestore(&priv->lock, flags);

    DRPAI_DEBUG_PRINT("end.\n");

    return IRQ_HANDLED;
}
#endif /* CONFIG_ARCH_R9A07G054 */

static irqreturn_t irq_drp_errint(int irq, void *dev)
{
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    void (*finish_callback)(int); /* For error simple ISP call back*/
#endif /* CONFIG_ARCH_R9A07G054 */

    DRPAI_DEBUG_PRINT("start.\n");

    spin_lock_irqsave(&priv->lock, flags);

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    finish_callback = priv->isp_finish_loc;

    if(NULL != finish_callback)
    {
        (*finish_callback)(-EIO);
    }
#endif /* CONFIG_ARCH_R9A07G054 */

    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    /* DRP error interrupt processing */
    R_DRPAI_DRP_Errint(drp_base_addr[0], aimac_base_address[0], 0);

    /* Internal state update */
    priv->drpai_status.err = DRPAI_ERRINFO_DRP_ERR;
    priv->drpai_status.status = DRPAI_STATUS_IDLE;
    priv->irq_flag = IRQ_CHECK_DISABLE;

#if defined(CONFIG_ARCH_R9A07G054)
    /* V2L conditional compilation */
    /* Wake up the process when it's not ISP mode*/
    if(NULL == finish_callback)
    {
        wake_up_interruptible( &drpai_waitq );
    }
    priv->isp_finish_loc = NULL;
#elif defined(CONFIG_ARCH_R9A09G011GBG) || defined(CONFIG_ARCH_R9A09G055MA3GBG)
    /* V2M(A) conditional compilation */
    /* Wake up the process */
    wake_up_interruptible( &drpai_waitq );
#endif 

    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);
    spin_unlock_irqrestore(&priv->lock, flags);
    DRPAI_DEBUG_PRINT("end.\n");

    return IRQ_HANDLED;
}

static irqreturn_t irq_mac_nmlint(int irq, void *dev)
{
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;

    DRPAI_DEBUG_PRINT("start.\n");

    spin_lock_irqsave(&priv->lock, flags);
    DRPAI_DEBUG_PRINT(" \n");
    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    /* AI-MAC normal interrupt processing */
    R_DRPAI_AIMAC_Nmlint(aimac_base_address[0], 0);

    /* Internal state update */
    priv->drpai_status.status = DRPAI_STATUS_IDLE;
    priv->irq_flag = IRQ_CHECK_DISABLE;

    /* Wake up the process */
    wake_up_interruptible( &drpai_waitq );

    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);
    spin_unlock_irqrestore(&priv->lock, flags);
    DRPAI_DEBUG_PRINT("end.\n");

    return IRQ_HANDLED;
}

static irqreturn_t irq_mac_errint(int irq, void *dev)
{
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;

    DRPAI_DEBUG_PRINT("start.\n");

    spin_lock_irqsave(&priv->lock, flags);
    DRPAI_DEBUG_PRINT(" \n");
    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    /* AI-MAC error interrupt processing */
    R_DRPAI_AIMAC_Errint(drp_base_addr[0], aimac_base_address[0], 0);

    /* Internal state update */
    priv->drpai_status.err = DRPAI_ERRINFO_AIMAC_ERR;
    priv->drpai_status.status = DRPAI_STATUS_IDLE;
    priv->irq_flag = IRQ_CHECK_DISABLE;

    /* Wake up the process */
    wake_up_interruptible( &drpai_waitq );

    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);
    spin_unlock_irqrestore(&priv->lock, flags);
    DRPAI_DEBUG_PRINT("end.\n");

    return IRQ_HANDLED;
}

static int drpai_regist_driver(void)
{
    int alloc_ret = 0;
    int cdev_err = 0;
    dev_t dev;
    int minor;
    int ptr_err;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Get free major number. */
    alloc_ret = alloc_chrdev_region(&dev, MINOR_BASE, MINOR_NUM, DRPAI_DRIVER_NAME);
    if (alloc_ret != 0) {
        pr_err("DRP-AI Driver: alloc_chrdev_region = %d\n", alloc_ret);
        return -1;
    }

    /* Save major number. */
    drpai_major = MAJOR(dev);
    dev = MKDEV(drpai_major, MINOR_BASE);

    /* Initialize cdev and registration handler table. */
    cdev_init(&drpai_cdev, &s_mydevice_fops);
    drpai_cdev.owner = THIS_MODULE;

    /* Registration cdev */
    cdev_err = cdev_add(&drpai_cdev, dev, MINOR_NUM);
    if (cdev_err != 0) {
        pr_err("DRP-AI Driver: cdev_add = %d\n", cdev_err);
        unregister_chrdev_region(dev, MINOR_NUM);
        return -1;
    }

    /* Cleate class "/sys/class/drpai/" */
    drpai_class = class_create(THIS_MODULE, DRPAI_DRIVER_NAME);
    if (IS_ERR(drpai_class)) {
        ptr_err = PTR_ERR(drpai_class);
        pr_err("DRP-AI Driver: class_create = %d\n", ptr_err);
        cdev_del(&drpai_cdev);
        unregister_chrdev_region(dev, MINOR_NUM);
        return -1;
    }

    /* Make "/sys/class/drpai/drpai*" */
    for (minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++) {
        drpai_device_array[minor - MINOR_BASE] =
        device_create(drpai_class, NULL, MKDEV(drpai_major, minor), NULL, DRPAI_DRIVER_NAME "%d", minor);
    }

    DRPAI_DEBUG_PRINT("end.\n");

    return 0;
}

static int drpai_regist_device(struct platform_device *pdev)
{
    struct resource *res;
    struct resource reserved_res;
    struct device_node *np;
    struct drpai_priv *priv;
    struct drpai_rw_status *drpai_rw_status;
    int irq, ret;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Intialize DRP-AI status to control */
    priv = devm_kzalloc(&pdev->dev, sizeof(struct drpai_priv), GFP_KERNEL);
    if (!priv) {
        dev_err(&pdev->dev, "cannot allocate private data\n");
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, priv);
    priv->pdev = pdev;
    priv->dev_name = dev_name(&pdev->dev);
    spin_lock_init(&priv->lock);
    sema_init(&priv->sem, DRPAI_DEV_NUM);
    refcount_set(&priv->count, 1);
    priv->drpai_status.status = DRPAI_STATUS_INIT; 
    priv->drpai_status.err    = DRPAI_ERRINFO_SUCCESS;
    drpai_priv = priv;

    /* Initialize list head */
    drpai_rw_status = devm_kzalloc(&pdev->dev, sizeof(struct drpai_rw_status), GFP_KERNEL);
    if (!drpai_rw_status) {
        dev_err(&pdev->dev, "cannot allocate sentinel data\n");
        return -ENOMEM;
    }
    drpai_rw_status->rw_status  = HEAD_SENTINEL;
    drpai_rw_status->param_info = NULL;
    INIT_LIST_HEAD(&drpai_rw_status->list);
    atomic_set(&drpai_rw_status->inout_flag, 0);
    drpai_rw_sentinel = drpai_rw_status;
    DRPAI_DEBUG_PRINT("HEAD  list %px rw_status %d prev %px next %px\n", &drpai_rw_sentinel->list, drpai_rw_sentinel->rw_status, drpai_rw_sentinel->list.prev, drpai_rw_sentinel->list.next);

    if (of_device_is_compatible(pdev->dev.of_node, "renesas,rzv2m-drpai"))
    {
        drpai_priv->dev_tag = DEVICE_RZV2M;
        dev_info(&pdev->dev, "DRP-AI Driver version : %s V2M\n", DRPAI_DRIVER_VERSION DRPAI_DRV_DEBUG_MODE DRPAI_DRV_DEBUG_WAIT_MODE);
    }
    else if (of_device_is_compatible(pdev->dev.of_node, "renesas,rzv2ma-drpai"))
    {
        drpai_priv->dev_tag = DEVICE_RZV2MA;
        dev_info(&pdev->dev, "DRP-AI Driver version : %s V2MA\n", DRPAI_DRIVER_VERSION DRPAI_DRV_DEBUG_MODE DRPAI_DRV_DEBUG_WAIT_MODE);
    }
    else if (of_device_is_compatible(pdev->dev.of_node, "renesas,rzv2l-drpai"))
    {
        drpai_priv->dev_tag = DEVICE_RZV2L;
        dev_info(&pdev->dev, "DRP-AI Driver version : %s V2L\n", DRPAI_DRIVER_VERSION DRPAI_DRV_DEBUG_MODE DRPAI_DRV_DEBUG_WAIT_MODE);
    }
    else
    {
        /* Do Nothing */
    }

    /* Get reserved register region from Device tree.*/
    np = of_parse_phandle(pdev->dev.of_node, "sysctrl", 0);
    if (!np) {
        dev_err(&pdev->dev, "No %s specified\n", "sysctrl");
        return -ENOMEM;
    }

    /* Convert register region to a struct resource */
    ret = of_address_to_resource(np, 0, &reserved_res);
    if (ret) {
        dev_err(&pdev->dev, "No memory address assigned to the region\n");
        return -ENOMEM;
    }
    sysctrl_region_base_addr = reserved_res.start;
    dev_info(&pdev->dev, "sysctrl register region start 0x%016llX\n", sysctrl_region_base_addr);

    /* Get reserved memory region from Device tree.*/
    np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
    if (!np) {
        dev_err(&pdev->dev, "No %s specified\n", "memory-region");
        return -ENOMEM;
    }

    /* Convert memory region to a struct resource */
    ret = of_address_to_resource(np, 0, &reserved_res);
    if (ret) {
        dev_err(&pdev->dev, "No memory address assigned to the region\n");
        return -ENOMEM;
    }
    drpai_region_base_addr = reserved_res.start;
    drpai_region_size = resource_size(&reserved_res);
    dev_info(&pdev->dev, "DRP-AI memory region start 0x%016llX, size 0x%08llX\n", drpai_region_base_addr, drpai_region_size);

    /* Get linux memory region from Device tree.*/
    np = of_parse_phandle(pdev->dev.of_node, "linux-memory-region", 0);
    if (!np) {
        dev_err(&pdev->dev, "No %s specified\n", "linux-memory-region");
        return -ENOMEM;
    }

    /* read linux start address and size */
    ret = of_address_to_resource(np, 0, &reserved_res);
    if (ret) {
        dev_err(&pdev->dev, "No address assigned to the linux-memory-region\n");
        return -ENOMEM;
    }
    drpai_linux_mem_start = reserved_res.start;
    drpai_linux_mem_size = resource_size(&reserved_res);
    dev_info(&pdev->dev, "linux-memory-region start 0x%016llX, size 0x%08llX\n", drpai_linux_mem_start, drpai_linux_mem_size);

    /* Convert DRP base address from physical to virtual */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "cannot get resources (reg)\n");
        return -EINVAL;
    }
    priv->drp_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
    if (!priv->drp_base) {
        dev_err(&pdev->dev, "cannot ioremap\n");
        return -EINVAL;
    }
    drp_base_addr[0] = priv->drp_base;
    drp_size = resource_size(res);
    dev_info(&pdev->dev, "DRP base address 0x%08llX, size 0x%08llX\n", res->start, drp_size);

    /* Convert AI-MAC base address from physical to virtual */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (!res) {
        dev_err(&pdev->dev, "cannot get resources (reg)\n");
        return -EINVAL;
    }
    priv->aimac_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
    if (!priv->aimac_base) {
        dev_err(&pdev->dev, "cannot ioremap\n");
        return -EINVAL;
    }
    aimac_base_address[0] = priv->aimac_base;
    aimac_size = resource_size(res);
    dev_info(&pdev->dev, "AI-MAC base address 0x%08llX, size 0x%08llX\n", res->start, aimac_size);

    /* Registering an interrupt handler */
    irq = platform_get_irq(pdev, 1);
    ret = devm_request_irq(&pdev->dev, irq, irq_drp_errint, 0, "drpa errint", priv);
    if (ret) {
        dev_err(&pdev->dev, "Failed to claim IRQ!\n");
        return ret;
    }
    irq = platform_get_irq(pdev, 2);
    ret = devm_request_irq(&pdev->dev, irq, irq_mac_nmlint, 0, "drpa mac_nmlint", priv);
    if (ret) {
        dev_err(&pdev->dev, "Failed to claim IRQ!\n");
        return ret;
    }
    irq = platform_get_irq(pdev, 3);
    ret = devm_request_irq(&pdev->dev, irq, irq_mac_errint, 0, "drpa mac_errint", priv);
    if (ret) {
        dev_err(&pdev->dev, "Failed to claim IRQ!\n");
        return ret;
    }

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054

    irq = platform_get_irq(pdev, 0);
    ret = devm_request_irq(&pdev->dev, irq, irq_drp_nmlint, 0, "drpa nmlint", priv);
    if (ret) {
        dev_err(&pdev->dev, "Failed to claim IRQ!\n");
        return ret;
    }
    /* Get clock controller info */
    priv->clk_int = devm_clk_get(&pdev->dev, "intclk");
    if (IS_ERR(priv->clk_int)) {
        dev_err(&pdev->dev, "missing controller clock");
        return PTR_ERR(priv->clk_int);
    }
    
    priv->clk_aclk_drp = devm_clk_get(&pdev->dev, "aclk_drp");
    if (IS_ERR(priv->clk_aclk_drp)) {
        dev_err(&pdev->dev, "missing controller clock");
        return PTR_ERR(priv->clk_aclk_drp);
    }
    
    priv->clk_mclk = devm_clk_get(&pdev->dev, "mclk");
    if (IS_ERR(priv->clk_mclk)) {
        dev_err(&pdev->dev, "missing controller clock");
        return PTR_ERR(priv->clk_mclk);
    }
    
    priv->clk_dclkin = devm_clk_get(&pdev->dev, "dclkin");
    if (IS_ERR(priv->clk_dclkin)) {
        dev_err(&pdev->dev, "missing controller clock");
        return PTR_ERR(priv->clk_dclkin);
    }
    
    priv->clk_aclk = devm_clk_get(&pdev->dev, "aclk");
    if (IS_ERR(priv->clk_aclk)) {
        dev_err(&pdev->dev, "missing controller clock");
        return PTR_ERR(priv->clk_aclk);
    }

#endif /* CONFIG_ARCH_R9A07G054 */

    /* Get reset controller info */
    priv->rstc = devm_reset_control_get(&pdev->dev, NULL);
    if (IS_ERR(priv->rstc))
    {
        dev_err(&pdev->dev, "Failed to get DRP CPG reset controller\n");
        return PTR_ERR(priv->rstc);
    }
    else
    {
        DRPAI_DEBUG_PRINT("Get DRP CPG reset controller\n");        
    } 

    DRPAI_DEBUG_PRINT("end.\n");

    return 0;
}

static void drpai_unregist_driver(void)
{
    dev_t dev = MKDEV(drpai_major, MINOR_BASE);
    int minor;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Delete "/sys/class/mydevice/mydevice*". */
    for (minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++) {
        device_destroy(drpai_class, MKDEV(drpai_major, minor));
    }

    /* Destroy "/sys/class/mydevice/". */
    class_destroy(drpai_class);

    /* Delete cdev from kernel. */
    cdev_del(&drpai_cdev);

    /* Unregistration */
    unregister_chrdev_region(dev, MINOR_NUM);
    DRPAI_DEBUG_PRINT("end.\n");
}

static void drpai_unregist_device(void)
{
    DRPAI_DEBUG_PRINT("start.\n");
    /* Do nothing */
    DRPAI_DEBUG_PRINT("end.\n");
}

static int8_t drpai_reset_device(uint32_t ch)
{
    int8_t retval;
    struct drpai_priv *priv = drpai_priv;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Reset DRP */
    if(R_DRPAI_SUCCESS != R_DRPAI_DRP_Reset(drp_base_addr[0], aimac_base_address[0], ch, &priv->lock)) 
    {
        goto err_reset;
    }

    /* Reset AI-MAC */
    if(R_DRPAI_SUCCESS != R_DRPAI_AIMAC_Reset(aimac_base_address[0], ch)) 
    {
        goto err_reset;
    }

    /* Reset CPG register */
    if(R_DRPAI_SUCCESS != R_DRPAI_CPG_Reset(priv->rstc)) 
    {
        goto err_reset;
    }

    retval = R_DRPAI_SUCCESS;
    goto end;
err_reset:
    retval = R_DRPAI_ERR_RESET;
    goto end;
end:
    DRPAI_DEBUG_PRINT("end.\n");

    return retval;
}

static void drpai_init_device(uint32_t ch)
{
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;

    DRPAI_DEBUG_PRINT("start.\n");

    spin_lock_irqsave(&priv->lock, flags);
    priv->irq_flag = IRQ_CHECK_DISABLE;
    spin_unlock_irqrestore(&priv->lock, flags);
    (void)R_DRPAI_DRP_Open(drp_base_addr[0], 0, &priv->lock);
    (void)R_DRPAI_AIMAC_Open(aimac_base_address[0], 0);

    DRPAI_DEBUG_PRINT("end.\n");
}

static ssize_t drpai_write_dynamic(struct file *filp, uint8_t *pvirt)
{
    ssize_t result = 0;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;

    DRPAI_DEBUG_PRINT("start.\n");

    if(0 != (drpai_rw_status->drpai_data_dynamic.file_type & DRPAI_FILE_TYPE_DESC))
    {
        result = drpai_write_dynamic_desc(filp, pvirt);
    }
    else if (0 != (drpai_rw_status->drpai_data_dynamic.file_type & DRPAI_FILE_TYPE_PARAM))
    {
        result = drpai_write_dynamic_param(filp, pvirt);
    }

    DRPAI_DEBUG_PRINT("end.\n");
    return result;
}

static ssize_t drpai_write_dynamic_desc(struct file *filp, uint8_t *pvirt)
{
    ssize_t result = 0;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    uint32_t entry_num = drpai_rw_status->drpai_data_dynamic.size / DRP_DESC_ENTRY_SIZE; /* number of descriptor entry */
    uint32_t entry_index;
    uint8_t *entry;
    uint8_t type;
    uint8_t cf;
    uint8_t mod;
    uint8_t func;

    DRPAI_DEBUG_PRINT("start.\n");

    /* drp_desc.bin/aimac_desc.bin address rewriting. */
    for(entry_index = 0; entry_index < entry_num; entry_index++)
    {
        entry = &pvirt[DRP_DESC_ENTRY_SIZE * entry_index];
        type  = ((entry[DRP_DESC_TYPE_INDEX] >> DRP_DESC_TYPE_RSHIFT) & DRP_DESC_TYPE_MASK);
        cf    = ((entry[DRP_DESC_CF_INDEX]   >> DRP_DESC_CF_RSHIFT)   & DRP_DESC_CF_MASK);
        mod   = ((entry[DRP_DESC_MOD_INDEX]  >> DRP_DESC_MOD_RSHIFT)  & DRP_DESC_MOD_MASK);
        func  = ((entry[DRP_DESC_FUNC_INDEX] >> DRP_DESC_FUNC_RSHIFT) & DRP_DESC_FUNC_MASK);

        if(0 == type)
        {
            switch (cf)
            {
            /* DRP / DMA Transfer */
            case DRP_DESC_CF_DRP_DMA:
                if(0 == mod)
                {
                    if(0 == func)
                    {
                        DRPAI_DEBUG_PRINT("entry = %d: cf = %d, mod = %d, func = %d\n", entry_index, cf, mod, func);
                        drpai_rewrite_address(&entry[DRP_DESC_ADDR_INDEX], drpai_rw_status->drpai_data_dynamic.start_address);
                    }
                }
                break;

            /* AIMAC / DMA Transfer */
            case DRP_DESC_CF_AIMAC_DMA0:
            case DRP_DESC_CF_AIMAC_DMA1:
            case DRP_DESC_CF_AIMAC_DMA2:
            case DRP_DESC_CF_AIMAC_DMA3:
                if(0 == mod)
                {
                    if(0 == func)
                    {
                        DRPAI_DEBUG_PRINT("entry = %d: cf = %d, mod = %d, func = %d\n", entry_index, cf, mod, func);
                        drpai_rewrite_address(&entry[DRP_DESC_ADDR_INDEX], drpai_rw_status->drpai_data_dynamic.start_address);
                    }
                }
                break;

            /* AIMAC / Parameter Transfer */
            case DRP_DESC_CF_AIMAC_PARAM:
                if(0 == mod)
                {
                    if(1 == func)
                    {
                        DRPAI_DEBUG_PRINT("entry = %d: cf = %d, mod = %d, func = %d\n", entry_index, cf, mod, func);
                        drpai_rewrite_address(&entry[DRP_DESC_ADDR_INDEX], drpai_rw_status->drpai_data_dynamic.start_address);
                    }
                }
                break;

            /* CFG Load */
            case DRP_DESC_CF_CFG_LOAD:
                DRPAI_DEBUG_PRINT("entry = %d: cf = %d, mod = %d, func = %d\n", entry_index, cf, mod, func);
                drpai_rewrite_address(&entry[DRP_DESC_ADDR_INDEX], drpai_rw_status->drpai_data_dynamic.start_address);
                break;

            default:
                break;
            }
        }
    }

    DRPAI_DEBUG_PRINT("end.\n");
    return result;
}

static ssize_t drpai_write_dynamic_param(struct file *filp, uint8_t *pvirt)
{
    ssize_t result = 0;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    char *ptr = drpai_rw_status->param_info;
    char *prev_ptr;
    char *buf = NULL;
    uint32_t buf_len;
    char *pbuf;
    unsigned long offset_add = 0;
    char *param_name;
    int is_raddr;
    int is_waddr;
    char *paddr;
    unsigned long param_offset;
    struct param_info_list *list_top = NULL;
    struct param_info_list *list_cur = NULL;

    DRPAI_DEBUG_PRINT("start.\n");

    if(NULL == ptr)
    {
        result = -EACCES;
        goto end;
    }

    /* drp_param_info.txt parsing */
    while(('\0' != *ptr) && (ptr < drpai_rw_status->param_info + drpai_rw_status->assign_param.info_size))
    {
        /* Save current pointer */
        prev_ptr = ptr;

        /* Get 1 line */
        ptr = strchr(ptr, '\n');
        if(NULL == ptr)
        {
            ptr = drpai_rw_status->param_info + drpai_rw_status->assign_param.info_size;
        }
        else
        {
            ptr += 1;
        }
        if(ptr <= prev_ptr)
        {
            break;
        }
        buf_len = ptr - prev_ptr;

        /* Copy only 1line to buffer */
        buf = vmalloc(buf_len + 1);
        if(NULL == buf)
        {
            result = -EFAULT;
            goto end;
        }
        strncpy(buf, prev_ptr, buf_len);
        buf[buf_len] = '\0';

        /* Search and set offset_add */
        pbuf = get_param_attr(buf, DRP_PARAM_ATTR_OFFSET_ADD, &offset_add, NULL);

        /* Processing not "OFFSET_ADD" lines */
        if(NULL == pbuf)
        {
            /* Register the offset values of the following parameters in the list */
            /* - Name is "raddr" or "waddr"                                       */
            /* - Name ends with "_ADDR"                                           */
            pbuf = get_param_attr(buf, DRP_PARAM_ATTR_PARAM, NULL, &param_name);
            if(NULL != pbuf)
            {
                is_raddr = strncmp(param_name, DRP_PARAM_NAME_RADDR, strlen(DRP_PARAM_NAME_RADDR));
                is_waddr = strncmp(param_name, DRP_PARAM_NAME_WADDR, strlen(DRP_PARAM_NAME_WADDR));
                paddr = strstr(param_name, DRP_PARAM_NAME_ADDR);
                if(NULL != paddr)
                {
                    paddr += strlen(DRP_PARAM_NAME_ADDR);
                }
                if((0 == is_raddr) || (0 == is_waddr) || ((NULL != paddr) && ('\0' == *paddr)))
                {
                    pbuf = get_param_attr(pbuf, DRP_PARAM_ATTR_OFFSET, &param_offset, NULL);
                    if(NULL != pbuf)
                    {
                        DRPAI_DEBUG_PRINT("offset_add = %d, param_name = %s, param_offset = %d\n", offset_add, param_name, param_offset);
                        list_cur = param_info_list_push(&list_top, list_cur, offset_add + param_offset);
                        if(NULL == list_cur)
                        {
                            result = -EFAULT;
                            goto end;
                        }
                    }
                }
            }
        }
        vfree(buf);
        buf = NULL;
    }

    /* drp_param.bin address rewriting. */
    list_cur = list_top;
    while (NULL != list_cur)
    {
        if((list_cur->offset + 4) <= drpai_rw_status->drpai_data_dynamic.size)
        {
            drpai_rewrite_address(&pvirt[list_cur->offset], drpai_rw_status->drpai_data_dynamic.start_address);
        }

        list_cur = list_cur->next;
    }

end:
    if(NULL != buf)
    {
        vfree(buf);
    }
    param_info_list_clear(list_top);

    DRPAI_DEBUG_PRINT("end.\n");
    return result;
}

static void drpai_rewrite_address(uint8_t *pos, uint32_t addr)
{
    uint32_t original_addr;

    DRPAI_DEBUG_PRINT("start.\n");

    original_addr = (((uint32_t)pos[3] & 0xFF) << 24) |
                    (((uint32_t)pos[2] & 0xFF) << 16) |
                    (((uint32_t)pos[1] & 0xFF) <<  8) |
                    (((uint32_t)pos[0] & 0xFF) <<  0);

    original_addr += addr;

    pos[0] = (uint8_t)((original_addr >>  0) & 0xFF);
    pos[1] = (uint8_t)((original_addr >>  8) & 0xFF);
    pos[2] = (uint8_t)((original_addr >> 16) & 0xFF);
    pos[3] = (uint8_t)((original_addr >> 24) & 0xFF);
    DRPAI_DEBUG_PRINT("end.\n");
}

static struct param_info_list* param_info_list_push(struct param_info_list **pptop, struct param_info_list* pcur, uint32_t offset)
{
    struct param_info_list *next = NULL;

    DRPAI_DEBUG_PRINT("start.\n");

    if(NULL == pptop)
    {
        goto end;
    }

    next = (struct param_info_list *)vmalloc(sizeof(struct param_info_list));
    if(NULL != next)
    {
        next->offset = offset;
        next->next = NULL;
        if(NULL == *pptop)
        {
            *pptop = next;
        }
        else
        {
            if(NULL == pcur)
            {
                vfree(next);
                next = NULL;
                goto end;
            }
            pcur->next = next;
        }
    }

end:
    DRPAI_DEBUG_PRINT("end.\n");
    return next;
}

static void param_info_list_clear(struct param_info_list* ptop)
{
    struct param_info_list* pos;
    struct param_info_list* npos;

    DRPAI_DEBUG_PRINT("start.\n");

    pos = ptop;
    while (pos != NULL)
    {
        npos = pos->next;
        vfree(pos);
        pos = npos;
    }

    DRPAI_DEBUG_PRINT("end.\n");
}

static long drpai_ioctl_assign(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    volatile void *p_virt_address = 0;
    struct drpai_priv *priv = drpai_priv;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    struct drpai_rw_status *entry;
    struct list_head *listitr;
    drpai_data_t drpai_data_buf;
    uint64_t addr, size;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_trylock(&rw_sem)))
    {
        result = -ERESTART;
        goto end;
    }

    DRPAI_DEBUG_PRINT(" \n");
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    /* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }

    /* Check status */
    if (DRPAI_STATUS_IDLE_RW != drpai_rw_status->rw_status)
    {
        result = -EACCES;
        goto end;
    }

    /* Copy arguments from user space to kernel space */
    if (copy_from_user(&drpai_data_buf, (void __user *)arg, sizeof(drpai_data_t)))
    {
        result = -EFAULT;
        goto end;
    }
    /* Check Argument */
    addr = priv->bank | (uint64_t)drpai_data_buf.address;
    size = (uint64_t)drpai_data_buf.size;
    if (0 != (addr & DRPAI_64BYTE_ALIGN))
    {
        result = -EINVAL;
        goto end;
    }
    if ((drpai_region_base_addr > addr) || 
       ((drpai_region_base_addr + drpai_region_size) <= (addr + size)))
    {
        result = -EINVAL;
        goto end;
    }
    if (0 == size)
    {
        result = -EINVAL;
        goto end;
    }

    /* Check the assigned address */
    DRPAI_DEBUG_PRINT("list %px prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->list.prev, drpai_rw_status->list.next);
    if(!list_empty(&drpai_rw_sentinel->list))
    {   
        DRPAI_DEBUG_PRINT("List is not empty\n");
        list_for_each(listitr, &drpai_rw_sentinel->list)
        {
            entry = list_entry(listitr, struct drpai_rw_status, list);
            DRPAI_DEBUG_PRINT("rw_status %d list %px prev %px next %px\n", entry->rw_status, &entry->list, entry->list.prev, entry->list.next);
            if( (DRPAI_STATUS_ASSIGN   == entry->rw_status) ||
                (DRPAI_STATUS_WRITE    == entry->rw_status) ||
                (DRPAI_STATUS_READ_MEM == entry->rw_status)    )
            {
                if(!(  (entry->drpai_data.address > (drpai_data_buf.address + drpai_data_buf.size - 1)) ||
                      ((entry->drpai_data.address + entry->drpai_data.size - 1) < drpai_data_buf.address)  ))
                {
                    result = -EINVAL;
                    goto end;
                }
            }
            else if(    (DRPAI_STATUS_ASSIGN_DYNAMIC == entry->rw_status)||
                        (DRPAI_STATUS_WRITE_DYNAMIC  == entry->rw_status)   )
            {
                if(!( ((entry->drpai_data_dynamic.start_address + entry->drpai_data_dynamic.offset) > (drpai_data_buf.address + drpai_data_buf.size - 1)) ||
                      ((entry->drpai_data_dynamic.start_address + entry->drpai_data_dynamic.offset + entry->drpai_data.size - 1) < drpai_data_buf.address)   ))
                {
                    result = -EINVAL;
                    goto end;
                }
            }
        }
    }

    /* Data cache invalidate. DRP-AI W -> CPU R */
    addr = priv->bank | (uint64_t)drpai_data_buf.address;
    p_virt_address = phys_to_virt(addr);
    if (p_virt_address == 0)
    {
        result = -EFAULT;
        goto end;
    }
    __inval_dcache_area((void *)p_virt_address, drpai_data_buf.size);

    /* Initialization of read / write processing variables */
    drpai_rw_status->drpai_data  = drpai_data_buf;
    drpai_rw_status->rw_status   = DRPAI_STATUS_ASSIGN;
    drpai_rw_status->write_count = 0;
    drpai_rw_status->read_count  = 0;
    /* Register assigned status */
    list_add(&drpai_rw_status->list, &drpai_rw_sentinel->list);
    DRPAI_DEBUG_PRINT("Registered list %px prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->list.prev, drpai_rw_status->list.next);

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ERESTART != result)
    {
        up(&rw_sem);
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl_assign_dynamic(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    struct drpai_priv *priv = drpai_priv;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    struct drpai_rw_status *entry;
    struct list_head *listitr;
    drpai_data_dynamic_t drpai_data_dynamic_buf;
    uint64_t addr, size;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_trylock(&rw_sem)))
    {
        result = -ERESTART;
        goto end;
    }

    DRPAI_DEBUG_PRINT(" \n");
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    /* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }

    /* Check status */
    if (DRPAI_STATUS_IDLE_RW != drpai_rw_status->rw_status)
    {
        result = -EACCES;
        goto end;
    }

    /* Copy arguments from user space to kernel space */
    if (copy_from_user(&drpai_data_dynamic_buf, (void __user *)arg, sizeof(drpai_data_dynamic_t)))
    {
        result = -EFAULT;
        goto end;
    }
    /* Check Argument */
    if (0 != (drpai_data_dynamic_buf.start_address & DRPAI_64BYTE_ALIGN))
    {
        result = -EINVAL;
        goto end;
    }
    addr = priv->bank | ((uint64_t)drpai_data_dynamic_buf.start_address + (uint64_t)drpai_data_dynamic_buf.offset);
    size = (uint64_t)drpai_data_dynamic_buf.size;
    if (0 != (addr & DRPAI_64BYTE_ALIGN))
    {
        result = -EINVAL;
        goto end;
    }
    if ((drpai_region_base_addr > addr) || 
       ((drpai_region_base_addr + drpai_region_size) <= (addr + size)))
    {
        result = -EINVAL;
        goto end;
    }
    if (0 == size)
    {
        result = -EINVAL;
        goto end;
    }

    /* Check the assigned address */
    DRPAI_DEBUG_PRINT("list %px prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->list.prev, drpai_rw_status->list.next);
    if(!list_empty(&drpai_rw_sentinel->list))
    {   
        DRPAI_DEBUG_PRINT("List is not empty\n");
        list_for_each(listitr, &drpai_rw_sentinel->list)
        {
            entry = list_entry(listitr, struct drpai_rw_status, list);
            DRPAI_DEBUG_PRINT("rw_status %d list %px prev %px next %px\n", entry->rw_status, &entry->list, entry->list.prev, entry->list.next);
            if( (DRPAI_STATUS_ASSIGN   == entry->rw_status) ||
                (DRPAI_STATUS_WRITE    == entry->rw_status) ||
                (DRPAI_STATUS_READ_MEM == entry->rw_status)    )
            {
                if(!(  (entry->drpai_data.address > (drpai_data_dynamic_buf.start_address + drpai_data_dynamic_buf.offset + drpai_data_dynamic_buf.size - 1)) ||
                      ((entry->drpai_data.address + entry->drpai_data.size - 1) < (drpai_data_dynamic_buf.start_address + drpai_data_dynamic_buf.offset))        ))
                {
                    result = -EINVAL;
                    goto end;
                }
            }
            else if(    (DRPAI_STATUS_ASSIGN_DYNAMIC == entry->rw_status)||
                        (DRPAI_STATUS_WRITE_DYNAMIC  == entry->rw_status)   )
            {
                if(!( ((entry->drpai_data_dynamic.start_address + entry->drpai_data_dynamic.offset) >
                           (drpai_data_dynamic_buf.start_address + drpai_data_dynamic_buf.offset + drpai_data_dynamic_buf.size - 1)) ||
                      ((entry->drpai_data_dynamic.start_address + entry->drpai_data_dynamic.offset + entry->drpai_data_dynamic.size - 1) <
                           (drpai_data_dynamic_buf.start_address + drpai_data_dynamic_buf.offset))                                          ))
                {
                    result = -EINVAL;
                    goto end;
                }
            }
        }
    }

    /* Initialization of read / write processing variables */
    drpai_rw_status->drpai_data_dynamic = drpai_data_dynamic_buf;
    drpai_rw_status->rw_status          = DRPAI_STATUS_ASSIGN_DYNAMIC;
    drpai_rw_status->write_count        = 0;
    drpai_rw_status->read_count         = 0;
    /* Register assigned status */
    list_add(&drpai_rw_status->list, &drpai_rw_sentinel->list);
    DRPAI_DEBUG_PRINT("Registered list %px prev %px next %px\n", &drpai_rw_status->list, drpai_rw_status->list.prev, drpai_rw_status->list.next);

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ERESTART != result)
    {
        up(&rw_sem);
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl_start(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int result = 0;
    drpai_data_t proc[DRPAI_INDEX_NUM];
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
/* DRP single operation */
    drpai_data_t proc_drp[DRPAI_SEQ_NUM * 2];
/* DRP single operation */
#endif /* CONFIG_ARCH_R9A07G054 */
    volatile void *p_drp_desc = 0;
    volatile void *p_aimac_desc = 0;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;
    int i;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    uint64_t addr, size;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
    {
        result = -ETIMEDOUT;
        goto end;
    }

    DRPAI_DEBUG_PRINT(" \n");
    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    /* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }

    /* Check status */
    spin_lock_irqsave(&priv->lock, flags);
    if (DRPAI_STATUS_RUN == priv->drpai_status.status)
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EBUSY;
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
/* DRP single operation */
    if(DRPAI_EXE_AI == exe_mode)
    {
#endif
        /* Copy arguments from user space to kernel space */
        if (copy_from_user(&proc[0], (void __user *)arg, sizeof(proc)))
        {
            result = -EFAULT;
            goto end;
        }
        /* Check Argument */
        for (i = DRPAI_INDEX_DRP_DESC; i < DRPAI_INDEX_NUM; i++)
        {
            addr = priv->bank | (uint64_t)proc[i].address;
            size = (uint64_t)proc[i].size;
            if (0 != (addr & DRPAI_64BYTE_ALIGN))
            {
                result = -EINVAL;
                goto end;
            }
            if ((drpai_region_base_addr > addr) || 
            ((drpai_region_base_addr + drpai_region_size) <= (addr + size)))
                {
                    result = -EINVAL;
                    goto end;
                }
        }

        /* Check if input is in linux memory region */
        if(0 == atomic_read(&drpai_rw_status->inout_flag))
        {
            DRPAI_DEBUG_PRINT("Use arg variable\n");
            addr = priv->bank | (uint64_t)proc[DRPAI_INDEX_INPUT].address;
            size = (uint64_t)proc[DRPAI_INDEX_INPUT].size;
            if(0 != drpai_flush_dcache_input_area(addr, size))
            {
                result = -EFAULT;
                goto end;
            }
            /* Change input address to value specified by user app. */
            addr = priv->bank | (uint64_t)proc[DRPAI_INDEX_DRP_PARAM].address;
            if(0 != drp_param_change32(addr, 0, proc[DRPAI_INDEX_INPUT].address))
            {
                result = -EFAULT;
                goto end;
            }
        }
        
        /* drp_desc.bin */
        addr = priv->bank | (uint64_t)proc[DRPAI_INDEX_DRP_DESC].address;
        p_drp_desc = phys_to_virt(addr);
        if (p_drp_desc == 0)
        {
            result = -EFAULT;
            goto end;
        }
        /* Changed link descriptor of drp_desc.bin */
        if (0 != (proc[DRPAI_INDEX_DRP_DESC].size & 0x0F))
        {
            result = -EINVAL;
            goto end;
        }
        iowrite8(0x08, p_drp_desc + proc[DRPAI_INDEX_DRP_DESC].size - 13);
        __flush_dcache_area((void *)(p_drp_desc + proc[DRPAI_INDEX_DRP_DESC].size - 13), 1);

        /* aimac_desc.bin */
        addr = priv->bank | (uint64_t)proc[DRPAI_INDEX_AIMAC_DESC].address;
        p_aimac_desc = phys_to_virt(addr);
        if (p_aimac_desc == 0)
        {
            result = -EFAULT;
            goto end;
        }
        /* Changed link descriptor of drp_desc.bin */
        if (0 != (proc[DRPAI_INDEX_AIMAC_DESC].size & 0x0F))
        {
            result = -EINVAL;
            goto end;
        }
        iowrite8(0x08, p_aimac_desc + proc[DRPAI_INDEX_AIMAC_DESC].size - 13);
        __flush_dcache_area((void *)(p_aimac_desc + proc[DRPAI_INDEX_AIMAC_DESC].size - 13), 1);

        spin_lock_irqsave(&priv->lock, flags);
        /* Init drpai_status.err */
        priv->drpai_status.err    = DRPAI_ERRINFO_SUCCESS;
        /* IDLE -> RUN */
        priv->drpai_status.status = DRPAI_STATUS_RUN;
        priv->irq_flag = IRQ_CHECK_ENABLE;
        spin_unlock_irqrestore(&priv->lock, flags);

        DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);

        /* Kick */
        (void)R_DRPAI_DRP_Start(drp_base_addr[0], 0, proc[DRPAI_INDEX_DRP_DESC].address);
        (void)R_DRPAI_AIMAC_Start(aimac_base_address[0], 0, proc[DRPAI_INDEX_AIMAC_DESC].address, &priv->lock);
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    }
    else if (DRPAI_EXE_DRP == exe_mode)
    {
        odif_intcnto.ch0 = 0;
        odif_intcnto.ch1 = 0;
        odif_intcnto.ch2 = 0;
        odif_intcnto.ch3 = 0;

        DRPAI_DEBUG_PRINT("DRP exe mode:%d\n", exe_mode);
        if (copy_from_user(&proc_drp[0], (void __user *)arg, sizeof(proc_drp)))
        {
            result = -EFAULT;
            goto end;
        }
        /* Check Argument */
        for (i = 0; i < (seq.num * 2); i++)
        {
            if (0 != (proc_drp[i].address & DRPAI_64BYTE_ALIGN))
            {
                result = -EINVAL;
                goto end;
            }
        }
        for (i = 0; i < seq.num; i++)
        {
            /* DRPcfg address and size settings */
            *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 4) = proc_drp[i * 2].address;
            *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 8) = proc_drp[i * 2].size;

            /* DRP param address and size settings */
            *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 36) = proc_drp[i * 2 + 1].address;
            *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 40) = proc_drp[i * 2 + 1].size;

            /* Link descriptor settings */
            if (i < (seq.num - 1))
            {
                /* LV enable */
                *(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 67) = 0x09;

                /* Link pointer settings */
                if (DRPAI_DRP_NOLOAD == proc_drp[(i + 1) * 2].address)
                {
                    *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 68)
                    = p_dmabuf_phyaddr + (DRPAI_SGL_DRP_DESC_SIZE * (i + 1)) + DRPAI_DESC_CMD_SIZE;
                }
                else
                {
                    *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 68)
                    = p_dmabuf_phyaddr + (DRPAI_SGL_DRP_DESC_SIZE * (i + 1));
                }
            }
            else
            {
                /* LV disable */
                *(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 67) = 0x08;
            }
        }
        __flush_dcache_area(p_dmabuf_vaddr, DRPAI_CMA_SIZE);

        /* DRPcfg load skip */
        if (DRPAI_DRP_NOLOAD == proc_drp[0].address)
        {
            p_dmabuf_phyaddr = p_dmabuf_phyaddr + DRPAI_DESC_CMD_SIZE;
        }

        spin_lock_irqsave(&priv->lock, flags);
        /* Init drpai_status.err */
        priv->drpai_status.err = DRPAI_ERRINFO_SUCCESS;
        /* IDLE -> RUN */
        priv->drpai_status.status = DRPAI_STATUS_RUN;
        priv->irq_flag = IRQ_CHECK_ENABLE;
        spin_unlock_irqrestore(&priv->lock, flags);
        DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);

        /* Kick */
        (void)R_DRPAI_DRP_Start(drp_base_addr[0], 0, p_dmabuf_phyaddr);
        (void)R_DRPAI_AIMAC_Start(aimac_base_address[0], 0, p_dmabuf_phyaddr + (DRPAI_SGL_DRP_DESC_SIZE * DRPAI_SEQ_NUM), &priv->lock);
    }
    else
    {
        // do nothing
    }
/* DRP single operation */
#endif /* CONFIG_ARCH_R9A07G054 */

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ETIMEDOUT != result)
    {
        up(&priv->sem);
    }
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl_reset(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    struct drpai_priv *priv = drpai_priv;
#ifdef DRPAI_DRV_DEBUG
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
#endif
    unsigned long flags;
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    void (*finish_callback)(int);
#endif /* CONFIG_ARCH_R9A07G054 */

    DRPAI_DEBUG_PRINT("start.\n");

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    /* ISP */
    spin_lock_irqsave(&priv->lock, flags);
    finish_callback = drpai_priv->isp_finish_loc;
    spin_unlock_irqrestore(&priv->lock, flags);
    /* ISP */
#endif /* CONFIG_ARCH_R9A07G054 */

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
    {
        result = -ETIMEDOUT;
        goto end;
    }
    DRPAI_DEBUG_PRINT(" \n");
    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    if(R_DRPAI_SUCCESS != drpai_reset_device(0))
    {
        result = -EIO;
        goto end;
    }
    drpai_init_device(0);

    /* Update internal state */
    spin_lock_irqsave(&priv->lock, flags);
    priv->drpai_status.err    = DRPAI_ERRINFO_RESET;
    priv->drpai_status.status = DRPAI_STATUS_IDLE;
    priv->irq_flag            = IRQ_CHECK_DISABLE;

    /* Wake up the process */
    wake_up_interruptible( &drpai_waitq );
    spin_unlock_irqrestore(&priv->lock, flags);

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    /* ISP */
    /* For reset ISP call back*/    
    if(NULL != finish_callback)
    {
        /* ERROR No.: ERESTART*/
        (*finish_callback)(-ERESTART);
    }
    spin_lock_irqsave(&priv->lock, flags);    
    drpai_priv->isp_finish_loc = NULL;
    spin_unlock_irqrestore(&priv->lock, flags);
    /* For reset ISP call back*/
    /* ISP */
#endif /* CONFIG_ARCH_R9A07G054 */

    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);

    result = 0;

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ETIMEDOUT != result)
    {
        up(&priv->sem);
    }
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl_get_status(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    drpai_status_t local_drpai_status;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
    {
        result = -ETIMEDOUT;
        goto end;
    }
    DRPAI_DEBUG_PRINT(" \n");
    /* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }

    /* Get the internal state of DRP-AI */
    spin_lock_irqsave(&priv->lock, flags);
    (void)R_DRPAI_Status(drp_base_addr[0], aimac_base_address[0], 0, &priv->drpai_status);

    /* Copy arguments from kernel space to user space */
    local_drpai_status = priv->drpai_status;
    spin_unlock_irqrestore(&priv->lock, flags);
    if (copy_to_user((void __user *)arg, &local_drpai_status, sizeof(drpai_status_t)))
    {
        result = -EFAULT;
        goto end;
    }

    /* Check status */
    if (DRPAI_STATUS_RUN == local_drpai_status.status)
    {
        result = -EBUSY;
        goto end;
    }

    /* Check DRP-AI H/W error */
    if ((DRPAI_ERRINFO_DRP_ERR == local_drpai_status.err) || (DRPAI_ERRINFO_AIMAC_ERR == local_drpai_status.err))
    {
        result = -EIO;
        goto end;
    }

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ETIMEDOUT != result)
    {
        up(&priv->sem);
    }
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl_reg_dump(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_timeout(&rw_sem, MAX_SEM_TIMEOUT)))
    {
        result = -ETIMEDOUT;
        goto end;
    }
    DRPAI_DEBUG_PRINT(" \n");
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    /* Check of writing and reading completion of DRP-AI obj file */
    if (DRPAI_STATUS_IDLE_RW != drpai_rw_status->rw_status)
    {
        result = -EACCES;
        goto end;
    }

    /* Initialization of register dump processing variables */
    drpai_rw_status->rw_status              = DRPAI_STATUS_DUMP_REG;
    drpai_rw_status->read_count             = 0;
    drpai_rw_status->drp_reg_offset_count   = 0;
    drpai_rw_status->aimac_reg_offset_count = 0;

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ETIMEDOUT != result)
    {
        up(&rw_sem);
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl_assign_param(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    drpai_assign_param_t drpai_assign_param_buf;
    struct drpai_priv *priv = drpai_priv;
    char *vbuf;
    uint64_t addr, size;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_trylock(&rw_sem)))
    {
        result = -ERESTART;
        goto end;
    }
	/* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);
    /* Check status */
    if (DRPAI_STATUS_IDLE_RW != drpai_rw_status->rw_status)
    {
        result = -EACCES;
        goto end;
    }
    if(drpai_rw_status->param_info)
    {
        result = -EFAULT;
        goto end;
    }

    /* Copy arguments from user space to kernel space */
    if (copy_from_user(&drpai_assign_param_buf, (void __user *)arg, sizeof(drpai_assign_param_t)))
    {
        result = -EFAULT;
        goto end;
    }
    if((0 == drpai_assign_param_buf.info_size) || (0 == drpai_assign_param_buf.obj.size))
    {
        result = -EINVAL;
        goto end;
    }
    addr = priv->bank | (uint64_t)drpai_assign_param_buf.obj.address;
    size = (uint64_t)drpai_assign_param_buf.obj.size;
    if ((drpai_region_base_addr > addr) || 
        ((drpai_region_base_addr + drpai_region_size) <= (addr + size)))
    {
        result = -EINVAL;
        goto end;
    }
    
    /* Allocate memory for *_param_info.txt */
    vbuf = vmalloc(drpai_assign_param_buf.info_size + 1);
    if(!vbuf){
        result = -EFAULT;
        goto end;
    }
    vbuf[drpai_assign_param_buf.info_size] = '\0';

    /* Initialization of read / write processing variables */
    drpai_rw_status->rw_status    = DRPAI_STATUS_ASSIGN_PARAM;
    drpai_rw_status->write_count  = 0;
    drpai_rw_status->assign_param = drpai_assign_param_buf;
    drpai_rw_status->param_info   = vbuf;

    DRPAI_DEBUG_WAIT();
end:
    if(-ERESTART != result)
    {
        up(&rw_sem);
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

/* Note: This function change line variable. so if you use, check your variables address */
static char* get_param_attr(char *line, char *attr, unsigned long *rvalue, char **str)
{
    char *result;
    char *ptr_stmp, *ptr_etmp;

    DRPAI_DEBUG_PRINT("start.\n");

    if((NULL == line) || (NULL == attr))
    {
        result = NULL;
        goto end;
    }

    ptr_stmp = strstr(line, attr);
    if(NULL == ptr_stmp)
    {
        result = NULL;
        goto end;
    }
    ptr_stmp += strlen(attr);
    ptr_etmp = strstr(ptr_stmp, ",");
    if(NULL == ptr_etmp)
    {
        result = NULL;
        goto end;
    }
    *ptr_etmp = '\0';
    if(NULL != rvalue)
    {
        if(0 != kstrtoul(ptr_stmp, 10, rvalue))
        {
            result = NULL;
            goto end;
        }
    }
    if(NULL != str)
    {
        *str = ptr_stmp;
    }
    result = ptr_etmp + 1;

end:
    DRPAI_DEBUG_PRINT("end.\n");
    return result;
}

static int8_t drpai_get_sys_bank(uint64_t *bank)
{
    int8_t result = 0;
    void __iomem *base = NULL;
    uint32_t reg_val;

    DRPAI_DEBUG_PRINT("start.\n");

    base = ioremap(sysctrl_region_base_addr, SYS_SIZE);
    if (!base) {
        result = -1;
        goto end;
    }
    reg_val = ioread32(base + SYS_DRP_BANK);
    *bank = ((uint64_t)reg_val & SYS_MASK_DRP) << SYS_SHIFT;
    DRPAI_DEBUG_PRINT("SYS_DRP_BANK = 0x%08X\n", reg_val);
    DRPAI_DEBUG_PRINT("bank = 0x%016llX\n", *bank);
    goto end;
end:
    if(base)
    {
        iounmap(base);
    }
    DRPAI_DEBUG_PRINT("end.\n");
    return result;
}

static int8_t drp_param_change16(uint64_t base, uint64_t offset, uint16_t value)
{
    int8_t result = 0;
    volatile void *virt_addr = 0;

    DRPAI_DEBUG_PRINT("start.\n");

    virt_addr = phys_to_virt(base + offset);
    if (0 == virt_addr)
    {
        result = -1;
        goto end;
    }
    iowrite16(value, (void *)virt_addr);
    __flush_dcache_area((void *)virt_addr, sizeof(value));
end:
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}
static int8_t drp_param_change32(uint64_t base, uint64_t offset, uint32_t value)
{
    int8_t result = 0;
    volatile void *virt_addr = 0;

    DRPAI_DEBUG_PRINT("start.\n");

    virt_addr = phys_to_virt(base + offset);
    if (0 == virt_addr)
    {
        result = -1;
        goto end;
    }
    iowrite32(value, (void *)virt_addr);
    __flush_dcache_area((void *)virt_addr, sizeof(value));
end:
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}
static int8_t drpai_flush_dcache_input_area(uint64_t addr, uint64_t size)
{
    int8_t result = 0;
    uint64_t flush_addr, flush_size;
    uint64_t input_saddr, input_eaddr, linux_mem_saddr, linux_mem_eaddr;
    volatile void *p_input = 0;

    DRPAI_DEBUG_PRINT("start.\n");

    input_saddr      = addr;
    input_eaddr      = addr + size - 1;
    linux_mem_saddr  = drpai_linux_mem_start;
    linux_mem_eaddr  = drpai_linux_mem_start + drpai_linux_mem_size - 1;
    if ((input_saddr >= linux_mem_saddr) && 
        (input_eaddr <= linux_mem_eaddr))
    {
        flush_addr = addr;
        flush_size = size;
    }
    else if ((input_saddr >= linux_mem_saddr) &&
             (input_saddr <= linux_mem_eaddr) &&
             (input_eaddr >  linux_mem_eaddr))
    {
        flush_addr = addr;
        flush_size = (drpai_linux_mem_start + drpai_linux_mem_size) - addr;
    }
    else if((input_eaddr >= linux_mem_saddr) &&
            (input_eaddr <= linux_mem_eaddr) &&
            (input_saddr <  linux_mem_saddr))
    {
        flush_addr = drpai_linux_mem_start;
        flush_size = (addr + size) - drpai_linux_mem_start;
    }
    else
    {
        flush_addr = 0;
        flush_size = 0;
    }
    DRPAI_DEBUG_PRINT("flush_addr = 0x%016llX, flush_size = 0x%08X\n", flush_addr, flush_size);
    if (0 != flush_size)
    {
        /* Input data area cache flush */
        p_input = phys_to_virt(flush_addr);
        if (0 == p_input)
        {
            result = -1;
            goto end;
        }
        __flush_dcache_area((void *)p_input, flush_size);
    }
end:
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static long drpai_ioctl_prepost_crop(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    drpai_crop_t crop_param_buf;
    char *buf = NULL;
    uint32_t buf_len;
    char *ptr;
    char *prev_ptr;
    unsigned long offset_add0;
    unsigned long offset_add1;
    int mode = 0;
    uint64_t addr;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Check the internal state of DRP-AI */
    spin_lock_irqsave(&priv->lock, flags);
    if(DRPAI_STATUS_RUN == priv->drpai_status.status)
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EBUSY;
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    if(unlikely(down_trylock(&rw_sem)))
    {
        result = -ERESTART;
        goto end;
    }
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    /* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }
    if((NULL == drpai_rw_status->param_info) ||
       (DRPAI_STATUS_ASSIGN_PARAM == drpai_rw_status->rw_status) ||
       (DRPAI_STATUS_WRITE_PARAM  == drpai_rw_status->rw_status))
    {
        result = -EACCES;
        goto end;
    }
    /* Copy arguments from user space to kernel space */
    if (copy_from_user(&crop_param_buf, (void __user *)arg, sizeof(drpai_crop_t)))
    {
        result = -EFAULT;
        goto end;
    }
    /* Check if there is in drpai dedicated area */
    if((drpai_rw_status->assign_param.obj.address != crop_param_buf.obj.address) ||
       (drpai_rw_status->assign_param.obj.size != crop_param_buf.obj.size))
    {
        result = -EINVAL;
        goto end;
    }

    /* Search argument */
    ptr = drpai_rw_status->param_info;
    while(1)
    {
        /* End of info_buf */
        if(('\0' == *ptr) || (ptr >= drpai_rw_status->param_info + drpai_rw_status->assign_param.info_size))
        {
            result = -EFAULT;
            goto end;
        }

        /* Save current pointer */
        prev_ptr = ptr;

        /* Get 1 line */
        ptr = strchr(ptr, '\n');
        if(NULL == ptr)
        {
            ptr = drpai_rw_status->param_info + drpai_rw_status->assign_param.info_size;
        }
        else
        {
            ptr += 1;
        }
        if(ptr <= prev_ptr)
        {
            result = -EFAULT;
            goto end;
        }
        buf_len = ptr - prev_ptr;

        /* Copy only 1line to buffer */
        buf = vmalloc(buf_len + 1);
        if(NULL == buf)
        {
            result = -EFAULT;
            goto end;
        }
        strncpy(buf, prev_ptr, buf_len);
        buf[buf_len] = '\0';

        if(0 == mode)
        {
            /* Check if there is DRP_LIB_NAME_CROP in this line */
            if(NULL != strstr(buf, DRP_LIB_NAME_CROP))
            {
                mode += 1;
                if(NULL == get_param_attr(buf, DRP_PARAM_ATTR_OFFSET_ADD, &offset_add0, NULL))
                {
                    result = -EFAULT;
                    goto end;
                }
            }
        }
        else if(1 == mode)
        {
            if(NULL != strstr(buf, DRP_PARAM_ATTR_OFFSET_ADD))
            {
                mode += 1;
                if(NULL == get_param_attr(buf, DRP_PARAM_ATTR_OFFSET_ADD, &offset_add1, NULL))
                {
                    result = -EFAULT;
                    goto end;
                }
                break;
            }
        }
        vfree(buf);
        buf = NULL;
    }

    DRPAI_DEBUG_PRINT("offset_add0=%d, offset_add1=%d\n", offset_add0, offset_add1);

    /* Change parameters of drp_param.bin to value specified by user app. */
    addr = priv->bank | (uint64_t)crop_param_buf.obj.address;
    if(0 != drp_param_change16(addr, (uint64_t)offset_add0 + DRP_PARAM_IMG_OWIDTH, crop_param_buf.img_owidth))
    {
        result = -EFAULT;
        goto end;
    }
    if(0 != drp_param_change16(addr, (uint64_t)offset_add0 + DRP_PARAM_IMG_OHEIGHT, crop_param_buf.img_oheight))
    {
        result = -EFAULT;
        goto end;
    }
    if(0 != drp_param_change16(addr, (uint64_t)offset_add0 + DRP_PARAM_CROP_POS_X, crop_param_buf.pos_x))
    {
        result = -EFAULT;
        goto end;
    }
    if(0 != drp_param_change16(addr, (uint64_t)offset_add0 + DRP_PARAM_CROP_POS_Y, crop_param_buf.pos_y))
    {
        result = -EFAULT;
        goto end;
    }
    if(0 != drp_param_change16(addr, (uint64_t)offset_add1 + DRP_PARAM_IMG_IWIDTH, crop_param_buf.img_owidth))
    {
        result = -EFAULT;
        goto end;
    }
    if(0 != drp_param_change16(addr, (uint64_t)offset_add1 + DRP_PARAM_IMG_IHEIGHT, crop_param_buf.img_oheight))
    {
        result = -EFAULT;
        goto end;
    }

    goto end;
end:
    DRPAI_DEBUG_WAIT();

    if(NULL != buf)
    {
        vfree(buf);
    }
    if((-EBUSY != result) || (-ERESTART != result))
    {
        up(&rw_sem);
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}
static long drpai_ioctl_prepost_inaddr(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    struct drpai_priv *priv = drpai_priv;
    struct drpai_rw_status *drpai_rw_status = filp->private_data;
    drpai_inout_t inout_param_buf;
    char *buf = NULL;
    uint32_t buf_len;
    char *ptr;
    char *prev_ptr;
    unsigned long flags;
    unsigned long offset_add;
    uint64_t addr, size;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Check the internal state of DRP-AI */
    spin_lock_irqsave(&priv->lock, flags);
    if(DRPAI_STATUS_RUN == priv->drpai_status.status)
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EBUSY;
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    if(unlikely(down_trylock(&rw_sem)))
    {
        result = -ERESTART;
        goto end;
    }
    DRPAI_DEBUG_PRINT("status_rw1:%d\n", drpai_rw_status->rw_status);

    /* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }
    if((NULL == drpai_rw_status->param_info) ||
       (DRPAI_STATUS_ASSIGN_PARAM == drpai_rw_status->rw_status) ||
       (DRPAI_STATUS_WRITE_PARAM  == drpai_rw_status->rw_status))
    {
        result = -EACCES;
        goto end;
    }
    /* Copy arguments from user space to kernel space */
    if (copy_from_user(&inout_param_buf, (void __user *)arg, sizeof(drpai_inout_t)))
    {
        result = -EFAULT;
        goto end;
    }
    /* Check if there is in drpai dedicated area */
    if((drpai_rw_status->assign_param.obj.address != inout_param_buf.obj.address) ||
       (drpai_rw_status->assign_param.obj.size != inout_param_buf.obj.size))
    {
        result = -EINVAL;
        goto end;
    }

    /* Search argument */
    ptr = drpai_rw_status->param_info;
    while(1)
    {
        /* End of info_buf */
        if(('\0' == *ptr) || (ptr >= drpai_rw_status->param_info + drpai_rw_status->assign_param.info_size))
        {
            result = -EFAULT;
            goto end;
        }

        /* Save current pointer */
        prev_ptr = ptr;

        /* Get 1 line */
        ptr = strchr(ptr, '\n');
        if(NULL == ptr)
        {
            ptr = drpai_rw_status->param_info + drpai_rw_status->assign_param.info_size;
        }
        else
        {
            ptr += 1;
        }
        if(ptr <= prev_ptr)
        {
            result = -EFAULT;
            goto end;
        }
        buf_len = ptr - prev_ptr;

        /* Copy only 1line to buffer */
        buf = vmalloc(buf_len + 1);
        if(NULL == buf)
        {
            result = -EFAULT;
            goto end;
        }
        strncpy(buf, prev_ptr, buf_len);
        buf[buf_len] = '\0';

        /* Check if there is DRP_PARAM_ATTR_PROP_INPUT in this line */
        if(NULL != strstr(buf, DRP_PARAM_ATTR_PROP_INPUT))
        {
            if(NULL != strstr(buf, inout_param_buf.name))
            {
                if(NULL == get_param_attr(buf, DRP_PARAM_ATTR_OFFSET_ADD, &offset_add, NULL))
                {
                    result = -EFAULT;
                    goto end;
                }
                break;
            }
        }
        vfree(buf);
        buf = NULL;
    }

    DRPAI_DEBUG_PRINT("offset_add=%d\n", offset_add);

    /* Check if input is in linux memory region */
    addr = priv->bank | (uint64_t)inout_param_buf.data.address;
    size = (uint64_t)inout_param_buf.data.size;
    if(0 != drpai_flush_dcache_input_area(addr, size))
    {
        result = -EFAULT;
        goto end;
    }
    /* Change parameters of drp_param.bin to value specified by user app. */
    addr = priv->bank | (uint64_t)inout_param_buf.obj.address;
    if(0 != drp_param_change32(addr, (uint64_t)offset_add + DRP_PARAM_raddr, inout_param_buf.data.address))
    {
        result = -EFAULT;
        goto end;
    }
    atomic_set(&drpai_rw_status->inout_flag, 1);

    goto end;
end:
    DRPAI_DEBUG_WAIT();

    if(NULL != buf)
    {
        vfree(buf);
    }
    if((-EBUSY != result) || (-ERESTART != result))
    {
        up(&rw_sem);
    }
    DRPAI_DEBUG_PRINT("status_rw2:%d\n", drpai_rw_status->rw_status);
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
static long drpai_ioctl_set_seq(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;

    DRPAI_DEBUG_PRINT(" start.\n");
    DRPAI_DEBUG_PRINT(" status1:%d\n", priv->drpai_status.status);

    /* Check the internal state of DRP-AI */
    spin_lock_irqsave(&priv->lock, flags);
    if (DRPAI_STATUS_RUN == priv->drpai_status.status)
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EBUSY;
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    /* Copy arguments from user space to kernel space */
    if (copy_from_user(&seq, (void __user *)arg, sizeof(drpai_seq_t)))
    {
        result = -EFAULT;
        goto end;
    }

    /* Check Argument DRP Simgle */
    if (DRPAI_SEQ_NUM < seq.num)
    {
        result = -EINVAL;
        goto end;
    }

    if (DRPAI_EXE_DRP == seq.order[0])
    {
        exe_mode = DRPAI_EXE_DRP;
    }
    else if (DRPAI_EXE_AI == seq.order[0])
    {
        exe_mode = DRPAI_EXE_AI;
    }
    else
    {
        result = -EINVAL;
        goto end;
    }

    DRPAI_DEBUG_PRINT(" DRP exe mode:%d\n", exe_mode);

    DRPAI_DEBUG_PRINT(" status2:%d\n", priv->drpai_status.status);
end:
    DRPAI_DEBUG_PRINT(" end.\n");

    return result;
}
/* DRP single operation */
#endif /* CONFIG_ARCH_R9A07G054 */

static long drpai_ioctl_get_drpai_area(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    drpai_data_t local_drpai_data;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Check NULL */
    if (0 == arg)
    {
        result = -EINVAL;
        goto end;
    }

    local_drpai_data.address = drpai_region_base_addr;
    local_drpai_data.size    = drpai_region_size;

    if (copy_to_user((void __user *)arg, &local_drpai_data, sizeof(drpai_data_t)))
    {
        result = -EFAULT;
        goto end;
    }

    goto end;
end:
    DRPAI_DEBUG_PRINT("end.\n");
    return result;
}

static int drpai_drp_cpg_init(void)
{
    int result;
    struct drpai_priv *priv = drpai_priv;
    int r_data;
    int32_t i = 0;
    bool is_stop = false;

    DRPAI_DEBUG_PRINT("start.\n");

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    /* Access clock interface */
    clk_prepare_enable(priv->clk_int);
    clk_prepare_enable(priv->clk_aclk_drp);
    clk_prepare_enable(priv->clk_mclk);
    clk_prepare_enable(priv->clk_dclkin);
    clk_prepare_enable(priv->clk_aclk);
#endif /* CONFIG_ARCH_R9A07G054 */

    r_data = reset_control_status(priv->rstc);
    DRPAI_DEBUG_PRINT("CPG reset_control_status before %d \n", r_data);
    
    /* Access reset controller interface */
    reset_control_reset(priv->rstc);

    /* Check reset status */
    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        udelay(1);
        i++;
        r_data = reset_control_status(priv->rstc);
        DRPAI_DEBUG_PRINT("CPG reset_control_status %d \n", r_data);
        if(CPG_RESET_SUCCESS == r_data)
        {
            is_stop = true;
            break;
        }
    }

    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        usleep_range(100, 200);
        i++;
        r_data = reset_control_status(priv->rstc);
        DRPAI_DEBUG_PRINT("CPG reset_control_status %d \n", r_data);
        if(CPG_RESET_SUCCESS == r_data)
        {
            is_stop = true;
            break;
        }
    }

    if(true == is_stop)
    {
        result =  R_DRPAI_SUCCESS;
    }
    else
    {
        result = R_DRPAI_ERR_RESET;
        DRPAI_DEBUG_PRINT("CPG Reset failed. Reset Control Status: %d\n", r_data);
    }

    goto end;
end:
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static int drpai_open_process(void)
{
    int ret;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;
    uint64_t bank;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Initialize CPG */
    if(R_DRPAI_SUCCESS != drpai_drp_cpg_init())
    {
        ret = -EIO;
        goto end;
    }

    /* Initialize DRP-AI */
    drpai_init_device(0);

    /* Reset DRP-AI */
    if(R_DRPAI_SUCCESS != drpai_reset_device(0))
    {
        ret = -EIO;
        goto end;
    }

    /* Initialize DRP-AI */
    drpai_init_device(0);

    /* Get sys register value */
    if(0 != drpai_get_sys_bank(&bank))
    {
        ret = -EFAULT;
        goto end;
    }

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    if(R_DRPAI_SUCCESS != drpai_drp_config_init())
    {
        ret = -ENOMEM;
        goto end;
    }
#endif /* CONFIG_ARCH_R9A07G054 */

    /* INIT -> IDLE */
    spin_lock_irqsave(&priv->lock, flags);
    priv->drpai_status.status = DRPAI_STATUS_IDLE;
    priv->bank = bank;
/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    exe_mode = DRPAI_EXE_AI;
#endif /* CONFIG_ARCH_R9A07G054 */
    spin_unlock_irqrestore(&priv->lock, flags);

    ret = R_DRPAI_SUCCESS;
    goto end;
end:
    DRPAI_DEBUG_PRINT("end.\n");
    return ret;
}

static int drpai_close_process(void)
{
    int ret;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;

    DRPAI_DEBUG_PRINT("start.\n");

    if(R_DRPAI_SUCCESS != drpai_reset_device(0))
    {
        ret = -EIO;
        goto end;
    }

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
    //CPG clock disable
    DRPAI_DEBUG_PRINT("CPG clock disable\n");
    clk_disable_unprepare(priv->clk_int);
    clk_disable_unprepare(priv->clk_aclk_drp);
    clk_disable_unprepare(priv->clk_mclk);
    clk_disable_unprepare(priv->clk_dclkin);
    clk_disable_unprepare(priv->clk_aclk);  

    drpai_drp_config_uninit();
#endif /* CONFIG_ARCH_R9A07G054 */

    /* IDLE -> INIT */
    /* RUN  -> INIT */
    spin_lock_irqsave(&priv->lock, flags);
    priv->drpai_status.status = DRPAI_STATUS_INIT;
    priv->drpai_status.err    = DRPAI_ERRINFO_SUCCESS;
    spin_unlock_irqrestore(&priv->lock, flags);

    ret = R_DRPAI_SUCCESS;
    goto end;
end:
    DRPAI_DEBUG_PRINT("end.\n");
    return ret;
}

/* V2L conditional compilation */
#ifdef CONFIG_ARCH_R9A07G054
/* ISP */
static int drpai_drp_config_init(void)
{
    int result;
    struct drpai_priv *priv = drpai_priv;
    struct device *dev = &priv->pdev->dev;
    int i;

    DRPAI_DEBUG_PRINT("start.\n");

    /* Get driver workspace from Linux CMA */
    p_dmabuf_vaddr = dma_alloc_coherent(dev, DRPAI_CMA_SIZE, &p_dmabuf_phyaddr, GFP_DMA);
    if (NULL == p_dmabuf_vaddr)
    {
        /* Error -ENOMEM */
        result = -1;
        goto end;
    }
    DRPAI_DEBUG_PRINT("dmabuf:0x%08X, dmaphys:0x%08X\n", p_dmabuf_vaddr, p_dmabuf_phyaddr);

    /* 64bytes alignment adjustment */
    if (0 != (p_dmabuf_phyaddr & DRPAI_64BYTE_ALIGN))
    {
        p_dmabuf_vaddr = p_dmabuf_vaddr + (0x40 - (p_dmabuf_phyaddr & DRPAI_64BYTE_ALIGN));
        p_dmabuf_phyaddr = p_dmabuf_phyaddr + (0x40 - (p_dmabuf_phyaddr & DRPAI_64BYTE_ALIGN));
    }

/* DRP single operation */
    /* Deploy drp_single_desc */
    for (i = 0; i < DRPAI_SEQ_NUM; i++)
    {
        memcpy(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i), &drp_single_desc_bin[0], sizeof(drp_single_desc_bin));
    }
/* DRP single operation */

    __flush_dcache_area(p_dmabuf_vaddr, DRPAI_CMA_SIZE);

    result = R_DRPAI_SUCCESS;

    goto end;
end:
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

static void drpai_drp_config_uninit(void)
{
    struct drpai_priv *priv = drpai_priv;
    struct device *dev = &priv->pdev->dev;

    DRPAI_DEBUG_PRINT("start.\n");

    dma_free_coherent(dev, DRPAI_CMA_SIZE, p_dmabuf_vaddr, p_dmabuf_phyaddr);
    DRPAI_DEBUG_PRINT("end.\n");
}

int drpai_open_k(void)
{
    int result = 0;
    struct drpai_priv *priv = drpai_priv;

    DRPAI_DEBUG_PRINT("start.\n");
    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
    {
        result = -ETIMEDOUT;
        DRPAI_DEBUG_PRINT("ETIMEDOUT.\n");
        goto end;
    }  

    if(likely(1 == refcount_read(&priv->count)))
    {
        result = drpai_open_process();
        if(R_DRPAI_SUCCESS != result)
        {
            goto end;
        }
    }

    /* Increment reference count */
    refcount_inc(&priv->count);

    DRPAI_DEBUG_PRINT("status2:%d\n", priv->drpai_status.status);

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ETIMEDOUT != result)
    {
        /* Return semaphore when no ETIMEOUT */
        up(&priv->sem);
    }

    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

int drpai_close_k(void)
{
    int result = 0;
    struct drpai_priv *priv = drpai_priv;
    unsigned long flags;

    DRPAI_DEBUG_PRINT("start.\n");
    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
    {
        result = -ETIMEDOUT;
        DRPAI_DEBUG_PRINT("ETIMEOUT.\n");
        goto end;
    } 

    /* Check status */
    spin_lock_irqsave(&priv->lock, flags);
    if (DRPAI_STATUS_INIT == priv->drpai_status.status)
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EACCES;        
        DRPAI_DEBUG_PRINT("EACCES.\n");
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    if(2 == refcount_read(&priv->count))
    {
        result = drpai_close_process();
        if(R_DRPAI_SUCCESS != result)
        {
            goto end;
        }
    }

    /* Decrement referenece count*/
    refcount_dec(&priv->count);

    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ETIMEDOUT != result)
    {
        /* Return semaphore when no ETIMEOUT */
        up(&priv->sem);
    }
    DRPAI_DEBUG_PRINT("end.\n");

    return result;
}

int drpai_start_k(drpai_data_t *arg, void (*isp_finish)(int result))
{
    int result = 0;
    struct drpai_priv *priv = drpai_priv;
    drpai_data_t *proc_k;
    unsigned long flags;
    int i;

    DRPAI_DEBUG_PRINT("start.\n");

    if(unlikely(down_timeout(&priv->sem, MAX_SEM_TIMEOUT))) 
    {
        result = -ETIMEDOUT;
        DRPAI_DEBUG_PRINT("ETIMEOUT.\n");
        goto end;
    }

    DRPAI_DEBUG_PRINT("status1:   %d\n", priv->drpai_status.status);

    /* Check H/W Error */
    spin_lock_irqsave(&priv->lock, flags);
    if ((DRPAI_ERRINFO_DRP_ERR == priv->drpai_status.err) || (DRPAI_ERRINFO_AIMAC_ERR == priv->drpai_status.err))
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EIO;   
        DRPAI_DEBUG_PRINT("EIO.\n");
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    /* Check status */
    spin_lock_irqsave(&priv->lock, flags);
    if (DRPAI_STATUS_RUN == priv->drpai_status.status)
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EBUSY;   
        DRPAI_DEBUG_PRINT("EBUSY.\n");
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    /* Check status */
    spin_lock_irqsave(&priv->lock, flags);
    if (DRPAI_STATUS_INIT == priv->drpai_status.status)
    {
        spin_unlock_irqrestore(&priv->lock, flags);
        result = -EACCES;   
        DRPAI_DEBUG_PRINT("EACCES.\n");
        goto end;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    /* Check NULL */
    if (NULL == isp_finish)
    {
        result = -EINVAL; 
        DRPAI_DEBUG_PRINT("EINVAL NULL function pointer.\n");
        goto end;
    }
    /* Referring the call back function info from ISP Lib */
    priv->isp_finish_loc = isp_finish;  

    /* Check NULL */
    if (NULL == arg)
    {
        result = -EINVAL; 
        DRPAI_DEBUG_PRINT("EINVAL NULL argument pointer.\n");
        goto end;
    }
    /* Referring the obj file info from ISP Lib */
    proc_k = arg;

    spin_lock_irqsave(&priv->lock, flags);
    odif_intcnto.ch0 = 0;
    odif_intcnto.ch1 = 0;
    odif_intcnto.ch2 = 0;
    odif_intcnto.ch3 = 0;
    spin_unlock_irqrestore(&priv->lock, flags);

    DRPAI_DEBUG_PRINT("ODIF_INTCNTO0 : 0x%08X\n", odif_intcnto.ch0);
    DRPAI_DEBUG_PRINT("ODIF_INTCNTO1 : 0x%08X\n", odif_intcnto.ch1);
    DRPAI_DEBUG_PRINT("ODIF_INTCNTO2 : 0x%08X\n", odif_intcnto.ch2);
    DRPAI_DEBUG_PRINT("ODIF_INTCNTO3 : 0x%08X\n", odif_intcnto.ch3);

    /* Check Argument 64-byte*/
    for (i = 0; i < 2; i++)
    {
        if (0 != (proc_k[i].address & DRPAI_64BYTE_ALIGN))
        {
            result = -EINVAL; 
            DRPAI_DEBUG_PRINT("EINVAL argument. Not 64-byte aligned.\n");
            goto end;
        }
    }

    for (i = 0; i < 1; i++)
    {
        /* DRPcfg address and size settings */
        *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 4) = proc_k[i * 2].address;
        *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 8) = proc_k[i * 2].size;

        DRPAI_DEBUG_PRINT("cfg_address:0x%08X, cfg_size:0x%08X\n", proc_k[i * 2].address, proc_k[i * 2].size);

        /* DRP param address and size settings */
        *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 36) = proc_k[i * 2 + 1].address;
        *(uint32_t*)(p_dmabuf_vaddr + (DRPAI_SGL_DRP_DESC_SIZE * i) + 40) = proc_k[i * 2 + 1].size;

        DRPAI_DEBUG_PRINT("parm_address:0x%08X, parm_size:0x%08X\n", proc_k[i * 2 + 1].address, proc_k[i * 2 + 1].size);

    }
    __flush_dcache_area(p_dmabuf_vaddr, DRPAI_CMA_SIZE);

    spin_lock_irqsave(&priv->lock, flags);
    /* Init drpai_status.err */
    priv->drpai_status.err = DRPAI_ERRINFO_SUCCESS;

    /* IDLE -> RUN */
    priv->drpai_status.status = DRPAI_STATUS_RUN;
    DRPAI_DEBUG_PRINT("status2:   %d\n", priv->drpai_status.status);
    spin_unlock_irqrestore(&priv->lock, flags);

    /* Kick */
    (void)R_DRPAI_DRP_Start(drp_base_addr[0], 0, p_dmabuf_phyaddr);
    (void)R_DRPAI_AIMAC_Start(aimac_base_address[0], 0, p_dmabuf_phyaddr + (DRPAI_SGL_DRP_DESC_SIZE * DRPAI_SEQ_NUM), &priv->lock);

    DRPAI_DEBUG_WAIT();
    goto end;
end:
    if(-ETIMEDOUT != result)
    {
        /* Return semaphore when no ETIMEOUT */
        up(&priv->sem);
    }
    DRPAI_DEBUG_PRINT("end.\n");
    
    return result;
}

/* Public to Kernel space */
EXPORT_SYMBOL(drpai_open_k);
EXPORT_SYMBOL(drpai_close_k);
EXPORT_SYMBOL(drpai_start_k);
/* ISP */
#endif /* CONFIG_ARCH_R9A07G054 */

module_platform_driver(drpai_platform_driver);
MODULE_DEVICE_TABLE(of, drpai_match);

#if defined(CONFIG_ARCH_R9A09G011GBG) 
/* V2M conditional compilation */
MODULE_DESCRIPTION("RZ/V2M DRPAI driver");
#elif defined(CONFIG_ARCH_R9A09G055MA3GBG)
/* V2MA conditional compilation */
MODULE_DESCRIPTION("RZ/V2MA DRPAI driver");
#elif defined(CONFIG_ARCH_R9A07G054)
/* V2L conditional compilation */
MODULE_DESCRIPTION("RZ/V2L DRPAI driver");
#endif
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("GPL v2");

