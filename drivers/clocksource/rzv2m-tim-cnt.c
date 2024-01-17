/*
 * Driver for the Renesas RZ/V2M Timer unit(TIM)
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
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

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/counter/tim_rzv2m.h>
#include <linux/delay.h>

#define DRIVER_NAME "tim"

static const unsigned int minor_base = 0;
static const unsigned int minor_num  = 1;

/* latch minor number */
static int minor_n = 0;

static unsigned int hw_tim_device_num = 0;
static struct class *tim_class = NULL;
dev_t g_tim_dev = 0;


struct tim_priv {
    struct cdev             tim_cdev;
    struct platform_device  *pdev;
    void __iomem            *base;
    const char              *dev_name;
    struct clk              *sys_clk;
    struct clk		    *apb_clk;
    struct tim_ioctl_if     ioctl_inf;
    unsigned int            cmd_count;
    struct completion hw_tim_wait_done;
    bool                    isUsed;
};

//TIM register offfset
#define TMD     0x00
#define CMD     0x04
#define TMCD    0x08
#define INTCLR  0x0C

//CMD register field
#define MAX_TIMCYS      0xFFFFFFFF /* Max value of 32 bits register is ffffffff */

//TMCD register field
#define TMCD_CS(div)        (div << 4)
#define TMCD_INTMODE_LEVEL  BIT(2)
#define TMCD_INTMODE_PLUSE  (0 << 2)
#define TMCD_CE             BIT(1)
#define TMCD_CAE            BIT(0)
#define TMCD_MAXDIV_C0C1C2  (0x7)
#define TMCD_TIM_START      (TMCD_CE | TMCD_CAE | TMCD_INTMODE_LEVEL)
#define TMCD_TIM_STOP       (0x0)

//INTCLR register field
#define INTCLR_INTCLEAR     (0x1)

#define MAX_TIM_USEC (MAX_TIMCYS * (1UL << (TMCD_MAXDIV_C0C1C2+1)))
#define MAX_TIM_TIMEOUT (549760)

/******************************************************************************
Private global variables and functions
******************************************************************************/
static int rzv2m_tim_open(struct inode *inode, struct file *file);
static int rzv2m_tim_close(struct inode *inode, struct file *file);
static long rzv2m_tim_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int rzv2m_tim_start(struct tim_priv *priv);
static void rzv2m_tim_stop(struct tim_priv *priv);
static uint32_t rzv2m_tim_read_cnt(struct tim_priv *priv);

static int rzv2m_calc_cycle_div(struct tim_priv *priv);
static irqreturn_t irq_tim_interrupt(int irq, void *dev);

static struct file_operations hw_tim_cdev_fops =
{
    .open           = rzv2m_tim_open,
    .release        = rzv2m_tim_close,
    .unlocked_ioctl = rzv2m_tim_ioctl,
    .compat_ioctl   = rzv2m_tim_ioctl,
};

//Regster write wait time
/* RZ/V2M, V2MA timer inclock is fixed at 2MHz. */
/* 5 x inclock = 3us is enough for the register reflection time */
#define DELAY_3u	3


static void rzv2m_tim_write_delay(u32 value, volatile void * addr){

	writel(value, addr);
	udelay(DELAY_3u);
}

static int rzv2m_tim_start(struct tim_priv *priv)
{
    int ret = 0;
    unsigned long time_left;

    if( 0 != (readl( priv->base + TMCD ) & ( TMCD_CE | TMCD_CAE )) ){
        rzv2m_tim_write_delay(TMCD_TIM_STOP, priv->base + TMCD);
    }

    switch(priv->ioctl_inf.mode)
    {
    case IOCTL_START_FREERUN:

        printk("IOCTL_START_FREERUN mode\n");

        if( priv->ioctl_inf.clk_div > TMCD_MAXDIV_C0C1C2 )
        {
            return -EINVAL;
        }else{
            writel(MAX_TIMCYS, priv->base + CMD);
            rzv2m_tim_write_delay((TMCD_CS(priv->ioctl_inf.clk_div) | TMCD_TIM_START), priv->base + TMCD);
        }
	
        break;
    case IOCTL_START_INTERRUPT:

        printk("IOCTL_START_INTERRUPT mode\n");
        reinit_completion(&priv->hw_tim_wait_done);
        ret = rzv2m_calc_cycle_div(priv);
        if( ret < 0 )
            return ret;

        time_left = wait_for_completion_timeout(&priv->hw_tim_wait_done, MAX_TIM_TIMEOUT * HZ );
        if (!time_left)
            return -ETIMEDOUT;

        break;
    default:
        return -EINVAL;
        break;
    }
    return ret;
}

static void rzv2m_tim_stop(struct tim_priv *priv)
{
    rzv2m_tim_write_delay(TMCD_TIM_STOP, priv->base + TMCD);
    return;
}

static int rzv2m_calc_cycle_div(struct tim_priv *priv)
{
    unsigned long clk_mhz = ((clk_get_rate(priv->sys_clk)/1000000)); /* MHz */
    unsigned char div;
    unsigned long   max_cnt;
    unsigned long   usec = priv->ioctl_inf.tm_usec;
    unsigned int    tim_cmd_val,tim_tmcd_val;

    priv->ioctl_inf.clk_div = 0;

    if (clk_mhz == 0)
        return -EINVAL;
     
     if ( (usec == 0) || ((MAX_TIM_USEC/clk_mhz) < usec) ){
	return -EINVAL;
     }

    for (div = 0; div < TMCD_MAXDIV_C0C1C2; div++) {
        max_cnt = ((MAX_TIMCYS * (1UL << (div+1)))/ clk_mhz);
        tim_cmd_val = usec / ( (1UL << (div+1) ) / clk_mhz );

	if( max_cnt > usec )
            break;
    }

    priv->ioctl_inf.clk_div = div;
    priv->cmd_count = tim_cmd_val;

    tim_tmcd_val = TMCD_CS(priv->ioctl_inf.clk_div) | TMCD_TIM_START;

    writel(tim_cmd_val,  priv->base + CMD);

    rzv2m_tim_write_delay(tim_tmcd_val, priv->base + TMCD);


    return 0;
}

static uint32_t rzv2m_tim_read_cnt(struct tim_priv *priv)
{
    return readl( priv->base + TMD );
}

static int rzv2m_tim_open(struct inode *inode, struct file *file)
{
    struct tim_priv *priv;

    priv = container_of(inode->i_cdev, struct tim_priv, tim_cdev);
    if (priv  == NULL) {
        printk(KERN_ERR "container_of\n");
        return -EFAULT;
    }

    if( priv->isUsed )
        return -EBUSY;
    else
    {
        priv->isUsed = true; //set to used flag with corresponding channel.
        file->private_data = priv;
        //printk("%s:remap_addr[0x%x] \n",__func__,priv->base);
    }
    return 0;
}

static int rzv2m_tim_close(struct inode *inode, struct file *file)
{

    struct tim_priv *priv = file->private_data;

    priv->isUsed = false;
    return 0;
}

static long rzv2m_tim_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

    struct tim_priv *priv = filp->private_data;
    int ioctl_ret = 0;

    if( priv->isUsed )
    {
        switch (cmd) {
        case IOCTL_TIM_START:
            //printk("IOCTL_TIM_START\n");
            if (raw_copy_from_user(&priv->ioctl_inf, (void __user *)arg, sizeof(struct tim_ioctl_if))) {
                return -EFAULT;
            }
            //printk("%s:remap_addr[0x%x] \n",__func__,priv->base);
            ioctl_ret = rzv2m_tim_start(priv);

            break;

        case IOCTL_TIM_STOP:
            //printk("IOCTL_TIM_STOP\n");
            rzv2m_tim_stop(priv);
            break;

        case IOCTL_GET_VALUES:
            //printk("IOCTL_GET_VALUES\n");
            priv->ioctl_inf.cnt_val = rzv2m_tim_read_cnt(priv);

            if (raw_copy_to_user((void __user *)arg, &priv->ioctl_inf, sizeof(struct tim_ioctl_if))) {
                ioctl_ret = (-EFAULT);
            }
            break;

        default:
            printk(KERN_WARNING "unsupported command %d\n", cmd);
            ioctl_ret = (-EINVAL);
            break;
        }
    }
    else
    {
        ioctl_ret = -EBUSY;
    }
    return ioctl_ret;
}

static irqreturn_t irq_tim_interrupt(int irq, void *dev)
{
    
    struct tim_priv *priv = dev;
    switch(priv->ioctl_inf.mode)
    {
    case IOCTL_START_FREERUN:
        rzv2m_tim_write_delay( INTCLR_INTCLEAR, priv->base + INTCLR);
        break;
    case IOCTL_START_INTERRUPT:
        writel(TMCD_TIM_STOP, priv->base + TMCD);
        rzv2m_tim_write_delay( INTCLR_INTCLEAR, priv->base + INTCLR);
        complete(&priv->hw_tim_wait_done);
        break;
    default:
        break;
    }
    return IRQ_HANDLED;
}


/******************************************************************************
* Function Name : tim_dev_probe
* Description   :
* Arguments     :
* Return Value  :
******************************************************************************/
static int tim_dev_probe(struct platform_device *pdev)
{
    struct tim_priv *priv;
    struct resource *res;
    int irq,ret;
    int cdev_err = 0;
   
    hw_tim_device_num = MAJOR(g_tim_dev);

    priv = devm_kzalloc(&pdev->dev, sizeof(struct tim_priv), GFP_KERNEL);
    if (!priv) {
        dev_err(&pdev->dev, "cannot allocate private data\n");
        return -ENOMEM;
    }

    init_completion(&priv->hw_tim_wait_done);

    cdev_init(&priv->tim_cdev, &hw_tim_cdev_fops);

    priv->tim_cdev.owner = THIS_MODULE;

    cdev_err = cdev_add(&priv->tim_cdev,  MKDEV(hw_tim_device_num, minor_n), 1);
    if (cdev_err != 0) {
        printk(KERN_ERR  "cdev_add = %d\n", cdev_err);
        return -1;
    }


    /* make /sys/class/mydevice/mydevice*  */
    device_create(tim_class, NULL, MKDEV(hw_tim_device_num, minor_n), NULL, "hw_tim%d", minor_n);

    platform_set_drvdata(pdev, priv);
    priv->pdev = pdev;
    /* Convert TIM base address from physical to virtual */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "cannot get resources\n");
        return -EINVAL;
    }

    /*Enable clk of each timer channels*/
    priv->sys_clk = devm_clk_get(&pdev->dev, "timclk");
    if (IS_ERR(priv->sys_clk)) {
        dev_err(&pdev->dev, "failed to get system clock\n");
        return PTR_ERR(priv->sys_clk);
    }
    ret = clk_prepare_enable(priv->sys_clk);
    if(ret)
	return ret;

    /*Enable apb of each timer channels*/
    priv->apb_clk = devm_clk_get(&pdev->dev, "apb");
    if (IS_ERR(priv->apb_clk)) {
        dev_err(&pdev->dev, "failed to get apb clock\n");
        return PTR_ERR(priv->apb_clk);
    }
    ret = clk_prepare_enable(priv->apb_clk);
    if(ret)
        return ret;

    //printk("%s:phy_addr[0x%x] \n",__func__,res->start);

    priv->base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
    if (!priv->base) {
        dev_err(&pdev->dev, "cannot ioremap\n");
        return -EINVAL;
    }

    //printk("%s:remap_addr[0x%x] \n",__func__,priv->base);


    /* Interrupt handler settings */
    irq = platform_get_irq(pdev, 0);
    ret = devm_request_irq(&pdev->dev, irq, irq_tim_interrupt, 0, pdev->name, priv);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to request IRQ\n");
        return ret;
    }

    priv->isUsed = false;
    minor_n = minor_n + 1;

    printk("%s:probed\n",pdev->name);

    return 0;

}
/******************************************************************************
End of function tim_dev_probe
******************************************************************************/


/******************************************************************************
* Function Name : tim_dev_probe
* Description   :
* Arguments     :
* Return Value  :
******************************************************************************/
static int tim_dev_remove(struct platform_device *pdev)
{
    return 0;
}
/******************************************************************************
End of function tim_dev_probe
******************************************************************************/

/* For the device tree */
static struct of_device_id tim_dev_of_match[] = {
	{ .compatible = "renesas,rzv2m-tim-cnt", },
	{ .compatible = "renesas,rzv2ma-tim-cnt", },
	{ /* end of list */ },
};

/* For the device tree */
static struct platform_driver tim_dev_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = tim_dev_of_match,
    },
    .probe      = tim_dev_probe,
    .remove     = tim_dev_remove,
};


/******************************************************************************
* Function Name : tim_device_init
* Description   :
* Arguments     :
* Return Value  :
******************************************************************************/
static int tim_device_init(void)
{
    int alloc_ret = 0;

    alloc_chrdev_region(&g_tim_dev, minor_base, minor_num, DRIVER_NAME);
     if (alloc_ret != 0) {
        printk(KERN_ERR  "alloc_chrdev_region = %d\n", alloc_ret);
        return -1;
    }

    tim_class = class_create(THIS_MODULE, "hw_tim");
    if (IS_ERR(tim_class)) {
        printk(KERN_ERR  "class_create\n");
        return -1;
    }

    return platform_driver_register(&tim_dev_driver);
}
/******************************************************************************
End of function tim_device_init
******************************************************************************/

/******************************************************************************
* Function Name : tim_device_exit
* Description   :
* Arguments     :
* Return Value  :
******************************************************************************/
static void tim_device_exit(void)
{
    if (g_tim_dev)
	unregister_chrdev_region(g_tim_dev, minor_num);
}
/******************************************************************************
End of function tim_device_exit
******************************************************************************/

module_init(tim_device_init);
module_exit(tim_device_exit);
MODULE_DEVICE_TABLE(of, tim_dev_of_match);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas RZ/V2M TIM driver");
MODULE_AUTHOR("Yoshifumi Ohtsuka <yoshifumi.ohtsuka.gx@bp.renesas.com>");
