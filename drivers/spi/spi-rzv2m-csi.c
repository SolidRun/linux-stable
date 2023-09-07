#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sh_dma.h>
#include <linux/spi/spi.h>
#include <asm/unaligned.h>
#include <linux/count_zeros.h>
#include <linux/spinlock.h>

#define MAX_BYTE_FIFO_SIZE 32			/* Maximum 32 byte was stored in FIFO */
#define CSI_WAIT_TIME 5000000 			/* 5s */
#define U_TIME_DELAY 1 				/* 1us */

/* CSI register */
#define CSI_MODE                        0x00    /* CSI mode control                  */
#define CSI_CLKSEL                      0x04    /* CSI clock select                  */
#define CSI_CNT                         0x08    /* CSI control                       */
#define CSI_INT                         0x0C    /* CSI interrupt status              */
#define CSI_IFIFOL                      0x10    /* CSI receive FIFO level display    */
#define CSI_OFIFOL                      0x14    /* CSI transmit FIFO level display   */
#define CSI_IFIFO                       0x18    /* CSI receive window                */
#define CSI_OFIFO                       0x1C    /* CSI transmit window               */
#define CSI_FIFOTRG                     0x20    /* CSI FIFO trigger level            */

/* CSI_MODE */
#define CSI_MODE_DATWT          0xF0000 	/* BITS(16, 19) */
#define CSI_MODE_CSIE           BIT(7)
#define CSI_MODE_TRMD           BIT(6)
#define CSI_MODE_CCL            BIT(5)
#define CSI_MODE_DIR            BIT(4)
#define CSI_MODE_CSOT           BIT(0)

/* CSI_CLKSEL */
#define CSI_CLKSEL_SS_ENA       BIT(19)
#define CSI_CLKSEL_SS_POL       BIT(18)
#define CSI_CLKSEL_CKP          BIT(17)
#define CSI_CLKSEL_DAP          BIT(16)
#define CSI_CLKSEL_SLAVE        BIT(15)
#define CSI_CLKSEL_CKS          0x7FFE 		/* BITS(1,14) */

/* CSI_CNT */
#define CSI_CNT_CSIRST          BIT(28)
#define CSI_CNT_T_TRGEN         BIT(27)
#define CSI_CNT_T_FIFOF         BIT(26)
#define CSI_CNT_T_DMAEN         BIT(24)
#define CSI_CNT_SS_MON          BIT(21)
#define CSI_CNT_R_TRGEN         BIT(19)
#define CSI_CNT_R_FIFOF         BIT(18)
#define CSI_CNT_R_DMAEN         BIT(16)
#define CSI_CNT_UNDER_E         BIT(13)
#define CSI_CNT_OVERF_E         BIT(12)
#define CSI_CNT_TREND_E         BIT(9)
#define CSI_CNT_CSIEND_E        BIT(8)
#define CSI_CNT_T_TRGR_E        BIT(4)
#define CSI_CNT_R_TRGR_E        BIT(0)

/* CSI_INT */
#define CSI_INT_UNDER           BIT(13)
#define CSI_INT_OVERF		BIT(12)
#define CSI_INT_TREND		BIT(9)
#define CSI_INT_CSIEND		BIT(8)
#define CSI_INT_T_TRGR		BIT(4)
#define CSI_INT_R_TRGR		BIT(0)

/* CSI_FIFOTRG */
#define CSI_FIFOTRG_T_TRG       0x700 		/* BITS(8,10) */
#define CSI_FIFOTRG_R_TRG       0x7 		/* BITS(0,2) */

#define TRY_TIMES		10

enum eFIFOTriggerLevel{
    emTriggerLevel	 = 0
    ,emNoneTriggerLevel  = 1
    ,emUnderTriggerLevel = 2
    ,emUpperTriggerLevel = 3
    ,emTriggerRequestDMALevel = 4
};

enum interrupt_type{
    R_TRGR_IRQ,
    T_TRGR_IRQ,
    CSIEND_IRQ,
    TREND_IRQ,
    OVERF_IRQ,
    UNDER_IRQ
};

enum eTransMode{
    emReceiveOnly       = 0
    ,emTransmitReceive  = 1
};

enum {
       CSI_SPI_MASTER,
       CSI_SPI_SLAVE,
};

struct rzv2m_csi_info {
       int mode;
       unsigned int interval_time;
       unsigned int transmit_trigger_level;
       unsigned int receive_trigger_level;
};

struct rzv2m_csi_priv {
       struct spi_master *master;
       void __iomem *regs;
       struct clk *clk;
       struct clk *pclk;
       int irq;
       struct device *dev;
       const void *txbuf;
       void *rxbuf;
       int bytes_to_transfer;
       int bytes_to_receive;

       struct rzv2m_csi_info *info;

       struct platform_device *pdev;
       spinlock_t lock_irq;

       unsigned int min_div_pow;
       struct completion done;
       struct completion rx_done;

       unsigned short unused_ss;
       bool native_cs_inited;
       bool native_cs_high;

       bool slave_aborted;
       bool transmission_completed;
       uint32_t time_wait;

       bool overflow_error_flag;
       bool underrun_error_flag;
       bool transmit_recept_flag;
};


/* -------------------prototype------------------- */
static u32 rzv2m_csi_reg_read_bit(struct rzv2m_csi_priv *p, int reg_offs, int bit_mask);
static void rzv2m_csi_reg_write_bit(struct rzv2m_csi_priv *p, int reg_offs, int bit_mask, u32 value);
static void rzv2m_csi_fill_txfifo(struct rzv2m_csi_priv *p, int size);
static void rzv2m_csi_read_rxfifo(struct rzv2m_csi_priv *p, u32 size);
static irqreturn_t rzv2m_csi_irq_handler(int irq, void *data);
static void rzv2m_csi_spi_set_clk_regs(struct rzv2m_csi_priv *p, unsigned long parent_rate, u32 spi_hz);
static int rzv2m_csi_setup_operation_mode(struct spi_master *master, struct spi_transfer *t);
static int rzv2m_csi_setup_clock_selection(struct spi_master *master, struct spi_device *spi);
static int rzv2m_csi_clear_all_irq(struct rzv2m_csi_priv *p);
static bool is_recv_only_mode(struct rzv2m_csi_priv *p);
static int rzv2m_csi_set_irq(struct spi_master *master, struct spi_transfer *t);
static int rzv2m_csi_clear_fifo_buffer(struct spi_master *master, struct spi_transfer *t);
static int rzv2m_csi_start_operation(struct rzv2m_csi_priv *p);
static int rzv2m_csi_stop_operation(struct rzv2m_csi_priv *p);
static bool is_16bit_data_leng(struct rzv2m_csi_priv *p);
static int rzv2m_csi_set_fifo_trg(struct spi_master *master, struct spi_transfer *t);
static void get_data_in_ififo(struct rzv2m_csi_priv *p);
static int rzv2m_csi_spi_setup(struct spi_device *spi);
static int rzv2m_csi_slave_abort(struct spi_master *master);
static struct rzv2m_csi_info* rzv2m_csi_parse_dt(struct device *dev);
/* -------------------end prototype------------------- */

static u32 rzv2m_csi_reg_read_bit(struct rzv2m_csi_priv *p, int reg_offs, int bit_mask){
       u32 value;

       value = readl(p->regs + reg_offs) & bit_mask;

       while((bit_mask%2) == 0)
       {
           value = value >> 1;
           bit_mask = bit_mask >> 1;
       }

       return (u32) value;
}

static void rzv2m_csi_reg_write_bit(struct rzv2m_csi_priv *p,
				    int reg_offs, int bit_mask, u32 value)
{
	int nr_zeros;
	u32 tmp;

	nr_zeros = count_trailing_zeros(bit_mask);
	value <<= nr_zeros;

	tmp = (readl(p->regs + reg_offs) & ~bit_mask) | value;
	writel(tmp, p->regs + reg_offs);
}

static void rzv2m_csi_fill_txfifo(struct rzv2m_csi_priv *p, int size)
{
       int i = 0;
       if ((p->bytes_to_transfer <= 0) || \
               (readl(p->regs + CSI_OFIFOL) == MAX_BYTE_FIFO_SIZE))
       {
               return; /* fixme. Check remain byte transfer. */
       }
       size = min_t(int, MAX_BYTE_FIFO_SIZE - readl(p->regs + CSI_OFIFOL), size);

       if(is_16bit_data_leng(p))
       {
               u16 *buf = (u16 *)p->txbuf;
               for(i = 0; i < size/2;i++)
	       {
                       writel(buf[i], (p->regs + CSI_OFIFO));
               }
       } 
       else
       {
               u8 *buf = (u8 *)p->txbuf;
               for(i = 0; i < size;i++)
	       {
                       writel(buf[i], (p->regs + CSI_OFIFO));
               }
       }
       p->txbuf += size;
       p->bytes_to_transfer -= size;
}

static void rzv2m_csi_read_rxfifo(struct rzv2m_csi_priv *p, u32 size)
{
    int i = 0;
       if(is_16bit_data_leng(p))
       {
               u16 *buf = (u16 *)p->rxbuf;
               for(i = 0; i < size/2;i++)
	       {
                       buf[i] = (u16)readl(p->regs + CSI_IFIFO);
               }
       } 
       else
       {
               u8 *buf = p->rxbuf;
               for(i = 0; i < size;i++)
	       {
                       buf[i] = (u8)readl(p->regs + CSI_IFIFO);
               }
       }
       p->rxbuf += size;
       p->bytes_to_receive += size;
}

static irqreturn_t rzv2m_csi_irq_handler(int irq, void *data)
{
       struct rzv2m_csi_priv *p = (struct rzv2m_csi_priv*)data;
       unsigned long flag;
       int tmp;

       spin_lock_irqsave(&p->lock_irq, flag);

       p->time_wait = 0; /* reset timeout wait */

       if (!spi_controller_is_slave(p->master))
       {
               if(is_recv_only_mode(p))
	       {
                       /* handle MASTER RECV only mode */
                       if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_R_TRGR) == 1)
		       {
                               get_data_in_ififo(p);
			       tmp = readl(p->regs +CSI_INT);
			       writel(tmp & CSI_INT_R_TRGR, p->regs + CSI_INT);
                               /* fixme. w/a for stopping transfer data in reception-only mode */
                               if(p->bytes_to_receive >= p->bytes_to_transfer)
			       {
                                       rzv2m_csi_stop_operation(p);
                               }
                       }
		       else
		       {
                               rzv2m_csi_clear_all_irq(p);
                       }
               }
	       else
	       {
                       /* handle MASTER SEND/RECV mode */
                       if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_OVERF) == 1)
		       {
                               get_data_in_ififo(p);
			       tmp = readl(p->regs +CSI_INT);
			       writel(tmp & CSI_INT_OVERF, p->regs + CSI_INT);
                               dev_err(&p->pdev->dev, "Overflow error \n");
                               p->overflow_error_flag = true;
                       }
		       else
		       {
                               if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_TREND) == 1)
			       {
                                       p->transmit_recept_flag = true;

                                       if(p->rxbuf != NULL)
                                               get_data_in_ififo(p);

                                       rzv2m_csi_fill_txfifo(p, min_t(int, p->bytes_to_transfer, MAX_BYTE_FIFO_SIZE));
				       tmp = readl(p->regs +CSI_INT);
				       writel(tmp & CSI_INT_TREND, p->regs + CSI_INT);

                               }
			       else
			       {
                                       if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_T_TRGR) == 1)
				       {
                                               p->transmit_recept_flag = true;

                                               if(p->rxbuf != NULL)
                                                       get_data_in_ififo(p);

                                               rzv2m_csi_fill_txfifo(p, min_t(int, p->bytes_to_transfer, MAX_BYTE_FIFO_SIZE));
					       tmp = readl(p->regs +CSI_INT);
					       writel(tmp & CSI_INT_T_TRGR, p->regs + CSI_INT);
                                       }
				       else
				       {
                                               rzv2m_csi_clear_all_irq(p);
                                       }
                               }
                       }
	       }
       }
       else
       {
               if(is_recv_only_mode(p))
	       {
                       /* handle SLAVE RECV only mode */
                       if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_OVERF) == 1)
		       {
                               get_data_in_ififo(p);
			       tmp = readl(p->regs +CSI_INT);
			       writel(tmp & CSI_INT_OVERF, p->regs + CSI_INT);
                               dev_err(&p->pdev->dev, "Overflow error \n");
                               p->overflow_error_flag = true;
                       }
		       else
		       {
                               if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_R_TRGR) == 1)
			       {
                                       get_data_in_ififo(p);
				       tmp = readl(p->regs +CSI_INT);
				       writel(tmp & CSI_INT_R_TRGR, p->regs + CSI_INT);
                               }
			       else
			       {
                                       rzv2m_csi_clear_all_irq(p);
                               }
                       }
               }
	       else
	       {
                       /* handle SLAVE SEND/RECV mode */
                       if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_OVERF) == 1)
		       {
                               get_data_in_ififo(p);
			       tmp = readl(p->regs +CSI_INT);
			       writel(tmp & CSI_INT_OVERF, p->regs + CSI_INT);
                               dev_err(&p->pdev->dev, "Overflow error \n");
                               p->overflow_error_flag = true;
                       }
		       else if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_UNDER) == 1)
		       {
                               rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_CSIRST, 0x1);
                               rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_CSIRST, 0x0);
                               rzv2m_csi_fill_txfifo(p, min_t(int, p->bytes_to_transfer, MAX_BYTE_FIFO_SIZE));
                               rzv2m_csi_start_operation(p);
                               dev_err(&p->pdev->dev, "Underrun error \n");
                               p->underrun_error_flag = true;
                       }
		       else
		       {
                               if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_TREND) == 1)
			       {
                                       p->transmit_recept_flag = true;
                                       if(p->rxbuf != NULL)
                                               get_data_in_ififo(p);

                                       rzv2m_csi_fill_txfifo(p, min_t(int, p->bytes_to_transfer, MAX_BYTE_FIFO_SIZE));
				       tmp = readl(p->regs +CSI_INT);
				       writel(tmp & CSI_INT_TREND, p->regs + CSI_INT);
                               }
			       else
			       {
                                       if(rzv2m_csi_reg_read_bit(p, CSI_INT, CSI_INT_T_TRGR) == 1)
				       {
                                               p->transmit_recept_flag = true;

                                               if(p->rxbuf != NULL)
                                                       get_data_in_ififo(p);

                                               rzv2m_csi_fill_txfifo(p, min_t(int, p->bytes_to_transfer, MAX_BYTE_FIFO_SIZE));
					       tmp = readl(p->regs +CSI_INT);
					       writel(tmp & CSI_INT_T_TRGR, p->regs + CSI_INT);
                                       }
				       else
				       {
                                               rzv2m_csi_clear_all_irq(p);
                                       }
                               }
                       }
               }
       }

       spin_unlock_irqrestore(&p->lock_irq, flag);

       return IRQ_HANDLED;
}

static void rzv2m_csi_spi_set_clk_regs(struct rzv2m_csi_priv *p,
                                     unsigned long parent_rate, u32 spi_hz)
{
       u32 cks;

       if (!spi_hz || !parent_rate) {
	       dev_err(&p->pdev->dev, "Invalid clock rate parameters %lu and %u\n",
                    parent_rate, spi_hz);
               return;
       }
       cks = parent_rate/(spi_hz*2);

       if (cks > 0x3FFF) 
       {
               dev_warn(&p->pdev->dev,
                       "Requested SPI transfer rate %d is too low. Transfer rate was set %ld\n", spi_hz, parent_rate/0x3FFF);
               cks = 0x3FFF;
       }
       else if(cks < 0x1)
       {
               dev_warn(&p->pdev->dev,
                       "Requested SPI transfer rate %d is too large. Transfer rate was set %ld\n", spi_hz, parent_rate/2);
               cks = 0x1;
       }
       rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_CKS, cks);
}

static int rzv2m_csi_setup_operation_mode(struct spi_master *master, struct spi_transfer *t)
{
       int ret = 0;
       struct rzv2m_csi_priv *p = spi_master_get_devdata(master);
       const void *tx_buf = t->tx_buf;
       void *rx_buf = t->rx_buf;
       struct rzv2m_csi_info *info = p->info;

       /* Configure pins before asserting CS */
       if (!spi_controller_is_slave(p->master))
       {
               rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_SLAVE, 0x0); /* Setting operation mode as master */
               if((rx_buf != NULL) && (tx_buf == NULL))
	       {
                       /* reception-only mode */
                       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_TRMD, 0x0);
               }
	       else
	       {
                       /* transmission and reception mode */
               rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_TRMD, 0x1);
               }

               if(info->interval_time < 16)
	       {
                       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_DATWT, info->interval_time);
               }
	       else
	       {
                       ret = -EINVAL;
               }
       }
       else
       {
               rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_SLAVE, 0x1); /* Setting operation mode as slave */
               if((rx_buf != NULL) && (tx_buf == NULL))
	       {
                       /* reception-only mode. fixme */
                       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_TRMD, 0x0);
               }
	       else
	       {
		       /* transmission and reception mode */
                       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_TRMD, 0x1);
               }
       }

       /* setup data length */
       if(t->bits_per_word == 16)
       {
	       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_CCL, 0x1); /* Setting serial data length to 16 bit */
       }
       else
       {
	       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_CCL, 0x0); /* Setting serial data length to 8 bit */
       }

       return ret;
}

static int rzv2m_csi_setup_clock_selection(struct spi_master *master, struct spi_device *spi)
{
       struct rzv2m_csi_priv *p = spi_master_get_devdata(master);
       u32 dap, ckp, lsb, ss_pol;
       uint32_t slave_count = 0;

       dap = !!(spi->mode & SPI_CPHA);
       ckp = !!(spi->mode & SPI_CPOL);

       lsb = !!(spi->mode & SPI_LSB_FIRST);

       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_DIR, lsb);
       rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_DIR, lsb);
       rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_CKP, ckp);
       rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_DAP, dap);

       if (spi_controller_is_slave(p->master))
       {
	       /* The slave select signal is used to select the slave that
               communicates with the master in a system in
               which multiple slaves are connected to one master. */

               if(slave_count > 1)
	       {
                       rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_SS_ENA, 0x1);
               }
	       else
	       {
                       rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_SS_ENA, 0x0);
               }

               ss_pol = !!(spi->mode & SPI_CS_HIGH);
               rzv2m_csi_reg_write_bit(p, CSI_CLKSEL, CSI_CLKSEL_SS_POL, ss_pol);
       }

       return 0;
}

static int rzv2m_csi_clear_all_irq(struct rzv2m_csi_priv *p)
{

       writel(0x3311, p->regs + CSI_INT);

       return 0;
}

static bool is_recv_only_mode(struct rzv2m_csi_priv *p)
{
       bool ret = false;

       u32 tmp = rzv2m_csi_reg_read_bit(p, CSI_MODE, CSI_MODE_TRMD);
       if(tmp != 0)
       {
               ret = false;
       }
       else
       {
               ret = true;
       }

       return ret;
}

static int rzv2m_csi_set_irq(struct spi_master *master, struct spi_transfer *t)
{
       int ret = 0;
       struct rzv2m_csi_priv *p = spi_master_get_devdata(master);
       void *rx_buf = t->rx_buf;

    if(!spi_controller_is_slave(master))
    {
               if(is_recv_only_mode(p))
	       {
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_CSIEND_E, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_R_TRGEN, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_R_TRGR_E, 0x1);
               }
	       else
	       {
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_TREND_E, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_T_TRGEN, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_T_TRGR_E, 0x1);
                       
		       /* fixme. conflict trans only - trans/recv */
                       if(rx_buf != NULL)
		       {
                               rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_OVERF_E, 0x1);
                       }
               }
    }
    else
    {
               if(is_recv_only_mode(p))
	       {
                       /* reg_write_bit(p, CSI_CNT, CSI_CNT_TREND_E, 0x1); */
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_CSIEND_E, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_R_TRGEN, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_R_TRGR_E, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_OVERF_E, 0x1);
               }
	       else
	       {
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_TREND_E, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_T_TRGEN, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_T_TRGR_E, 0x1);
                       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_UNDER_E, 0x1);

                       /* fixme. conflict trans only - trans/recv */
                       if(rx_buf != NULL)
		       {
                               rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_OVERF_E, 0x1);
                       }
               }
    }

       return ret;
}

static int rzv2m_csi_clear_fifo_buffer(struct spi_master *master, struct spi_transfer *t)
{
       int ret = 0;
       struct rzv2m_csi_priv *p = spi_master_get_devdata(master);

       writel(0x0, p->regs + CSI_IFIFOL);
       writel(0x0, p->regs + CSI_OFIFOL);

       return ret;
}

static int rzv2m_csi_start_operation(struct rzv2m_csi_priv *p)
{
       int ret = 0;
       bool flag_end = false;
       int count = 0;
       int tmp;

       do
       {
               rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_CSIE, 0x1);
               tmp = rzv2m_csi_reg_read_bit(p, CSI_MODE, CSI_MODE_CSOT);
               if((tmp != 0) || (count > TRY_TIMES))
	       {
                       flag_end = true;
               }
               count++;
       }
       while(!flag_end);

       return ret;
}

static int rzv2m_csi_stop_operation(struct rzv2m_csi_priv *p)
{
       int ret = 0;
       bool flag_end = false;
       int count = 0;
       int tmp;

       do
       {
               rzv2m_csi_reg_write_bit(p, CSI_MODE, CSI_MODE_CSIE, 0x0);
               tmp = rzv2m_csi_reg_read_bit(p, CSI_MODE, CSI_MODE_CSOT);
               if((tmp == 0) || (count > TRY_TIMES))
	       {
                       flag_end = true;
               }
               count++;
       }
       while(!flag_end);

       return ret;
}

static bool is_16bit_data_leng(struct rzv2m_csi_priv *p)
{
       u32 tmp;
       bool ret;

       tmp = rzv2m_csi_reg_read_bit(p, CSI_MODE, CSI_MODE_CCL);
       if(tmp != 0)
       {
               ret = true;
       }
       else
       {
               ret = false;
       }
       return ret;
}


static int rzv2m_csi_set_fifo_trg(struct spi_master *master, struct spi_transfer *t)
{
       int ret = 0;
       struct rzv2m_csi_priv *p = spi_master_get_devdata(master);
       struct rzv2m_csi_info *info = p->info;
       const void *tx_buf = t->tx_buf;
       void *rx_buf = t->rx_buf;

       if(tx_buf != NULL)
       {
               if(is_16bit_data_leng(p))
	       {
		       /* 16 bit mode */
                       if((info->transmit_trigger_level > 4))
		       {
                               ret = -EINVAL;
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_T_TRG, 4);
                       }
		       else
		       {
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_T_TRG, info->transmit_trigger_level);
                       }
               } 
	       else
	       {
		       /* 8 bit mode */
                       if((info->transmit_trigger_level > 5))
		       {
                               ret = -EINVAL;
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_T_TRG, 5);
                       }
		       else
		       {
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_T_TRG, info->transmit_trigger_level);
                       }
               }
       }

       if(rx_buf != NULL)
       {
               /* setting for receive trigger level */
               if(is_16bit_data_leng(p))
	       {
		       /* 16 bit mode */
                       if((info->receive_trigger_level > 4))
		       {
                               ret = -EINVAL;
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_R_TRG, 4);
                       }
		       else
		       {
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_R_TRG, info->receive_trigger_level);
                       }
               }
	       else
	       {
		       /* 8 bit mode */
                       if((info->receive_trigger_level > 5))
		       {
                               ret = -EINVAL;
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_R_TRG, 5);
                       }
		       else
		       {
                               rzv2m_csi_reg_write_bit(p, CSI_FIFOTRG, CSI_FIFOTRG_R_TRG, info->receive_trigger_level);
                       }
               }
       }
       
       return ret;
}

static void get_data_in_ififo(struct rzv2m_csi_priv *p)
{
       unsigned int tmp;

       while((tmp = readl(p->regs + CSI_IFIFOL)) != 0)
       {
               rzv2m_csi_read_rxfifo(p, tmp);
       }

       return;
}

static void overflow_error_handler(struct rzv2m_csi_priv *p)
{
       p->overflow_error_flag = false;
       return;
}

static void underrun_error_handler(struct rzv2m_csi_priv *p)
{
       p->underrun_error_flag = false;
       return;
}


static int rzv2m_csi_start_transfer(struct spi_master *master,
                                     struct spi_device *qspi,
                                     struct spi_transfer *transfer)
{
       struct rzv2m_csi_priv *p = spi_master_get_devdata(master);
       struct platform_device *pdev = p->pdev;
       struct device *dev = &pdev->dev;
       int ret;

       p->txbuf = transfer->tx_buf;
       p->rxbuf = transfer->rx_buf;
       p->transmission_completed = false;

       /* Release of CSI reset and selection of data transfer method */
       writel(0x0, p->regs + CSI_CNT);

       /* setup operation mode */
       ret = rzv2m_csi_setup_operation_mode(master, transfer);
       if (ret) 
       {
               dev_warn(dev, "rzv2m_csi_setup_operation_mode failed %d\n", ret);
       }

       ret = rzv2m_csi_setup_clock_selection(master, qspi);
       if (ret)
       {
               dev_warn(dev, "rzv2m_csi_setup_clock_selection failed %d\n", ret);
       }

       /* setup clocks frequent division */
       if (!spi_controller_is_slave(p->master))
       {
               rzv2m_csi_spi_set_clk_regs(p, clk_get_rate(p->clk), transfer->speed_hz);
       }

       /* Perform CSI reset and enable CSI_CLKSEL setting */
       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_CSIRST, 0x1);

       while(!(rzv2m_csi_reg_read_bit(p, CSI_CNT, CSI_CNT_CSIRST) == 0x1))
       {
               udelay(10);
       }

       /* Canceling CSI reset */
       rzv2m_csi_reg_write_bit(p, CSI_CNT, CSI_CNT_CSIRST, 0x0);

       while(!(rzv2m_csi_reg_read_bit(p, CSI_CNT, CSI_CNT_CSIRST) == 0))
       {
               udelay(10);
       }

       /* Clear all interrupt factor flags */
       ret = rzv2m_csi_clear_all_irq(p);
       if (ret) 
       {
               dev_warn(dev, "rzv2m_csi_clear_all_irq failed %d\n", ret);
       }

       /* Set the receive trigger level */
       ret = rzv2m_csi_set_fifo_trg(master, transfer);
       if (ret) 
       {
               dev_warn(dev, "rzv2m_csi_clear_all_irq failed %d\n", ret);
       }

       /* Enable interrupt */
       ret = rzv2m_csi_set_irq(master, transfer);
       if (ret) 
       {
               dev_warn(dev, "rzv2m_csi_set_irq failed %d\n", ret);
       }

       p->bytes_to_transfer = transfer->len;
       p->bytes_to_receive = 0;
       p->time_wait = 0;
       p->overflow_error_flag = false;
       p->underrun_error_flag = false;
       p->transmit_recept_flag = false;

       /* clear receive buffer */
       if(p->rxbuf != NULL)
       {
               memset(p->rxbuf, 0, p->bytes_to_transfer);
       }

       /* Clear the receive FIFO buffer */
       ret = rzv2m_csi_clear_fifo_buffer(master, transfer);
       if (ret) 
       {
               dev_warn(dev, "rzv2m_csi_clear_fifo_buffer failed %d\n", ret);
       }

       if(is_recv_only_mode(p))
       {
	       ret = rzv2m_csi_start_operation(p);
               /* stop function will be handled in IRQ Handler */
       } 
       else
       {
               rzv2m_csi_fill_txfifo(p, min_t(int, p->bytes_to_transfer, MAX_BYTE_FIFO_SIZE));
	       /* Start transmission / reception operation */
	       ret = rzv2m_csi_start_operation(p);
	       if (ret) 
	       {
		       dev_err(dev, "rzv2m_csi_start_operation failed %d\n", ret);
	       }
               if(p->rxbuf != NULL)
	       {
                       while(p->bytes_to_receive < transfer->len)
		       {
                               if(p->overflow_error_flag == true)
                                       overflow_error_handler(p);

                               if(p->underrun_error_flag == true)
                                       underrun_error_handler(p);

                               if(p->transmit_recept_flag == true)
			       {
                                       get_data_in_ififo(p);
                                       p->transmit_recept_flag = false;
                               }

                               if(p->time_wait++ > CSI_WAIT_TIME)
			       {
                                       break;
                               }
                               udelay(U_TIME_DELAY);
                       }
               } 
	       else
	       {
		       while(p->bytes_to_transfer > 0)
		       {
                               rzv2m_csi_fill_txfifo(p, min_t(int, p->bytes_to_transfer, MAX_BYTE_FIFO_SIZE));
                       }
               }

               /* get remain data */
               get_data_in_ififo(p);

               if(p->bytes_to_receive < transfer->len)
	       {
		       dev_err(dev, "Receive/Total = %d/%d byte\n", p->bytes_to_receive, transfer->len);
               }
	       rzv2m_csi_stop_operation(p);
       }

       return ret;
}

static int rzv2m_csi_spi_setup(struct spi_device *spi)
{

       return 0;
}

static int rzv2m_csi_slave_abort(struct spi_master *master)
{
       struct rzv2m_csi_priv *p = spi_master_get_devdata(master);

       p->slave_aborted = true;

       return 0;
}

static struct rzv2m_csi_info* rzv2m_csi_parse_dt(struct device *dev)
{
	uint32_t slave_count = 0;
       struct rzv2m_csi_info *info;
       struct device_node *np = dev->of_node;

       info = devm_kzalloc(dev, sizeof(struct rzv2m_csi_info), GFP_KERNEL);
       if (!info)
               return NULL;

       info->mode = of_property_read_bool(np, "spi-slave") ? CSI_SPI_SLAVE : CSI_SPI_MASTER;

       if(info->mode == CSI_SPI_SLAVE)
       {
               slave_count++;
       }

       of_property_read_u32(np, "interval_time", &info->interval_time);
       of_property_read_u32(np, "tx_trigger_lvl", &info->transmit_trigger_level);
       of_property_read_u32(np, "rx_trigger_lvl", &info->receive_trigger_level);

       return info;
}

static int rzv2m_csi_probe(struct platform_device *pdev)
{
       int ret = 0;
       int irq = 0;
       struct spi_master *master;
       struct resource *res;
       struct device *dev = &pdev->dev;
       struct rzv2m_csi_priv *p;
       struct rzv2m_csi_info *info;

       info = rzv2m_csi_parse_dt(&pdev->dev);
       if(!info)
       {
               dev_err(&pdev->dev, "failed to obtain device info\n");
               return -ENXIO;
       }

       if (info->mode == CSI_SPI_SLAVE) 
       {
               master = spi_alloc_slave(&pdev->dev, sizeof(struct rzv2m_csi_priv));
       }
       else
       {
	       master = spi_alloc_master(&pdev->dev, sizeof(struct rzv2m_csi_priv));
       }
       if (!master)
       {
               dev_err(&pdev->dev, "failed to alloc master\n");
               return -ENOMEM;
       }
       p = spi_master_get_devdata(master);

       platform_set_drvdata(pdev, p);
       p->master = master;
       p->info = info;
       p->pdev = pdev;

       res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
       p->regs = devm_ioremap_resource(dev, res);
       if (IS_ERR(p->regs))
       {
               ret = PTR_ERR(p->regs);
               goto clk_dis_all;
       }

       p->clk = devm_clk_get(&pdev->dev, NULL);
       if (IS_ERR(p->clk)) 
       {
               dev_err(&pdev->dev, "cannot get csi_clk\n");
               ret = PTR_ERR(p->clk);
               goto clk_dis_clk;
       }

       ret = clk_prepare_enable(p->clk);
       if (ret)
       {
               dev_err(dev, "Unable to enable device clock.\n");
               goto clk_dis_clk;
       }

       irq = platform_get_irq(pdev, 0);
       if (irq < 0)
       {
               dev_err(&pdev->dev, "cannot get IRQ\n");
               ret = irq;
               goto clk_dis_all;
       }
       ret = devm_request_irq(&pdev->dev, irq, rzv2m_csi_irq_handler, 0,
                              dev_name(&pdev->dev), p);

       if(ret)
       {
	       dev_err(&pdev->dev, "cannot request IRQ\n");
	       goto clk_dis_all;
       }

       spin_lock_init(&p->lock_irq);
       
       /* init master code */
       master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH; /* initial setting */
       master->mode_bits |= SPI_LSB_FIRST | SPI_3WIRE;
       master->dev.of_node = pdev->dev.of_node;
       master->setup = rzv2m_csi_spi_setup;
       master->slave_abort = rzv2m_csi_slave_abort;
       master->bits_per_word_mask = SPI_BPW_RANGE_MASK(8, 16);
       master->transfer_one = rzv2m_csi_start_transfer;

       ret = devm_spi_register_master(&pdev->dev, master);
       if (ret < 0)
       {
               dev_err(&pdev->dev, "spi_register_master error.\n");
               goto clk_dis_all;
       }

       dev_info(&pdev->dev, "probed\n");

       return 0;

clk_dis_all:
       pm_runtime_set_suspended(&pdev->dev);
       pm_runtime_disable(&pdev->dev);
clk_dis_clk:
       clk_disable_unprepare(p->clk);
       spi_master_put(master);
       return ret;
}

static int rzv2m_csi_remove(struct platform_device *pdev)
{
       pm_runtime_disable(&pdev->dev);
       return 0;
}

static const struct of_device_id rzv2m_csi_match[] = {
       { .compatible = "renesas,rzv2m-csi", },
       { .compatible = "renesas,rzv2ma-csi", },
       { /* End of table */ }
};
MODULE_DEVICE_TABLE(of, rzv2m_csi_match);

static struct platform_driver rzv2m_csi_drv = {
       .probe          = rzv2m_csi_probe,
       .remove         = rzv2m_csi_remove,
       .driver         = 
       {
	       .name           = "rzv2m_csi",
               .of_match_table = of_match_ptr(rzv2m_csi_match),
       },
};

module_platform_driver(rzv2m_csi_drv);

MODULE_DESCRIPTION("Clock Serial Interface Driver");
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("GPL v2");
