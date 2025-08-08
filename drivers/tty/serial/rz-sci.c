// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Renesas Serial Communication Interface driver. (SCI with no FIFO / with FIFO)
 * based off of the drivers/tty/serial/sh-sci.c to support UART mode.
 */
#undef DEBUG

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/ctype.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ktime.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/serial.h>
#include <linux/serial_sci.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysrq.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include "serial_mctrl_gpio.h"
#include "rz-sci.h"

/* Offsets into the sci_port->irqs array */
enum {
	SCIx_ERI_IRQ,
	SCIx_RXI_IRQ,
	SCIx_TXI_IRQ,
	SCIx_TEI_IRQ,
	SCIx_NR_IRQS,

	SCIx_MUX_IRQ = SCIx_NR_IRQS,	/* special case */
};

#define SCIx_IRQ_IS_MUXED(port)		\
	(((port)->irqs[SCIx_ERI_IRQ] ==	\
	 (port)->irqs[SCIx_RXI_IRQ]) ||	\
	((port)->irqs[SCIx_ERI_IRQ] &&	\
	 ((port)->irqs[SCIx_RXI_IRQ] < 0)))

enum SCI_CLKS {
	SCI_FCK,		/* Functional Clock */
	SCI_NUM_CLKS
};

/* Bit x set means sampling rate x + 1 is supported */
#define SCI_SR(x)		BIT((x) - 1)

#define min_sr(_port)		ffs((_port)->sampling_rate_mask)
#define max_sr(_port)		fls((_port)->sampling_rate_mask)

/* Iterate over all supported sampling rates, from high to low */
#define for_each_sr(_sr, _port)						\
	for ((_sr) = max_sr(_port); (_sr) >= min_sr(_port); (_sr)--)	\
		if ((_port)->sampling_rate_mask & SCI_SR((_sr)))

struct plat_sci_reg {
	u8 offset, size;
};

struct sci_port_params {
	const struct plat_sci_reg regs[SCIx_NR_REGS];
	unsigned int fifosize;
	unsigned int sampling_rate_mask;
	unsigned int error_mask;
	unsigned int error_clear;
};

struct sci_port {
	struct uart_port	port;

	/* Platform configuration */
	const struct sci_port_params	*params;
	const struct plat_sci_port	*cfg;
	unsigned int			sampling_rate_mask;
	resource_size_t			reg_size;
	struct mctrl_gpios		*gpios;

	/* Clocks */
	struct clk			*clks[SCI_NUM_CLKS];
	unsigned long			clk_rates[SCI_NUM_CLKS];

	int				irqs[SCIx_NR_IRQS];
	char				*irqstr[SCIx_NR_IRQS];

	int				rx_trigger;
	struct timer_list		rx_fifo_timer;
	int				rx_fifo_timeout;
};

#define SCI_NPORTS CONFIG_SERIAL_RZ_SCI_NR_UARTS

static struct sci_port sci_ports[SCI_NPORTS];
static unsigned long sci_ports_in_use;
static struct uart_driver sci_uart_driver;

static inline struct sci_port *
to_sci_port(struct uart_port *uart)
{
	return container_of(uart, struct sci_port, port);
}

static const struct sci_port_params sci_port_params[SCIx_NR_REGTYPES] = {
	/*
	 * Common SCI definitions, dependent on the port's regshift
	 * value.
	 */
	[SCIx_SCI_REGTYPE] = {
		.regs = {
			[RDR]		= { 0x00,  32 },
			[TDR]		= { 0x04,  32 },
			[CCR0]		= { 0x08,  32 },
			[CCR1]		= { 0x0C,  32 },
			[CCR2]		= { 0x10,  32 },
			[CCR3]		= { 0x14,  32 },
			[CCR4]		= { 0x18,  32 },
			[CSR]		= { 0x48,  32 },
			[CFCLR]		= { 0x68,  32 },
		},
		.fifosize = 1,
		.sampling_rate_mask = SCI_SR(32),
		.error_mask = RSCI_DEFAULT_ERROR_MASK | CSR_ORER,
		.error_clear = RSCI_ERROR_CLEAR,
	},

	[SCIx_RZ_SCIFA_REGTYPE] = {
		.regs = {
			[RDR]		= { 0x00,  32 },
			[TDR]		= { 0x04,  32 },
			[CCR0]		= { 0x08,  32 },
			[CCR1]		= { 0x0C,  32 },
			[CCR2]		= { 0x10,  32 },
			[CCR3]		= { 0x14,  32 },
			[CCR4]		= { 0x18,  32 },
			[FCR]		= { 0x24,  32 },
			[CSR]		= { 0x48,  32 },
			[FRSR]		= { 0x50,  32 },
			[FTSR]		= { 0x54,  32 },
			[CFCLR]		= { 0x68,  32 },
			[FFCLR]		= { 0x70,  32 },
		},
		.fifosize = 16,
		.sampling_rate_mask = SCI_SR(32),
		.error_mask = RSCI_DEFAULT_ERROR_MASK,
		.error_clear = RSCI_ERROR_CLEAR,
	},
};

#define sci_getreg(up, offset)		(&to_sci_port(up)->params->regs[offset])

static unsigned int sci_serial_in(struct uart_port *p, int offset)
{
	const struct plat_sci_reg *reg = sci_getreg(p, offset);

	if (reg->size == 8)
		return ioread8(p->membase + reg->offset);
	else if (reg->size == 32)
		return ioread32(p->membase + reg->offset);
	else
		WARN(1, "Invalid register access\n");

	return 0;
}

static void sci_serial_out(struct uart_port *p, int offset, int value)
{
	const struct plat_sci_reg *reg = sci_getreg(p, offset);

	if (reg->size == 8)
		iowrite8(value, p->membase + reg->offset);
	else if (reg->size == 32)
		iowrite32(value, p->membase + reg->offset);
	else
		WARN(1, "Invalid register access\n");
}

static void sci_port_enable(struct sci_port *sci_port)
{
	unsigned int i;

	if (!sci_port->port.dev)
		return;

	pm_runtime_get_sync(sci_port->port.dev);

	for (i = 0; i < SCI_NUM_CLKS; i++) {
		clk_prepare_enable(sci_port->clks[i]);
		sci_port->clk_rates[i] = clk_get_rate(sci_port->clks[i]);
	}
	sci_port->port.uartclk = sci_port->clk_rates[SCI_FCK];
}

static void sci_port_disable(struct sci_port *sci_port)
{
	unsigned int i;

	if (!sci_port->port.dev)
		return;

	for (i = SCI_NUM_CLKS; i-- > 0; )
		clk_disable_unprepare(sci_port->clks[i]);

	pm_runtime_put_sync(sci_port->port.dev);
}

static void sci_start_tx(struct uart_port *port)
{
	unsigned int ctrl;

	/* TE (Transmit Enable) must be set after setting TIE (Transmit Interrupt Enable)
	 * or in the same instruction to start the transmit process.
	 */
	ctrl = serial_port_in(port, CCR0);
	ctrl |= CCR0_TIE | CCR0_TE;
	serial_port_out(port, CCR0, ctrl);
}

static void sci_stop_tx(struct uart_port *port)
{
	unsigned int ctrl;

	ctrl = serial_port_in(port, CCR0);
	ctrl &= ~CCR0_TIE;

	serial_port_out(port, CCR0, ctrl);
}

static void sci_start_rx(struct uart_port *port)
{
	unsigned int ctrl;

	ctrl = serial_port_in(port, CCR0);
	ctrl |= CCR0_RIE;

	serial_port_out(port, CCR0, ctrl);
}

static void sci_stop_rx(struct uart_port *port)
{
	unsigned int ctrl;

	ctrl = serial_port_in(port, CCR0);
	ctrl &= ~CCR0_RIE;

	serial_port_out(port, CCR0, ctrl);
}

static void sci_clear_CFC(struct uart_port *port, unsigned int mask)
{
	serial_port_out(port, CFCLR, mask);
}

static void sci_clear_DRxC(struct uart_port *port)
{
	serial_port_out(port, CFCLR, CFCLR_RDRFC);

	if (port->type == PORT_SCIF)
		serial_port_out(port, FFCLR, FFCLR_DRC);
}

#if defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_SERIAL_RZ_SCI_CONSOLE) || \
	defined(CONFIG_SERIAL_RZ_SCI_EARLYCON)

#ifdef CONFIG_CONSOLE_POLL
static int sci_poll_get_char(struct uart_port *port)
{
	unsigned int status, frsr_status;
	int c;

	do {
		status = serial_port_in(port, CSR);
		if (status & SCxSR_ERRORS(port)) {
			sci_clear_CFC(port, SCxSR_ERROR_CLEAR(port));
			continue;
		}
		break;
	} while (1);

	frsr_status = (port->type == PORT_SCIF) ? serial_port_in(port, FRSR) : 0;
	if (!(status & CSR_RDRF) && !(frsr_status & FRSR_DR))
		return NO_POLL_CHAR;

	c = serial_port_in(port, RDR) & RDR_RDAT_MSK;

	/* Dummy read */
	serial_port_in(port, CSR);
	sci_clear_DRxC(port);

	return c;
}
#endif

static void sci_poll_put_char(struct uart_port *port, unsigned char c)
{
	unsigned int status;

	do {
		status = serial_port_in(port, CSR);
	} while (!(status & CSR_TDRE));

	serial_port_out(port, TDR, c);
	sci_clear_CFC(port, CFCLR_TDREC);
}
#endif /* CONFIG_CONSOLE_POLL || CONFIG_SERIAL_RZ_SCI_CONSOLE ||
	  CONFIG_SERIAL_RZ_SCI_EARLYCON */

static void sci_init_pins(struct uart_port *port, unsigned int cflag)
{
	struct sci_port *s = to_sci_port(port);

	/*
	 * Use port-specific handler if provided.
	 */
	if (s->cfg->ops && s->cfg->ops->init_pins) {
		s->cfg->ops->init_pins(port, cflag);
		return;
	}
}

static int sci_txfill(struct uart_port *port)
{
	if (port->type == PORT_SCIF)
		return serial_port_in(port, FTSR);
	else
		return !(serial_port_in(port, CSR) & CSR_TDRE);
}

static int sci_txroom(struct uart_port *port)
{
	return port->fifosize - sci_txfill(port);
}

static int sci_rxfill(struct uart_port *port)
{
	if (port->type == PORT_SCIF)
		return (serial_port_in(port, FRSR) & FRSR_R5_0) >> 8;
	else
		return (serial_port_in(port, CSR) & CSR_RDRF) != 0;
}

/* ********************************************************************** *
 *                   the interrupt related routines                       *
 * ********************************************************************** */

static void sci_transmit_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int stopped = uart_tx_stopped(port);
	unsigned int status;
	unsigned int ctrl;
	int count;

	status = serial_port_in(port, CSR);
	if (!(status & CSR_TDRE)) {
		ctrl = serial_port_in(port, CCR0);
		if (uart_circ_empty(xmit))
			ctrl &= ~CCR0_TIE;
		else
			ctrl |= CCR0_TIE;
		serial_port_out(port, CCR0, ctrl);
		return;
	}

	count = sci_txroom(port);

	do {
		unsigned char c;

		if (port->x_char) {
			c = port->x_char;
			port->x_char = 0;
		} else if (!uart_circ_empty(xmit) && !stopped) {
			c = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		} else {
			break;
		}

		sci_clear_CFC(port, CFCLR_TDREC);
		serial_port_out(port, TDR, c);

		port->icount.tx++;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
	if (uart_circ_empty(xmit)) {
		ctrl = serial_port_in(port, CCR0);
		ctrl &= ~CCR0_TIE;
		ctrl |= CCR0_TEIE;
		serial_port_out(port, CCR0, ctrl);
	}
}

static void sci_receive_chars(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	int i, count, copied = 0;
	unsigned int status, frsr_status;
	unsigned char flag;

	status = serial_port_in(port, CSR);
	frsr_status = (port->type == PORT_SCIF) ? serial_port_in(port, FRSR) : 0;

	if (!(status & CSR_RDRF) && !(frsr_status & FRSR_DR))
		return;

	while (1) {
		/* Don't copy more bytes than there is room for in the buffer */
		count = tty_buffer_request_room(tport, sci_rxfill(port));

		/* If for any reason we can't copy more data, we're done! */
		if (count == 0)
			break;

		if (port->type == PORT_SCI) {
			char c = serial_port_in(port, RDR) & RDR_RDAT_MSK;

			if (uart_handle_sysrq_char(port, c))
				count = 0;
			else
				tty_insert_flip_char(tport, c, TTY_NORMAL);
		} else {
			for (i = 0; i < count; i++) {
				char c;

				status = serial_port_in(port, CSR);
				c = serial_port_in(port, RDR) & RDR_RDAT_MSK;

				if (uart_handle_sysrq_char(port, c)) {
					count--; i--;
					continue;
				}

				/* Store data and status */
				if (status & CSR_FER) {
					flag = TTY_FRAME;
					port->icount.frame++;
					dev_notice(port->dev, "frame error\n");
				} else if (status & CSR_PER) {
					flag = TTY_PARITY;
					port->icount.parity++;
					dev_notice(port->dev, "parity error\n");
				} else
					flag = TTY_NORMAL;

				tty_insert_flip_char(tport, c, flag);
			}
		}

		serial_port_in(port, CSR); /* dummy read */
		sci_clear_DRxC(port);

		copied += count;
		port->icount.rx += count;
	}

	if (copied) {
		/* Tell the rest of the system the news. New characters! */
		tty_flip_buffer_push(tport);
	} else {
		/* TTY buffers full; read from RX reg to prevent lockup */
		serial_port_in(port, RDR);
		serial_port_in(port, CSR); /* dummy read */
		sci_clear_DRxC(port);
	}
}

static int sci_handle_errors(struct uart_port *port)
{
	int copied = 0;
	unsigned int status = serial_port_in(port, CSR);
	struct tty_port *tport = &port->state->port;

	/* Handle overruns */
	if (status & CSR_ORER) {
		port->icount.overrun++;

		/* overrun error */
		if (tty_insert_flip_char(tport, 0, TTY_OVERRUN))
			copied++;

		dev_notice(port->dev, "overrun error\n");
	}

	if (status & CSR_FER) {
		/* frame error */
		port->icount.frame++;

		if (tty_insert_flip_char(tport, 0, TTY_FRAME))
			copied++;

		dev_notice(port->dev, "frame error\n");
	}

	if (status & CSR_PER) {
		/* parity error */
		port->icount.parity++;

		if (tty_insert_flip_char(tport, 0, TTY_PARITY))
			copied++;

		dev_notice(port->dev, "parity error\n");
	}

	if (copied)
		tty_flip_buffer_push(tport);

	return copied;
}

static int sci_handle_fifo_overrun(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	int copied = 0;
	unsigned int status;


	status = serial_port_in(port, CSR);
	if (status & CSR_ORER) {
		sci_clear_CFC(port,  CFCLR_ORERC);
		port->icount.overrun++;

		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
		tty_flip_buffer_push(tport);

		dev_dbg(port->dev, "overrun error\n");
		copied++;
	}

	return copied;
}

static int scif_set_rtrg(struct uart_port *port, int rx_trig)
{
	unsigned int bits;

	if (port->type != PORT_SCIF) {
		WARN(1, "unknown FIFO configuration");
		return 1;
	}

	if (rx_trig >= port->fifosize)
		rx_trig = port->fifosize - 1;
	else if (rx_trig < 1)
		rx_trig = 1;

	bits = rx_trig << 16;
	serial_port_out(port, FCR, (serial_port_in(port, FCR) & ~FCR_RTRG4_0) | bits);

	return rx_trig;
}

static ssize_t rx_fifo_trigger_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct uart_port *port = dev_get_drvdata(dev);
	struct sci_port *sci = to_sci_port(port);

	return sprintf(buf, "%d\n", sci->rx_trigger);
}

static ssize_t rx_fifo_trigger_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct uart_port *port = dev_get_drvdata(dev);
	struct sci_port *sci = to_sci_port(port);
	int ret;
	long r;

	ret = kstrtol(buf, 0, &r);
	if (ret)
		return ret;

	sci->rx_trigger = scif_set_rtrg(port, r);

	return count;
}

static DEVICE_ATTR_RW(rx_fifo_trigger);

static irqreturn_t sci_rx_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;

	sci_receive_chars(port);

	return IRQ_HANDLED;
}

static irqreturn_t sci_tx_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	sci_transmit_chars(port);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t sci_tx_end_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	unsigned long flags;
	unsigned int ctrl;

	spin_lock_irqsave(&port->lock, flags);
	ctrl = serial_port_in(port, CCR0);
	ctrl &= ~(CCR0_TE | CCR0_TEIE);
	serial_port_out(port, CCR0, ctrl);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t sci_er_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;

	/* Handle errors */
	if (port->type == PORT_SCI) {
		if (sci_handle_errors(port)) {
			/* discard character in rx buffer */
			serial_port_in(port, CSR);
			sci_clear_DRxC(port);
		}
	} else {
		sci_handle_fifo_overrun(port);
		sci_receive_chars(port);
	}

	sci_clear_CFC(port, SCxSR_ERROR_CLEAR(port));

	/* Kick the transmission */
	sci_tx_interrupt(irq, ptr);

	return IRQ_HANDLED;
}

static const struct sci_irq_desc {
	const char	*desc;
	irq_handler_t	handler;
} sci_irq_desc[] = {
	/*
	 * Split out handlers, the default case.
	 */
	[SCIx_ERI_IRQ] = {
		.desc = "rx err",
		.handler = sci_er_interrupt,
	},

	[SCIx_RXI_IRQ] = {
		.desc = "rx full",
		.handler = sci_rx_interrupt,
	},

	[SCIx_TXI_IRQ] = {
		.desc = "tx empty",
		.handler = sci_tx_interrupt,
	},

	[SCIx_TEI_IRQ] = {
		.desc = "tx end",
		.handler = sci_tx_end_interrupt,
	},

};

static int sci_request_irq(struct sci_port *port)
{
	struct uart_port *up = &port->port;
	int i, j, w, ret = 0;

	for (i = j = 0; i < SCIx_NR_IRQS; i++, j++) {
		const struct sci_irq_desc *desc;
		int irq;

		/* Check if already registered (muxed) */
		for (w = 0; w < i; w++)
			if (port->irqs[w] == port->irqs[i])
				w = i + 1;
		if (w > i)
			continue;

		if (SCIx_IRQ_IS_MUXED(port)) {
			i = SCIx_MUX_IRQ;
			irq = up->irq;
		} else {
			irq = port->irqs[i];

			/*
			 * Certain port types won't support all of the
			 * available interrupt sources.
			 */
			if (unlikely(irq < 0))
				continue;
		}

		desc = sci_irq_desc + i;
		port->irqstr[j] = kasprintf(GFP_KERNEL, "%s:%s",
						dev_name(up->dev), desc->desc);
		if (!port->irqstr[j]) {
			ret = -ENOMEM;
			goto out_nomem;
		}

		ret = request_irq(irq, desc->handler, up->irqflags,
				  port->irqstr[j], port);
		if (unlikely(ret)) {
			dev_err(up->dev, "Can't allocate %s IRQ\n", desc->desc);
			goto out_noirq;
		}
	}

	return 0;

out_noirq:
	while (--i >= 0)
		free_irq(port->irqs[i], port);

out_nomem:
	while (--j >= 0)
		kfree(port->irqstr[j]);

	return ret;
}

static void sci_free_irq(struct sci_port *port)
{
	int i, j;

	/*
	 * Intentionally in reverse order so we iterate over the muxed
	 * IRQ first.
	 */
	for (i = 0; i < SCIx_NR_IRQS; i++) {
		int irq = port->irqs[i];

		/*
		 * Certain port types won't support all of the available
		 * interrupt sources.
		 */
		if (unlikely(irq < 0))
			continue;

		/* Check if already freed (irq was muxed) */
		for (j = 0; j < i; j++)
			if (port->irqs[j] == irq)
				j = i + 1;
		if (j > i)
			continue;

		free_irq(port->irqs[i], port);
		kfree(port->irqstr[i]);

		if (SCIx_IRQ_IS_MUXED(port)) {
			/* If there's only one IRQ, we're done. */
			return;
		}
	}
}

static unsigned int sci_tx_empty(struct uart_port *port)
{
	unsigned int status = serial_port_in(port, CSR);
	unsigned int in_tx_fifo = sci_txfill(port);

	return (status & CSR_TEND) && !in_tx_fifo ? TIOCSER_TEMT : 0;
}

static void sci_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	if (mctrl & TIOCM_LOOP) {

		/*
		 * Standard loopback mode.
		 */
		if (port->type == PORT_SCIF)
			serial_port_out(port, CCR1,
					serial_port_in(port, CCR1) |
					CCR1_SPLP);
	}
}

static unsigned int sci_get_mctrl(struct uart_port *port)
{
	struct sci_port *s = to_sci_port(port);
	struct mctrl_gpios *gpios = s->gpios;
	unsigned int mctrl = 0;

	mctrl_gpio_get(gpios, &mctrl);

	/*
	 * CTS/RTS is handled in hardware when supported, while nothing
	 * else is wired up.
	 */
	if (!mctrl_gpio_to_gpiod(gpios, UART_GPIO_CTS))
		mctrl |= TIOCM_CTS;
	if (!mctrl_gpio_to_gpiod(gpios, UART_GPIO_DSR))
		mctrl |= TIOCM_DSR;
	if (!mctrl_gpio_to_gpiod(gpios, UART_GPIO_DCD))
		mctrl |= TIOCM_CAR;

	return mctrl;
}

static void sci_enable_ms(struct uart_port *port)
{
	mctrl_gpio_enable_ms(to_sci_port(port)->gpios);
}

static int sci_startup(struct uart_port *port)
{
	struct sci_port *s = to_sci_port(port);
	int ret;

	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);

	ret = sci_request_irq(s);
	if (unlikely(ret < 0))
		return ret;

	return 0;
}

static void sci_shutdown(struct uart_port *port)
{
	struct sci_port *s = to_sci_port(port);
	unsigned long flags;

	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);

	mctrl_gpio_disable_ms(to_sci_port(port)->gpios);

	spin_lock_irqsave(&port->lock, flags);
	sci_stop_rx(port);
	sci_stop_tx(port);
	/*
	 * Stop RX and TX, disable related interrupts, keep clock source
	 */
	serial_port_out(port, CCR0, 0);
	spin_unlock_irqrestore(&port->lock, flags);

	if (s->rx_trigger > 1 && s->rx_fifo_timeout > 0)
		del_timer_sync(&s->rx_fifo_timer);
	sci_free_irq(s);
}

/* calculate sample rate, BRR, and clock select */
static int sci_scbrr_calc(struct sci_port *s, unsigned int bps,
			  unsigned int *brr, unsigned int *srr,
			  unsigned int *cks)
{
	unsigned long freq = s->clk_rates[SCI_FCK];
	unsigned int sr, br, prediv, scrate, c;
	int err, min_err = INT_MAX;

	freq *= 2;

	/*
	 * Find the combination of sample rate and clock select with the
	 * smallest deviation from the desired baud rate.
	 * Prefer high sample rates to maximise the receive margin.
	 *
	 * M: Receive margin (%)
	 * N: Ratio of bit rate to clock (N = sampling rate)
	 * D: Clock duty (D = 0 to 1.0)
	 * L: Frame length (L = 9 to 12)
	 * F: Absolute value of clock frequency deviation
	 *
	 *  M = |(0.5 - 1 / 2 * N) - ((L - 0.5) * F) -
	 *      (|D - 0.5| / N * (1 + F))|
	 *  NOTE: Usually, treat D for 0.5, F is 0 by this calculation.
	 */
	for_each_sr(sr, s) {
		for (c = 0; c <= 3; c++) {
			prediv = sr << (2 * c + 1);

			/*
			 * We need to calculate:
			 *
			 *     br = freq / (prediv * bps) clamped to [1..256]
			 *     err = freq / (br * prediv) - bps
			 *
			 * Watch out for overflow when calculating the desired
			 * sampling clock rate!
			 */
			if (bps > UINT_MAX / prediv)
				break;

			scrate = prediv * bps;
			br = DIV_ROUND_CLOSEST(freq, scrate);
			br = clamp(br, 1U, 256U);

			err = DIV_ROUND_CLOSEST(freq, br * prediv) - bps;
			if (abs(err) >= abs(min_err))
				continue;

			min_err = err;
			*brr = br - 1;
			*srr = sr - 1;
			*cks = c;

			if (!err)
				goto found;
		}
	}

found:
	dev_dbg(s->port.dev, "BRR: %u%+d bps using N %u SR %u cks %u\n", bps,
		min_err, *brr, *srr + 1, *cks);
	return min_err;
}

static void sci_set_termios(struct uart_port *port, struct ktermios *termios,
				const struct ktermios *old)
{
	unsigned int baud, i, bits;
	unsigned int brr = 255, cks = 0, srr = 15;
	unsigned int brr1 = 255, cks1 = 0, srr1 = 15;
	struct sci_port *s = to_sci_port(port);
	int min_err = INT_MAX, err;
	unsigned long max_freq = 0;
	int best_clk = -1;
	unsigned long flags;
	unsigned int ccr0_val = 0, ccr1_val = 0, ccr4_val = 0;
	unsigned int ccr2_val = CCR2_INIT, ccr3_val = CCR3_INIT;

	if ((termios->c_cflag & CSIZE) == CS7) {
		ccr3_val |= CCR3_CHR0;
	} else {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
	}
	if (termios->c_cflag & PARENB)
		ccr1_val |= CCR1_PE;
	if (termios->c_cflag & PARODD)
		ccr1_val |= (CCR1_PE | CCR1_PM);
	if (termios->c_cflag & CSTOPB)
		ccr3_val |= CCR3_STP;

	/*
	 * earlyprintk comes here early on with port->uartclk set to zero.
	 * the clock framework is not up and running at this point so here
	 * we assume that 115200 is the maximum baud rate. please note that
	 * the baud rate is not programmed during earlyprintk - it is assumed
	 * that the previous boot loader has enabled required clocks and
	 * setup the baud rate generator hardware for us already.
	 */
	if (!port->uartclk) {
		baud = uart_get_baud_rate(port, termios, old, 0, 115200);
		goto done;
	}

	for (i = 0; i < SCI_NUM_CLKS; i++)
		max_freq = max(max_freq, s->clk_rates[i]);

	baud = uart_get_baud_rate(port, termios, old, 0, max_freq / min_sr(s));
	if (!baud)
		goto done;

	/* Divided Functional Clock using standard Bit Rate Register */
	err = sci_scbrr_calc(s, baud, &brr1, &srr1, &cks1);
	if (abs(err) < abs(min_err)) {
		best_clk = SCI_FCK;
		ccr0_val = 0;
		min_err = err;
		brr = brr1;
		srr = srr1;
		cks = cks1;
	}

done:
	if (best_clk >= 0)
		dev_dbg(port->dev, "Using clk %pC for %u%+d bps\n",
			s->clks[best_clk], baud, min_err);

	sci_port_enable(s);

	spin_lock_irqsave(&port->lock, flags);

	uart_update_timeout(port, termios->c_cflag, baud);

	/* byte size and parity */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		bits = 7;
		break;
	case CS6:
		bits = 8;
		break;
	case CS7:
		bits = 9;
		break;
	default:
		bits = 10;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		bits++;
	if (termios->c_cflag & PARENB)
		bits++;


	serial_port_out(port, CCR0, ccr0_val);

	if (port->type == PORT_SCIF)
		ccr3_val |= CCR3_FM;
	serial_port_out(port, CCR3, ccr3_val);

	ccr2_val |= (cks << 20) | (brr << 8);
	serial_port_out(port, CCR2, ccr2_val);

	serial_port_out(port, CCR1, ccr1_val);

	serial_port_out(port, CCR4, ccr4_val);

	if (port->type == PORT_SCIF) {
		unsigned int ctrl = serial_port_in(port, FCR);

		ctrl |= (FCR_RFRST | FCR_TFRST);
		serial_port_out(port, FCR, ctrl);

		if (s->rx_trigger > 1)
			scif_set_rtrg(port, s->rx_trigger);
	}

	sci_init_pins(port, termios->c_cflag);

	port->status &= ~UPSTAT_AUTOCTS;

	serial_port_out(port, CFCLR, CFCLR_CLRFLAG);

	if (port->type == PORT_SCIF)
		serial_port_out(port, FFCLR, FFCLR_DRC);

	ccr0_val |= CCR0_RE;
	serial_port_out(port, CCR0, ccr0_val);

	if ((termios->c_cflag & CREAD) != 0)
		sci_start_rx(port);

	spin_unlock_irqrestore(&port->lock, flags);

	sci_port_disable(s);

	if (UART_ENABLE_MS(port, termios->c_cflag))
		sci_enable_ms(port);
}

static void sci_pm(struct uart_port *port, unsigned int state,
		   unsigned int oldstate)
{
	struct sci_port *sci_port = to_sci_port(port);

	switch (state) {
	case UART_PM_STATE_OFF:
		sci_port_disable(sci_port);
		break;
	default:
		sci_port_enable(sci_port);
		break;
	}
}

static const char *sci_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_SCI:
		return "sci";
	case PORT_SCIF:
		return "scif";
	}

	return NULL;
}

static int sci_remap_port(struct uart_port *port)
{
	struct sci_port *sport = to_sci_port(port);

	/*
	 * Nothing to do if there's already an established membase.
	 */
	if (port->membase)
		return 0;

	if (port->dev->of_node || (port->flags & UPF_IOREMAP)) {
		port->membase = ioremap(port->mapbase, sport->reg_size);
		if (unlikely(!port->membase)) {
			dev_err(port->dev, "can't remap port#%d\n", port->line);
			return -ENXIO;
		}
	} else {
		/*
		 * For the simple (and majority of) cases where we don't
		 * need to do any remapping, just cast the cookie
		 * directly.
		 */
		port->membase = (void __iomem *)(uintptr_t)port->mapbase;
	}

	return 0;
}

static void sci_release_port(struct uart_port *port)
{
	struct sci_port *sport = to_sci_port(port);

	if (port->dev->of_node || (port->flags & UPF_IOREMAP)) {
		iounmap(port->membase);
		port->membase = NULL;
	}

	release_mem_region(port->mapbase, sport->reg_size);
}

static int sci_request_port(struct uart_port *port)
{
	struct resource *res;
	struct sci_port *sport = to_sci_port(port);
	int ret;

	res = request_mem_region(port->mapbase, sport->reg_size,
				 dev_name(port->dev));
	if (unlikely(res == NULL)) {
		dev_err(port->dev, "request_mem_region failed.");
		return -EBUSY;
	}

	ret = sci_remap_port(port);
	if (unlikely(ret != 0)) {
		release_resource(res);
		return ret;
	}

	return 0;
}

static void sci_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		struct sci_port *sport = to_sci_port(port);

		port->type = sport->cfg->type;
		sci_request_port(port);
	}
}

static int sci_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->baud_base < 2400)
		/* No paper tape reader for Mitch.. */
		return -EINVAL;

	return 0;
}

static const struct uart_ops sci_uart_ops = {
	.tx_empty	= sci_tx_empty,
	.set_mctrl	= sci_set_mctrl,
	.get_mctrl	= sci_get_mctrl,
	.start_tx	= sci_start_tx,
	.stop_tx	= sci_stop_tx,
	.stop_rx	= sci_stop_rx,
	.enable_ms	= sci_enable_ms,
	.startup	= sci_startup,
	.shutdown	= sci_shutdown,
	.set_termios	= sci_set_termios,
	.pm		= sci_pm,
	.type		= sci_type,
	.release_port	= sci_release_port,
	.request_port	= sci_request_port,
	.config_port	= sci_config_port,
	.verify_port	= sci_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= sci_poll_get_char,
	.poll_put_char	= sci_poll_put_char,
#endif
};

static int sci_init_clocks(struct sci_port *sci_port, struct device *dev)
{
	const char *clk_names[] = {
		[SCI_FCK] = "fck",
	};
	struct clk *clk;
	unsigned int i;

	for (i = 0; i < SCI_NUM_CLKS; i++) {
		clk = devm_clk_get_optional(dev, clk_names[i]);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		if (!clk)
			dev_dbg(dev, "failed to get %s\n", clk_names[i]);
		else
			dev_dbg(dev, "clk %s is %pC rate %lu\n", clk_names[i],
				clk, clk_get_rate(clk));
		sci_port->clks[i] = clk;
	}
	return 0;
}

static const struct sci_port_params *
sci_probe_regmap(const struct plat_sci_port *cfg)
{
	unsigned int regtype;

	if (cfg->regtype != SCIx_PROBE_REGTYPE)
		return &sci_port_params[cfg->regtype];

	switch (cfg->type) {
	case PORT_SCI:
		regtype = SCIx_SCI_REGTYPE;
		break;
	case PORT_SCIF:
		regtype = SCIx_RZ_SCIFA_REGTYPE;
		break;
	default:
		pr_err("Can't probe register map for given port\n");
		return NULL;
	}

	return &sci_port_params[regtype];
}

static int sci_init_single(struct platform_device *dev,
			   struct sci_port *sci_port, unsigned int index,
			   const struct plat_sci_port *p, bool early)
{
	struct uart_port *port = &sci_port->port;
	const struct resource *res;
	unsigned int i;
	int ret;

	sci_port->cfg	= p;

	port->ops	= &sci_uart_ops;
	port->iotype	= UPIO_MEM;
	port->line	= index;
	port->has_sysrq = IS_ENABLED(CONFIG_SERIAL_RZ_SCI_CONSOLE);

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENOMEM;

	port->mapbase = res->start;
	sci_port->reg_size = resource_size(res);

	for (i = 0; i < ARRAY_SIZE(sci_port->irqs); ++i) {
		if (i)
			sci_port->irqs[i] = platform_get_irq_optional(dev, i);
		else
			sci_port->irqs[i] = platform_get_irq(dev, i);
	}

	if (sci_port->irqs[0] < 0)
		return -ENXIO;

	if (sci_port->irqs[1] < 0)
		for (i = 1; i < ARRAY_SIZE(sci_port->irqs); i++)
			sci_port->irqs[i] = sci_port->irqs[0];

	sci_port->params = sci_probe_regmap(p);
	if (unlikely(sci_port->params == NULL))
		return -EINVAL;

	switch (p->type) {
	case PORT_SCIF:
		sci_port->rx_trigger = 15;
		break;
	default:
		sci_port->rx_trigger = 1;
		break;
	}

	sci_port->rx_fifo_timeout = 0;

	sci_port->sampling_rate_mask = p->sampling_rate
				     ? SCI_SR(p->sampling_rate)
				     : sci_port->params->sampling_rate_mask;

	if (!early) {
		ret = sci_init_clocks(sci_port, &dev->dev);
		if (ret < 0)
			return ret;

		port->dev = &dev->dev;

		pm_runtime_enable(&dev->dev);
	}

	port->type		= p->type;
	port->flags		= UPF_FIXED_PORT | UPF_BOOT_AUTOCONF | p->flags;
	port->fifosize		= sci_port->params->fifosize;

	/*
	 * The UART port needs an IRQ value, so we peg this to the RX IRQ
	 * for the multi-IRQ ports, which is where we are primarily
	 * concerned with the shutdown path synchronization.
	 *
	 * For the muxed case there's nothing more to do.
	 */
	port->irq		= sci_port->irqs[SCIx_RXI_IRQ];
	port->irqflags		= 0;

	port->serial_in		= sci_serial_in;
	port->serial_out	= sci_serial_out;

	return 0;
}

static void sci_cleanup_single(struct sci_port *port)
{
	pm_runtime_disable(port->port.dev);
}

#if defined(CONFIG_SERIAL_RZ_SCI_CONSOLE) || \
	defined(CONFIG_SERIAL_RZ_SCI_EARLYCON)
static void serial_console_putchar(struct uart_port *port, unsigned char ch)
{
	sci_poll_put_char(port, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 */
static void serial_console_write(struct console *co, const char *s,
				 unsigned int count)
{
	struct sci_port *sci_port = &sci_ports[co->index];
	struct uart_port *port = &sci_port->port;
	unsigned int bits, ctrl, ctrl_temp;
	unsigned long flags;
	int locked = 1;

	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&port->lock, flags);
	else
		spin_lock_irqsave(&port->lock, flags);

	/* first save CCR0 then disable interrupts, keep clock source */
	ctrl = serial_port_in(port, CCR0);
	ctrl_temp = CCR0_RE | CCR0_TE | CCR0_TIE;
	serial_port_out(port, CCR0, ctrl_temp);

	uart_console_write(port, s, count, serial_console_putchar);

	/* wait until fifo is empty and last bit has been transmitted */
	bits = CSR_TDRE | CSR_TEND;
	while ((serial_port_in(port, CSR) & bits) != bits)
		cpu_relax();

	/* restore the CCR0 */
	serial_port_out(port, CCR0, ctrl);

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

static int serial_console_setup(struct console *co, char *options)
{
	struct sci_port *sci_port;
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	/*
	 * Refuse to handle any bogus ports.
	 */
	if (co->index < 0 || co->index >= SCI_NPORTS)
		return -ENODEV;

	sci_port = &sci_ports[co->index];
	port = &sci_port->port;

	/*
	 * Refuse to handle uninitialized ports.
	 */
	if (!port->ops)
		return -ENODEV;

	ret = sci_remap_port(port);
	if (unlikely(ret != 0))
		return ret;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console serial_console = {
	.name		= "ttySC",
	.device		= uart_console_device,
	.write		= serial_console_write,
	.setup		= serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &sci_uart_driver,
};


#define SCI_CONSOLE	(&serial_console)

#endif /* CONFIG_SERIAL_RZ_SCI_CONSOLE || CONFIG_SERIAL_RZ_SCI_EARLYCON */

static const char banner[] __initconst = "Renesas SCI(F) driver initialized";

static DEFINE_MUTEX(sci_uart_registration_lock);
static struct uart_driver sci_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "rsci",
	.dev_name	= "ttySC",
	.major		= SCI_MAJOR,
	.minor		= SCI_MINOR_START,
	.nr		= SCI_NPORTS,
	.cons		= SCI_CONSOLE,
};

static int sci_remove(struct platform_device *dev)
{
	struct sci_port *port = platform_get_drvdata(dev);

	sci_ports_in_use &= ~BIT(port->port.line);
	uart_remove_one_port(&sci_uart_driver, &port->port);

	sci_cleanup_single(port);

	if (port->port.fifosize > 1)
		device_remove_file(&dev->dev, &dev_attr_rx_fifo_trigger);

	return 0;
}

#define SCI_OF_DATA(type, regtype)	((void *)((type) << 16 | (regtype)))
#define SCI_OF_TYPE(data)		((unsigned long)(data) >> 16)
#define SCI_OF_REGTYPE(data)		((unsigned long)(data) & 0xffff)

static const struct of_device_id of_sci_match[] __maybe_unused = {
	/* Generic types */
	{
		.compatible = "renesas,rz-rsci",
		.data = SCI_OF_DATA(PORT_SCI, SCIx_SCI_REGTYPE),
	},
	{
		.compatible = "renesas,rz-rscif",
		.data = SCI_OF_DATA(PORT_SCIF, SCIx_RZ_SCIFA_REGTYPE),
	},
	{
		/* Terminator */
	},
};
MODULE_DEVICE_TABLE(of, of_sci_match);

static void sci_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static struct plat_sci_port *sci_parse_dt(struct platform_device *pdev,
					  unsigned int *dev_id)
{
	struct device_node *np = pdev->dev.of_node;
	struct reset_control *rstc;
	struct plat_sci_port *p;
	struct sci_port *sp;
	const void *data;
	int id, ret;

	if (!IS_ENABLED(CONFIG_OF) || !np)
		return ERR_PTR(-EINVAL);

	data = of_device_get_match_data(&pdev->dev);

	rstc = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(rstc))
		return ERR_PTR(dev_err_probe(&pdev->dev, PTR_ERR(rstc),
						 "failed to get reset ctrl\n"));

	ret = reset_control_deassert(rstc);
	if (ret) {
		dev_err(&pdev->dev, "failed to deassert reset %d\n", ret);
		return ERR_PTR(ret);
	}

	ret = devm_add_action_or_reset(&pdev->dev, sci_reset_control_assert, rstc);
	if (ret) {
		dev_err(&pdev->dev, "failed to register assert devm action, %d\n",
			ret);
		return ERR_PTR(ret);
	}

	p = devm_kzalloc(&pdev->dev, sizeof(struct plat_sci_port), GFP_KERNEL);
	if (!p)
		return ERR_PTR(-ENOMEM);

	/* Get the line number from the aliases node. */
	id = of_alias_get_id(np, "serial");
	if (id < 0 && ~sci_ports_in_use)
		id = ffz(sci_ports_in_use);
	if (id < 0) {
		dev_err(&pdev->dev, "failed to get alias id (%d)\n", id);
		return ERR_PTR(-EINVAL);
	}
	if (id >= ARRAY_SIZE(sci_ports)) {
		dev_err(&pdev->dev, "serial%d out of range\n", id);
		return ERR_PTR(-EINVAL);
	}

	sp = &sci_ports[id];
	*dev_id = id;

	p->type = SCI_OF_TYPE(data);
	p->regtype = SCI_OF_REGTYPE(data);

	return p;
}

static int sci_probe_single(struct platform_device *dev,
					  unsigned int index,
					  struct plat_sci_port *p,
					  struct sci_port *sciport)
{
	int ret;

	/* Sanity check */
	if (unlikely(index >= SCI_NPORTS)) {
		dev_notice(&dev->dev, "Attempting to register port %d when only %d are available\n",
			   index + 1, SCI_NPORTS);
		dev_notice(&dev->dev, "Consider bumping CONFIG_SERIAL_RZ_SCI_NR_UARTS!\n");
		return -EINVAL;
	}
	BUILD_BUG_ON(SCI_NPORTS > sizeof(sci_ports_in_use) * 8);
	if (sci_ports_in_use & BIT(index))
		return -EBUSY;

	mutex_lock(&sci_uart_registration_lock);
	if (!sci_uart_driver.state) {
		ret = uart_register_driver(&sci_uart_driver);
		if (ret) {
			mutex_unlock(&sci_uart_registration_lock);
			return ret;
		}
	}
	mutex_unlock(&sci_uart_registration_lock);

	ret = sci_init_single(dev, sciport, index, p, false);
	if (ret)
		return ret;

	sciport->gpios = mctrl_gpio_init(&sciport->port, 0);
	if (IS_ERR(sciport->gpios))
		return PTR_ERR(sciport->gpios);

	ret = uart_add_one_port(&sci_uart_driver, &sciport->port);
	if (ret) {
		sci_cleanup_single(sciport);
		return ret;
	}

	return 0;
}

static int sci_probe(struct platform_device *dev)
{
	struct plat_sci_port *p;
	struct sci_port *sp;
	unsigned int dev_id;
	int ret;

	if (dev->dev.of_node) {
		p = sci_parse_dt(dev, &dev_id);
		if (IS_ERR(p))
			return PTR_ERR(p);
	} else {
		p = dev->dev.platform_data;
		if (p == NULL) {
			dev_err(&dev->dev, "no platform data supplied\n");
			return -EINVAL;
		}

		dev_id = dev->id;
	}

	sp = &sci_ports[dev_id];
	platform_set_drvdata(dev, sp);

	ret = sci_probe_single(dev, dev_id, p, sp);
	if (ret)
		return ret;

	if (sp->port.fifosize > 1) {
		ret = device_create_file(&dev->dev, &dev_attr_rx_fifo_trigger);
		if (ret)
			return ret;
	}

	sci_ports_in_use |= BIT(dev_id);
	return 0;
}

static __maybe_unused int sci_suspend(struct device *dev)
{
	struct sci_port *sport = dev_get_drvdata(dev);

	if (sport)
		uart_suspend_port(&sci_uart_driver, &sport->port);

	return 0;
}

static __maybe_unused int sci_resume(struct device *dev)
{
	struct sci_port *sport = dev_get_drvdata(dev);

	if (sport)
		uart_resume_port(&sci_uart_driver, &sport->port);

	return 0;
}

static SIMPLE_DEV_PM_OPS(sci_dev_pm_ops, sci_suspend, sci_resume);

static struct platform_driver sci_driver = {
	.probe		= sci_probe,
	.remove		= sci_remove,
	.driver		= {
		.name	= "rz-sci",
		.pm	= &sci_dev_pm_ops,
		.of_match_table = of_match_ptr(of_sci_match),
	},
};

static int __init sci_init(void)
{
	pr_info("%s\n", banner);

	return platform_driver_register(&sci_driver);
}

static void __exit sci_exit(void)
{
	platform_driver_unregister(&sci_driver);

	if (sci_uart_driver.state)
		uart_unregister_driver(&sci_uart_driver);
}

#ifdef CONFIG_SERIAL_RZ_SCI_EARLYCON
static struct plat_sci_port port_cfg __initdata;

static int __init early_console_setup(struct earlycon_device *device,
					  int type)
{
	if (!device->port.membase)
		return -ENODEV;

	device->port.serial_in = sci_serial_in;
	device->port.serial_out	= sci_serial_out;
	device->port.type = type;
	memcpy(&sci_ports[0].port, &device->port, sizeof(struct uart_port));
	port_cfg.type = type;
	sci_ports[0].cfg = &port_cfg;
	sci_ports[0].params = sci_probe_regmap(&port_cfg);
	port_cfg.scscr = sci_serial_in(&sci_ports[0].port, CCR0);
	sci_serial_out(&sci_ports[0].port, CCR0,
			   CCR0_RE | CCR0_TE | port_cfg.scscr);

	device->con->write = serial_console_write;
	return 0;
}
static int __init sci_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	return early_console_setup(device, PORT_SCI);
}
static int __init scif_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	return early_console_setup(device, PORT_SCIF);
}

OF_EARLYCON_DECLARE(sci, "renesas,rz-rsci", sci_early_console_setup);
OF_EARLYCON_DECLARE(sci, "renesas,rz-rscif", scif_early_console_setup);
#endif /* CONFIG_SERIAL_RZ_SCI_EARLYCON */

module_init(sci_init);
module_exit(sci_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rz-sci");
MODULE_AUTHOR("Linh Hua <linh.hua.jc@renesas.com>");
MODULE_DESCRIPTION("Renesas Serial Communication Interface (SCI) driver");
