// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas On-Chip Realtime Clock RTCA-3 Support
 *
 * Based on drivers/rtc/rtc-sh.c
 *
 */
#include <linux/module.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/time.h>

/* Default values for RZ/G3S RTC */
#define rtc_reg_size		sizeof(u16)  // rtc reg_size = 2
#define RTC_BIT_INVERTED	0	/* no chip bugs */
#define RTC_CAP_4_DIGIT_YEAR	(0 << 0)
#define RTC_DEF_CAPABILITIES	RTC_CAP_4_DIGIT_YEAR

#define DRV_NAME	"renesas-rtca-3"

#define RTC_REG(r)	((r) * rtc_reg_size)

#define R64CNT		RTC_REG(0)

#define RSECCNT		RTC_REG(1)	/* RTC sec */
#define RMINCNT		RTC_REG(2)	/* RTC min */
#define RHRCNT		RTC_REG(3)	/* RTC hour */
#define RWKCNT		RTC_REG(4)	/* RTC week */
#define RDAYCNT		RTC_REG(5)	/* RTC day */
#define RMONCNT		RTC_REG(6)	/* RTC month */
#define RYRCNT		RTC_REG(7)	/* RTC year */
#define RSECAR		RTC_REG(8)	/* ALARM sec */
#define RMINAR		RTC_REG(9)	/* ALARM min */
#define RHRAR		RTC_REG(10)	/* ALARM hour */
#define RWKAR		RTC_REG(11)	/* ALARM week */
#define RDAYAR		RTC_REG(12)	/* ALARM day */
#define RMONAR		RTC_REG(13)	/* ALARM month */
#define RYRAR		RTC_REG(14)     /* ALARM year */
#define RYRAREN		RTC_REG(15)     /* ALARM enable year */
#define RSR		RTC_REG(16)	/* Status */

#define RCR1		RTC_REG(17)	/* Control */
#define RCR2		RTC_REG(18)	/* Control */

#define RADJ		RTC_REG(23)	/* Time error adj registor */
#define RTCCR0		RTC_REG(32)     /* Time Capture ctrl registor */
#define RSECCP0		RTC_REG(41)     /* Sec Capture registor */
#define RMINCP0		RTC_REG(42)     /* Min Capture registor */
#define RHRCP0		RTC_REG(43)     /* Hour Capture registor */
#define RDAYCP0		RTC_REG(45)     /* Day Capture registor */
#define RMONCP0		RTC_REG(46)     /* Mon Capture registor */

/*
 * Note on RYRAR: Up until this point most of the register
 * definitions are consistent across all of the available parts. However,
 * the placement of the optional RYRAR (the RYRAR control
 * register used to control RYRCNT/RYRAR compare) varies considerably
 * across various parts, occasionally being mapped in to a completely
 * unrelated address space. For proper RYRAR support a separate resource
 * would have to be handed off, but as this is purely optional in
 * practice, we simply opt not to support it, thereby keeping the code
 * quite a bit more simplified.
 */

/* ALARM Bits - or with BCD encoded value */
#define AR_ENB		0x80	/* Enable for alarm cmp   */

/* Period Bits */
#define PF_HP		0x100	/* Enable Half Period to support 8,32,128Hz */
#define PF_COUNT	0x200	/* Half periodic counter */
#define PF_OXS		0x400	/* Periodic One x Second */
#define PF_KOU		0x800	/* Kernel or User periodic request 1=kernel */
#define PF_MASK		0xf00

/* RSR Bits */
#define RSR_PF		0x04
#define RSR_CF		0x02
#define RSR_AF		0x01

/* RCR1 Bits */
#define RCR1_PES	0xF0	/* Periodic Interrupt Select*/
#define RCR1_PIE	0x04	/* Periodic Interrupt Enable*/
#define RCR1_CIE	0x02	/* Carry Interrupt Enable   */
#define RCR1_AIE	0x01	/* Alarm Interrupt Enable   */

/* RCR2 Bits */
#define RCR2_CNTMD	0x80	/* Count Mode Select/ Binary count mode*/
#define RCR2_HR24	0x40	/* Hours Mode */
#define RCR2_AADJP	0x20	/* Automatic Adjustment Period Select */
#define RCR2_AADJE	0x10	/* Automatic Adjustment Enable */
#define RCR2_ADJ	0x04	/* ADJustment (30-second)  */
#define RCR2_RESET	0x02	/* Reset bit	       */
#define RCR2_START	0x01	/* Start bit	       */

/* RTCCR0 Bits */
#define RTCCR0_TCCT0	0x01	/* Time capture ctr0		*/
#define RTCCR0_TCCT1	0x02	/* Time capture ctr1		*/
#define RTCCR0_TCST	0x04	/* Time capture status		*/
#define RTCCR0_TCNF	0x00	/* Noise filter ctr0		*/
#define RTCCR0_TCEN	0x80    /* Time capture enable		*/

struct rtca3_priv {
	void __iomem		*regbase;
	unsigned long		regsize;
	struct resource		*res;
	int			alarm_irq;
	int			periodic_irq;
	int			carry_irq;
	struct clk		*clk;
	struct rtc_device	*rtc_dev;
	spinlock_t		lock;
	unsigned long		capabilities;	/* See asm/rtc.h for cap bits */
	unsigned short		periodic_freq;
	int count_mode;
};

static int rtca3_irq(struct rtca3_priv *rtc)
{
	unsigned int pending;
	u8 tmp;

	udelay(80);
	tmp = readb(rtc->regbase + RSR);
	pending = tmp & RSR_CF;
	tmp &= ~RSR_CF;
	writeb(tmp, rtc->regbase + RSR);

	/* Users have requested One x Second IRQ */
	if (pending && rtc->periodic_freq & PF_OXS)
		rtc_update_irq(rtc->rtc_dev, 1, RTC_UF | RTC_IRQF);

	return pending;
}

static int rtca3_alr(struct rtca3_priv *rtc)
{
	unsigned int pending;
	u8 tmp;

	tmp = readb(rtc->regbase + RSR);
	pending = tmp & RSR_AF;
	tmp &= ~RSR_AF;
	writeb(tmp, rtc->regbase + RCR1);

	tmp = readb(rtc->regbase + RSR);
	tmp &= ~RCR1_AIE;
	writeb(tmp, rtc->regbase + RCR1);

	if (pending)
		rtc_update_irq(rtc->rtc_dev, 1, RTC_AF | RTC_IRQF);

	return pending;
}

static int rtca3_per(struct rtca3_priv *rtc)
{
	unsigned int pending;
	u8 tmp;

	tmp = readb(rtc->regbase + RSR);
	pending = tmp & RSR_PF;
	tmp &= ~RSR_PF;
	writeb(tmp, rtc->regbase + RSR);

	if (!pending)
		return 0;

	/* Half period enabled than one skipped and the next notified */
	if ((rtc->periodic_freq & PF_HP) && (rtc->periodic_freq & PF_COUNT))
		rtc->periodic_freq &= ~PF_COUNT;
	else {
		if (rtc->periodic_freq & PF_HP)
			rtc->periodic_freq |= PF_COUNT;
		rtc_update_irq(rtc->rtc_dev, 1, RTC_PF | RTC_IRQF);
	}

	return pending;
}

static irqreturn_t rtca3_interrupt(int irq, void *dev_id)
{
	struct rtca3_priv *rtc = dev_id;
	int ret;

	spin_lock(&rtc->lock);
	ret = rtca3_irq(rtc);
	spin_unlock(&rtc->lock);

	return IRQ_RETVAL(ret);
}

static irqreturn_t rtca3_alarm(int irq, void *dev_id)
{
	struct rtca3_priv *rtc = dev_id;
	int ret;

	spin_lock(&rtc->lock);
	ret = rtca3_alr(rtc);
	spin_unlock(&rtc->lock);

	return IRQ_RETVAL(ret);
}

static irqreturn_t rtca3_periodic(int irq, void *dev_id)
{
	struct rtca3_priv *rtc = dev_id;
	int ret;

	spin_lock(&rtc->lock);
	ret = rtca3_per(rtc);
	spin_unlock(&rtc->lock);

	return IRQ_RETVAL(ret);
}

static irqreturn_t rtca3_shared(int irq, void *dev_id)
{
	struct rtca3_priv *rtc = dev_id;
	int ret;

	spin_lock(&rtc->lock);
	ret = rtca3_irq(rtc);
	ret |= rtca3_alr(rtc);
	ret |= rtca3_per(rtc);
	spin_unlock(&rtc->lock);

	return IRQ_RETVAL(ret);
}

static inline void rtca3_setaie(struct device *dev, unsigned int enable)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);
	u8 tmp;

	spin_lock_irq(&rtc->lock);
	tmp = readb(rtc->regbase + RCR1);
	if (enable)
		tmp |= RCR1_AIE;
	else
		tmp &= ~RCR1_AIE;

	writeb(tmp, rtc->regbase + RCR1);
	spin_unlock_irq(&rtc->lock);
}

static int rtca3_proc(struct device *dev, struct seq_file *seq)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);
	u8 tmp;

	tmp = readb(rtc->regbase + RCR1);
	seq_printf(seq, "carry_IRQ\t: %s\n", (tmp & RCR1_CIE) ? "yes" : "no");

	tmp = readb(rtc->regbase + RCR1);
	seq_printf(seq, "periodic_IRQ\t: %s\n",
		   (tmp & RCR1_PES) ? "yes" : "no");

	return 0;
}

static inline void rtca3_setcie(struct device *dev, unsigned int enable)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);
	u8 tmp;

	spin_lock_irq(&rtc->lock);

	tmp = readb(rtc->regbase + RCR1);
	if (!enable)
		tmp &= ~RCR1_CIE;
	else
		tmp |= RCR1_CIE;

	writeb(tmp, rtc->regbase + RCR1);

	spin_unlock_irq(&rtc->lock);
}

static int rtca3_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	rtca3_setaie(dev, enabled);
	return 0;
}

static int rtca3_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);
	unsigned int sec128, sec2, yr, yr100, cf_bit;

	udelay(80);

	do {
		u8 tmp;

		spin_lock_irq(&rtc->lock);

		tmp = readb(rtc->regbase + RSR);
		tmp &= ~RSR_CF; /* Clear CF-bit */
		writeb(tmp, rtc->regbase + RSR);

		tmp = readb(rtc->regbase + RCR1);
		tmp |= RCR1_CIE;
		writeb(tmp, rtc->regbase + RCR1);

		sec128 = readb(rtc->regbase + R64CNT);
		/* Check binary count mode */
		if (rtc->count_mode) {
			tm->tm_sec	= readb(rtc->regbase + RSECCNT);
			tm->tm_min	= readb(rtc->regbase + RMINCNT);
			tm->tm_hour	= readb(rtc->regbase + RHRCNT);
			tm->tm_wday	= readb(rtc->regbase + RWKCNT);
			tm->tm_mday	= 1;
			tm->tm_mon	= 1;
		} else {
			tm->tm_sec	= bcd2bin(readb(rtc->regbase + RSECCNT));
			tm->tm_min	= bcd2bin(readb(rtc->regbase + RMINCNT));
			tm->tm_hour	= bcd2bin(readb(rtc->regbase + RHRCNT));
			tm->tm_wday	= bcd2bin(readb(rtc->regbase + RWKCNT));
			tm->tm_mday	= bcd2bin(readb(rtc->regbase + RDAYCNT));
			tm->tm_mon	= bcd2bin(readb(rtc->regbase + RMONCNT)) - 1;

		}

		if (rtc->capabilities & RTC_CAP_4_DIGIT_YEAR) {
			yr  = readw(rtc->regbase + RYRCNT);
			yr100 = bcd2bin(yr >> 8);
			yr &= 0xff;
		} else {
			yr  = readw(rtc->regbase + RYRCNT);
			yr100 = bcd2bin((yr == 0x99) ? 0x19 : 0x20);
		}

		tm->tm_year = (yr100 * 100 + bcd2bin(yr)) - 1900;

		sec2 = readb(rtc->regbase + R64CNT);
		cf_bit = readb(rtc->regbase + RSR) & RSR_CF;

		spin_unlock_irq(&rtc->lock);
	} while (cf_bit != 0 || ((sec128 ^ sec2) & RTC_BIT_INVERTED) != 0);

#if RTC_BIT_INVERTED != 0
	if ((sec128 & RTC_BIT_INVERTED))
		tm->tm_sec--;
#endif

	/* only keep the carry interrupt enabled if UIE is on */
	if (!(rtc->periodic_freq & PF_OXS))
		rtca3_setcie(dev, 0);

	dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon + 1, tm->tm_year, tm->tm_wday);

	return 0;
}

static int rtca3_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);
	int year;

	spin_lock_irq(&rtc->lock);

	/* Stop RTC */
	writeb(0x00, rtc->regbase + RCR2);

	/* check binary count mode*/
	if (rtc->count_mode) {
		writeb(tm->tm_sec,  rtc->regbase + RSECCNT);
		writeb(tm->tm_min,  rtc->regbase + RMINCNT);
		writeb(tm->tm_hour, rtc->regbase + RHRCNT);
		writeb(tm->tm_wday, rtc->regbase + RWKCNT);
		writeb(1, rtc->regbase + RDAYCNT);
		writeb(1, rtc->regbase + RMONCNT);
	} else {
		writeb(bin2bcd(tm->tm_sec),  rtc->regbase + RSECCNT);
		writeb(bin2bcd(tm->tm_min),  rtc->regbase + RMINCNT);
		writeb(bin2bcd(tm->tm_hour), rtc->regbase + RHRCNT);
		writeb(bin2bcd(tm->tm_wday), rtc->regbase + RWKCNT);
		writeb(bin2bcd(tm->tm_mday), rtc->regbase + RDAYCNT);
		writeb(bin2bcd(tm->tm_mon + 1), rtc->regbase + RMONCNT);
	}

	if (rtc->capabilities & RTC_CAP_4_DIGIT_YEAR) {
		year = (bin2bcd((tm->tm_year + 1900) / 100) << 8) |
			bin2bcd(tm->tm_year % 100);
		writew(year, rtc->regbase + RYRCNT);
	} else {
		year = tm->tm_year % 100;
		writew(bin2bcd(year), rtc->regbase + RYRCNT);
	}

	/* Start RTC */
	if (rtc->count_mode)
		writeb(0x81, rtc->regbase + RCR2);
	else
		writeb(0x01, rtc->regbase + RCR2);

	spin_unlock_irq(&rtc->lock);

	return 0;
}

static inline int rtca3_read_alarm_value(struct rtca3_priv *rtc, int reg_off)
{
	unsigned int byte;
	int value = -1;		/* return -1 for ignored values */

	byte = readb(rtc->regbase + reg_off);
	if (rtc->count_mode) {
		value = byte;
	} else if (byte & AR_ENB) {
		byte &= ~AR_ENB;	/* strip the enable bit */
		value = bcd2bin(byte);
	}

	return value;
}

static int rtca3_read_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;

	spin_lock_irq(&rtc->lock);

	tm->tm_sec	= rtca3_read_alarm_value(rtc, RSECAR);
	tm->tm_min	= rtca3_read_alarm_value(rtc, RMINAR);
	tm->tm_hour	= rtca3_read_alarm_value(rtc, RHRAR);
	tm->tm_wday	= rtca3_read_alarm_value(rtc, RWKAR);
	tm->tm_mday	= rtca3_read_alarm_value(rtc, RDAYAR);
	tm->tm_mon	= rtca3_read_alarm_value(rtc, RMONAR);
	if (tm->tm_mon > 0)
		tm->tm_mon -= 1; /* RTC is 1-12, tm_mon is 0-11 */

	wkalrm->enabled = (readb(rtc->regbase + RCR1) & RCR1_AIE) ? 1 : 0;

	spin_unlock_irq(&rtc->lock);

	return 0;
}

static inline void rtca3_write_alarm_value(struct rtca3_priv *rtc, int value, int reg_off)
{
	/* < 0 for a value that is ignored */
	if (value < 0)
		writeb(0, rtc->regbase + reg_off);
	else {
		if (rtc->count_mode)
			writeb(value,  rtc->regbase + reg_off);
		else
			writeb(bin2bcd(value) | AR_ENB,  rtc->regbase + reg_off);
	}
}

static int rtca3_set_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);
	u8 rcr1;
	struct rtc_time *tm = &wkalrm->time;
	int mon;

	spin_lock_irq(&rtc->lock);

	/* disable alarm interrupt and clear the alarm flag */
	rcr1 = readb(rtc->regbase + RSR);
	rcr1 &= ~RSR_AF;
	writeb(rcr1, rtc->regbase + RSR);

	rcr1 = readb(rtc->regbase + RCR1);
	rcr1 &= ~RCR1_AIE;
	writeb(rcr1, rtc->regbase + RCR1);

	/* set alarm time */
	rtca3_write_alarm_value(rtc, tm->tm_sec,  RSECAR);
	rtca3_write_alarm_value(rtc, tm->tm_min,  RMINAR);
	rtca3_write_alarm_value(rtc, tm->tm_hour, RHRAR);
	rtca3_write_alarm_value(rtc, tm->tm_wday, RWKAR);
	rtca3_write_alarm_value(rtc, tm->tm_mday, RDAYAR);
	mon = tm->tm_mon;
	if (mon >= 0)
		mon += 1;
	rtca3_write_alarm_value(rtc, mon, RMONAR);

	if (wkalrm->enabled) {
		rcr1 |= RCR1_AIE;
		writeb(rcr1, rtc->regbase + RCR1);
	}

	spin_unlock_irq(&rtc->lock);

	return 0;
}

static int rtca3_set_start(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct rtca3_priv *rtc = dev_get_drvdata(&pdev->dev);
	u32 val, reg;
	int ret;

	if (!of_property_read_u32(np, "binary_cntmd", &val))
		rtc->count_mode = val;
	else
		rtc->count_mode = 0;

	/* Set RTC running for count mode */
	if (rtc->count_mode) {
		/* Wait for RTC stop completed */
		writeb(0x80, rtc->regbase + RCR2);
		ret = readb_poll_timeout(rtc->regbase + RCR2, reg,
					 !(reg & RCR2_START), 10, 200);
		if (ret)
			return ret;

		/* Set count mode and execute reset */
		writeb(0x82, rtc->regbase + RCR2);
		ret = readb_poll_timeout(rtc->regbase + RCR2, reg,
					 !(reg & RCR2_RESET), 10, 200);
		if (ret)
			return ret;

		/* Wait for RTC start completed  */
		writeb(0x81, rtc->regbase + RCR2);
		ret = readb_poll_timeout(rtc->regbase + RCR2, reg,
					 reg & RCR2_START, 10, 200);
		if (ret)
			return ret;
	} else {
		/* Wait for RTC stop completed */
		writeb(0x00, rtc->regbase + RCR2);
		ret = readb_poll_timeout(rtc->regbase + RCR2, reg,
					 !(reg & RCR2_START), 10, 200);
		if (ret)
			return ret;

		/* Set count mode and execute reset */
		writeb(0x02, rtc->regbase + RCR2);
		ret = readb_poll_timeout(rtc->regbase + RCR2, reg,
					 !(reg & RCR2_RESET), 10, 200);
		if (ret)
			return ret;

		/* Wait for RTC start completed  */
		writeb(0x01, rtc->regbase + RCR2);
		ret = readb_poll_timeout(rtc->regbase + RCR2, reg,
					 reg & RCR2_START, 10, 200);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct rtc_class_ops rtca3_ops = {
	.read_time	= rtca3_read_time,
	.set_time	= rtca3_set_time,
	.read_alarm	= rtca3_read_alarm,
	.set_alarm	= rtca3_set_alarm,
	.proc		= rtca3_proc,
	.alarm_irq_enable = rtca3_alarm_irq_enable,
};

static int rtca3_probe(struct platform_device *pdev)
{
	struct rtca3_priv *rtc;
	struct resource *res;
	char clk_name[6];
	int clk_id, ret;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	spin_lock_init(&rtc->lock);
	/* get periodic/carry/alarm irqs */
	ret = platform_get_irq_byname(pdev, "period");
	if (unlikely(ret <= 0)) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		return -ENOENT;
	}
	rtc->periodic_irq = ret;

	rtc->carry_irq = platform_get_irq_byname(pdev, "carry");
	if (unlikely(rtc->periodic_irq <= 0))
		dev_warn(&pdev->dev, "Carry interrupt is not set\n");

	rtc->alarm_irq = platform_get_irq_byname(pdev, "alarm");
	if (unlikely(rtc->alarm_irq <= 0))
		dev_warn(&pdev->dev, "Alarm interrupt is not set\n");

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(res == NULL)) {
		dev_err(&pdev->dev, "No IO resource\n");
		return -ENOENT;
	}

	rtc->regsize = resource_size(res);

	rtc->res = devm_request_mem_region(&pdev->dev, res->start,
					rtc->regsize, pdev->name);
	if (unlikely(!rtc->res))
		return -EBUSY;

	rtc->regbase = devm_ioremap(&pdev->dev, rtc->res->start, rtc->regsize);

	if (unlikely(!rtc->regbase))
		return -EINVAL;

	if (!pdev->dev.of_node) {
		clk_id = pdev->id;
		/* With a single device, the clock id is still "rtc0" */
		if (clk_id < 0)
			clk_id = 0;

		snprintf(clk_name, sizeof(clk_name), "rtc%d", clk_id);
	} else
		snprintf(clk_name, sizeof(clk_name), "fck");

	rtc->clk = devm_clk_get(&pdev->dev, clk_name);
	if (IS_ERR(rtc->clk)) {
		/*
		 * No error handling for rtc->clk intentionally, not all
		 * platforms will have a unique clock for the RTC, and
		 * the clk API can handle the struct clk pointer being
		 * NULL.
		 */
		rtc->clk = NULL;
	}

	rtc->rtc_dev = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc->rtc_dev))
		return PTR_ERR(rtc->rtc_dev);

	clk_enable(rtc->clk);

	rtc->capabilities = RTC_DEF_CAPABILITIES;

	if (rtc->carry_irq <= 0) {
		/* register shared periodic/carry/alarm irq */
		ret = devm_request_irq(&pdev->dev, rtc->periodic_irq,
				rtca3_shared, 0, "rtca-3", rtc);
		if (unlikely(ret)) {
			dev_err(&pdev->dev,
				"request IRQ failed with %d, IRQ %d\n", ret,
				rtc->periodic_irq);
			goto err_unmap;
		}
	} else {
		/* register periodic/carry/alarm irqs */
		ret = devm_request_irq(&pdev->dev, rtc->periodic_irq,
				rtca3_periodic, 0, "rtca-3 period", rtc);
		if (unlikely(ret)) {
			dev_err(&pdev->dev,
				"request period IRQ failed with %d, IRQ %d\n",
				ret, rtc->periodic_irq);
			goto err_unmap;
		}

		ret = devm_request_irq(&pdev->dev, rtc->carry_irq,
				rtca3_interrupt, 0, "rtca-3 carry", rtc);
		if (unlikely(ret)) {
			dev_err(&pdev->dev,
				"request carry IRQ failed with %d, IRQ %d\n",
				ret, rtc->carry_irq);
			goto err_unmap;
		}

		ret = devm_request_irq(&pdev->dev, rtc->alarm_irq,
				rtca3_alarm, 0, "rtca-3 alarm", rtc);
		if (unlikely(ret)) {
			dev_err(&pdev->dev,
				"request alarm IRQ failed with %d, IRQ %d\n",
				ret, rtc->alarm_irq);
			goto err_unmap;
		}
	}

	platform_set_drvdata(pdev, rtc);
	/* Disable Alarm and Carry interrupts */
	rtca3_setaie(&pdev->dev, 0);
	rtca3_setcie(&pdev->dev, 0);

	/* Setting for RTC start */
	ret = rtca3_set_start(pdev);
	if (ret)
		goto err_unmap;

	rtc->rtc_dev->ops = &rtca3_ops;
	rtc->rtc_dev->max_user_freq = 256;

	if (rtc->capabilities & RTC_CAP_4_DIGIT_YEAR) {
		rtc->rtc_dev->range_min = RTC_TIMESTAMP_BEGIN_1900;
		rtc->rtc_dev->range_max = RTC_TIMESTAMP_END_9999;
	} else {
		rtc->rtc_dev->range_min = mktime64(1999, 1, 1, 0, 0, 0);
		rtc->rtc_dev->range_max = mktime64(2098, 12, 31, 23, 59, 59);
	}

	ret = rtc_register_device(rtc->rtc_dev);
	if (ret)
		goto err_unmap;

	device_init_wakeup(&pdev->dev, 1);
	return 0;
err_unmap:
	clk_disable(rtc->clk);

	return ret;
}

static int __exit rtca3_remove(struct platform_device *pdev)
{
	struct rtca3_priv *rtc = platform_get_drvdata(pdev);

	rtca3_setaie(&pdev->dev, 0);
	rtca3_setcie(&pdev->dev, 0);

	clk_disable(rtc->clk);

	return 0;
}

static void rtca3_set_irq_wake(struct device *dev, int enabled)
{
	struct rtca3_priv *rtc = dev_get_drvdata(dev);

	irq_set_irq_wake(rtc->periodic_irq, enabled);

	if (rtc->carry_irq > 0) {
		irq_set_irq_wake(rtc->carry_irq, enabled);
		irq_set_irq_wake(rtc->alarm_irq, enabled);
	}
}

static int __maybe_unused rtca3_suspend(struct device *dev)
{
	if (device_may_wakeup(dev))
		rtca3_set_irq_wake(dev, 1);

	return 0;
}

static int __maybe_unused rtca3_resume(struct device *dev)
{
	if (device_may_wakeup(dev))
		rtca3_set_irq_wake(dev, 0);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rtca3_pm_ops, rtca3_suspend, rtca3_resume);

static const struct of_device_id rtca3_of_match[] = {
	{ .compatible = "renesas,rtca-3", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtca3_of_match);

static struct platform_driver rtca3_platform_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.pm	= &rtca3_pm_ops,
		.of_match_table = rtca3_of_match,
	},
	.probe		= rtca3_probe,
	.remove		= rtca3_remove,
};

static __init int rtca3_initcall(void)
{
	platform_driver_register(&rtca3_platform_driver);
	return 0;
}

static __exit void rtca3_exitcall(void)
{
	platform_driver_unregister(&rtca3_platform_driver);
}

late_initcall_sync(rtca3_initcall);
module_exit(rtca3_exitcall);

MODULE_DESCRIPTION("RZ/G3S on-chip RTC driver");
MODULE_AUTHOR("Nghia Nguyen <nghia.nguyen.xm@renesas.com>");
MODULE_LICENSE("GPL v2");
