// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/V2M PWM Timer (PWM) driver
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 *
 * Hardware manual for this IP can be found here
 * https://www.renesas.com/in/en/document/mah/rzv2m-users-manual-hardware?language=en
 *
 * Limitations:
 * - Changes to the duty cycle configuration get effective only after the next
 *   period end.
 * - The duty cycle can be changed only by modifying the PWMLOW register
 *   value and changing the pulse width at low level. The duty cycle becomes
 *   0% for the low width when the value of the PWMLOW register is 0x0h
 *   and 100% for the low width when the value of the PWMLOW > PWMCYC.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/device/driver.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/reset.h>
#include <linux/time.h>
#include <linux/module.h>

#define RZV2M_PWMCTR    0x0
#define RZV2M_PWMCYC    0x4
#define RZV2M_PWMLOW    0x8
#define RZV2M_PWMCNT    0xc

#define RZV2M_PWMCTR_PWMPS      GENMASK(17, 16)
#define RZV2M_PWMCTR_PWMHL      BIT(3)
#define RZV2M_PWMCTR_PWMTM      BIT(2)
#define RZV2M_PWMCTR_PWME       BIT(1)

#define RZV2M_PWMCYC_PERIOD     GENMASK(23, 0)

struct rzv2m_pwm_chip {
        struct pwm_chip chip;
        void __iomem *mmio;
        struct reset_control *rstc;
        struct clk *apb_clk;
        struct clk *pwm_clk;
        unsigned long rate;
        unsigned long delay;
        unsigned long pwm_cyc;
        enum pwm_polarity polarity;
        bool is_ch_enabled;
};

static inline struct rzv2m_pwm_chip *to_rzv2m_pwm_chip(struct pwm_chip *chip)
{
        return container_of(chip, struct rzv2m_pwm_chip, chip);
}

static void rzv2m_pwm_wait_delay(struct rzv2m_pwm_chip *rzv2m_pwm)
{
        /* delay timer when change the setting register */
        ndelay(rzv2m_pwm->delay);
}

static void rzv2m_pwm_write(struct rzv2m_pwm_chip *rzv2m_pwm, u32 reg, u32 data)
{
        writel(data, rzv2m_pwm->mmio + reg);
}

static u32 rzv2m_pwm_read(struct rzv2m_pwm_chip *rzv2m_pwm, u32 reg)
{
        return readl(rzv2m_pwm->mmio + reg);
}

static void rzv2m_pwm_modify(struct rzv2m_pwm_chip *rzv2m_pwm, u32 reg, u32 clr,
                             u32 set)
{
        rzv2m_pwm_write(rzv2m_pwm, reg,
                        (rzv2m_pwm_read(rzv2m_pwm, reg) & ~clr) | set);
}

static u8 rzv2m_pwm_calculate_prescale(struct rzv2m_pwm_chip *rzv2m_pwm,
                                       u64 period_cycles)
{
        u32 prescaled_period_cycles;
        u8 prescale;

        prescaled_period_cycles = period_cycles >> 24;
        if (prescaled_period_cycles >= 256)
                prescale = 3;
        else
                prescale = (fls(prescaled_period_cycles) + 3) / 4;

        return prescale;
}

static int rzv2m_pwm_enable(struct rzv2m_pwm_chip *rzv2m_pwm)
{
        int rc;

        rc = pm_runtime_resume_and_get(rzv2m_pwm->chip.dev);
        if (rc)
                return rc;

        rzv2m_pwm_modify(rzv2m_pwm, RZV2M_PWMCTR, RZV2M_PWMCTR_PWME,
                         RZV2M_PWMCTR_PWME);
        rzv2m_pwm_wait_delay(rzv2m_pwm);
        rzv2m_pwm->is_ch_enabled = true;

        return 0;
}

static void rzv2m_pwm_disable(struct rzv2m_pwm_chip *rzv2m_pwm)
{
        rzv2m_pwm_modify(rzv2m_pwm, RZV2M_PWMCTR, RZV2M_PWMCTR_PWME, 0);
        rzv2m_pwm_wait_delay(rzv2m_pwm);
        pm_runtime_put_sync(rzv2m_pwm->chip.dev);
        rzv2m_pwm->is_ch_enabled = false;
}

static void rzv2m_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
                               struct pwm_state *state)
{
        struct rzv2m_pwm_chip *rzv2m_pwm = to_rzv2m_pwm_chip(chip);
        u32 period, low, val;
        u8 prescale;

        pm_runtime_get_sync(chip->dev);
        val = rzv2m_pwm_read(rzv2m_pwm, RZV2M_PWMCTR);
        state->enabled = FIELD_GET(RZV2M_PWMCTR_PWME, val);
        state->polarity = FIELD_GET(RZV2M_PWMCTR_PWMHL, val) ?
                PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;
        prescale = FIELD_GET(RZV2M_PWMCTR_PWMPS, val);
        period = rzv2m_pwm_read(rzv2m_pwm, RZV2M_PWMCYC);
        low = rzv2m_pwm_read(rzv2m_pwm, RZV2M_PWMLOW);
        if (low > period) /* 0% duty cycle */
                val = 0;
        else if (!low) /* 100% duty cycle */
                val = period;
        else
                val = period - low;

        if (period)
                period += 1;

        state->period = DIV_ROUND_UP_ULL(NSEC_PER_SEC * (u64)period << (4 * prescale),
                                         rzv2m_pwm->rate);
        state->duty_cycle = DIV_ROUND_UP_ULL(NSEC_PER_SEC * (u64)val << (4 * prescale),
                                             rzv2m_pwm->rate);
        pm_runtime_put(chip->dev);
}

static int rzv2m_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                            const struct pwm_state *state)
{
        struct rzv2m_pwm_chip *rzv2m_pwm = to_rzv2m_pwm_chip(chip);
        unsigned long pwm_cyc, pwm_low;
        u8 prescale;
        u32 pwm_ctr;
        u64 pc, dc;

        /*
         * Refuse clk rates > 1 GHz to prevent overflowing the following
         * calculation.
         */
        if (rzv2m_pwm->rate > NSEC_PER_SEC)
                return -EINVAL;

        /*
         * Formula for calculating PWM Cycle Setting Register
         * PWM cycle = (PWM period(ns) / (PWM_CLK period(ns) × Div ratio)) - 1
         */
        pc = mul_u64_u32_div(state->period, rzv2m_pwm->rate, NSEC_PER_SEC);
        dc = mul_u64_u32_div(state->duty_cycle, rzv2m_pwm->rate, NSEC_PER_SEC);
        prescale = rzv2m_pwm_calculate_prescale(rzv2m_pwm, pc);

        pwm_cyc = pc >> (4 * prescale);
        if (pwm_cyc)
                pwm_cyc -= 1;

        if (!FIELD_FIT(RZV2M_PWMCYC_PERIOD, pwm_cyc))
                pwm_cyc = FIELD_MAX(RZV2M_PWMCYC_PERIOD);

        /*
         * Formula for calculating PWMLOW register
         * PWMLOW register = PWM cycle * Low pulse width ratio (%)
         */
        pwm_low = dc >> (4 * prescale);
        if (!dc) /* 0% duty cycle */
                pwm_low = pwm_cyc + 1;
        else if (pc == dc) /* 100% duty cycle */
                pwm_low = 0;
        else
                pwm_low = pwm_cyc - pwm_low;

        /*
         * If the PWM channel is disabled, make sure to turn on the clock
         * before writing the register.
         */
        if (!pwm->state.enabled)
                pm_runtime_get_sync(rzv2m_pwm->chip.dev);

        /*
         * To change the setting value of the PWM cycle setting register
         * (PWMm_PWMCYC) or polarity, set the PWME bit of the PWM control
         * register (PWMm_PWMCTR) to 0b and stop the counter operation.
         */
        if (rzv2m_pwm->polarity != state->polarity || rzv2m_pwm->pwm_cyc != pwm_cyc) {
                rzv2m_pwm_modify(rzv2m_pwm, RZV2M_PWMCTR, RZV2M_PWMCTR_PWME, 0);
                rzv2m_pwm_wait_delay(rzv2m_pwm);
        }

        rzv2m_pwm_write(rzv2m_pwm, RZV2M_PWMCYC, pwm_cyc);
        rzv2m_pwm_write(rzv2m_pwm, RZV2M_PWMLOW, pwm_low);

        pwm_ctr = FIELD_PREP(RZV2M_PWMCTR_PWMPS, prescale);
        if (state->polarity == PWM_POLARITY_INVERSED)
                pwm_ctr |= RZV2M_PWMCTR_PWMHL;

        rzv2m_pwm_modify(rzv2m_pwm, RZV2M_PWMCTR, RZV2M_PWMCTR_PWMTM |
                         RZV2M_PWMCTR_PWMPS | RZV2M_PWMCTR_PWMHL, pwm_ctr);

        if (rzv2m_pwm->polarity != state->polarity || rzv2m_pwm->pwm_cyc != pwm_cyc) {
                rzv2m_pwm->polarity = state->polarity;
                rzv2m_pwm->pwm_cyc = pwm_cyc;
                rzv2m_pwm_modify(rzv2m_pwm, RZV2M_PWMCTR, RZV2M_PWMCTR_PWME,
                                 RZV2M_PWMCTR_PWME);
        }

        rzv2m_pwm_wait_delay(rzv2m_pwm);

        /* If the PWM is not enabled, turn the clock off again to save power. */
        if (!pwm->state.enabled)
                pm_runtime_put(rzv2m_pwm->chip.dev);

        return 0;
}

static int rzv2m_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
                           const struct pwm_state *state)
{
        struct rzv2m_pwm_chip *rzv2m_pwm = to_rzv2m_pwm_chip(chip);
        bool enabled = pwm->state.enabled;
        int ret;

        if (!state->enabled) {
                if (enabled)
                        rzv2m_pwm_disable(rzv2m_pwm);

                return 0;
        }

        ret = rzv2m_pwm_config(chip, pwm, state);
        if (ret)
                return ret;

        if (!enabled)
                ret = rzv2m_pwm_enable(rzv2m_pwm);

        return ret;
}

static const struct pwm_ops rzv2m_pwm_ops = {
        .get_state = rzv2m_pwm_get_state,
        .apply = rzv2m_pwm_apply,
        .owner = THIS_MODULE,
};

static int rzv2m_pwm_pm_runtime_suspend(struct device *dev)
{
        struct rzv2m_pwm_chip *rzv2m_pwm = dev_get_drvdata(dev);

        clk_disable_unprepare(rzv2m_pwm->pwm_clk);
        clk_disable_unprepare(rzv2m_pwm->apb_clk);

        return 0;
}

static int rzv2m_pwm_pm_runtime_resume(struct device *dev)
{
        struct rzv2m_pwm_chip *rzv2m_pwm = dev_get_drvdata(dev);
        int err;

        err = clk_prepare_enable(rzv2m_pwm->apb_clk);
        if (err)
                return err;

        err = clk_prepare_enable(rzv2m_pwm->pwm_clk);
        if (err)
                return err;

        return 0;
}

static SIMPLE_DEV_PM_OPS(rzv2m_pwm_pm_ops, rzv2m_pwm_pm_runtime_suspend, rzv2m_pwm_pm_runtime_resume);

static void rzv2m_pwm_reset_assert_pm_disable(void *data)
{
        struct rzv2m_pwm_chip *rzv2m_pwm = data;

        /*
         * The below check is for making balanced PM usage count in probe/remove
         * eg: boot loader is turning on PWM and probe increments the PM usage
         * count. Before apply, if there is unbind/remove callback we need to
         * decrement the PM usage count.
         */
        if (rzv2m_pwm->is_ch_enabled)
                pm_runtime_put(rzv2m_pwm->chip.dev);

        clk_rate_exclusive_put(rzv2m_pwm->pwm_clk);
        clk_rate_exclusive_put(rzv2m_pwm->apb_clk);
        pm_runtime_disable(rzv2m_pwm->chip.dev);
        pm_runtime_set_suspended(rzv2m_pwm->chip.dev);
        reset_control_assert(rzv2m_pwm->rstc);
}

static int rzv2m_pwm_probe(struct platform_device *pdev)
{
        struct rzv2m_pwm_chip *rzv2m_pwm;
        unsigned long apb_clk_rate;
        int ret;

        rzv2m_pwm = devm_kzalloc(&pdev->dev, sizeof(*rzv2m_pwm), GFP_KERNEL);
        if (!rzv2m_pwm)
                return -ENOMEM;

        rzv2m_pwm->mmio = devm_platform_ioremap_resource(pdev, 0);
        if (IS_ERR(rzv2m_pwm->mmio))
                return PTR_ERR(rzv2m_pwm->mmio);

        rzv2m_pwm->apb_clk = devm_clk_get(&pdev->dev, "apb");
        if (IS_ERR(rzv2m_pwm->apb_clk))
                return dev_err_probe(&pdev->dev, PTR_ERR(rzv2m_pwm->apb_clk),
                                     "cannot get apb clock\n");

        rzv2m_pwm->pwm_clk = devm_clk_get(&pdev->dev, "pwm");
        if (IS_ERR(rzv2m_pwm->pwm_clk))
                return dev_err_probe(&pdev->dev, PTR_ERR(rzv2m_pwm->pwm_clk),
                                     "cannot get pwm clock\n");

        rzv2m_pwm->rstc = devm_reset_control_get_shared(&pdev->dev, NULL);
        if (IS_ERR(rzv2m_pwm->rstc))
                return dev_err_probe(&pdev->dev, PTR_ERR(rzv2m_pwm->rstc),
                                     "get reset failed\n");

        platform_set_drvdata(pdev, rzv2m_pwm);
        ret = reset_control_deassert(rzv2m_pwm->rstc);
        if (ret) {
                return dev_err_probe(&pdev->dev, ret,
                              "cannot deassert reset control\n");
        }

        ret = clk_prepare_enable(rzv2m_pwm->apb_clk);
        if (ret < 0)
                goto err_reset;

        ret = clk_prepare_enable(rzv2m_pwm->pwm_clk);
        if (ret < 0)
                goto disable_apb_clk;

        clk_rate_exclusive_get(rzv2m_pwm->apb_clk);
        clk_rate_exclusive_get(rzv2m_pwm->pwm_clk);
        apb_clk_rate = clk_get_rate(rzv2m_pwm->apb_clk);
        if (!apb_clk_rate)
                goto err_rate_put;

        rzv2m_pwm->rate = clk_get_rate(rzv2m_pwm->pwm_clk);
        if (!rzv2m_pwm->rate)
                goto err_rate_put;

        /*
         * The registers other than the PWM interrupt register (PWMINT) are
         * always synchronized with PWM_CLK at regular intervals. It takes some
         * time (Min: 2 × PCLK + 4 × PWM_CLK to Max: 6 × PCLK + 9 × PWM_CLK) for
         * the value set in the register to be reflected in the PWM circuit
         * because there is a synchronizer between the register and the PWM
         * circuit.
         */
        rzv2m_pwm->delay = 6 * DIV_ROUND_UP(NSEC_PER_SEC, apb_clk_rate) +
                9 * DIV_ROUND_UP(NSEC_PER_SEC, rzv2m_pwm->rate);

        pm_runtime_set_active(&pdev->dev);
        pm_runtime_enable(&pdev->dev);

        /*
         *  We need to keep the clock on, in case the bootloader has enabled the
         *  PWM and is running during probe().
         */
        if (!!(rzv2m_pwm_read(rzv2m_pwm, RZV2M_PWMCTR) & RZV2M_PWMCTR_PWME)) {
                u32 val;

                pm_runtime_get_sync(&pdev->dev);
                rzv2m_pwm->is_ch_enabled = true;
                rzv2m_pwm->pwm_cyc = rzv2m_pwm_read(rzv2m_pwm, RZV2M_PWMCYC);
                val = rzv2m_pwm_read(rzv2m_pwm, RZV2M_PWMCTR);
                rzv2m_pwm->polarity = FIELD_GET(RZV2M_PWMCTR_PWMHL, val) ?
                                PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;
        }

        rzv2m_pwm->chip.dev = &pdev->dev;
        ret = devm_add_action_or_reset(&pdev->dev,
                                       rzv2m_pwm_reset_assert_pm_disable,
                                       rzv2m_pwm);
        if (ret)
                return ret;

        rzv2m_pwm->chip.ops = &rzv2m_pwm_ops;
        rzv2m_pwm->chip.npwm = 1;
        rzv2m_pwm->chip.base = -1;

        ret = pwmchip_add(&rzv2m_pwm->chip);
        if (ret)
                return dev_err_probe(&pdev->dev, ret, "failed to add PWM chip\n");

        pm_runtime_idle(&pdev->dev);

	dev_info(&pdev->dev, "probed\n");
        return 0;

err_rate_put:
        clk_rate_exclusive_put(rzv2m_pwm->pwm_clk);
        clk_rate_exclusive_put(rzv2m_pwm->apb_clk);
        clk_disable_unprepare(rzv2m_pwm->pwm_clk);
disable_apb_clk:
        clk_disable_unprepare(rzv2m_pwm->apb_clk);
err_reset:
        reset_control_assert(rzv2m_pwm->rstc);
        return ret;
}

static const struct of_device_id rzv2m_pwm_of_table[] = {
        { .compatible = "renesas,rzv2m-pwm", },
        { .compatible = "renesas,rzv2ma-pwm", },
        { /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzv2m_pwm_of_table);

static struct platform_driver rzv2m_pwm_driver = {
        .driver = {
                .name = "pwm-rzv2m",
                .pm = pm_ptr(&rzv2m_pwm_pm_ops),
                .of_match_table = of_match_ptr(rzv2m_pwm_of_table),
        },
        .probe = rzv2m_pwm_probe,
};
module_platform_driver(rzv2m_pwm_driver);

MODULE_AUTHOR("Biju Das <biju.das.jz@bp.renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/V2M PWM Timer Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-rzv2m");

