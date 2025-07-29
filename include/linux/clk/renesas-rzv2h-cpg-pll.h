/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Renesas RZ/V2H(P) CPG PLL helper
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 */
#ifndef __RENESAS_RZV2H_CPG_PLL_H__
#define __RENESAS_RZV2H_CPG_PLL_H__

#include <linux/clk-provider.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/units.h>

#define RZ_V2H_OSC_CLK_IN_MEGA		(24 * MEGA)
#define RZV2H_MAX_DIV_TABLES		(16)

/**
 * struct rzv2h_pll_limits - PLL parameter constraints
 *
 * This structure defines the minimum and maximum allowed values for
 * various parameters used to configure a PLL. These limits ensure
 * the PLL operates within valid and stable ranges.
 *
 * @fout: Output frequency range (in MHz)
 * @fout.min: Minimum allowed output frequency
 * @fout.max: Maximum allowed output frequency
 *
 * @fvco: PLL oscillation frequency range (in MHz)
 * @fvco.min: Minimum allowed VCO frequency
 * @fvco.max: Maximum allowed VCO frequency
 *
 * @m: Main-divider range
 * @m.min: Minimum main-divider value
 * @m.max: Maximum main-divider value
 *
 * @p: Pre-divider range
 * @p.min: Minimum pre-divider value
 * @p.max: Maximum pre-divider value
 *
 * @s: Divider range
 * @s.min: Minimum divider value
 * @s.max: Maximum divider value
 *
 * @k: Delta-sigma modulator range (signed)
 * @k.min: Minimum delta-sigma value
 * @k.max: Maximum delta-sigma value
 */
struct rzv2h_pll_limits {
	struct {
		u32 min;
		u32 max;
	} fout;

	struct {
		u32 min;
		u32 max;
	} fvco;

	struct {
		u16 min;
		u16 max;
	} m;

	struct {
		u8 min;
		u8 max;
	} p;

	struct {
		u8 min;
		u8 max;
	} s;

	struct {
		s16 min;
		s16 max;
	} k;
};

/**
 * struct rzv2h_pll_pars - PLL configuration parameters
 *
 * This structure contains the configuration parameters for the
 * Phase-Locked Loop (PLL), used to achieve a specific output frequency.
 *
 * @m: Main divider value
 * @p: Pre-divider value
 * @s: Output divider value
 * @k: Delta-sigma modulation value
 * @freq_millihz: Calculated PLL output frequency in millihertz
 * @error_millihz: Frequency error from target in millihertz (signed)
 */
struct rzv2h_pll_pars {
	u16 m;
	u8 p;
	u8 s;
	s16 k;
	u64 freq_millihz;
	s64 error_millihz;
};

/**
 * struct rzv2h_pll_div_pars - PLL parameters with post-divider
 *
 * This structure is used for PLLs that include an additional post-divider
 * stage after the main PLL block. It contains both the PLL configuration
 * parameters and the resulting frequency/error values after the divider.
 *
 * @pll: Main PLL configuration parameters (see struct rzv2h_pll_pars)
 *
 * @div: Post-divider configuration and result
 * @div.divider_value: Divider applied to the PLL output
 * @div.freq_millihz: Output frequency after divider in millihertz
 * @div.error_millihz: Frequency error from target in millihertz (signed)
 */
struct rzv2h_pll_div_pars {
	struct rzv2h_pll_pars pll;
	struct {
		u8 divider_value;
		u64 freq_millihz;
		s64 error_millihz;
	} div;
};

#define RZV2H_CPG_PLL_DSI_LIMITS(name)					\
	static const struct rzv2h_pll_limits (name) = {			\
		.fout = { .min = 25 * MEGA, .max = 375 * MEGA },	\
		.fvco = { .min = 1600 * MEGA, .max = 3200 * MEGA },	\
		.m = { .min = 64, .max = 533 },				\
		.p = { .min = 1, .max = 4 },				\
		.s = { .min = 0, .max = 6 },				\
		.k = { .min = -32768, .max = 32767 },			\
	}								\

/**
 * rzv2h_get_pll_pars - Finds the best combination of PLL parameters
 * for a given frequency.
 *
 * @limits: Pointer to the structure containing the limits for the PLL parameters
 * @pars: Pointer to the structure where the best calculated PLL parameters values
 * will be stored
 * @freq_millihz: Target output frequency in millihertz
 *
 * This function calculates the best set of PLL parameters (M, K, P, S) to achieve
 * the desired frequency.
 * There is no direct formula to calculate the PLL parameters, as it's an open
 * system of equations, therefore this function uses an iterative approach to
 * determine the best solution. The best solution is one that minimizes the error
 * (desired frequency - actual frequency).
 *
 * Return: true if a valid set of parameters values is found, false otherwise.
 */
static __maybe_unused bool
rzv2h_get_pll_pars(const struct rzv2h_pll_limits *limits,
		   struct rzv2h_pll_pars *pars, u64 freq_millihz)
{
	u64 fout_min_millihz = mul_u32_u32(limits->fout.min, MILLI);
	u64 fout_max_millihz = mul_u32_u32(limits->fout.max, MILLI);
	struct rzv2h_pll_pars p, best;

	if (freq_millihz > fout_max_millihz ||
	    freq_millihz < fout_min_millihz)
		return false;

	/* Initialize best error to maximum possible value */
	best.error_millihz = S64_MAX;

	for (p.p = limits->p.min; p.p <= limits->p.max; p.p++) {
		u32 fref = RZ_V2H_OSC_CLK_IN_MEGA / p.p;
		u16 divider;

		for (divider = 1 << limits->s.min, p.s = limits->s.min;
			p.s <= limits->s.max; p.s++, divider <<= 1) {
			for (p.m = limits->m.min; p.m <= limits->m.max; p.m++) {
				u64 output_m, output_k_range;
				s64 pll_k, output_k;
				u64 fvco, output;

				/*
				 * The frequency generated by the PLL + divider
				 * is calculated as follows:
				 *
				 * With:
				 * Freq = Ffout = Ffvco / 2^(pll_s)
				 * Ffvco = (pll_m + (pll_k / 65536)) * Ffref
				 * Ffref = 24MHz / pll_p
				 *
				 * Freq can also be rewritten as:
				 * Freq = Ffvco / 2^(pll_s)
				 *      = ((pll_m + (pll_k / 65536)) * Ffref) / 2^(pll_s)
				 *      = (pll_m * Ffref) / 2^(pll_s) + ((pll_k / 65536) * Ffref) / 2^(pll_s)
				 *      = output_m + output_k
				 *
				 * Every parameter has been determined at this
				 * point, but pll_k.
				 *
				 * Considering that:
				 * limits->k.min <= pll_k <= limits->k.max
				 * Then:
				 * -0.5 <= (pll_k / 65536) < 0.5
				 * Therefore:
				 * -Ffref / (2 * 2^(pll_s)) <= output_k < Ffref / (2 * 2^(pll_s))
				 */

				/* Compute output M component (in mHz) */
				output_m = DIV_ROUND_CLOSEST_ULL(mul_u32_u32(p.m, fref) * MILLI,
								 divider);
				/* Compute range for output K (in mHz) */
				output_k_range = DIV_ROUND_CLOSEST_ULL(mul_u32_u32(fref, MILLI),
								       2 * divider);
				/*
				 * No point in continuing if we can't achieve
				 * the desired frequency
				 */
				if (freq_millihz <  (output_m - output_k_range) ||
				    freq_millihz >= (output_m + output_k_range)) {
					continue;
				}

				/*
				 * Compute the K component
				 *
				 * Since:
				 * Freq = output_m + output_k
				 * Then:
				 * output_k = Freq - output_m
				 *          = ((pll_k / 65536) * Ffref) / 2^(pll_s)
				 * Therefore:
				 * pll_k = (output_k * 65536 * 2^(pll_s)) / Ffref
				 */
				output_k = freq_millihz - output_m;
				pll_k = div_s64(output_k * 65536ULL * divider,
						fref);
				pll_k = DIV_S64_ROUND_CLOSEST(pll_k, MILLI);

				/* Validate K value within allowed limits */
				if (pll_k < limits->k.min ||
				    pll_k > limits->k.max)
					continue;

				p.k = pll_k;

				/* Compute (Ffvco * 65536) */
				fvco = mul_u32_u32(p.m * 65536 + p.k, fref);
				if (fvco < mul_u32_u32(limits->fvco.min, 65536) ||
				    fvco > mul_u32_u32(limits->fvco.max, 65536))
					continue;

				/* PLL_M component of (output * 65536 * PLL_P) */
				output = mul_u32_u32(p.m * 65536, RZ_V2H_OSC_CLK_IN_MEGA);
				/* PLL_K component of (output * 65536 * PLL_P) */
				output += p.k * RZ_V2H_OSC_CLK_IN_MEGA;
				/* Make it in mHz */
				output *= MILLI;
				output = DIV_U64_ROUND_CLOSEST(output, 65536 * p.p * divider);

				/* Check output frequency against limits */
				if (output < fout_min_millihz ||
				    output > fout_max_millihz)
					continue;

				p.error_millihz = freq_millihz - output;
				p.freq_millihz = output;

				/* If an exact match is found, return immediately */
				if (p.error_millihz == 0) {
					*pars = p;
					return true;
				}

				/* Update best match if error is smaller */
				if (abs(best.error_millihz) > abs(p.error_millihz))
					best = p;
			}
		}
	}

	/* If no valid parameters were found, return false */
	if (best.error_millihz == S64_MAX)
		return false;

	*pars = best;
	return true;
}

/*
 * rzv2h_get_pll_div_pars - Finds the best combination of PLL parameters
 * and divider value for a given frequency.
 *
 * @limits: Pointer to the structure containing the limits for the PLL parameters
 * @pars: Pointer to the structure where the best calculated PLL parameters and
 * divider values will be stored
 * @divider: Divider value to be applied to the PLL output
 * @freq_millihz: Target output frequency in millihertz
 *
 * This function calculates the best set of PLL parameters (M, K, P, S) where
 * the divider value is already known. See rzv2h_get_pll_pars() for more details
 * on how the PLL parameters are calculated.
 */
static __maybe_unused bool
rzv2h_get_pll_div_pars(const struct rzv2h_pll_limits *limits,
		       struct rzv2h_pll_div_pars *pars, u8 divider,
		       u64 freq_millihz)
{
	if (!rzv2h_get_pll_pars(limits, &pars->pll, freq_millihz * divider))
		return false;

	pars->div.divider_value = divider;
	pars->div.freq_millihz = DIV_U64_ROUND_CLOSEST(pars->pll.freq_millihz, divider);
	pars->div.error_millihz = freq_millihz - pars->div.freq_millihz;

	return true;
}

/*
 * rzv2h_get_pll_divs_pars - Finds the best combination of PLL parameters
 * and divider value for a given frequency.
 *
 * @limits: Pointer to the structure containing the limits for the PLL parameters
 * @pars: Pointer to the structure where the best calculated PLL parameters and
 * divider values will be stored
 * @table: Pointer to the array of valid divider values
 * @table_size: Size of the divider values array
 * @freq_millihz: Target output frequency in millihertz
 *
 * This function calculates the best set of PLL parameters (M, K, P, S) and divider
 * value to achieve the desired frequency. See rzv2h_get_pll_pars() for more details
 * on how the PLL parameters are calculated.
 *
 * freq_millihz is the desired frequency generated by the PLL followed by a
 * a gear.
 */
static __maybe_unused bool
rzv2h_get_pll_divs_pars(const struct rzv2h_pll_limits *limits,
			struct rzv2h_pll_div_pars *pars,
			const u8 *table, u8 table_size, u64 freq_millihz)
{
	struct rzv2h_pll_div_pars p, best;

	best.div.error_millihz = S64_MAX;
	p.div.error_millihz = S64_MAX;
	for (unsigned int i = 0; i < table_size; i++) {
		if (!rzv2h_get_pll_div_pars(limits, &p, table[i], freq_millihz))
			continue;

		if (p.div.error_millihz == 0) {
			*pars = p;
			return true;
		}

		if (abs(best.div.error_millihz) > abs(p.div.error_millihz))
			best = p;
	}

	if (best.div.error_millihz == S64_MAX)
		return false;

	*pars = best;
	return true;
}

/*
 * rzv2h_get_pll_dtable_pars - Finds the best combination of PLL parameters
 * and divider value for a given frequency using a divider table.
 *
 * @limits: Pointer to the structure containing the limits for the PLL parameters
 * @pars: Pointer to the structure where the best calculated PLL parameters and
 * divider values will be stored
 * @dtable: Pointer to the array of valid divider values
 * @freq_millihz: Target output frequency in millihertz
 *
 * See rzv2h_get_pll_divs_pars() for more details on how the PLL
 * parameters and divider values are calculated.
 */
static __maybe_unused bool
rzv2h_get_pll_dtable_pars(const struct rzv2h_pll_limits *limits,
			  struct rzv2h_pll_div_pars *pars,
			  const struct clk_div_table *dtable, u64 freq_millihz)
{
	const struct clk_div_table *div = dtable;
	u8 table[RZV2H_MAX_DIV_TABLES] = { 0 };
	unsigned int i = 0;

	for (; div->div; div++) {
		if (i >= RZV2H_MAX_DIV_TABLES)
			return false;
		table[i++] = div->div;
	}

	return rzv2h_get_pll_divs_pars(limits, pars, table, i, freq_millihz);
}

#endif	/* __RENESAS_RZV2H_CPG_PLL_H__ */
