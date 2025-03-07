/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Renesas RZ/V2H(P) Clock Pulse Generator
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 */

#ifndef __RENESAS_RZV2H_CPG_H__
#define __RENESAS_RZV2H_CPG_H__

/**
 * struct pll - Structure for PLL
 *
 * @conf: pll conf
 * @min: pll minimum frequency in MHz
 * @max: pll maximum frequency in MHz
 */
struct pll {
	unsigned int conf;
	unsigned long min;
	unsigned long max;
};

/**
 * struct mux - Structure for static mux switching
 *
 * @offset: register offset
 * @shift: position of the mux switching bit
 * @width: width of the mux switching
 * @mux_flags: flags for mux switching clock.
 * @parent_names: name of parent clocks
 * @num_parents: number of parent clocks.
 */
struct mux {
	unsigned int offset:11;
	unsigned int shift:4;
	unsigned int width:4;
	int mux_flags;
	const char * const *parent_names;
	int num_parents;
};

/**
 * struct ddiv - Structure for static/dynamic switching divider
 *
 * @offset: register offset
 * @shift: position of the divider bit
 * @width: width of the divider
 * @monbit: monitor bit in CPG_CLKSTATUS0 register
 */
struct sddiv {
	unsigned int offset:11;
	unsigned int shift:4;
	unsigned int width:4;
	s8 monbit;
};

#define PLL_PACK(_conf, _min, _max)	\
	((struct pll){ \
		.conf = _conf, \
		.min = _min, \
		.max = _max, \
	})

#define SSEL_PACK(_offset, _shift, _width) \
	((struct mux){ \
		.offset = _offset, \
		.shift = _shift, \
		.width = _width, \
	})

#define DDIV_PACK(_offset, _shift, _width, _monbit) \
	((struct sddiv){ \
		.offset = _offset, \
		.shift = _shift, \
		.width = _width, \
		.monbit = _monbit \
	})

#define SDIV_PACK(_offset, _shift, _width) \
	((struct sddiv){ \
		.offset = _offset, \
		.shift = _shift, \
		.width = _width, \
		.monbit = -1 \
	})

#define CPG_SSEL(x)		(0x300 + 4 * (x))
#define CPG_CDDIV(x)		(0x400 + 4 * (x))
#define CPG_CSDIV(x)		(0x500 + 4 * (x))

#define SSELx_SELCTLy(x, y)		SSEL_PACK(CPG_SSEL(x), (y) * 4, 1)
#define CDDIVx_DIVCTLy(x, y, w)		DDIV_PACK(CPG_CDDIV(x), (y) * 4, w, (x) * 4 + (y))
#define CSDIVx_DIVCTLy(x, y, w)		SDIV_PACK(CPG_CSDIV(x), (y) * 4, w)

#define EXTAL_FREQ_IN_MEGA_HZ   (24)

/**
 * Definitions of CPG Core Clocks
 *
 * These include:
 *   - Clock outputs exported to DT
 *   - External input clocks
 *   - Internal CPG clocks
 */
struct cpg_core_clk {
	const char *name;
	unsigned int id;
	unsigned int parent;
	unsigned int div;
	unsigned int mult;
	unsigned int type;
	union {
		unsigned int conf;
		struct sddiv sddiv;
		struct mux mux;
		struct pll pll;
	} cfg;
	const struct clk_div_table *dtable;
	unsigned long min;
	unsigned long max;
	u32 flag;
};

enum clk_types {
	/* Generic */
	CLK_TYPE_IN,		/* External Clock Input */
	CLK_TYPE_FF,		/* Fixed Factor Clock */
	CLK_TYPE_PLL,
	CLK_TYPE_PLLDSI,
	CLK_TYPE_DDIV,		/* Dynamic Switching Divider */
	CLK_TYPE_SDIV,		/* Static Switching Divider */
	CLK_TYPE_MUX,		/* Static Mux Switching */
	CLK_TYPE_PLLDSI_SDIV,
	CLK_TYPE_PLL_DIV,	/* Clock for divider after PLL clock */
};

/* BIT(31) indicates if CLK1/2 are accessible or not */
#define PLL_CONF(n)		(BIT(31) | ((n) & ~GENMASK(31, 16)))
#define PLL_CLK_ACCESS(n)	((n) & BIT(31) ? 1 : 0)
#define PLL_STBY_OFFSET(n)	(((n) & ~GENMASK(31, 16)) - (0x4))
#define PLL_CLK1_OFFSET(n)	((n) & ~GENMASK(31, 16))
#define PLL_CLK2_OFFSET(n)	(((n) & ~GENMASK(31, 16)) + (0x4))
#define PLL_MON_OFFSET(n)	(((n) & ~GENMASK(31, 16)) + (0xC))

#define PLL_STBY_RESETB		BIT(0)
#define PLL_STBY_RESETB_WEN	BIT(16)
#define PLL_MON_RESETB		BIT(0)
#define PLL_MON_LOCK		BIT(4)

#define DEF_TYPE(_name, _id, _type...) \
	{ .name = _name, .id = _id, .type = _type }
#define DEF_BASE(_name, _id, _type, _parent...) \
	DEF_TYPE(_name, _id, _type, .parent = _parent)
#define DEF_PLL(_name, _id, _parent, _pll_packed) \
	DEF_TYPE(_name, _id, CLK_TYPE_PLL, .parent = _parent, \
		.cfg.pll = _pll_packed)
#define DEF_PLLDSI(_name, _id, _parent, _pll_packed) \
	DEF_TYPE(_name, _id, CLK_TYPE_PLLDSI, .parent = _parent, \
		.cfg.pll = _pll_packed)
#define DEF_INPUT(_name, _id) \
	DEF_TYPE(_name, _id, CLK_TYPE_IN)
#define DEF_FIXED(_name, _id, _parent, _mult, _div) \
	DEF_BASE(_name, _id, CLK_TYPE_FF, _parent, .div = _div, .mult = _mult)
#define DEF_DDIV(_name, _id, _parent, _ddiv_packed, _dtable) \
	DEF_TYPE(_name, _id, CLK_TYPE_DDIV, \
		.cfg.sddiv = _ddiv_packed, \
		.parent = _parent, \
		.dtable = _dtable, \
		.flag = CLK_DIVIDER_HIWORD_MASK)
#define DEF_SDIV(_name, _id, _parent, _sdiv_packed, _dtable) \
	DEF_TYPE(_name, _id, CLK_TYPE_SDIV, \
		.cfg.sddiv = _sdiv_packed, \
		.parent = _parent, \
		.dtable = _dtable, \
		.flag = CLK_DIVIDER_HIWORD_MASK)
#define DEF_MUX(_name, _id, _mux_packed, _parent_names) \
	DEF_TYPE(_name, _id, CLK_TYPE_MUX, \
		.cfg.mux = _mux_packed, \
		.cfg.mux.parent_names = _parent_names, \
		.cfg.mux.num_parents = ARRAY_SIZE(_parent_names), \
		.cfg.mux.mux_flags = CLK_MUX_HIWORD_MASK)
#define DEF_PLLDSI_SDIV(_name, _id, _parent, _sdiv_packed, _dtable) \
	DEF_TYPE(_name, _id, CLK_TYPE_PLLDSI_SDIV, \
		 .cfg.sddiv = _sdiv_packed, \
		 .dtable = _dtable, \
		 .parent = _parent, \
		 .flag = CLK_DIVIDER_HIWORD_MASK)
#define DEF_MUX_FLAGS(_name, _id, _mux_packed, _parent_names, _flags) \
	DEF_TYPE(_name, _id, CLK_TYPE_MUX, \
		 .cfg.mux = _mux_packed, \
		 .cfg.mux.parent_names = _parent_names, \
		 .cfg.mux.num_parents = ARRAY_SIZE(_parent_names), \
		 .cfg.mux.mux_flags = CLK_MUX_HIWORD_MASK, \
		 .flag = _flags)
#define DEF_PLL_DIV(_name, _id, _parent, _mult, _div) \
	DEF_TYPE(_name, _id, CLK_TYPE_PLL_DIV, .parent = _parent, \
		.div = _div, .mult = _mult)

/**
 * struct rzv2h_mod_clk - Module Clocks definitions
 *
 * @name: handle between common and hardware-specific interfaces
 * @parent: id of parent clock
 * @critical: flag to indicate the clock is critical
 * @on_index: control register index
 * @on_bit: ON bit
 * @mon_index: monitor register index
 * @mon_bit: monitor bit
 */
struct rzv2h_mod_clk {
	const char *name;
	u16 parent;
	bool critical;
	u8 on_index;
	u8 on_bit;
	s8 mon_index;
	u8 mon_bit;
};

#define DEF_MOD_BASE(_name, _parent, _critical, _onindex, _onbit, _monindex, _monbit) \
	{ \
		.name = (_name), \
		.parent = (_parent), \
		.critical = (_critical), \
		.on_index = (_onindex), \
		.on_bit = (_onbit), \
		.mon_index = (_monindex), \
		.mon_bit = (_monbit), \
	}

#define DEF_MOD(_name, _parent, _onindex, _onbit, _monindex, _monbit)		\
	DEF_MOD_BASE(_name, _parent, false, _onindex, _onbit, _monindex, _monbit)

#define DEF_MOD_CRITICAL(_name, _parent, _onindex, _onbit, _monindex, _monbit)	\
	DEF_MOD_BASE(_name, _parent, true, _onindex, _onbit, _monindex, _monbit)

/**
 * struct rzv2h_reset - Reset definitions
 *
 * @reset_index: reset register index
 * @reset_bit: reset bit
 * @mon_index: monitor register index
 * @mon_bit: monitor bit
 */
struct rzv2h_reset {
	u8 reset_index;
	u8 reset_bit;
	u8 mon_index;
	u8 mon_bit;
};

#define DEF_RST_BASE(_resindex, _resbit, _monindex, _monbit)	\
	{ \
		.reset_index = (_resindex), \
		.reset_bit = (_resbit), \
		.mon_index = (_monindex), \
		.mon_bit = (_monbit), \
	}

#define DEF_RST(_resindex, _resbit, _monindex, _monbit)	\
	DEF_RST_BASE(_resindex, _resbit, _monindex, _monbit)

/**
 * struct rzv2h_cpg_info - SoC-specific CPG Description
 *
 * @core_clks: Array of Core Clock definitions
 * @num_core_clks: Number of entries in core_clks[]
 * @last_dt_core_clk: ID of the last Core Clock exported to DT
 * @num_total_core_clks: Total number of Core Clocks (exported + internal)
 *
 * @mod_clks: Array of Module Clock definitions
 * @num_mod_clks: Number of entries in mod_clks[]
 * @num_hw_mod_clks: Number of Module Clocks supported by the hardware
 *
 * @resets: Array of Module Reset definitions
 * @num_resets: Number of entries in resets[]
 */
struct rzv2h_cpg_info {
	/* Core Clocks */
	const struct cpg_core_clk *core_clks;
	unsigned int num_core_clks;
	unsigned int last_dt_core_clk;
	unsigned int num_total_core_clks;

	/* Module Clocks */
	const struct rzv2h_mod_clk *mod_clks;
	unsigned int num_mod_clks;
	unsigned int num_hw_mod_clks;

	/* Resets */
	const struct rzv2h_reset *resets;
	unsigned int num_resets;
};

extern const struct rzv2h_cpg_info r9a09g047_cpg_info;
extern const struct rzv2h_cpg_info r9a09g057_cpg_info;

#endif	/* __RENESAS_RZV2H_CPG_H__ */
