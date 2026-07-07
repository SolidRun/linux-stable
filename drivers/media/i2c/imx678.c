// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony imx678 sensor driver
 *
 * Copyright (C) 2021 Intel Corporation
 */
#include "imx678.h"
#include <asm/unaligned.h>
#include <linux/videodev2.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/kernel.h>
#include "sensor_id.h"

#define DEFAULT_MODE_IDX 0

#define CLOCK_FREQ_HZ (74250000)

/* Streaming Mode */
#define IMX678_REG_MODE_SELECT 0x3000
#define IMX678_MODE_STANDBY 0x01
#define IMX678_MODE_STREAMING 0x00

/* Lines per frame */
#define IMX678_REG_LPFR 0x3028 // VMAX
#define IMX678_MAX_LPFR_4K (1 << 20) - 2 // max even value of unsigned 20bit
#define IMX678_MAX_VBLANK_4K (IMX678_MAX_LPFR_4K - 2160) // vmax - height

#define IMX678_REG_HMAX 0x302C

/* defaults */
#define IMX678_DEFAULT_RHS1 0x91
#define IMX678_DEFAULT_RHS2 0xaa
#define IMX678_EXPOSURE_DEFAULT 0x0648

/* gaps */
#define IMX678_SHR0_RHS2_GAP 7
#define IMX678_SHR0_FSC_GAP 3
#define IMX678_SHR1_MIN_GAP 7
#define IMX678_SHR1_RHS1_GAP 3
#define IMX678_SHR2_RHS1_GAP 7
#define IMX678_SHR2_RHS2_GAP 3

#define IMX678_INTEGER_STEP 1

/* Exposure control LEF */
#define IMX678_REG_SHUTTER 0x3050
#define IMX678_EXPOSURE_STEP 1

/* Exposure control SEF1 */
#define IMX678_REG_SHUTTER_SHORT 0x3054
#define IMX678_EXPOSURE_SHORT_STEP 1

/* Exposure control SEF2 */
#define IMX678_REG_SHUTTER_VERY_SHORT 0x3058
#define IMX678_EXPOSURE_VERY_SHORT_STEP 1

/* Analog gain control */
#define IMX678_REG_AGAIN 0x3070
#define IMX678_REG_AGAIN_SHORT 0x3072
#define IMX678_REG_AGAIN_VERY_SHORT 0x3074
#define IMX678_AGAIN_MIN 0
#define IMX678_AGAIN_MAX 240
#define IMX678_AGAIN_STEP 1
#define IMX678_AGAIN_DEFAULT 0

/* Wide Dynamic Range control */
#define IMX678_WDR_MIN 0
#define IMX678_WDR_MAX 1
#define IMX678_WDR_STEP 1
#define IMX678_WDR_DEFAULT 0

/* Hcg control */
#define IMX678_REG_HCG 0x3030
#define IMX678_HCG_MIN 0
#define IMX678_HCG_MAX 1
#define IMX678_HCG_STEP 1
#define IMX678_HCG_DEFAULT 0

/* HDR custom RHS1 control */
#define IMX678_REG_RHS1 0x3060
#define IMX678_CUSTOM_RHS1_MIN 0
#define IMX678_CUSTOM_RHS1_MAX 65535
/* Default 0 value means no custom value (value will be taken from the preset) */
#define IMX678_CUSTOM_RHS1_DEFAULT 0

/* Group hold register */
#define IMX678_REG_HOLD 0x3001

/* Input clock rate */
enum imx678_input_clk_rate_code {
	INPUT_CLK_74_25_MHZ = 0,
	INPUT_CLK_37_125_MHZ,
	INPUT_CLK_72_MHZ,
	INPUT_CLK_27_MHZ,
	INPUT_CLK_24_MHZ,
	INPUT_CLK_36_MHZ,
	INPUT_CLK_18_MHZ,
	INPUT_CLK_13_5_MHZ,
};

#define IMX678_INCLK_CODE INPUT_CLK_24_MHZ
#define IMX678_INCLK_RATE 24000000

/* CSI2 HW configuration */
#define IMX678_NUM_DATA_LANES 4

#define IMX678_REG_MIN 0x00
#define IMX678_REG_MAX 0xfffff

#define IMX678_TPG_EN_DUOUT 0x30e0 /* TEST PATTERN ENABLE */
#define IMX678_TPG_PATSEL_DUOUT 0x30e2 /*Patsel mode */
#define IMX678_TPG_COLOR_WIDTH 0x30e4 /*color width */

#define NON_NEGATIVE(val) ((val) < 0 ? 0 : (val))
#define MAX(val1, val2) ((val1) < (val2) ? (val2) : (val1))
#define MIN(val1, val2) ((val1) < (val2) ? (val1) : (val2))

static u32 imx678_reg_shutter[3] = {IMX678_REG_SHUTTER, IMX678_REG_SHUTTER_SHORT, IMX678_REG_SHUTTER_VERY_SHORT};
static u32 imx678_reg_again[3] = {IMX678_REG_AGAIN, IMX678_REG_AGAIN_SHORT, IMX678_REG_AGAIN_VERY_SHORT};

enum imx678_exposure_type {
	LEF,
	SEF1,
	SEF2,
};

static int imx678_set_ctrl(struct v4l2_ctrl *ctrl);
static int imx678_get_ctrl(struct v4l2_ctrl *ctrl);

/*
 * imx678 test pattern related structure
 */
enum { TEST_PATTERN_DISABLED = 0,
       TEST_PATTERN_ALL_000H,
       TEST_PATTERN_ALL_FFFH,
       TEST_PATTERN_ALL_555H,
       TEST_PATTERN_ALL_AAAH,
       TEST_PATTERN_VSP_5AH, /* VERTICAL STRIPE PATTERN 555H/AAAH */
       TEST_PATTERN_VSP_A5H, /* VERTICAL STRIPE PATTERN AAAH/555H */
       TEST_PATTERN_VSP_05H, /* VERTICAL STRIPE PATTERN 000H/555H */
       TEST_PATTERN_VSP_50H, /* VERTICAL STRIPE PATTERN 555H/000H */
       TEST_PATTERN_VSP_0FH, /* VERTICAL STRIPE PATTERN 000H/FFFH */
       TEST_PATTERN_VSP_F0H, /* VERTICAL STRIPE PATTERN FFFH/000H */
       TEST_PATTERN_H_COLOR_BARS,
       TEST_PATTERN_V_COLOR_BARS,
};

/**
 * enum imx678_test_pattern_menu - imx678 test pattern options
 */
static const char *const imx678_test_pattern_menu[] = {
	"Disabled",
	"All 000h Pattern",
	"All FFFh Pattern",
	"All 555h Pattern",
	"All AAAh Pattern",
	"Vertical Stripe (555h / AAAh)",
	"Vertical Stripe (AAAh / 555h)",
	"Vertical Stripe (000h / 555h)",
	"Vertical Stripe (555h / 000h)",
	"Vertical Stripe (000h / FFFh)",
	"Vertical Stripe (FFFh / 000h)",
	"Horizontal Color Bars",
	"Vertical Color Bars",
};

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops imx678_ctrl_ops = {
	.s_ctrl = imx678_set_ctrl,
};

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops imx678_get_ctrl_ops = {
	.g_volatile_ctrl = imx678_get_ctrl,
};

/**
 * struct imx678_reg - imx678 sensor register
 * @address: Register address
 * @val: Register value
 */
struct imx678_reg {
	u16 address;
	u8 val;
};

/**
 * struct imx678_reg_list - imx678 sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct imx678_reg_list {
	u32 num_of_regs;
	const struct imx678_reg *regs;
};

u32 _get_mode_reg_val_by_address(const struct imx678_reg_list *reg_list, u16 reg_address, int num_bytes) {
	u32 left = 0;
	u32 right = reg_list->num_of_regs - 1;
	u32 val = 0;
	int i = 0;
	while (left <= right) {
		u32 mid = left + (right - left) / 2;
		if (reg_list->regs[mid].address == reg_address) {
		val = 0;
		for (i = 0; i < num_bytes; i++) {
			val |= (reg_list->regs[mid + i].val >> 8*i) & 0xff;
		}
		return val;
		}
		if (reg_list->regs[mid].address < reg_address) {
			left = mid + 1;
		} else {
			right = mid - 1;
		}
	}
	return 0;
}

/**
 * struct imx678_mode - imx678 sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @hblank: Horizontal blanking in lines
 * @vblank: Vertical blanking in lines
 * @vblank_min: Minimal vertical blanking in lines
 * @vblank_max: Maximum vertical blanking in lines
 * @pclk: Sensor pixel clock
 * @link_freq_idx: Link frequency index
 * @reg_list: Register list for sensor mode
 */
struct imx678_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 hblank;
	u32 vblank;
	u32 vblank_min;
	u32 vblank_max;
	u64 pclk;
	u32 link_freq_idx;
	u32 rhs1;
	u32 rhs2;
	u8 dol;
	struct imx678_reg_list reg_list;
	struct v4l2_fract frame_interval;
};

int compare_imx678_mode(const struct imx678_mode *mode1, const struct imx678_mode *mode2) {
	bool not_equal = (mode1->width != mode2->width) ||
           (mode1->height != mode2->height) ||
           (mode1->code != mode2->code) ||
           (mode1->hblank != mode2->hblank) ||
           (mode1->vblank != mode2->vblank) ||
           (mode1->vblank_min != mode2->vblank_min) ||
           (mode1->vblank_max != mode2->vblank_max) ||
           (mode1->pclk != mode2->pclk);
	return not_equal;
}

struct exp_gain_ctrl_cluster {
	struct v4l2_ctrl *exp_ctrl;
	struct v4l2_ctrl *again_ctrl;
};

/**
 * struct imx678 - imx678 sensor device structure
 * @dev: Pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @ctrl_handler: V4L2 control handler
 * @link_freq_ctrl: Pointer to link frequency control
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @test_pattern_ctrl: pointer to test pattern control
 * @mode_sel_ctrl: pointer to mode select control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @vblank: Vertical blanking in lines
 * @cur_mode: Pointer to current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 * @streaming: Flag indicating streaming state
 */
struct imx678 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *xmaster_gpio;
	struct clk *inclk;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *rhs1_ctrl;
	struct v4l2_ctrl *rhs2_ctrl;
	struct v4l2_ctrl *shr0_ctrl;
	struct v4l2_ctrl *shr1_ctrl;
	struct v4l2_ctrl *shr2_ctrl;
	struct v4l2_ctrl *vmax_ctrl;
	struct v4l2_ctrl *hmax_ctrl;
	struct v4l2_ctrl *test_pattern_ctrl;
	struct v4l2_ctrl *mode_sel_ctrl;
	struct v4l2_ctrl *hcg_ctrl;
	struct v4l2_ctrl *custom_rhs1_ctrl;
	struct exp_gain_ctrl_cluster lef;
	struct exp_gain_ctrl_cluster sef1;
	struct exp_gain_ctrl_cluster sef2;
	u32 vblank;
	const struct imx678_mode *cur_mode;
	struct imx678_mode custom_mode;
	u32 custom_rhs1_value;
	u32 mode_idx;
	char mode_string[SENSOR_MODE_STRING_LENGTH];
	struct mutex mutex;
	bool streaming;
	bool hdr_enabled;
	struct v4l2_subdev_format curr_fmt;
};

static const s64 link_freq[] = {
	891000000, 1440000000, 1782000000,
};

/* Sensor mode registers */
static const struct imx678_reg mode_3840x2160_regs[] = {
	{0x3000, 0x01}, // STANDBY					*imx678
	{0x3002, 0x01}, // XMSTA					*imx678
	{0x3018, 0x04}, // WINMODE					*imx678
	//{0x37b0, 0x36}, // ?
	//{0x304c, 0x00}, // OPB_SIZE_V				*no in imx678
	//{0x300c, 0x3b}, // BC_WAIT_TIME			*no in imx678
	//{0x300d, 0x2a}, // CPWAIT_TIME			*no in imx678
	{0x302c, 0x26}, // HMAX						*imx678
	{0x302d, 0x02}, // HMAX						*imx678
	{0x3014, IMX678_INCLK_CODE}, // INCK_SEL					*imx678
	{0x3040, 0x03}, // LANEMODE - 4 lanes		*imx678
	//{0x301a, 0x00}, // WDMODE=Normal			*imx678
	//{0x3022, 0x01}, // ADBIT=12Bit			*imx678
	{0x303c, 0x00}, // PIX_HST=00				*imx678
	{0x303e, 0x00}, // PIX_HWIDTH_LSB=3840		*imx678
	{0x303f, 0x0f}, // PIX_HWIDTH_MSB			*imx678
	{0x3044, 0x00}, // PIX_VST=00				*imx678
	{0x3046, 0x70}, // PIX_VWIDTH_LSB=2160		*imx678
	{0x3047, 0x08}, // PIX_VWIDTH_MSB			*imx678
	{0x3050, 0x03},
	{0x3051, 0x00},
	{0x30a6, 0x00}, // XVS_DRV, XHS_DRV			*imx678
	{0x3460, 0x22},
	{0x355A, 0x64},
	{0x3A02, 0x7A},
	{0x3A10, 0xEC},
	{0x3A12, 0x71},
	{0x3A14, 0xDE},
	{0x3A20, 0x2B},
	{0x3A24, 0x22},
	{0x3A25, 0x25},
	{0x3A26, 0x2A},
	{0x3A27, 0x2C},
	{0x3A28, 0x39},
	{0x3A29, 0x38},
	{0x3A30, 0x04},
	{0x3A31, 0x04},
	{0x3A32, 0x03},
	{0x3A33, 0x03},
	{0x3A34, 0x09},
	{0x3A35, 0x06},
	{0x3A38, 0xCD},
	{0x3A3A, 0x4C},
	{0x3A3C, 0xB9},
	{0x3A3E, 0x30},
	{0x3A40, 0x2C},
	{0x3A42, 0x39},
	{0x3A4E, 0x00},
	{0x3A52, 0x00},
	{0x3A56, 0x00},
	{0x3A5A, 0x00},
	{0x3A5E, 0x00},
	{0x3A62, 0x00},
	{0x3A6E, 0xA0},
	{0x3A70, 0x50},
	{0x3A8C, 0x04},
	{0x3A8D, 0x03},
	{0x3A8E, 0x09},
	{0x3A90, 0x38},
	{0x3A91, 0x42},
	{0x3A92, 0x3C},
	{0x3B0E, 0xF3},
	{0x3B12, 0xE5},
	{0x3B27, 0xC0},
	{0x3B2E, 0xEF},
	{0x3B30, 0x6A},
	{0x3B32, 0xF6},
	{0x3B36, 0xE1},
	{0x3B3A, 0xE8},
	{0x3B5A, 0x17},
	{0x3B5E, 0xEF},
	{0x3B60, 0x6A},
	{0x3B62, 0xF6},
	{0x3B66, 0xE1},
	{0x3B6A, 0xE8},
	{0x3B88, 0xEC},
	{0x3B8A, 0xED},
	{0x3B94, 0x71},
	{0x3B96, 0x72},
	{0x3B98, 0xDE},
	{0x3B9A, 0xDF},
	{0x3C0F, 0x06},
	{0x3C10, 0x06},
	{0x3C11, 0x06},
	{0x3C12, 0x06},
	{0x3C13, 0x06},
	{0x3C18, 0x20},
	{0x3C3A, 0x7A},
	{0x3C40, 0xF4},
	{0x3C48, 0xE6},
	{0x3C54, 0xCE},
	{0x3C56, 0xD0},
	{0x3C6C, 0x53},
	{0x3C6E, 0x55},
	{0x3C70, 0xC0},
	{0x3C72, 0xC2},
	{0x3C7E, 0xCE},
	{0x3C8C, 0xCF},
	{0x3C8E, 0xEB},
	{0x3C98, 0x54},
	{0x3C9A, 0x70},
	{0x3C9C, 0xC1},
	{0x3C9E, 0xDD},
	{0x3CB0, 0x7A},
	{0x3CB2, 0xBA},
	{0x3CC8, 0xBC},
	{0x3CCA, 0x7C},
	{0x3CD4, 0xEA},
	{0x3CD5, 0x01},
	{0x3CD6, 0x4A},
	{0x3CD8, 0x00},
	{0x3CD9, 0x00},
	{0x3CDA, 0xFF},
	{0x3CDB, 0x03},
	{0x3CDC, 0x00},
	{0x3CDD, 0x00},
	{0x3CDE, 0xFF},
	{0x3CDF, 0x03},
	{0x3CE4, 0x4C},
	{0x3CE6, 0xEC},
	{0x3CE7, 0x01},
	{0x3CE8, 0xFF},
	{0x3CE9, 0x03},
	{0x3CEA, 0x00},
	{0x3CEB, 0x00},
	{0x3CEC, 0xFF},
	{0x3CED, 0x03},
	{0x3CEE, 0x00},
	{0x3CEF, 0x00},
	{0x3E28, 0x82},
	{0x3E2A, 0x80},
	{0x3E30, 0x85},
	{0x3E32, 0x7D},
	{0x3E5C, 0xCE},
	{0x3E5E, 0xD3},
	{0x3E70, 0x53},
	{0x3E72, 0x58},
	{0x3E74, 0xC0},
	{0x3E76, 0xC5},
	{0x3E78, 0xC0},
	{0x3E79, 0x01},
	{0x3E7A, 0xD4},
	{0x3E7B, 0x01},
	{0x3EB4, 0x0B},
	{0x3EB5, 0x02},
	{0x3EB6, 0x4D},
	{0x3EEC, 0xF3},
	{0x3EEE, 0xE7},
	{0x3F01, 0x01},
	{0x3F24, 0x10},
	{0x3F28, 0x2D},
	{0x3F2A, 0x2D},
	{0x3F2C, 0x2D},
	{0x3F2E, 0x2D},
	{0x3F30, 0x23},
	{0x3F38, 0x2D},
	{0x3F3A, 0x2D},
	{0x3F3C, 0x2D},
	{0x3F3E, 0x28},
	{0x3F40, 0x1E},
	{0x3F48, 0x2D},
	{0x3F4A, 0x2D},
	{0x4004, 0xE4},
	{0x4006, 0xFF},
	{0x4018, 0x69},
	{0x401A, 0x84},
	{0x401C, 0xD6},
	{0x401E, 0xF1},
	{0x4038, 0xDE},
	{0x403A, 0x00},
	{0x403B, 0x01},
	{0x404C, 0x63},
	{0x404E, 0x85},
	{0x4050, 0xD0},
	{0x4052, 0xF2},
	{0x4108, 0xDD},
	{0x410A, 0xF7},
	{0x411C, 0x62},
	{0x411E, 0x7C},
	{0x4120, 0xCF},
	{0x4122, 0xE9},
	{0x4138, 0xE6},
	{0x413A, 0xF1},
	{0x414C, 0x6B},
	{0x414E, 0x76},
	{0x4150, 0xD8},
	{0x4152, 0xE3},
	{0x417E, 0x03},
	{0x417F, 0x01},
	{0x4186, 0xE0},
	{0x4190, 0xF3},
	{0x4192, 0xF7},
	{0x419C, 0x78},
	{0x419E, 0x7C},
	{0x41A0, 0xE5},
	{0x41A2, 0xE9},
	{0x41C8, 0xE2},
	{0x41CA, 0xFD},
	{0x41DC, 0x67},
	{0x41DE, 0x82},
	{0x41E0, 0xD4},
	{0x41E2, 0xEF},
	{0x4200, 0xDE},
	{0x4202, 0xDA},
	{0x4218, 0x63},
	{0x421A, 0x5F},
	{0x421C, 0xD0},
	{0x421E, 0xCC},
	{0x425A, 0x82},
	{0x425C, 0xEF},
	{0x4348, 0xFE},
	{0x4349, 0x06},
	{0x4352, 0xCE},
	{0x4420, 0x0B},
	{0x4421, 0x02},
	{0x4422, 0x4D},
	{0x4426, 0xF5},
	{0x442A, 0xE7},
	{0x4432, 0xF5},
	{0x4436, 0xE7},
	{0x4466, 0xB4},
	{0x446E, 0x32},
	{0x449F, 0x1C},
	{0x44A4, 0x2C},
	{0x44A6, 0x2C},
	{0x44A8, 0x2C},
	{0x44AA, 0x2C},
	{0x44B4, 0x2C},
	{0x44B6, 0x2C},
	{0x44B8, 0x2C},
	{0x44BA, 0x2C},
	{0x44C4, 0x2C},
	{0x44C6, 0x2C},
	{0x44C8, 0x2C},
	{0x4506, 0xF3},
	{0x450E, 0xE5},
	{0x4516, 0xF3},
	{0x4522, 0xE5},
	{0x4524, 0xF3},
	{0x452C, 0xE5},
	{0x453C, 0x22},
	{0x453D, 0x1B},
	{0x453E, 0x1B},
	{0x453F, 0x15},
	{0x4540, 0x15},
	{0x4541, 0x15},
	{0x4542, 0x15},
	{0x4543, 0x15},
	{0x4544, 0x15},
	{0x4548, 0x00},
	{0x4549, 0x01},
	{0x454A, 0x01},
	{0x454B, 0x06},
	{0x454C, 0x06},
	{0x454D, 0x06},
	{0x454E, 0x06},
	{0x454F, 0x06},
	{0x4550, 0x06},
	{0x4554, 0x55},
	{0x4555, 0x02},
	{0x4556, 0x42},
	{0x4557, 0x05},
	{0x4558, 0xFD},
	{0x4559, 0x05},
	{0x455A, 0x94},
	{0x455B, 0x06},
	{0x455D, 0x06},
	{0x455E, 0x49},
	{0x455F, 0x07},
	{0x4560, 0x7F},
	{0x4561, 0x07},
	{0x4562, 0xA5},
	{0x4564, 0x55},
	{0x4565, 0x02},
	{0x4566, 0x42},
	{0x4567, 0x05},
	{0x4568, 0xFD},
	{0x4569, 0x05},
	{0x456A, 0x94},
	{0x456B, 0x06},
	{0x456D, 0x06},
	{0x456E, 0x49},
	{0x456F, 0x07},
	{0x4572, 0xA5},
	{0x460C, 0x7D},
	{0x460E, 0xB1},
	{0x4614, 0xA8},
	{0x4616, 0xB2},
	{0x461C, 0x7E},
	{0x461E, 0xA7},
	{0x4624, 0xA8},
	{0x4626, 0xB2},
	{0x462C, 0x7E},
	{0x462E, 0x8A},
	{0x4630, 0x94},
	{0x4632, 0xA7},
	{0x4634, 0xFB},
	{0x4636, 0x2F},
	{0x4638, 0x81},
	{0x4639, 0x01},
	{0x463A, 0xB5},
	{0x463B, 0x01},
	{0x463C, 0x26},
	{0x463E, 0x30},
	{0x4640, 0xAC},
	{0x4641, 0x01},
	{0x4642, 0xB6},
	{0x4643, 0x01},
	{0x4644, 0xFC},
	{0x4646, 0x25},
	{0x4648, 0x82},
	{0x4649, 0x01},
	{0x464A, 0xAB},
	{0x464B, 0x01},
	{0x464C, 0x26},
	{0x464E, 0x30},
	{0x4654, 0xFC},
	{0x4656, 0x08},
	{0x4658, 0x12},
	{0x465A, 0x25},
	{0x4662, 0xFC},
	{0x46A2, 0xFB},
	{0x46D6, 0xF3},
	{0x46E6, 0x00},
	{0x46E8, 0xFF},
	{0x46E9, 0x03},
	{0x46EC, 0x7A},
	{0x46EE, 0xE5},
	{0x46F4, 0xEE},
	{0x46F6, 0xF2},
	{0x470C, 0xFF},
	{0x470D, 0x03},
	{0x470E, 0x00},
	{0x4714, 0xE0},
	{0x4716, 0xE4},
	{0x471E, 0xED},
	{0x472E, 0x00},
	{0x4730, 0xFF},
	{0x4731, 0x03},
	{0x4734, 0x7B},
	{0x4736, 0xDF},
	{0x4754, 0x7D},
	{0x4756, 0x8B},
	{0x4758, 0x93},
	{0x475A, 0xB1},
	{0x475C, 0xFB},
	{0x475E, 0x09},
	{0x4760, 0x11},
	{0x4762, 0x2F},
	{0x4766, 0xCC},
	{0x4776, 0xCB},
	{0x477E, 0x4A},
	{0x478E, 0x49},
	{0x4794, 0x7C},
	{0x4796, 0x8F},
	{0x4798, 0xB3},
	{0x4799, 0x00},
	{0x479A, 0xCC},
	{0x479C, 0xC1},
	{0x479E, 0xCB},
	{0x47A4, 0x7D},
	{0x47A6, 0x8E},
	{0x47A8, 0xB4},
	{0x47A9, 0x00},
	{0x47AA, 0xC0},
	{0x47AC, 0xFA},
	{0x47AE, 0x0D},
	{0x47B0, 0x31},
	{0x47B1, 0x01},
	{0x47B2, 0x4A},
	{0x47B3, 0x01},
	{0x47B4, 0x3F},
	{0x47B6, 0x49},
	{0x47BC, 0xFB},
	{0x47BE, 0x0C},
	{0x47C0, 0x32},
	{0x47C1, 0x01},
	{0x47C2, 0x3E},
	{0x47C3, 0x01},
};

static const struct imx678_reg mode_1920x1080_sdr_binning_regs[] = {
    { 0x3000, 0x01 }, // STANDBY                    *imx678
    { 0x3002, 0x00 }, // XMSTA                  *imx678
    { 0x3014, IMX678_INCLK_CODE }, // INCK_SEL          *imx678
    { 0x3018, 0x04 }, // WINMODE - crop                 *imx678
    { 0x301A, 0x00 }, //WDMODE[7:0] - sdr
    { 0x301B, 0x01 }, //ADDMODE[1:0] - binning
    // { 0x301C, 0x01 }, //THIN_V_EN[7:0] - subsampling
    { 0x3022, 0x00 }, //ADBIT[1:0] // 10 bit
    { 0x3023, 0x01 }, //MDBIT[1:0] // raw12 IMPORTANT
    { 0x3028, 0x5e }, //VMAX[15:0]
    { 0x3029, 0x1a }, //VMAX[15:0] // 6750
    { 0x302C, 0x26 }, //HMAX[15:0] // 550
    { 0x302D, 0x02 },
    { 0x303c, 0x00 }, // PIX_HST=00             *imx678
    { 0x303e, 0x00 }, // PIX_HWIDTH_LSB=3840        *imx678
    { 0x303f, 0x0f }, // PIX_HWIDTH_MSB         *imx678
    { 0x3044, 0x00 }, // PIX_VST=00             *imx678
    { 0x3046, 0x70 }, // PIX_VWIDTH_LSB=2160        *imx678
    { 0x3047, 0x08 }, // PIX_VWIDTH_MSB         *imx678
    { 0x3050, 0x18 }, //SHR0[19:0]
    { 0x3051, 0x15 }, 
    // { 0x3054, 0x07 }, //SHR1[19:0]
    // { 0x3058, 0x4A }, //SHR2[19:0]
    // { 0x3059, 0x00 }, 
    // { 0x3060, 0x40 }, //RHS1[19:0]
    // { 0x3061, 0x00 }, 
    // { 0x3064, 0x53 }, //RHS2[19:0]
    { 0x3065, 0x00 }, { 0x30A6, 0x00 }, //XVS_DRV[1:0]
    { 0x3460, 0x22 }, { 0x355A, 0x64 }, { 0x3A02, 0x7A }, { 0x3A10, 0xEC },
    { 0x3A12, 0x71 }, { 0x3A14, 0xDE }, { 0x3A20, 0x2B }, { 0x3A24, 0x22 },
    { 0x3A25, 0x25 }, { 0x3A26, 0x2A }, { 0x3A27, 0x2C }, { 0x3A28, 0x39 },
    { 0x3A29, 0x38 }, { 0x3A30, 0x04 }, { 0x3A31, 0x04 }, { 0x3A32, 0x03 },
    { 0x3A33, 0x03 }, { 0x3A34, 0x09 }, { 0x3A35, 0x06 }, { 0x3A38, 0xCD },
    { 0x3A3A, 0x4C }, { 0x3A3C, 0xB9 }, { 0x3A3E, 0x30 }, { 0x3A40, 0x2C },
    { 0x3A42, 0x39 }, { 0x3A4E, 0x00 }, { 0x3A52, 0x00 }, { 0x3A56, 0x00 },
    { 0x3A5A, 0x00 }, { 0x3A5E, 0x00 }, { 0x3A62, 0x00 }, { 0x3A6E, 0xA0 },
    { 0x3A70, 0x50 }, { 0x3A8C, 0x04 }, { 0x3A8D, 0x03 }, { 0x3A8E, 0x09 },
    { 0x3A90, 0x38 }, { 0x3A91, 0x42 }, { 0x3A92, 0x3C }, { 0x3B0E, 0xF3 },
    { 0x3B12, 0xE5 }, { 0x3B27, 0xC0 }, { 0x3B2E, 0xEF }, { 0x3B30, 0x6A },
    { 0x3B32, 0xF6 }, { 0x3B36, 0xE1 }, { 0x3B3A, 0xE8 }, { 0x3B5A, 0x17 },
    { 0x3B5E, 0xEF }, { 0x3B60, 0x6A }, { 0x3B62, 0xF6 }, { 0x3B66, 0xE1 },
    { 0x3B6A, 0xE8 }, { 0x3B88, 0xEC }, { 0x3B8A, 0xED }, { 0x3B94, 0x71 },
    { 0x3B96, 0x72 }, { 0x3B98, 0xDE }, { 0x3B9A, 0xDF }, { 0x3C0F, 0x06 },
    { 0x3C10, 0x06 }, { 0x3C11, 0x06 }, { 0x3C12, 0x06 }, { 0x3C13, 0x06 },
    { 0x3C18, 0x20 }, { 0x3C3A, 0x7A }, { 0x3C40, 0xF4 }, { 0x3C48, 0xE6 },
    { 0x3C54, 0xCE }, { 0x3C56, 0xD0 }, { 0x3C6C, 0x53 }, { 0x3C6E, 0x55 },
    { 0x3C70, 0xC0 }, { 0x3C72, 0xC2 }, { 0x3C7E, 0xCE }, { 0x3C8C, 0xCF },
    { 0x3C8E, 0xEB }, { 0x3C98, 0x54 }, { 0x3C9A, 0x70 }, { 0x3C9C, 0xC1 },
    { 0x3C9E, 0xDD }, { 0x3CB0, 0x7A }, { 0x3CB2, 0xBA }, { 0x3CC8, 0xBC },
    { 0x3CCA, 0x7C }, { 0x3CD4, 0xEA }, { 0x3CD5, 0x01 }, { 0x3CD6, 0x4A },
    { 0x3CD8, 0x00 }, { 0x3CD9, 0x00 }, { 0x3CDA, 0xFF }, { 0x3CDB, 0x03 },
    { 0x3CDC, 0x00 }, { 0x3CDD, 0x00 }, { 0x3CDE, 0xFF }, { 0x3CDF, 0x03 },
    { 0x3CE4, 0x4C }, { 0x3CE6, 0xEC }, { 0x3CE7, 0x01 }, { 0x3CE8, 0xFF },
    { 0x3CE9, 0x03 }, { 0x3CEA, 0x00 }, { 0x3CEB, 0x00 }, { 0x3CEC, 0xFF },
    { 0x3CED, 0x03 }, { 0x3CEE, 0x00 }, { 0x3CEF, 0x00 }, { 0x3E28, 0x82 },
    { 0x3E2A, 0x80 }, { 0x3E30, 0x85 }, { 0x3E32, 0x7D }, { 0x3E5C, 0xCE },
    { 0x3E5E, 0xD3 }, { 0x3E70, 0x53 }, { 0x3E72, 0x58 }, { 0x3E74, 0xC0 },
    { 0x3E76, 0xC5 }, { 0x3E78, 0xC0 }, { 0x3E79, 0x01 }, { 0x3E7A, 0xD4 },
    { 0x3E7B, 0x01 }, { 0x3EB4, 0x0B }, { 0x3EB5, 0x02 }, { 0x3EB6, 0x4D },
    { 0x3EEC, 0xF3 }, { 0x3EEE, 0xE7 }, { 0x3F01, 0x01 }, { 0x3F24, 0x10 },
    { 0x3F28, 0x2D }, { 0x3F2A, 0x2D }, { 0x3F2C, 0x2D }, { 0x3F2E, 0x2D },
    { 0x3F30, 0x23 }, { 0x3F38, 0x2D }, { 0x3F3A, 0x2D }, { 0x3F3C, 0x2D },
    { 0x3F3E, 0x28 }, { 0x3F40, 0x1E }, { 0x3F48, 0x2D }, { 0x3F4A, 0x2D },
    { 0x4004, 0xE4 }, { 0x4006, 0xFF }, { 0x4018, 0x69 }, { 0x401A, 0x84 },
    { 0x401C, 0xD6 }, { 0x401E, 0xF1 }, { 0x4038, 0xDE }, { 0x403A, 0x00 },
    { 0x403B, 0x01 }, { 0x404C, 0x63 }, { 0x404E, 0x85 }, { 0x4050, 0xD0 },
    { 0x4052, 0xF2 }, { 0x4108, 0xDD }, { 0x410A, 0xF7 }, { 0x411C, 0x62 },
    { 0x411E, 0x7C }, { 0x4120, 0xCF }, { 0x4122, 0xE9 }, { 0x4138, 0xE6 },
    { 0x413A, 0xF1 }, { 0x414C, 0x6B }, { 0x414E, 0x76 }, { 0x4150, 0xD8 },
    { 0x4152, 0xE3 }, { 0x417E, 0x03 }, { 0x417F, 0x01 }, { 0x4186, 0xE0 },
    { 0x4190, 0xF3 }, { 0x4192, 0xF7 }, { 0x419C, 0x78 }, { 0x419E, 0x7C },
    { 0x41A0, 0xE5 }, { 0x41A2, 0xE9 }, { 0x41C8, 0xE2 }, { 0x41CA, 0xFD },
    { 0x41DC, 0x67 }, { 0x41DE, 0x82 }, { 0x41E0, 0xD4 }, { 0x41E2, 0xEF },
    { 0x4200, 0xDE }, { 0x4202, 0xDA }, { 0x4218, 0x63 }, { 0x421A, 0x5F },
    { 0x421C, 0xD0 }, { 0x421E, 0xCC }, { 0x425A, 0x82 }, { 0x425C, 0xEF },
    { 0x4348, 0xFE }, { 0x4349, 0x06 }, { 0x4352, 0xCE }, { 0x4420, 0x0B },
    { 0x4421, 0x02 }, { 0x4422, 0x4D }, { 0x4426, 0xF5 }, { 0x442A, 0xE7 },
    { 0x4432, 0xF5 }, { 0x4436, 0xE7 }, { 0x4466, 0xB4 }, { 0x446E, 0x32 },
    { 0x449F, 0x1C }, { 0x44A4, 0x2C }, { 0x44A6, 0x2C }, { 0x44A8, 0x2C },
    { 0x44AA, 0x2C }, { 0x44B4, 0x2C }, { 0x44B6, 0x2C }, { 0x44B8, 0x2C },
    { 0x44BA, 0x2C }, { 0x44C4, 0x2C }, { 0x44C6, 0x2C }, { 0x44C8, 0x2C },
    { 0x4506, 0xF3 }, { 0x450E, 0xE5 }, { 0x4516, 0xF3 }, { 0x4522, 0xE5 },
    { 0x4524, 0xF3 }, { 0x452C, 0xE5 }, { 0x453C, 0x22 }, { 0x453D, 0x1B },
    { 0x453E, 0x1B }, { 0x453F, 0x15 }, { 0x4540, 0x15 }, { 0x4541, 0x15 },
    { 0x4542, 0x15 }, { 0x4543, 0x15 }, { 0x4544, 0x15 }, { 0x4548, 0x00 },
    { 0x4549, 0x01 }, { 0x454A, 0x01 }, { 0x454B, 0x06 }, { 0x454C, 0x06 },
    { 0x454D, 0x06 }, { 0x454E, 0x06 }, { 0x454F, 0x06 }, { 0x4550, 0x06 },
    { 0x4554, 0x55 }, { 0x4555, 0x02 }, { 0x4556, 0x42 }, { 0x4557, 0x05 },
    { 0x4558, 0xFD }, { 0x4559, 0x05 }, { 0x455A, 0x94 }, { 0x455B, 0x06 },
    { 0x455D, 0x06 }, { 0x455E, 0x49 }, { 0x455F, 0x07 }, { 0x4560, 0x7F },
    { 0x4561, 0x07 }, { 0x4562, 0xA5 }, { 0x4564, 0x55 }, { 0x4565, 0x02 },
    { 0x4566, 0x42 }, { 0x4567, 0x05 }, { 0x4568, 0xFD }, { 0x4569, 0x05 },
    { 0x456A, 0x94 }, { 0x456B, 0x06 }, { 0x456D, 0x06 }, { 0x456E, 0x49 },
    { 0x456F, 0x07 }, { 0x4572, 0xA5 }, { 0x460C, 0x7D }, { 0x460E, 0xB1 },
    { 0x4614, 0xA8 }, { 0x4616, 0xB2 }, { 0x461C, 0x7E }, { 0x461E, 0xA7 },
    { 0x4624, 0xA8 }, { 0x4626, 0xB2 }, { 0x462C, 0x7E }, { 0x462E, 0x8A },
    { 0x4630, 0x94 }, { 0x4632, 0xA7 }, { 0x4634, 0xFB }, { 0x4636, 0x2F },
    { 0x4638, 0x81 }, { 0x4639, 0x01 }, { 0x463A, 0xB5 }, { 0x463B, 0x01 },
    { 0x463C, 0x26 }, { 0x463E, 0x30 }, { 0x4640, 0xAC }, { 0x4641, 0x01 },
    { 0x4642, 0xB6 }, { 0x4643, 0x01 }, { 0x4644, 0xFC }, { 0x4646, 0x25 },
    { 0x4648, 0x82 }, { 0x4649, 0x01 }, { 0x464A, 0xAB }, { 0x464B, 0x01 },
    { 0x464C, 0x26 }, { 0x464E, 0x30 }, { 0x4654, 0xFC }, { 0x4656, 0x08 },
    { 0x4658, 0x12 }, { 0x465A, 0x25 }, { 0x4662, 0xFC }, { 0x46A2, 0xFB },
    { 0x46D6, 0xF3 }, { 0x46E6, 0x00 }, { 0x46E8, 0xFF }, { 0x46E9, 0x03 },
    { 0x46EC, 0x7A }, { 0x46EE, 0xE5 }, { 0x46F4, 0xEE }, { 0x46F6, 0xF2 },
    { 0x470C, 0xFF }, { 0x470D, 0x03 }, { 0x470E, 0x00 }, { 0x4714, 0xE0 },
    { 0x4716, 0xE4 }, { 0x471E, 0xED }, { 0x472E, 0x00 }, { 0x4730, 0xFF },
    { 0x4731, 0x03 }, { 0x4734, 0x7B }, { 0x4736, 0xDF }, { 0x4754, 0x7D },
    { 0x4756, 0x8B }, { 0x4758, 0x93 }, { 0x475A, 0xB1 }, { 0x475C, 0xFB },
    { 0x475E, 0x09 }, { 0x4760, 0x11 }, { 0x4762, 0x2F }, { 0x4766, 0xCC },
    { 0x4776, 0xCB }, { 0x477E, 0x4A }, { 0x478E, 0x49 }, { 0x4794, 0x7C },
    { 0x4796, 0x8F }, { 0x4798, 0xB3 }, { 0x4799, 0x00 }, { 0x479A, 0xCC },
    { 0x479C, 0xC1 }, { 0x479E, 0xCB }, { 0x47A4, 0x7D }, { 0x47A6, 0x8E },
    { 0x47A8, 0xB4 }, { 0x47A9, 0x00 }, { 0x47AA, 0xC0 }, { 0x47AC, 0xFA },
    { 0x47AE, 0x0D }, { 0x47B0, 0x31 }, { 0x47B1, 0x01 }, { 0x47B2, 0x4A },
    { 0x47B3, 0x01 }, { 0x47B4, 0x3F }, { 0x47B6, 0x49 }, { 0x47BC, 0xFB },
    { 0x47BE, 0x0C }, { 0x47C0, 0x32 }, { 0x47C1, 0x01 }, { 0x47C2, 0x3E },
    { 0x47C3, 0x01 }, { 0x4E3C, 0x07 }
};

static const struct imx678_reg mode_4k_3dol_20fps_all_pixel[] = {
	/* 0x3000: Using default value (STANDBY) */
	/* 0x3001: Using default value (REGHOLD) */
	/* 0x3002: Using default value (XMSTA) */
	{ 0x3014, IMX678_INCLK_CODE }, // MANUAL (default(0x00) -> our clock's value)
	{ 0x3015, 0x02 }, /* DATARATE_SEL[3:0] */
	{ 0x3018, 0x04 }, // MANUAL (default(0x00) -> recommended resolution)
	/* 0x3019: Using default value (CFMODE[1:0]) */
	{ 0x301A, 0x02 }, /* WDMODE[7:0] */
	/* 0x301B: Using default value (ADDMODE[1:0]) */
	{ 0x301C, 0x01 }, /* THIN_V_EN[7:0] */
	/* 0x301E: Using default value (VCMODE[7:0]) */
	/* 0x3020: Using default value (HREVERSE) */
	/* 0x3021: Using default value (VREVERSE) */
	/* 0x3022: Using default value (ADBIT[1:0]) */
    { 0x3023, 0x01 }, // MANUAL (default(0x01) -> same value, write explicitly)
	/* 0x3028: Using default value (VMAX[19:0]) */
	/* 0x3029: Using default value */
	/* 0x302A: Using default value */
	{ 0x302C, 0x26 }, /* HMAX[15:0] */
	{ 0x302D, 0x02 },
	/* 0x3030: Using default value (FDG_SEL0[1:0]) */
	/* 0x3031: Using default value (FDG_SEL1[1:0]) */
	/* 0x3032: Using default value (FDG_SEL2[1:0]) */
	/* 0x303C: Using default value (PIX_HST[12:0]) */
	/* 0x303D: Using default value */
	{ 0x303E, 0x00 }, // MANUAL (default(0x10) -> recommended resolution)
	{ 0x303F, 0x0F }, // MANUAL (default(0x0F) -> recommended resolution)
	/* 0x3040: Using default value (LANEMODE[2:0]) */
	/* 0x3042: Using default value (XSIZE_OVERLAP[10:0]) */
	/* 0x3043: Using default value */
	/* 0x3044: Using default value (PIX_VST[11:0]) */
	/* 0x3045: Using default value */
	{ 0x3046, 0x70 }, // MANUAL (default(0x84) -> recommended resolution)
	{ 0x3047, 0x08 }, // MANUAL (default(0x08) -> recommended resolution)
	{ 0x3050, 0x18 }, /* SHR0[19:0] */
	{ 0x3051, 0x15 },
	/* 0x3052: Using default value */
	{ 0x3054, 0x07 }, /* SHR1[19:0] */
	/* 0x3055: Using default value */
	/* 0x3056: Using default value */
	{ 0x3058, 0x4A }, /* SHR2[19:0] */
	{ 0x3059, 0x00 },
	/* 0x305A: Using default value */
	{ 0x3060, 0xF3 }, /* RHS1[19:0] */
	{ 0x3061, 0x01 },
	/* 0x3062: Using default value */
	{ 0x3064, 0x30 }, /* RHS2[19:0] */
	{ 0x3065, 0x02 },
	/* 0x3066: Using default value */
	/* 0x3069: Using default value (CHDR_GAIN_EN[7:0]) */
	/* 0x306B: Using default value */
	/* 0x3070: Using default value (GAIN[10:0]) */
	/* 0x3071: Using default value */
	/* 0x3072: Using default value (GAIN_1[10:0]) */
	/* 0x3073: Using default value */
	/* 0x3074: Using default value (GAIN_2[10:0]) */
	/* 0x3075: Using default value */
	/* 0x3081: Using default value (EXP_GAIN[7:0]) */
	/* 0x308C: Using default value (CHDR_DGAIN0_HG[15:0]) */
	/* 0x308D: Using default value */
	/* 0x3094: Using default value (CHDR_AGAIN0_LG[10:0]) */
	/* 0x3095: Using default value */
	/* 0x309C: Using default value (CHDR_AGAIN0_HG[10:0]) */
	/* 0x309D: Using default value */
	/* 0x30A4: Using default value (XVSOUTSEL[1:0]) */
	{ 0x30A6, 0x00 }, /* XVS_DRV[1:0] */
	/* 0x30CC: Using default value */
	/* 0x30CD: Using default value */
	/* 0x30DC: Using default value (BLKLEVEL[11:0]) */
	/* 0x30DD: Using default value */
	{ 0x3400, 0x00 }, /* GAIN_PGC_FIDMD - 0: set individual exposure gains*/
	{ 0x3460, 0x22 },
	{ 0x355A, 0x64 },
	{ 0x3A02, 0x7A },
	{ 0x3A10, 0xEC },
	{ 0x3A12, 0x71 },
	{ 0x3A14, 0xDE },
	{ 0x3A20, 0x2B },
	{ 0x3A24, 0x22 },
	{ 0x3A25, 0x25 },
	{ 0x3A26, 0x2A },
	{ 0x3A27, 0x2C },
	{ 0x3A28, 0x39 },
	{ 0x3A29, 0x38 },
	{ 0x3A30, 0x04 },
	{ 0x3A31, 0x04 },
	{ 0x3A32, 0x03 },
	{ 0x3A33, 0x03 },
	{ 0x3A34, 0x09 },
	{ 0x3A35, 0x06 },
	{ 0x3A38, 0xCD },
	{ 0x3A3A, 0x4C },
	{ 0x3A3C, 0xB9 },
	{ 0x3A3E, 0x30 },
	{ 0x3A40, 0x2C },
	{ 0x3A42, 0x39 },
	{ 0x3A4E, 0x00 },
	{ 0x3A52, 0x00 },
	{ 0x3A56, 0x00 },
	{ 0x3A5A, 0x00 },
	{ 0x3A5E, 0x00 },
	{ 0x3A62, 0x00 },
	/* 0x3A64: Using default value */
	{ 0x3A6E, 0xA0 },
	{ 0x3A70, 0x50 },
	{ 0x3A8C, 0x04 },
	{ 0x3A8D, 0x03 },
	{ 0x3A8E, 0x09 },
	{ 0x3A90, 0x38 },
	{ 0x3A91, 0x42 },
	{ 0x3A92, 0x3C },
	{ 0x3B0E, 0xF3 },
	{ 0x3B12, 0xE5 },
	{ 0x3B27, 0xC0 },
	{ 0x3B2E, 0xEF },
	{ 0x3B30, 0x6A },
	{ 0x3B32, 0xF6 },
	{ 0x3B36, 0xE1 },
	{ 0x3B3A, 0xE8 },
	{ 0x3B5A, 0x17 },
	{ 0x3B5E, 0xEF },
	{ 0x3B60, 0x6A },
	{ 0x3B62, 0xF6 },
	{ 0x3B66, 0xE1 },
	{ 0x3B6A, 0xE8 },
	{ 0x3B88, 0xEC },
	{ 0x3B8A, 0xED },
	{ 0x3B94, 0x71 },
	{ 0x3B96, 0x72 },
	{ 0x3B98, 0xDE },
	{ 0x3B9A, 0xDF },
	{ 0x3C0F, 0x06 },
	{ 0x3C10, 0x06 },
	{ 0x3C11, 0x06 },
	{ 0x3C12, 0x06 },
	{ 0x3C13, 0x06 },
	{ 0x3C18, 0x20 },
	/* 0x3C37: Using default value */
	{ 0x3C3A, 0x7A },
	{ 0x3C40, 0xF4 },
	{ 0x3C48, 0xE6 },
	{ 0x3C54, 0xCE },
	{ 0x3C56, 0xD0 },
	{ 0x3C6C, 0x53 },
	{ 0x3C6E, 0x55 },
	{ 0x3C70, 0xC0 },
	{ 0x3C72, 0xC2 },
	{ 0x3C7E, 0xCE },
	{ 0x3C8C, 0xCF },
	{ 0x3C8E, 0xEB },
	{ 0x3C98, 0x54 },
	{ 0x3C9A, 0x70 },
	{ 0x3C9C, 0xC1 },
	{ 0x3C9E, 0xDD },
	{ 0x3CB0, 0x7A },
	{ 0x3CB2, 0xBA },
	{ 0x3CC8, 0xBC },
	{ 0x3CCA, 0x7C },
	{ 0x3CD4, 0xEA },
	{ 0x3CD5, 0x01 },
	{ 0x3CD6, 0x4A },
	{ 0x3CD8, 0x00 },
	{ 0x3CD9, 0x00 },
	{ 0x3CDA, 0xFF },
	{ 0x3CDB, 0x03 },
	{ 0x3CDC, 0x00 },
	{ 0x3CDD, 0x00 },
	{ 0x3CDE, 0xFF },
	{ 0x3CDF, 0x03 },
	{ 0x3CE4, 0x4C },
	{ 0x3CE6, 0xEC },
	{ 0x3CE7, 0x01 },
	{ 0x3CE8, 0xFF },
	{ 0x3CE9, 0x03 },
	{ 0x3CEA, 0x00 },
	{ 0x3CEB, 0x00 },
	{ 0x3CEC, 0xFF },
	{ 0x3CED, 0x03 },
	{ 0x3CEE, 0x00 },
	{ 0x3CEF, 0x00 },
	/* 0x3CF2: Using default value */
	/* 0x3CF3: Using default value */
	/* 0x3CF4: Using default value */
	{ 0x3E28, 0x82 },
	{ 0x3E2A, 0x80 },
	{ 0x3E30, 0x85 },
	{ 0x3E32, 0x7D },
	{ 0x3E5C, 0xCE },
	{ 0x3E5E, 0xD3 },
	{ 0x3E70, 0x53 },
	{ 0x3E72, 0x58 },
	{ 0x3E74, 0xC0 },
	{ 0x3E76, 0xC5 },
	{ 0x3E78, 0xC0 },
	{ 0x3E79, 0x01 },
	{ 0x3E7A, 0xD4 },
	{ 0x3E7B, 0x01 },
	{ 0x3EB4, 0x0B },
	{ 0x3EB5, 0x02 },
	{ 0x3EB6, 0x4D },
	/* 0x3EB7: Using default value */
	{ 0x3EEC, 0xF3 },
	{ 0x3EEE, 0xE7 },
	{ 0x3F01, 0x01 },
	{ 0x3F24, 0x10 },
	{ 0x3F28, 0x2D },
	{ 0x3F2A, 0x2D },
	{ 0x3F2C, 0x2D },
	{ 0x3F2E, 0x2D },
	{ 0x3F30, 0x23 },
	{ 0x3F38, 0x2D },
	{ 0x3F3A, 0x2D },
	{ 0x3F3C, 0x2D },
	{ 0x3F3E, 0x28 },
	{ 0x3F40, 0x1E },
	{ 0x3F48, 0x2D },
	{ 0x3F4A, 0x2D },
	/* 0x3F4C: Using default value */
	{ 0x4004, 0xE4 },
	{ 0x4006, 0xFF },
	{ 0x4018, 0x69 },
	{ 0x401A, 0x84 },
	{ 0x401C, 0xD6 },
	{ 0x401E, 0xF1 },
	{ 0x4038, 0xDE },
	{ 0x403A, 0x00 },
	{ 0x403B, 0x01 },
	{ 0x404C, 0x63 },
	{ 0x404E, 0x85 },
	{ 0x4050, 0xD0 },
	{ 0x4052, 0xF2 },
	{ 0x4108, 0xDD },
	{ 0x410A, 0xF7 },
	{ 0x411C, 0x62 },
	{ 0x411E, 0x7C },
	{ 0x4120, 0xCF },
	{ 0x4122, 0xE9 },
	{ 0x4138, 0xE6 },
	{ 0x413A, 0xF1 },
	{ 0x414C, 0x6B },
	{ 0x414E, 0x76 },
	{ 0x4150, 0xD8 },
	{ 0x4152, 0xE3 },
	{ 0x417E, 0x03 },
	{ 0x417F, 0x01 },
	{ 0x4186, 0xE0 },
	{ 0x4190, 0xF3 },
	{ 0x4192, 0xF7 },
	{ 0x419C, 0x78 },
	{ 0x419E, 0x7C },
	{ 0x41A0, 0xE5 },
	{ 0x41A2, 0xE9 },
	{ 0x41C8, 0xE2 },
	{ 0x41CA, 0xFD },
	{ 0x41DC, 0x67 },
	{ 0x41DE, 0x82 },
	{ 0x41E0, 0xD4 },
	{ 0x41E2, 0xEF },
	{ 0x4200, 0xDE },
	{ 0x4202, 0xDA },
	{ 0x4218, 0x63 },
	{ 0x421A, 0x5F },
	{ 0x421C, 0xD0 },
	{ 0x421E, 0xCC },
	{ 0x425A, 0x82 },
	{ 0x425C, 0xEF },
	{ 0x4348, 0xFE },
	{ 0x4349, 0x06 },
	{ 0x4352, 0xCE },
	{ 0x4420, 0x0B },
	{ 0x4421, 0x02 },
	{ 0x4422, 0x4D },
	/* 0x4423: Using default value */
	{ 0x4426, 0xF5 },
	{ 0x442A, 0xE7 },
	{ 0x4432, 0xF5 },
	{ 0x4436, 0xE7 },
	{ 0x4466, 0xB4 },
	{ 0x446E, 0x32 },
	{ 0x449F, 0x1C },
	{ 0x44A4, 0x2C },
	{ 0x44A6, 0x2C },
	{ 0x44A8, 0x2C },
	{ 0x44AA, 0x2C },
	{ 0x44B4, 0x2C },
	{ 0x44B6, 0x2C },
	{ 0x44B8, 0x2C },
	{ 0x44BA, 0x2C },
	{ 0x44C4, 0x2C },
	{ 0x44C6, 0x2C },
	{ 0x44C8, 0x2C },
	{ 0x4506, 0xF3 },
	{ 0x450E, 0xE5 },
	{ 0x4516, 0xF3 },
	{ 0x4522, 0xE5 },
	{ 0x4524, 0xF3 },
	{ 0x452C, 0xE5 },
	{ 0x453C, 0x22 },
	{ 0x453D, 0x1B },
	{ 0x453E, 0x1B },
	{ 0x453F, 0x15 },
	{ 0x4540, 0x15 },
	{ 0x4541, 0x15 },
	{ 0x4542, 0x15 },
	{ 0x4543, 0x15 },
	{ 0x4544, 0x15 },
	{ 0x4548, 0x00 },
	{ 0x4549, 0x01 },
	{ 0x454A, 0x01 },
	{ 0x454B, 0x06 },
	{ 0x454C, 0x06 },
	{ 0x454D, 0x06 },
	{ 0x454E, 0x06 },
	{ 0x454F, 0x06 },
	{ 0x4550, 0x06 },
	{ 0x4554, 0x55 },
	{ 0x4555, 0x02 },
	{ 0x4556, 0x42 },
	{ 0x4557, 0x05 },
	{ 0x4558, 0xFD },
	{ 0x4559, 0x05 },
	{ 0x455A, 0x94 },
	{ 0x455B, 0x06 },
	{ 0x455D, 0x06 },
	{ 0x455E, 0x49 },
	{ 0x455F, 0x07 },
	{ 0x4560, 0x7F },
	{ 0x4561, 0x07 },
	{ 0x4562, 0xA5 },
	{ 0x4564, 0x55 },
	{ 0x4565, 0x02 },
	{ 0x4566, 0x42 },
	{ 0x4567, 0x05 },
	{ 0x4568, 0xFD },
	{ 0x4569, 0x05 },
	{ 0x456A, 0x94 },
	{ 0x456B, 0x06 },
	{ 0x456D, 0x06 },
	{ 0x456E, 0x49 },
	{ 0x456F, 0x07 },
	{ 0x4572, 0xA5 },
	{ 0x460C, 0x7D },
	{ 0x460E, 0xB1 },
	{ 0x4614, 0xA8 },
	{ 0x4616, 0xB2 },
	{ 0x461C, 0x7E },
	{ 0x461E, 0xA7 },
	{ 0x4624, 0xA8 },
	{ 0x4626, 0xB2 },
	{ 0x462C, 0x7E },
	{ 0x462E, 0x8A },
	{ 0x4630, 0x94 },
	{ 0x4632, 0xA7 },
	{ 0x4634, 0xFB },
	{ 0x4636, 0x2F },
	{ 0x4638, 0x81 },
	{ 0x4639, 0x01 },
	{ 0x463A, 0xB5 },
	{ 0x463B, 0x01 },
	{ 0x463C, 0x26 },
	{ 0x463E, 0x30 },
	{ 0x4640, 0xAC },
	{ 0x4641, 0x01 },
	{ 0x4642, 0xB6 },
	{ 0x4643, 0x01 },
	{ 0x4644, 0xFC },
	{ 0x4646, 0x25 },
	{ 0x4648, 0x82 },
	{ 0x4649, 0x01 },
	{ 0x464A, 0xAB },
	{ 0x464B, 0x01 },
	{ 0x464C, 0x26 },
	{ 0x464E, 0x30 },
	{ 0x4654, 0xFC },
	{ 0x4656, 0x08 },
	{ 0x4658, 0x12 },
	{ 0x465A, 0x25 },
	{ 0x4662, 0xFC },
	{ 0x46A2, 0xFB },
	{ 0x46D6, 0xF3 },
	{ 0x46E6, 0x00 },
	{ 0x46E8, 0xFF },
	{ 0x46E9, 0x03 },
	{ 0x46EC, 0x7A },
	{ 0x46EE, 0xE5 },
	{ 0x46F4, 0xEE },
	{ 0x46F6, 0xF2 },
	{ 0x470C, 0xFF },
	{ 0x470D, 0x03 },
	{ 0x470E, 0x00 },
	{ 0x4714, 0xE0 },
	{ 0x4716, 0xE4 },
	{ 0x471E, 0xED },
	{ 0x472E, 0x00 },
	{ 0x4730, 0xFF },
	{ 0x4731, 0x03 },
	{ 0x4734, 0x7B },
	{ 0x4736, 0xDF },
	{ 0x4754, 0x7D },
	{ 0x4756, 0x8B },
	{ 0x4758, 0x93 },
	{ 0x475A, 0xB1 },
	{ 0x475C, 0xFB },
	{ 0x475E, 0x09 },
	{ 0x4760, 0x11 },
	{ 0x4762, 0x2F },
	{ 0x4766, 0xCC },
	{ 0x4776, 0xCB },
	{ 0x477E, 0x4A },
	{ 0x478E, 0x49 },
	{ 0x4794, 0x7C },
	{ 0x4796, 0x8F },
	{ 0x4798, 0xB3 },
	{ 0x4799, 0x00 },
	{ 0x479A, 0xCC },
	{ 0x479C, 0xC1 },
	{ 0x479E, 0xCB },
	{ 0x47A4, 0x7D },
	{ 0x47A6, 0x8E },
	{ 0x47A8, 0xB4 },
	{ 0x47A9, 0x00 },
	{ 0x47AA, 0xC0 },
	{ 0x47AC, 0xFA },
	{ 0x47AE, 0x0D },
	{ 0x47B0, 0x31 },
	{ 0x47B1, 0x01 },
	{ 0x47B2, 0x4A },
	{ 0x47B3, 0x01 },
	{ 0x47B4, 0x3F },
	{ 0x47B6, 0x49 },
	{ 0x47BC, 0xFB },
	{ 0x47BE, 0x0C },
	{ 0x47C0, 0x32 },
	{ 0x47C1, 0x01 },
	{ 0x47C2, 0x3E },
	{ 0x47C3, 0x01 },
	{ 0x4E3C, 0x07 },
};

static const struct imx678_reg mode_4k_2dol_all_pixel[] = {
	/* 0x3000: Using default value (STANDBY) */
	/* 0x3001: Using default value (REGHOLD) */
	/* 0x3002: Using default value (XMSTA) */
	{ 0x3014, IMX678_INCLK_CODE }, // MANUAL (default(0x00) -> our clock's value)
	{ 0x3015, 0x02 }, /* DATARATE_SEL[3:0] */
	{ 0x3018, 0x04 }, // MANUAL (default(0x00) -> recommended resolution)
	/* 0x3019: Using default value (CFMODE[1:0]) */
	{ 0x301A, 0x01 }, /* WDMODE[7:0] */
	/* 0x301B: Using default value (ADDMODE[1:0]) */
	{ 0x301C, 0x01 }, /* THIN_V_EN[7:0] */
	/* 0x301E: Using default value (VCMODE[7:0]) */
	/* 0x3020: Using default value (HREVERSE) */
	/* 0x3021: Using default value (VREVERSE) */
	/* 0x3022: Using default value (ADBIT[1:0]) */
	{ 0x3023, 0x01 }, // MANUAL (default(0x01) -> same value, write explicitly)
	/* 0x3028: Using default value (VMAX[19:0]) */
	/* 0x3029: Using default value */
	/* 0x302A: Using default value */
	{ 0x302C, 0x26 }, /* HMAX[15:0] */
	{ 0x302D, 0x02 },
	/* 0x3030: Using default value (FDG_SEL0[1:0]) */
	/* 0x3031: Using default value (FDG_SEL1[1:0]) */
	/* 0x3032: Using default value (FDG_SEL2[1:0]) */
	/* 0x303C: Using default value (PIX_HST[12:0]) */
	/* 0x303D: Using default value */
	{ 0x303E, 0x00 }, // MANUAL (default(0x10) -> recommended resolution)
	{ 0x303F, 0x0F }, // MANUAL (default(0x0F) -> recommended resolution)
	/* 0x3040: Using default value (LANEMODE[2:0]) */
	/* 0x3042: Using default value (XSIZE_OVERLAP[10:0]) */
	/* 0x3043: Using default value */
	/* 0x3044: Using default value (PIX_VST[11:0]) */
	/* 0x3045: Using default value */
	{ 0x3046, 0x70 }, // MANUAL (default(0x84) -> recommended resolution)
	{ 0x3047, 0x08 }, // MANUAL (default(0x08) -> recommended resolution)
	{ 0x3050, 0xEC }, /* SHR0[19:0] */
	{ 0x3051, 0x04 },
	/* 0x3052: Using default value */
	{ 0x3054, 0x05 }, /* SHR1[19:0] */
	/* 0x3055: Using default value */
	/* 0x3056: Using default value */
	{ 0x3058, 0x4A }, /* SHR2[19:0] */
	{ 0x3059, 0x00 },
	/* 0x305A: Using default value */
	{ 0x3060, 0x1B }, // MANUAL (increase max SEF value)
	{ 0x3061, 0x01 }, // MANUAL (increase max SEF value)
	/* 0x3062: Using default value */
	{ 0x3064, 0x53 }, /* RHS2[19:0] */
	{ 0x3065, 0x00 },
	/* 0x3066: Using default value */
	/* 0x3069: Using default value (CHDR_GAIN_EN[7:0]) */
	/* 0x306B: Using default value */
	/* 0x3070: Using default value (GAIN[10:0]) */
	/* 0x3071: Using default value */
	/* 0x3072: Using default value (GAIN_1[10:0]) */
	/* 0x3073: Using default value */
	/* 0x3074: Using default value (GAIN_2[10:0]) */
	/* 0x3075: Using default value */
	/* 0x3081: Using default value (EXP_GAIN[7:0]) */
	/* 0x308C: Using default value (CHDR_DGAIN0_HG[15:0]) */
	/* 0x308D: Using default value */
	/* 0x3094: Using default value (CHDR_AGAIN0_LG[10:0]) */
	/* 0x3095: Using default value */
	/* 0x309C: Using default value (CHDR_AGAIN0_HG[10:0]) */
	/* 0x309D: Using default value */
	/* 0x30A4: Using default value (XVSOUTSEL[1:0]) */
	{ 0x30A6, 0x00 }, /* XVS_DRV[1:0] */
	/* 0x30CC: Using default value */
	/* 0x30CD: Using default value */
	/* 0x30DC: Using default value (BLKLEVEL[11:0]) */
	/* 0x30DD: Using default value */
	{ 0x3400, 0x00 }, /* GAIN_PGC_FIDMD - 0: set individual exposure gains*/
	{ 0x3460, 0x22 },
	{ 0x355A, 0x64 },
	{ 0x3A02, 0x7A },
	{ 0x3A10, 0xEC },
	{ 0x3A12, 0x71 },
	{ 0x3A14, 0xDE },
	{ 0x3A20, 0x2B },
	{ 0x3A24, 0x22 },
	{ 0x3A25, 0x25 },
	{ 0x3A26, 0x2A },
	{ 0x3A27, 0x2C },
	{ 0x3A28, 0x39 },
	{ 0x3A29, 0x38 },
	{ 0x3A30, 0x04 },
	{ 0x3A31, 0x04 },
	{ 0x3A32, 0x03 },
	{ 0x3A33, 0x03 },
	{ 0x3A34, 0x09 },
	{ 0x3A35, 0x06 },
	{ 0x3A38, 0xCD },
	{ 0x3A3A, 0x4C },
	{ 0x3A3C, 0xB9 },
	{ 0x3A3E, 0x30 },
	{ 0x3A40, 0x2C },
	{ 0x3A42, 0x39 },
	{ 0x3A4E, 0x00 },
	{ 0x3A52, 0x00 },
	{ 0x3A56, 0x00 },
	{ 0x3A5A, 0x00 },
	{ 0x3A5E, 0x00 },
	{ 0x3A62, 0x00 },
	/* 0x3A64: Using default value */
	{ 0x3A6E, 0xA0 },
	{ 0x3A70, 0x50 },
	{ 0x3A8C, 0x04 },
	{ 0x3A8D, 0x03 },
	{ 0x3A8E, 0x09 },
	{ 0x3A90, 0x38 },
	{ 0x3A91, 0x42 },
	{ 0x3A92, 0x3C },
	{ 0x3B0E, 0xF3 },
	{ 0x3B12, 0xE5 },
	{ 0x3B27, 0xC0 },
	{ 0x3B2E, 0xEF },
	{ 0x3B30, 0x6A },
	{ 0x3B32, 0xF6 },
	{ 0x3B36, 0xE1 },
	{ 0x3B3A, 0xE8 },
	{ 0x3B5A, 0x17 },
	{ 0x3B5E, 0xEF },
	{ 0x3B60, 0x6A },
	{ 0x3B62, 0xF6 },
	{ 0x3B66, 0xE1 },
	{ 0x3B6A, 0xE8 },
	{ 0x3B88, 0xEC },
	{ 0x3B8A, 0xED },
	{ 0x3B94, 0x71 },
	{ 0x3B96, 0x72 },
	{ 0x3B98, 0xDE },
	{ 0x3B9A, 0xDF },
	{ 0x3C0F, 0x06 },
	{ 0x3C10, 0x06 },
	{ 0x3C11, 0x06 },
	{ 0x3C12, 0x06 },
	{ 0x3C13, 0x06 },
	{ 0x3C18, 0x20 },
	/* 0x3C37: Using default value */
	{ 0x3C3A, 0x7A },
	{ 0x3C40, 0xF4 },
	{ 0x3C48, 0xE6 },
	{ 0x3C54, 0xCE },
	{ 0x3C56, 0xD0 },
	{ 0x3C6C, 0x53 },
	{ 0x3C6E, 0x55 },
	{ 0x3C70, 0xC0 },
	{ 0x3C72, 0xC2 },
	{ 0x3C7E, 0xCE },
	{ 0x3C8C, 0xCF },
	{ 0x3C8E, 0xEB },
	{ 0x3C98, 0x54 },
	{ 0x3C9A, 0x70 },
	{ 0x3C9C, 0xC1 },
	{ 0x3C9E, 0xDD },
	{ 0x3CB0, 0x7A },
	{ 0x3CB2, 0xBA },
	{ 0x3CC8, 0xBC },
	{ 0x3CCA, 0x7C },
	{ 0x3CD4, 0xEA },
	{ 0x3CD5, 0x01 },
	{ 0x3CD6, 0x4A },
	{ 0x3CD8, 0x00 },
	{ 0x3CD9, 0x00 },
	{ 0x3CDA, 0xFF },
	{ 0x3CDB, 0x03 },
	{ 0x3CDC, 0x00 },
	{ 0x3CDD, 0x00 },
	{ 0x3CDE, 0xFF },
	{ 0x3CDF, 0x03 },
	{ 0x3CE4, 0x4C },
	{ 0x3CE6, 0xEC },
	{ 0x3CE7, 0x01 },
	{ 0x3CE8, 0xFF },
	{ 0x3CE9, 0x03 },
	{ 0x3CEA, 0x00 },
	{ 0x3CEB, 0x00 },
	{ 0x3CEC, 0xFF },
	{ 0x3CED, 0x03 },
	{ 0x3CEE, 0x00 },
	{ 0x3CEF, 0x00 },
	/* 0x3CF2: Using default value */
	/* 0x3CF3: Using default value */
	/* 0x3CF4: Using default value */
	{ 0x3E28, 0x82 },
	{ 0x3E2A, 0x80 },
	{ 0x3E30, 0x85 },
	{ 0x3E32, 0x7D },
	{ 0x3E5C, 0xCE },
	{ 0x3E5E, 0xD3 },
	{ 0x3E70, 0x53 },
	{ 0x3E72, 0x58 },
	{ 0x3E74, 0xC0 },
	{ 0x3E76, 0xC5 },
	{ 0x3E78, 0xC0 },
	{ 0x3E79, 0x01 },
	{ 0x3E7A, 0xD4 },
	{ 0x3E7B, 0x01 },
	{ 0x3EB4, 0x0B },
	{ 0x3EB5, 0x02 },
	{ 0x3EB6, 0x4D },
	/* 0x3EB7: Using default value */
	{ 0x3EEC, 0xF3 },
	{ 0x3EEE, 0xE7 },
	{ 0x3F01, 0x01 },
	{ 0x3F24, 0x10 },
	{ 0x3F28, 0x2D },
	{ 0x3F2A, 0x2D },
	{ 0x3F2C, 0x2D },
	{ 0x3F2E, 0x2D },
	{ 0x3F30, 0x23 },
	{ 0x3F38, 0x2D },
	{ 0x3F3A, 0x2D },
	{ 0x3F3C, 0x2D },
	{ 0x3F3E, 0x28 },
	{ 0x3F40, 0x1E },
	{ 0x3F48, 0x2D },
	{ 0x3F4A, 0x2D },
	/* 0x3F4C: Using default value */
	{ 0x4004, 0xE4 },
	{ 0x4006, 0xFF },
	{ 0x4018, 0x69 },
	{ 0x401A, 0x84 },
	{ 0x401C, 0xD6 },
	{ 0x401E, 0xF1 },
	{ 0x4038, 0xDE },
	{ 0x403A, 0x00 },
	{ 0x403B, 0x01 },
	{ 0x404C, 0x63 },
	{ 0x404E, 0x85 },
	{ 0x4050, 0xD0 },
	{ 0x4052, 0xF2 },
	{ 0x4108, 0xDD },
	{ 0x410A, 0xF7 },
	{ 0x411C, 0x62 },
	{ 0x411E, 0x7C },
	{ 0x4120, 0xCF },
	{ 0x4122, 0xE9 },
	{ 0x4138, 0xE6 },
	{ 0x413A, 0xF1 },
	{ 0x414C, 0x6B },
	{ 0x414E, 0x76 },
	{ 0x4150, 0xD8 },
	{ 0x4152, 0xE3 },
	{ 0x417E, 0x03 },
	{ 0x417F, 0x01 },
	{ 0x4186, 0xE0 },
	{ 0x4190, 0xF3 },
	{ 0x4192, 0xF7 },
	{ 0x419C, 0x78 },
	{ 0x419E, 0x7C },
	{ 0x41A0, 0xE5 },
	{ 0x41A2, 0xE9 },
	{ 0x41C8, 0xE2 },
	{ 0x41CA, 0xFD },
	{ 0x41DC, 0x67 },
	{ 0x41DE, 0x82 },
	{ 0x41E0, 0xD4 },
	{ 0x41E2, 0xEF },
	{ 0x4200, 0xDE },
	{ 0x4202, 0xDA },
	{ 0x4218, 0x63 },
	{ 0x421A, 0x5F },
	{ 0x421C, 0xD0 },
	{ 0x421E, 0xCC },
	{ 0x425A, 0x82 },
	{ 0x425C, 0xEF },
	{ 0x4348, 0xFE },
	{ 0x4349, 0x06 },
	{ 0x4352, 0xCE },
	{ 0x4420, 0x0B },
	{ 0x4421, 0x02 },
	{ 0x4422, 0x4D },
	/* 0x4423: Using default value */
	{ 0x4426, 0xF5 },
	{ 0x442A, 0xE7 },
	{ 0x4432, 0xF5 },
	{ 0x4436, 0xE7 },
	{ 0x4466, 0xB4 },
	{ 0x446E, 0x32 },
	{ 0x449F, 0x1C },
	{ 0x44A4, 0x2C },
	{ 0x44A6, 0x2C },
	{ 0x44A8, 0x2C },
	{ 0x44AA, 0x2C },
	{ 0x44B4, 0x2C },
	{ 0x44B6, 0x2C },
	{ 0x44B8, 0x2C },
	{ 0x44BA, 0x2C },
	{ 0x44C4, 0x2C },
	{ 0x44C6, 0x2C },
	{ 0x44C8, 0x2C },
	{ 0x4506, 0xF3 },
	{ 0x450E, 0xE5 },
	{ 0x4516, 0xF3 },
	{ 0x4522, 0xE5 },
	{ 0x4524, 0xF3 },
	{ 0x452C, 0xE5 },
	{ 0x453C, 0x22 },
	{ 0x453D, 0x1B },
	{ 0x453E, 0x1B },
	{ 0x453F, 0x15 },
	{ 0x4540, 0x15 },
	{ 0x4541, 0x15 },
	{ 0x4542, 0x15 },
	{ 0x4543, 0x15 },
	{ 0x4544, 0x15 },
	{ 0x4548, 0x00 },
	{ 0x4549, 0x01 },
	{ 0x454A, 0x01 },
	{ 0x454B, 0x06 },
	{ 0x454C, 0x06 },
	{ 0x454D, 0x06 },
	{ 0x454E, 0x06 },
	{ 0x454F, 0x06 },
	{ 0x4550, 0x06 },
	{ 0x4554, 0x55 },
	{ 0x4555, 0x02 },
	{ 0x4556, 0x42 },
	{ 0x4557, 0x05 },
	{ 0x4558, 0xFD },
	{ 0x4559, 0x05 },
	{ 0x455A, 0x94 },
	{ 0x455B, 0x06 },
	{ 0x455D, 0x06 },
	{ 0x455E, 0x49 },
	{ 0x455F, 0x07 },
	{ 0x4560, 0x7F },
	{ 0x4561, 0x07 },
	{ 0x4562, 0xA5 },
	{ 0x4564, 0x55 },
	{ 0x4565, 0x02 },
	{ 0x4566, 0x42 },
	{ 0x4567, 0x05 },
	{ 0x4568, 0xFD },
	{ 0x4569, 0x05 },
	{ 0x456A, 0x94 },
	{ 0x456B, 0x06 },
	{ 0x456D, 0x06 },
	{ 0x456E, 0x49 },
	{ 0x456F, 0x07 },
	{ 0x4572, 0xA5 },
	{ 0x460C, 0x7D },
	{ 0x460E, 0xB1 },
	{ 0x4614, 0xA8 },
	{ 0x4616, 0xB2 },
	{ 0x461C, 0x7E },
	{ 0x461E, 0xA7 },
	{ 0x4624, 0xA8 },
	{ 0x4626, 0xB2 },
	{ 0x462C, 0x7E },
	{ 0x462E, 0x8A },
	{ 0x4630, 0x94 },
	{ 0x4632, 0xA7 },
	{ 0x4634, 0xFB },
	{ 0x4636, 0x2F },
	{ 0x4638, 0x81 },
	{ 0x4639, 0x01 },
	{ 0x463A, 0xB5 },
	{ 0x463B, 0x01 },
	{ 0x463C, 0x26 },
	{ 0x463E, 0x30 },
	{ 0x4640, 0xAC },
	{ 0x4641, 0x01 },
	{ 0x4642, 0xB6 },
	{ 0x4643, 0x01 },
	{ 0x4644, 0xFC },
	{ 0x4646, 0x25 },
	{ 0x4648, 0x82 },
	{ 0x4649, 0x01 },
	{ 0x464A, 0xAB },
	{ 0x464B, 0x01 },
	{ 0x464C, 0x26 },
	{ 0x464E, 0x30 },
	{ 0x4654, 0xFC },
	{ 0x4656, 0x08 },
	{ 0x4658, 0x12 },
	{ 0x465A, 0x25 },
	{ 0x4662, 0xFC },
	{ 0x46A2, 0xFB },
	{ 0x46D6, 0xF3 },
	{ 0x46E6, 0x00 },
	{ 0x46E8, 0xFF },
	{ 0x46E9, 0x03 },
	{ 0x46EC, 0x7A },
	{ 0x46EE, 0xE5 },
	{ 0x46F4, 0xEE },
	{ 0x46F6, 0xF2 },
	{ 0x470C, 0xFF },
	{ 0x470D, 0x03 },
	{ 0x470E, 0x00 },
	{ 0x4714, 0xE0 },
	{ 0x4716, 0xE4 },
	{ 0x471E, 0xED },
	{ 0x472E, 0x00 },
	{ 0x4730, 0xFF },
	{ 0x4731, 0x03 },
	{ 0x4734, 0x7B },
	{ 0x4736, 0xDF },
	{ 0x4754, 0x7D },
	{ 0x4756, 0x8B },
	{ 0x4758, 0x93 },
	{ 0x475A, 0xB1 },
	{ 0x475C, 0xFB },
	{ 0x475E, 0x09 },
	{ 0x4760, 0x11 },
	{ 0x4762, 0x2F },
	{ 0x4766, 0xCC },
	{ 0x4776, 0xCB },
	{ 0x477E, 0x4A },
	{ 0x478E, 0x49 },
	{ 0x4794, 0x7C },
	{ 0x4796, 0x8F },
	{ 0x4798, 0xB3 },
	{ 0x4799, 0x00 },
	{ 0x479A, 0xCC },
	{ 0x479C, 0xC1 },
	{ 0x479E, 0xCB },
	{ 0x47A4, 0x7D },
	{ 0x47A6, 0x8E },
	{ 0x47A8, 0xB4 },
	{ 0x47A9, 0x00 },
	{ 0x47AA, 0xC0 },
	{ 0x47AC, 0xFA },
	{ 0x47AE, 0x0D },
	{ 0x47B0, 0x31 },
	{ 0x47B1, 0x01 },
	{ 0x47B2, 0x4A },
	{ 0x47B3, 0x01 },
	{ 0x47B4, 0x3F },
	{ 0x47B6, 0x49 },
	{ 0x47BC, 0xFB },
	{ 0x47BE, 0x0C },
	{ 0x47C0, 0x32 },
	{ 0x47C1, 0x01 },
	{ 0x47C2, 0x3E },
	{ 0x47C3, 0x01 },
	{ 0x4E3C, 0x07 },
};


static const struct imx678_reg mode_4k_3dol_all_pixel[] = {
	/* 0x3000: Using default value (STANDBY) */
	/* 0x3001: Using default value (REGHOLD) */
	/* 0x3002: Using default value (XMSTA) */
	{ 0x3014, IMX678_INCLK_CODE }, // MANUAL (default(0x00) -> our clock's value)
	{ 0x3015, 0x05 }, /* DATARATE_SEL[3:0] */
	{ 0x3018, 0x04 }, // MANUAL (default(0x00) -> recommended resolution)
	/* 0x3019: Using default value (CFMODE[1:0]) */
	{ 0x301A, 0x02 }, /* WDMODE[7:0] */
	/* 0x301B: Using default value (ADDMODE[1:0]) */
	{ 0x301C, 0x01 }, /* THIN_V_EN[7:0] */
	/* 0x301E: Using default value (VCMODE[7:0]) */
	/* 0x3020: Using default value (HREVERSE) */
	/* 0x3021: Using default value (VREVERSE) */
	/* 0x3022: Using default value (ADBIT[1:0]) */
	{ 0x3023, 0x01 }, // MANUAL (default(0x01) -> same value, write explicitly)
	/* 0x3028: Using default value (VMAX[19:0]) */
	/* 0x3029: Using default value */
	/* 0x302A: Using default value */
	{ 0x302C, 0x28 }, /* HMAX[15:0] */
	{ 0x302D, 0x05 },
	/* 0x3030: Using default value (FDG_SEL0[1:0]) */
	/* 0x3031: Using default value (FDG_SEL1[1:0]) */
	/* 0x3032: Using default value (FDG_SEL2[1:0]) */
	/* 0x303C: Using default value (PIX_HST[12:0]) */
	/* 0x303D: Using default value */
	{ 0x303E, 0x00 }, // MANUAL (default(0x10) -> recommended resolution)
	{ 0x303F, 0x0F }, // MANUAL (default(0x0F) -> recommended resolution)
	/* 0x3040: Using default value (LANEMODE[2:0]) */
	/* 0x3042: Using default value (XSIZE_OVERLAP[10:0]) */
	/* 0x3043: Using default value */
	/* 0x3044: Using default value (PIX_VST[11:0]) */
	/* 0x3045: Using default value */
	{ 0x3046, 0x70 }, // MANUAL (default(0x84) -> recommended resolution)
	{ 0x3047, 0x08 }, // MANUAL (default(0x08) -> recommended resolution)
	{ 0x3050, 0x18 }, /* SHR0[19:0] */
	{ 0x3051, 0x15 },
	/* 0x3052: Using default value */
	{ 0x3054, 0x07 }, /* SHR1[19:0] */
	/* 0x3055: Using default value */
	/* 0x3056: Using default value */
	{ 0x3058, 0x4A }, /* SHR2[19:0] */
	{ 0x3059, 0x00 },
	/* 0x305A: Using default value */
	{ 0x3060, 0x40 }, /* RHS1[19:0] */
	{ 0x3061, 0x00 },
	/* 0x3062: Using default value */
	{ 0x3064, 0x53 }, /* RHS2[19:0] */
	{ 0x3065, 0x00 },
	/* 0x3066: Using default value */
	/* 0x3069: Using default value (CHDR_GAIN_EN[7:0]) */
	/* 0x306B: Using default value */
	/* 0x3070: Using default value (GAIN[10:0]) */
	/* 0x3071: Using default value */
	/* 0x3072: Using default value (GAIN_1[10:0]) */
	/* 0x3073: Using default value */
	/* 0x3074: Using default value (GAIN_2[10:0]) */
	/* 0x3075: Using default value */
	/* 0x3081: Using default value (EXP_GAIN[7:0]) */
	/* 0x308C: Using default value (CHDR_DGAIN0_HG[15:0]) */
	/* 0x308D: Using default value */
	/* 0x3094: Using default value (CHDR_AGAIN0_LG[10:0]) */
	/* 0x3095: Using default value */
	/* 0x309C: Using default value (CHDR_AGAIN0_HG[10:0]) */
	/* 0x309D: Using default value */
	/* 0x30A4: Using default value (XVSOUTSEL[1:0]) */
	{ 0x30A6, 0x00 }, /* XVS_DRV[1:0] */
	/* 0x30CC: Using default value */
	/* 0x30CD: Using default value */
	/* 0x30DC: Using default value (BLKLEVEL[11:0]) */
	/* 0x30DD: Using default value */
	{ 0x3400, 0x00 }, /* GAIN_PGC_FIDMD - 0: set individual exposure gains*/
	{ 0x3460, 0x22 },
	{ 0x355A, 0x64 },
	{ 0x3A02, 0x7A },
	{ 0x3A10, 0xEC },
	{ 0x3A12, 0x71 },
	{ 0x3A14, 0xDE },
	{ 0x3A20, 0x2B },
	{ 0x3A24, 0x22 },
	{ 0x3A25, 0x25 },
	{ 0x3A26, 0x2A },
	{ 0x3A27, 0x2C },
	{ 0x3A28, 0x39 },
	{ 0x3A29, 0x38 },
	{ 0x3A30, 0x04 },
	{ 0x3A31, 0x04 },
	{ 0x3A32, 0x03 },
	{ 0x3A33, 0x03 },
	{ 0x3A34, 0x09 },
	{ 0x3A35, 0x06 },
	{ 0x3A38, 0xCD },
	{ 0x3A3A, 0x4C },
	{ 0x3A3C, 0xB9 },
	{ 0x3A3E, 0x30 },
	{ 0x3A40, 0x2C },
	{ 0x3A42, 0x39 },
	{ 0x3A4E, 0x00 },
	{ 0x3A52, 0x00 },
	{ 0x3A56, 0x00 },
	{ 0x3A5A, 0x00 },
	{ 0x3A5E, 0x00 },
	{ 0x3A62, 0x00 },
	/* 0x3A64: Using default value */
	{ 0x3A6E, 0xA0 },
	{ 0x3A70, 0x50 },
	{ 0x3A8C, 0x04 },
	{ 0x3A8D, 0x03 },
	{ 0x3A8E, 0x09 },
	{ 0x3A90, 0x38 },
	{ 0x3A91, 0x42 },
	{ 0x3A92, 0x3C },
	{ 0x3B0E, 0xF3 },
	{ 0x3B12, 0xE5 },
	{ 0x3B27, 0xC0 },
	{ 0x3B2E, 0xEF },
	{ 0x3B30, 0x6A },
	{ 0x3B32, 0xF6 },
	{ 0x3B36, 0xE1 },
	{ 0x3B3A, 0xE8 },
	{ 0x3B5A, 0x17 },
	{ 0x3B5E, 0xEF },
	{ 0x3B60, 0x6A },
	{ 0x3B62, 0xF6 },
	{ 0x3B66, 0xE1 },
	{ 0x3B6A, 0xE8 },
	{ 0x3B88, 0xEC },
	{ 0x3B8A, 0xED },
	{ 0x3B94, 0x71 },
	{ 0x3B96, 0x72 },
	{ 0x3B98, 0xDE },
	{ 0x3B9A, 0xDF },
	{ 0x3C0F, 0x06 },
	{ 0x3C10, 0x06 },
	{ 0x3C11, 0x06 },
	{ 0x3C12, 0x06 },
	{ 0x3C13, 0x06 },
	{ 0x3C18, 0x20 },
	/* 0x3C37: Using default value */
	{ 0x3C3A, 0x7A },
	{ 0x3C40, 0xF4 },
	{ 0x3C48, 0xE6 },
	{ 0x3C54, 0xCE },
	{ 0x3C56, 0xD0 },
	{ 0x3C6C, 0x53 },
	{ 0x3C6E, 0x55 },
	{ 0x3C70, 0xC0 },
	{ 0x3C72, 0xC2 },
	{ 0x3C7E, 0xCE },
	{ 0x3C8C, 0xCF },
	{ 0x3C8E, 0xEB },
	{ 0x3C98, 0x54 },
	{ 0x3C9A, 0x70 },
	{ 0x3C9C, 0xC1 },
	{ 0x3C9E, 0xDD },
	{ 0x3CB0, 0x7A },
	{ 0x3CB2, 0xBA },
	{ 0x3CC8, 0xBC },
	{ 0x3CCA, 0x7C },
	{ 0x3CD4, 0xEA },
	{ 0x3CD5, 0x01 },
	{ 0x3CD6, 0x4A },
	{ 0x3CD8, 0x00 },
	{ 0x3CD9, 0x00 },
	{ 0x3CDA, 0xFF },
	{ 0x3CDB, 0x03 },
	{ 0x3CDC, 0x00 },
	{ 0x3CDD, 0x00 },
	{ 0x3CDE, 0xFF },
	{ 0x3CDF, 0x03 },
	{ 0x3CE4, 0x4C },
	{ 0x3CE6, 0xEC },
	{ 0x3CE7, 0x01 },
	{ 0x3CE8, 0xFF },
	{ 0x3CE9, 0x03 },
	{ 0x3CEA, 0x00 },
	{ 0x3CEB, 0x00 },
	{ 0x3CEC, 0xFF },
	{ 0x3CED, 0x03 },
	{ 0x3CEE, 0x00 },
	{ 0x3CEF, 0x00 },
	/* 0x3CF2: Using default value */
	/* 0x3CF3: Using default value */
	/* 0x3CF4: Using default value */
	{ 0x3E28, 0x82 },
	{ 0x3E2A, 0x80 },
	{ 0x3E30, 0x85 },
	{ 0x3E32, 0x7D },
	{ 0x3E5C, 0xCE },
	{ 0x3E5E, 0xD3 },
	{ 0x3E70, 0x53 },
	{ 0x3E72, 0x58 },
	{ 0x3E74, 0xC0 },
	{ 0x3E76, 0xC5 },
	{ 0x3E78, 0xC0 },
	{ 0x3E79, 0x01 },
	{ 0x3E7A, 0xD4 },
	{ 0x3E7B, 0x01 },
	{ 0x3EB4, 0x0B },
	{ 0x3EB5, 0x02 },
	{ 0x3EB6, 0x4D },
	/* 0x3EB7: Using default value */
	{ 0x3EEC, 0xF3 },
	{ 0x3EEE, 0xE7 },
	{ 0x3F01, 0x01 },
	{ 0x3F24, 0x10 },
	{ 0x3F28, 0x2D },
	{ 0x3F2A, 0x2D },
	{ 0x3F2C, 0x2D },
	{ 0x3F2E, 0x2D },
	{ 0x3F30, 0x23 },
	{ 0x3F38, 0x2D },
	{ 0x3F3A, 0x2D },
	{ 0x3F3C, 0x2D },
	{ 0x3F3E, 0x28 },
	{ 0x3F40, 0x1E },
	{ 0x3F48, 0x2D },
	{ 0x3F4A, 0x2D },
	/* 0x3F4C: Using default value */
	{ 0x4004, 0xE4 },
	{ 0x4006, 0xFF },
	{ 0x4018, 0x69 },
	{ 0x401A, 0x84 },
	{ 0x401C, 0xD6 },
	{ 0x401E, 0xF1 },
	{ 0x4038, 0xDE },
	{ 0x403A, 0x00 },
	{ 0x403B, 0x01 },
	{ 0x404C, 0x63 },
	{ 0x404E, 0x85 },
	{ 0x4050, 0xD0 },
	{ 0x4052, 0xF2 },
	{ 0x4108, 0xDD },
	{ 0x410A, 0xF7 },
	{ 0x411C, 0x62 },
	{ 0x411E, 0x7C },
	{ 0x4120, 0xCF },
	{ 0x4122, 0xE9 },
	{ 0x4138, 0xE6 },
	{ 0x413A, 0xF1 },
	{ 0x414C, 0x6B },
	{ 0x414E, 0x76 },
	{ 0x4150, 0xD8 },
	{ 0x4152, 0xE3 },
	{ 0x417E, 0x03 },
	{ 0x417F, 0x01 },
	{ 0x4186, 0xE0 },
	{ 0x4190, 0xF3 },
	{ 0x4192, 0xF7 },
	{ 0x419C, 0x78 },
	{ 0x419E, 0x7C },
	{ 0x41A0, 0xE5 },
	{ 0x41A2, 0xE9 },
	{ 0x41C8, 0xE2 },
	{ 0x41CA, 0xFD },
	{ 0x41DC, 0x67 },
	{ 0x41DE, 0x82 },
	{ 0x41E0, 0xD4 },
	{ 0x41E2, 0xEF },
	{ 0x4200, 0xDE },
	{ 0x4202, 0xDA },
	{ 0x4218, 0x63 },
	{ 0x421A, 0x5F },
	{ 0x421C, 0xD0 },
	{ 0x421E, 0xCC },
	{ 0x425A, 0x82 },
	{ 0x425C, 0xEF },
	{ 0x4348, 0xFE },
	{ 0x4349, 0x06 },
	{ 0x4352, 0xCE },
	{ 0x4420, 0x0B },
	{ 0x4421, 0x02 },
	{ 0x4422, 0x4D },
	/* 0x4423: Using default value */
	{ 0x4426, 0xF5 },
	{ 0x442A, 0xE7 },
	{ 0x4432, 0xF5 },
	{ 0x4436, 0xE7 },
	{ 0x4466, 0xB4 },
	{ 0x446E, 0x32 },
	{ 0x449F, 0x1C },
	{ 0x44A4, 0x2C },
	{ 0x44A6, 0x2C },
	{ 0x44A8, 0x2C },
	{ 0x44AA, 0x2C },
	{ 0x44B4, 0x2C },
	{ 0x44B6, 0x2C },
	{ 0x44B8, 0x2C },
	{ 0x44BA, 0x2C },
	{ 0x44C4, 0x2C },
	{ 0x44C6, 0x2C },
	{ 0x44C8, 0x2C },
	{ 0x4506, 0xF3 },
	{ 0x450E, 0xE5 },
	{ 0x4516, 0xF3 },
	{ 0x4522, 0xE5 },
	{ 0x4524, 0xF3 },
	{ 0x452C, 0xE5 },
	{ 0x453C, 0x22 },
	{ 0x453D, 0x1B },
	{ 0x453E, 0x1B },
	{ 0x453F, 0x15 },
	{ 0x4540, 0x15 },
	{ 0x4541, 0x15 },
	{ 0x4542, 0x15 },
	{ 0x4543, 0x15 },
	{ 0x4544, 0x15 },
	{ 0x4548, 0x00 },
	{ 0x4549, 0x01 },
	{ 0x454A, 0x01 },
	{ 0x454B, 0x06 },
	{ 0x454C, 0x06 },
	{ 0x454D, 0x06 },
	{ 0x454E, 0x06 },
	{ 0x454F, 0x06 },
	{ 0x4550, 0x06 },
	{ 0x4554, 0x55 },
	{ 0x4555, 0x02 },
	{ 0x4556, 0x42 },
	{ 0x4557, 0x05 },
	{ 0x4558, 0xFD },
	{ 0x4559, 0x05 },
	{ 0x455A, 0x94 },
	{ 0x455B, 0x06 },
	{ 0x455D, 0x06 },
	{ 0x455E, 0x49 },
	{ 0x455F, 0x07 },
	{ 0x4560, 0x7F },
	{ 0x4561, 0x07 },
	{ 0x4562, 0xA5 },
	{ 0x4564, 0x55 },
	{ 0x4565, 0x02 },
	{ 0x4566, 0x42 },
	{ 0x4567, 0x05 },
	{ 0x4568, 0xFD },
	{ 0x4569, 0x05 },
	{ 0x456A, 0x94 },
	{ 0x456B, 0x06 },
	{ 0x456D, 0x06 },
	{ 0x456E, 0x49 },
	{ 0x456F, 0x07 },
	{ 0x4572, 0xA5 },
	{ 0x460C, 0x7D },
	{ 0x460E, 0xB1 },
	{ 0x4614, 0xA8 },
	{ 0x4616, 0xB2 },
	{ 0x461C, 0x7E },
	{ 0x461E, 0xA7 },
	{ 0x4624, 0xA8 },
	{ 0x4626, 0xB2 },
	{ 0x462C, 0x7E },
	{ 0x462E, 0x8A },
	{ 0x4630, 0x94 },
	{ 0x4632, 0xA7 },
	{ 0x4634, 0xFB },
	{ 0x4636, 0x2F },
	{ 0x4638, 0x81 },
	{ 0x4639, 0x01 },
	{ 0x463A, 0xB5 },
	{ 0x463B, 0x01 },
	{ 0x463C, 0x26 },
	{ 0x463E, 0x30 },
	{ 0x4640, 0xAC },
	{ 0x4641, 0x01 },
	{ 0x4642, 0xB6 },
	{ 0x4643, 0x01 },
	{ 0x4644, 0xFC },
	{ 0x4646, 0x25 },
	{ 0x4648, 0x82 },
	{ 0x4649, 0x01 },
	{ 0x464A, 0xAB },
	{ 0x464B, 0x01 },
	{ 0x464C, 0x26 },
	{ 0x464E, 0x30 },
	{ 0x4654, 0xFC },
	{ 0x4656, 0x08 },
	{ 0x4658, 0x12 },
	{ 0x465A, 0x25 },
	{ 0x4662, 0xFC },
	{ 0x46A2, 0xFB },
	{ 0x46D6, 0xF3 },
	{ 0x46E6, 0x00 },
	{ 0x46E8, 0xFF },
	{ 0x46E9, 0x03 },
	{ 0x46EC, 0x7A },
	{ 0x46EE, 0xE5 },
	{ 0x46F4, 0xEE },
	{ 0x46F6, 0xF2 },
	{ 0x470C, 0xFF },
	{ 0x470D, 0x03 },
	{ 0x470E, 0x00 },
	{ 0x4714, 0xE0 },
	{ 0x4716, 0xE4 },
	{ 0x471E, 0xED },
	{ 0x472E, 0x00 },
	{ 0x4730, 0xFF },
	{ 0x4731, 0x03 },
	{ 0x4734, 0x7B },
	{ 0x4736, 0xDF },
	{ 0x4754, 0x7D },
	{ 0x4756, 0x8B },
	{ 0x4758, 0x93 },
	{ 0x475A, 0xB1 },
	{ 0x475C, 0xFB },
	{ 0x475E, 0x09 },
	{ 0x4760, 0x11 },
	{ 0x4762, 0x2F },
	{ 0x4766, 0xCC },
	{ 0x4776, 0xCB },
	{ 0x477E, 0x4A },
	{ 0x478E, 0x49 },
	{ 0x4794, 0x7C },
	{ 0x4796, 0x8F },
	{ 0x4798, 0xB3 },
	{ 0x4799, 0x00 },
	{ 0x479A, 0xCC },
	{ 0x479C, 0xC1 },
	{ 0x479E, 0xCB },
	{ 0x47A4, 0x7D },
	{ 0x47A6, 0x8E },
	{ 0x47A8, 0xB4 },
	{ 0x47A9, 0x00 },
	{ 0x47AA, 0xC0 },
	{ 0x47AC, 0xFA },
	{ 0x47AE, 0x0D },
	{ 0x47B0, 0x31 },
	{ 0x47B1, 0x01 },
	{ 0x47B2, 0x4A },
	{ 0x47B3, 0x01 },
	{ 0x47B4, 0x3F },
	{ 0x47B6, 0x49 },
	{ 0x47BC, 0xFB },
	{ 0x47BE, 0x0C },
	{ 0x47C0, 0x32 },
	{ 0x47C1, 0x01 },
	{ 0x47C2, 0x3E },
	{ 0x47C3, 0x01 },
	{ 0x4E3C, 0x07 },
};


static const struct imx678_reg imx678_tpg_en_regs[] = {
	//TPG config
	{ 0x3042, 0x00 }, //XSIZE_OVERLAP
	{ 0x30e0, 0x01 }, //TPG_EN_DUOUT
	{ 0x30e4, 0x13 }, //TPG_COLORWIDTH
};


static const struct imx678_reg mode_1920x1080_3dol_binning_20fps_regs[] = {
	{ 0x3002, 0x01 }, /* XMSTA */
	{ 0x3014, IMX678_INCLK_CODE }, /* INCK_SEL[3:0] */
	{ 0x3015, 0x03 },
	{ 0x3018, 0x04 }, /* WINMODE[4:0] */  // MANUAL (crop 3856x2180 to 3840x2160)
	{ 0x301A, 0x02 },
	{ 0x301B, 0x01 },
	{ 0x301C, 0x01 },
	{ 0x3022, 0x00 },
	{ 0x302C, 0x26 },
	{ 0x302D, 0x02 },
	{ 0x303E, 0x00 }, /* PIX_HWIDTH[12:0] */  // MANUAL (crop 3856 to 3840)
	{ 0x303F, 0x0F }, // MANUAL (crop 3856 to 3840)
	{ 0x3040, 0x03 }, /* LANEMODE[2:0] */
	{ 0x3046, 0x70 }, /* PIX_VWIDTH[11:0] */  // MANUAL (crop 2180 to 2160)
	{ 0x3047, 0x08 }, // MANUAL (crop 2180 to 2160)
	{ 0x3050, 0xB6 },
	{ 0x3051, 0x0D },
	{ 0x3054, 0x07 },
	{ 0x3058, 0x98 },
	{ 0x3059, 0x00 },
	{ 0x3060, 0x91 },
	{ 0x3061, 0x00 },
	{ 0x3064, 0xAA },
	{ 0x3065, 0x00 },
	{ 0x30A6, 0x00 },
	{ 0x3400, 0x00 }, /* GAIN_PGC_FIDMD - 0: set individual exposure gains*/
	{ 0x3460, 0x22 },
	{ 0x355A, 0x64 },
	{ 0x3A02, 0x7A },
	{ 0x3A10, 0xEC },
	{ 0x3A12, 0x71 },
	{ 0x3A14, 0xDE },
	{ 0x3A20, 0x2B },
	{ 0x3A24, 0x22 },
	{ 0x3A25, 0x25 },
	{ 0x3A26, 0x2A },
	{ 0x3A27, 0x2C },
	{ 0x3A28, 0x39 },
	{ 0x3A29, 0x38 },
	{ 0x3A30, 0x04 },
	{ 0x3A31, 0x04 },
	{ 0x3A32, 0x03 },
	{ 0x3A33, 0x03 },
	{ 0x3A34, 0x09 },
	{ 0x3A35, 0x06 },
	{ 0x3A38, 0xCD },
	{ 0x3A3A, 0x4C },
	{ 0x3A3C, 0xB9 },
	{ 0x3A3E, 0x30 },
	{ 0x3A40, 0x2C },
	{ 0x3A42, 0x39 },
	{ 0x3A4E, 0x00 },
	{ 0x3A52, 0x00 },
	{ 0x3A56, 0x00 },
	{ 0x3A5A, 0x00 },
	{ 0x3A5E, 0x00 },
	{ 0x3A62, 0x00 },
	{ 0x3A6E, 0xA0 },
	{ 0x3A70, 0x50 },
	{ 0x3A8C, 0x04 },
	{ 0x3A8D, 0x03 },
	{ 0x3A8E, 0x09 },
	{ 0x3A90, 0x38 },
	{ 0x3A91, 0x42 },
	{ 0x3A92, 0x3C },
	{ 0x3B0E, 0xF3 },
	{ 0x3B12, 0xE5 },
	{ 0x3B27, 0xC0 },
	{ 0x3B2E, 0xEF },
	{ 0x3B30, 0x6A },
	{ 0x3B32, 0xF6 },
	{ 0x3B36, 0xE1 },
	{ 0x3B3A, 0xE8 },
	{ 0x3B5A, 0x17 },
	{ 0x3B5E, 0xEF },
	{ 0x3B60, 0x6A },
	{ 0x3B62, 0xF6 },
	{ 0x3B66, 0xE1 },
	{ 0x3B6A, 0xE8 },
	{ 0x3B88, 0xEC },
	{ 0x3B8A, 0xED },
	{ 0x3B94, 0x71 },
	{ 0x3B96, 0x72 },
	{ 0x3B98, 0xDE },
	{ 0x3B9A, 0xDF },
	{ 0x3C0F, 0x06 },
	{ 0x3C10, 0x06 },
	{ 0x3C11, 0x06 },
	{ 0x3C12, 0x06 },
	{ 0x3C13, 0x06 },
	{ 0x3C18, 0x20 },
	{ 0x3C3A, 0x7A },
	{ 0x3C40, 0xF4 },
	{ 0x3C48, 0xE6 },
	{ 0x3C54, 0xCE },
	{ 0x3C56, 0xD0 },
	{ 0x3C6C, 0x53 },
	{ 0x3C6E, 0x55 },
	{ 0x3C70, 0xC0 },
	{ 0x3C72, 0xC2 },
	{ 0x3C7E, 0xCE },
	{ 0x3C8C, 0xCF },
	{ 0x3C8E, 0xEB },
	{ 0x3C98, 0x54 },
	{ 0x3C9A, 0x70 },
	{ 0x3C9C, 0xC1 },
	{ 0x3C9E, 0xDD },
	{ 0x3CB0, 0x7A },
	{ 0x3CB2, 0xBA },
	{ 0x3CC8, 0xBC },
	{ 0x3CCA, 0x7C },
	{ 0x3CD4, 0xEA },
	{ 0x3CD5, 0x01 },
	{ 0x3CD6, 0x4A },
	{ 0x3CD8, 0x00 },
	{ 0x3CD9, 0x00 },
	{ 0x3CDA, 0xFF },
	{ 0x3CDB, 0x03 },
	{ 0x3CDC, 0x00 },
	{ 0x3CDD, 0x00 },
	{ 0x3CDE, 0xFF },
	{ 0x3CDF, 0x03 },
	{ 0x3CE4, 0x4C },
	{ 0x3CE6, 0xEC },
	{ 0x3CE7, 0x01 },
	{ 0x3CE8, 0xFF },
	{ 0x3CE9, 0x03 },
	{ 0x3CEA, 0x00 },
	{ 0x3CEB, 0x00 },
	{ 0x3CEC, 0xFF },
	{ 0x3CED, 0x03 },
	{ 0x3CEE, 0x00 },
	{ 0x3CEF, 0x00 },
	{ 0x3E28, 0x82 },
	{ 0x3E2A, 0x80 },
	{ 0x3E30, 0x85 },
	{ 0x3E32, 0x7D },
	{ 0x3E5C, 0xCE },
	{ 0x3E5E, 0xD3 },
	{ 0x3E70, 0x53 },
	{ 0x3E72, 0x58 },
	{ 0x3E74, 0xC0 },
	{ 0x3E76, 0xC5 },
	{ 0x3E78, 0xC0 },
	{ 0x3E79, 0x01 },
	{ 0x3E7A, 0xD4 },
	{ 0x3E7B, 0x01 },
	{ 0x3EB4, 0x0B },
	{ 0x3EB5, 0x02 },
	{ 0x3EB6, 0x4D },
	{ 0x3EEC, 0xF3 },
	{ 0x3EEE, 0xE7 },
	{ 0x3F01, 0x01 },
	{ 0x3F24, 0x10 },
	{ 0x3F28, 0x2D },
	{ 0x3F2A, 0x2D },
	{ 0x3F2C, 0x2D },
	{ 0x3F2E, 0x2D },
	{ 0x3F30, 0x23 },
	{ 0x3F38, 0x2D },
	{ 0x3F3A, 0x2D },
	{ 0x3F3C, 0x2D },
	{ 0x3F3E, 0x28 },
	{ 0x3F40, 0x1E },
	{ 0x3F48, 0x2D },
	{ 0x3F4A, 0x2D },
	{ 0x4004, 0xE4 },
	{ 0x4006, 0xFF },
	{ 0x4018, 0x69 },
	{ 0x401A, 0x84 },
	{ 0x401C, 0xD6 },
	{ 0x401E, 0xF1 },
	{ 0x4038, 0xDE },
	{ 0x403A, 0x00 },
	{ 0x403B, 0x01 },
	{ 0x404C, 0x63 },
	{ 0x404E, 0x85 },
	{ 0x4050, 0xD0 },
	{ 0x4052, 0xF2 },
	{ 0x4108, 0xDD },
	{ 0x410A, 0xF7 },
	{ 0x411C, 0x62 },
	{ 0x411E, 0x7C },
	{ 0x4120, 0xCF },
	{ 0x4122, 0xE9 },
	{ 0x4138, 0xE6 },
	{ 0x413A, 0xF1 },
	{ 0x414C, 0x6B },
	{ 0x414E, 0x76 },
	{ 0x4150, 0xD8 },
	{ 0x4152, 0xE3 },
	{ 0x417E, 0x03 },
	{ 0x417F, 0x01 },
	{ 0x4186, 0xE0 },
	{ 0x4190, 0xF3 },
	{ 0x4192, 0xF7 },
	{ 0x419C, 0x78 },
	{ 0x419E, 0x7C },
	{ 0x41A0, 0xE5 },
	{ 0x41A2, 0xE9 },
	{ 0x41C8, 0xE2 },
	{ 0x41CA, 0xFD },
	{ 0x41DC, 0x67 },
	{ 0x41DE, 0x82 },
	{ 0x41E0, 0xD4 },
	{ 0x41E2, 0xEF },
	{ 0x4200, 0xDE },
	{ 0x4202, 0xDA },
	{ 0x4218, 0x63 },
	{ 0x421A, 0x5F },
	{ 0x421C, 0xD0 },
	{ 0x421E, 0xCC },
	{ 0x425A, 0x82 },
	{ 0x425C, 0xEF },
	{ 0x4348, 0xFE },
	{ 0x4349, 0x06 },
	{ 0x4352, 0xCE },
	{ 0x4420, 0x0B },
	{ 0x4421, 0x02 },
	{ 0x4422, 0x4D },
	{ 0x4426, 0xF5 },
	{ 0x442A, 0xE7 },
	{ 0x4432, 0xF5 },
	{ 0x4436, 0xE7 },
	{ 0x4466, 0xB4 },
	{ 0x446E, 0x32 },
	{ 0x449F, 0x1C },
	{ 0x44A4, 0x2C },
	{ 0x44A6, 0x2C },
	{ 0x44A8, 0x2C },
	{ 0x44AA, 0x2C },
	{ 0x44B4, 0x2C },
	{ 0x44B6, 0x2C },
	{ 0x44B8, 0x2C },
	{ 0x44BA, 0x2C },
	{ 0x44C4, 0x2C },
	{ 0x44C6, 0x2C },
	{ 0x44C8, 0x2C },
	{ 0x4506, 0xF3 },
	{ 0x450E, 0xE5 },
	{ 0x4516, 0xF3 },
	{ 0x4522, 0xE5 },
	{ 0x4524, 0xF3 },
	{ 0x452C, 0xE5 },
	{ 0x453C, 0x22 },
	{ 0x453D, 0x1B },
	{ 0x453E, 0x1B },
	{ 0x453F, 0x15 },
	{ 0x4540, 0x15 },
	{ 0x4541, 0x15 },
	{ 0x4542, 0x15 },
	{ 0x4543, 0x15 },
	{ 0x4544, 0x15 },
	{ 0x4548, 0x00 },
	{ 0x4549, 0x01 },
	{ 0x454A, 0x01 },
	{ 0x454B, 0x06 },
	{ 0x454C, 0x06 },
	{ 0x454D, 0x06 },
	{ 0x454E, 0x06 },
	{ 0x454F, 0x06 },
	{ 0x4550, 0x06 },
	{ 0x4554, 0x55 },
	{ 0x4555, 0x02 },
	{ 0x4556, 0x42 },
	{ 0x4557, 0x05 },
	{ 0x4558, 0xFD },
	{ 0x4559, 0x05 },
	{ 0x455A, 0x94 },
	{ 0x455B, 0x06 },
	{ 0x455D, 0x06 },
	{ 0x455E, 0x49 },
	{ 0x455F, 0x07 },
	{ 0x4560, 0x7F },
	{ 0x4561, 0x07 },
	{ 0x4562, 0xA5 },
	{ 0x4564, 0x55 },
	{ 0x4565, 0x02 },
	{ 0x4566, 0x42 },
	{ 0x4567, 0x05 },
	{ 0x4568, 0xFD },
	{ 0x4569, 0x05 },
	{ 0x456A, 0x94 },
	{ 0x456B, 0x06 },
	{ 0x456D, 0x06 },
	{ 0x456E, 0x49 },
	{ 0x456F, 0x07 },
	{ 0x4572, 0xA5 },
	{ 0x460C, 0x7D },
	{ 0x460E, 0xB1 },
	{ 0x4614, 0xA8 },
	{ 0x4616, 0xB2 },
	{ 0x461C, 0x7E },
	{ 0x461E, 0xA7 },
	{ 0x4624, 0xA8 },
	{ 0x4626, 0xB2 },
	{ 0x462C, 0x7E },
	{ 0x462E, 0x8A },
	{ 0x4630, 0x94 },
	{ 0x4632, 0xA7 },
	{ 0x4634, 0xFB },
	{ 0x4636, 0x2F },
	{ 0x4638, 0x81 },
	{ 0x4639, 0x01 },
	{ 0x463A, 0xB5 },
	{ 0x463B, 0x01 },
	{ 0x463C, 0x26 },
	{ 0x463E, 0x30 },
	{ 0x4640, 0xAC },
	{ 0x4641, 0x01 },
	{ 0x4642, 0xB6 },
	{ 0x4643, 0x01 },
	{ 0x4644, 0xFC },
	{ 0x4646, 0x25 },
	{ 0x4648, 0x82 },
	{ 0x4649, 0x01 },
	{ 0x464A, 0xAB },
	{ 0x464B, 0x01 },
	{ 0x464C, 0x26 },
	{ 0x464E, 0x30 },
	{ 0x4654, 0xFC },
	{ 0x4656, 0x08 },
	{ 0x4658, 0x12 },
	{ 0x465A, 0x25 },
	{ 0x4662, 0xFC },
	{ 0x46A2, 0xFB },
	{ 0x46D6, 0xF3 },
	{ 0x46E6, 0x00 },
	{ 0x46E8, 0xFF },
	{ 0x46E9, 0x03 },
	{ 0x46EC, 0x7A },
	{ 0x46EE, 0xE5 },
	{ 0x46F4, 0xEE },
	{ 0x46F6, 0xF2 },
	{ 0x470C, 0xFF },
	{ 0x470D, 0x03 },
	{ 0x470E, 0x00 },
	{ 0x4714, 0xE0 },
	{ 0x4716, 0xE4 },
	{ 0x471E, 0xED },
	{ 0x472E, 0x00 },
	{ 0x4730, 0xFF },
	{ 0x4731, 0x03 },
	{ 0x4734, 0x7B },
	{ 0x4736, 0xDF },
	{ 0x4754, 0x7D },
	{ 0x4756, 0x8B },
	{ 0x4758, 0x93 },
	{ 0x475A, 0xB1 },
	{ 0x475C, 0xFB },
	{ 0x475E, 0x09 },
	{ 0x4760, 0x11 },
	{ 0x4762, 0x2F },
	{ 0x4766, 0xCC },
	{ 0x4776, 0xCB },
	{ 0x477E, 0x4A },
	{ 0x478E, 0x49 },
	{ 0x4794, 0x7C },
	{ 0x4796, 0x8F },
	{ 0x4798, 0xB3 },
	{ 0x4799, 0x00 },
	{ 0x479A, 0xCC },
	{ 0x479C, 0xC1 },
	{ 0x479E, 0xCB },
	{ 0x47A4, 0x7D },
	{ 0x47A6, 0x8E },
	{ 0x47A8, 0xB4 },
	{ 0x47A9, 0x00 },
	{ 0x47AA, 0xC0 },
	{ 0x47AC, 0xFA },
	{ 0x47AE, 0x0D },
	{ 0x47B0, 0x31 },
	{ 0x47B1, 0x01 },
	{ 0x47B2, 0x4A },
	{ 0x47B3, 0x01 },
	{ 0x47B4, 0x3F },
	{ 0x47B6, 0x49 },
	{ 0x47BC, 0xFB },
	{ 0x47BE, 0x0C },
	{ 0x47C0, 0x32 },
	{ 0x47C1, 0x01 },
	{ 0x47C2, 0x3E },
	{ 0x47C3, 0x01 },
	{ 0x4E3C, 0x07 }
};

/* Supported sensor mode configurations */
static const struct imx678_mode supported_sdr_modes[] = {
	{
	.width = 3840,
	.height = 2160,
	.hblank = 550,
	.vblank = 2340,
	.vblank_min = 90,
	.vblank_max = IMX678_MAX_VBLANK_4K,
	.rhs1 = 0x0,
	.rhs2 = 0x0,
	.link_freq_idx = 2,
	.pclk = link_freq[2],
	.code = MEDIA_BUS_FMT_SRGGB12_1X12,
	.dol = 1,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
		.regs = mode_3840x2160_regs,
	},
	.frame_interval = {
		.denominator = 30,
		.numerator = 1,
	},
	},
	{
	.width = 3840,
	.height = 2160,
	.hblank = 550,
	.vblank = 90,
	.vblank_min = 90,
	.vblank_max = IMX678_MAX_VBLANK_4K,
	.rhs1 = 0x0,
	.rhs2 = 0x0,
	.link_freq_idx = 2,
	.pclk = link_freq[2],
	.code = MEDIA_BUS_FMT_SRGGB12_1X12,
	.dol = 1,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
		.regs = mode_3840x2160_regs,
	},
	.frame_interval = {
		.denominator = 60,
		.numerator = 1,
	},
	},
	{
	.width = 1920,
	.height = 1080,
	.hblank = 550,
	.vblank = 3420,
	.vblank_min = 90,
	.vblank_max = 132840,
	.rhs1 = 0x0,
	.rhs2 = 0x0,
	.link_freq_idx = 2,
	.pclk = link_freq[2],
	.code = MEDIA_BUS_FMT_SRGGB12_1X12,
	.dol = 1,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_1920x1080_sdr_binning_regs),
		.regs = mode_1920x1080_sdr_binning_regs,
	},
	.frame_interval = {
		.denominator = 30,
		.numerator = 1,
	},
	},
	{
	.width = 3840,
	.height = 2160,
	.hblank = 550,
	.vblank = 1636,
	.vblank_min = 90,
	.vblank_max = IMX678_MAX_VBLANK_4K,
	.rhs1 = 0x0,
	.rhs2 = 0x0,
	.link_freq_idx = 2,
	.pclk = link_freq[2],
	.code = MEDIA_BUS_FMT_SRGGB12_1X12,
	.dol = 1,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
		.regs = mode_3840x2160_regs,
	},
	.frame_interval = {
		.denominator = 24,
		.numerator = 1,
	},
	},
	{
	.width = 3840,
	.height = 2160,
	.hblank = 550,
	.vblank = 6840,
	.vblank_min = 90,
	.vblank_max = IMX678_MAX_VBLANK_4K,
	.rhs1 = 0x0,
	.rhs2 = 0x0,
	.link_freq_idx = 2,
	.pclk = link_freq[2],
	.code = MEDIA_BUS_FMT_SRGGB12_1X12,
	.dol = 1,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
		.regs = mode_3840x2160_regs,
	},
	.frame_interval = {
		.denominator = 15,
		.numerator = 1,
	},
	},
};

static const struct imx678_mode supported_hdr_modes[] = {
	{
    .width = 3840,
    .height = 2160,
    .hblank = 1320,
    .vblank = 90,
    .vblank_min = 90,
    .vblank_max = 132840,
    .rhs1 = 0x11b,
    .rhs2 = 0x0,
    .link_freq_idx = 2,
    .pclk = link_freq[2],
    .code = MEDIA_BUS_FMT_SRGGB12_2X12,
    .dol = 2,
    .reg_list = {
        .num_of_regs = ARRAY_SIZE(mode_4k_2dol_all_pixel),
        .regs = mode_4k_2dol_all_pixel,
    },
    .frame_interval = {
        .denominator = 30,
        .numerator = 1,
    },
    },
    {
    .width = 3840,
    .height = 2160,
    .hblank = 1320,
    .vblank = 90,
    .vblank_min = 90,
    .vblank_max = 132840,
	.rhs1 = 0x40,
	.rhs2 = 0x53,
    .link_freq_idx = 0,
    .pclk = link_freq[0],
    .code = MEDIA_BUS_FMT_SRGGB12_3X12,
    .dol = 3,
    .reg_list = {
        .num_of_regs = ARRAY_SIZE(mode_4k_3dol_all_pixel),
        .regs = mode_4k_3dol_all_pixel,
    },
    .frame_interval = {
        .denominator = 8,
        .numerator = 1,
    },
    },
	{
    .width = 3840,
    .height = 2160,
    .hblank = 550, /*change in registers*/
    .vblank = 90,  /*  */
    .vblank_min = 90,
    .vblank_max = 132840,
	.rhs1 = 0x1F3, /* change in registers */
	.rhs2 = 0x230, /* change in registers */
    .link_freq_idx = 2,
    .pclk = link_freq[2],
    .code = MEDIA_BUS_FMT_SRGGB12_3X12,
    .dol = 3,
    .reg_list = {
        .num_of_regs = ARRAY_SIZE(mode_4k_3dol_20fps_all_pixel),
        .regs = mode_4k_3dol_20fps_all_pixel,
    },
    .frame_interval = {
        .denominator = 20,
        .numerator = 1,
    },
    },

	{
	.width = 1920,
	.height = 1080,
	.hblank = 550,
	.vblank = 1170,
	.vblank_min = 90,
	.vblank_max = 132840,
	.rhs1 = 0x91,
	.rhs2 = 0xAA,
	.link_freq_idx = 1,
	.pclk = link_freq[1],
	.code = MEDIA_BUS_FMT_SRGGB12_3X12,
	.dol = 3,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_1920x1080_3dol_binning_20fps_regs),
		.regs = mode_1920x1080_3dol_binning_20fps_regs,
	},
	.frame_interval = {
		.denominator = 20,
		.numerator = 1,
	},
	},
};

struct v4l2_ctrl_config imx678_custom_ctrls[] = {
	{
		.ops = &imx678_ctrl_ops,
		.id = IMX678_CID_ANALOGUE_GAIN_SHORT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.name = "analogue_gain_short",
		.step = IMX678_AGAIN_STEP,
		.min = IMX678_AGAIN_MIN,
		.max = IMX678_AGAIN_MAX,
		.def = IMX678_AGAIN_DEFAULT,
	},
	{
		.ops = &imx678_ctrl_ops,
		.id = IMX678_CID_ANALOGUE_GAIN_VERY_SHORT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.name = "analogue_gain_very_short",
		.step = IMX678_AGAIN_STEP,
		.min = IMX678_AGAIN_MIN,
		.max = IMX678_AGAIN_MAX,
		.def = IMX678_AGAIN_DEFAULT,
	},
	{
		.ops = &imx678_ctrl_ops,
		.id = IMX678_CID_EXPOSURE_SHORT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.name = "exposure_short",
		.step = IMX678_EXPOSURE_SHORT_STEP,
	},
	{
		.ops = &imx678_ctrl_ops,
		.id = IMX678_CID_EXPOSURE_VERY_SHORT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.name = "exposure_very_short",
		.step = IMX678_EXPOSURE_VERY_SHORT_STEP,
	},
	{
		.ops = &imx678_ctrl_ops,
		.id = IMX678_CID_HCG,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.name = "hcg",
		.step = IMX678_HCG_STEP,
		.min = IMX678_HCG_MIN,
		.max = IMX678_HCG_MAX,
		.def = IMX678_HCG_DEFAULT,
	},
	{
		.ops = &imx678_ctrl_ops,
		.id = IMX678_CID_CUSTOM_RHS1,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.name = "custom_rhs1",
		.step = IMX678_INTEGER_STEP,
		.min = IMX678_CUSTOM_RHS1_MIN,
		.max = IMX678_CUSTOM_RHS1_MAX,
		.def = IMX678_CUSTOM_RHS1_DEFAULT,
	},
	{
		.ops = &imx678_get_ctrl_ops,
		.id = IMX678_CID_RHS1,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "readout_timing_short",
		.step = IMX678_INTEGER_STEP,
	},
	{
		.ops = &imx678_get_ctrl_ops,
		.id = IMX678_CID_RHS2,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "readout_timing_very_short",
		.step = IMX678_INTEGER_STEP,
	},
	{
		.ops = &imx678_get_ctrl_ops,
		.id = IMX678_CID_SHR0,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "shutter_timing_long",
		.step = IMX678_INTEGER_STEP,
	},
	{
		.ops = &imx678_get_ctrl_ops,
		.id = IMX678_CID_SHR1,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "shutter_timing_short",
		.step = IMX678_EXPOSURE_SHORT_STEP,
	},
	{
		.ops = &imx678_get_ctrl_ops,
		.id = IMX678_CID_SHR2,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "shutter_timing_very_short",
		.step = IMX678_EXPOSURE_VERY_SHORT_STEP,
	},
	{
		.ops = &imx678_get_ctrl_ops,
		.id = IMX678_CID_VMAX,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "vertical_span",
		.step = IMX678_INTEGER_STEP,
	},
	{
		.ops = &imx678_get_ctrl_ops,
		.id = IMX678_CID_HMAX,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "horizontal_span",
		.step = IMX678_INTEGER_STEP,
	},
};

static void convert_v4l2_subdev_code_to_sensor_code(struct imx678* imx678, struct v4l2_mbus_framefmt* fmt) {
	if (imx678->hdr_enabled) {
		/* hdr mode: all fmt->code should be 2xnum_bits or 3xnum_bits, but not 1xnum_bits*/
		if (fmt->code == MEDIA_BUS_FMT_SRGGB12_1X12) {
            // TODO: Make this properly configurable (probably just by configurable mode index)
			fmt->code = MEDIA_BUS_FMT_SRGGB12_2X12;
		}
	} else {
		/* sdr mode: all fmt->code should be 1xnum_bits */
		return;
	}	
}

static struct v4l2_ctrl_config *get_custom_ctrl_by_id(u32 id) {
	int i;

	for (i = 0; i < ARRAY_SIZE(imx678_custom_ctrls); i++) {
		if (imx678_custom_ctrls[i].id == id)
			return &imx678_custom_ctrls[i];
	}

	pr_err("Invalid control id: %d\n", id);
	return NULL;
}

static void imx678_setup_custom_ctrl(struct imx678 *imx678, struct v4l2_ctrl **ctrl, u32 id) {
	struct v4l2_ctrl_config *config = get_custom_ctrl_by_id(id);

	if (!config) {
		dev_err(imx678->dev, "Setup invalid custom control id: %d\n", id);
		return;
	}

	*ctrl = v4l2_ctrl_new_custom(&imx678->ctrl_handler, config, NULL);

	if (!*ctrl)
		dev_err(imx678->dev, "Failed to initialize custom control %s, handler error: %d\n",
			config->name, imx678->ctrl_handler.error);
}

static void imx678_setup_custom_ctrl_limits(
	struct imx678 *imx678, struct v4l2_ctrl **ctrl, u32 id,
	s64 min, s64 max, s64 def)
{
	struct v4l2_ctrl_config *config = get_custom_ctrl_by_id(id);

	if (!config) {
		dev_err(imx678->dev, "Setup invalid custom control id: %d\n", id);
		return;
	}

	config->min = min;
	config->max = max;
	config->def = def;
	
	*ctrl = v4l2_ctrl_new_custom(&imx678->ctrl_handler, config, NULL);

	if (!*ctrl)
		dev_err(imx678->dev, "Failed to initialize custom control %s, handler error: %d\n",
			config->name, imx678->ctrl_handler.error);
}

/**
 * to_imx678() - imv678 V4L2 sub-device to imx678 device.
 * @subdev: pointer to imx678 V4L2 sub-device
 *
 * Return: pointer to imx678 device
 */
static inline struct imx678 *to_imx678(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct imx678, sd);
}

/**
 * imx678_read_reg() - Read registers.
 * @imx678: pointer to imx678 device
 * @reg: register address
 * @len: length of bytes to read. Max supported bytes is 4
 * @val: pointer to register value to be filled.
 *
 * Big endian register addresses with little endian values.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_read_reg(struct imx678 *imx678, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct i2c_msg msgs[2] = { 0 };
	u8 addr_buf[2] = { 0 };
	u8 data_buf[4] = { 0 };
	int ret;

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_le32(data_buf);

	return 0;
}

/**
 * imx678_write_reg() - Write register
 * @imx678: pointer to imx678 device
 * @reg: register address
 * @len: length of bytes. Max supported bytes is 4
 * @val: register value
 *
 * Big endian register addresses with little endian values.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_write_reg(struct imx678 *imx678, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	u8 buf[6] = { 0 };

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_le32(val, buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/**
 * imx678_write_regs() - Write a list of registers
 * @imx678: pointer to imx678 device
 * @regs: list of registers to be written
 * @len: length of registers array
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_write_regs(struct imx678 *imx678,
			     const struct imx678_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx678_write_reg(imx678, regs[i].address, 1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

typedef struct ExposureLimits_t {

    u32 lpfr;
	u32 min_lpfr;
	u32 max_lpfr;
	u32 lef_reg;
    u32 shr0_min;
    u32 shr0_max;
    u32 exp_lef_min;
    u32 exp_lef_max;
    u32 exp_lef_default;

    u32 sef1_reg;
    u32 shr1_min;
    u32 shr1_max;
    u32 exp_sef1_min;
    u32 exp_sef1_max;
    u32 exp_sef1_default;

    u32 sef2_reg;
    u32 shr2_min;
    u32 shr2_max;
    u32 exp_sef2_min;
    u32 exp_sef2_max;
    u32 exp_sef2_default;
} * ExposureLimits;

void calculate_exposure_limits(struct imx678* imx678, ExposureLimits limits) {
	const int rhs1 = imx678->cur_mode->rhs1 > 0 ? imx678->cur_mode->rhs1 : IMX678_DEFAULT_RHS1;
	const int rhs2 = imx678->cur_mode->rhs2 > 0 ? imx678->cur_mode->rhs2 : IMX678_DEFAULT_RHS2;
	u32 shr0, shr1, shr2;

	limits->lpfr = imx678->cur_mode->dol * (imx678->vblank + imx678->cur_mode->height);
	limits->min_lpfr = imx678->cur_mode->dol * (imx678->cur_mode->vblank_min + imx678->cur_mode->height);
	limits->max_lpfr = imx678->cur_mode->dol * (imx678->cur_mode->vblank_max + imx678->cur_mode->height);

	limits->lef_reg = IMX678_REG_SHUTTER;
	limits->shr0_min = imx678->hdr_enabled ? imx678->cur_mode->rhs2 + IMX678_SHR0_RHS2_GAP : IMX678_SHR0_FSC_GAP;
	limits->shr0_max = NON_NEGATIVE(limits->max_lpfr - IMX678_SHR0_FSC_GAP);
	limits->exp_lef_min = IMX678_SHR0_FSC_GAP;
	limits->exp_lef_max = NON_NEGATIVE(limits->max_lpfr - limits->shr0_min);
	shr0 = imx678->vblank;
	limits->exp_lef_default = MAX(limits->exp_lef_min, NON_NEGATIVE((int)limits->lpfr - (int)shr0));

	limits->sef1_reg = IMX678_REG_SHUTTER_SHORT;
	limits->shr1_min = IMX678_SHR1_MIN_GAP;
	limits->shr1_max = NON_NEGATIVE(rhs1 - IMX678_SHR1_RHS1_GAP);
	limits->exp_sef1_min = NON_NEGATIVE(rhs1 - limits->shr1_max);
	limits->exp_sef1_max = NON_NEGATIVE(rhs1 - limits->shr1_min);
	shr1 = MAX(limits->shr1_min, _get_mode_reg_val_by_address(&imx678->cur_mode->reg_list, limits->sef1_reg, 3));
	limits->exp_sef1_default = MAX(limits->exp_sef1_min, NON_NEGATIVE((int)rhs1 - (int)shr1));

	limits->sef2_reg = IMX678_REG_SHUTTER_VERY_SHORT;
	limits->shr2_min = rhs1 + IMX678_SHR2_RHS1_GAP;
	limits->shr2_max = NON_NEGATIVE(rhs2 - IMX678_SHR2_RHS2_GAP);
	limits->exp_sef2_min = NON_NEGATIVE(rhs2 - limits->shr2_max);
	limits->exp_sef2_max = NON_NEGATIVE(rhs2 - limits->shr2_min);
	shr2 = MAX(limits->shr2_min, _get_mode_reg_val_by_address(&imx678->cur_mode->reg_list, limits->sef2_reg, 3));
	limits->exp_sef2_default = MAX(limits->exp_sef2_min, NON_NEGATIVE((int)rhs2 - (int)shr2));
}

/**
 * imx678_update_controls() - Update control ranges based on streaming mode
 * @imx678: pointer to imx678 device
 * @mode: pointer to imx678_mode sensor mode
 *
 * Return: 0 if successful, error code otherwise.
 */
/*
static int imx678_update_controls(struct imx678* imx678,
	const struct imx678_mode* mode)
{
	int ret;

	ret = __v4l2_ctrl_s_ctrl(imx678->link_freq_ctrl, mode->link_freq_idx);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx678->hblank_ctrl, mode->hblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_modify_range(imx678->vblank_ctrl, mode->vblank_min,
		mode->vblank_max, 1, mode->vblank);
}
*/

static int imx678_set_ctrl_range_and_value(struct imx678 *imx678,
		struct v4l2_ctrl *ctrl, u32 min, u32 max, u32 step, u32 def)
{
	int ret;

	ret = __v4l2_ctrl_modify_range(ctrl, min, max, step, def);
	if (ret) {
		dev_err(imx678->dev, "Failed to modify control %s range. "
			"ret=%d. min=%d, max=%d, default=%d",
			ctrl->name, ret, min, max, def);
		return ret;
	}

	ret = __v4l2_ctrl_s_ctrl(ctrl, def);
	if (ret) {
		dev_err(imx678->dev, "Failed to set control %s to default %d. ret=%d",
			ctrl->name, def, ret);
		return ret;
	}

	return 0;
}

/**
 * imx678_update_exp_vblank_controls() - Update control ranges based on streaming mode
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_update_exp_vblank_controls(struct imx678* imx678)
{
	struct ExposureLimits_t limits;
	const struct imx678_mode *mode = imx678->cur_mode;
	int ret;

	memset(&limits, 0, sizeof(struct ExposureLimits_t));
	calculate_exposure_limits(imx678, &limits);

	ret = imx678_set_ctrl_range_and_value(imx678, imx678->lef.exp_ctrl, limits.exp_lef_min,
		limits.exp_lef_max, IMX678_EXPOSURE_STEP, limits.exp_lef_default);
	if (ret) {
		dev_err(imx678->dev, "Failed to update LEF exposure range and value\n");
		return ret;
	}

	if (imx678->cur_mode->dol >= 2) {
		ret = imx678_set_ctrl_range_and_value(imx678, imx678->sef1.exp_ctrl, limits.exp_sef1_min,
			limits.exp_sef1_max, IMX678_EXPOSURE_SHORT_STEP, limits.exp_sef1_default);
		if (ret) {
			dev_err(imx678->dev, "Failed to update SEF1 exposure range and value\n");
			return ret;
		}
	}

	if (imx678->cur_mode->dol >= 3) {
		ret = imx678_set_ctrl_range_and_value(imx678, imx678->sef2.exp_ctrl, limits.exp_sef2_min,
			limits.exp_sef2_max, IMX678_EXPOSURE_VERY_SHORT_STEP, limits.exp_sef2_default);
		if (ret) {
			dev_err(imx678->dev, "Failed to update SEF2 exposure range and value\n");
			return ret;
		}
	}

	ret = imx678_set_ctrl_range_and_value(imx678, imx678->vblank_ctrl, mode->vblank_min,
		mode->vblank_max, 1, imx678->vblank);
	if (ret) {
		dev_err(imx678->dev, "Failed to update vblank range and value\n");
		return ret;
	}

	return 0;
}

static int imx678_set_hcg_mode(struct imx678 *imx678, u32 hcg)
{
	int ret;
	ret = imx678_write_reg(imx678, IMX678_REG_HCG, 1, hcg);
	if (ret) {
        dev_err(imx678->dev, "Failed to write HCG register: %d\n", ret);
        return ret;
    }

    dev_dbg(imx678->dev, "HCG mode set to %s\n", hcg ? "enabled" : "disabled");
    
	return 0;
}
/**
 * imx678_update_exp_gain() - Set updated exposure and gain
 * @imx678: pointer to imx678 device
 * @exposure_time: updated exposure time value
 * @gain: updated analog gain value
 * @exposure_type: exposure type (0: LEF, 1: SEF1, 2: SEF2)
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_update_exp_gain(struct imx678 *imx678, u32 exposure_time, u32 gain, enum imx678_exposure_type exposure_type)
{
	u32 lpfr, shutter, desired_fps_numerator, desired_fps_denominator;
	int ret;

	switch (exposure_type) {
		case (LEF): {
			// Minimum number of lines until starting exposure
			u32 shr0_min_gap = imx678->hdr_enabled ? imx678->cur_mode->rhs2 + IMX678_SHR0_RHS2_GAP : IMX678_SHR0_FSC_GAP;
			shr0_min_gap = MAX(shr0_min_gap, imx678->cur_mode->vblank_min);
			
			// The desired fps is blocked by cur_mode->frame_interval (for example 30/1 for 30fps)
			// Note that this calculation does not take into account HDR - so HDR is broken by this v4l control
			desired_fps_numerator = CLOCK_FREQ_HZ / imx678->cur_mode->hblank;
			desired_fps_denominator = exposure_time + shr0_min_gap;
			
			// Note that frame_interval is in opposite units from fps, so 30fps will be represented by 1/30
			// This checks if: desired_fps_numerator / desired_fps_denominator > imx678->cur_mode->frame_interval.denominator / imx678->cur_mode->frame_interval.numerator
			// But accounts for int rounding
			if (desired_fps_numerator * imx678->cur_mode->frame_interval.numerator > imx678->cur_mode->frame_interval.denominator * desired_fps_denominator) {
				desired_fps_numerator = imx678->cur_mode->frame_interval.denominator;
				desired_fps_denominator = imx678->cur_mode->frame_interval.numerator;
			}

			if (!imx678->hdr_enabled) {
				// Calculate number of lines per frame, according to the desired fps
				// lines-per-frame should allow at least shr0_min_gap gap for vblank.
				lpfr = ((CLOCK_FREQ_HZ / desired_fps_numerator) * desired_fps_denominator) / imx678->cur_mode->hblank;
				lpfr = MAX(lpfr, imx678->cur_mode->height + shr0_min_gap);

				imx678->vblank = lpfr - exposure_time;
				__v4l2_ctrl_s_ctrl(imx678->vblank_ctrl, imx678->vblank);
			} else {
				// In HDR mode, we need to set the LPFR register to the maximum value
				lpfr = imx678->vblank + imx678->cur_mode->height;
			}

			shutter = NON_NEGATIVE(imx678->cur_mode->dol * (int)lpfr - (int)exposure_time);
			imx678->lef.exp_ctrl->val = exposure_time;
			break;
		}
		case (SEF1): {
			shutter = NON_NEGATIVE((int)imx678->cur_mode->rhs1 - (int)exposure_time);
			imx678->sef1.exp_ctrl->val = exposure_time;
			break;
		}
		case (SEF2): {
			shutter = NON_NEGATIVE((int)imx678->cur_mode->rhs2 - (int)exposure_time);
			imx678->sef2.exp_ctrl->val = exposure_time;
			break;
		}
	}

	dev_dbg(imx678->dev, "Set long exposure time %u analog gain %u sh%d %u lpfr %u",
		 exposure_time, gain, exposure_type, shutter, lpfr);

	ret = imx678_write_reg(imx678, IMX678_REG_HOLD, 1, 1);
	if (ret)
		return ret;

	if (exposure_type == LEF) {
		ret = imx678_write_reg(imx678, IMX678_REG_LPFR, 3, lpfr);
		if (ret)
			goto error_release_group_hold;
	}

	ret = imx678_write_reg(imx678, imx678_reg_shutter[exposure_type], 3, shutter);
	if (ret)
		goto error_release_group_hold;

	ret = imx678_write_reg(imx678, imx678_reg_again[exposure_type], 2, gain);

error_release_group_hold:
	imx678_write_reg(imx678, IMX678_REG_HOLD, 1, 0);

	return ret;
}

/*
 * imx678_set_test_pattern - Function called when setting test pattern
 * @priv: Pointer to device structure
 * @val: Variable for test pattern
 *
 * Set to different test patterns based on input value.
 *
 * Return: 0 on success
*/
static int imx678_set_test_pattern(struct imx678 *imx678, int val)
{
	int ret = 0;

	if (TEST_PATTERN_DISABLED == val)
		ret = imx678_write_reg(imx678, IMX678_TPG_EN_DUOUT, 1, val);
	else {
		ret = imx678_write_reg(imx678, IMX678_TPG_PATSEL_DUOUT, 1,
				       val - 1);
		if (!ret) {
			ret = imx678_write_regs(imx678, imx678_tpg_en_regs,
						ARRAY_SIZE(imx678_tpg_en_regs));
		}
	}
	return ret;
}

static int search_mode(const struct imx678_mode *mode, bool *o_hdr)
{
	// First search in HDR modes, then SDR modes
	if (mode >= supported_hdr_modes && mode < supported_hdr_modes + ARRAY_SIZE(supported_hdr_modes)) {
		if (o_hdr) *o_hdr = true;
		return mode - supported_hdr_modes;
	} else if (mode >= supported_sdr_modes && mode < supported_sdr_modes + ARRAY_SIZE(supported_sdr_modes)) {
		if (o_hdr) *o_hdr = false;
		return mode - supported_sdr_modes;
	}

	pr_err("Error. selected mode was not found!\n");
	return -1;
}

static void imx678_set_mode_string(struct imx678 *imx678)
{
	int fps;

	fps = imx678->cur_mode->frame_interval.denominator / 
		imx678->cur_mode->frame_interval.numerator;

	switch (imx678->cur_mode->dol) {
    case 1:
        if (imx678->hdr_enabled) {
            dev_err(imx678->dev, "Invalid HDR mode with dol=%d\n", imx678->cur_mode->dol);
            return;
        }
        snprintf(imx678->mode_string, SENSOR_MODE_STRING_LENGTH, "SDR #%d %dfps", imx678->mode_idx, fps);
        break;
	case 2:
	case 3:
        if (!imx678->hdr_enabled) {
            dev_err(imx678->dev, "Invalid SDR mode with dol=%d\n", imx678->cur_mode->dol);
            return;
        }
		snprintf(imx678->mode_string, SENSOR_MODE_STRING_LENGTH, "HDR #%d, %dDOL %dfps", 
			imx678->mode_idx, imx678->cur_mode->dol, fps);
        break;
	default:
		dev_err(imx678->dev, "Invalid mode with dol=%d\n", imx678->cur_mode->dol);
        break;
	}
}

static void imx678_set_mode(struct imx678 *imx678, const struct imx678_mode *mode)
{
	int ret;
	imx678->cur_mode = mode;
	imx678->vblank = mode->vblank;

    /* if the mode is from the modes list (not custom mode), set the mode index */
    if (imx678->cur_mode != &imx678->custom_mode) {
        imx678->mode_idx = search_mode(mode, NULL);
    }

    imx678_set_mode_string(imx678);

	/* set the link freq index and the pixel rate controls */
	if (imx678->link_freq_ctrl) {
		ret = __v4l2_ctrl_s_ctrl(imx678->link_freq_ctrl, mode->link_freq_idx);
		if (ret)
			dev_err(imx678->dev, "Failed to set link freq index to %d.", mode->link_freq_idx);
	}
	if (imx678->pclk_ctrl) {
		ret = __v4l2_ctrl_s_ctrl_int64(imx678->pclk_ctrl, mode->pclk);
		if (ret)
			dev_err(imx678->dev, "Failed to set pixel rate to %lld.", mode->pclk);
	}

	if (imx678->hdr_enabled) {
		if (mode->dol <= 1)
			dev_err(imx678->dev, "Set to invalid HDR mode with DOL %d", mode->dol);
	} else {
		if (mode->dol > 1)
			dev_err(imx678->dev, "Set to invalid SDR mode with DOL %d", mode->dol);
	}
}

static void imx678_set_exp_activity(struct imx678 *imx678)
{
	int dol = imx678->cur_mode->dol;
	bool sef1, sef2;

	sef1 = dol >= 2;
	sef2 = dol >= 3;

	v4l2_ctrl_activate(imx678->sef1.again_ctrl, sef1);
	v4l2_ctrl_activate(imx678->sef1.exp_ctrl, sef1);
	v4l2_ctrl_activate(imx678->rhs1_ctrl, sef1);
	v4l2_ctrl_activate(imx678->shr1_ctrl, sef1);

	v4l2_ctrl_activate(imx678->sef2.again_ctrl, sef2);
	v4l2_ctrl_activate(imx678->sef2.exp_ctrl, sef2);
	v4l2_ctrl_activate(imx678->rhs2_ctrl, sef2);
	v4l2_ctrl_activate(imx678->shr2_ctrl, sef2);
}

static int imx678_set_hdr_mode(struct imx678 *imx678, bool enable)
{
	const struct imx678_mode *prev_mode = NULL;
	int ret, revert_ret;

	ret = 0;
	if (imx678->hdr_enabled != enable) {
		imx678->hdr_enabled = enable;
		prev_mode = imx678->cur_mode;

		imx678_set_mode(imx678, imx678->hdr_enabled ? 
								&supported_hdr_modes[DEFAULT_MODE_IDX] :
								&supported_sdr_modes[DEFAULT_MODE_IDX]);

		ret = imx678_update_exp_vblank_controls(imx678);
		if (ret) {
			dev_warn(imx678->dev, "Failed to update exp controls, trying to revert to previous mode\n");

			imx678->hdr_enabled = !imx678->hdr_enabled;
			imx678_set_mode(imx678, prev_mode);

			revert_ret = imx678_update_exp_vblank_controls(imx678);
			if (revert_ret)
				dev_err(imx678->dev, "Failed to revert to previous mode (hdr_enabled back to %d, ret=%d)\n",
					 imx678->hdr_enabled, revert_ret);
		}

		dev_dbg(imx678->dev, "Set HDR mode to %d", imx678->hdr_enabled);
		imx678_set_exp_activity(imx678);
	}

	return ret;
}

static int imx678_is_rhs1_value_supported(struct imx678 *imx678, u32 rhs1)
{
    /* For 2/2 binning, the RHS1 requirements are more strict,
     * but we can't know which mode is used at this point, so we use all pixel mode requirements */
    return (rhs1 - 1) % 2 == 0;
}

static int imx678_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx678 *imx678 = container_of(ctrl->handler, struct imx678, ctrl_handler);
	u16 reg = 0;
	u32 len = 0;
	int ret = 0;

	switch (ctrl->id) {
	case IMX678_CID_RHS1:
		ctrl->val = imx678->custom_rhs1_value ? imx678->custom_rhs1_value : imx678->cur_mode->rhs1;
		break;
	case IMX678_CID_RHS2:
		ctrl->val = imx678->cur_mode->rhs2;
		break;
    case IMX678_CID_CUSTOM_RHS1:
		ctrl->val = imx678->custom_rhs1_value;
		break;
	case IMX678_CID_SHR0:
		reg = IMX678_REG_SHUTTER;
		len = 3;
		break;
	case IMX678_CID_SHR1:
		reg = IMX678_REG_SHUTTER_SHORT;
		len = 3;
		break;
	case IMX678_CID_SHR2:
		reg = IMX678_REG_SHUTTER_VERY_SHORT;
		len = 3;
		break;
	case IMX678_CID_VMAX:
		reg = IMX678_REG_LPFR;
		len = 3;
		break;
	case IMX678_CID_HMAX:
		reg = IMX678_REG_HMAX;
		len = 2;
		break;
	default:
		dev_err(imx678->dev, "Invalid control %d", ctrl->id);
		return -EINVAL;
	}

	if (reg && len) {
		if (!imx678->streaming) {
			dev_warn(imx678->dev, "Cannot read register 0x%x from sensor while not streaming\n", reg);
			return -EBUSY;
		}

		ret = imx678_read_reg(imx678, reg, len, &ctrl->val);
		if (ret)
			dev_err(imx678->dev, "Failed to read register %d", reg);
	}

	return ret;
}

/**
 * imx678_set_ctrl() - Set subdevice control
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Supported controls:
 * - V4L2_CID_VBLANK
 * - V4L2_CID_TEST_PATTERN
 * - cluster controls:
 *   - V4L2_CID_ANALOGUE_GAIN
 *   - V4L2_CID_EXPOSURE
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx678 *imx678 =
		container_of(ctrl->handler, struct imx678, ctrl_handler);
	u32 analog_gain, exposure, lpfr, max_lpfr;
	int ret = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		imx678->vblank = imx678->vblank_ctrl->val;
		max_lpfr = (imx678->cur_mode->vblank_max + imx678->cur_mode->height) * imx678->cur_mode->dol;
		lpfr = (imx678->vblank + imx678->cur_mode->height) * imx678->cur_mode->dol;

		dev_dbg(imx678->dev, "Received vblank %u, new lpfr %u",
			imx678->vblank, lpfr);
		ret = __v4l2_ctrl_modify_range(
			imx678->lef.exp_ctrl, IMX678_SHR0_FSC_GAP,
			max_lpfr - imx678->cur_mode->rhs2 - IMX678_SHR0_RHS2_GAP,
			IMX678_EXPOSURE_STEP, lpfr - imx678->cur_mode->rhs2 - IMX678_SHR0_RHS2_GAP);
		break;
	case V4L2_CID_EXPOSURE:
		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(imx678->dev))
			return 0;

		exposure = ctrl->val;
		analog_gain = imx678->lef.again_ctrl->val;

		dev_dbg(imx678->dev, "Received exp %u analog gain %u", exposure,
			analog_gain);

		ret = imx678_update_exp_gain(imx678, exposure, analog_gain, LEF);

		pm_runtime_put(imx678->dev);

		break;
	case IMX678_CID_EXPOSURE_SHORT:
		if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
			return 0;

		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(imx678->dev))
			return 0;

		exposure = ctrl->val;
		analog_gain = imx678->sef1.again_ctrl->val;

		dev_dbg(imx678->dev, "Received exp %u analog gain %u", exposure,
			analog_gain);

		ret = imx678_update_exp_gain(imx678, exposure, analog_gain, SEF1);

		pm_runtime_put(imx678->dev);

		break;
	case IMX678_CID_EXPOSURE_VERY_SHORT:
		if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
			return 0;

		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(imx678->dev))
			return 0;

		exposure = ctrl->val;
		analog_gain = imx678->sef2.again_ctrl->val;

		dev_dbg(imx678->dev, "Received exp %u analog gain %u", exposure,
			analog_gain);

		ret = imx678_update_exp_gain(imx678, exposure, analog_gain, SEF2);

		pm_runtime_put(imx678->dev);

		break;
	case V4L2_CID_TEST_PATTERN:
		if (!pm_runtime_get_if_in_use(imx678->dev))
			return 0;
		ret = imx678_set_test_pattern(imx678, ctrl->val);

		pm_runtime_put(imx678->dev);

		break;
	case IMX678_CID_HCG:
		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(imx678->dev))
			return 0;
		
		dev_dbg(imx678->dev, "Setting HCG to %u\n", ctrl->val);

		ret = imx678_set_hcg_mode(imx678, ctrl->val);
		if (ret) {
			dev_err(imx678->dev, "Failed to set HCG mode: %d\n", ret);
		}
		pm_runtime_put(imx678->dev);
    	break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		if (imx678->streaming) {
			dev_warn(imx678->dev, "Cannot set WDR mode while streaming\n");
			return -EBUSY;
		}

		ret = imx678_set_hdr_mode(imx678, ctrl->val);
		break;
	case V4L2_CID_LINK_FREQ:
	case V4L2_CID_PIXEL_RATE:
		ret = 0;
        break;
	case IMX678_CID_CUSTOM_RHS1:
		if (ctrl->val != 0 && !imx678_is_rhs1_value_supported(imx678, ctrl->val)) {
			dev_err(imx678->dev, "Invalid rhs1 value %u\n", ctrl->val);
			return -EINVAL;
		}

		if (imx678->streaming) {
			dev_warn(imx678->dev, "Cannot set custom rhs1 while streaming\n");
			return -EBUSY;
		}

        imx678->custom_rhs1_value = ctrl->val;
		ret = 0;

		if (ctrl->val == 0) {
			// Set the mode back to the builtin mode
			imx678_set_mode(imx678, imx678->hdr_enabled ?
									&supported_hdr_modes[imx678->mode_idx] : 
									&supported_sdr_modes[imx678->mode_idx]);
			ret = imx678_update_exp_vblank_controls(imx678);
		}
		break;
	default:
		dev_err(imx678->dev, "Invalid control %d", ctrl->id);
		ret = -EINVAL;
	}

	return ret;
}

/**
 * imx678_enum_mbus_code() - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to imx678 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device state
 * @code: V4L2 sub-device code enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx678_mode *supported_modes;
	struct imx678 *imx678 = to_imx678(sd);

	if (code->index > 0)
		return -EINVAL;

	if (imx678->hdr_enabled)
		supported_modes = (struct imx678_mode *)supported_hdr_modes;
	else
		supported_modes = (struct imx678_mode *)supported_sdr_modes;
	
	mutex_lock(&imx678->mutex);
	code->code = supported_modes[DEFAULT_MODE_IDX].code;
	mutex_unlock(&imx678->mutex);
	return 0;
}

/**
 * imx678_enum_frame_size() - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to imx678 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device state
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	struct imx678_mode *supported_modes;
	struct imx678 *imx678 = to_imx678(sd);
	int mode_count, i = 0;
	int min_width = INT_MAX;
	int min_height = INT_MAX;
	int max_width = 0;
	int max_height = 0;

	if (fsize->index > 0)
		return -EINVAL;

	if(imx678->hdr_enabled){
		supported_modes = (struct imx678_mode *)supported_hdr_modes;
		mode_count = ARRAY_SIZE(supported_hdr_modes);
	}
	else {
		supported_modes = (struct imx678_mode *)supported_sdr_modes;
		mode_count = ARRAY_SIZE(supported_sdr_modes);
	}

	mutex_lock(&imx678->mutex);
	for(i = 0; i < mode_count; i++){
		if (fsize->code == supported_modes[i].code) {
			if(supported_modes[i].width > max_width)
				max_width = supported_modes[i].width;
			else if(supported_modes[i].width < min_width)
				min_width = supported_modes[i].width;
			else if(supported_modes[i].height > max_height)
				max_height = supported_modes[i].height;
			else if(supported_modes[i].height < min_height)
				min_height = supported_modes[i].height;
		}
	}

	if (max_width == 0 || max_height == 0 ||
		min_width == INT_MAX || min_height == INT_MAX) {
		pr_debug("%s: Invalid code %d\n", __func__, fsize->code);
		mutex_unlock(&imx678->mutex);
		return -EINVAL;
	}

	fsize->min_width = min_width;
	fsize->max_width = max_width;
	fsize->min_height = min_height;
	fsize->max_height = max_height;
	mutex_unlock(&imx678->mutex);

	return 0;
}

/**
 * imx678_fill_pad_format() - Fill subdevice pad format
 *                            from selected sensor mode
 * @imx678: pointer to imx678 device
 * @mode: pointer to imx678_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 */
static void imx678_fill_pad_format(struct imx678 *imx678,
				   const struct imx678_mode *mode,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

/**
 * imx678_get_pad_format() - Get subdevice pad format
 * @sd: pointer to imx678 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device state
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx678 *imx678 = to_imx678(sd);

	mutex_lock(&imx678->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		imx678_fill_pad_format(imx678, imx678->cur_mode, fmt);
	}

	mutex_unlock(&imx678->mutex);

	return 0;
}

static int imx678_get_fmt_mode(struct imx678* imx678, struct v4l2_subdev_format* fmt, const struct imx678_mode** mode){
	struct imx678_mode *supported_modes;
	int mode_count = 0;
	int index = 0;

	if(!imx678 || !fmt || !mode)
		return -EINVAL;

	if(imx678->hdr_enabled){
		supported_modes = (struct imx678_mode *)supported_hdr_modes;
		mode_count = ARRAY_SIZE(supported_hdr_modes);
	}
	else {
		supported_modes = (struct imx678_mode *)supported_sdr_modes;
		mode_count = ARRAY_SIZE(supported_sdr_modes);
	}
	
	convert_v4l2_subdev_code_to_sensor_code(imx678, &fmt->format);
	for (index = 0; index < mode_count; ++index) {
		if (supported_modes[index].width == fmt->format.width && 
		   supported_modes[index].height == fmt->format.height && 
		   supported_modes[index].code == fmt->format.code) {
			*mode = &supported_modes[index];
			return 0;
		}
	}

	return -EINVAL;
}

/**
 * imx678_set_pad_format() - Set subdevice pad format
 * @sd: pointer to imx678 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device state
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx678 *imx678 = to_imx678(sd);
	const struct imx678_mode *mode;
	int ret = 0;
	mutex_lock(&imx678->mutex);

	if (imx678->streaming) {
		dev_err(imx678->dev,
			"Cannot set pad format while streaming\n");
		return -EINVAL;
	}
	
	ret = imx678_get_fmt_mode(imx678, fmt, &mode);
	if(ret){
		pr_err("%s - get_fmt failed with %d (format: %dx%d, code: 0x%x)\n", __func__,
			ret, fmt->format.width, fmt->format.height, fmt->format.code);
		goto out;
	}

	imx678_fill_pad_format(imx678, mode, &imx678->curr_fmt);
	// even if which is V4L2_SUBDEV_FORMAT_TRY, update current format for tuning case
	memcpy(&imx678->curr_fmt, fmt, sizeof(struct v4l2_subdev_format));
	if (compare_imx678_mode(mode, imx678->cur_mode)) {
		imx678_set_mode(imx678, mode);
		ret = imx678_update_exp_vblank_controls(imx678);
	}
#ifdef IMX678_UPDATE_CONTROLS_TRY_FMT
		ret = imx678_update_controls(imx687, mode);
#endif

out:
	mutex_unlock(&imx678->mutex);
	return ret;
}

/**
 * imx678_init_pad_cfg() - Initialize sub-device pad configuration
 * @sd: pointer to imx678 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device state
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_init_pad_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state)
{
	struct imx678_mode *supported_modes;
	struct imx678 *imx678 = to_imx678(sd);
	struct v4l2_subdev_format fmt = { 0 };

	if (imx678->streaming)
		return 0;

	if (imx678->hdr_enabled)
		supported_modes = (struct imx678_mode *)supported_hdr_modes;
	else
		supported_modes = (struct imx678_mode *)supported_sdr_modes;

	fmt.which =
		sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	imx678_fill_pad_format(imx678, &supported_modes[DEFAULT_MODE_IDX],
			       &fmt);

	return imx678_set_pad_format(sd, sd_state, &fmt);
}

static int imx678_setup_custom_values(struct imx678 *imx678)
{
    // Use custom mode to avoid modifying the builtin modes
    struct imx678_mode *mode = &imx678->custom_mode;
	int ret = 0;

    // The RHS1 control is only relevant for HDR mode
    if (imx678->hdr_enabled && (imx678->custom_rhs1_value != IMX678_CUSTOM_RHS1_DEFAULT)) {
        memcpy(mode, imx678->cur_mode, sizeof(struct imx678_mode));
	    mode->rhs1 = imx678->custom_rhs1_value;

        ret = imx678_write_reg(imx678, IMX678_REG_RHS1, 2, mode->rhs1);
        if (ret) {
            dev_err(imx678->dev, "Failed to write custom rhs1 value");
            return ret;
        }

        imx678_set_mode(imx678, mode);
        ret = imx678_update_exp_vblank_controls(imx678);
        if (ret) {
            dev_err(imx678->dev, "Failed to update exp and vblank controls for custom rhs1");
            return ret;
        }
    }

	return ret;
}

/**
 * imx678_start_streaming() - Start sensor stream
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_start_streaming(struct imx678 *imx678)
{
	const struct imx678_reg_list *reg_list;
	int ret;

	/* Write sensor mode registers */
	dev_dbg(imx678->dev, "%s - hdr_enabled: %d\n", __func__, imx678->hdr_enabled);
	reg_list = &imx678->cur_mode->reg_list;
	ret = imx678_write_regs(imx678, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(imx678->dev, "fail to write initial registers");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	ret = __v4l2_ctrl_handler_setup(imx678->sd.ctrl_handler);
	if (ret) {
		dev_err(imx678->dev, "fail to setup handler (%d)", ret);
		return ret;
	}

	/* Setup custom controls */
    ret = imx678_setup_custom_values(imx678);
    if (ret) {
        dev_err(imx678->dev, "failed to setup custom values");
        return ret;
    }

	/* Start streaming */
	ret = imx678_write_reg(imx678, IMX678_REG_MODE_SELECT, 1,
			       IMX678_MODE_STREAMING);
	if (ret) {
		dev_err(imx678->dev, "fail to start streaming");
		return ret;
	}
	/* Start streaming */
	ret = imx678_write_reg(imx678, 0x3002, 1, 0);
	if (ret) {
		dev_err(imx678->dev, "fail to start streaming");
		return ret;
	}

	dev_info(imx678->dev, "imx678: start_streaming successful (%s)", imx678->mode_string);
	return 0;
}

/**
 * imx678_stop_streaming() - Stop sensor stream
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_stop_streaming(struct imx678 *imx678)
{
	return imx678_write_reg(imx678, IMX678_REG_MODE_SELECT, 1,
				IMX678_MODE_STANDBY);
}

/**
 * imx678_set_stream() - Enable sensor streaming
 * @sd: pointer to imx678 subdevice
 * @enable: set to enable sensor streaming
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	mutex_lock(&imx678->mutex);

	if (imx678->streaming == enable) {
		mutex_unlock(&imx678->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(imx678->dev);
		if (ret < 0)
			goto error_unlock;

		ret = imx678_start_streaming(imx678);
		if (ret)
			goto error_power_off;
	} else {
		imx678_stop_streaming(imx678);
		pm_runtime_put(imx678->dev);
	}

	imx678->streaming = enable;

	mutex_unlock(&imx678->mutex);

	return 0;

error_power_off:
	pm_runtime_put(imx678->dev);
error_unlock:
	mutex_unlock(&imx678->mutex);

	return ret;
}

static int
imx678_find_nearest_frame_interval_mode(struct imx678 *imx678,
					struct v4l2_subdev_frame_interval *fi,
					struct imx678_mode const **mode)
{
	struct imx678_mode const* curr_mode;
	struct v4l2_mbus_framefmt* framefmt;
	struct imx678_mode *supported_modes;

	int min_diff = INT_MAX;
	int curr_diff;
	int i, found = 0, mode_count = 0;

	if (!imx678 || !fi) {
		return -EINVAL;
	}

	if(imx678->hdr_enabled){
		supported_modes = (struct imx678_mode *)supported_hdr_modes;
		mode_count = ARRAY_SIZE(supported_hdr_modes);
	}
	else {
		supported_modes = (struct imx678_mode *)supported_sdr_modes;
		mode_count = ARRAY_SIZE(supported_sdr_modes);
	}

	framefmt = &imx678->curr_fmt.format;

	for (i = 0; i < mode_count; ++i) {
		curr_mode = &supported_modes[i];

		if(curr_mode->width != framefmt->width || curr_mode->height != framefmt->height || curr_mode->code != framefmt->code)
			continue;
		found = 1;

		curr_diff = abs(curr_mode->frame_interval.denominator -
				(int)(fi->interval.denominator /
				      fi->interval.numerator));
		if (curr_diff == 0) {
			*mode = curr_mode;
			return 0;
		}
		if (curr_diff < min_diff) {
			min_diff = curr_diff;
			*mode = curr_mode;
		}
	}

	if(!found){
		return -ENOTSUPP;
	}

	return 0;
}

/**
 * imx678_s_frame_interval - Set the frame interval
 * @sd: Pointer to V4L2 Sub device structure
 * @fi: Pointer to V4l2 Sub device frame interval structure
 *
 * This function is used to set the frame intervavl.
 *
 * Return: 0 on success
 */
static int imx678_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx678 *imx678 = to_imx678(sd);
	struct imx678_mode const *mode;
	int ret;

	ret = pm_runtime_resume_and_get(imx678->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&imx678->mutex);

	ret = imx678_find_nearest_frame_interval_mode(imx678, fi, &mode);

	if (ret == 0) {
		fi->interval = mode->frame_interval;
		if (compare_imx678_mode(mode, imx678->cur_mode)) {
			imx678_set_mode(imx678, mode);
			ret = imx678_update_exp_vblank_controls(imx678);
		}
	}

	mutex_unlock(&imx678->mutex);
	pm_runtime_put(imx678->dev);

	return ret;
}

static int imx678_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx678 *imx678 = to_imx678(sd);

	mutex_lock(&imx678->mutex);
	fi->interval = imx678->cur_mode->frame_interval;
	mutex_unlock(&imx678->mutex);

	return 0;
}

/**
 * imx678_detect() - Detect imx678 sensor
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int imx678_detect(struct imx678 *imx678)
{
	int ret;
	u32 val;


	ret = imx678_read_reg(imx678, GENERIC_SENSOR_ID_REG, 1, &val);
	if (ret)
		return ret;

	if (val != SENSOR_ID_IMX678) {
		dev_info(imx678->dev,
			"sensor is not connected: (expected %x, found %x)",
			SENSOR_ID_IMX678, val);
		return -ENXIO;
	}

	dev_info(imx678->dev, "sensor detected!");
	return 0;
}

/**
 * imx678_parse_hw_config() - Parse HW configuration and check if supported
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_parse_hw_config(struct imx678 *imx678)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx678->dev);
	struct v4l2_fwnode_endpoint bus_cfg = { .bus_type =
							V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *ep;
	unsigned long rate;
	int ret;
	int i, j;

	if (!fwnode)
		return -ENXIO;

	/* Request optional reset pin */
	imx678->reset_gpio =
		devm_gpiod_get_optional(imx678->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx678->reset_gpio)) {
		dev_err(imx678->dev, "failed to get reset gpio %ld",
			PTR_ERR(imx678->reset_gpio));
		return PTR_ERR(imx678->reset_gpio);
	}

	imx678->xmaster_gpio =
		devm_gpiod_get_optional(imx678->dev, "xmaster", GPIOD_OUT_LOW);
	if (IS_ERR(imx678->xmaster_gpio)) {
		dev_err(imx678->dev, "failed to get xmaster gpio %ld",
			PTR_ERR(imx678->xmaster_gpio));
		return PTR_ERR(imx678->xmaster_gpio);
	}

	/* Get sensor input clock */
	imx678->inclk = devm_clk_get(imx678->dev, NULL);
	if (IS_ERR(imx678->inclk)) {
		dev_err(imx678->dev, "could not get inclk");
		return PTR_ERR(imx678->inclk);
	}

	rate = clk_get_rate(imx678->inclk);
	if (rate != IMX678_INCLK_RATE) {
		dev_err(imx678->dev, "inclk frequency mismatch");
		return -EINVAL;
	}

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX678_NUM_DATA_LANES) {
		dev_err(imx678->dev,
			"number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx678->dev, "no link frequencies defined");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	/* check if all the required frequencies are provided in the device tree */
	for (i = 0; i < ARRAY_SIZE(link_freq); i++) {
		for (j = 0; j < bus_cfg.nr_of_link_frequencies; j++) {
			if (bus_cfg.link_frequencies[j] ==
			    link_freq[i]) {
				break;
			}
		}
		if (j == bus_cfg.nr_of_link_frequencies) {
			dev_err(imx678->dev,
				"required link frequency %lld not supported in device tree",
				link_freq[i]);
			ret = -EINVAL;
			goto done_endpoint_free;
		}
	}

	ret = 0;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops imx678_video_ops = {
	.s_stream = imx678_set_stream,
	.s_frame_interval = imx678_s_frame_interval,
	.g_frame_interval = imx678_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx678_pad_ops = {
	.init_cfg = imx678_init_pad_cfg,
	.enum_mbus_code = imx678_enum_mbus_code,
	.enum_frame_size = imx678_enum_frame_size,
	.get_fmt = imx678_get_pad_format,
	.set_fmt = imx678_set_pad_format,
};

static const struct v4l2_subdev_ops imx678_subdev_ops = {
	.video = &imx678_video_ops,
	.pad = &imx678_pad_ops,
};

/**
 * imx678_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	gpiod_set_value_cansleep(imx678->reset_gpio, 1);

	ret = clk_prepare_enable(imx678->inclk);
	if (ret) {
		dev_err(imx678->dev, "fail to enable inclk");
		goto error_reset;
	}

	usleep_range(18000, 20000);

	return 0;

error_reset:
	gpiod_set_value_cansleep(imx678->reset_gpio, 0);

	return ret;
}

/**
 * imx678_power_off() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx678 *imx678 = to_imx678(sd);

	gpiod_set_value_cansleep(imx678->reset_gpio, 0);

	clk_disable_unprepare(imx678->inclk);

	return 0;
}

/**
 * imx678_init_controls() - Initialize sensor subdevice controls
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_init_controls(struct imx678 *imx678)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &imx678->ctrl_handler;
	const struct imx678_mode *mode = imx678->cur_mode;
	struct ExposureLimits_t limits;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, ARRAY_SIZE(imx678_custom_ctrls));
	if (ret) {
		dev_err(imx678->dev, "failed to init control handler (%d)", ret);
		return ret;
	}

	memset(&limits, 0, sizeof(struct ExposureLimits_t));
	calculate_exposure_limits(imx678, &limits);

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &imx678->mutex;

	/* Initialize exposure and gain LEF */
	imx678->lef.exp_ctrl = v4l2_ctrl_new_std(
		ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_EXPOSURE,
		limits.exp_lef_min, limits.exp_lef_max,
		IMX678_EXPOSURE_STEP, limits.exp_lef_default);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create LEF exposure control (%d)", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	imx678->lef.again_ctrl =
		v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
				  V4L2_CID_ANALOGUE_GAIN, IMX678_AGAIN_MIN,
				  IMX678_AGAIN_MAX, IMX678_AGAIN_STEP,
				  IMX678_AGAIN_DEFAULT);
	
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create LEF gain control (%d)", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	v4l2_ctrl_cluster(2, &imx678->lef.exp_ctrl);

	/* Initialize exposure and gain SEF1 */
	imx678_setup_custom_ctrl_limits(imx678, &imx678->sef1.exp_ctrl, IMX678_CID_EXPOSURE_SHORT,
		limits.exp_sef1_min, limits.exp_sef1_max, limits.exp_sef1_default);
	imx678_setup_custom_ctrl(imx678, &imx678->sef1.again_ctrl, IMX678_CID_ANALOGUE_GAIN_SHORT);
	v4l2_ctrl_cluster(2, &imx678->sef1.exp_ctrl);

	/* Initialize exposure and gain SEF2 */
	imx678_setup_custom_ctrl_limits(imx678, &imx678->sef2.exp_ctrl, IMX678_CID_EXPOSURE_VERY_SHORT,
		limits.exp_sef2_min, limits.exp_sef2_max, limits.exp_sef2_default);
	imx678_setup_custom_ctrl(imx678, &imx678->sef2.again_ctrl, IMX678_CID_ANALOGUE_GAIN_VERY_SHORT);
	v4l2_ctrl_cluster(2, &imx678->sef2.exp_ctrl);

	/* Read only HDR custom controls */
	imx678_setup_custom_ctrl(imx678, &imx678->rhs1_ctrl, IMX678_CID_RHS1);
	imx678_setup_custom_ctrl(imx678, &imx678->rhs2_ctrl, IMX678_CID_RHS2);
	imx678_setup_custom_ctrl(imx678, &imx678->shr0_ctrl, IMX678_CID_SHR0);
	imx678_setup_custom_ctrl(imx678, &imx678->shr1_ctrl, IMX678_CID_SHR1);
	imx678_setup_custom_ctrl(imx678, &imx678->shr2_ctrl, IMX678_CID_SHR2);
	
	/* Other read only custom controls */
	imx678_setup_custom_ctrl(imx678, &imx678->vmax_ctrl, IMX678_CID_VMAX);
	imx678_setup_custom_ctrl(imx678, &imx678->hmax_ctrl, IMX678_CID_HMAX);

	/* Initialize HCG control */
	imx678_setup_custom_ctrl(imx678, &imx678->hcg_ctrl, IMX678_CID_HCG);

	 /* Custom value controls */
	imx678_setup_custom_ctrl_limits(imx678, &imx678->custom_rhs1_ctrl, IMX678_CID_CUSTOM_RHS1,
		IMX678_CUSTOM_RHS1_MIN, IMX678_CUSTOM_RHS1_MAX, IMX678_CUSTOM_RHS1_DEFAULT);

	imx678->vblank_ctrl =
		v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_VBLANK,
				  mode->vblank_min, mode->vblank_max, 1,
				  mode->vblank);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create vblank control (%d)", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	imx678->test_pattern_ctrl = v4l2_ctrl_new_std_menu_items(
		ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(imx678_test_pattern_menu) - 1, 0, 0,
		imx678_test_pattern_menu);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create test pattern control (%d)", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}
	
	imx678->mode_sel_ctrl = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
				V4L2_CID_WIDE_DYNAMIC_RANGE, IMX678_WDR_MIN,
				IMX678_WDR_MAX, IMX678_WDR_STEP,
				IMX678_WDR_DEFAULT);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create mode select control (%d)", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}
	
	/* Read only controls */
	imx678->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&imx678_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						link_freq[0],
						link_freq[ARRAY_SIZE(link_freq) - 1],
						1,
						mode->pclk);
	if (imx678->pclk_ctrl)
		imx678->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create pixel rate control link_freq min (%lld), link_freq max (%lld), pclk %llu",
			link_freq[0], link_freq[ARRAY_SIZE(link_freq) - 1], mode->pclk);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	imx678->link_freq_ctrl = v4l2_ctrl_new_int_menu(
		ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_LINK_FREQ,
		ARRAY_SIZE(link_freq) - 1, mode->link_freq_idx, link_freq);
	if (imx678->link_freq_ctrl)
		imx678->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create link frequency control (%d)", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	imx678->hblank_ctrl =
		v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_HBLANK,
				  IMX678_REG_MIN, IMX678_REG_MAX, 1,
				  mode->hblank);
	if (imx678->hblank_ctrl)
		imx678->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx678->dev, "failed to create hblank control (%d)", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	imx678->sd.ctrl_handler = ctrl_hdlr;
	
	return 0;
}

/**
 * imx678_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_probe(struct i2c_client *client)
{
	struct imx678 *imx678;
	int ret;
	imx678 = devm_kzalloc(&client->dev, sizeof(*imx678), GFP_KERNEL);
	if (!imx678)
		return -ENOMEM;

	imx678->dev = &client->dev;
	dev_info(imx678->dev, "probe started");

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx678->sd, client, &imx678_subdev_ops);

	ret = imx678_parse_hw_config(imx678);
	if (ret) {
		dev_err(imx678->dev, "HW configuration is not supported");
		return ret;
	}

	mutex_init(&imx678->mutex);

	ret = imx678_power_on(imx678->dev);
	if (ret) {
		dev_err(imx678->dev, "failed to power-on the sensor");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = imx678_detect(imx678);
	if (ret == -ENXIO) {
		// imx678 is not connected, but another sensor might be
		goto error_power_off;
	} else if (ret) {
		dev_err(imx678->dev, "failed to find sensor: %d", ret);
		goto error_power_off;
	}

	/* Set default mode to max resolution sdr */
	imx678_set_mode(imx678, &supported_sdr_modes[DEFAULT_MODE_IDX]);

	/* Initialize custom values to default */
	imx678->custom_rhs1_value = IMX678_CUSTOM_RHS1_DEFAULT;

	ret = imx678_init_controls(imx678);
	if (ret) {
		dev_err(imx678->dev, "failed to init controls: %d", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	imx678->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx678->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx678->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx678->sd.entity, 1, &imx678->pad);
	if (ret) {
		dev_err(imx678->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx678->sd);
	if (ret < 0) {
		dev_err(imx678->dev, "failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(imx678->dev);
	pm_runtime_enable(imx678->dev);
	pm_runtime_idle(imx678->dev);

	dev_info(imx678->dev, "probe finished successfully");
	return 0;

error_media_entity:
	media_entity_cleanup(&imx678->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(imx678->sd.ctrl_handler);
error_power_off:
	imx678_power_off(imx678->dev);
error_mutex_destroy:
	mutex_destroy(&imx678->mutex);

	if (ret == -ENXIO) {
		dev_info(imx678->dev, "exit probe, sensor not connected");
	} else {
		dev_err(imx678->dev, "probe failed with %d", ret);
	}
	return ret;
}

/**
 * imx678_remove() - I2C client device unbinding
 * @client: pointer to I2C client device
 */
static void imx678_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	pm_runtime_suspended(&client->dev);

	mutex_destroy(&imx678->mutex);
}

static const struct dev_pm_ops imx678_pm_ops = { SET_RUNTIME_PM_OPS(
	imx678_power_off, imx678_power_on, NULL) };

static const struct of_device_id imx678_of_match[] = {
	{ .compatible = "sony,imx678" },
	{}
};

MODULE_DEVICE_TABLE(of, imx678_of_match);

static struct i2c_driver imx678_driver = {
	.probe_new = imx678_probe,
	.remove = imx678_remove,
	.driver = {
		.name = "imx678",
		.pm = &imx678_pm_ops,
		.of_match_table = imx678_of_match,
	},
};

module_i2c_driver(imx678_driver);

MODULE_DESCRIPTION("Sony imx678 sensor driver");
MODULE_LICENSE("GPL");
