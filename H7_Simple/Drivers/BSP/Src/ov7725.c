/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7725 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "camera.h"
#include "ov7725.h"
#include "ov7725_regs.h"

static uint8_t g_slv_addr;

static const uint8_t default_regs[][2] = {

// From App Note.

    {COM12,         0x03},
    {HSTART,        0x22},
    {HSIZE,         0xa4},
    {VSTART,        0x07},
    {VSIZE,         0xf0},
    {HREF,          0x00},
    {HOUTSIZE,      0xa0},
    {VOUTSIZE,      0xf0},
    {EXHCH,         0x00},
    {CLKRC,         0xC0}, // {CLKRC, 0x01},

    {TGT_B,         0x7f},
    {FIXGAIN,       0x09},
    {AWB_CTRL0,     0xe0},
    {DSP_CTRL1,     0xff},
    {DSP_CTRL2,     0x20 | DSP_CTRL2_VDCW_EN | DSP_CTRL2_HDCW_EN | DSP_CTRL2_VZOOM_EN | DSP_CTRL2_HZOOM_EN}, // {DSP_CTRL2, 0x20},
    {DSP_CTRL3,     0x00},
    {DSP_CTRL4,     0x48},

    {COM8,          0xf0},
    {COM4,          OMV_OV7725_PLL_CONFIG}, // {COM4, 0x41},
    {COM6,          0xc5},
    {COM9,          0x11},
    {BDBASE,        0x7f},
    {BDSTEP,        0x03},
    {AEW,           0x40},
    {AEB,           0x30},
    {VPT,           0xa1},
    {EXHCL,         0x00},
    {AWB_CTRL3,     0xaa},
    {COM8,          0xff},

    {EDGE1,         0x05},
    {DNSOFF,        0x01},
    {EDGE2,         0x03},
    {EDGE3,         0x00},
    {MTX1,          0xb0},
    {MTX2,          0x9d},
    {MTX3,          0x13},
    {MTX4,          0x16},
    {MTX5,          0x7b},
    {MTX6,          0x91},
    {MTX_CTRL,      0x1e},
    {BRIGHTNESS,    0x08},
    {CONTRAST,      0x20},
    {UVADJ0,        0x81},
    {SDE,           SDE_CONT_BRIGHT_EN | SDE_SATURATION_EN},

    {GAM1,          0x0c},
    {GAM2,          0x16},
    {GAM3,          0x2a},
    {GAM4,          0x4e},
    {GAM5,          0x61},
    {GAM6,          0x6f},
    {GAM7,          0x7b},
    {GAM8,          0x86},
    {GAM9,          0x8e},
    {GAM10,         0x97},
    {GAM11,         0xa4},
    {GAM12,         0xaf},
    {GAM13,         0xc5},
    {GAM14,         0xd7},
    {GAM15,         0xe8},
    {SLOP,          0x20},

    {DM_LNL,        0x00},
    {BDBASE,        OMV_OV7725_BANDING}, // {BDBASE, 0x7f}
    {BDSTEP,        0x03},

    {LC_RADI,       0x10},
    {LC_COEF,       0x10},
    {LC_COEFB,      0x14},
    {LC_COEFR,      0x17},
    {LC_CTR,        0x01}, // {LC_CTR, 0x05},

    {COM5,          0xf5}, // {COM5, 0x65},

// OpenMV Custom.

    {COM7,          COM7_FMT_RGB565},

// End.

    {0x00,          0x00},
};

#define NUM_BRIGHTNESS_LEVELS (9)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS][2] = {
    {0x38, 0x0e}, /* -4 */
    {0x28, 0x0e}, /* -3 */
    {0x18, 0x0e}, /* -2 */
    {0x08, 0x0e}, /* -1 */
    {0x08, 0x06}, /*  0 */
    {0x18, 0x06}, /* +1 */
    {0x28, 0x06}, /* +2 */
    {0x38, 0x06}, /* +3 */
    {0x48, 0x06}, /* +4 */
};

#define NUM_CONTRAST_LEVELS (9)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS][1] = {
    {0x10}, /* -4 */
    {0x14}, /* -3 */
    {0x18}, /* -2 */
    {0x1C}, /* -1 */
    {0x20}, /*  0 */
    {0x24}, /* +1 */
    {0x28}, /* +2 */
    {0x2C}, /* +3 */
    {0x30}, /* +4 */
};

#define NUM_SATURATION_LEVELS (9)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS][2] = {
    {0x00, 0x00}, /* -4 */
    {0x10, 0x10}, /* -3 */
    {0x20, 0x20}, /* -2 */
    {0x30, 0x30}, /* -1 */
    {0x40, 0x40}, /*  0 */
    {0x50, 0x50}, /* +1 */
    {0x60, 0x60}, /* +2 */
    {0x70, 0x70}, /* +3 */
    {0x80, 0x80}, /* +4 */
};

const int resolution[][2] = {
    {0,    0   },
    // C/SIF Resolutions
    {88,   72  },    /* QQCIF     */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {88,   60  },    /* QQSIF     */
    {176,  120 },    /* QSIF      */
    {352,  240 },    /* SIF       */
    // VGA Resolutions
    {40,   30  },    /* QQQQVGA   */
    {80,   60  },    /* QQQVGA    */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {640,  480 },    /* VGA       */
    {60,   40  },    /* HQQQVGA   */
    {120,  80  },    /* HQQVGA    */
    {240,  160 },    /* HQVGA     */
    // FFT Resolutions
    {64,   32  },    /* 64x32     */
    {64,   64  },    /* 64x64     */
    {128,  64  },    /* 128x64    */
    {128,  128 },    /* 128x64    */
    // Other
    {128,  160 },    /* LCD       */
    {128,  160 },    /* QQVGA2    */
    {720,  480 },    /* WVGA      */
    {752,  480 },    /* WVGA2     */
    {800,  600 },    /* SVGA      */
    {1280, 1024},    /* SXGA      */
    {1600, 1200},    /* UXGA      */
};


static int reset();
static int reset()
{
    // Reset all registers
    CAMERA_IO_Write(g_slv_addr, COM7, COM7_RESET);

    // Delay 2 ms

    // Write default regsiters
    for (int i = 0; default_regs[i][0]; i++) {
        CAMERA_IO_Write(g_slv_addr, default_regs[i][0], default_regs[i][1]);
    }

    // Delay 300 ms

    return 0;
}

static int sleep(int enable);
static int sleep(int enable)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM2);

    if (enable) {
        reg |= COM2_SOFT_SLEEP;
    } else {
        reg &= ~COM2_SOFT_SLEEP;
    }

    // Write back register
    CAMERA_IO_Write(g_slv_addr, COM2, reg);
    return 0;
}

static int read_reg(uint8_t reg_addr);
static int read_reg(uint8_t reg_addr)
{
    uint8_t reg_data;
    reg_data = CAMERA_IO_Read(g_slv_addr, reg_addr);
    return reg_data;
}

static int write_reg(uint8_t reg_addr, uint16_t reg_data);
static int write_reg(uint8_t reg_addr, uint16_t reg_data)
{
    CAMERA_IO_Write(g_slv_addr, reg_addr, reg_data);
    return 0;
}

static int set_pixformat(pixformat_t pixformat);
static int set_pixformat(pixformat_t pixformat)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM7);

    switch (pixformat) {
        case PIXFORMAT_RGB565:
            reg = COM7_SET_FMT(reg, COM7_FMT_RGB);
            reg = COM7_SET_FMT(reg, COM7_FMT_RGB565);
            CAMERA_IO_Write(g_slv_addr, DSP_CTRL4, DSP_CTRL4_YUV_RGB);
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
            reg = COM7_SET_FMT(reg, COM7_FMT_YUV);
            CAMERA_IO_Write(g_slv_addr, DSP_CTRL4, DSP_CTRL4_YUV_RGB);
            break;
        case PIXFORMAT_BAYER:
            reg = COM7_SET_FMT(reg, COM7_FMT_R_BAYER);
            CAMERA_IO_Write(g_slv_addr, DSP_CTRL4, DSP_CTRL4_RAW8);
            break;
        default:
            return -1;
    }

    // Write back register
    CAMERA_IO_Write(g_slv_addr, COM7, reg);
    return 0;
}

static int set_framesize(framesize_t framesize);
static int set_framesize(framesize_t framesize)
{
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];
    
    printf("W %d H %d \n", w, h);

    // Write MSBs
    CAMERA_IO_Write(g_slv_addr, HOUTSIZE, w>>2);
    CAMERA_IO_Write(g_slv_addr, VOUTSIZE, h>>1);

    // Write LSBs
    CAMERA_IO_Write(g_slv_addr, EXHCH, ((w&0x3) | ((h&0x1) << 2)));

    if ((w <= 320) && (h <= 240)) {
        // Set QVGA Resolution
        uint8_t reg;
        reg = CAMERA_IO_Read(g_slv_addr, COM7);
        reg = COM7_SET_RES(reg, COM7_RES_QVGA);
        CAMERA_IO_Write(g_slv_addr, COM7, reg);

        // Set QVGA Window Size
        CAMERA_IO_Write(g_slv_addr, HSTART, 0x3F);
        CAMERA_IO_Write(g_slv_addr, HSIZE,  0x50);
        CAMERA_IO_Write(g_slv_addr, VSTART, 0x03);
        CAMERA_IO_Write(g_slv_addr, VSIZE,  0x78);

        // Enable auto-scaling/zooming factors
        CAMERA_IO_Write(g_slv_addr, DSPAUTO, 0xFF);
    } else {
        // Set VGA Resolution
        uint8_t reg;
        reg = CAMERA_IO_Read(g_slv_addr, COM7);
        reg = COM7_SET_RES(reg, COM7_RES_VGA);
        CAMERA_IO_Write(g_slv_addr, COM7, reg);

        // Set VGA Window Size
        CAMERA_IO_Write(g_slv_addr, HSTART, 0x23);
        CAMERA_IO_Write(g_slv_addr, HSIZE,  0xA0);
        CAMERA_IO_Write(g_slv_addr, VSTART, 0x07);
        CAMERA_IO_Write(g_slv_addr, VSIZE,  0xF0);

        // Disable auto-scaling/zooming factors
        CAMERA_IO_Write(g_slv_addr, DSPAUTO, 0xF3);

        // Clear auto-scaling/zooming factors
        CAMERA_IO_Write(g_slv_addr, SCAL0, 0x00);
        CAMERA_IO_Write(g_slv_addr, SCAL1, 0x40);
        CAMERA_IO_Write(g_slv_addr, SCAL2, 0x40);
    }

    return 0;
}

static int set_framerate(framerate_t framerate);
static int set_framerate(framerate_t framerate)
{
    return 0;
}

static int set_contrast(int level);
static int set_contrast(int level)
{
    level += (NUM_CONTRAST_LEVELS / 2);
    if (level < 0 || level >= NUM_CONTRAST_LEVELS) {
        return -1;
    }

    CAMERA_IO_Write(g_slv_addr, CONTRAST, contrast_regs[level][0]);
    return 0;
}

static int set_brightness(int level);
static int set_brightness(int level)
{
    level += (NUM_BRIGHTNESS_LEVELS / 2);
    if (level < 0 || level >= NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    CAMERA_IO_Write(g_slv_addr, BRIGHTNESS, brightness_regs[level][0]);
    CAMERA_IO_Write(g_slv_addr, SIGN_BIT,   brightness_regs[level][1]);
    return 0;
}

static int set_saturation(int level);
static int set_saturation(int level)
{
    int ret=0;
    level += (NUM_SATURATION_LEVELS / 2 );
    if (level < 0 || level >= NUM_SATURATION_LEVELS) {
        return -1;
    }

    CAMERA_IO_Write(g_slv_addr, USAT, saturation_regs[level][0]);
    CAMERA_IO_Write(g_slv_addr, VSAT, saturation_regs[level][1]);
    return 0;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM9);

    // Set gain ceiling
    reg = COM9_SET_AGC(reg, gainceiling);
    CAMERA_IO_Write(g_slv_addr, COM9, reg);
    return 0;
}

static int set_colorbar(int enable);
static int set_colorbar(int enable)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM3);

    // Enable colorbar test pattern output
    reg = COM3_SET_CBAR(reg, enable);
    CAMERA_IO_Write(g_slv_addr, COM3, reg);

    // Enable DSP colorbar output
    reg = CAMERA_IO_Read(g_slv_addr, DSP_CTRL3);
    reg = DSP_CTRL3_SET_CBAR(reg, enable);
    CAMERA_IO_Write(g_slv_addr, DSP_CTRL3, reg) ;
    return 0;
}

static int set_auto_gain(int enable, float gain_db, float gain_db_ceiling);
static int set_auto_gain(int enable, float gain_db, float gain_db_ceiling)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM8);
    CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AGC(reg, (enable != 0)));

    if ((enable == 0) && (!isnanf(gain_db)) && (!isinff(gain_db))) {
        float gain = IM_MAX(IM_MIN(fast_expf((gain_db / 20.0) * fast_log(10.0)), 32.0), 1.0);

        int gain_temp = fast_roundf(fast_log2(IM_MAX(gain / 2.0, 1.0)));
        int gain_hi = 0xF >> (4 - gain_temp);
        int gain_lo = IM_MIN(fast_roundf(((gain / (1 << gain_temp)) - 1.0) * 16.0), 15);

        CAMERA_IO_Write(g_slv_addr, GAIN, (gain_hi << 4) | (gain_lo << 0));
    } else if ((enable != 0) && (!isnanf(gain_db_ceiling)) && (!isinff(gain_db_ceiling))) {
        float gain_ceiling = IM_MAX(IM_MIN(fast_expf((gain_db_ceiling / 20.0) * fast_log(10.0)), 32.0), 2.0);

        reg = CAMERA_IO_Read(g_slv_addr, COM9);
        CAMERA_IO_Write(g_slv_addr, COM9, (reg & 0x8F) | ((fast_ceilf(fast_log2(gain_ceiling)) - 1) << 4));
    }

    return 0;
}

static int get_gain_db(float *gain_db);
static int get_gain_db(float *gain_db)
{
    uint8_t reg, gain;
    reg = CAMERA_IO_Read(g_slv_addr, COM8);

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AGC(reg, 0));
    // }
    // DISABLED

    gain = CAMERA_IO_Read(g_slv_addr, GAIN);

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AGC(reg, 1));
    // }
    // DISABLED

    int hi_gain = 1 << (((gain >> 7) & 1) + ((gain >> 6) & 1) + ((gain >> 5) & 1) + ((gain >> 4) & 1));
    float lo_gain = 1.0 + (((gain >> 0) & 0xF) / 16.0);
    *gain_db = 20.0 * (fast_log(hi_gain * lo_gain) / fast_log(10.0));

    return 0;
}

static int set_auto_exposure(int enable, int exposure_us);
static int set_auto_exposure(int enable, int exposure_us)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM8);
    CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AEC(reg, (enable != 0)));

    if ((enable == 0) && (exposure_us >= 0)) {
        reg = CAMERA_IO_Read(g_slv_addr, COM7);

        int t_line = (reg & COM7_RES_QVGA) ? (320 + 256) : (640 + 144);
        int t_pclk = (COM7_GET_FMT(reg) == COM7_FMT_P_BAYER) ? 1 : 2;

        reg = CAMERA_IO_Read(g_slv_addr, COM4);
        int pll_mult = 0;

        if (COM4_GET_PLL(reg) == COM4_PLL_BYPASS) pll_mult = 1;
        if (COM4_GET_PLL(reg) == COM4_PLL_4x) pll_mult = 4;
        if (COM4_GET_PLL(reg) == COM4_PLL_6x) pll_mult = 6;
        if (COM4_GET_PLL(reg) == COM4_PLL_8x) pll_mult = 8;

        reg = CAMERA_IO_Read(g_slv_addr, CLKRC);
        int clk_rc = 0;

        if (reg & CLKRC_NO_PRESCALE) {
            clk_rc = 1;
        } else {
            clk_rc = ((reg & CLKRC_PRESCALER) + 1) * 2;
        }

        int exposure = IM_MAX(IM_MIN(((exposure_us*(((OMV_XCLK_FREQUENCY/clk_rc)*pll_mult)/1000000))/t_pclk)/t_line,0xFFFF),0x0000);

        CAMERA_IO_Write(g_slv_addr, AEC, ((exposure >> 0) & 0xFF));
        CAMERA_IO_Write(g_slv_addr, AECH, ((exposure >> 8) & 0xFF));
    }

    return 0;
}

static int get_exposure_us(int *exposure_us);
static int get_exposure_us(int *exposure_us)
{
    uint8_t reg, aec_l, aec_h;
    reg = CAMERA_IO_Read(g_slv_addr, COM8);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AEC(reg, 0));
    // }
    // DISABLED

    aec_l = CAMERA_IO_Read(g_slv_addr, AEC);
    aec_h = CAMERA_IO_Read(g_slv_addr, AECH);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AEC(reg, 1));
    // }
    // DISABLED

    reg = CAMERA_IO_Read(g_slv_addr, COM7);

    int t_line = (reg & COM7_RES_QVGA) ? (320 + 256) : (640 + 144);
    int t_pclk = (COM7_GET_FMT(reg) == COM7_FMT_P_BAYER) ? 1 : 2;

    reg = CAMERA_IO_Read(g_slv_addr, COM4);
    int pll_mult = 0;

    if (COM4_GET_PLL(reg) == COM4_PLL_BYPASS) pll_mult = 1;
    if (COM4_GET_PLL(reg) == COM4_PLL_4x) pll_mult = 4;
    if (COM4_GET_PLL(reg) == COM4_PLL_6x) pll_mult = 6;
    if (COM4_GET_PLL(reg) == COM4_PLL_8x) pll_mult = 8;

    reg = CAMERA_IO_Read(g_slv_addr, CLKRC);
    int clk_rc = 0;

    if (reg & CLKRC_NO_PRESCALE) {
        clk_rc = 1;
    } else {
        clk_rc = ((reg & CLKRC_PRESCALER) + 1) * 2;
    }

    *exposure_us = (((aec_h<<8)+(aec_l<<0))*t_line*t_pclk)/(((OMV_XCLK_FREQUENCY/clk_rc)*pll_mult)/1000000);

    return 0;
}

static int set_auto_whitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db);
static int set_auto_whitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM8);
    CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AWB(reg, (enable != 0)));

    if ((enable == 0) && (!isnanf(r_gain_db)) && (!isnanf(g_gain_db)) && (!isnanf(b_gain_db))
                      && (!isinff(r_gain_db)) && (!isinff(g_gain_db)) && (!isinff(b_gain_db))) {
        reg = CAMERA_IO_Read(g_slv_addr, AWB_CTRL1);
        float gain_div = (reg & 0x2) ? 64.0 : 128.0;

        int r_gain = IM_MAX(IM_MIN(fast_roundf(fast_expf((r_gain_db / 20.0) * fast_log(10.0)) * gain_div), 255), 0);
        int g_gain = IM_MAX(IM_MIN(fast_roundf(fast_expf((g_gain_db / 20.0) * fast_log(10.0)) * gain_div), 255), 0);
        int b_gain = IM_MAX(IM_MIN(fast_roundf(fast_expf((b_gain_db / 20.0) * fast_log(10.0)) * gain_div), 255), 0);

        CAMERA_IO_Write(g_slv_addr, BLUE, b_gain);
        CAMERA_IO_Write(g_slv_addr, RED, r_gain);
        CAMERA_IO_Write(g_slv_addr, GREEN, g_gain);
    }

    return 0;
}

static int get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db);
static int get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    uint8_t reg, blue, red, green;
    reg = CAMERA_IO_Read(g_slv_addr, COM8);

    // DISABLED
    // if (reg & COM8_AWB_EN) {
    //     ret |= CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AWB(reg, 0));
    // }
    // DISABLED

    blue = CAMERA_IO_Read(g_slv_addr, BLUE);
    red = CAMERA_IO_Read(g_slv_addr, RED);
    green = CAMERA_IO_Read(g_slv_addr, GREEN);

    // DISABLED
    // if (reg & COM8_AWB_EN) {
    //     ret |= CAMERA_IO_Write(g_slv_addr, COM8, COM8_SET_AWB(reg, 1));
    // }
    // DISABLED

    reg = CAMERA_IO_Read(g_slv_addr, AWB_CTRL1);
    float gain_div = (reg & 0x2) ? 64.0 : 128.0;

    *r_gain_db = 20.0 * (fast_log(red / gain_div) / fast_log(10.0));
    *g_gain_db = 20.0 * (fast_log(green / gain_div) / fast_log(10.0));
    *b_gain_db = 20.0 * (fast_log(blue / gain_div) / fast_log(10.0));

    return 0;
}

static int set_hmirror(int enable);
static int set_hmirror(int enable)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM3);
    CAMERA_IO_Write(g_slv_addr, COM3, COM3_SET_MIRROR(reg, enable)) ;

    return 0;
}

static int set_vflip(int enable);
static int set_vflip(int enable)
{
    uint8_t reg;
    reg = CAMERA_IO_Read(g_slv_addr, COM3);
    CAMERA_IO_Write(g_slv_addr, COM3, COM3_SET_FLIP(reg, enable));

    return 0;
}

static int set_special_effect(sde_t sde);
static int set_special_effect(sde_t sde)
{

    switch (sde) {
        case SDE_NEGATIVE:
            CAMERA_IO_Write(g_slv_addr, SDE, 0x46);
            break;
        case SDE_NORMAL:
            CAMERA_IO_Write(g_slv_addr, SDE, 0x06);
            CAMERA_IO_Write(g_slv_addr, UFIX, 0x80);
            CAMERA_IO_Write(g_slv_addr, VFIX, 0x80);
            break;
        default:
            return -1;
    }

    return 0;
}

static int set_lens_correction(int enable, int radi, int coef);
static int set_lens_correction(int enable, int radi, int coef)
{

    CAMERA_IO_Write(g_slv_addr, LC_CTR, (enable&0x01));
    CAMERA_IO_Write(g_slv_addr, LC_RADI, radi);
    CAMERA_IO_Write(g_slv_addr, LC_COEF, coef);

    return 0;
}

CAMERA_DrvTypeDef ov7725_drv = 
{
  ov7725_Init,
  ov7725_ReadID,  
  ov7725_Config,
};



void ov7725_Init(uint16_t DeviceAddr, uint32_t resolution)
{
    g_slv_addr = DeviceAddr;
    reset();
    set_pixformat (PIXFORMAT_BAYER);
    set_framesize (resolution);
    
}

void     ov7725_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t BR_value)
{
}

uint16_t ov7725_ReadID(uint16_t DeviceAddr)
{
    CAMERA_IO_Read (DeviceAddr, PID);
}