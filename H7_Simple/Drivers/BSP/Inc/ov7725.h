/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7725 driver.
 *
 */
#ifndef __OV7725_H__
#define __OV7725_H__
#include "camera.h"

#define OMV_ARCH_STR            "OMV3 F7 512" // 33 chars max
#define OMV_BOARD_TYPE          "M7"
#define OMV_UNIQUE_ID_ADDR      0x1FF0F420

// Flash sectors for the bootloader.
// Flash FS sector, main FW sector, max sector.
#define OMV_FLASH_LAYOUT        {1, 4, 11}

#define OMV_XCLK_MCO            (0U)
#define OMV_XCLK_TIM            (1U)

// Sensor external clock source.
#define OMV_XCLK_SOURCE         (OMV_XCLK_TIM)

// Sensor external clock timer frequency.
#define OMV_XCLK_FREQUENCY      (9000000)

// Sensor PLL register value.
#define OMV_OV7725_PLL_CONFIG   (0x81)  // x6

// Sensor Banding Filter Value
#define OMV_OV7725_BANDING      (0x8F)

typedef enum {
    PIXFORMAT_INVALID = 0,
    PIXFORMAT_BINARY,    // 1BPP/BINARY
    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    PIXFORMAT_RGB565,    // 2BPP/RGB565
    PIXFORMAT_YUV422,    // 2BPP/YUV422
    PIXFORMAT_BAYER,     // 1BPP/RAW
    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
} pixformat_t;

typedef enum {
    FRAMESIZE_INVALID = 0,
    // C/SIF Resolutions
    FRAMESIZE_QQCIF,    // 88x72
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_CIF,      // 352x288
    FRAMESIZE_QQSIF,    // 88x60
    FRAMESIZE_QSIF,     // 176x120
    FRAMESIZE_SIF,      // 352x240
    // VGA Resolutions
    FRAMESIZE_QQQQVGA,  // 40x30
    FRAMESIZE_QQQVGA,   // 80x60
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_HQQQVGA,  // 60x40
    FRAMESIZE_HQQVGA,   // 120x80
    FRAMESIZE_HQVGA,    // 240x160
    // FFT Resolutions
    FRAMESIZE_64X32,    // 64x32
    FRAMESIZE_64X64,    // 64x64
    FRAMESIZE_128X64,   // 128x64
    FRAMESIZE_128X128,  // 128x128
    // Other
    FRAMESIZE_LCD,      // 128x160
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_WVGA,     // 720x480
    FRAMESIZE_WVGA2,    // 752x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
} framesize_t;

typedef enum {
    FRAMERATE_2FPS =0x9F,
    FRAMERATE_8FPS =0x87,
    FRAMERATE_15FPS=0x83,
    FRAMERATE_30FPS=0x81,
    FRAMERATE_60FPS=0x80,
} framerate_t;

typedef enum {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
    GAINCEILING_32X,
    GAINCEILING_64X,
    GAINCEILING_128X,
} gainceiling_t;

typedef enum {
    SDE_NORMAL,
    SDE_NEGATIVE,
} sde_t;

typedef enum {
    ATTR_CONTRAST=0,
    ATTR_BRIGHTNESS,
    ATTR_SATURATION,
    ATTR_GAINCEILING,
} sensor_attr_t;

typedef enum {
    ACTIVE_LOW,
    ACTIVE_HIGH
} polarity_t;

#define OV7725_ID 1876

#define OV7725_I2C_ADDR 0x42

/// 1 <= N <= 63
#define OV7725_PLL_N    2

/// 32 <= M <= 255 
#define OV7725_PLL_M    40

/// 1 <= P1 <= 16 
#define OV7725_PLL_P1   10

/// 4 <= P2 <= 16 
#define OV7725_PLL_P2   5


#define OV7725_LINE_LEN 1388

#define OV7725_FRAME_LEN 960+23  

#define CAM_FREQ 25

#define CAM_PCLK ((CAM_FREQ * OV7725_PLL_M)/(OV7725_PLL_N * OV7725_PLL_P1 * OV7725_PLL_P2))

#define ROW_TIME OV7725_LINE_LEN/CAM_PCLK

/** @defgroup OV9655_Exported_Functions
  * @{
  */ 
void     ov7725_Init(uint16_t DeviceAddr, uint32_t resolution);
void     ov7725_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t BR_value);
uint16_t ov7725_ReadID(uint16_t DeviceAddr);

void     CAMERA_IO_Init(void);
//void     CAMERA_IO_Write(uint8_t addr, uint16_t reg, uint16_t value);
//uint8_t  CAMERA_IO_Read(uint8_t addr, uint16_t reg);
void     CAMERA_Delay(uint32_t delay);

/* CAMERA driver structure */
extern CAMERA_DrvTypeDef   ov7725_drv;
#endif // __OV7725_H__
