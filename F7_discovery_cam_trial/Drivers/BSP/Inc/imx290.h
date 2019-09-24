/*
 *  imx290.h : <Write brief>
 *  Copyright (C) 2019  Appiko
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#ifndef IMX290_H
#define IMX290_H

#include "camera.h"

#define VREVERSE_HREVERSE_WINMODE       0x3007
#define FRSEL_FDG_SEL                   0x3009
#define VMAX_0_7                        0x3018
#define VMAX_8_15                       0x3019
#define VMAX_16_17                      0x301A
#define CSI_LANE_MODE                   0x3443
#define EXTCK_FREQ_0_7                  0x3444
#define EXTCK_FREQ_8_15                 0x3445
#define PHYSICAL_LANE_NUM               0x3407
#define WINWV_OB                        0x303A
#define WINPH_0_7                       0x3040
#define WINPH_8_10                      0x3041
#define WINWH_0_7                       0x3042
#define WINWH_8_10                      0x3043
#define WINPV_0_7                       0x303C
#define WINPV_8_10                      0x303D
#define WINWV_0_7                       0x303E
#define WINWV_8_10                      0x303F
#define XVSOUTSEL_XHSOUTSEL             0x304B
#define IMX290_STANDBY                  0x3000
#define IMX290_REGHOLD                  0x3001
#define IMX290_XMSTA                    0x3002
#define IMX290_GAIN                     0x3014


#define IMX290_I2C_ADDR 0x34

/** @defgroup IMX290_Exported_Functions
  * @{
  */ 
void     imx290_Init(uint16_t DeviceAddr, uint32_t resolution);
void     imx290_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t BR_value);
uint16_t imx290_ReadID(uint16_t DeviceAddr);

void     CAMERA_IO_Init(void);
void     CAMERA_IO_Write(uint8_t addr, uint16_t reg, uint8_t value);
uint8_t  CAMERA_IO_Read(uint16_t addr, uint16_t reg);
void     CAMERA_Delay(uint32_t delay);

/* CAMERA driver structure */
extern CAMERA_DrvTypeDef   imx290_drv;
#endif /* IMX290_H */

