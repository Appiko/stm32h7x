/*
 *  camera.h : <Write brief>
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


#ifndef CAMERA_H
#define CAMERA_H

#include "stdint.h"


#define CAMERA_R160x120                 0x00   /* QQVGA Resolution                     */
#define CAMERA_R320x240                 0x01   /* QVGA Resolution                      */
#define CAMERA_R480x272                 0x02   /* 480x272 Resolution                   */
#define CAMERA_R640x480                 0x03   /* VGA Resolution                       */  
#define CAMERA_RAW                      0x04   /* VGA Resolution                       */  

/** @defgroup CAMERA_Driver_structure  Camera Driver structure
  * @{
  */
typedef struct
{
  void     (*Init)(uint16_t, uint32_t);
  uint16_t (*ReadID)(uint16_t);  
  void     (*Config)(uint16_t, uint32_t, uint32_t, uint32_t);
}CAMERA_DrvTypeDef;

/**
  * @brief  Initializes Camera low level.
  * @retval None
  */
void     CAMERA_IO_Init(void);

/**
  * @brief  Camera delay 
  * @param  Delay: Delay in ms
  * @retval None
  */
void     CAMERA_Delay(uint32_t delay);


/**
  * @brief  Camera writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
void CAMERA_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);

/**
  * @brief  Camera reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
uint16_t CAMERA_IO_Read(uint8_t Addr, uint16_t Reg);



#endif /* CAMERA_H */

