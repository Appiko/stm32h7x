/*
 *  camera.c : <Write brief>
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
#include "camera.h"
#include "i2c.h"

#define CAM_8BIT

extern I2C_HandleTypeDef hi2c1;

void     CAMERA_IO_Init(void)
{
//    MX_I2C1_Init ();
}

void     CAMERA_Delay(uint32_t delay)
{
    HAL_Delay (delay);
}

#if defined CAM_16BIT
void CAMERA_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
    uint8_t val_buf[2];
    val_buf[0] = Value>>8;
    val_buf[1] = Value&0xFF;
    HAL_StatusTypeDef status = HAL_OK;
  
    status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT,
                               val_buf, 2, 1000);
    

}

uint16_t CAMERA_IO_Read(uint8_t Addr, uint16_t Reg)
{
    uint8_t read_value[2];
    uint16_t ret_value = 0;

    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Read(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT,
                              read_value, 2, 1000);
    ret_value = read_value[0]<<8|read_value[1];
    return ret_value;
}
#elif defined CAM_8BIT
void CAMERA_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
    uint8_t val_buf[2];
    val_buf[0] = Reg&0xFF;
    val_buf[1] = Value&0xFF;
    HAL_StatusTypeDef status = HAL_OK;
  
    status = HAL_I2C_Master_Transmit (&hi2c1, Addr, val_buf, 2, 1000);
    

}

uint16_t CAMERA_IO_Read(uint8_t Addr, uint16_t Reg)
{
    uint8_t read_value = 0;
//    uint16_t ret_value = 0;

    HAL_StatusTypeDef status = HAL_OK;
    HAL_I2C_Master_Transmit (&hi2c1, Addr, &Reg, 1, 100);
    HAL_I2C_Master_Receive (&hi2c1, Addr, &read_value, 1, 100);
    if(status)
    {
        printf("Status : %d Err : 0x%x %x\n", status,  hi2c1.ErrorCode,hi2c1.Mode);
    }
//    ret_value = read_value[0]&0xff;
//    return ret_value;
    return read_value;
}
#endif
