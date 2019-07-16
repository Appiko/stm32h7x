/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#include <stdbool.h>
#include "cambus.h"
#include "i2c.h"
#include "stm32h7xx_hal_i2c.h"
#include "string.h"

#define I2C_TIMEOUT 1000

I2C_HandleTypeDef I2CHandle = {};
int cambus_init()
{
    memcpy (&I2CHandle, i2c_get_i2c1_handle(), sizeof(I2C_HandleTypeDef));
    return 0;
}

int cambus_scan()
{
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
        __disable_irq();
        if (HAL_I2C_IsDeviceReady(i2c_get_i2c1_handle(), addr << 1, 10, I2C_TIMEOUT) == HAL_OK) {
            __enable_irq();
            return (addr << 1);
        }
        __enable_irq();
    }

    return 0;
}

int cambus_readb(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data)
{
    int ret = 0;

    __disable_irq();
    if((HAL_I2C_Master_Transmit(&I2CHandle, slv_addr, &reg_addr, 1, I2C_TIMEOUT) != HAL_OK)
    || (HAL_I2C_Master_Receive(&I2CHandle, slv_addr, reg_data, 1, I2C_TIMEOUT) != HAL_OK)) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)
{
    int ret=0;
    uint8_t buf[] = {reg_addr, reg_data};

    __disable_irq();
    if(HAL_I2C_Master_Transmit(&I2CHandle, slv_addr, buf, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_readw(uint8_t slv_addr, uint8_t reg_addr, uint16_t *reg_data)
{
    int ret=0;
    __disable_irq();
    if (HAL_I2C_Mem_Read(&I2CHandle, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_8BIT, (uint8_t*) reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    *reg_data = (*reg_data >> 8) | (*reg_data << 8);
    return ret;
}

int cambus_writew(uint8_t slv_addr, uint8_t reg_addr, uint16_t reg_data)
{
    int ret=0;
    reg_data = (reg_data >> 8) | (reg_data << 8);
    __disable_irq();
    if (HAL_I2C_Mem_Write(&I2CHandle, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_8BIT, (uint8_t*) &reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_readw2(uint8_t slv_addr, uint16_t reg_addr, uint16_t *reg_data)
{
    int ret=0;
    __disable_irq();
    if (HAL_I2C_Mem_Read(&I2CHandle, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_16BIT, (uint8_t*) reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    *reg_data = (*reg_data >> 8) | (*reg_data << 8);
    return ret;
}

int cambus_writew2(uint8_t slv_addr, uint16_t reg_addr, uint16_t reg_data)
{
    int ret=0;
    reg_data = (reg_data >> 8) | (reg_data << 8);
    __disable_irq();
    if (HAL_I2C_Mem_Write(&I2CHandle, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_16BIT, (uint8_t*) &reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}
