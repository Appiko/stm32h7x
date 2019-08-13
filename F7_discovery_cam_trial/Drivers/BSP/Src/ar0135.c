#include "ar0135.h"
#include "stm32f7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
CAMERA_DrvTypeDef ar0135_drv = 
{
  ar0135_Init,
  ar0135_ReadID,  
  ar0135_Config,
};


static uint16_t AR320x240_EXT_TRIG[][2] = 
{
	{ROW_SPEED, 16},
	{PRE_PLL_CLK_DIV, 2},
	{PLL_MULTIPLIER, 47},
	{VT_SYS_CLK_DIV, 1 },
	{VT_PIX_CLK_DIV, 8},
	{DIGITAL_BINNING, 0},
	{DIGITAL_TEST, 0x0480},
	{RESET_REGISTER, 0x1990},
	{LINE_LENGTH_PCK, 1650},
	{Y_ADDR_START, 0},
	{X_ADDR_START, 0},
	{Y_ADDR_END, 240},
	{X_ADDR_END, 320},
	{FRAME_LENGTH_LINES, 990},
	{COARSE_INTEGRATION_TIME, 64},
	{FINE_INTEGRATION_TIME, 192},
	{Y_ODD_INC, 1},
	{Y_ADDR_START_CB, 0},
	{X_ADDR_START_CB, 0},
	{Y_ADDR_END_CB, 959},
	{X_ADDR_END_CB, 1279},
	{FRAME_LENGTH_LINES_CB, 990},
	{COARSE_INTEGRATION_TIME_CB, 100},
	{FINE_INTEGRATION_TIME_CB, 192},
	{Y_ODD_INC_CB, 1},
	{READ_MODE, 0},
	{EMBEDDED_DATA_CTRL, 6530},
	{HISPI_CONTROL_STATUS, 32768},
	{AE_CTRL_REG, 0x0000},
	{GLOBAL_GAIN, 0xF0},
	{GREEN1_GAIN, 48},
	{BLUE_GAIN, 66},
	{RED_GAIN, 66},
	{GREEN2_GAIN, 48},
	{AR0135_FLASH, 100},
	{PLL_MULTIPLIER, 0x003A}    
};

static uint16_t AR320x240_STREAM[][2] = 
{
	{ROW_SPEED, 16},
	{PRE_PLL_CLK_DIV, 2},
	{PLL_MULTIPLIER, 47},
	{VT_SYS_CLK_DIV, 1 },
	{VT_PIX_CLK_DIV, 8},
	{DIGITAL_BINNING, 0},
	{DIGITAL_TEST, 0x0480},
	/*{RESET_REGISTER, 0x1990},*/
	{LINE_LENGTH_PCK, 1650},
	{Y_ADDR_START, 0},
	{X_ADDR_START, 0},
	{Y_ADDR_END, 240},
	{X_ADDR_END, 320},
	{FRAME_LENGTH_LINES, 990},
	{COARSE_INTEGRATION_TIME, 64},
	{FINE_INTEGRATION_TIME, 192},
	{Y_ODD_INC, 1},
	{Y_ADDR_START_CB, 0},
	{X_ADDR_START_CB, 0},
	{Y_ADDR_END_CB, 959},
	{X_ADDR_END_CB, 1279},
	{FRAME_LENGTH_LINES_CB, 990},
	{COARSE_INTEGRATION_TIME_CB, 100},
	{FINE_INTEGRATION_TIME_CB, 192},
	{Y_ODD_INC_CB, 1},
	{READ_MODE, 0},
	{EMBEDDED_DATA_CTRL, 6530},
	{HISPI_CONTROL_STATUS, 32768},
	{AE_CTRL_REG, 0x0000},
	{GLOBAL_GAIN, 0xF0},
	{GREEN1_GAIN, 48},
	{BLUE_GAIN, 66},
	{RED_GAIN, 66},
	{GREEN2_GAIN, 48},
	{PLL_MULTIPLIER, 0x003A}    
};

static uint16_t ARRAW_EXT_TRIG[][2]=
{
	{ROW_SPEED, 16},
	{PRE_PLL_CLK_DIV, 2},
	{PLL_MULTIPLIER, 47},
	{VT_SYS_CLK_DIV, 1 },
	{VT_PIX_CLK_DIV, 8},
	{DIGITAL_BINNING, 0},
	{DIGITAL_TEST, 0x0480},
	{RESET_REGISTER, 0x1990},
	{LINE_LENGTH_PCK, 1650},
	{Y_ADDR_START, 0},
	{X_ADDR_START, 0},
	{Y_ADDR_END, 959},
	{X_ADDR_END, 1279},
	{FRAME_LENGTH_LINES, 990},
	{COARSE_INTEGRATION_TIME, 64},
	{FINE_INTEGRATION_TIME, 192},
	{Y_ODD_INC, 1},
	{Y_ADDR_START_CB, 0},
	{X_ADDR_START_CB, 0},
	{Y_ADDR_END_CB, 959},
	{X_ADDR_END_CB, 1279},
	{FRAME_LENGTH_LINES_CB, 990},
	{COARSE_INTEGRATION_TIME_CB, 100},
	{FINE_INTEGRATION_TIME_CB, 192},
	{Y_ODD_INC_CB, 1},
	{READ_MODE, 0},
	{EMBEDDED_DATA_CTRL, 6530},
	{HISPI_CONTROL_STATUS, 32768},
	{AE_CTRL_REG, 0x0000},
	{GLOBAL_GAIN, 0xF0},
	{GREEN1_GAIN, 48},
	{BLUE_GAIN, 66},
	{RED_GAIN, 66},
	{GREEN2_GAIN, 48},
	{AR0135_FLASH, 100},
	{PLL_MULTIPLIER, 0x003A}
};

static uint16_t AR360x240_GREENTEST[][2] = 
{
	/*{DIGITAL_TEST, 0x0480},*/
	{RESET_REGISTER, 0x10DC},
	{PRE_PLL_CLK_DIV, 2},
	{PLL_MULTIPLIER, 0x20},
	{VT_SYS_CLK_DIV, 1},
	{VT_PIX_CLK_DIV, 16},
	{DIGITAL_BINNING, 0},
	{Y_ADDR_START, 0},
	{X_ADDR_START, 0},
	{Y_ADDR_END, 240},
	{X_ADDR_END, 320},
	{FRAME_LENGTH_LINES, 240},
	{COARSE_INTEGRATION_TIME, 64},
	{FINE_INTEGRATION_TIME, 192},
	{Y_ODD_INC, 1},
	{Y_ADDR_START_CB, 0},
	{X_ADDR_START_CB, 0},
	{Y_ADDR_END_CB, 240},
	{X_ADDR_END_CB, 320},
	{TEST_DATA_GREENR, 0xFFF},
	{TEST_DATA_GREENB, 0},
	{TEST_DATA_RED, 0},
	{TEST_DATA_BLUE, 0},
	{TEST_PATTERN_MODE, 1}

};



void ar0135_Init(uint16_t DeviceAddr, uint32_t resolution)
{
    uint32_t index;
    /* Initialize I2C */
    CAMERA_IO_Init();   
  
    switch(resolution)
    {
        case CAMERA_R320x240 :
            for(index=0; index<(sizeof(AR360x240_GREENTEST)/4); index++)
            {
              CAMERA_IO_Write(DeviceAddr, AR360x240_GREENTEST[index][0], AR360x240_GREENTEST[index][1]);
              CAMERA_Delay(2);
            } 
            printf("Test %d, Gr %d R %d B %d Gb %d\n",
                   CAMERA_IO_Read (DeviceAddr, TEST_PATTERN_MODE),
                   CAMERA_IO_Read (DeviceAddr, TEST_DATA_GREENR),
                   CAMERA_IO_Read (DeviceAddr, TEST_DATA_RED),
                   CAMERA_IO_Read (DeviceAddr, TEST_DATA_BLUE),
                   CAMERA_IO_Read (DeviceAddr, TEST_DATA_GREENB));
            break;
        case CAMERA_RAW : 
            for(index=0; index<(sizeof(ARRAW_EXT_TRIG)/4); index++)
            {
              CAMERA_IO_Write(DeviceAddr, ARRAW_EXT_TRIG[index][0], ARRAW_EXT_TRIG[index][1]);
              CAMERA_Delay(2);
            } 
            break;
        break;
    }
   
}
void ar0135_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t BR_value)
{
}
uint16_t ar0135_ReadID(uint16_t DeviceAddr)
{
  /* Initialize I2C */
  CAMERA_IO_Init();
//  HAL_I2C_Init (&hi2c1);
  
  /* Get the camera ID */
  return (CAMERA_IO_Read(DeviceAddr, CHIP_VERSION_REG));
}
