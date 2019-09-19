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
	{DIGITAL_TEST, 0x04B0},
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
	{RESET_REGISTER, 0x1D8},    //0
	{DIGITAL_TEST, 0x04B0},     //1
	{PRE_PLL_CLK_DIV, AR0135_PLL_N},       //2
	{PLL_MULTIPLIER, AR0135_PLL_M},       //4
	{VT_SYS_CLK_DIV, AR0135_PLL_P1},        //5
	{VT_PIX_CLK_DIV, AR0135_PLL_P2},        //6
	{DIGITAL_BINNING, 0},       //7
	{ROW_SPEED, 16},             //8
	{Y_ADDR_START, 0},          //9
	{X_ADDR_START, 0},          //10
	{Y_ADDR_END, 959},          //11
	{X_ADDR_END, 1279},         //12
	{FRAME_LENGTH_LINES, 960+23},//13
	{LINE_LENGTH_PCK, 1280+108},//14
//	{COARSE_INTEGRATION_TIME, 25},//15
//	{FINE_INTEGRATION_TIME, 0}, //16
	{Y_ODD_INC, 1},             //17
	{Y_ADDR_START_CB, 0},       //18
	{X_ADDR_START_CB, 0},       //19
	{Y_ADDR_END_CB, 240},       //20
	{X_ADDR_END_CB, 320},       //21
	{GREEN1_GAIN, 24},          //23
	{BLUE_GAIN, 30},            //24
	{RED_GAIN, 30},             //25
	{GREEN2_GAIN, 24},          //26
//	{GLOBAL_GAIN, 0xF0},        //22
	{TEST_DATA_GREENR, 0x020},      //27
	{TEST_DATA_GREENB, 0x000},      //28
	{TEST_DATA_RED, 0x000},         //29
	{TEST_DATA_BLUE, 0x000},    //30
	{TEST_PATTERN_MODE, 0},     //31
	{AR0135_FLASH, 0x100},      //32
};

static uint16_t AR360x240_GREENTEST[][2] = 
{
	{RESET_REGISTER, 0x1D8},
	{DIGITAL_TEST, 0x04A0},
	{PRE_PLL_CLK_DIV, 2},
	{PLL_MULTIPLIER, 40},
	{VT_SYS_CLK_DIV, 5},
	{VT_PIX_CLK_DIV, 2},
	{DIGITAL_BINNING, 0},
	{ROW_SPEED, 0},
	{Y_ADDR_START, 0},
	{X_ADDR_START, 0},
	{Y_ADDR_END, 239},
	{X_ADDR_END, 319},
	{FRAME_LENGTH_LINES, 330},
	{COARSE_INTEGRATION_TIME, 64},
	{FINE_INTEGRATION_TIME, 192},
	{Y_ODD_INC, 1},
	{Y_ADDR_START_CB, 0},
	{X_ADDR_START_CB, 0},
	{Y_ADDR_END_CB, 240},
	{X_ADDR_END_CB, 320},
	{TEST_DATA_GREENR, 0},
	{TEST_DATA_GREENB, 0},
	{TEST_DATA_RED, 0},
	{TEST_DATA_BLUE, 0xFFF},
	{TEST_PATTERN_MODE, 0},
	{GLOBAL_GAIN, 0xF0},
	{GREEN1_GAIN, 48},
	{BLUE_GAIN, 66},
	{RED_GAIN, 66},
	{GREEN2_GAIN, 48},

};



void ar0135_Init(uint16_t DeviceAddr, uint32_t resolution)
{
    uint32_t index;
    /* Initialize I2C */
    CAMERA_IO_Init();   
//    CAMERA_IO_Write (DeviceAddr, RESET_REGISTER, 0x01);
    
    
  
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
void ar0135_Config(uint16_t DeviceAddr, uint32_t reg, uint32_t value, uint32_t Unused)
{
    CAMERA_IO_Write (DeviceAddr, reg, value);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
uint16_t ar0135_ReadID(uint16_t DeviceAddr)
{
  /* Initialize I2C */
  CAMERA_IO_Init();
//  HAL_I2C_Init (&hi2c1);
  
  /* Get the camera ID */
  return (CAMERA_IO_Read(DeviceAddr, CHIP_VERSION_REG));
}
