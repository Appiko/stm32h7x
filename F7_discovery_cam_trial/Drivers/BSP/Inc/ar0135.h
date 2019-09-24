#include "camera.h"

/*Register defines*/
#define CHIP_VERSION_REG  		0x3000  //default value is 1876	
#define Y_ADDR_START  			0x3002  //default value is 0
#define X_ADDR_START  			0x3004  //default value is 0
#define Y_ADDR_END  			0x3006  //default value is 959
#define X_ADDR_END  			0x3008  //default value is 1279
#define FRAME_LENGTH_LINES        	0x300A  //default value is 990		
#define LINE_LENGTH_PCK  		0x300C  //default value is 1388	
#define REVISION_NUMBER  		0x300E  //default value is 36	
#define LOCK_CONTROL  			0x3010  //default value is 48879
#define COARSE_INTEGRATION_TIME         0x3012  //default value is 100			
#define FINE_INTEGRATION_TIME           0x3014  //default value is 0		
#define COARSE_INTEGRATION_TIME_CB      0x3016  //default value is 16				
#define FINE_INTEGRATION_TIME_CB        0x3018  //default value is 0			
#define RESET_REGISTER  		0x301A  //default value is 4312	
#define DATA_PEDESTAL  			0x301E  //default value is 168
#define GPI_STATUS  			0x3026  //default value is 0
#define ROW_SPEED                       0x3028  //default value is 16
#define VT_PIX_CLK_DIV  		0x302A  //default value is 8	
#define VT_SYS_CLK_DIV  		0x302C  //default value is 1	
#define PRE_PLL_CLK_DIV  		0x302E  //default value is 2	
#define PLL_MULTIPLIER  		0x3030  //default value is 44	
#define DIGITAL_BINNING  		0x3032  //default value is 0	
#define FRAME_COUNT  			0x303A  //default value is 0
#define FRAME_STATUS  			0x303C  //default value is 0
#define READ_MODE                       0x3040  //default value is 0
#define DARK_CONTROL  			0x3044  //default value is 1028
#define AR0135_FLASH			0x3046  //default value is 0
#define GREEN1_GAIN  			0x3056  //default value is 32
#define BLUE_GAIN  			0x3058  //default value is 32
#define RED_GAIN  			0x305A  //default value is 32
#define GREEN2_GAIN  			0x305C  //default value is 32
#define GLOBAL_GAIN  			0x305E  //default value is 32
#define EMBEDDED_DATA_CTRL              0x3064  //default value is 6530		
#define DATAPATH_SELECT  		0x306E  //default value is 36864	
#define TEST_PATTERN_MODE  		0x3070  //default value is 0	
#define TEST_DATA_RED  			0x3072  //default value is 0
#define TEST_DATA_GREENR  		0x3074  //default value is 0	
#define TEST_DATA_BLUE  		0x3076  //default value is 0	
#define TEST_DATA_GREENB  		0x3078  //default value is 0	
#define TEST_RAW_MODE  			0x307A  //default value is 0
#define SEQ_DATA_PORT  			0x3086  //default value is 0
#define SEQ_CTRL_PORT  			0x3088  //default value is 49152
#define X_ADDR_START_CB  		0x308A  //default value is 2	
#define Y_ADDR_START_CB  		0x308C  //default value is 4	
#define X_ADDR_END_CB  			0x308E  //default value is 1281
#define Y_ADDR_END_CB  			0x3090  //default value is 963
#define X_EVEN_INC  			0x30A0  //default value is 1
#define X_ODD_INC  			0x30A2  //default value is 1
#define Y_EVEN_INC  			0x30A4  //default value is 1
#define Y_ODD_INC  			0x30A6  //default value is 1
#define Y_ODD_INC_CB  			0x30A8  //default value is 63
#define FRAME_LENGTH_LINES_CB           0x30AA  //default value is 90		
#define FRAME_EXPOSURE  		0x30AC  //default value is 16	
#define DIGITAL_TEST  			0x30B0  //default value is 1152
#define TEMPSENS_DATA  			0x30B2  //default value is 0
#define TEMPSENS_CTRL  			0x30B4  //default value is 0
#define GREEN1_GAIN_CB  		0x30BC  //default value is 32	
#define BLUE_GAIN_CB  			0x30BE  //default value is 32
#define RED_GAIN_CB  			0x30C0  //default value is 32
#define GREEN2_GAIN_CB  		0x30C2  //default value is 32	
#define GLOBAL_GAIN_CB  		0x30C4  //default value is 32	
#define TEMPSENS_CALIB1  		0x30C6  //default value is 0	
#define TEMPSENS_CALIB2  		0x30C8  //default value is 0	
#define TEMPSENS_CALIB3  		0x30CA  //default value is 0	
#define TEMPSENS_CALIB4  		0x30CC  //default value is 0	
#define COLUMN_CORRECTION  		0x30D4  //default value is 57351	
#define AE_CTRL_REG  			0x3100  //default value is 0
#define AE_LUMA_TARGET_REG              0x3102  //default value is 1280		
#define AE_MIN_EV_STEP_REG              0x3108  //default value is 112		
#define AE_MAX_EV_STEP_REG              0x310A  //default value is 8		
#define AE_DAMP_OFFSET_REG              0x310C  //default value is 512		
#define AE_DAMP_GAIN_REG  		0x310E  //default value is 8192	
#define AE_DAMP_MAX_REG  		0x3110  //default value is 320	
#define AE_MAX_EXPOSURE_REG             0x311C  //default value is 672		
#define AE_MIN_EXPOSURE_REG             0x311E  //default value is 1		
#define AE_DARK_CUR_THRESH_REG          0x3124  //default value is 32767			
#define AE_CURRENT_GAINS  		0x312A  //default value is 32	
#define AE_ROI_X_START_OFFSET           0x3140  //default value is 0		
#define AE_ROI_Y_START_OFFSET           0x3142  //default value is 0		
#define AE_ROI_X_SIZE  			0x3144  //default value is 1280
#define AE_ROI_Y_SIZE  			0x3146  //default value is 960
#define AE_MEAN_L  			0x3152  //default value is 0
#define AE_COARSE_INTEGRATION_TIME      0x3164  //default value is 0				
#define AE_AG_EXPOSURE_HI  		0x3166  //default value is 986	
#define AE_AG_EXPOSURE_LO  		0x3168  //default value is 419	
#define DELTA_DK_LEVEL  		0x3188  //default value is 0	
#define HISPI_TIMING  			0x31C0  //default value is 0
#define HISPI_CONTROL_STATUS            0x31C6  //default value is 32768		
#define HISPI_CRC_0  			0x31C8  //default value is 65535
#define HISPI_CRC_1  			0x31CA  //default value is 65535
#define HISPI_CRC_2  			0x31CC  //default value is 65535
#define HISPI_CRC_3  			0x31CE  //default value is 65535
#define STAT_FRAME_ID  			0x31D2  //default value is 0
#define I2C_WRT_CHECKSUM  		0x31D6  //default value is 65535	
#define HORIZONTAL_CURSOR_POSITION      0x31E8  //default value is 0				
#define VERTICAL_CURSOR_POSITION        0x31EA  //default value is 0			
#define HORIZONTAL_CURSOR_WIDTH         0x31EC  //default value is 0			
#define VERTICAL_CURSOR_WIDTH           0x31EE  //default value is 0		
#define I2C_IDS  			0x31FC  //default value is 12320

/* The default slave addresses used by the AR0135AT ar 0x20 (write address)
 * and 0x21 (read address) in accordance with the specification.
 */
#define AR0135_ID 1876

#define AR0135_I2C_ADDR 0x20

/// 1 <= N <= 63
#define AR0135_PLL_N    2

/// 32 <= M <= 255 
#define AR0135_PLL_M    40

/// 1 <= P1 <= 16 
#define AR0135_PLL_P1   10

/// 4 <= P2 <= 16 
#define AR0135_PLL_P2   5


#define AR0135_LINE_LEN 1388

#define AR0135_FRAME_LEN 960+23  

#define CAM_FREQ 25

#define CAM_PCLK ((CAM_FREQ * AR0135_PLL_M)/(AR0135_PLL_N * AR0135_PLL_P1 * AR0135_PLL_P2))

#define ROW_TIME AR0135_LINE_LEN/CAM_PCLK

/** @defgroup OV9655_Exported_Functions
  * @{
  */ 
void     ar0135_Init(uint16_t DeviceAddr, uint32_t resolution);
void     ar0135_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t BR_value);
uint16_t ar0135_ReadID(uint16_t DeviceAddr);

void     CAMERA_IO_Init(void);
void     CAMERA_IO_Write(uint8_t addr, uint16_t reg, uint16_t value);
uint8_t  CAMERA_IO_Read(uint16_t addr, uint16_t reg);
void     CAMERA_Delay(uint32_t delay);

/* CAMERA driver structure */
extern CAMERA_DrvTypeDef   ar0135_drv;
/**
  * @}
  */    

