/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dcmi.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"
#include "ov9655.h"
//#include "ar0135.h"
#include "rk043fn48h.h"
#include "fonts.h"
//#include "font24.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PIX_H 1024
#define PIX_W 1280

uint8_t ppm_header[] = {'P','5','\n','1','2','8','0',' ','1','0','2','4','\n','2','5','5','\n'};

uint8_t CAMERA_Init(uint32_t );
static void LTDC_Init(uint32_t, uint16_t, uint16_t, uint16_t, uint16_t);
void LCD_GPIO_Init(LTDC_HandleTypeDef *, void *);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t _fb_base_lr[250][330];
extern uint8_t _fb_base_hr[PIX_W][PIX_H];
extern uint8_t _fb_base_ppm[PIX_W*3][PIX_H];
FIL fp;
//static uint8_t _fb_base_ppm[PIX_W*3];

typedef enum
{
CAMERA_OK = 0x00,
CAMERA_ERROR = 0x01,
CAMERA_TIMEOUT = 0x02,
CAMERA_NOT_DETECTED = 0x03,
CAMERA_NOT_SUPPORTED = 0x04
} Camera_StatusTypeDef;
typedef struct
{
uint32_t TextColor;
uint32_t BackColor;
sFONT *pFont;
}LCD_DrawPropTypeDef;
typedef struct
{
int16_t X;
int16_t Y;
}Point, * pPoint;
static LCD_DrawPropTypeDef DrawProp[2];
LTDC_HandleTypeDef hltdc;
LTDC_LayerCfgTypeDef
layer_cfg;
static RCC_PeriphCLKInitTypeDef periph_clk_init_struct;
CAMERA_DrvTypeDef *camera_driv;
/* Camera module I2C HW address */
static uint32_t CameraHwAddress;
/* Image size */
uint32_t Im_size = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint8_t img_bayer2rgb (uint8_t index);
uint8_t img_file_append (uint8_t * p_data, uint32_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_DCMI_Init();
  MX_FMC_Init();
//  MX_I2C1_Init();
  MX_I2C3_Init();
//  MX_LTDC_Init();
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_MspInit (&huart1);
//  HAL_DCMI_MspInit (&hdcmi);
//  HAL_I2C_MspInit (&hi2c1);
  printf("Hello world\n");
  printf("Macro : 0x%x, LR : 0x%x, HR : 0x%x\n",FRAME_BUFFER, _fb_base_lr, _fb_base_hr);
//  uint8_t i2c_addr = 00;
//  i2c_addr = cambus_scan ();  
//  printf("First I2C device found : 0x%x\n",i2c_addr);

  
  //  LTDC_Init((uint32_t)_fb_base_lr, 0, 0, 320, 240);
  LTDC_Init((uint32_t)_fb_base_lr, 80, 16, 400, 256);
  BSP_SDRAM_Init();
  Im_size = 0x9600;
//  memset (_fb_base_lr, 0xff, Im_size);
  CAMERA_Init(CAMERA_R320x240);
  HAL_Delay(100);
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)_fb_base_lr, Im_size);
//  CAMERA_Init(CAMERA_RAW);
//  HAL_Delay(100);
//  Im_size = 1336400;
//  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)_fb_base_hr, Im_size);
//  printf("DCMI : XCount %d XSize %d\n", hdcmi.XferCount, hdcmi.XferSize);
//  printf("DMA : Src 0x%x Dst0 0x%x Dst1 0x%x Len %d\n", hdcmi.DMA_Handle->Instance->PAR
//      , hdcmi.DMA_Handle->Instance->M0AR, hdcmi.DMA_Handle->Instance->M1AR
//      , hdcmi.DMA_Handle->Instance->NDTR);
//  while(hdcmi.State != HAL_DCMI_STATE_READY);
//  printf("DCMI : state %d, Error %d\n",hdcmi.State, hdcmi.ErrorCode);
//  printf("DMA : state %d, Error %d\n",hdcmi.DMA_Handle->State, hdcmi.DMA_Handle->ErrorCode);
//  while(hdcmi.DMA_Handle->State != HAL_DMA_STATE_READY);
//  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)_fb_base_lr, Im_size);
  
  
  
  if(BSP_SD_IsDetected ())
  {
      printf("SD Card detected\n");
      uint32_t status = 0;
      status = SD_initialize (0);
      if(status  == RES_OK)
      {
          printf("SD Card ready\n");
          HAL_Delay (100);
          status = f_mount (&SDFatFS, (const TCHAR*)SDPath, 1);
          if(status == FR_OK)
          {
              printf("Mount successful\n");
              printf("SD Type %d Speed %d clk_div %d\n", hsd1.SdCard.CardType, hsd1.SdCard.Class, hsd1.Init.ClockDiv);
              status = f_open (&fp, "RGB_Array.ppm", FA_CREATE_ALWAYS | FA_WRITE);
              if(status == FR_OK)
              {
                  printf("File open\n");
                  uint8_t byteswritten;
                  HAL_Delay (10);
//                  printf("Frame buff pointer : 0x%x\n", _fb_base_hr);
//                  status = f_write (&fp,ppm_header, sizeof(ppm_header), &byteswritten);
//                  HAL_Delay (10);
                  for(uint32_t i = 0; i < PIX_H; i++)
                  {
                      status = img_bayer2rgb (i);
                  }
                  status = f_write (&fp,(void *)_fb_base_ppm, sizeof(_fb_base_ppm), &byteswritten);
                  if(status == FR_OK)
                  {
                      printf("File write done\n");
                      status = f_close (&fp);
                      if(status == FR_OK)
                      {
                          printf("Safe to remove SD Card\n");
                      }
                      else{printf("Failed while appending : %d\n", status);}
                  }
                  else{printf("File write failed : %d\n", status);}
              }
              else {printf("File open failed : %d\n",status);}
          }
          else {printf("Mount Failed : %d\n", status);}
      }
      else {printf("SD Card not ready : %d\n", hsd1.State);}
  }
  else{printf("SD Card not present\n");}
  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS , (uint32_t)FRAME_BUFFER, Im_size);

  
  for(uint16_t i = 0; i < 24; i++)
  {
      for(uint32_t j=0; j<32;j++) printf("%d ", _fb_base_lr[i][j]);
      printf("\n");
  }
  printf("DCMI : state %d, Error %d\n",hdcmi.State, hdcmi.ErrorCode);
  printf("DMA : state %d, Error %d\n",hdcmi.DMA_Handle->State, hdcmi.DMA_Handle->ErrorCode);

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */


int cambus_scan()
{
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
        __disable_irq() ;
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 10, 100) == HAL_OK) {
            __enable_irq() ;
            return (addr << 1) ;
        }
        __enable_irq() ;
    }

    return 0;
}


void LCD_GPIO_Init(LTDC_HandleTypeDef *hltdc, void *Params)
{
    GPIO_InitTypeDef gpio_init_structure;
    /* Enable the LTDC and DMA2D clocks */
    __HAL_RCC_LTDC_CLK_ENABLE();
    __HAL_RCC_DMA2D_CLK_ENABLE();
    /* Enable GPIOs clock */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();
    /*** LTDC Pins configuration ***/
    /* GPIOE configuration */
    gpio_init_structure.Pin = GPIO_PIN_4;
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOE, &gpio_init_structure);
    /* GPIOG configuration */
    gpio_init_structure.Pin = GPIO_PIN_12;
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOG, &gpio_init_structure);
    /* GPIOI LTDC alternate configuration */
    gpio_init_structure.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_13 |
    GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOI, &gpio_init_structure);
    /* GPIOJ configuration */
    gpio_init_structure.Pin
    = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
    GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_5
    | GPIO_PIN_6 | GPIO_PIN_7 |GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
    GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init_structure.Mode
    = GPIO_MODE_AF_PP;
    gpio_init_structure.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOJ, &gpio_init_structure);
    /* GPIOK configuration */
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOK, &gpio_init_structure);
    /* LCD_DISP GPIO configuration */
    gpio_init_structure.Pin = GPIO_PIN_12;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    /* LCD_DISP pin has to be manually controlled */
    HAL_GPIO_Init(GPIOI, &gpio_init_structure);
    /* LCD_BL_CTRL GPIO configuration */
    /* LCD_BL_CTRL pin has to be manually controlled */
    gpio_init_structure.Pin = GPIO_PIN_3;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOK, &gpio_init_structure);
}

static void LTDC_Init(uint32_t FB_Address,
uint16_t Xpos, uint16_t Ypos,
uint16_t Width, uint16_t Height)
{
    /* Timing Configuration */
    hltdc.Init.HorizontalSync = (RK043FN48H_HSYNC - 1);
    hltdc.Init.VerticalSync = (RK043FN48H_VSYNC - 1);
    hltdc.Init.AccumulatedHBP = (RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
    hltdc.Init.AccumulatedVBP = (RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
    hltdc.Init.AccumulatedActiveH = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC +
    RK043FN48H_VBP - 1);
    hltdc.Init.AccumulatedActiveW = (RK043FN48H_WIDTH + RK043FN48H_HSYNC +
    RK043FN48H_HBP - 1);
    hltdc.Init.TotalHeigh = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC +
    RK043FN48H_VBP + RK043FN48H_VFP - 1);
    hltdc.Init.TotalWidth = (RK043FN48H_WIDTH + RK043FN48H_HSYNC +
    RK043FN48H_HBP + RK043FN48H_HFP - 1);
    /* LCD clock configuration */
    periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
    periph_clk_init_struct.PLLSAI.PLLSAIR = RK043FN48H_FREQUENCY_DIVIDER;
    periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
    HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
    /* Initialize the LCD pixel width and pixel height */
    hltdc.LayerCfg->ImageWidth
    = RK043FN48H_WIDTH;
    hltdc.LayerCfg->ImageHeight = RK043FN48H_HEIGHT;
    hltdc.Init.Backcolor.Blue = 0;/* Background value */
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;
    /* Polarity */
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Instance = LTDC;
    if(HAL_LTDC_GetState(&hltdc) == HAL_LTDC_STATE_RESET)
    {
        LCD_GPIO_Init(&hltdc, NULL);
    }
    HAL_LTDC_Init(&hltdc);
    /* Assert display enable LCD_DISP pin */
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);
    /* Assert backlight LCD_BL_CTRL pin */
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
    DrawProp[0].pFont = &Font24 ;
    /* Layer Init */
    layer_cfg.WindowX0 = Xpos;
    layer_cfg.WindowX1 = Width;
    layer_cfg.WindowY0 = Ypos;
    layer_cfg.WindowY1 = Height;
    layer_cfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    layer_cfg.FBStartAdress = FB_Address;
    layer_cfg.Alpha = 255;
    layer_cfg.Alpha0 = 0;
    layer_cfg.Backcolor.Blue = 0;
    layer_cfg.Backcolor.Green = 0;
    layer_cfg.Backcolor.Red = 0;
    layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    layer_cfg.ImageWidth = Width - Xpos;
    layer_cfg.ImageHeight = Height - Ypos;
    HAL_LTDC_ConfigLayer(&hltdc, &layer_cfg, 1);
    DrawProp[1].BackColor = ((uint32_t)0xFFFFFFFF);
    DrawProp[1].pFont = &Font24;
    DrawProp[1].TextColor = ((uint32_t)0xFF000000);
}

uint8_t CAMERA_Init(uint32_t Resolution) /*Camera initialization*/
{
//    memcpy ();
    uint8_t status = CAMERA_ERROR;
    CameraHwAddress = CAMERA_I2C_ADDRESS;
    /* Read ID of Camera module via I2C */
    if(ov9655_ReadID(CameraHwAddress) == OV9655_ID)
    {
        camera_driv = &ov9655_drv;/* Initialize the camera driver structure */
        {
            printf("Correct address.!\n");
            
            camera_driv->Init(CameraHwAddress, Resolution);
            HAL_DCMI_DisableCROP(&hdcmi);
        }
        status = CAMERA_OK; /* Return CAMERA_OK status */
    }
    else
    {
        status = CAMERA_NOT_SUPPORTED; /* Return CAMERA_NOT_SUPPORTED status */
    }
    return status;
}

uint8_t img_file_append (uint8_t * p_data, uint32_t len)
{
    static uint8_t res;
    static uint32_t byteswritten;
    res = f_write (&fp, p_data, len, (void *)byteswritten);
    return res;
    
}

uint8_t img_bayer2rgb (uint8_t index)
{
    //set last byte to '\n'
    //GBRG bayer format
    //if index = 0 || 479 (first and last row)
    if (index == 0 || index == (PIX_H-1))
    {
        //whole row zero
        memset (&_fb_base_ppm[0][index], 0, PIX_W*3);
    }
    //check if index is odd
    else if (index % 2 != 0)
    {
        //run for start from red
        for(uint32_t raw_i = 0, img_i = 0; raw_i < PIX_W; raw_i++, img_i += 3)
        {
            //0th byte || (PIX_W - 1) byte 
            if (raw_i == 0 || raw_i == (PIX_W - 1))
            {
                //all zero
                _fb_base_ppm[img_i][index] = 0;
                _fb_base_ppm[img_i + 1][index] = 0;
                _fb_base_ppm[img_i + 2][index] = 0;
            }
            //check if odd byte
            else if (raw_i % 2 != 0)
            {
                //red = avg of prev and next
                _fb_base_ppm[img_i][index] = (_fb_base_hr[raw_i - 1][index] +
                    _fb_base_hr[raw_i + 1][index])/2;
                //green = reading
                _fb_base_ppm[img_i + 1][index] = _fb_base_hr[raw_i][index];
                //blue = avg of top and bottom
                _fb_base_ppm[img_i + 2][index] = (_fb_base_hr[raw_i][index-1] + 
                    _fb_base_hr[raw_i][index+1])/2;
            }
            //if even 
            else
            {
                //red = reading
                _fb_base_ppm[img_i][index] = _fb_base_hr[raw_i][index];
                //green  = avg of top bottom sideways
                _fb_base_ppm[img_i + 1][index] = (_fb_base_hr[raw_i][index-1] + 
                    _fb_base_hr[raw_i][index+1] +
                    _fb_base_hr[raw_i - 1][index] +
                    _fb_base_hr[raw_i + 1][index])/4;
                //blue = avg of corners
                _fb_base_ppm[img_i + 2][index] = (_fb_base_hr[raw_i - 1][index-1] + 
                    _fb_base_hr[raw_i + 1][index-1] + 
                    _fb_base_hr[raw_i - 1][index+1] +
                    _fb_base_hr[raw_i + 1][index+1])/4;
            }
        }
    
    }
    //else index is even
    else
    {
        //run for start from blue
        for(uint32_t raw_i = 0, img_i = 0; raw_i < PIX_W; raw_i++, img_i += 3)
        {
            //0th byte || (PIX_W - 1)th byte:
            if (raw_i == 0 || raw_i == (PIX_W - 1))
            {
                //all zeros
                _fb_base_ppm[img_i][index] = 0;
                _fb_base_ppm[img_i+1][index] = 0;
                _fb_base_ppm[img_i+2][index] = 0;
            }
            //check if odd byte
            else if(raw_i%2 != 0)
            {
                //red = avg of corners
                _fb_base_ppm[img_i][index] = (_fb_base_hr[raw_i - 1][index-1] + 
                    _fb_base_hr[raw_i + 1][index-1] + 
                    _fb_base_hr[raw_i - 1][index+1] +
                    _fb_base_hr[raw_i + 1][index+1])/4;
                //green = avg of top bottom sideways
                _fb_base_ppm[img_i+1][index] = (_fb_base_hr[raw_i][index-1] + 
                    _fb_base_hr[raw_i][index+1] +
                    _fb_base_hr[raw_i - 1][index] +
                    _fb_base_hr[raw_i + 1][index])/4;
                //blue = reading
                _fb_base_ppm[img_i+2][index] = _fb_base_hr[raw_i][index];
            }
            //if even
            else
            {
                //red = avg of top and bottom
                _fb_base_ppm[img_i][index] = (_fb_base_hr[ raw_i][index-1] + 
                    _fb_base_hr[raw_i][index+1])/2;
                //green = reading
                _fb_base_ppm[img_i+1][index] = _fb_base_hr[raw_i][index];
                //blue = avg of prev and next
                _fb_base_ppm[img_i+2][index] = (_fb_base_hr[ raw_i - 1][index] +
                    _fb_base_hr[raw_i + 1][index])/2;
            }
        }
    }
    
    
    //append file
//    uint8_t status;
//     status = img_file_append (_fb_base_ppm, sizeof(_fb_base_ppm));
//     printf("%d : %d\n", index, status);
//     return status;
    return 0;
            
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    printf("!!...Error...!!\n");

    while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
