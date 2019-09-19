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
#include <stdio.h>
#include <string.h>

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
#include "stdbool.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"
//#include "ov9655.h"
#include "ar0135.h"
#include "rk043fn48h.h"
#include "fonts.h"
//#include "font24.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PIX_H 960
#define PIX_W 1280


#define FILE_OPER_MAX_TRY 3

#define FLASH_TICKS 600

#define SHOT_TICKS 1250


#define CAM_FREQ 25

#define CAM_PCLK ((CAM_FREQ * AR0135_PLL_M)/(AR0135_PLL_N * AR0135_PLL_P1 * AR0135_PLL_P2))

#define ROW_TIME AR0135_LINE_LEN/CAM_PCLK
typedef struct 
{
    uint8_t b;
    uint8_t g;
    uint8_t r;
}RGB_data_t;

uint8_t ppm_header[] = {'P','6','\n','1','2','8','0',' ','9','6','0','\n','2','5','5','\n'};

uint8_t CAMERA_Init(uint32_t );
static void LTDC_Init(uint32_t, uint16_t, uint16_t, uint16_t, uint16_t);
void LCD_GPIO_Init(LTDC_HandleTypeDef *, void *);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//extern uint8_t _fb_base_lr[240][320];
//extern RGB_data_t _fb_base_lcd[240][320];
extern RGB_data_t _fb_base_rgb[240][320];
extern uint8_t _fb_base_hr[PIX_H ][PIX_W];
//extern RGB_data_t _fb_base_ppm[PIX_H][PIX_W];
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

/* Expo us */
static uint32_t g_expo_us;

static uint32_t g_file_no;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint8_t img_bayer2rgb (uint32_t index);
uint8_t img_compress_16 (uint32_t index);
uint8_t img_compress_rgb (void);
uint8_t lcd_bayer2rgb (uint32_t index);
uint8_t img_file_append (uint8_t * p_data, uint32_t len);


void cam_init (uint32_t resolution);

void cam_shot_start (uint32_t not_used);

void cam_shot_stop (uint32_t not_used);

void cam_flash_start (uint32_t not_used);

void cam_flash_stop (uint32_t not_used);

void cam_save (uint32_t file_no);

void get_file_no ();

void ( * cam_func[CAM_INVALID]) (uint32_t) = {cam_init, cam_shot_start, cam_shot_stop, 
cam_flash_start, cam_flash_stop, cam_save};

void sd_init (void);

void trig_init ();


volatile cam_state_t cam_state = CAM_INVALID;

volatile uint32_t cam_param = 0;

//temp

volatile uint8_t cam_state_change = 0;
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
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_MspInit (&huart1);
//  HAL_DCMI_MspInit (&hdcmi);
//  HAL_I2C_MspInit (&hi2c1);
  printf("Hello world\n");
  printf("Macro : 0x%x, LCD : 0x%x, HR : 0x%x\n",FRAME_BUFFER, _fb_base_rgb, _fb_base_hr);
//  uint8_t i2c_addr = 00;
//  i2c_addr = cambus_scan ();  
//  printf("First I2C device found : 0x%x\n",i2c_addr);

  
//    LTDC_Init((uint32_t)_fb_base_rgb, 0, 0, 320, 240);
  LTDC_Init((uint32_t)_fb_base_rgb, 80, 16, 400, 255);
  BSP_SDRAM_Init();
//  HAL_GPIO_TogglePin (ARDUINO_A0_GPIO_Port, ARDUINO_A0_Pin);
  sd_init();
  get_file_no ();
  trig_init ();

  Im_size = (1280*960)/4;
  memset (_fb_base_hr, 0, Im_size*4);
  memset (_fb_base_rgb, 0, 320*240);
  CAMERA_Init(CAMERA_RAW);
  HAL_Delay(100);
  cam_state = CAM_INIT;
  cam_param = 15;
  cam_state_change = true;
//  printf("DMA : Src 0x%x Dst0 0x%x Dst1 0x%x Len %d, Im_size %d\n", hdcmi.DMA_Handle->Instance->PAR
//      , hdcmi.DMA_Handle->Instance->M0AR, hdcmi.DMA_Handle->Instance->M1AR
//      , hdcmi.DMA_Handle->Instance->NDTR, Im_size);
//  while(hdcmi.State != HAL_DCMI_STATE_READY);


//  HAL_DCMI_Stop (&hdcmi);
//  
//  Im_size = (320*240)/4;
//  memset (_fb_base_lr, 0, Im_size*4);
//  CAMERA_Init(CAMERA_R320x240);
//  HAL_Delay(100);
//  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)_fb_base_lr, Im_size);
//  HAL_GPIO_WritePin (ARDUINO_A2_GPIO_Port, ARDUINO_A2_Pin, 1);
//  HAL_Delay (1);
//  HAL_GPIO_WritePin (ARDUINO_A2_GPIO_Port, ARDUINO_A2_Pin, 0);
//  HAL_Delay(1500);
//
//  printf("DMA : Src 0x%x Dst0 0x%x Dst1 0x%x Len %d, Im_size %d\n", hdcmi.DMA_Handle->Instance->PAR
//      , hdcmi.DMA_Handle->Instance->M0AR, hdcmi.DMA_Handle->Instance->M1AR
//      , hdcmi.DMA_Handle->Instance->NDTR, Im_size);
  
//  while(hdcmi.DMA_Handle->Instance->NDTR);
//  printf("DCMI : XCount %d XSize %d\n", hdcmi.XferCount, hdcmi.XferSize);
//  printf("DCMI : state %d, Error %d\n",hdcmi.State, hdcmi.ErrorCode);
//  printf("DMA : state %d, Error %d\n",hdcmi.DMA_Handle->State, hdcmi.DMA_Handle->ErrorCode);
//  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)_fb_base_lr, Im_size);

    
  
  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS , (uint32_t)FRAME_BUFFER, Im_size);
/*
  printf("\nLow Res : ");
  printf("Bayer\n");
  for(uint16_t i = 0; i < 10; i++)
  {
      for(uint32_t j=0; j<15;j++) printf("%d ", _fb_base_hr[i][j]);
      printf("\n");
  }

  printf("\nRGB\n");
  for(i = 0; i < 10; i++)
  {
      for(j=0; j<5; j++)
      {
        printf("%d ", _fb_base_lcd[i][j].r);
        printf("%d ", _fb_base_lcd[i][j].g);
        printf("%d ", _fb_base_lcd[i][j].b);
        printf("   ");
      }
      printf("\n");
  }

  printf("\nHigh Res : ");
  printf("Bayer\n");
  for(uint16_t i = 950; i < 960; i++)
  {
      for(uint32_t j=1275; j<1280;j++) printf("%d ", _fb_base_hr[i][j]);
      printf("\n");
  }

  printf("\nRGB\n");
  for(i = 950; i < 960; i++)
  {
      for(j=1275; j<1280; j++)
      {
        printf("%d ", _fb_base_ppm[i][j].r);
        printf("%d ", _fb_base_ppm[i][j].g);
        printf("%d ", _fb_base_ppm[i][j].b);
        printf("   ");
      }
      printf("\n");
  }

 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
      
      if(cam_state_change)
      {
        cam_state_change = false;
        cam_func[cam_state] (cam_param);
      }
      __WFI();
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
  RCC_OscInitStruct.PLL.PLLN = 432;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
void HAL_DCMI_VsyncEventCallback (DCMI_HandleTypeDef* dcmiHandle)
{
    cam_state = CAM_SAVE;
    cam_state_change = 1;
    cam_param = ++g_file_no;
//    dcmi_done ();
    printf("%s\n", __func__);
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM1)
    {
        switch(htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1 :
                cam_state = CAM_SHOT_END;
                cam_state_change = true;
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2 :
                cam_state = CAM_FLASH_END;
                cam_state_change = true;
                break;
                
        }
    }
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case ARDUINO_A3_Pin : 
        case USER_BUTTON_Pin : 
            {
                cam_state = CAM_SHOT_START;
                cam_state_change = true;
                cam_param = 0;
                break;
            }
        case ARDUINO_A1_Pin : 
            {
                cam_state = CAM_FLASH_START;
                cam_state_change = true;
                cam_param = 95;
                break;
            }
    }
}

void trig_init ()
{
    static GPIO_InitTypeDef GPIO_InitStruct = {0};

    __disable_irq() ;

    HAL_GPIO_DeInit(ARDUINO_A3_GPIO_Port, ARDUINO_A3_Pin);
    HAL_GPIO_DeInit(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    HAL_GPIO_DeInit(ARDUINO_A1_GPIO_Port, ARDUINO_A1_Pin);
  
    /*Configure GPIO pin : PtPin */
    __HAL_GPIO_EXTI_CLEAR_IT( ARDUINO_A1_Pin);
    
    GPIO_InitStruct.Pin = ARDUINO_A1_Pin; //Flash
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ARDUINO_A1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USER_BUTTON_Pin; //User button
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ARDUINO_A3_Pin; // SenseDev
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ARDUINO_A3_GPIO_Port, &GPIO_InitStruct);

    
    __enable_irq() ;

//    CAMERA_Init(CAMERA_RAW);

}

void sd_init (void)
{
    uint32_t status;
  if(BSP_SD_IsDetected ())
  {
      printf("SD Card detected\n");
      status = SD_initialize (0);
      if(status  == RES_OK)
      {
          printf("SD Card ready\n");
          HAL_Delay (100);
          status = f_mount (&SDFatFS, (const TCHAR*)SDPath, 1);
          if(status == FR_OK)
          {
              printf("Mount successful\n");
    
          }
          else {printf("Mount Failed : %d\n", status);}
      }
      else {printf("SD Card not ready : %d\n", hsd1.State);}
  }
  else{printf("SD Card not present\n");}
}

void write_sd_card (uint32_t file_no)
{
//    while(hdcmi.State != HAL_DCMI_STATE_READY);
    static attempts_remain = FILE_OPER_MAX_TRY;
    uint32_t status = 0;
    uint8_t file_name[] = {'x','x','x','x','.','r','g','b', '\0'};
    
    file_name[0] = (file_no/1000) + 0x30;
    file_name[1] = ((file_no%1000)/100) + 0x30;
    file_name[2] = ((file_no%100)/10) + 0x30;
    file_name[3] = (file_no%10) + 0x30;
    
    printf("Attempt : %d\n", FILE_OPER_MAX_TRY - attempts_remain + 1);
    printf ("upcoming file name : %x %x %x %x\n", file_name[0],file_name[1],file_name[2],file_name[3]);
    if(attempts_remain > 0)
    {
        status = f_open (&fp, (char *)file_name, FA_CREATE_ALWAYS | FA_WRITE);
    //    status = f_open (&fp, "0020.rgb", FA_CREATE_ALWAYS | FA_WRITE);
        if(status == FR_OK)
        {
            uint8_t byteswritten;
            printf("File open\n");
            status = f_write (&fp,(void *)_fb_base_hr, sizeof(_fb_base_hr), &byteswritten);
            if(status == FR_OK)
            {
                status = f_close (&fp);
                if(status == FR_OK)
                {
                    printf("Safe to remove SD Card\n");
                    attempts_remain = FILE_OPER_MAX_TRY;
                    return status;
                }
                else
                {
                    printf("Failed while appending : %d\n", status);
                    f_close (&fp);
                    attempts_remain--;
                    write_sd_card (file_no);
                }
            }
            else
            {
                printf("File write failed : %d\n", status);
                f_close (&fp);
                attempts_remain--;
                write_sd_card (file_no);
            }
        }
        else
        {
            printf("File open failed : %d\n",status);
            f_close (&fp); 
            attempts_remain--;
            write_sd_card (file_no);
        }
    }
    else 
    {
        printf("All attempts failed\n");
        g_file_no--;
        attempts_remain = FILE_OPER_MAX_TRY;
    }

}

uint32_t file_name_to_no (uint8_t * f_name)
{
    uint32_t file_no = 0;
    file_no = ((f_name[0] > 0x29) && (f_name[0] < 0x3A)) ?
        (((f_name[0] - 0x30) * 1000) + 
        ((f_name[1] - 0x30) * 100) +
        ((f_name[2] - 0x30) * 10) +
        ((f_name[3] - 0x30) * 1)) :
        g_file_no;
    return (file_no < g_file_no) ? g_file_no : file_no;
}

void get_file_no ()
{
    printf("%s\n", __func__);
    uint32_t status;
    uint32_t no_bytes;
    uint8_t file_buff[4];
    FILINFO file_info;
    DIR dp;
    status = f_opendir (&dp, "/");
    if(status == FR_OK)
    {
        do
        {
            status = f_readdir (&dp, &file_info);
            if(status == FR_OK)
            {
                
                g_file_no = file_name_to_no ((uint8_t *)file_info.fname); 
                
                printf("%s\n",file_info.fname);
            }else{printf("Read dir failed : %d\n",status);}
            
        }while(file_info.fname[0] != '\0');
        printf("File Number : %d\n", g_file_no);
        status = f_closedir (&dp);
        if(status == FR_OK)
        {
            printf("DIR Close\n");
        }else{printf("Close DIR failed : 0x%x\n", status);}
    }else{printf("DIR open failed : 0x%x\n", status);}
}

void cam_init (uint32_t expo_ms)
{
    printf("%s\n", __func__);
    uint16_t rows;
    uint32_t row_time = ROW_TIME;
    g_expo_us = expo_ms * 1000;
    rows = (g_expo_us/row_time); // line_length(1388)/PCLK(50MHz)
    CAMERA_IO_Write (CameraHwAddress, COARSE_INTEGRATION_TIME, rows);
//    CAMERA_Init (resolution);
//    trig_init ();
}

void cam_shot_start (uint32_t not_used)
{
    printf("%s\n", __func__);
    static GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pin : PtPin */
    __disable_irq() ;

    __HAL_GPIO_EXTI_CLEAR_IT(USER_BUTTON_Pin | ARDUINO_A3_Pin);
    
    HAL_GPIO_DeInit(ARDUINO_A3_GPIO_Port, ARDUINO_A3_Pin);
    HAL_GPIO_DeInit(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    HAL_GPIO_DeInit(ARDUINO_A1_GPIO_Port, ARDUINO_A1_Pin);

    GPIO_InitStruct.Pin = USER_BUTTON_Pin;  // User Button
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; // With button
    HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ARDUINO_A3_Pin;  // SenseDev
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // With SenseDev
    HAL_GPIO_Init(ARDUINO_A3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ARDUINO_A1_Pin;  // Flash
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ARDUINO_A1_GPIO_Port, &GPIO_InitStruct);

    
    
    __enable_irq() ;
    htim1.Instance->CCR1 = htim1.Instance->CNT + SHOT_TICKS;
    HAL_TIM_OC_Start_IT (&htim1, TIM_CHANNEL_1);
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)_fb_base_hr, Im_size);
    HAL_GPIO_WritePin (ARDUINO_A2_GPIO_Port, ARDUINO_A2_Pin, 1);

}

void cam_shot_stop (uint32_t not_used)
{
    printf("%s\n", __func__);
    HAL_TIM_OC_Stop_IT (&htim1, TIM_CHANNEL_1);
    HAL_GPIO_WritePin (ARDUINO_A2_GPIO_Port, ARDUINO_A2_Pin, 0);
    __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_VSYNC);
    
}

void cam_flash_start (uint32_t flash_percent)
{
    printf("%s\n", __func__);
    uint32_t flash_ticks;
    flash_ticks = (g_expo_us * flash_percent)/100;
    flash_ticks = flash_ticks / 0.8333; //us to ticks
    HAL_GPIO_WritePin (ARDUINO_A0_GPIO_Port, ARDUINO_A0_Pin, 1);
    htim1.Instance->CCR2 = htim1.Instance->CNT + flash_ticks;
    HAL_TIM_OC_Start_IT (&htim1, TIM_CHANNEL_2);
    
}

void cam_flash_stop (uint32_t not_used)
{
    
    printf("%s\n", __func__);
    HAL_GPIO_WritePin (ARDUINO_A0_GPIO_Port, ARDUINO_A0_Pin, 0);
    HAL_TIM_OC_Stop_IT (&htim1, TIM_CHANNEL_2);
    HAL_TIM_OC_DeInit (&htim1);
}

void cam_save (uint32_t file_no)
{
    printf("%s\n", __func__);
//  uint32_t i, j;  
//  for(i = 0; i < PIX_H; i++)
//  {
//      img_bayer2rgb (i);
//  }

    static GPIO_InitTypeDef GPIO_InitStruct = {0};
//      HAL_DCMI_Stop (&hdcmi);
//    while(hdcmi.DMA_Handle->State != HAL_DMA_STATE_READY);
    img_compress_rgb ();
    HAL_Delay (1);
    if(HAL_GPIO_ReadPin (SD_DETECT_GPIO_PORT, SD_DETECT_PIN) == GPIO_PIN_RESET)
    {
        write_sd_card (file_no);
    }
    HAL_ADC_Start (&hadc3);
    printf("ADC value is : %d\n", HAL_ADC_GetValue (&hadc3));
    trig_init ();
    printf("DCMI : state %d, Error %d\n",hdcmi.State, hdcmi.ErrorCode);
    printf("DMA : state %d, Error %d\n",hdcmi.DMA_Handle->State, hdcmi.DMA_Handle->ErrorCode);
//    cam_state = CAM_INIT;
//    cam_state_change = true;
//    cam_param = CAMERA_RAW;
//      HAL_DCMI_DeInit (&hdcmi);
//      HAL_DCMI_Init (&hdcmi);
}

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
    hltdc.LayerCfg->ImageWidth = PIX_W/4;
    hltdc.LayerCfg->ImageHeight = PIX_H/4;
//    hltdc.LayerCfg->ImageWidth = RK043FN48H_WIDTH;
//    hltdc.LayerCfg->ImageHeight = RK043FN48H_HEIGHT;
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
    layer_cfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
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
    if(ar0135_ReadID(CameraHwAddress) == AR0135_ID)
    {
        camera_driv = &ar0135_drv;/* Initialize the camera driver structure */
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

uint8_t img_compress_rgb (void)
{
    static uint32_t cmpr_index;
    for(uint32_t index = 0; index < PIX_H; index += 4)
    {
        cmpr_index = index/4;
        for(uint32_t hr_i = 0, lr_i = 0; hr_i < PIX_W; hr_i += 4, lr_i++)
        {
            _fb_base_rgb[cmpr_index][lr_i].r = _fb_base_hr[index][hr_i + 1];
            _fb_base_rgb[cmpr_index][lr_i].g = (_fb_base_hr[index][hr_i]
                + _fb_base_hr[index + 1][hr_i + 1])/2;
            _fb_base_rgb[cmpr_index][lr_i].b = _fb_base_hr[index+1][hr_i];
        }
    }
    return 0;
}
/*
uint8_t img_compress_16 (uint32_t index)
{
    static uint32_t cmpr_index;
    cmpr_index = index/4;
    for(uint32_t hr_i = 0, lr_i = 0; hr_i < PIX_W; hr_i += 8, lr_i += 2)
    {
        _fb_base_lr[cmpr_index][lr_i] = _fb_base_hr[index][hr_i];
        _fb_base_lr[cmpr_index][lr_i+1] = _fb_base_hr[index][hr_i+1];
        _fb_base_lr[cmpr_index+1][lr_i] = _fb_base_hr[index+1][hr_i];
        _fb_base_lr[cmpr_index+1][lr_i+1] = _fb_base_hr[index+1][hr_i+1];
    }
}



uint8_t img_bayer2rgb (uint32_t index)
{
    //set last byte to '\n'
    //GBRG bayer format
    //if index = 0 || 479 (first and last row)
    if (index == 0 || index == (PIX_H-1))
    {
        //whole row zero
        memset (&_fb_base_ppm[index][0], 0, PIX_W*3);
    }
    //check if index is odd : BG
    else if (index % 2 != 0)
    {
        //run for start from blue
        for(uint32_t raw_i = 0; raw_i < PIX_W; raw_i++)
        {
            //0th byte || (PIX_W - 1) byte 
            if (raw_i == 0 || raw_i == (PIX_W - 1))
            {
                //all zero
                _fb_base_ppm[index][raw_i].r = 0;
                _fb_base_ppm[index][raw_i].g = 0;
                _fb_base_ppm[index][raw_i].b = 0;
            }
            //check if odd byte : Gb
            else if (raw_i % 2 != 0)
            {
                //red = vertical avg
                _fb_base_ppm[index][raw_i].r = (_fb_base_hr[index - 1][raw_i] +
                    _fb_base_hr[index + 1][raw_i])/2;
                //green = reading
                _fb_base_ppm[index][raw_i].g = (_fb_base_hr[index][raw_i]
                    + _fb_base_hr[index - 1][raw_i - 1] 
                    + _fb_base_hr[index - 1][raw_i + 1]
                    + _fb_base_hr[index + 1][raw_i - 1]
                    + _fb_base_hr[index + 1][raw_i + 1])/5;
                //blue = horizontal avg
                _fb_base_ppm[index][raw_i].b = (_fb_base_hr[index][raw_i - 1] 
                    + _fb_base_hr[index][raw_i + 1])/2;
            }
            //if even : B
            else
            {
                //red = diagonal avg
                _fb_base_ppm[index][raw_i].r = (_fb_base_hr[index - 1][raw_i - 1] 
                    + _fb_base_hr[index - 1][raw_i + 1]
                    + _fb_base_hr[index + 1][raw_i - 1]
                    + _fb_base_hr[index + 1][raw_i + 1])/4;
                //green = vertical and horizontal avg
                _fb_base_ppm[index][raw_i].g = (_fb_base_hr[index][raw_i - 1]
                    + _fb_base_hr[index][raw_i + 1]
                    + _fb_base_hr[index - 1][raw_i]
                    + _fb_base_hr[index + 1][raw_i])/4;
                //blue = reading
                _fb_base_ppm[index][raw_i].b = _fb_base_hr[index][raw_i];
            }
        }
    
    }
    //else index is even : GR
    else
    {
        //run for start from green
        for(uint32_t raw_i = 0; raw_i < PIX_W; raw_i++)
        {
            //0th byte || (PIX_W - 1)th byte:
            if (raw_i == 0 || raw_i == (PIX_W - 1))
            {
                //all zeros
                _fb_base_ppm[index][raw_i].r = 0;
                _fb_base_ppm[index][raw_i].g = 0;
                _fb_base_ppm[index][raw_i].b = 0;
            }
            //check if odd byte : R
            else if(raw_i%2 != 0)
            {
                //red = reading
                _fb_base_ppm[index][raw_i].r = _fb_base_hr[index][raw_i];
                //green = horizontal and vertical avg
                _fb_base_ppm[index][raw_i].g = (_fb_base_hr[index][raw_i - 1]
                    + _fb_base_hr[index][raw_i + 1]
                    + _fb_base_hr[index - 1][raw_i]
                    + _fb_base_hr[index + 1][raw_i])/4;
                //blue = diagonal avg
                _fb_base_ppm[index][raw_i].b = (_fb_base_hr[index - 1][raw_i - 1] 
                    + _fb_base_hr[index - 1][raw_i + 1]
                    + _fb_base_hr[index + 1][raw_i - 1]
                    + _fb_base_hr[index + 1][raw_i + 1])/4;
            }
            //if even : Gr
            else
            {
                //red = horizontal avg
                _fb_base_ppm[index][raw_i].r = (_fb_base_hr[index][raw_i - 1] 
                    + _fb_base_hr[index][raw_i + 1])/2;
                //green = reading + diagonal corner
                _fb_base_ppm[index][raw_i].g = (_fb_base_hr[index][raw_i] 
                    + _fb_base_hr[index - 1][raw_i - 1] 
                    + _fb_base_hr[index - 1][raw_i + 1]
                    + _fb_base_hr[index + 1][raw_i - 1]
                    + _fb_base_hr[index + 1][raw_i + 1])/5;
                //blue = vertical avg
                _fb_base_ppm[index][raw_i].b = (_fb_base_hr[index - 1][raw_i] 
                    + _fb_base_hr[index + 1][raw_i])/2;
            }
        }
    }
    
    

    return 0;
            
}

    //Split function into 4 functions
    //0. Zeros
    //1. Odd rows odd columns
    //2. Even rows odd columns
    //3. Odd rows even columns
    //4. Even rows even columns
uint8_t lcd_bayer2rgb (uint32_t index)
{
    //set last byte to '\n'
    //GBRG bayer format
    //if index = 0 || 479 (first and last row)
    if (index == 0 || index == (239))
    {
        //whole row zero
        memset (&_fb_base_lcd[index][0], 0, 320*3);
    }
    //check if index is odd : BG
    else if (index % 2 != 0)
    {
        //run for start from blue
        for(uint32_t raw_i = 0; raw_i < 320; raw_i++)
        {
            //0th byte || (PIX_W - 1) byte 
            if (raw_i == 0 || raw_i == (320 - 1))
            {
                //all zero
                _fb_base_lcd[index][raw_i].r = 0;
                _fb_base_lcd[index][raw_i].g = 0;
                _fb_base_lcd[index][raw_i].b = 0;
            }
            //check if odd byte : Gb
            else if (raw_i % 2 != 0)
            {
                //red = vertical avg
                _fb_base_lcd[index][raw_i].r = (_fb_base_lr[index - 1][raw_i] +
                    _fb_base_lr[index + 1][raw_i])/2;
                //green = reading
                _fb_base_lcd[index][raw_i].g = (_fb_base_lr[index][raw_i]
                    + _fb_base_lr[index - 1][raw_i - 1] 
                    + _fb_base_lr[index - 1][raw_i + 1]
                    + _fb_base_lr[index + 1][raw_i - 1]
                    + _fb_base_lr[index + 1][raw_i + 1])/5;
                //blue = horizontal avg
                _fb_base_lcd[index][raw_i].b = (_fb_base_lr[index][raw_i - 1] 
                    + _fb_base_lr[index][raw_i + 1])/2;
            }
            //if even : B
            else
            {
                //red = diagonal avg
                _fb_base_lcd[index][raw_i].r = (_fb_base_lr[index - 1][raw_i - 1] 
                    + _fb_base_lr[index - 1][raw_i + 1]
                    + _fb_base_lr[index + 1][raw_i - 1]
                    + _fb_base_lr[index + 1][raw_i + 1])/4;
                //green = vertical and horizontal avg
                _fb_base_lcd[index][raw_i].g = (_fb_base_lr[index][raw_i - 1]
                    + _fb_base_lr[index][raw_i + 1]
                    + _fb_base_lr[index - 1][raw_i]
                    + _fb_base_lr[index + 1][raw_i])/4;
                //blue = reading
                _fb_base_lcd[index][raw_i].b = _fb_base_lr[index][raw_i];
            }
        }
    
    }
    //else index is even : GR
    else
    {
        //run for start from green
        for(uint32_t raw_i = 0; raw_i < 320; raw_i++)
        {
            //0th byte || (PIX_W - 1)th byte:
            if (raw_i == 0 || raw_i == (320 - 1))
            {
                //all zeros
                _fb_base_lcd[index][raw_i].r = 0;
                _fb_base_lcd[index][raw_i].g = 0;
                _fb_base_lcd[index][raw_i].b = 0;
            }
            //check if odd byte : R
            else if(raw_i%2 != 0)
            {
                //red = reading
                _fb_base_lcd[index][raw_i].r = _fb_base_lr[index][raw_i];
                //green = horizontal and vertical avg
                _fb_base_lcd[index][raw_i].g = (_fb_base_lr[index][raw_i - 1]
                    + _fb_base_lr[index][raw_i + 1]
                    + _fb_base_lr[index - 1][raw_i]
                    + _fb_base_lr[index + 1][raw_i])/4;
                //blue = diagonal avg
                _fb_base_lcd[index][raw_i].b = (_fb_base_lr[index - 1][raw_i - 1] 
                    + _fb_base_lr[index - 1][raw_i + 1]
                    + _fb_base_lr[index + 1][raw_i - 1]
                    + _fb_base_lr[index + 1][raw_i + 1])/4;
            }
            //if even : Gr
            else
            {
                //red = horizontal avg
                _fb_base_lcd[index][raw_i].r = (_fb_base_lr[index][raw_i - 1] 
                    + _fb_base_lr[index][raw_i + 1])/2;
                //green = reading + diagonal corner
                _fb_base_lcd[index][raw_i].g = (_fb_base_lr[index][raw_i] 
                    + _fb_base_lr[index - 1][raw_i - 1] 
                    + _fb_base_lr[index - 1][raw_i + 1]
                    + _fb_base_lr[index + 1][raw_i - 1]
                    + _fb_base_lr[index + 1][raw_i + 1])/5;
                //blue = vertical avg
                _fb_base_lcd[index][raw_i].b = (_fb_base_lr[index - 1][raw_i] 
                    + _fb_base_lr[index + 1][raw_i])/2;
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
*/
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
