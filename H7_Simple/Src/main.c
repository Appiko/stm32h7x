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
#include "dcmi.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "camera.h"
#include "stdio.h"
#include "ar0135.h"
#include "ov7725.h"
#include "ov7725_regs.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    CAM_TRIG,
    CAM_FLASH,
    CAM_SAVE,
    CAM_DONE,
        
}camera_state_t;

typedef enum
{
    AT_READ,
    AT_SEND,
    AT_UPDATE,
}at_state_t;

typedef enum
{
    MOD_INIT,
    MOD_STRAT,
    MOD_STOP,
}module_state_t;

typedef enum
{
    MOTION_MOD,
    LIGHT_MOD,
    CAMERA_MOD,
    MOD_MAX,
        
}modules_t;


#define FILE_OPER_MAX_TRY 3

#define FLASH_TICKS 600

#define SHOT_TICKS 1250

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int cambus_scan()
{
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
//        HAL_Delay (10);
        __disable_irq() ;
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 100) == HAL_OK) {
            __enable_irq() ;
            return (addr << 1) ;
        }
        __enable_irq() ;
    }

    return 0;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t _fb_base;

FIL fp;

uint8_t * p_fb_base = &_fb_base;

static uint32_t g_expo_us;

static uint32_t g_file_no;

const uint8_t ppm_header[] = {'P','5','\n','3','2','0',' ','2','4','0','\n','2','5','5','\n'};


volatile module_state_t g_arr_mod_state[MOD_MAX]; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void print_idata ()
{
    for(uint32_t row = 100; row < 110; row++)
    {
        for(uint32_t col = 100; col < 110; col++)
        {
            printf("%d ",p_fb_base[row * col]);
        }
        printf("\n");
    }
}
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_DCMI_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  
  
//  HAL_I2C_MspInit (&hi2c1);
  printf("Hello World\n");
  
  printf ("FB Base : 0x%x Pointer : 0x%x\n", &_fb_base, p_fb_base);
  printf ("Size of PPM Header : %d\n", sizeof(ppm_header));
  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
  memset (p_fb_base, 0, 240*576);
  HAL_Delay (10);
  
  g_arr_mod_state[CAMERA_MOD] = MOD_INIT;
  sd_init ();
  get_file_no ();
  print_idata ();

  uint32_t cam = cambus_scan ();
  printf ("Cam : 0x%x\n", cam);
  printf("Version : 0x%x\n",CAMERA_IO_Read (0x42, VER));
  
  
  
  ov7725_drv.Init(cam, FRAMESIZE_QVGA);
////    
  HAL_Delay (100);
//
  HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)p_fb_base, 200*200);
  g_arr_mod_state[CAMERA_MOD] = MOD_STRAT;

  HAL_Delay (1000);
//  while(HAL_DCMI_GetState (&hdcmi) == HAL_DCMI_STATE_BUSY);
//
  printf ("DCMI State 0x%x Error 0x%x\n",hdcmi.State, hdcmi.ErrorCode);
  printf ("DMA State 0x%x Error 0x%x\n",hdcmi.DMA_Handle->State, hdcmi.DMA_Handle->ErrorCode);
  print_idata ();
//  memcpy (p_fb_base, ppm_header, sizeof(ppm_header));
//  
  g_file_no++;
  write_sd_card (g_file_no);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 80;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DCMI_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DCMI_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_DCMI_LineEventCallback (DCMI_HandleTypeDef* hdcmi)
{
    static uint32_t hsync_count = 0;
    
//    printf("Hsync : %d\n",hsync_count++);
}

void HAL_DCMI_VsyncEventCallback (DCMI_HandleTypeDef* hdcmi)
{
//    printf("Vsync\n");
  g_arr_mod_state[CAMERA_MOD] = MOD_STOP;
}


void sd_init (void)
{
    uint32_t status;
  if(BSP_SD_IsDetected ())
  {
      printf("SD Card detected\n");
      HAL_Delay (100);
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
      else {printf("SD Card not ready : %d 0x%x\n", hsd1.State, status);}
  }
  else{printf("SD Card not present\n");}
}

void write_sd_card (uint32_t file_no)
{
//    while(hdcmi.State != HAL_DCMI_STATE_READY);
    static attempts_remain = FILE_OPER_MAX_TRY;
    uint32_t status = 0;
    uint8_t file_name[] = {'f','o','o','_','x','x','x','x','.','r','g','b', '\0'};
    
    file_name[4] = (file_no/1000) + 0x30;
    file_name[5] = ((file_no%1000)/100) + 0x30;
    file_name[6] = ((file_no%100)/10) + 0x30;
    file_name[7] = (file_no%10) + 0x30;
    
    printf("Attempt : %d\n", FILE_OPER_MAX_TRY - attempts_remain + 1);
    printf ("upcoming file name : %x %x %x %x\n", file_name[4],file_name[5],file_name[6],file_name[7]);
    if(attempts_remain > 0)
    {
        status = f_open (&fp, (char *)file_name, FA_CREATE_ALWAYS | FA_WRITE);
//        status = f_open (&fp, "0020.rgb", FA_CREATE_ALWAYS | FA_WRITE);
        if(status == FR_OK)
        {
            uint8_t byteswritten;
            printf("File open\n");
            status = f_write (&fp,(void *)p_fb_base, 320*240, &byteswritten);
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
    file_no = ((f_name[4] > 0x29) && (f_name[4] < 0x3A)) ?
        (((f_name[4] - 0x30) * 1000) + 
        ((f_name[5] - 0x30) * 100) +
        ((f_name[6] - 0x30) * 10) +
        ((f_name[7] - 0x30) * 1)) :
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
