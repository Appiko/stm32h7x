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
#include "dma.h"
#include "mdma.h"
#include "sdmmc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFF_BYTES 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t _bla[1024 * BUFF_BYTES];
extern uint8_t _blu[1024 * BUFF_BYTES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//uint8_t test_file_append ();

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
  MX_MDMA_Init();
  MX_SDMMC1_SD_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
  
  printf("Hello world from SDMMC testing app\n");
  
  printf("Bla address 0x%x Blu address 0x%x\n", _bla, _blu);
  
//  HAL_SD_InitCard (&hsd1);
  
  uint32_t buff_size = 1024 * BUFF_BYTES;
  
  uint32_t status;
  memset(_bla, 0x35, buff_size);
  memset(_blu, 0, buff_size);
  
  HAL_Delay (100);

//  status = HAL_SD_ConfigSpeedBusOperation (&hsd1, SDMMC_HSpeed_CLK_DIV);
  status = HAL_SD_ConfigWideBusOperation (&hsd1, SDMMC_BUS_WIDE_4B);
  printf("Bus status : %d %d, Bus 0x%x, clk_div/2 %d\n", status, hsd1.ErrorCode,
         hsd1.Init.BusWide, hsd1.Init.ClockDiv);

  printf("SD Card Info : %d %d\n", hsd1.SdCard.CardType,
         hsd1.SdCard.CardVersion);
  
  printf("Block info : %d %d %d %d\n",hsd1.SdCard.BlockSize, hsd1.SdCard.BlockNbr,
         hsd1.SdCard.LogBlockSize, hsd1.SdCard.LogBlockNbr);

  printf("Blu before : %x %x %x %x\n", _blu[0], _blu[256], _blu[512], _blu[1023]);
  
  printf("Bla before : %x %x %x %x\n", _bla[0], _bla[256], _bla[512], _bla[1023]);
  
  printf("Length : %d\n", buff_size/hsd1.SdCard.BlockSize);
//  HAL_GPIO_TogglePin (SIG_1_GPIO_Port, SIG_1_Pin);
  status = HAL_SD_WriteBlocks_DMA (&hsd1, _bla, hsd1.SdCard.BlockNbr/2, buff_size/hsd1.SdCard.BlockSize);
  while (hsd1.State != HAL_SD_STATE_READY);
  HAL_GPIO_TogglePin (SIG_1_GPIO_Port, SIG_1_Pin);
  printf("Write status : status %d, state %d ,Error code %d, Context 0x%x\n", status, hsd1.State, hsd1.ErrorCode, hsd1.Context);
  
  
  HAL_Delay (100);
//  while(hsd1.ErrorCode == HAL_SD_ERROR_NONE);
  
  status = HAL_SD_ReadBlocks_DMA (&hsd1, _blu, hsd1.SdCard.BlockNbr/2, buff_size/hsd1.SdCard.BlockSize);
  printf("Read status : status %d, state %d ,Error code %d, Context 0x%x\n", status, hsd1.State, hsd1.ErrorCode, hsd1.Context);
  
  HAL_Delay (10);
  printf("Blu after : %x %x %x %x\n", _blu[0], _blu[256], _blu[512], _blu[1023]);
  
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SDMMC;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//
//uint8_t test_file_append ()
//{
//    static uint32_t res;
//    uint32_t byteswritten;
//    uint8_t p_data[512];
//    uint32_t len = sizeof(p_data);
//    for(uint32_t i = 0; i < len-1; i++)
//    {
//        p_data[i] = i%255;
//    }
//    p_data[len-1] = '\n';
////    res = f_mount (&SDFatFS, (TCHAR const *)SDPath, 0); 
////    if(res == FR_OK)
//    {
////        if(f_open (&p_file, "bla_bla.txt", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
//        {
//            HAL_GPIO_WritePin (SIG_2_GPIO_Port, SIG_2_Pin, 0);
//            res = f_write (&p_file, p_data, len, (void *)byteswritten);
//            if((res == FR_OK))
//            {
//                HAL_GPIO_WritePin (SIG_2_GPIO_Port, SIG_2_Pin, 1);
////                f_close(&p_file);
//            }
//        }
////        else
////        {
////            printf("Append fail\n");
////        }
//    }
////    else
////    {
////        printf("Mount Fail\n");
////    }
//    return res;
//    
//}

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
