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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stdio.h"
#include "ov7725.h"
#include "cambus.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PIX_H 480
#define PIX_W 640
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef uint8_t row_t[PIX_H][PIX_W];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t arr_pix_row[(PIX_W * 3)] = {};
FIL f_img;

const uint8_t ppm_header[] = {'P','6','\n','1','6',' ','0','5','\n','2','5','5','\n'};
FRESULT res;
uint32_t byteswritten;
const TCHAR * filename = "bayer2ppm_01.ppm";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

uint8_t img_file_init (void);

void img_bayer2rgb_mid (uint32_t index, uint8_t * p_bayer);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  sensor_t sensor;

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
  uart_msp_init();
  dcmi_msp_init ();
  tim_msp_init ();
  tim_pwm1_start ();
  i2c_msp_init ();
  sdmmc_msp_init ();
  printf("Hello World\n");
  HAL_GPIO_WritePin (LED_R_GPIO_Port, LED_R_Pin, 0);
  HAL_Delay (50);
  HAL_GPIO_WritePin (LED_R_GPIO_Port, LED_R_Pin, 1);
  HAL_GPIO_WritePin (LED_G_GPIO_Port, LED_G_Pin, 1);
  uint8_t ver_addr = 10;
  uint8_t ver[] = {0, 0} ;
  uint32_t status_tx = 0;
  uint32_t status_rx = 0;
  status_tx = HAL_I2C_Master_Transmit (i2c_get_i2c1_handle(), 66, &ver_addr, 1, 100);
  status_rx = HAL_I2C_Master_Receive (i2c_get_i2c1_handle(), 66, ver, 2, 100);
  printf("Status TX 0x%x, Status RX 0x%x Ver 0x%x ID 0x%x\n", status_tx, status_rx, ver[0], ver[1]);
//  printf("Err 0x%x, Mode 0x%x, State 0x%x\n", HAL_I2C_GetError (i2c_get_i2c1_handle ()),
//  HAL_I2C_GetMode (i2c_get_i2c1_handle ()),HAL_I2C_GetState (i2c_get_i2c1_handle () );
  uint8_t fs_status;
//  fs_status = fatfs_init ();
//  printf("FatFs Status : %d\n", fs_status);
//  img_file_init ();
  printf("I2C device found : %d\n", cambus_scan());
  {
      sensor.framesize = FRAMESIZE_VGA;
      sensor.slv_addr = cambus_scan();
  }
  ov7725_init (&sensor);
  
//  sensor.set_pixformat(&sensor, PIXFORMAT_RGB565);
  image_t img = 
  {
      .bpp = 2,
  };
  
  sensor.snapshot(&sensor, &img);


//  printf("Image dimensions(h x w) : %d x %d\n", img.h, img.w);
//  printf("Image bpp : %d\n", img.bpp);
  for(uint32_t i=0; i< img.h; i++)
  {
      img_bayer2rgb_mid (i, img.pixels);
  }
  
//  printf("\n\n");

//  for(uint32_t pix = 0; pix < img.h*img.w*2; pix++)
//  {
//      printf("%d ", img.pixels[pix]);
//  }
  
    printf("\nEOF \n");
//  HAL_GPIO_WritePin (LED_G_GPIO_Port, LED_G_Pin, 0);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);
}

/* USER CODE BEGIN 4 */
uint8_t img_file_init (void)
{
    res = f_mount (fatfs_get_fs_ptr (), (TCHAR const*)SDPath, 0); 
    if(res == FR_OK)
    {
        if(f_open (&f_img, filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
        {
            res = f_write (&f_img, ppm_header, sizeof(ppm_header), (void *)byteswritten);
            if((byteswritten > 0) &&(res == FR_OK))
            {
                f_close(&f_img);
            }
        }
    }
    return res;
}

uint8_t img_file_append (uint8_t * p_data, uint32_t len)
{
    res = f_mount (fatfs_get_fs_ptr (), fatfs_get_fs_path (), 0); 
    if(res == FR_OK)
    {
        if(f_open (&f_img, filename, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
        {
            res = f_write (&f_img, p_data, len, (void *)byteswritten);
            if((byteswritten > 0) &&(res == FR_OK))
            {
                f_close(&f_img);
            }
        }
    }
    return res;
    
}

void img_bayer2rgb_mid (uint32_t index, uint8_t * p_bayer)
{
    //set last byte to '\n'
    //GBRG bayer format
    //if index = 0 || 479 (first and last row)
    if (index == 0 || index == (PIX_H-1))
    {
        //whole row zero
        memset (arr_pix_row, 0, PIX_W*3);
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
                arr_pix_row[img_i] = 0;
                arr_pix_row[img_i + 1] = 0;
                arr_pix_row[img_i + 2] = 0;
            }
            //check if odd byte
            else if (raw_i % 2 != 0)
            {
                //red = avg of prev and next
                arr_pix_row[img_i] = (p_bayer[(index * PIX_W) + raw_i - 1] +
                    p_bayer[(index * PIX_W) + raw_i + 1])/2;
                //green = reading
                arr_pix_row[img_i + 1] = p_bayer[(index * PIX_W) + raw_i];
                //blue = avg of top and bottom
                arr_pix_row[img_i + 2] = (p_bayer[((index-1) * PIX_W) + raw_i] + 
                    p_bayer[((index+1) * PIX_W) + raw_i])/2;
            }
            //if even 
            else
            {
                //red = reading
                arr_pix_row[img_i] = p_bayer[(index * PIX_W) + raw_i];
                //green  = avg of top bottom sideways
                arr_pix_row[img_i + 1] = (p_bayer[((index-1) * PIX_W) + raw_i] + 
                    p_bayer[((index+1) * PIX_W) + raw_i] +
                    p_bayer[(index * PIX_W) + raw_i - 1] +
                    p_bayer[(index * PIX_W) + raw_i + 1])/4;
                //blue = avg of corners
                arr_pix_row[img_i + 2] = (p_bayer[((index-1) * PIX_W) + raw_i - 1] + 
                    p_bayer[((index-1) * PIX_W) + raw_i + 1] + 
                    p_bayer[((index+1) * PIX_W) + raw_i - 1] +
                    p_bayer[((index+1) * PIX_W) + raw_i + 1])/4;
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
                arr_pix_row[img_i] = 0;
                arr_pix_row[img_i+1] = 0;
                arr_pix_row[img_i+2] = 0;
            }
            //check if odd byte
            else if(raw_i%2 != 0)
            {
                //red = avg of corners
                arr_pix_row[img_i] = (p_bayer[((index-1) * PIX_W) + raw_i - 1] + 
                    p_bayer[((index-1) * PIX_W) + raw_i + 1] + 
                    p_bayer[((index+1) * PIX_W) + raw_i - 1] +
                    p_bayer[((index+1) * PIX_W) + raw_i + 1])/4;
                //green = avg of top bottom sideways
                arr_pix_row[img_i+1] = (p_bayer[((index-1) * PIX_W) + raw_i] + 
                    p_bayer[((index+1) * PIX_W) + raw_i] +
                    p_bayer[(index * PIX_W) + raw_i - 1] +
                    p_bayer[(index * PIX_W) + raw_i + 1])/4;
                //blue = reading
                arr_pix_row[img_i+2] = p_bayer[(index * PIX_W) + raw_i];
            }
            //if even
            else
            {
                //red = avg of top and bottom
                arr_pix_row[img_i] = (p_bayer[((index-1) * PIX_W) + raw_i] + 
                    p_bayer[((index+1) * PIX_W) + raw_i])/2;
                //green = reading
                arr_pix_row[img_i+1] = p_bayer[(index * PIX_W) + raw_i];
                //blue = avg of prev and next
                arr_pix_row[img_i+2] = (p_bayer[(index * PIX_W) + raw_i - 1] +
                    p_bayer[(index * PIX_W) + raw_i + 1])/2;
            }
        }
    }
    
    
    //append file
//    uint8_t status;
//     status = img_file_append (arr_pix_row, sizeof(arr_pix_row));
//     printf("%d : %d\n", index, status);
    
    //print row
    for(uint i = 0; i < PIX_W * 3; i++)
    {
        printf("%c", arr_pix_row[i]);
    }
            
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
