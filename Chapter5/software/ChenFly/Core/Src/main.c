/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler.h"
#include "motor.h"
#include "icm20602.h"
#include "flightcontrol.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUFF_SIZE1 128
uint8_t rx_data1[BUFF_SIZE1] = {0x00};
#define BUFF_SIZE4 128
uint8_t rx_data4[BUFF_SIZE4] = {0x00};

extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint16_t motorA_speed;
extern uint16_t motorB_speed;
extern uint16_t motorC_speed;
extern uint16_t motorD_speed;
extern uint8_t sbus_original_data[25];

extern float roll_P, roll_I, roll_D;
extern float pitch_P, pitch_I, pitch_D;
extern float yaw_P, yaw_I, yaw_D;
uint8_t send_pid[23] = {0xFE, 0xEF, 0x17, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_data1, BUFF_SIZE1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_data4, BUFF_SIZE4);
  Scheduler_init();
  motor_init();
  icm20602_init_spi();
  gyroOffsetInit();
  init_flightcontrol();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Scheduler_run();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
		HAL_UART_DMAStop(huart);
        if (Size <= BUFF_SIZE1)
        {
			if(rx_data1[0] == 0xFE && rx_data1[1] == 0xEF && rx_data1[3] == 0x01)
			{
				motorA_speed = 1000+10*rx_data1[4];
				motorB_speed = 1000+10*rx_data1[5];
				motorC_speed = 1000+10*rx_data1[6];
				motorD_speed = 1000+10*rx_data1[7];
			}
			else if(rx_data1[0] == 0xFE && rx_data1[1] == 0xEF && rx_data1[3] == 0x02)
			{
				uint16_t temp;
				temp = (uint16_t)(roll_P*1000.0f);
				memcpy(send_pid+4, &temp, 2);
				temp = (uint16_t)(roll_I*1000.0f);
				memcpy(send_pid+6, &temp, 2);
				temp = (uint16_t)(roll_D*1000.0f);
				memcpy(send_pid+8, &temp, 2);
				temp = (uint16_t)(pitch_P*1000.0f);
				memcpy(send_pid+10, &temp, 2);
				temp = (uint16_t)(pitch_I*1000.0f);
				memcpy(send_pid+12, &temp, 2);
				temp = (uint16_t)(pitch_D*1000.0f);
				memcpy(send_pid+14, &temp, 2);
				temp = (uint16_t)(yaw_P*1000.0f);
				memcpy(send_pid+16, &temp, 2);
				temp = (uint16_t)(yaw_I*1000.0f);
				memcpy(send_pid+18, &temp, 2);
				temp = (uint16_t)(yaw_D*1000.0f);
				memcpy(send_pid+20, &temp, 2);
				HAL_UART_Transmit_DMA(&huart1, send_pid, 23);
			}
			else if(rx_data1[0] == 0xFE && rx_data1[1] == 0xEF && rx_data1[3] == 0x03)
			{
				uint16_t temp;
				memcpy(&temp, rx_data1+4, 2);
				roll_P = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+6, 2);
				roll_I = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+8, 2);
				roll_D = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+10, 2);
				pitch_P = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+12, 2);
				pitch_I = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+14, 2);
				pitch_D = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+16, 2);
				yaw_P = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+18, 2);
				yaw_I = ((float)temp)/1000.0f;
				memcpy(&temp, rx_data1+20, 2);
				yaw_D = ((float)temp)/1000.0f;
			}
			else
			{
				
			}
        }
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_data1, BUFF_SIZE1);
    }
	else if (huart->Instance == UART4)
	{
		HAL_UART_DMAStop(huart);
        if (Size == 25 && rx_data4[0] == 0x0F)
        {
			for(int i = 0; i < 25; i++)
			{
				sbus_original_data[i] = rx_data4[i];
			}
        }
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_data4, BUFF_SIZE4);
	}
	else
	{
	  
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

