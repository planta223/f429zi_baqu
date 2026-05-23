/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "iwdg.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "encoder.h"
#include "motor.h"
#include "control.h"
#include "config.h"
#include "ethernet.h"

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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  Encoder_Init();
  Motor_Init();
  Control_Init();
  Encoder_Reset();
  Control_Reset();
  Ethernet_Init();
  Control_SetTargetSteeringDeg(0.0f);
  Control_Enable(); // open-loop 사용 안할시 활성화

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_control_tick_ms = HAL_GetTick();

  while (1)
  {
	    uint32_t now_ms = HAL_GetTick();

	    /*
	     * LwIP polling.
	     *
	     * bare-metal LwIP 구조에서는 이 함수가 계속 호출되어야
	     * UDP 수신 callback이 동작한다.
	     */
	    MX_LWIP_Process();

	    /*
	     * ESTOP 처리.
	     *
	     * ASMS ESTOP 또는 PC misc bit7 ESTOP가 들어오면
	     * 제어기를 disable하고 PWM 출력을 정지한다.
	     */
	    if (Ethernet_ConsumeEmergencyRequest()) {
	        Control_Disable();
	        Motor_Stop();
	    }

	    /*
	     * Ethernet 수신 명령 처리.
	     *
	     * ASMS MANUAL packet 또는 PC AUTO packet에서 변환된
	     * 목표 조향각을 control target으로 반영한다.
	     */
	    if (Ethernet_HasNewData()) {
	        Ethernet_Packet_t packet = Ethernet_GetLatestData();
	        SteerMode_t mode = Ethernet_GetCurrentMode();

	        if ((mode == STEER_MODE_AUTO) ||
	            (mode == STEER_MODE_MANUAL)) {

	            Control_SetTargetSteeringDeg(packet.steering_deg);

	            /*
	             * 매 packet마다 Control_Enable()을 호출하면
	             * integral이 계속 reset될 수 있으므로,
	             * disable 상태일 때만 enable한다.
	             */
	            if (Control_IsEnabled() == 0U) {
	                Control_Enable();
	            }
	        }
	    }

	    /*
	     * 통신 timeout 처리.
	     *
	     * 마지막 유효 수신 이후 ETHERNET_TIMEOUT_MS가 지나면
	     * stale command를 유지하지 않고 정지한다.
	     */
	    if ((Ethernet_GetLastRxTick() != 0U) &&
	        ((uint32_t)(now_ms - Ethernet_GetLastRxTick()) > ETHERNET_TIMEOUT_MS)) {
	        Control_Disable();
	        Motor_Stop();
	    }

	    /*
	     * 1 ms control loop.
	     */
	    if ((uint32_t)(now_ms - last_control_tick_ms) >= CONTROL_PERIOD_MS) {
	        last_control_tick_ms += CONTROL_PERIOD_MS;

	        Encoder_Update();
	        Control_Update();
	    }

	    HAL_IWDG_Refresh(&hiwdg);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
