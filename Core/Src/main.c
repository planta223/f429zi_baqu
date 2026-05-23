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

/* Ethernet monitor */
volatile uint8_t  g_mon_eth_initialized = 0U;       // Ethernet_Init 성공 여부
volatile uint8_t  g_mon_eth_mode = 0U;              // 현재 조향 모드
volatile uint32_t g_mon_eth_last_rx_tick = 0U;      // 마지막 UDP 수신 tick
volatile uint32_t g_mon_now_tick = 0U;              // 현재 HAL tick
volatile uint32_t g_mon_eth_rx_age_ms = 0U;         // 마지막 수신 이후 경과 시간
volatile uint8_t  g_mon_eth_timeout = 0U;           // timeout 상태

/* Latest Ethernet packet monitor */
volatile uint8_t  g_mon_packet_source = 0U;         // 0:none, ASMS/PC source enum 값
volatile float    g_mon_packet_steering_deg = 0.0f; // Ethernet에서 변환된 목표 조향각
volatile uint16_t g_mon_packet_asms_adc_raw = 0U;   // ASMS ADC raw
volatile int32_t  g_mon_packet_pc_steer_raw = 0;    // PC steering raw
volatile uint32_t g_mon_packet_speed_raw = 0U;      // PC speed raw
volatile uint8_t  g_mon_packet_misc = 0U;           // PC misc byte

/* Control monitor */
volatile uint8_t  g_mon_control_enabled = 0U;       // 제어 활성화 여부
volatile uint8_t  g_mon_control_reached = 0U;       // 목표 도달 여부
volatile float    g_mon_target_steering_deg = 0.0f; // 목표 조향각 [deg]
volatile float    g_mon_target_motor_deg = 0.0f;    // 목표 모터각 [deg]
volatile float    g_mon_current_steering_deg = 0.0f;// 현재 조향각 [deg]
volatile float    g_mon_current_motor_deg = 0.0f;   // 현재 모터각 [deg]
volatile float    g_mon_error_motor_deg = 0.0f;     // 모터축 기준 오차 [deg]
volatile float    g_mon_output_freq_hz = 0.0f;      // 제어 출력 주파수 [Hz]

/* Motor monitor */
volatile int32_t  g_mon_motor_requested_freq_hz = 0;// 요청 주파수 [Hz]
volatile uint32_t g_mon_motor_applied_freq_hz = 0U; // 실제 적용 주파수 [Hz]
volatile uint8_t  g_mon_motor_direction = 0U;       // 모터 방향 enum 값
volatile uint8_t  g_mon_motor_output_active = 0U;   // PWM 출력 활성 여부
volatile uint8_t  g_mon_motor_reverse_guard = 0U;   // 방향 전환 guard 동작 여부

/* Encoder monitor */
volatile int64_t  g_mon_encoder_total_count = 0;    // 누적 엔코더 카운트
volatile int32_t  g_mon_encoder_delta_count = 0;    // 1주기 변화량
volatile float    g_mon_encoder_motor_deg = 0.0f;   // 엔코더 기준 모터각 [deg]
volatile float    g_mon_encoder_steering_deg = 0.0f;// 엔코더 기준 조향각 [deg]

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void App_UpdateMonitorVariables(uint32_t now_ms)
{
    Control_State_t control_state;
    Motor_State_t motor_state;
    Encoder_t encoder_state;
    uint32_t last_rx_tick;

    control_state = Control_GetState();
    motor_state = Motor_GetState();
    encoder_state = Encoder_GetState();

    last_rx_tick = Ethernet_GetLastRxTick();

    g_mon_now_tick = now_ms;

    g_mon_eth_initialized = Ethernet_IsInitialized() ? 1U : 0U;
    g_mon_eth_mode = (uint8_t)Ethernet_GetCurrentMode();
    g_mon_eth_last_rx_tick = last_rx_tick;

    if (last_rx_tick == 0U) {
        g_mon_eth_rx_age_ms = 0U;
        g_mon_eth_timeout = 0U;
    } else {
        g_mon_eth_rx_age_ms = (uint32_t)(now_ms - last_rx_tick);
        g_mon_eth_timeout =
            (g_mon_eth_rx_age_ms > ETHERNET_TIMEOUT_MS) ? 1U : 0U;
    }

    g_mon_control_enabled = control_state.enabled;
    g_mon_control_reached = control_state.reached;
    g_mon_target_steering_deg = control_state.target_steering_deg;
    g_mon_target_motor_deg = control_state.target_motor_deg;
    g_mon_current_steering_deg = control_state.current_steering_deg;
    g_mon_current_motor_deg = control_state.current_motor_deg;
    g_mon_error_motor_deg = control_state.error_motor_deg;
    g_mon_output_freq_hz = control_state.output_freq_hz;

    g_mon_motor_requested_freq_hz = motor_state.requested_freq_hz;
    g_mon_motor_applied_freq_hz = motor_state.applied_freq_hz;
    g_mon_motor_direction = (uint8_t)motor_state.direction;
    g_mon_motor_output_active = motor_state.output_active;
    g_mon_motor_reverse_guard = motor_state.reverse_guard_active;

    g_mon_encoder_total_count = encoder_state.total_count;
    g_mon_encoder_delta_count = encoder_state.delta_count;
    g_mon_encoder_motor_deg = encoder_state.motor_deg;
    g_mon_encoder_steering_deg = encoder_state.steering_deg;
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

	        g_mon_packet_source = (uint8_t)packet.source;
	        g_mon_packet_steering_deg = packet.steering_deg;
	        g_mon_packet_asms_adc_raw = packet.asms_adc_raw;
	        g_mon_packet_pc_steer_raw = packet.pc_steer_raw;
	        g_mon_packet_speed_raw = packet.speed_raw;
	        g_mon_packet_misc = packet.misc;

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

	        App_UpdateMonitorVariables(now_ms);
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
