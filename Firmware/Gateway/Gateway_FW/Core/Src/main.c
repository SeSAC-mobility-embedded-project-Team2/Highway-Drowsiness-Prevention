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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_message.h" // 공통 헤더
#include <gateway_defines.h>
#include <fuzzy_logic.h>
#include <comm_manager.h>
#include <stdio.h>
#include "unity.h"
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t last_execution_time = 0;
volatile uint8_t timer_100ms_flag = 0;
// === 전역 변수 실제 생성 (메모리 할당) ===
SystemState_t current_state = STATE_NORMAL;
float prev_steering_angle = 0;
uint32_t no_op_timer = 0;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// UART 버퍼 및 구조체 변수들
uint8_t uart_rx_buffer[8];
extern VisionData_t vision_data;
extern ChassisData_t chassis_data;
extern BodyData_t body_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void Update_System_State();
extern void Run_ASPICE_Unit_Tests(void);
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
	printf("system start...\r\n");
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    printf("\r\n=======================================================\r\n");
    printf("   ASPICE SWE.4 Unit Verification Report               \r\n");
    printf("-------------------------------------------------------\r\n");
    printf("   Target Project : Drowsiness Prevention System       \r\n");
    printf("   SW Version     : V0.8                               \r\n");
    printf("   Test Date      : 2026-01-16 (Manual Run)            \r\n");
    printf("=======================================================\r\n\r\n");

    Run_ASPICE_Unit_Tests();
    printf("============================================\r\n");
  // === 1. CAN 필터 및 시작 설정  ===
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    // === 2. UART(Vision) 수신 인터럽트 시작 ===
    HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 8);

    // === 3. Timer3 인터럽트 시작 ===
    HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (timer_100ms_flag == 1)
	  {
		  timer_100ms_flag = 0;

		  Update_System_State();
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// printf 출력을 위한 리타겟팅 함수
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    // 디버깅용 UART 채널.
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 30);
    return len;
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        timer_100ms_flag = 1;
    }
}

// 1. UART 수신 콜백 (Vision)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 매니저에게 버퍼 넘기기
    DMS_Process_UART_Data(huart, uart_rx_buffer);
}

// 2. CAN 수신 콜백 (Chassis, Body)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // 매니저에게 패킷 넘기기
        DMS_Process_CAN_Data(&RxHeader, RxData);
    }
}

void Update_System_State()
{
	// 1. 데이터 스냅샷(Snapshot)을 위한 로컬 변수 선언
	VisionData_t  vision_data_local;
	ChassisData_t chassis_data_local;
	BodyData_t    body_data_local;

	// 2. 크리티컬 섹션 (Critical Section): 인터럽트 잠시 중단
	__disable_irq();

    // 3. 전역 변수 값을 로컬 변수로 안전하게 복사
	vision_data_local  = vision_data;
	chassis_data_local = chassis_data;
	body_data_local = body_data;

    // 4. 인터럽트 다시 허용
    __enable_irq();

    // -----------------------------------------------------------
    // 이제부터는 전역변수 대신 로컬 변수(_local)만 사용합니다.
    // -----------------------------------------------------------


    // 비전, 섀시, 바디에서 에러 플래그가 하나라도 0이 아니면 고장 처리
    if (vision_data_local.err_flag != 0 || chassis_data_local.err_flag != 0 || body_data_local.err_flag != 0)
    {
        printf("🔧 SENSOR ERROR DETECTED! (Fail-Safe Mode)\r\n");

        current_state = STATE_FAULT;

        DMS_Send_Control_Signal(&huart3, STATE_FAULT, 1, 1);

        return;
    }

    float current_angle = chassis_data_local.steering_angle;
    // 변화량 계산 (ABS 매크로 사용)
    float angle_diff = (int32_t)current_angle - prev_steering_angle;

    if (angle_diff < 0) angle_diff = -angle_diff;

    // 변화량이 2.0도 미만이면 무조작으로 간주
    if (angle_diff < 2.0f)
    {
        no_op_timer += 100; // 100ms 증가 (루프 주기)
    }
    else
    {
        no_op_timer = 0; // 조작 감지 시 리셋
        prev_steering_angle = current_angle; // 기준점 갱신
    }

    // 얼굴 인식 여부에 따른 데이터 필터링
    uint8_t safe_perclos = 0;

    if (vision_data_local.is_face_detected == 1)
    {
        safe_perclos = vision_data_local.perclos; // 얼굴 있으면 측정값 사용
    }
    else
    {
        safe_perclos = 0; // 얼굴 없으면 0점 (위험도 계산에서 제외됨)
    }

    // ms -> sec 변환
    float no_op_sec = no_op_timer / 1000.0f;

    uint8_t risk_score = Compute_Integrated_Risk(
                            safe_perclos,
                            chassis_data_local.steering_std_dev,
                            body_data_local.hands_off_sec,
                            body_data_local.head_delta_cm,
							no_op_sec
                         );

    if (current_state == STATE_NORMAL)
    {
    	if (risk_score >= 75)
    	{
    		current_state = STATE_WARNING;
    	}
    }
    else if (current_state == STATE_WARNING)
    {
    	if (risk_score >= 95)
    	{
    		current_state = STATE_DANGER;  // 95점 이상 위험
    	}
    	else if (risk_score < 60)
    	{
    		current_state = STATE_NORMAL; // 복귀
    	}
    }
    else if (current_state == STATE_DANGER)
    {
    	if (risk_score < 85)
    	{
    		current_state = STATE_WARNING;
    	}
    }

    // 1. MRM 트리거 조건 판단 (예: Danger 상태이거나, 센서가 다 죽었거나)
    uint8_t mrm_cmd = 0;
    if (current_state == STATE_DANGER || current_state == STATE_FAULT)
    {
        mrm_cmd = 1; // 멈춰!
    }

    // 2. 에러 플래그 판단 (예: Vision 데이터가 너무 안 들어올 때)
    uint8_t sys_err = 0;
    if (vision_data_local.err_flag != 0) // Vision 센서 에러 시
    {
        sys_err = 1; // LogicFail (또는 ICD에 맞는 에러코드)
    }

    // 3. 제어 신호 전송 (ICD V0.1.2 규격)
    DMS_Send_Control_Signal(&huart3, current_state, mrm_cmd, sys_err);

    // 수정된 printf (무조작 시간 확인용)
//        printf("Risk: %d | Eye_safe : %d%% | detected : %d | Hands: %.1fs | Head: %.1f | Steer: %.1f | NoOp: %.1fs\r\n",
//                risk_score,
//				safe_perclos,
//				vision_data_local.is_face_detected,
//                body_data_local.hands_off_sec,
//                body_data_local.head_delta_cm,
//                chassis_data_local.steering_std_dev,
//                no_op_sec
//                );
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
#ifdef USE_FULL_ASSERT
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
