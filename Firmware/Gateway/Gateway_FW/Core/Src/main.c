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
#include "can_message.h" // 공통 헤더
#include <stdio.h>       // printf용
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// test comment2
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// 시스템 상태 정의
typedef enum {
    STATE_NORMAL = 0,
    STATE_WARNING, // 주의 (환기, 부저 1단계)
    STATE_DANGER,  // 위험 (감속, 비상등, 부저 2단계)
    STATE_FAULT    // 고장 (통신 두절 등)
} SystemState_t;

SystemState_t current_state = STATE_NORMAL;
// 1. CAN 수신용 변수
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// 2. UART(Vision) 수신용 변수
uint8_t rx_byte; // 1바이트씩 검사할 임시 변수
Vision_UART_Packet_t vision_rx_packet; // 최종 저장할 구조체

// 3. 데이터 저장소 (디버깅용)
Chassis_Data_t chassis_info;
Body_Data_t body_info;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Update_System_State();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf 출력을 USART2(PC 연결)로 보내는 설정
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

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
    // "Vision 패킷 크기만큼 데이터가 들어오면 알려줘!"
//    HAL_UART_Receive_IT(&huart1, (uint8_t*)&vision_rx_packet, sizeof(Vision_UART_Packet_t));
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    printf("Gateway System Started...\r\n"); // PC 터미널에서 보이면 성공!
    printf("Size of Struct: %d bytes\r\n", sizeof(Vision_UART_Packet_t));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  chassis_info.steering_std_dev = 50;
	  printf("=== Raw Data Analysis ===\r\n");
	  // 받은 데이터를 바이트 배열처럼 접근해서 출력
	  uint8_t *ptr = (uint8_t*)&vision_rx_packet;
	  printf("RX: ");
	  for(int i=0; i<10; i++) {
		  printf("%02X ", ptr[i]);// 02X: 16진수 두 글자로 출력 (예: FF 05 1A...)
	  }
	  printf("\r\n");
	  printf("[Vision] PERCLOS: %d\r\n", vision_rx_packet.perclos);
	  printf("-------------------------\r\n\r\n");

	  // 상태 판단 실행
	  Update_System_State();
	  printf("[State] Current: ");
	  switch (current_state)
	  {
	  case STATE_NORMAL:
		  printf("🟢 NORMAL (Safe)\r\n");
		  // (초록 LED 켜기 등의 코드 추가 가능)
		  break;
	  case STATE_WARNING:
		  printf("🟡 WARNING (Drowsy!)\r\n");
		  // [동작] 창문 개방 명령 전송 코드
		  break;
	  case STATE_DANGER:
		  printf("🔴 DANGER (Emergency!)\r\n");
		  // [동작] 비상등 점멸, 모터 감속 명령 전송 코드
		  break;
	  case STATE_FAULT:
		  break;
	  }
	  printf("-------------------------\r\n\r\n");
	  HAL_Delay(500); // 0.5초 대기

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

// 1. CAN 메시지가 도착하면 실행되는 함수
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // 메시지 가져오기
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    return;
  }

  // ID 별로 분류해서 저장
  if (RxHeader.StdId == 0x201) // Chassis ECU
  {
    memcpy(&chassis_info, RxData, sizeof(Chassis_Data_t));
  }
  else if (RxHeader.StdId == 0x301) // Body ECU
  {
    memcpy(&body_info, RxData, sizeof(Body_Data_t));
  }
}

// 2. UART(Vision) 데이터가 도착하면 실행되는 함수
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t rx_index = 0;
  static uint8_t rx_buffer[10];

  if (huart->Instance == USART1)
  {

    if (rx_index == 0)
    {
      if (rx_byte == 0xFF)
      {
        rx_buffer[rx_index++] = rx_byte;
      }
    }
    else
    {
      rx_buffer[rx_index++] = rx_byte;


      if (rx_index >= 10)
      {
    	memcpy(&vision_rx_packet, rx_buffer, 10);
        rx_index = 0; // 초기화
      }
    }

    // 다음 바이트 수신 대기
//    HAL_UART_Receive_IT(&huart1, (uint8_t*)&vision_rx_packet, sizeof(Vision_UART_Packet_t));
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
  }
}

/* USER CODE BEGIN 4 */

// ... (기존 콜백 함수들은 그대로 두세요) ...

// CAN 송신 함수 (코드를 깔끔하게 하기 위해 분리)
void Send_System_State_To_CAN(uint8_t state, uint8_t perclos)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0,}; // 0으로 초기화
    uint32_t TxMailbox;

    // === CAN 메시지 설정 ===
    TxHeader.StdId = 0x100;         // Gateway의 메시지 ID (예: 0x100)
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;               // 데이터 길이: 8바이트만 사용
    TxHeader.TransmitGlobalTime = DISABLE;

    // === 데이터 채우기 ===
    TxData[0] = state;    // Byte 0: 현재 상태 (0:Normal, 1:Warning, 2:Danger)
    TxData[1] = perclos;  // Byte 1: 졸음 수치 (0~100)
    // 나머지 TxData[2]~[7]은 0

    // === 발사! ===
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        // 주석 해제!
        printf("CAN Tx Error (Mailbox Full or Bus Off)\r\n");
    }
    else
    {
        // 성공 시에도 출력해서 확인
        printf("CAN Tx Success! (Sent to Mailbox)\r\n");
    }
}

void Update_System_State()
{
    // 1. 판단 로직 (기존과 동일)
    uint8_t is_drowsy_vision = (vision_rx_packet.perclos > 80) || (vision_rx_packet.eye_state);
    uint8_t is_unstable_steering = (chassis_info.steering_std_dev > 30);

    // 상태 천이 로직
    if (is_drowsy_vision && is_unstable_steering)
    {
        if (current_state != STATE_DANGER) printf(">>> DETECTED: DANGER!\r\n");
        current_state = STATE_DANGER;
    }
    else if (is_drowsy_vision || is_unstable_steering)
    {
        if (current_state != STATE_WARNING) printf(">>> DETECTED: WARNING!\r\n");
        current_state = STATE_WARNING;
    }
    else
    {
        if (current_state != STATE_NORMAL) printf(">>> RECOVERED: Normal\r\n");
        current_state = STATE_NORMAL;
    }

    // === ★추가된 부분★: 판단 결과를 CAN으로 전송 ===
    // 상태값이나 PERCLOS가 변할 때만 보내는 게 정석이지만,
    // 지금은 테스트니까 쿨타임 없이 매번 보냅니다. (메인 루프 딜레이 따름)
    Send_System_State_To_CAN((uint8_t)current_state, vision_rx_packet.perclos);
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
