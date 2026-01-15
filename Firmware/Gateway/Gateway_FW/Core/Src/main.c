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
#define ABS(x) ((x) > 0 ? (x) : -(x)) // 절대값 계산용 매크로 (math.h 없어도 됨)
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

int16_t prev_steering_angle = 0;
uint32_t no_op_timer = 0; // 무조작 시간 카운터

// 1. CAN 수신용 변수
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// 전역 변수 (CAN 수신값 저장용)
uint8_t  can_perclos = 0;
float    can_steer_std = 0.0f;
float    can_hands_off_sec = 0.0f;
float    can_head_delta = 0.0f;

// 2. UART(Vision) 수신용 변수
uint8_t rx_byte; // 1바이트씩 검사할 임시 변수
Vision_UART_Packet_t vision_rx_packet; // 최종 저장할 구조체
uint8_t uart_rx_buffer[10]; // 수신 버퍼
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
//    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 8);
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


// ==========================================
// [CAN Logic Engine Final]
// ICD V0.1.1 기반 CAN 통신 로직
// ==========================================

// 1. CAN 메시지가 도착하면 실행되는 함수
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) return;

    // === 2. Chassis Node (0x201) ===
    else if (RxHeader.StdId == 0x201)
    {
        // Byte 0-1: Steering_StdDev (uint16_t, Factor 0.01)
        uint16_t raw_std = (RxData[1] << 8) | RxData[0];
        can_steer_std = raw_std * 0.01f; // 2000 -> 20.0 변환
    }

    // === 3. Body Node (0x301) - 전처리된 센서 값 ===
    else if (RxHeader.StdId == 0x301)
    {
        // Byte 0: Head_Delta_cm (int8_t, Factor 1)
        int8_t raw_head = (int8_t)RxData[0];
        can_head_delta = (float)raw_head;

        // Byte 1: Hands_Off_Time (uint8_t, Factor 0.1)
        uint8_t raw_time = RxData[1];
        can_hands_off_sec = raw_time * 0.1f; // 50 -> 5.0초 변환
    }


}

// UART 수신 완료 콜백 (인터럽트)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Vision 센서가 연결된 UART 채널인지 확인 (예: huart1)
    if (huart->Instance == USART1)
    {
        // ICD V0.1.1 파싱 (순서대로 매핑)
        // [Byte 0] PERCLOS
        vision_rx_packet.perclos = uart_rx_buffer[0];

        // [Byte 1] Eye_State (Bit 0), Face_Detect (Bit 1)
        uint8_t flags = uart_rx_buffer[1];
        vision_rx_packet.eye_state = (flags & 0x01);       // 0번 비트
        vision_rx_packet.face_flag = (flags >> 1) & 0x01; // 1번 비트

        // [Byte 7] Alive Count (하위 4비트), Err Flag (상위 4비트)
        uint8_t status = uart_rx_buffer[7];
        vision_rx_packet.alive_cnt = status & 0x0F;
        vision_rx_packet.err_flag = (status >> 4) & 0x0F;

        // 다음 데이터 수신 대기 (필수!)
        HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 8); // 8바이트씩 수신
    }
}

//// 2. UART(Vision) 데이터가 도착하면 실행되는 함수
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  static uint8_t rx_index = 0;
//  static uint8_t rx_buffer[10];
//
//  if (huart->Instance == USART1)
//  {
//
//    if (rx_index == 0)
//    {
//      if (rx_byte == 0xFF)
//      {
//        rx_buffer[rx_index++] = rx_byte;
//      }
//    }
//    else
//    {
//      rx_buffer[rx_index++] = rx_byte;
//
//
//      if (rx_index >= 10)
//      {
//    	memcpy(&vision_rx_packet, rx_buffer, 10);
//        rx_index = 0; // 초기화
//      }
//    }
//
//    // 다음 바이트 수신 대기
////    HAL_UART_Receive_IT(&huart1, (uint8_t*)&vision_rx_packet, sizeof(Vision_UART_Packet_t));
//    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
//  }
//}

// ==========================================
// [Fuzzy Logic Engine Final]
// ICD V0.1.1 기반 통합 졸음 판단
// ==========================================

// 1. 퍼지 멤버십 함수 (사다리꼴)
// 입력값 x가 [a, b] 구간에서 0->1로 상승, [c, d] 구간에서 1->0으로 하강
// 여기서는 위험도 계산이므로 b 이후로는 계속 1.0 유지하는 형태(Open Right)를 주로 씀
float Fuzzy_Trapezoid(float x, float a, float b)
{
    if (x <= a) return 0.0f;           // 안전 구간
    if (x >= b) return 1.0f;           // 위험 구간 (Max)
    return (x - a) / (b - a);          // 위험도 상승 구간 (0.0 ~ 1.0)
}

// 2. 통합 위험도 계산 함수
// 모든 입력값은 Factor가 적용된 "실수(float)" 형태여야 함
uint8_t Compute_Integrated_Risk(uint8_t perclos, float steer_std, float hands_off_sec, float head_delta, float no_op_sec)
{
    //============================[Step 1] Fuzzification (입력값 -> 위험도 0.0~1.0 변환)==================

    // 1. Vision (PERCLOS): 40% 부터 위험 시작, 60%면 만점
    float score_eye = Fuzzy_Trapezoid((float)perclos, 40.0f, 60.0f);

    // 2. Chassis (Steering): 표준편차 20부터 위험 시작, 40이면 만점
    float score_steer = Fuzzy_Trapezoid(steer_std, 20.0f, 40.0f);

    // 3. Body (Hands Off): 2.0초부터 위험 시작, 5.0초면 만점
    // [근거] NHTSA 기준 2초 이상 주시 태만 위험
    float score_hands = Fuzzy_Trapezoid(hands_off_sec, 2.0f, 5.0f);

    // 4. Body (Head Delta): 5cm부터 위험 시작, 15cm면 만점
    // 숙이거나(-) 젖히거나(+) 모두 위험하므로 절대값 사용
    float abs_head = (float)ABS(head_delta);
    float score_head = Fuzzy_Trapezoid(abs_head, 5.0f, 15.0f);

    // 10초 이상 가만히 있으면 점수가 오르기 시작해서 15초면 50점(주의) 정도 줌. 10초~15초 사이 주의 단계 상승
    float score_noop  = Fuzzy_Trapezoid(no_op_sec, 10.0f, 20.0f) * 0.6f; // 최대 60점까지만 (경고 수준)


    //================================[Step 2] Rule Evaluation (규칙 적용)=============================

    // === [솔루션 1] 눈부심 방지 (False Alarm Rejection) ===
    // 눈은 감겼는데(1.0), 핸들/손/머리가 너무 멀쩡하면(0.2 이하) -> 눈부심으로 간주하고 점수 삭감
    if (score_eye > 0.8f && score_steer < 0.2f && score_hands < 0.2f && score_head < 0.2f)
    {
        score_eye *= 0.3f; // 점수를 30%로 깎아버림 (Normal 유지)
    }

    // [Rule 1] 서서히 오는 졸음 (눈 + 핸들)
    // 눈도 감기고 핸들도 흔들리면 위험도 증가 (Max 연산)
    float chronic_drowsiness = (score_eye > score_steer) ? score_eye : score_steer;

    // [Rule 2] 급박한 위험 (손 뗌 OR 고개 푹)
    // 이 둘 중 하나라도 발생하면 즉시 위험도 100%로 치솟아야 함
    float acute_danger = (score_hands > score_head) ? score_hands : score_head;

    // ★ 무조작 룰 통합
    // 무조작은 '은근한 졸음'이므로 chronic에 포함
    if (score_noop > chronic_drowsiness) chronic_drowsiness = score_noop;

    //=================================[Step 3] Defuzzification (최종 점수 산출)=========================

    // 안전 최우선: "만성 졸음"과 "급박한 위험" 중 더 높은 점수 채택
    float final_risk = (chronic_drowsiness > acute_danger) ? chronic_drowsiness : acute_danger;

    // 0~100점으로 변환
    return (uint8_t)(final_risk * 100.0f);

}

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

// 메인 루프에서 호출하는 함수
void Update_System_State()
{

    // === [솔루션 3] 고장 감지 (Fail-Safe) ===
    // 비전이나 섀시 쪽에서 에러 플래그가 하나라도 0이 아니면 고장 처리
    if (vision_rx_packet.err_flag != 0 || chassis_info.err_flag != 0)
    {
        printf("🔧 SENSOR ERROR DETECTED! (Fail-Safe Mode)\r\n");
        // 여기서 별도의 LED를 켜거나 기능을 제한할 수 있음
        return; // 로직 중단
    }

    // === [솔루션 2] 무조작(No-Op) 감지 ===
    // 현재 조향각(ICD V0.1의 Steering_Angle_Cur 사용 가정)
    // 변화량이 2도 미만이면 타이머 증가
    int16_t diff = chassis_info.steering_angle - prev_steering_angle;
    if (diff < 0) diff = -diff; // 절대값

    if (diff < 20) // 2.0도 미만 (Factor 0.1 가정 시 값 20)
    {
        no_op_timer += 100; // 100ms 증가
    }
    else
    {
        no_op_timer = 0; // 움직임 감지되면 리셋
        prev_steering_angle = chassis_info.steering_angle; // 기준점 갱신
    }

    // 10초 이상 무조작이면 '좀비/눈뜬졸음' 의심 (Factor 0.1 -> 10.0초)
    float no_op_sec = no_op_timer / 1000.0f;



    // 퍼지 로직 계산
    uint8_t risk_score = Compute_Integrated_Risk(
                            can_perclos,
                            can_steer_std,
                            can_hands_off_sec,
                            can_head_delta,
							no_op_sec
                         );

    // 상태 천이 (Hysteresis 적용)
    switch (current_state)
    {
        case STATE_NORMAL:
            if (risk_score >= 80) current_state = STATE_WARNING; // 80점 이상 주의
            break;

        case STATE_WARNING:
            if (risk_score >= 95) current_state = STATE_DANGER;  // 95점 이상 위험
            else if (risk_score < 60) current_state = STATE_NORMAL; // 복귀
            break;

        case STATE_DANGER:
            if (risk_score < 85) current_state = STATE_WARNING;
            break;
        case STATE_FAULT:
        	break;
    }

    // 디버깅용 출력
    printf("Risk: %d (NoOp: %.1fs)\r\n", risk_score, no_op_sec);
    printf("Risk: %d (Hands: %.1fs, Head: %.1fcm)\r\n", risk_score, can_hands_off_sec, can_head_delta);
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
