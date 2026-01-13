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
#include "stm32f1xx_nucleo.h"
#include "Buzzer.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFSIZE 128
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*------------------------- 전역 변수 및 상태 정의 추가 -----------------------------*/
// 시스템 상태 정의
typedef enum {
	STATUS_NORMAL  = 0,
	STATUS_WARNING = 1,
	STATUS_DANGER  = 2,
	STATUS_FAULT   = 3
} SystemStatus_t;

// 현재 시스템 상태 플래그
SystemStatus_t currentSystemStatus = STATUS_NORMAL;
uint8_t mrmTriggerFlag = 0;		// MRM : Minimum Risk Maneuver, '최소 위험 기동'

// UART 패킷 수신용
uint8_t rxPacket[4];				// [Header, Alert_Level, MRM_Trigger, Checksum]
uint8_t rxIndex = 0;				// 수신 바이트 위치 카운트 ... ?
uint8_t uart_rcvbyte;				// 1바이트 임시 수신용
volatile uint8_t bPacketReady = 0;	// 패킷 완성 플래그

/*------------------------- 전역 변수 및 상태 정의 추가 -----------------------------*/

//for UART
/* buffer */
static uint8_t buff[BUFSIZE] = "Hello World\r\n";
uint8_t uart_rcvbuf;
volatile uint8_t bUART_RX = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*-------------------------- 수신 콜백 함수 수정 (패킷 조립) --------------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		// 1. 헤더 찾기 (패킷의 시작)
		// 시작 신호를 통일 해야함 (0xAA)
		if(rxIndex == 0 && uart_rcvbyte == 0xAA) {
			rxPacket[rxIndex++] = uart_rcvbyte;
		}
		// 2. 나머지 데이터 채우기
		else if(rxIndex > 0) {
			rxPacket[rxIndex++] = uart_rcvbyte;

			// 3. 4바이트 패킷이 완성되었는가?
			if(rxIndex >= 4) {
				bPacketReady = 1;	// 패킷 완성
				rxIndex = 0; 		// 다음 패킷을 위해 인덱스 초기화
			}
		}

		// 다시 1바이트 수신 대기
		HAL_UART_Receive_IT(&huart2, &uart_rcvbyte, 1);
	}
}

/*----------------------------- 수신 콜백 함수 수정 -------------------------------*/

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, buff, strlen((const char *)buff), 100);

  // 최초 수신 시작

  HAL_UART_Receive_IT(&huart2, &uart_rcvbyte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* 에코 로직은 패킷 수신과 충돌할 수 있으므로 잠시 주석 처리 */
	  /*
	  // for UART_RX
	  if(bUART_RX)
	  {
		  bUART_RX = 0;
		  HAL_UART_Transmit(&huart2, &uart_rcvbuf, 1, 300);
		  HAL_UART_Receive_IT(&huart2, &uart_rcvbuf, 1);
	  }
	  */

	  /*----------------------------- 패킷 검사 및 해석 -------------------------------*/

	  if(bPacketReady)
	  {
		  bPacketReady = 0;

		  uint8_t alertLevel = rxPacket[1]; // ICD : Byte 0 (Aleret_Level)
		  uint8_t mrmTrigger = rxPacket[2];	// ICD : Byte 1 (MRM_Trigger)
		  uint8_t checksum 	 = rxPacket[3];

		  // 데이터 무결성 검사 (Header 제외 간단한 합산)
		  if(checksum == (uint8_t)(alertLevel + mrmTrigger))
		  {
			  // 위험 상태 판단 로직
			  if(alertLevel == 2 || mrmTrigger == 1)
			  {
				  currentSystemStatus = STATUS_DANGER;
			  }
			  else if(alertLevel == 1)
			  {
			      currentSystemStatus = STATUS_WARNING;
			  }
			  else if(alertLevel == 3)
			  {
			      currentSystemStatus = STATUS_FAULT;
			  }
			  else
			  {
			      currentSystemStatus = STATUS_NORMAL;
			  }

			  // (디버깅용) 현재 상태를 UART로 출력
			  char msg[30];
			  sprintf(msg, "Status Updated: %d\r\n", currentSystemStatus);
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
		  }
	  }

	  // --- 여기서부터 상태 플래그에 따라 모듈 제어 예정 ---
	  switch(currentSystemStatus)
	  {
	      case STATUS_NORMAL:
	           // TODO: LED 정지, 모터 정상 동작 로직 위치
	           break;
	      case STATUS_WARNING:
	            // TODO: 주의 알람 로직 위치
	    	    break;
	      case STATUS_DANGER:
	            // TODO: 즉시 정지 및 비상등 로직 위치
	    	    break;
	      default:
	            break;
	  }

	  /*----------------------------- 패킷 수신 함수 추가 -------------------------------*/

	  // 업데이트된 상태에 따라 부저 소리 조절
	  Buzzer_Update(currentSystemStatus);

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
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
