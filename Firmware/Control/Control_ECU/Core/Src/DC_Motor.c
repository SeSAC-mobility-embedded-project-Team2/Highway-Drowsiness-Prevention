#include "DC_Motor.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2; // main.c의 UART 핸들러 사용

/* 정적 변수 정의 */
static TIM_HandleTypeDef *pMotorTimer;
static uint32_t lastSpeedTick = 0;
static uint16_t currentSpeed = MOTOR_SPEED_NORMAL;
static uint8_t isDecelerating = 0;
static uint8_t lastStatus = 0xFF;

static char debugBuf[128]; // 디버그 문자열 버퍼

/**
  * @brief DC 모터 초기화 및 디버그 출력
  */
void DC_Motor_Init(TIM_HandleTypeDef *htim) {
    pMotorTimer = htim;

    // GPIO 초기 설정
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // DIR: High (정방향)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // BRK: Low (해제)

    HAL_TIM_PWM_Start(pMotorTimer, TIM_CHANNEL_1);

    sprintf(debugBuf, "\r\n[MOTOR] Init Complete. DIR: SET, BRK: RESET\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)debugBuf, strlen(debugBuf), 10);

    DC_Motor_SetSpeed(MOTOR_SPEED_NORMAL);
}

/**
  * @brief 속도 설정 및 실제 레지스터/핀 상태 확인
  */
void DC_Motor_SetSpeed(uint32_t speed) {
    __HAL_TIM_SET_COMPARE(pMotorTimer, TIM_CHANNEL_1, speed);

    // 실제 하드웨어 핀 상태 읽기 (교차 검증)
    GPIO_PinState dir = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
    GPIO_PinState brk = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);

    // [핵심 디버그] 현재 속도와 물리 핀 상태 출력
    sprintf(debugBuf, "[MOTOR LOG] Speed: %lu, DIR: %s, BRK: %s\r\n",
            speed,
            (dir == GPIO_PIN_SET) ? "HIGH" : "LOW",
            (brk == GPIO_PIN_SET) ? "HIGH" : "LOW");
    HAL_UART_Transmit(&huart2, (uint8_t*)debugBuf, strlen(debugBuf), 10);
}

/**
  * @brief 시스템 상태에 따른 감속 로직 및 디버그
  */
void DC_Motor_Update(uint8_t currentStatus) {
    // 1. 상태가 변할 때 알림
    if (currentStatus != lastStatus) {
        sprintf(debugBuf, ">> [MOTOR] Status Trigger: %d -> %d\r\n", lastStatus, currentStatus);
        HAL_UART_Transmit(&huart2, (uint8_t*)debugBuf, strlen(debugBuf), 10);

        if (currentStatus == 2) { // DANGER
            isDecelerating = 1;
            lastSpeedTick = HAL_GetTick();
        }
        else if (currentStatus == 0) { // NORMAL
            isDecelerating = 0;
            currentSpeed = MOTOR_SPEED_NORMAL;
            DC_Motor_SetSpeed(currentSpeed);
        }
        lastStatus = currentStatus;
    }

    // 2. 선형 감속 프로세스 디버그 (500ms 마다 진행 상황 출력)
    if (isDecelerating && currentStatus == 2) {
        if (HAL_GetTick() - lastSpeedTick >= STEP_INTERVAL_MS) {
            lastSpeedTick = HAL_GetTick();

            if (currentSpeed > MOTOR_SPEED_SAFE) {
                currentSpeed -= 10;
                if (currentSpeed < MOTOR_SPEED_SAFE) currentSpeed = MOTOR_SPEED_SAFE;

                DC_Motor_SetSpeed(currentSpeed);
            } else {
                isDecelerating = 0;
                sprintf(debugBuf, "[MOTOR] Deceleration Finished. Safe Speed Maintained.\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)debugBuf, strlen(debugBuf), 10);
            }
        }
    }
}
