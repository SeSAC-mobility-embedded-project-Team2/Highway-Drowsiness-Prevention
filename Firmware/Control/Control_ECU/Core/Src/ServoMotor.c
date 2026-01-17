#include "ServoMotor.h"

static TIM_HandleTypeDef *pServoTimer;
static uint8_t currentWindowState = WINDOW_CLOSED; // 물리적 개폐 상태
static uint32_t lastActionTick = 0;                 // 동작 후 보호 시간용

uint32_t g_rotation_time_open = DEFAULT_OPEN_TIME;
uint32_t g_rotation_time_close = DEFAULT_CLOSE_TIME;

void Servo_Init(TIM_HandleTypeDef *htim) {
    pServoTimer = htim;
    HAL_TIM_PWM_Start(pServoTimer, TIM_CHANNEL_2);
    Servo_Stop();
    currentWindowState = WINDOW_CLOSED;
    lastActionTick = 0;
}

void Servo_SetAngle(uint32_t value) {
    if (pServoTimer != NULL) {
        __HAL_TIM_SET_COMPARE(pServoTimer, TIM_CHANNEL_2, value);
    }
}

void Servo_Stop(void) {
    Servo_SetAngle(SERVO_STOP_VALUE);
}

void Servo_Update(uint8_t currentStatus) {
    uint32_t currentTick = HAL_GetTick();

    // [수정] 3초 보호 시간 가드 (3000ms)
    if (currentTick - lastActionTick < 3000) return;

    // 모든 UART2 출력(printf, sprintf, HAL_UART_Transmit)은 삭제하거나 주석 처리합니다.

    // 1. 열기 동작
    if ((currentStatus == 1 || currentStatus == 2) && currentWindowState == WINDOW_CLOSED) {
        Servo_SetAngle(SERVO_SPEED_CW);
        HAL_Delay(g_rotation_time_open);
        Servo_Stop();

        currentWindowState = WINDOW_OPEN;
        lastActionTick = HAL_GetTick();
    }
    // 2. 닫기 동작
    else if (currentStatus == 0 && currentWindowState == WINDOW_OPEN) {
        Servo_SetAngle(SERVO_SPEED_CCW);
        HAL_Delay(g_rotation_time_close);
        Servo_Stop();

        currentWindowState = WINDOW_CLOSED;
        lastActionTick = HAL_GetTick();
    }
}
