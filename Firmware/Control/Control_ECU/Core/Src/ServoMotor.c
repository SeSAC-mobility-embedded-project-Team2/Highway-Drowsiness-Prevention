/*
 * ServoMotor.c
 *
 *  Created on: 2026. 1. 14.
 *      Author: jw007
 */

#include "ServoMotor.h"

static TIM_HandleTypeDef *pServoTimer;

// 초기값을 알 수 없는 상태(예: 0xFF)로 두어 첫 신호에 반응하게 함
static uint8_t lastStatus = 0xFF;


/**
  * @brief 서보모터 초기화 및 PWM 시작
  */
void Servo_Init(TIM_HandleTypeDef *htim) {
    pServoTimer = htim;

    // TIM2 Channel 2 PWM 시작
    HAL_TIM_PWM_Start(pServoTimer, TIM_CHANNEL_2);

    // 초기 상태: 창문 닫힘 (0도)
    Servo_Stop();

    lastStatus = 0;
}

/**
  * @brief PWM Compare 값 변경 (각도 조절)
  */
void Servo_SetAngle(uint32_t value) {
    // TIM2_CH2의 Compare 레지스터 값을 업데이트
    __HAL_TIM_SET_COMPARE(pServoTimer, TIM_CHANNEL_2, value);

    /**
     * __HAL_TIM_SET_COMPARE는
     */
}

void Servo_Stop(void)
{
	Servo_SetAngle(SERVO_STOP_VALUE); // 함수 재사용
}

/**
  * @brief 시스템 상태에 따른 서보모터 제어 (ICD 기준)
  * - 정상(0): 창문 닫힘
  * - 주의(1): 창문 90도 개방 (환기)
  * - 위험(2): 창문 개방 상태 유지
  */
void Servo_Update(uint8_t currentStatus) {

	// 이전 상태와 현재 상태가 같다면 아무것도 하지 않고 리턴 (핵심!)
	if (currentStatus == lastStatus) return;

    // 1. 정상 -> 주의/위험 (창문 열기)
    if (lastStatus == 0 && (currentStatus == 1 || currentStatus == 2))
    {
    	Servo_SetAngle(SERVO_SPEED_CW); 		 	// 정방향 저속 회전
        HAL_Delay(ROTATION_TIME_OPEN); 	// 90도만큼 돌 때까지 대기
        Servo_Stop();
    }

    // 2. 주의/위험 -> 정상 (창문 닫기)
    else if ((lastStatus == 1 || lastStatus == 2) && currentStatus == 0)
    {
    	Servo_SetAngle(SERVO_SPEED_CCW); 			// 역방향 저속 회전
        HAL_Delay(ROTATION_TIME_CLOSE); 	// 90도만큼 돌 때까지 대기
        Servo_Stop();
    }

    // 처리가 끝난 후 현재 상태를 마지막 상태로 저장
    lastStatus = currentStatus;
}
