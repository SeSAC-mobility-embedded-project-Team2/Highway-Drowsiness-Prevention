/*
 * DC_Motor.h
 *
 *  Created on: 2026. 1. 15.
 *      Author: jw007
 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

#include "main.h"

/* 속도 설정 (ARR: 999 기준) */
#define MOTOR_SPEED_NORMAL  900   // 정상 주행 속도 (90%)
#define MOTOR_SPEED_SAFE    300   // 위험 시 안전 속도 (30%)
#define DECEL_TIME_MS       3000  // 감속 소요 시간 (3초)
#define STEP_INTERVAL_MS    50    // 속도 업데이트 간격 (50ms 마다)

/* 함수 선언 */
void DC_Motor_Init(TIM_HandleTypeDef *htim);
void DC_Motor_Update(uint8_t currentStatus);
void DC_Motor_SetSpeed(uint32_t speed);

#endif /* INC_DC_MOTOR_H_ */
