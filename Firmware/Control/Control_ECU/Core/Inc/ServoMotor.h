/*
 * ServoMotor.h
 *
 *  Created on: 2026. 1. 14.
 *      Author: jw007
 */

#ifndef INC_SERVOMOTOR_H_
#define INC_SERVOMOTOR_H_

#include "main.h"

/* 무한 회전 서보모터 제어 설정 */

/*
 * 서보 모터는 20ms(50Hz) 주기로 신호를 받고, 1.5ms 동안 high 신호일 때 중앙 인식
 *
 * <설정값>
 * ARR -> 19999 + 1 -> 20000
 * 20000칸이 시간으로 치면 20ms -> 1ms는 1000칸
 * 표준 중립 신호 1.5ms 는 칸수로 1500 임
 *
 */


/* 무한 회전 서보모터 제어 설정 */
#define SERVO_STOP_VALUE    1500  // 실험을 통해 멈추는 값으로 수정 (예: 1485)
#define SERVO_SPEED_CW      1600  // 정방향 회전 속도
#define SERVO_SPEED_CCW     1400  // 역방향 회전 속도

/* PC_Comm에서 수정할 수 있도록 추가할 매크로 및 전역 변수 선언 */
#define DEFAULT_OPEN_TIME  1180  // 90도 회전 소요 시간(ms)
#define DEFAULT_CLOSE_TIME 1302	 // 1300이 정배

// 창문의 물리적 상태 정의
#define WINDOW_CLOSED       0
#define WINDOW_OPEN         1

// PC_Comm(USRAT2 전용)에서 수정할 수 있도록 전역 변수로 선언
extern uint32_t g_rotation_time_open;
extern uint32_t g_rotation_time_close;

/* 함수 선언 */
void Servo_Init(TIM_HandleTypeDef *htim);
void Servo_Update(uint8_t systemStatus);
void Servo_SetAngle(uint32_t value);
void Servo_Stop(void);

#endif /* INC_SERVOMOTOR_H_ */
