/*
 * Buzzer.h
 *
 *  Created on: 2026. 1. 13.
 *      Author: jw007
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "main.h"

/* 부저 제어 설정 */
#define BUZZER_GPIO_PORT  BUZZER_GPIO_Port  // PA0 포트
#define BUZZER_GPIO_PIN   BUZZER_Pin        // PA0 핀

/* 함수 선언 (외부 main.c에서 불러다 쓸 함수들) */
void Buzzer_Init(void);                      // 부저 초기 설정
void Buzzer_Update(uint8_t systemStatus);    // 상태에 따라 소리 패턴 변경 [cite: 69]
void Buzzer_Off(void);                       // 부저 즉시 끄기

#endif /* INC_BUZZER_H_ */
