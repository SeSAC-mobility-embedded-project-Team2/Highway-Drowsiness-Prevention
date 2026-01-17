/*
 * PC_Comm.h
 *
 *  Created on: 2026. 1. 17.
 *      Author: jw007
 */

#ifndef INC_PC_COMM_H_
#define INC_PC_COMM_H_

#include "main.h"

/* UART2 관련 인터페이스 함수 */
void PC_Comm_Init(UART_HandleTypeDef *huart);
void PC_Comm_HandleByte(uint8_t byte); // 인터럽트 콜백에서 호출
void PC_Comm_Process(void);            // 메인 루프에서 호출

#endif /* INC_PC_COMM_H_ */
