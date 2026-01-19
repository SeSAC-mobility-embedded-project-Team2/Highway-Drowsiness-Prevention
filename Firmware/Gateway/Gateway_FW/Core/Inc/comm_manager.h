/* Core/Inc/comm_manager.h */
#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include "main.h"        // HAL 라이브러리 (CAN/UART 핸들용)
#include "gateway_defines.h" // 데이터 구조체 정의

// UART 수신 처리 (Vision -> Gateway)
void DMS_Process_UART_Data(UART_HandleTypeDef *huart, uint8_t *buffer);

// CAN 수신 처리 (Chassis/Body -> Gateway)
void DMS_Process_CAN_Data(CAN_RxHeaderTypeDef *header, uint8_t *data);

// Control ECU로 명령 전송 (UART)
void DMS_Send_Control_Signal(UART_HandleTypeDef *huart, SystemState_t state, uint8_t mrm_active, uint8_t err_flag);

void DMS_Send_Dashboard_Data(UART_HandleTypeDef *huart, uint8_t risk_score);

extern uint8_t g_last_tx_packet[10];

#endif /* COMM_MANAGER_H */
