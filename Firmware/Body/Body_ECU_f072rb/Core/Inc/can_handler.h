/*
 * can_handler.h
 *
 *  Created on: 2026. 1. 15.
 *      Author: DDU
 */

#ifndef INC_CAN_HANDLER_H_
#define INC_CAN_HANDLER_H_

#include "main.h"

void CAN_Config_Filter(void); // CAN 필터 설정 (수신 안 하더라도 기본 설정 필요)
void CAN_Tx_SensorData(int8_t head_delta, uint8_t hands_off_val, uint8_t raw_dist, uint8_t touch_raw);

#endif /* INC_CAN_HANDLER_H_ */
