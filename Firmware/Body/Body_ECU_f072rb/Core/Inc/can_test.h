/*
 * can_test.h
 *
 *  Created on: 2026. 1. 16.
 *      Author: DDU
 */

#ifndef INC_CAN_TEST_H_
#define INC_CAN_TEST_H_

#include "main.h"

// 테스트용 수신 데이터 구조체 (ID 201h용)
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint32_t count;      // 수신된 총 메시지 개수
} CAN_RxTest_t;

// 함수 프로토타입
void CAN_Test_Config_Filter(CAN_HandleTypeDef *hcan);
void CAN_Test_Receive_Check(CAN_HandleTypeDef *hcan);

#endif /* INC_CAN_TEST_H_ */
