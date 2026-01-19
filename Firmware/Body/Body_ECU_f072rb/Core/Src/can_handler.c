/*
 * can_handler.c
 *
 *  Created on: 2026. 1. 14.
 *      Author: DDU
 */

#include "can_handler.h"
#include "can_message.h"
#include <string.h>
#include <stdio.h>

extern CAN_HandleTypeDef hcan;
static uint8_t body_alive_counter = 0;

// CAN 필터 설정 및 시작 (기존과 동일)
void CAN_Config_Filter(void) {
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    HAL_CAN_Start(&hcan);
}

/**
 * @brief Body ECU 센서 데이터 CAN 전송 (ID: 0x301)
 * @param head_delta: 머리 위치 변화량 (int8_t)
 * @param hands_off_val: 손 뗀 시간 (0.1s 단위)
 * @param raw_dist: 초음파 원본 거리 (uint8_t)
 * @param touch_raw: 터치 센서 원본 (0 or 1)
 */
/**
 * @brief Body ECU 센서 데이터 CAN 전송 (ID: 0x301)
 */
void CAN_Tx_SensorData(int8_t head_delta, uint8_t hands_off_val, uint8_t raw_dist, uint8_t touch_raw) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t tx_data[8] = {0};

    // 8바이트 배열을 Body_Data_t 구조체 포인터로 매핑
    Body_Data_t *pMsg = (Body_Data_t *)tx_data;

    // 1. 데이터 매핑 (ICD V1.1 멤버 이름 적용)
    // head_delta(변화량)을 distance_head 필드에 할당
    pMsg->distance_head = (uint8_t)head_delta;      // Byte 0

    // touch_raw(현재 터치 여부)를 touch_handle 비트에 할당
    pMsg->touch_handle = (touch_raw > 0) ? 1 : 0;   // Byte 1, bit 0

    // hands_off_val(누적 시간)과 raw_dist는 reserved 공간을 활용하여 전송
    pMsg->reserved[0] = hands_off_val;              // Byte 2
    pMsg->reserved[1] = raw_dist;                   // Byte 3

    // 2. 에러 플래그 (SonarFail 판단)
//	uint8_t sonar_fail = (raw_dist == 0 || raw_dist > 400) ? 1 : 0;

    // 3. Safety Fields (Byte 7) 업데이트
    pMsg->alive_cnt = (body_alive_counter & 0x0F);
//    pMsg->err_flag = (current_err & 0x0F);

    // 4. Alive Counter 업데이트 (0~15 순환)
    body_alive_counter = (body_alive_counter + 1) % 16;

    // 5. CAN 전송 헤더 설정
    TxHeader.StdId = CAN_ID_BODY; // 0x301
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 6. CAN 메시지 전송
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_data, &TxMailbox) != HAL_OK) {
        // 전송 에러 시 처리 (필요 시 작성)
    }
}
