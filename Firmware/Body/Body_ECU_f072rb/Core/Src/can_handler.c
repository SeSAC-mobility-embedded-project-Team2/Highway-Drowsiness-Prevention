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
 * @brief 명세서(0x301) 기준 CAN 전송
 * @param head_delta: 머리 위치 변화량 (int8_t)
 * @param hands_off_val: 손 뗀 시간 (0.1s 단위, 50 = 5.0초)
 * @param raw_dist: 초음파 원본 거리 (uint8_t)
 * @param touch_raw: 터치 센서 원본 (0 or 1)
 */
void CAN_Tx_SensorData(int8_t head_delta, uint8_t hands_off_val, uint8_t raw_dist, uint8_t touch_raw) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t tx_data[8] = {0}; // 8바이트 데이터 필드

    /* --- [명세서 기반 데이터 매핑] --- */

    // Byte 0: Head_Delta_cm (int8_t, Factor 1)
    tx_data[0] = (int8_t)head_delta;

    // Byte 1: Hands_Off_Time (uint8_t, Factor 0.1)
    // 수신측에서 이 값에 0.1을 곱함. (예: 50 전송 시 5.0초로 인식)
    tx_data[1] = hands_off_val;

    // Byte 2: Sonar_Raw_Dist (uint8_t, Factor 1)
    tx_data[2] = raw_dist;

    // Byte 3: Touch_Raw_Val (Bit 0 사용)
    tx_data[3] = (touch_raw & 0x01);

    // Byte 4~6: Reserved (공백)

    // Byte 7: DMS_Alive_Cnt(Bit 0~3) & DMS_Err_Flag(Bit 4~7)
    // 에러 상태: 0:OK, 1:SonarFail, 2:TouchFail
    uint8_t err_flag = 0;
    if (raw_dist == 0 || raw_dist > 400) err_flag = 1; // 초음파 이상 예시

    tx_data[7] = (body_alive_counter & 0x0F) | ((err_flag & 0x0F) << 4);

    /* --- [CAN 전송 설정] --- */
    TxHeader.StdId = 0x301;
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_data, &TxMailbox) == HAL_OK) {
            body_alive_counter = (body_alive_counter + 1) % 16;
        }
    }
}
