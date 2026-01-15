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
 * @brief 센서 데이터를 CAN 8바이트 프레임으로 변환하여 송신 (ID: 0x301)
 * @param head_delta: 거리 변화량 (int8_t, -30 ~ +30 cm)
 * @param touch: 터치 센서 상태 (uint8_t, 0 or 1)
 * @param co2: CO2 농도 (uint16_t, ppm)
 * @param baseline: 3초 캘리브레이션으로 설정된 기준 거리 (uint16_t, cm)
 */
void CAN_Tx_SensorData(int8_t head_delta, uint8_t touch, uint16_t co2, uint16_t baseline) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t tx_data[8] = {0}; // 명세서 규격에 맞춘 8바이트 배열

    /* --- [데이터 매핑 시작] --- */

    // Byte 0: Head_Delta (-30 ~ +30)
    tx_data[0] = (int8_t)head_delta;

    // Byte 1: Touch_Handle (Bit 0 사용)
    tx_data[1] = (touch & 0x01);

    // Byte 2~3: Baseline_Dist (Little Endian)
    tx_data[2] = (uint8_t)(baseline & 0xFF);
    tx_data[3] = (uint8_t)((baseline >> 8) & 0xFF);

    // Byte 4~5: CO2_Value (Little Endian)
    tx_data[4] = (uint8_t)(co2 & 0xFF);
    tx_data[5] = (uint8_t)((co2 >> 8) & 0xFF);

    // Byte 6: (Reserved) - 현재 사용 안 함

    // Byte 7: 상하위 4비트씩 나누어 대입
    // 하위 4비트(0~3): Alive_Cnt
    // 상위 4비트(4~7): Err_Flag
    uint8_t err_status = (head_delta > 30 || head_delta < -30) ? 1 : 0;
    tx_data[7] = (body_alive_counter & 0x0F) | ((err_status & 0x0F) << 4);

    /* --- [CAN 전송 설정] --- */
    TxHeader.StdId = CAN_ID_BODY; // 0x301
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_data, &TxMailbox) == HAL_OK) {
            body_alive_counter = (body_alive_counter + 1) % 16;

            // 데이터 분석용 시리얼 로그
            printf("[CAN TX] Delta:%d | Base:%u | CO2:%u | Alive:%d | Byte7:0x%02X\r\n",
                    head_delta, baseline, co2, body_alive_counter, tx_data[7]);
        }
    }
}
