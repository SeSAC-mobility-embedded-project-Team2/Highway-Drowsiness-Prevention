/*
 * can_test.c
 *
 *  Created on: 2026. 1. 16.
 *      Author: DDU
 */


#include "can_test.h"
#include <stdio.h>

CAN_RxTest_t rx_test_data = {0};

/**
 * @brief ID 201h 메시지만 수신하도록 필터 설정
 */
void CAN_Test_Config_Filter(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef sFilterConfig;

    // ID 201h (Standard ID) 설정
    // 0x201을 32비트 필터 레지스터 형식(왼쪽으로 21비트 시프트)으로 변환
    uint32_t filter_id = (0x201 << 21);
    uint32_t filter_mask = (0x7FF << 21); // 모든 비트 검사

    sFilterConfig.FilterBank = 1;             // 기존 0번과 겹치지 않게 1번 뱅크 사용
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (uint16_t)(filter_id >> 16);
    sFilterConfig.FilterIdLow = (uint16_t)(filter_id & 0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (uint16_t)(filter_mask >> 16);
    sFilterConfig.FilterMaskIdLow = (uint16_t)(filter_mask & 0xFFFF);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        printf("[CAN TEST] Filter Config Error!\r\n");
    }
}

/**
 * @brief FIFO0에 대기 중인 메시지가 있는지 확인하고 수신 처리
 */
void CAN_Test_Receive_Check(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // FIFO 0에 수신된 메시지가 있는지 확인
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
            // 데이터 저장
            rx_test_data.id = rxHeader.StdId;
            rx_test_data.dlc = rxHeader.DLC;
            rx_test_data.count++;
            for(int i=0; i<8; i++) rx_test_data.data[i] = rxData[i];

            // 시리얼 로그 출력 (ID 201h 확인)
            printf("[RX TEST] ID:0x%03lX | Count:%lu | Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    rx_test_data.id, rx_test_data.count,
                    rxData[0], rxData[1], rxData[2], rxData[3],
                    rxData[4], rxData[5], rxData[6], rxData[7]);
        }
    }
}
