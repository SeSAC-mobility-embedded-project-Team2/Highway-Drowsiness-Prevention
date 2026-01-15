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
    Body_Data_t body_msg = {0}; // 8바이트 구조체 초기화

    /* --- [1단계: 구조체 필드 매핑 (Byte 0 ~ 3)] --- */
    // Byte 0: 머리 위치 변화량 (2의 보수 형식으로 음수 표현 가능)
    body_msg.distance_head = (int8_t)head_delta;

    // Byte 1: 터치 상태 (1: 접촉, 0: 미접촉)
    body_msg.touch_handle = (touch == 1) ? 1 : 0;

    // Byte 2: 메시지 생존 카운터 (0~15 순환하며 수신측에서 데이터 갱신 확인)
    body_msg.alive_cnt = body_alive_counter;

    // Byte 3: 에러 플래그 (임계치 ±30cm 초과 시 1, 정상 시 0)
    body_msg.err_flag = (head_delta > 30 || head_delta < -30) ? 1 : 0;

    /* --- [2단계: 직접 메모리 접근 매핑 (Byte 4 ~ 7)] --- */
    // 구조체 뒤쪽의 예약된 공간(Reserved)에 직접 바이트 단위로 데이터를 주입합니다.
    uint8_t *pPayload = (uint8_t*)&body_msg;

    // Byte 4 & 5: 기준 거리 (Baseline) - Little Endian 방식
    pPayload[4] = (uint8_t)(baseline & 0xFF);        // 하위 바이트 (LSB)
    pPayload[5] = (uint8_t)((baseline >> 8) & 0xFF); // 상위 바이트 (MSB)

    // Byte 6 & 7: CO2 농도 값 - Little Endian 방식
    pPayload[6] = (uint8_t)(co2 & 0xFF);             // 하위 바이트 (LSB)
    pPayload[7] = (uint8_t)((co2 >> 8) & 0xFF);      // 상위 바이트 (MSB)

    /* --- [3단계: CAN 통신 헤더 설정] --- */
    TxHeader.StdId = CAN_ID_BODY; // 전송 ID 설정 (0x301)
    TxHeader.RTR = CAN_RTR_DATA;  // 데이터 프레임 사용
    TxHeader.IDE = CAN_ID_STD;    // 표준 ID (11비트) 사용
    TxHeader.DLC = 8;             // 데이터 길이는 무조건 8바이트

    /* --- [4단계: 실제 전송 프로세스] --- */
    // 송신 우편함(Mailbox)에 빈 자리가 있는지 확인
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        // 메시지를 CAN 버스로 발사
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, (uint8_t*)&body_msg, &TxMailbox) == HAL_OK) {
            // 전송 성공 시 롤링 카운터 업데이트
            body_alive_counter = (body_alive_counter + 1) % 16;

            // 시리얼 모니터로 전송 상태 모니터링
            printf("[CAN TX OK] Delta:%d | Base:%u | CO2:%u | Touch:%d\r\n",
                    head_delta, baseline, co2, touch);
        }
    }
}
