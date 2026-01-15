/*
 * sensor_touch.c
 *
 *  Created on: 2026. 1. 14.
 *      Author: DDU
 */

#include "sensor_touch.h"

/**
 * @brief TTP223 센서의 입력을 읽어 상태를 반환
 */
uint8_t Touch_Get_Status(void) {
    // CubeMX에서 설정된 라벨(Touch_I_O_1)을 직접 사용
    GPIO_PinState pinState = HAL_GPIO_ReadPin(Touch_I_O_1_GPIO_Port, Touch_I_O_1_Pin);

    // TTP223은 기본적으로 터치 시 High(SET) 신호를 보냄
    if (pinState == GPIO_PIN_SET) {
        return TOUCH_STATUS_NORMAL;   // 1 반환 (정상)
    } else {
        return TOUCH_STATUS_ABNORMAL; // 0 반환 (비정상)
    }
}
