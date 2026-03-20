/*
 * sensor_touch.h
 *
 *  Created on: 2026. 1. 14.
 *      Author: DDU
 */

#ifndef INC_SENSOR_TOUCH_H_
#define INC_SENSOR_TOUCH_H_

#include "main.h"

/* 터치 상태 정의 (사용자 가독성을 위해 상수로 정의) */
#define TOUCH_STATUS_ABNORMAL 0  // 손을 뗌 (비정상)
#define TOUCH_STATUS_NORMAL   1  // 잡고 있음 (정상)

/**
 * @brief 터치 센서(PA1)의 상태를 읽어 정상/비정상 여부를 반환합니다.
 * @return 1 (Normal), 0 (Abnormal)
 */
uint8_t Touch_Get_Status(void);

#endif /* INC_SENSOR_TOUCH_H_ */
