/*
 * sensor_ultrasonic.c
 *
 *  Created on: 2026. 1. 14.
 *      Author: DDU
 */

#include "sensor_ultrasonic.h"

extern TIM_HandleTypeDef htim3; // CubeMX에서 설정한 TIM3
Ultrasonic_t hultrasonic = {0};

static uint16_t dynamic_baseline = 10;

// 기준 거리를 설정하는 함수
void Ultrasonic_Set_Baseline(uint16_t dist) {
    if (dist > 0 && dist < 400) {
        dynamic_baseline = dist;
    }
}

// 1. 트리거 신호 발생 (10us High)
void Ultrasonic_Trigger(void) {
    HAL_GPIO_WritePin(Supersonic_trig_GPIO_Port, Supersonic_trig_Pin, GPIO_PIN_SET); // Trig PB6 High
    delay_us(10);
    HAL_GPIO_WritePin(Supersonic_trig_GPIO_Port, Supersonic_trig_Pin, GPIO_PIN_RESET); // Trig PB6 Low
}

uint16_t distance = 0;
// 2. 입력 캡처 인터럽트 처리 (PA6 - TIM3 CH1)
void Ultrasonic_Capture_Callback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (hultrasonic.is_first_captured == 0) { // 상승 엣지 검출
            hultrasonic.capture_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            hultrasonic.is_first_captured = 1;
            // 하강 엣지 캡처로 변경
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else { // 하강 엣지 검출
            hultrasonic.capture_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            // 오버플로우 처리
            if (hultrasonic.capture_end >= hultrasonic.capture_start) {
                hultrasonic.diff_time = hultrasonic.capture_end - hultrasonic.capture_start;
            } else {
                hultrasonic.diff_time = (65535 - hultrasonic.capture_start) + hultrasonic.capture_end;
            }

            // 거리 계산 (1tick = 1us, 왕복 거리이므로 58로 나눔)
            distance = hultrasonic.diff_time / 58;

            // 3. 이동평균 필터 적용 (노이즈 제거)
            if (distance < 400) { // 유효 범위 내 값만 처리
                hultrasonic.filter_sum -= hultrasonic.filter_buf[hultrasonic.filter_idx];
                hultrasonic.filter_buf[hultrasonic.filter_idx] = distance;
                hultrasonic.filter_sum += distance;
                hultrasonic.filter_idx = (hultrasonic.filter_idx + 1) % FILTER_SIZE;
                hultrasonic.avg_distance = hultrasonic.filter_sum / FILTER_SIZE;
            }

            hultrasonic.is_first_captured = 0;
            // 다시 상승 엣지 캡처로 변경
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

/**
 * @brief 머리 위치 변화량 계산 (현재 평균 거리 - 기준 거리)
 * @return int8_t: 변화량 (-30 ~ +30 cm)
 */
int8_t Ultrasonic_Get_Head_Delta(void) {
	// Delta = 현재 평균 거리 - 동적 기준 거리
	int16_t delta = (int16_t)hultrasonic.avg_distance - (int16_t)dynamic_baseline;

    // 범위 제한 (-30 ~ +30 cm)
    if (delta > 30) delta = 30;
    else if (delta < -30) delta = -30;

    return (int8_t)delta;
}

uint16_t Ultrasonic_Get_Avg_Distance(void) {
    return hultrasonic.avg_distance;
}

uint16_t Ultrasonic_Get_Distance(void) {
    return distance;
}

uint16_t Ultrasonic_Get_Baseline(void) {
    return dynamic_baseline;
}
