/*
 * sensor_ultrasonic.h
 *
 *  Created on: 2026. 1. 14.
 *      Author: DDU
 */

#ifndef INC_SENSOR_ULTRASONIC_H_
#define INC_SENSOR_ULTRASONIC_H_

#include "main.h"

#define FILTER_SIZE 10 // 10개의 데이터를 평균내어 노이즈 제거

typedef struct{
	uint32_t capture_start;
	uint32_t capture_end;
	uint32_t diff_time;
	uint8_t is_first_captured; // 0: 상승엣지 대기, 1: 하강엣지 대기

	// 이동평균 필터 관련
	uint16_t filter_buf[FILTER_SIZE];
	uint8_t filter_idx;
	uint32_t filter_sum;
	uint16_t avg_distance;
}Ultrasonic_t;

void Ultrasonic_Trigger(void);
void Ultrasonic_Capture_Callback(TIM_HandleTypeDef *htim);
void Ultrasonic_Set_Baseline(uint16_t dist);

uint16_t Ultrasonic_Get_Distance(void);
uint16_t Ultrasonic_Get_Avg_Distance(void);
uint16_t Ultrasonic_Get_Baseline(void);
int8_t Ultrasonic_Get_Head_Delta(void);

#endif /* INC_SENSOR_ULTRASONIC_H_ */
