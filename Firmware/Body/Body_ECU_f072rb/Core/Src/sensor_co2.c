/*
 * sensor_co2.c
 *
 *  Created on: 2026. 1. 14.
 *      Author: DDU
 */

#include "sensor_co2.h"

// main.c에 정의된 ADC 핸들러를 가져옵니다.
extern ADC_HandleTypeDef hadc;

static CO2_t hco2 = {0};

void CO2_Update(void) {
    // 1. ADC 샘플링 시작 및 값 읽기
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
        uint16_t raw_adc = HAL_ADC_GetValue(&hadc);

        // 2. 이동평균 필터 적용
        hco2.filter_sum -= hco2.filter_buf[hco2.filter_idx];
        hco2.filter_buf[hco2.filter_idx] = raw_adc;
        hco2.filter_sum += raw_adc;
        hco2.filter_idx = (hco2.filter_idx + 1) % CO2_FILTER_SIZE;
        hco2.avg_value = hco2.filter_sum / CO2_FILTER_SIZE;
    }
    HAL_ADC_Stop(&hadc);
}

uint16_t CO2_Get_Value(void) {
    return hco2.avg_value;
}
