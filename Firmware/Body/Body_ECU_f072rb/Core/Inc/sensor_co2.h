/*
 * sensor_co2.h
 *
 *  Created on: 2026. 1. 14.
 *      Author: DDU
 */

#ifndef INC_SENSOR_CO2_H_
#define INC_SENSOR_CO2_H_

#include "main.h"

#define CO2_FILTER_SIZE 10

typedef struct {
    uint16_t filter_buf[CO2_FILTER_SIZE];
    uint8_t  filter_idx;
    uint32_t filter_sum;
    uint16_t avg_value;
} CO2_t;

void CO2_Update(void);
uint16_t CO2_Get_Value(void);

#endif /* INC_SENSOR_CO2_H_ */
