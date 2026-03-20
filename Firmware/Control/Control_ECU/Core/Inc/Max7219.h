/*
 * Max7219.h
 *
 *  Created on: 2026. 1. 15.
 *      Author: jw007
 */

#ifndef __MAX7219_H__
#define __MAX7219_H__

#include "main.h"

#define NUM_MODULES     4
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_SCAN_LIMIT  0x0B
#define REG_SHUTDOWN    0x0C
#define REG_DISPLAY_TEST 0x0F

void Max7219_Init(void);
void Max7219_All_Off(void);
void Max7219_SendRow(uint8_t row, uint8_t* data);
void Max7219_ScrollText(const char* str, uint8_t repeats);
void Max7219_Update(uint8_t currentStatus); // 추가된 선언

#endif /* INC_MAX7219_H_ */
