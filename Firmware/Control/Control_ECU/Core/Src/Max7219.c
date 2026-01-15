#include "Max7219.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

// 8x8 웅장한 폰트 (W, A, R, N, I, G, D, E, Space)
const uint8_t Font8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space (Index 0)
    {0xFF, 0x02, 0x04, 0x08, 0x04, 0x02, 0xFF, 0x00}, // W (Index 1)
    {0xFC, 0x12, 0x11, 0x11, 0x11, 0x12, 0xFC, 0x00}, // A (Index 2)
    {0xFF, 0x11, 0x11, 0x11, 0x11, 0x22, 0x4C, 0x00}, // R (Index 3)
    {0xFF, 0x04, 0x08, 0x10, 0x20, 0x40, 0xFF, 0x00}, // N (Index 4)
    {0x00, 0x41, 0xFF, 0x41, 0x00, 0x00, 0x00, 0x00}, // I (Index 5)
    {0x3E, 0x41, 0x49, 0x49, 0x49, 0x49, 0x7A, 0x00}, // G (Index 6)
    {0xFF, 0x41, 0x41, 0x41, 0x41, 0x22, 0x1C, 0x00}, // D (Index 7)
    {0xFF, 0x49, 0x49, 0x49, 0x49, 0x41, 0x41, 0x00}  // E (Index 8)
};

uint8_t GetIdx(char c) {
    switch(c) {
        case 'W': return 1; case 'A': return 2; case 'R': return 3;
        case 'N': return 4; case 'I': return 5; case 'G': return 6;
        case 'D': return 7; case 'E': return 8; default: return 0;
    }
}

void Max7219_SendRow(uint8_t row, uint8_t* data) {
    HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_RESET);
    for (int i = NUM_MODULES - 1; i >= 0; i--) {
        HAL_SPI_Transmit(&hspi1, &row, 1, 10);
        HAL_SPI_Transmit(&hspi1, &data[i], 1, 10);
    }
    HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_SET);
}

void Max7219_Init(void) {
    for(uint8_t reg = 0x09; reg <= 0x0F; reg++) {
        uint8_t val = (reg == REG_INTENSITY) ? 0x01 : (reg == REG_SCAN_LIMIT) ? 0x07 : (reg == REG_SHUTDOWN) ? 0x01 : 0x00;
        uint8_t arr[4] = {val, val, val, val};
        Max7219_SendRow(reg, arr);
    }
    Max7219_All_Off();
}

void Max7219_All_Off(void) {
    uint8_t off[4] = {0, 0, 0, 0};
    for (uint8_t i = 1; i <= 8; i++) Max7219_SendRow(i, off);
}

// 스크롤링 및 회전 핵심 로직
void Max7219_ScrollText(const char* str, uint8_t repeats) {
    int strLen = strlen(str);
    int totalCols = strLen * 8;
    uint8_t buffer[512] = {0,};

    for (int i = 0; i < strLen; i++) {
        uint8_t idx = GetIdx(str[i]);
        for (int c = 0; c < 8; c++) buffer[i * 8 + c] = Font8x8[idx][c];
    }

    for (uint8_t r = 0; r < repeats; r++) { //
        for (int shift = 0; shift < totalCols + 32; shift++) {
            uint8_t hardware_frame[8][4] = {0,};

            for (int m = 0; m < 4; m++) {
                for (int x = 0; x < 8; x++) {
                    int bufIdx = shift + (m * 8) + x;
                    uint8_t colData = (bufIdx < totalCols) ? buffer[bufIdx] : 0x00;

                    // 글자를 세워주는 90도 회전 매핑 알고리즘
                    for (int y = 0; y < 8; y++) {
                        if (colData & (1 << y)) hardware_frame[y][m] |= (1 << (7 - x));
                    }
                }
            }
            for (uint8_t row = 1; row <= 8; row++) Max7219_SendRow(row, hardware_frame[row-1]);
            HAL_Delay(500); // 스크롤 속도
        }
    }
}
