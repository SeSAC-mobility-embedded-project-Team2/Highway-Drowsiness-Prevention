#include "Max7219.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

// 8x8 웅장한 폰트 (W, A, R, N, I, G, D, E, Space)
const uint8_t Font8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space (0)
	{0xFF, 0x40, 0x20, 0x10, 0x20, 0x40, 0xFF, 0x00}, // W (1)
    {0xFC, 0x12, 0x11, 0x11, 0x11, 0x12, 0xFC, 0x00}, // A (2)
    {0xFF, 0x11, 0x11, 0x11, 0x11, 0x22, 0x4C, 0x00}, // R (3)
    {0xFF, 0x04, 0x08, 0x10, 0x20, 0x40, 0xFF, 0x00}, // N (4)
	{0x00, 0x81, 0xFF, 0x81, 0x00, 0x00, 0x00, 0x00}, // I (5)
    {0x3E, 0x41, 0x49, 0x49, 0x49, 0x49, 0x7A, 0x00}, // G (6)
	{0x7F, 0x41, 0x41, 0x41, 0x41, 0x22, 0x1C, 0x00}, // D (7)
	{0xFF, 0x89, 0x89, 0x89, 0x89, 0x89, 0x89, 0x00}  // E (8)
};

uint8_t GetIdx(char c) {
    switch(c) {
        case 'W': return 1; case 'A': return 2; case 'R': return 3;
        case 'N': return 4; case 'I': return 5; case 'G': return 6;
        case 'D': return 7; case 'E': return 8; default: return 0;
    }
}

// [1] 전송 로직: DIN(M1)이 마지막에 데이터를 받도록 전송 순서 조정
void Max7219_SendRow(uint8_t row, uint8_t* data) {
    HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_RESET);
    // Daisy Chain: MCU -> M1(DIN) -> M2 -> M3 -> M4 순서일 때,
    // M4 데이터부터 먼저 보내야 M1 데이터가 가장 마지막에 멈춥니다.
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

// [2], [3] 스크롤링 및 매핑 로직 수정
void Max7219_ScrollText(const char* str, uint8_t repeats) {
    int strLen = strlen(str);
    int totalCols = strLen * 8;
    uint8_t buffer[512] = {0,};

    for (int i = 0; i < strLen; i++) {
        uint8_t idx = GetIdx(str[i]);
        for (int c = 0; c < 8; c++) buffer[i * 8 + c] = Font8x8[idx][c];
    }

    for (uint8_t r = 0; r < repeats; r++) { //
        // shift 가 0일 때: M1(DIN쪽) 우측 끝에서 글자가 나타남
        // shift 가 totalCols + 32일 때: M4(좌측 끝) 밖으로 완전히 사라짐
        for (int shift = 0; shift <= totalCols + 32; shift++) {
            uint8_t hardware_frame[8][4] = {0,};

            for (int m = 0; m < 4; m++) { // m=0(M1, DIN쪽), m=3(M4)
                for (int x = 0; x < 8; x++) {
                    // [1] DIN(M1)이 우측 끝(pixels 24-31)이 되도록 인덱스 계산
                    int bufIdx = shift + ( (3-m) * 8 ) + x - 31;

                    // [2] 유효 범위 체크: 버퍼 이외의 영역은 0x00 처리하여 반복 현상 방지
                    if (bufIdx >= 0 && bufIdx < totalCols) {
                        uint8_t colData = buffer[bufIdx];
                        // 90도 회전 매핑 (글자 세우기)
                        for (int y = 0; y < 8; y++) {
                            if (colData & (1 << y)) {
                                hardware_frame[y][m] |= (1 << (7 - x));
                            }
                        }
                    }
                }
            }
            // 프레임 전송
            for (uint8_t row = 1; row <= 8; row++) Max7219_SendRow(row, hardware_frame[row-1]);
            HAL_Delay(35); // 낮을수록 빠름
        }
    }
    Max7219_All_Off();
}
