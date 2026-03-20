#include "Max7219.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

// 폰트 데이터 및 GetIdx 함수 (기존과 동일)
const uint8_t Font8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0xFF, 0x40, 0x20, 0x10, 0x20, 0x40, 0xFF, 0x00}, // W
    {0xFC, 0x12, 0x11, 0x11, 0x11, 0x12, 0xFC, 0x00}, // A
    {0xFF, 0x11, 0x11, 0x11, 0x11, 0x22, 0x4C, 0x00}, // R
    {0xFF, 0x04, 0x08, 0x10, 0x20, 0x40, 0xFF, 0x00}, // N
    {0x00, 0x81, 0xFF, 0x81, 0x00, 0x00, 0x00, 0x00}, // I
    {0x3E, 0x41, 0x49, 0x49, 0x49, 0x49, 0x7A, 0x00}, // G
    {0x7F, 0x41, 0x41, 0x41, 0x41, 0x22, 0x1C, 0x00}, // D
    {0xFF, 0x89, 0x89, 0x89, 0x89, 0x89, 0x89, 0x00}  // E
};

uint8_t GetIdx(char c) {
    switch(c) {
        case 'W': return 1; case 'A': return 2; case 'R': return 3;
        case 'N': return 4; case 'I': return 5; case 'G': return 6;
        case 'D': return 7; case 'E': return 8; default: return 0;
    }
}

/* 비차단 제어를 위한 정적 변수 */
static uint8_t  lastStatus = 0xFF;
static uint32_t lastScrollTick = 0;
static int      scrollShift = 0;
static uint8_t  repeatCount = 0;
static char     currentText[20];
static int      totalCols = 0;
static uint8_t  textBuffer[160]; // 최대 20글자 지원 (20*8)

// 한 프레임을 계산하여 전송하는 내부 함수
void Max7219_RenderFrame(int shift) {
    uint8_t hardware_frame[8][4] = {0,};
    for (int m = 0; m < 4; m++) {
        for (int x = 0; x < 8; x++) {
            int bufIdx = shift + ((3 - m) * 8) + x - 31;
            if (bufIdx >= 0 && bufIdx < totalCols) {
                uint8_t colData = textBuffer[bufIdx];
                for (int y = 0; y < 8; y++) {
                    if (colData & (1 << y)) hardware_frame[y][m] |= (1 << (7 - x));
                }
            }
        }
    }
    for (uint8_t row = 1; row <= 8; row++) Max7219_SendRow(row, hardware_frame[row - 1]);
}

void Max7219_Update(uint8_t currentStatus) {
    // 1. 상태가 변했을 때 텍스트 버퍼 생성 (최초 1회)
    if (currentStatus != lastStatus) {
        if (currentStatus == 1)      strcpy(currentText, "WARNING");
        else if (currentStatus == 2) strcpy(currentText, "DANGER");
        else {
            Max7219_All_Off();
            lastStatus = currentStatus;
            return;
        }

        int strLen = strlen(currentText);
        totalCols = strLen * 8;
        for (int i = 0; i < strLen; i++) {
            uint8_t idx = GetIdx(currentText[i]);
            for (int c = 0; c < 8; c++) textBuffer[i * 8 + c] = Font8x8[idx][c];
        }

        scrollShift = 0;
        repeatCount = 0;
        lastStatus = currentStatus;
    }

    if (currentStatus == 0) return;

    // 2. 3회 반복 제한 및 비차단 스크롤 (35ms 마다 이동)
    if (repeatCount < 3) {
        if (HAL_GetTick() - lastScrollTick > 35) {
            lastScrollTick = HAL_GetTick();

            Max7219_RenderFrame(scrollShift);
            scrollShift++;

            if (scrollShift > totalCols + 32) {
                scrollShift = 0;
                repeatCount++;
                if (repeatCount >= 3) Max7219_All_Off();
            }
        }
    }
}

// 초기화, 전송, All_Off 함수는 기존과 동일하게 유지...
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
