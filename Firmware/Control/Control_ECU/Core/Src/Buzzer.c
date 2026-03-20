#include "Buzzer.h"

static uint32_t lastTick = 0; // 시간 측정을 위한 변수

/**
  * @brief 부저 초기화
  */
void Buzzer_Init(void) {
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
}

/**
  * @brief 시스템 상태에 따른 부저 로직 (주의 2초 주기 / 위험 0.2초 주기)
  * [cite_start]@param systemStatus: 0(정상), 1(주의), 2(위험) [cite: 69]
  */
void Buzzer_Update(uint8_t systemStatus)
{
    uint32_t currentTick = HAL_GetTick();

    if (systemStatus == 0) 		 // 정상 (Normal) [cite: 69]
    {
        Buzzer_Off();
    }
    else if (systemStatus == 1)   // 주의 (Warning): 2초 주기 [cite: 69]
    {
        // 0.5초 ON, 1.5초 OFF (총 2초)
        if ((currentTick - lastTick) < 500)
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_SET);
        }
        else if ((currentTick - lastTick) < 2000)
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
        }
        else
        {
            lastTick = currentTick; // 2초 완료 후 리셋
        }
    }
    else if (systemStatus == 2) // 위험 (Danger): 0.2초 주기 [cite: 69]
    {
        // 0.1초 ON, 0.1초 OFF (5Hz 고속 경고음)
        if ((currentTick - lastTick) < 100)
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_SET);
        }
        else if ((currentTick - lastTick) < 200)
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
        }
        else
        {
            lastTick = currentTick; // 0.2초 완료 후 리셋
        }
    }
}

/**
  * @brief 부저 끄기
  */
void Buzzer_Off(void) {
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
}
