/*
 * can_message.h
 * 졸음 방지 시스템 통신 프로토콜 정의 (ICD V1.1 기준)
 * 작성일: 2026.01.13
 */

#ifndef INC_CAN_MESSAGE_H_
#define INC_CAN_MESSAGE_H_

#include <stdint.h>
#include <stdbool.h>

// ==========================================
// 1. CAN ID 정의 (Message ID)
// ==========================================
#define CAN_ID_VISION   0x101  // Vision Unit (UART -> CAN Gateway 내부 처리용)
#define CAN_ID_CHASSIS  0x201  // Chassis ECU
#define CAN_ID_BODY     0x301  // Body ECU
#define CAN_ID_SYSTEM   0x401  // Gateway -> Control ECU

// ==========================================
// 2. 데이터 구조체 정의 (Payload Structure)
// __attribute__((packed))는 바이트 정렬을 빈틈없이 채우기 위함입니다.
// ==========================================

/* * [0x201] Chassis ECU Status
 * 주기: 100ms
 */
typedef struct __attribute__((packed)) {
    uint16_t steering_std_dev;  // Byte 0-1: 조향 표준편차 (Factor: 0.01)
    int16_t  steering_angle;    // Byte 2-3: 현재 조향각 (Factor: 0.1)
    uint8_t  reserved[3];       // Byte 4-6: 예비 공간 (0x00으로 채움)

    // Byte 7: Safety Fields
    uint8_t  alive_cnt : 4;     // 하위 4비트: 롤링 카운터 (0~15)
    uint8_t  err_flag  : 4;     // 상위 4비트: 0=OK, 1=EncoderFail
} Chassis_Data_t;

/* * [0x301] Body ECU Status
 * 주기: 100ms
 */
typedef struct __attribute__((packed)) {
    uint8_t  distance_head;     // Byte 0: 머리 거리 (cm)
    uint8_t  touch_handle : 1;  // Byte 1[0]: 핸들 파지 여부 (1=On)
    uint8_t  reserved_bit : 7;  // Byte 1[1-7]: 예비 비트
    uint8_t  reserved[5];       // Byte 2-6: 예비 공간

    // Byte 7: Safety Fields
    uint8_t  alive_cnt : 4;     // 하위 4비트: 롤링 카운터
    uint8_t  err_flag  : 4;     // 상위 4비트: 0=OK, 1=SonarFail
} Body_Data_t;

/* * [0x401] System Command (Gateway -> Control)
 * 주기: 100ms
 */
typedef struct __attribute__((packed)) {
    uint8_t  alert_level;       // Byte 0: 0=Norm, 1=Warn, 2=Dang, 3=Fault
    uint8_t  mrm_trigger  : 1;  // Byte 1[0]: 최소위험기동 트리거 (1=Active)
    uint8_t  reserved_bit : 7;  // Byte 1[1-7]: 예비 비트
    uint8_t  reserved[5];       // Byte 2-6: 예비 공간

    // Byte 7: Safety Fields
    uint8_t  alive_cnt : 4;     // 하위 4비트: 롤링 카운터
    uint8_t  err_flag  : 4;     // 상위 4비트: 0=OK, 1=LogicFail
} System_Cmd_t;

// ==========================================
// 3. Vision Unit UART 패킷 구조 (참고용)
// Vision은 CAN이 아니라 UART를 쓰지만, 데이터 구조는 맞춰줍니다.
// ==========================================
typedef struct __attribute__((packed)) {
    uint8_t  header;            // 0xFF (Start Byte)
    uint8_t  perclos;           // Byte 0: PERCLOS (%)
    uint8_t  eye_state   : 1;   // Byte 1[0]: 1=Closed
    uint8_t  face_flag   : 1;   // Byte 1[1]: 1=Detected
    uint8_t  reserved_bit: 6;   // Byte 1[2-7]
    uint8_t  reserved[5];       // Byte 2-6
    uint8_t  alive_cnt   : 4;   // Byte 7[0-3]
    uint8_t  err_flag    : 4;   // Byte 7[4-7]
    uint8_t  checksum;          // XOR Checksum
} Vision_UART_Packet_t;

#endif /* INC_CAN_MESSAGE_H_ */
