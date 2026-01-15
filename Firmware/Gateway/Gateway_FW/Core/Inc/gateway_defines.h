#ifndef GATEWAY_DEFINES_H
#define GATEWAY_DEFINES_H

#include <stdint.h>

// ==========================================
// 1. 시스템 상태 정의 (Enum)
// ==========================================
typedef enum {
    STATE_NORMAL = 0,
    STATE_WARNING,
    STATE_DANGER,
    STATE_FAULT
} SystemState_t;

// ==========================================
// 2. 센서 데이터 구조체 정의
// ==========================================
typedef struct {
    uint8_t perclos;
    uint8_t is_eye_closed;
    uint8_t is_face_detected;
    uint8_t alive_cnt;
    uint8_t err_flag;
} VisionData_t;

// Chassis (조향) 데이터
typedef struct {
    float    steering_std_dev; // 조향 표준편차
    int16_t  steering_angle;   // 현재 조향각
    uint8_t  alive_cnt;
    uint8_t  err_flag;
} ChassisData_t;

// Body (센서 노드) 데이터
typedef struct {
    float head_delta_cm;   // 머리 위치 변화량
    float hands_off_sec;   // 손 뗀 시간
    float no_op_sec;       // 무조작 시간
} BodyData_t;

// ==========================================
// 3.
// ==========================================
extern SystemState_t current_state;
extern int16_t prev_steering_angle;
extern uint32_t no_op_timer;

extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];

extern uint8_t uart_rx_buffer[10];
extern VisionData_t vision_rx_packet; // 구조체 이름 확인 필요
extern ChassisData_t chassis_info;
extern BodyData_t body_info;

#endif
