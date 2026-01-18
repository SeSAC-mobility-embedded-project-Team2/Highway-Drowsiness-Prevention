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
    float  steering_angle;   // 현재 조향각
    uint8_t  alive_cnt;
    uint8_t  err_flag;
} ChassisData_t;

// Body (센서 노드) 데이터
typedef struct {
    float head_delta_cm;   // 머리 위치 변화량
    float hands_off_sec;   // 손 뗀 시간
    float no_op_sec;       // 무조작 시간
    uint8_t  alive_cnt : 4;// 하위 4비트: 롤링 카운터
    uint8_t  err_flag  : 4;// 상위 4비트: 0=OK, 1=SonarFail, 2=TouchFail
} BodyData_t;

typedef struct __attribute__((packed)) {
    uint16_t header;        // 헤더: 0xFCFD (식별자)

    // 1. Vision Data (5 bytes)
    uint8_t perclos;
    uint8_t is_eye_closed;
    uint8_t is_face_detected;
    uint8_t v_alive_cnt;
    uint8_t v_err_flag;

    // 2. Chassis Data (10 bytes)
    float   steering_std_dev;
    float   steering_angle;
    uint8_t c_alive_cnt;
    uint8_t c_err_flag;

    // 3. Body Data (14 bytes)
    float   head_delta_cm;
    float   hands_off_sec;
    float   no_op_sec;
    uint8_t b_alive_cnt;
    uint8_t b_err_flag;

    // 4. Integrated Result (1 byte)
    uint8_t risk_level;     // 퍼지 로직 결과값

} DashboardPacket_t;

//=====main.c 에서 할당 =====
extern SystemState_t current_state;
extern float prev_steering_angle;
extern uint32_t no_op_timer;
extern CAN_RxHeaderTypeDef RxHeader;

//=====comm_manager.c 에서 할당 =====
extern uint8_t uart_rx_buffer[8];
extern VisionData_t vision_data;
extern ChassisData_t chassis_data;
extern BodyData_t body_data;

#endif
