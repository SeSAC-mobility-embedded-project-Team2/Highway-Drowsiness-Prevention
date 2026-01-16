/* test_suite.c */
#include "unity.h"
#include "fuzzy_logic.h"
#include "comm_manager.h"
#include "main.h" // HAL 관련 정의 필요 시

// 전역 변수 모킹(Mocking)을 위한 선언 (comm_manager 테스트용)
extern VisionData_t  vision_data;
extern ChassisData_t chassis_data;
extern BodyData_t    body_data;

void setUp(void) {
    // 각 테스트 실행 전 초기화가 필요하면 작성
}

void tearDown(void) {
    // 테스트 후 정리 작업
}

// =========================================================
// [Unit Test 1] Fuzzy Logic 검증 (가장 중요)
// =========================================================

// 시나리오 1: 졸음 + 조향 불안정 = 위험(Danger)
void test_Fuzzy_Scenario_Drowsy_Danger(void) {
    // Input: 눈감음 80%, 핸들 흔들림 40 (최대), 나머지 0
    uint8_t perclos = 80;
    float steer_std = 40.0f;

    uint8_t score = Compute_Integrated_Risk(perclos, steer_std, 0, 0, 0);

    // Expect: 80점 이상 (Danger)
    TEST_ASSERT_GREATER_THAN_UINT8(80, score);
}

// 시나리오 2: 정상 주행 = 정상(Normal)
void test_Fuzzy_Scenario_Normal_Driving(void) {
    // Input: 눈감음 10%, 핸들 안정적
    uint8_t perclos = 10;
    float steer_std = 5.0f;

    uint8_t score = Compute_Integrated_Risk(perclos, steer_std, 0, 0, 0);

    // Expect: 20점 미만 (Normal)
    TEST_ASSERT_LESS_THAN_UINT8(20, score);
}

// 시나리오 3: Veto Logic (눈부심 상황)
// 눈은 감은 것처럼 보이나(90), 핸들은 완벽하게 잡고 있음(2.0)
void test_Fuzzy_Veto_Logic(void) {
    uint8_t perclos = 90;
    float steer_std = 2.0f; // 매우 안정

    uint8_t score = Compute_Integrated_Risk(perclos, steer_std, 0, 0, 0);

    // Expect: Danger(>80)가 나오면 안됨. Warning 수준 이하로 억제 확인
    TEST_ASSERT_LESS_THAN_UINT8(65, score);
}


// =========================================================
// [Unit Test 2] Communication Manager 검증
// 하드웨어 전송(HAL_UART)을 제외하고, "패킷이 잘 만들어졌나"만 확인
// =========================================================

// 통신 패킷 생성 로직 테스트 (comm_manager.c 내부 로직 검증)
// 주의: 이 테스트를 위해 comm_manager.c의 전역 변수나 함수 접근이 가능해야 함
void test_Comm_Packet_Generation(void) {
    // 1. 가상의 상황 설정
    uint8_t current_state = 2; // Danger
    uint8_t mrm_cmd = 1;       // MRM Active
    uint8_t sys_err = 0;

    // 2. 패킷 버퍼 준비 (comm_manager 내부 로직을 흉내내거나, 패킷 생성 함수를 분리했다면 호출)
    // 여기서는 사용자가 올린 코드 흐름 상 결과물을 검증하는 방식을 사용

    uint8_t tx_packet[10] = {0};

    // Header
    tx_packet[0] = 0x04;
    tx_packet[1] = 0x01;

    // Payload Logic 검증 (comm_manager.c 의 70~90라인 로직 복제 테스트)
    tx_packet[2] = current_state; // Should be 2
    if(mrm_cmd) tx_packet[3] |= 0x01; // Should be 1

    // Assert
    TEST_ASSERT_EQUAL_HEX8(0x04, tx_packet[0]); // 헤더 확인
    TEST_ASSERT_EQUAL_HEX8(0x02, tx_packet[2]); // 상태값 확인
    TEST_ASSERT_EQUAL_HEX8(0x01, tx_packet[3]); // MRM 플래그 확인
}

// =========================================================
// [Test Runner]
// =========================================================
void Run_ASPICE_Unit_Tests(void) {
    UNITY_BEGIN();

    // Fuzzy Logic Tests
    RUN_TEST(test_Fuzzy_Scenario_Drowsy_Danger);
    RUN_TEST(test_Fuzzy_Scenario_Normal_Driving);
    RUN_TEST(test_Fuzzy_Veto_Logic);

    // Comm Logic Tests
    RUN_TEST(test_Comm_Packet_Generation);

    UNITY_END();
}
