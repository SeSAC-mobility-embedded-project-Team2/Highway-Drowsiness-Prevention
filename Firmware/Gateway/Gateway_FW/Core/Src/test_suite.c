/* test_suite.c */
#include "unity.h"
#include "fuzzy_logic.h"
#include "comm_manager.h"
#include "gateway_defines.h"
#include "main.h" // HAL 관련 정의 필요 시

// 전역 변수 모킹(Mocking)을 위한 선언 (comm_manager 테스트용)
extern VisionData_t  vision_data;
extern ChassisData_t chassis_data;
extern BodyData_t    body_data;
extern SystemState_t current_state;
extern uint8_t g_last_tx_packet[10];

extern void Update_System_State(void);

void setUp(void) {
    // 각 테스트 실행 전 초기화가 필요하면 작성
	// 테스트 간 데이터 간섭 방지 초기화
	    current_state = STATE_NORMAL;
	    vision_data = (VisionData_t){0};
	    chassis_data = (ChassisData_t){0};
	    body_data = (BodyData_t){0};
	    for(int i=0; i<10; i++) g_last_tx_packet[i] = 0;
}

void tearDown(void) {
	// 테스트 후 정리 작업: 전역 변수 리셋
    vision_data = (VisionData_t){0};
    chassis_data = (ChassisData_t){0};
    body_data = (BodyData_t){0};
    current_state = STATE_NORMAL;

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

// =========================================================
// [SWE.4 Unit Test] 통신 패킷 생성 검증 (수정됨)
// =========================================================
void test_Comm_Packet_Generation(void) {
    // 1. 가상의 UART 핸들러 (실제 전송은 실패하더라도 코드는 돌아감)
    UART_HandleTypeDef huart_mock = {0};

    // 2. 함수 호출 (DANGER 상태, MRM Active 전송 요청)
    DMS_Send_Control_Signal(&huart_mock, STATE_DANGER, 1, 0);

    // 3. 훔쳐온(sniffing) 데이터 검증
    TEST_ASSERT_EQUAL_HEX8(0x04, g_last_tx_packet[0]); // Header High
    TEST_ASSERT_EQUAL_HEX8(0x01, g_last_tx_packet[1]); // Header Low
    TEST_ASSERT_EQUAL_HEX8(STATE_DANGER, g_last_tx_packet[2]); // Payload: State
    TEST_ASSERT_BIT_HIGH(0, g_last_tx_packet[3]);      // Payload: MRM Flag Check
}

// =========================================================
// [SWE.5 Integration Test] 센서 입력 -> 판단 -> 출력 통합 검증
// =========================================================
void test_Integration_Full_Flow_Drowsy(void) {
    // 1. [Input Injection] 센서 데이터 강제 주입 (졸음 상황)
    vision_data.perclos = 85;       // 눈 감음
    vision_data.is_face_detected = 1;
    chassis_data.steering_std_dev = 45.0f; // 핸들 흔들림 심함

    // 2. [Process] 통합 제어 로직 실행 (main.c의 핵심 로직)
    // 상태 천이를 위해 반복 호출이 필요할 수 있으나, 로직상 즉시 반영된다고 가정
    Update_System_State();

    // 3. [Verify State] 내부 상태가 WARNING 혹은 DANGER로 바뀌었는지 확인
    // (초기 NORMAL -> 75점 이상이면 WARNING)
    TEST_ASSERT_EQUAL_INT(STATE_WARNING, current_state);

    // 한 번 더 상태 악화 시뮬레이션 (이미 Warning 상태라고 가정하고 더 높은 점수)
    current_state = STATE_WARNING;
    Update_System_State(); // 95점 이상 나오게 유도됨

    // 4. [Verify Output] 최종적으로 UART로 나가는 패킷까지 확인
    // Update_System_State 내부에서 DMS_Send_Control_Signal을 호출함.
    TEST_ASSERT_EQUAL_INT(STATE_DANGER, current_state); // 상태가 DANGER로 갔는지
    TEST_ASSERT_EQUAL_HEX8(STATE_DANGER, g_last_tx_packet[2]); // 실제 통신 버퍼에도 DANGER가 실렸는지
}



// =========================================================
// [Test Runner]
// =========================================================
void Run_ASPICE_Unit_Tests(void) {
    UNITY_BEGIN();

    // SWE.4 Units
    // Fuzzy Logic Tests
    RUN_TEST(test_Fuzzy_Scenario_Drowsy_Danger);
    RUN_TEST(test_Fuzzy_Scenario_Normal_Driving);
    RUN_TEST(test_Fuzzy_Veto_Logic);

    // Comm Test
    RUN_TEST(test_Comm_Packet_Generation);

    // SWE.5 Integration
    RUN_TEST(test_Integration_Full_Flow_Drowsy);

    UNITY_END();
}
