#include "fuzzy_logic.h"
#include <math.h>

// 절대값 매크로
#ifndef ABS
#define ABS(x) ((x) > 0 ? (x) : -(x))
#endif

// 사다리꼴 멤버십 함수
static float Fuzzy_Trapezoid(float x, float a, float b)
{
    if (x <= a) return 0.0f;
    if (x >= b) return 1.0f;
    return (x - a) / (b - a);
}

// V3.0
uint8_t Compute_Integrated_Risk(uint8_t perclos, float steer_std, float hands_off_sec, float head_delta, float no_op_sec)
{
    // [Step 1] Fuzzification
    float risk_eye   = Fuzzy_Trapezoid((float)perclos, 40.0f, 60.0f);
    float risk_steer = Fuzzy_Trapezoid(steer_std, 20.0f, 40.0f);
    float risk_hands = Fuzzy_Trapezoid(hands_off_sec, 2.0f, 5.0f);
    float risk_head  = Fuzzy_Trapezoid(ABS(head_delta), 5.0f, 15.0f);
    float risk_noop  = Fuzzy_Trapezoid(no_op_sec, 10.0f, 20.0f) * 0.6f; // 최대 60점(Warning) 제한

    // [Step 2] Rule Evaluation (눈부심 방지)
    // 눈만 위험하고(>0.8), 나머지는 아주 멀쩡하면(<0.2) -> 눈부심(Glare)으로 간주
    if (risk_eye > 0.8f)
    {
        if (risk_steer < 0.2f && risk_hands < 0.2f && risk_head < 0.2f)
        {
             risk_eye = 0.3f; // 강제 Normal 처리
        }
    }

    // [Step 3] Defuzzification (MAX Rule)
    // 그룹 1: 만성 졸음 (눈, 핸들흔들림, 무조작)
    float risk_slow = risk_eye;
    if (risk_steer > risk_slow) risk_slow = risk_steer;
    if (risk_noop  > risk_slow) risk_slow = risk_noop;

    // 그룹 2: 급박한 위험 (손뗌, 고개숙임)
    float risk_fast = risk_hands;
    if (risk_head > risk_fast) risk_fast = risk_head;

    // 최종: 둘 중 더 위험한 것 선택
    float final_risk = (risk_slow > risk_fast) ? risk_slow : risk_fast;

    return (uint8_t)(final_risk * 100.0f);
}
