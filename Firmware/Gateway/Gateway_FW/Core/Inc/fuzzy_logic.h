#ifndef FUZZY_LOGIC_H
#define FUZZY_LOGIC_H

#include <stdint.h>

uint8_t Compute_Integrated_Risk(uint8_t perclos, float steer_std, float hands_off_sec, float head_delta, float no_op_sec);

#endif
