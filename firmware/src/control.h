#pragma once
#include "hardware.h"

extern float K[4];
extern float state[4];

// LQR setpoint references — subtracted from state[0]/state[1] inside
// compute_control so the cart can be commanded to a non-zero target while
// still balancing. Both default to 0 (balance in place).
extern float lqr_x_ref;
extern float lqr_xdot_ref;

float compute_control();
