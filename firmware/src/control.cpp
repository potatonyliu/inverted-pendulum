#include "control.h"

float K[4] = {20.0, 50.0, 152.4200, 30.0335};
float state[4] = {0.0, 0.0, 0.0, 0.0};
float lqr_x_ref    = 0.0f;
float lqr_xdot_ref = 0.0f;

float compute_control(){
    float u = -K[0] * (state[0] - lqr_x_ref)
              -K[1] * (state[1] - lqr_xdot_ref)
              -K[2] *  state[2]
              -K[3] *  state[3];
    // u = constrain(u, -MAX_FORCE, MAX_FORCE);
    return u;
}
