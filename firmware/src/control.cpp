#include "control.h"

float K[4] = {300, -6.6257, 1000, 21.2535};
float state[4] = {0.0, 0.0, 0.0, 0.0};

float compute_control(){
    float u = -K[0] * state[0] - K[1] * state[1] - K[2] * state[2] - K[3] * state[3];
    // u = constrain(u, -MAX_FORCE, MAX_FORCE);
    return u;
}
