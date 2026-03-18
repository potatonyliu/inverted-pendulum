#pragma once
#include "hardware.h"

extern float K[4];
extern float state[4];

float compute_control();
