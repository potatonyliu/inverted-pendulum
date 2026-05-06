#pragma once
#include <Arduino.h>

enum SystemState { IDLE, RUNNING, ACCELERATING, TESTING, JOYSTICK, AUTO_SWINGUP };
extern volatile SystemState currentState;
extern const char* event;
