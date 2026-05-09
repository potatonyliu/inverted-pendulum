#pragma once
#include <Arduino.h>

enum SystemState { IDLE, RUNNING, ACCELERATING, TESTING, JOYSTICK, SWINGUP };
extern volatile SystemState currentState;
extern const char* event;
