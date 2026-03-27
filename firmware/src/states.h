#pragma once
#include <Arduino.h>

enum SystemState { IDLE, RUNNING, ACCELERATING, TESTING };
extern volatile SystemState currentState;
extern const char* event;
