#include "states.h"
volatile SystemState currentState = IDLE;
volatile Mode currentMode = MODE_IDLE;
const char* event = "";
