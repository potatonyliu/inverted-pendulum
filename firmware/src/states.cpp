#include "states.h"
volatile SystemState currentState = IDLE;
volatile SystemSubState subState = INITIALIZING;
volatile Mode currentMode = MODE_IDLE;
const char* event = "";
