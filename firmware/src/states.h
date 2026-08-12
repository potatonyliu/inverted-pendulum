#pragma once
#include <Arduino.h>

enum SystemState { 
    IDLE, 
    RUNNING, 
    ACCELERATING, 
    TESTING, 
    JOYSTICK, 
    SWINGUP, 
    RECENTER,
    INTENTIONAL_FALL };

enum SystemSubState { 
    INITIALIZING,
    DOWN_AND_STILL, 
    LOW_SWING, 
    HIGH_SWING,
    ALMOST_THERE,
    BALANCING };

// Mode is the user's high-level intent (set by an A/B/Y/X press on the
// controller). State is what the system is currently doing (LQR running,
// joystick driving, swinging up, idle). Most state transitions inside a
// mode are automatic — e.g. JOYSTICK → RUNNING when the pendulum reaches
// upright in MODE_BALANCE_ASSIST.
enum Mode { MODE_IDLE, MODE_JOYSTICK, MODE_BALANCE_ASSIST, MODE_AUTO };

extern volatile SystemState currentState;
extern volatile SystemSubState subState;
extern volatile Mode currentMode;
extern const char* event;
