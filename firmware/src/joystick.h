#pragma once
#include <Arduino.h>

// Button IDs — used with joystick_button_held() / joystick_consume_press().
// Bit positions inside the HID report are best-guess for 8BitDo Pro 2 mode D
// and are verified at runtime via the [JOY-BTN] debug print in joystick.cpp.
constexpr uint8_t JOY_BTN_A      = 0;
constexpr uint8_t JOY_BTN_B      = 1;
constexpr uint8_t JOY_BTN_X      = 2;
constexpr uint8_t JOY_BTN_Y      = 3;
constexpr uint8_t JOY_BTN_L1     = 4;
constexpr uint8_t JOY_BTN_R1     = 5;
constexpr uint8_t JOY_BTN_SELECT = 6;
constexpr uint8_t JOY_BTN_START  = 7;
constexpr uint8_t JOY_BTN_HOME   = 8;
constexpr uint8_t JOY_BTN_COUNT  = 9;

void joystick_setup();
void joystick_tick(float xdot);
bool joystick_connected();

// Held: true while the button is currently down.
// Consume_press: edge-triggered — returns true exactly once on the rising
// edge of a press, then resets. Use for state-changing actions.
bool joystick_button_held(uint8_t btn);
bool joystick_consume_press(uint8_t btn);

// Left-stick X, normalised to [-1.0, +1.0] with deadzone applied. Sign is
// flipped to match the cart-direction convention (positive = cart right).
float joystick_lx_normalised();

// Drive the motor straight from LX, bypassing the LQR. Returns false if
// BT is disconnected or the latest report is stale (in which case the
// motor is coasted) so the caller can fall back to its default behaviour.
bool joystick_drive_motor_direct();

// Rumble — stubs in this commit, output reports wired up in commit 9.
// pulse:      strong-motor intensity 0-255, duration in ms; non-blocking.
// continuous: sets a continuous strong-motor level; call again to update,
//             or with 0 to stop. Pulses override continuous while active.
void joystick_rumble_pulse(uint8_t strong, uint16_t duration_ms);
void joystick_rumble_continuous(uint8_t strong);
