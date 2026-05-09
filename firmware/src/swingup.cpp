// Energy-pumping swing-up. Three phases:
//   1. Centering: drive cart to x≈0 so subsequent rocking has rail margin
//      on both sides. Critical for AUTO-mode loop, where the cart can be
//      anywhere after a crash.
//   2. Rocking: alternate ±255 every 1 s until pendulum has any swing.
//   3. Pumping: bang-bang in the direction of phidot. Periodic kick if the
//      pendulum stalls.
// Active only when currentState == SWINGUP. Hand-off to LQR happens in
// main.cpp when |phi| < 0.2.

#include <Arduino.h>
#include "hardware.h"
#include "states.h"

namespace {
constexpr float CENTER_THRESHOLD_M = 0.05f;  // 5 cm
constexpr int   CENTER_PWM         = 150;    // bang-bang centering drive

bool          centered     = false;
bool          startup_done = false;
unsigned long rock_t0      = 0;
unsigned long kick_t0      = 0;
bool          rock_dir     = true;
bool          kick_dir     = true;
}

void swingup_enter() {
    centered     = false;
    startup_done = false;
    rock_t0      = millis();
    kick_t0      = millis();
    rock_dir     = true;
    kick_dir     = true;
}

void swingup_tick(float x, float /*xdot*/, float phi, float phidot) {
    (void)phi;
    if (currentState != SWINGUP) return;

    // Phase 1: centering. Sticky once done — we don't re-engage if the cart
    // drifts during pumping, because that would fight the energy buildup.
    if (!centered) {
        if (fabsf(x) < CENTER_THRESHOLD_M) {
            centered = true;
            event = "swingup_centered";
        } else {
            ENA = (x > 0) ? -CENTER_PWM : CENTER_PWM;
            update_motor_directly();
            return;
        }
    }

    // Phase 2: initial rocking, until pendulum has any measurable swing.
    if (!startup_done) {
        if (millis() - rock_t0 > 1000) {
            rock_dir = !rock_dir;
            rock_t0 = millis();
        }
        ENA = rock_dir ? 255 : -255;
        update_motor_directly();
        if (fabsf(phidot) > 0.5f) {
            startup_done = true;
            event = "swingup_started";
        }
        return;
    }

    // Phase 3: energy pumping — push in direction of motion. Periodic kick
    // to revive a stalled pendulum.
    if (fabsf(phidot) < 0.05f) {
        if (millis() - kick_t0 > 1500) {
            kick_dir = !kick_dir;
            kick_t0 = millis();
        }
        ENA = kick_dir ? 255 : -255;
    } else {
        ENA = phidot > 0 ? 255 : -255;
    }
    update_motor_directly();
}
