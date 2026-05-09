// Energy-pumping swing-up: rocks the cart back and forth to build pendulum
// energy, then bang-bangs in the direction of phidot until the pendulum
// reaches upright. Active only when currentState == SWINGUP. Hand-off to
// LQR happens in main.cpp when |phi| < 0.2.

#include <Arduino.h>
#include "hardware.h"
#include "states.h"

namespace {
bool          startup_done = false;
unsigned long rock_t0      = 0;
unsigned long kick_t0      = 0;
bool          rock_dir     = true;
bool          kick_dir     = true;
}

void swingup_enter() {
    startup_done = false;
    rock_t0      = millis();
    kick_t0      = millis();
    rock_dir     = true;
    kick_dir     = true;
}

void swingup_tick(float phi, float phidot) {
    if (currentState != SWINGUP) return;

    // Initial rocking: alternate direction every 1s until the pendulum has
    // any measurable swing speed.
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

    // Energy pumping: push in direction of motion. If pendulum is nearly
    // stationary, periodic kick to wake it back up.
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
