// Energy-pumping swing-up. Algorithm is the same as the standalone
// `firmware/swingup` file from the swing-up branch. Motor drive goes
// through update_motor_directly() (analogWrite-based PWM) instead of
// raw digitalWrite on ENA — the original could use digitalWrite because
// its coast_motor() also used digitalWrite, so ENA_PIN stayed in SIO
// mode. Main's coast_motor() uses analogWrite(0), which leaves the pin
// in PWM hardware-function, and mixing digitalWrite calls on top of
// that produces asymmetric drive. Active only when currentState ==
// SWINGUP. Hand-off to LQR happens in main.cpp when |phi| < 0.2.

#include <Arduino.h>
#include "hardware.h"
#include "states.h"

static void motor_forward() {
    ENA = 255;
    update_motor_directly();
}

static void motor_backward() {
    ENA = -255;
    update_motor_directly();
}

void swingup_enter() {
    // No-op: statics inside swingup_tick persist exactly like the
    // original program-statics.
}

void swingup_tick() {
    if (currentState != SWINGUP) return;

    noInterrupts();
    long p_ticks = pendulum_ticks;
    interrupts();

    float phi_raw = p_ticks * RADIANS_PER_TICK;
    float phi = phi_raw - PI;
    if (phi >  PI) phi -= 2.0f * PI;
    if (phi < -PI) phi += 2.0f * PI;

    static float         phi_prev = 0.0f;
    static float         phidot   = 0.0f;
    static unsigned long t_vel    = 0;
    if (millis() - t_vel >= 50) {
        float new_phidot = (phi - phi_prev) / 0.1f;
        phidot   = 0.7f * phidot + 0.3f * new_phidot;
        phidot   = constrain(phidot, -15.0f, 15.0f);
        phi_prev = phi;
        t_vel    = millis();
    }

    static bool          startup_done = false;
    static unsigned long rock_timer   = 0;
    static bool          rock_dir     = true;
    if (!startup_done) {
        if (millis() - rock_timer > 1000) {
            rock_dir   = !rock_dir;
            rock_timer = millis();
        }
        if (rock_dir) motor_forward();
        else          motor_backward();
        if (fabsf(phidot) > 0.5f) {
            startup_done = true;
            event = "swingup_started";
        }
        return;
    }

    if (fabsf(phidot) < 0.05f) {
        static bool          kick_dir   = true;
        static unsigned long kick_timer = 0;
        if (millis() - kick_timer > 1500) {
            kick_dir   = !kick_dir;
            kick_timer = millis();
        }
        if (kick_dir) motor_forward();
        else          motor_backward();
    } else if (phidot > 0) {
        motor_forward();
    } else {
        motor_backward();
    }
}
