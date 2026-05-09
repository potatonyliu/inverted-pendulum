#include "hardware.h"
#include "control.h"
#include "states.h"
#include "joystick.h"

extern void swingup_enter();
extern void swingup_tick(float x, float xdot, float phi, float phidot);

unsigned long t1;
unsigned long t0;
unsigned long last_print;
float force_out;
float x = 0;
float phi = 0;
float xdot;
float phidot;

// EMA
float prev_xdot = 0.0;
float prev_phidot = 0.0;
float alpha = 1;
float beta = 0.2;

// --- Alpha-Beta tracker state ---
float x_hat = 0.0f;
float xdot_hat = 0.0f;
float phi_hat = 0.0f;
float phidot_hat = 0.0f;

// --- Tracker gains ---
const float DT         = 1e-3f;
const float ALPHA_X    = 0.2f;   // position correction gain, cart
const float BETA_X     = 0.05f;  // velocity correction gain, cart
const float ALPHA_PHI  = 0.2f;   // position correction gain, pendulum
const float BETA_PHI   = 0.05f;  // velocity correction gain, pendulum

bool csv_mode = false;

void setup(){
    Serial.begin(115200);
    hardware_setup();
    joystick_setup();
    t0 = micros();
    t1 = micros();
    last_print = micros();
}

void loop() {

    if (micros()-t1 >= 1000){
        x = read_position();
        phi = read_angle();
        xdot = read_cart_velocity();
        xdot = alpha * xdot + (1-alpha) * prev_xdot;
         prev_xdot = xdot;
        phidot = read_pendulum_velocity();
        phidot = beta * phidot + (1-beta) * prev_phidot;
        prev_phidot = phidot;

        // TRACKING LOOP
        // x_hat    += xdot_hat * DT;      // predict
        // float e_x = x - x_hat;          // residual (meters)
        // x_hat    += ALPHA_X * e_x;
        // xdot_hat += (BETA_X / DT) * e_x;

        // phi_hat    += phidot_hat * DT;  // predict
        // float e_phi = phi - phi_hat;    // residual (radians)
        // phi_hat    += ALPHA_PHI * e_phi;
        // phidot_hat += (BETA_PHI / DT) * e_phi;

        state[0] = x;
        state[1] = xdot;
        state[2] = phi;
        state[3] = phidot;

        // L1 nudge in B-mode: stick → cart velocity setpoint, integrated
        // into x_ref so the cart holds wherever you parked it on release.
        // Only meaningful while LQR is RUNNING.
        {
            static unsigned long last_nudge_us = 0;
            constexpr float MAX_NUDGE_VEL = 0.3f;  // m/s at full stick
            bool nudge_active = (currentMode == MODE_BALANCE_ASSIST
                                 && currentState == RUNNING
                                 && joystick_button_held(JOY_BTN_L1));
            if (nudge_active) {
                unsigned long now = micros();
                if (last_nudge_us != 0) {
                    float dt   = (now - last_nudge_us) * 1e-6f;
                    float stick = joystick_lx_normalised();   // -1..+1
                    lqr_xdot_ref = stick * MAX_NUDGE_VEL;
                    lqr_x_ref   += lqr_xdot_ref * dt;
                }
                last_nudge_us = now;
            } else {
                last_nudge_us = 0;
                lqr_xdot_ref  = 0.0f;
                // x_ref persists between L1 holds so the cart holds the
                // last commanded position. It's reset on entry to RUNNING.
            }
        }

        force_out = compute_control();
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'w' && currentState == IDLE) { currentState = RUNNING; event = "start"; t0 = micros();}
            if (c == 's' && currentState == RUNNING) { currentState = IDLE; event = "manual_stop"; }
            if (c == 'j' && currentState == IDLE) { currentState = JOYSTICK; event = "joystick_on"; }
            else if (c == 'j' && currentState == JOYSTICK) { currentState = IDLE; event = "joystick_off"; }
            if (c == 'u' && currentState == IDLE) { currentState = SWINGUP; event = "swingup_on"; swingup_enter(); }
            else if (c == 'u' && currentState == SWINGUP) { currentState = IDLE; event = "swingup_off"; }
        }

        // Controller-driven mode/state transitions. ABYX is edge-triggered
        // (one shot per press). L1+R1 simultaneous = e-stop, fires once on
        // the transition into IDLE so it doesn't churn while held.
        if (joystick_consume_press(JOY_BTN_A)) {
            currentMode = MODE_AUTO; currentState = SWINGUP;
            event = "mode_auto"; swingup_enter();
        }
        if (joystick_consume_press(JOY_BTN_B)) {
            currentMode = MODE_BALANCE_ASSIST; currentState = JOYSTICK;
            event = "mode_balance_assist";
        }
        if (joystick_consume_press(JOY_BTN_Y)) {
            currentMode = MODE_JOYSTICK; currentState = JOYSTICK;
            event = "mode_joystick";
        }
        if (joystick_consume_press(JOY_BTN_X)) {
            currentMode = MODE_IDLE; currentState = IDLE;
            event = "mode_idle"; coast_motor();
        }
        if (joystick_button_held(JOY_BTN_L1) && joystick_button_held(JOY_BTN_R1)
                && currentState != IDLE) {
            currentMode = MODE_IDLE; currentState = IDLE;
            event = "estop"; coast_motor();
        }

        t1 = micros();
    }

    // Auto-balance: hand off to LQR when pendulum is swung near upright.
    if ((currentState == JOYSTICK || currentState == SWINGUP) && fabsf(phi) < 0.2f) {
        currentState = RUNNING;
        event = "auto_balance";
        t0 = micros();
        // Fresh balance run starts targeting x=0, regardless of any prior
        // L1 nudges that may have pushed the reference around.
        lqr_x_ref    = 0.0f;
        lqr_xdot_ref = 0.0f;
    }

    // State machine
    if (currentState == RUNNING) {
        if (phi > PI/2.0 or phi < -PI/2.0) {
            // Mode-aware crash recovery: AUTO retries the swing-up cycle,
            // BALANCE_ASSIST drops back to manual joystick within the same
            // mode, everything else (incl. serial 'w' and MODE_JOYSTICK)
            // falls back to IDLE.
            coast_motor();
            if (currentMode == MODE_AUTO) {
                currentState = SWINGUP;
                event = "crash → swingup";
                swingup_enter();
            } else if (currentMode == MODE_BALANCE_ASSIST) {
                currentState = JOYSTICK;
                event = "crash → joystick";
            } else {
                currentState = IDLE;
                event = "crash (angle)";
            }
        } else if (currentMode == MODE_BALANCE_ASSIST && joystick_button_held(JOY_BTN_R1)) {
            // R1 hard override: skip LQR entirely, drive cart from stick.
            // If BT drops mid-override, joystick_drive_motor_direct returns
            // false and we fall back to LQR (cart will probably crash, but
            // that's better than a stuck motor command).
            if (!joystick_drive_motor_direct()) {
                update_motor(force_out, state[1]);
            }
        } else {
            update_motor(force_out, state[1]);
        }
    } else if (currentState == IDLE) {
        coast_motor();
    } else if (currentState == JOYSTICK) {
        joystick_tick(state[1]);
    } else if (currentState == SWINGUP) {
        swingup_tick(state[0], state[1], state[2], state[3]);
    }

    if (micros() - last_print >= (csv_mode ? 10000 : 100000)) {
        if (csv_mode) {
            // time_us, state, force, x, xdot, phi, phidot, event
            Serial.print(micros() - t0);   Serial.print(",");
            Serial.print(currentState);     Serial.print(",");
            Serial.print(force_out, 4);     Serial.print(",");
            Serial.print(x, 4);             Serial.print(",");
            Serial.print(xdot, 4);          Serial.print(",");
            Serial.print(phi, 4);           Serial.print(",");
            Serial.print(phidot, 4);        Serial.print(",");
            Serial.println(event);
        } else {
            Serial.print(micros()/1000);    Serial.println("ms");
            Serial.print("State: ");        Serial.println(currentState);
            Serial.print("Force: ");        Serial.println(force_out);
            Serial.print("x: ");            Serial.println(x);
            Serial.print("xdot: ");         Serial.println(xdot);
            Serial.print("phi: ");          Serial.println(phi);
            Serial.print("phidot: ");       Serial.println(phidot);
            Serial.print("Cart Ticks: ");       Serial.println(cart_ticks);
            Serial.print("PWM: ");       Serial.println(ENA);
            if (event[0]) { Serial.print("Event: "); Serial.println(event); }
            Serial.println("=======================================");
        }
        event = "";
        last_print = micros();
    }
}
// pio device monitor -b 115200 | tee "../logs/main_$(date +%Y%m%d_%H%M%S).csv"
