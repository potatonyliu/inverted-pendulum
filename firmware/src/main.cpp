#include "hardware.h"
#include "control.h"
#include "states.h"

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
float beta = 1;

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

bool csv_mode = true;

void setup(){
    Serial.begin(115200);
    hardware_setup();
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
        force_out = compute_control();
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'w' && currentState == IDLE) { currentState = RUNNING; event = "start"; t0 = micros();}
            if (c == 's' && currentState == RUNNING) { currentState = IDLE; event = "manual_stop"; }
        }
        t1 = micros();
    }

    // State machine
    if (currentState == RUNNING) {
        if (phi > PI/2.0 or phi < -PI/2.0) {
            currentState = IDLE;
            event = "crash (angle)";
        }
        update_motor(force_out, state[1]);
    } else if (currentState == IDLE) {
        coast_motor();
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
