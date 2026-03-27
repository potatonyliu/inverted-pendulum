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
float prev_x = 0.0;
float prev_phi = 0.0;

bool csv_mode = false;

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
        xdot = (x-prev_x)/0.001;
        phidot = (phi-prev_phi)/0.001;
        prev_x = x;
        prev_phi = phi;
        state[0] = x;
        state[1] = xdot;
        state[2] = phi;
        state[3] = phidot;
        force_out = compute_control();
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'w' && currentState == IDLE) { currentState = RUNNING; event = "start"; }
            if (c == 's' && currentState == RUNNING) { currentState = IDLE; event = "manual_stop"; }
        }
        t1 = micros();
    }

    // Crash detection
    if (currentState == RUNNING && (x > 0.6 || x < -0.6)) {
        currentState = IDLE;
        event = "crash";
    }

    // State machine
    if (currentState == RUNNING) {
        update_motor(force_out);
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
            if (event[0]) { Serial.print("Event: "); Serial.println(event); }
            Serial.println("=======================================");
        }
        event = "";
        last_print = micros();
    }
}
// pio device monitor -b 115200 | tee "../logs/main_$(date +%Y%m%d_%H%M%S).csv"
