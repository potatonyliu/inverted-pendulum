#include "hardware.h"
#include "control.h"
#include "states.h"

unsigned long t0;
unsigned long t1;
unsigned long t3;
unsigned long last_print;
float force_out;
float x = 0;
float phi = 0;
float xdot;
float phidot;
float xddot;
float magXddot;
float phiddot;
float prev_xdot = 0.0;
float prev_phidot = 0.0;

float acc_threshold = 0.01;
int consecutive_count_below_threshold = 0;
bool csv_mode = true;

void setup(){
    Serial.begin(115200);
    hardware_setup();
    ENA = 255;
    t3 = micros();
    t1 = micros();
    t0 = micros();
    last_print = micros();
}

void loop() {
    // Serial input
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'w' && currentState == IDLE) {
            currentState = RUNNING;
            t0 = micros();
            event = "experiment_start";
        }
        if (c == 's') {
            currentState = IDLE;
            event = "manual_stop";
        }
    }
    
    if (micros()-t3 >= 100000){
        xddot = (xdot-prev_xdot)/0.1;
        phiddot = (phidot-prev_phidot)/0.1;
        prev_xdot = xdot;
        prev_phidot = phidot;
        t3 = micros();
    }

    // 1ms control loop
    if (micros()-t1 >= 1000){
        x = read_position();
        phi = read_angle();
        xdot = read_cart_velocity();
        phidot = read_pendulum_velocity();
        state[0] = x;
        state[1] = xdot;
        state[2] = phi;
        state[3] = phidot;
        t1 = micros();
    }

        if (currentState == RUNNING || currentState == ACCELERATING){
            ENA = 255;
    }

    // Transition: RUNNING → ACCELERATING after 1 second

    // State machine
    if (currentState == RUNNING || currentState == ACCELERATING) {
        update_motor_directly();
    } else if (currentState == TESTING) {
        coast_motor();
    } else if (currentState == IDLE) {
        coast_motor();
    }

    if (micros() - last_print >= (csv_mode ? 10000 : 100000)) {
        if (csv_mode) {
            // time_us, state, ENA, x, xdot, xddot, phi, phidot, phiddot, event
            Serial.print(micros() - t0);   Serial.print(",");
            Serial.print(currentState);     Serial.print(",");
            Serial.print(ENA);              Serial.print(",");
            Serial.print(x, 4);             Serial.print(",");
            Serial.print(xdot, 4);          Serial.print(",");
            Serial.print(xddot, 4);         Serial.print(",");
            Serial.print(phi, 4);           Serial.print(",");
            Serial.print(phidot, 4);        Serial.print(",");
            Serial.print(phiddot, 4);       Serial.print(",");
            Serial.println(event);
        } else {
            Serial.print(micros()/1000);    Serial.println("ms");
            Serial.print("State: ");        Serial.println(currentState);
            Serial.print("ENA: ");          Serial.println(ENA);
            Serial.print("x: ");            Serial.println(x);
            Serial.print("xdot: ");         Serial.println(xdot);
            Serial.print("xddot: ");        Serial.println(xddot);
            Serial.print("phi: ");          Serial.println(phi);
            Serial.print("phidot: ");       Serial.println(phidot);
            Serial.print("phiddot: ");      Serial.println(phiddot);
            if (event[0]) { Serial.print("Event: "); Serial.println(event); }
            Serial.println("=======================================");
        }
        event = "";
        last_print = micros();
    }
}
// pio device monitor -b 115200 | tee "../logs/motor_test_$(date +%Y%m%d_%H%M%S).csv"
// pio device list
