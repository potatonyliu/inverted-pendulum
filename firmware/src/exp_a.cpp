#include "hardware.h"
#include "control.h"
#include "states.h"

unsigned long t0;
unsigned long t1;
unsigned long t2;
unsigned long last_print;
float x = 0;
float xdot      = 0;   // current method: tick-timing
float xdot_10ms = 0;   // original method: 10ms finite difference
float prev_x    = 0.0;

bool csv_mode = true;

void setup(){
    Serial.begin(115200);
    hardware_setup();
    ENA = 255;
    t2 = micros();
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

    // 10ms: original finite-difference xdot
    if (micros()-t2 >= 10000){
        xdot_10ms = (x - prev_x) / 0.01;
        prev_x = x;
        t2 = micros();
    }

    // 1ms: read position + current tick-timing xdot
    if (micros()-t1 >= 1000){
        x    = read_position();
        xdot = read_cart_velocity();
        t1 = micros();
    }

    if (currentState == RUNNING || currentState == ACCELERATING){
        ENA = 255;
        if (currentState == RUNNING && t0 + 1300000 < micros()) {
            currentState = IDLE;
            event = "coast down";
        }
    }

    // State machine
    if (currentState == RUNNING || currentState == ACCELERATING) {
        update_motor_directly();
    } else if (currentState == TESTING) {
        coast_motor();
    } else if (currentState == IDLE) {
        coast_motor();
    }

    if (micros() - last_print >= 10000) {
        // time_us, state, ENA, xdot_tick, xdot_10ms, event
        Serial.print(micros() - t0);   Serial.print(",");
        Serial.print(currentState);     Serial.print(",");
        Serial.print(ENA);              Serial.print(",");
        Serial.print(x, 4);          Serial.print(",");
        Serial.print(xdot, 4);          Serial.print(",");
        Serial.print(xdot_10ms, 4);     Serial.print(",");
        Serial.println(event);
        event = "";
        last_print = micros();
    }
}
// pio device monitor -b 115200 | tee "../logs/exp_a_$(date +%Y%m%d_%H%M%S).csv"
// pio device list
