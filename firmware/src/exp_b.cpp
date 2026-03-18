#include "hardware.h"
#include "control.h"

unsigned long t0;
unsigned long t1;
unsigned long last_print;
float force_out;
float x = 0;
float phi = 0;
float xdot;
float phidot;
float xddot;
float magXddot;
float phiddot;
float prev_x = 0.0;
float prev_phi = 0.0;
float prev_xdot = 0.0;
float prev_phidot = 0.0;

enum SystemState { IDLE, RUNNING, ACCELERATING, TESTING };
SystemState currentState = IDLE;

float acc_threshold = 0.01;
int consecutive_count_below_threshold = 0;

void setup(){
    Serial.begin(115200);
    hardware_setup();
    ENA = 100;
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
            Serial.println("--- EXPERIMENT START ---");
        }
        if (c == 's') currentState = IDLE;
    }

    // 1ms control loop
    if (micros()-t1 >= 1000){
        x = read_position();
        phi = read_angle();
        xdot = (x-prev_x)/0.001;
        phidot = (phi-prev_phi)/0.001;
        xddot = (xdot-prev_xdot)/0.001;
        phiddot = (phidot-prev_phidot)/0.001;
        prev_x = x;
        prev_phi = phi;
        prev_xdot = xdot;
        prev_phidot = phidot;
        state[0] = x;
        state[1] = xdot;
        state[2] = phi;
        state[3] = phidot;
        t1 = micros();

        if (currentState == RUNNING || currentState == ACCELERATING){
            magXddot = (xddot >= 0) ? xddot : -xddot;
            if (magXddot <= acc_threshold){
                consecutive_count_below_threshold++;
            } else {
                consecutive_count_below_threshold = 0;
            }
            if (consecutive_count_below_threshold >= 10){
                Serial.print("--- CONSTANT SPEED AT ");
                Serial.print(xdot);
                Serial.println("m/s ---");
                Serial.print("--- acceleration = ");
                Serial.print(xddot);
                Serial.println("m/s^2 ---");
                Serial.println("--- TEST START ---");
                currentState = TESTING;
            }
        }
    }

    // Transition: RUNNING → ACCELERATING after 1 second
    if (currentState == RUNNING && micros()-t0 >= 1000000){
        currentState = ACCELERATING;
    }

    // Crash detection
    if ((currentState == RUNNING || currentState == ACCELERATING || currentState == TESTING)
        && (x > 1.2 || x < -0.1)) {
        Serial.println("--- DETECTED CRASH ---");
        currentState = IDLE;
    }

    // State machine
    if (currentState == RUNNING || currentState == ACCELERATING) {
        update_motor_directly();
    } else if (currentState == TESTING) {
        coast_motor();
    } else if (currentState == IDLE) {
        coast_motor();
    }

    // Serial print logs
    if (micros() - last_print >= 10000) {
        Serial.print(micros() - t0);
        Serial.print(",");
        Serial.print(currentState);
        Serial.print(",");
        Serial.print(ENA);
        Serial.print(",");
        Serial.print(x, 4);
        Serial.print(",");
        Serial.print(xdot, 4);
        Serial.print(",");
        Serial.print(phi, 4);
        Serial.print(",");
        Serial.println(phidot, 4);
        last_print = micros();
    }
}
// pio device monitor -b 115200 | tee "motor_test_$(date +%Y%m%d_%H%M%S).csv"
// pio device list
