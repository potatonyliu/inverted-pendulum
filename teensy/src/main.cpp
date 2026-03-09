#include "core_pins.h"
#include "usb_serial.h"
#include <Arduino.h>

#define XA_PIN 1
#define XB_PIN 2
#define PA_PIN 3
#define PB_PIN 4
#define IN1_PIN 5
#define IN2_PIN 6
#define ENA_PIN 7

const float MAX_FORCE = 10;
const float PULLEY_DIAMETER = 0.0225;
const float METERS_PER_TICK = PI * PULLEY_DIAMETER / 403.2;
const float RADIANS_PER_TICK = 2.0 * PI / 2400.0;

volatile long pendulum_ticks = 0;
volatile long cart_ticks = 0;

unsigned long t0;
unsigned long last_print;
float K[4] = {-1.0000, -3.1708, 80.2158, 24.9766};
float state[4] = {0.0, 0.0, 0.0, 0.0};
float F;
float x;
float phi;
float xdot;
float phidot;
float prev_x = 0.0;
float prev_phi = 0.0;

// meters
float read_position(){
    return cart_ticks * METERS_PER_TICK;
}

// radian
float read_angle(){
    return pendulum_ticks * RADIANS_PER_TICK;
}

// Newton
void update_motor(float force){
    if (force >= 0){
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);

    }
    else{
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
    }
    
    float magForce = (force >= 0) ? force : -force;
    int pwm = (int)(magForce / MAX_FORCE * 255);
    analogWrite(ENA_PIN, pwm);
}

void pendulum_A_handler(){
    int A = digitalReadFast(PA_PIN);
    int B = digitalReadFast(PB_PIN);
    if (A == B){
        pendulum_ticks--;
    }
    else{
        pendulum_ticks++;
    }
}

void pendulum_B_handler(){
    int A = digitalReadFast(PA_PIN);
    int B = digitalReadFast(PB_PIN);
    if (A != B){
        pendulum_ticks--;
    }
    else{
        pendulum_ticks++;
    }
}

void cart_A_handler(){
    int A = digitalReadFast(XA_PIN);
    int B = digitalReadFast(XB_PIN);
    if (A == B){
        cart_ticks--;
    }
    else{
        cart_ticks++;
    }
}

void cart_B_handler(){
    int A = digitalReadFast(XA_PIN);
    int B = digitalReadFast(XB_PIN);
    if (A != B){
        cart_ticks--;
    }
    else{
        cart_ticks++;
    }
}

float compute_control(){
    float u = -K[0] * state[0] - K[1] * state[1] - K[2] * state[2] - K[3] * state[3];
    u = constrain(u, -MAX_FORCE, MAX_FORCE);
    return u;
}

void setup(){
    Serial.begin(115200);
    pinMode(XA_PIN, INPUT);
    pinMode(XB_PIN, INPUT);
    pinMode(PA_PIN, INPUT);
    pinMode(PB_PIN, INPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    t0 = micros();
    last_print = micros();
    attachInterrupt(XA_PIN, cart_A_handler, CHANGE);
    attachInterrupt(XB_PIN, cart_B_handler, CHANGE);
    attachInterrupt(PA_PIN, pendulum_A_handler, CHANGE);
    attachInterrupt(PB_PIN, pendulum_B_handler, CHANGE);

}

void loop() {
    if (micros()-t0 >= 1000){
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
        F = compute_control();
        update_motor(F);
        t0 = micros();
    }

    // Serial print logs
    if (micros() - last_print >= 100000) {
        Serial.print(micros()/1000);
        Serial.println("ms");
        Serial.print("Force: ");
        Serial.println(F);
        Serial.print("x: ");
        Serial.println(x);
        Serial.print("xdot: ");
        Serial.println(xdot);
        Serial.print("phi: ");
        Serial.println(phi);
        Serial.print("phidot: ");
        Serial.println(phidot);
        Serial.println("=======================================");
        last_print = micros();
    }
   }

