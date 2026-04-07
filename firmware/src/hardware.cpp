#include "hardware.h"
#include "states.h"

volatile long pendulum_ticks = 0;
volatile long cart_ticks = 0;
int ENA = 0;

volatile float cart_tick_vel = 0.0f;
volatile float pendulum_tick_vel = 0.0f;
volatile unsigned long cart_last_tick_us = 0;
volatile unsigned long pendulum_last_tick_us = 0;

// meters
float read_position(){
    return cart_ticks * METERS_PER_TICK;
}

// radians
float read_angle(){
    return pendulum_ticks * RADIANS_PER_TICK + PI;
}

// Returns 0 if no tick received within stale_us (system stopped)
float read_cart_velocity(unsigned long stale_us){
    if (micros() - cart_last_tick_us > stale_us) return 0.0f;
    return cart_tick_vel;
}

float read_pendulum_velocity(unsigned long stale_us){
    if (micros() - pendulum_last_tick_us > stale_us) return 0.0f;
    return pendulum_tick_vel;
}

void update_motor(float force, float xdot){
    float u = (force * TAU / CART_MASS + xdot) * (255.0 / V_SS);

    if (u >= 0){
        digitalWrite(IN_PIN, HIGH);
    }
    else{
        digitalWrite(IN_PIN, LOW);
    }

    int pwm = (int)(u >= 0 ? u : -u);
    if (pwm > 255) pwm = 255;
    ENA = pwm;
    analogWrite(ENA_PIN, pwm);
}

// Newtons
// void update_motor(float force){
//     if (force >= 0){
//         digitalWrite(IN1_PIN, HIGH);
//         digitalWrite(IN2_PIN, LOW);
//     }
//     else{
//         digitalWrite(IN1_PIN, LOW);
//         digitalWrite(IN2_PIN, HIGH);
//     }
//     float magForce = (force >= 0) ? force : -force;
//     int pwm = (int)(magForce / MAX_FORCE * 255);
//     analogWrite(ENA_PIN, pwm);
// }

void update_motor_directly(){
    if (ENA >= 0){
        digitalWrite(IN2_PIN, HIGH);
    }
    else{
        digitalWrite(IN2_PIN, LOW);
    }
    int pwm = (int)(ENA >= 0 ? ENA : -ENA);
    analogWrite(ENA_PIN, pwm);
}

void coast_motor(){
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ENA = 0;
    analogWrite(ENA_PIN, 0);
}

void pendulum_A_handler(){
    unsigned long now = micros();
    int A = digitalRead(PA_PIN);
    int B = digitalRead(PB_PIN);
    int dir;
    if (A == B) { pendulum_ticks--; dir = -1; }
    else        { pendulum_ticks++; dir =  1; }
    unsigned long dt = now - pendulum_last_tick_us;
    if (dt > 0)
        pendulum_tick_vel = dir * RADIANS_PER_TICK / (dt * 1e-6f);
    pendulum_last_tick_us = now;
}

void pendulum_B_handler(){
    unsigned long now = micros();
    int A = digitalRead(PA_PIN);
    int B = digitalRead(PB_PIN);
    int dir;
    if (A != B) { pendulum_ticks--; dir = -1; }
    else        { pendulum_ticks++; dir =  1; }
    unsigned long dt = now - pendulum_last_tick_us;
    if (dt > 0)
        pendulum_tick_vel = dir * RADIANS_PER_TICK / (dt * 1e-6f);
    pendulum_last_tick_us = now;
}

void cart_A_handler(){
    unsigned long now = micros();
    int A = digitalRead(XA_PIN);
    int B = digitalRead(XB_PIN);
    int dir;
    if (A == B) { cart_ticks--; dir = -1; }
    else        { cart_ticks++; dir =  1; }
    unsigned long dt = now - cart_last_tick_us;
    if (dt > 0)
        cart_tick_vel = dir * METERS_PER_TICK / (dt * 1e-6f);
    cart_last_tick_us = now;
}

void cart_B_handler(){
    unsigned long now = micros();
    int A = digitalRead(XA_PIN);
    int B = digitalRead(XB_PIN);
    int dir;
    if (A != B) { cart_ticks--; dir = -1; }
    else        { cart_ticks++; dir =  1; }
    unsigned long dt = now - cart_last_tick_us;
    if (dt > 0)
        cart_tick_vel = dir * METERS_PER_TICK / (dt * 1e-6f);
    cart_last_tick_us = now;
}

void limit_switch_handler(){
    // currentState = IDLE;
    event = "crash (limit_switch - DISABLED)";
}

void hardware_setup(){
    pinMode(XA_PIN, INPUT);
    pinMode(XB_PIN, INPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    attachInterrupt(XA_PIN, cart_A_handler, CHANGE);
    attachInterrupt(XB_PIN, cart_B_handler, CHANGE);
    attachInterrupt(PA_PIN, pendulum_A_handler, CHANGE);
    attachInterrupt(PB_PIN, pendulum_B_handler, CHANGE);
    attachInterrupt(LIMIT_L_PIN, limit_switch_handler, FALLING);
    attachInterrupt(LIMIT_R_PIN, limit_switch_handler, FALLING);
    pinMode(PA_PIN, INPUT_PULLUP);
    pinMode(PB_PIN, INPUT_PULLUP);
    pinMode(LIMIT_L_PIN, INPUT_PULLUP);
    pinMode(LIMIT_R_PIN, INPUT_PULLUP);
}
