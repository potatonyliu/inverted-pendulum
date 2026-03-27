#include "hardware.h"
#include "states.h"

volatile long pendulum_ticks = 0;
volatile long cart_ticks = 0;
int ENA = 0;

// meters
float read_position(){
    return cart_ticks * METERS_PER_TICK;
}

// radians
float read_angle(){
    return pendulum_ticks * RADIANS_PER_TICK;
}

// Newtons
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

void update_motor_directly(){
    if (ENA >= 0){
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
    }
    else{
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
    }
    analogWrite(ENA_PIN, ENA);
}

void coast_motor(){
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ENA = 0;
    analogWrite(ENA_PIN, 0);
}

void pendulum_A_handler(){
    int A = digitalRead(PA_PIN);
    int B = digitalRead(PB_PIN);
    if (A == B) pendulum_ticks--;
    else        pendulum_ticks++;
}

void pendulum_B_handler(){
    int A = digitalRead(PA_PIN);
    int B = digitalRead(PB_PIN);
    if (A != B) pendulum_ticks--;
    else        pendulum_ticks++;
}

void cart_A_handler(){
    int A = digitalRead(XA_PIN);
    int B = digitalRead(XB_PIN);
    if (A == B) cart_ticks--;
    else        cart_ticks++;
}

void cart_B_handler(){
    int A = digitalRead(XA_PIN);
    int B = digitalRead(XB_PIN);
    if (A != B) cart_ticks--;
    else        cart_ticks++;
}

void limit_switch_handler(){
    currentState = IDLE;
    event = "crash (limit_switch)";
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
