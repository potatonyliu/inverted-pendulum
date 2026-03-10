#include <Arduino.h>

#define PA_PIN 2
#define PB_PIN 3
#define IN1_PIN 5
#define IN2_PIN 6
#define ENA_PIN 7

const float MAX_FORCE = 3;
const float PULLEY_DIAMETER = 0.0225;
const float METERS_PER_TICK = PI * PULLEY_DIAMETER / 403.2;
const float RADIANS_PER_TICK = 2.0 * PI / 2400.0;

volatile long pendulum_ticks = 0;
volatile long cart_ticks = 0;

unsigned long t0;
unsigned long last_print;
float K[4] = {-1.0000, -3.1708, 80.2158, 24.9766};
float state[4] = {0.0, 0.0, 0.1, 0.0};
float F;
float x = 0;
float phi = PI;
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
    int A = digitalRead(PA_PIN);
    int B = digitalRead(PB_PIN);
    if (A == B){
        pendulum_ticks--;
    }
    else{
        pendulum_ticks++;
    }
}

void pendulum_B_handler(){
    int A = digitalRead(PA_PIN);
    int B = digitalRead(PB_PIN);
    if (A != B){
        pendulum_ticks--;
    }
    else{
        pendulum_ticks++;
    }
}


float compute_control(){
    float u = -K[0] * state[0] - K[1] * state[1] - K[2] * state[2] - K[3] * state[3];
    u = constrain(u, -MAX_FORCE, MAX_FORCE);
    return u;
}

void setup(){
    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    Serial.println("System Initialized...");
    pinMode(PA_PIN, INPUT_PULLUP);
    pinMode(PB_PIN, INPUT_PULLUP);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    t0 = micros();
    last_print = micros();
    attachInterrupt(digitalPinToInterrupt(PA_PIN), pendulum_A_handler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PB_PIN), pendulum_B_handler, CHANGE);

}

void loop() {
    Serial.println(read_angle(), 10);
    delay(20);
}

