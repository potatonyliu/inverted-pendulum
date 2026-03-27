#pragma once
#include <Arduino.h>

#define XA_PIN 1
#define XB_PIN 2
#define PA_PIN 3
#define PB_PIN 9
#define IN1_PIN 5
#define IN2_PIN 6
#define ENA_PIN 7
#define LIMIT_L_PIN 10
#define LIMIT_R_PIN 11

const float MAX_FORCE = 3;
const float PULLEY_DIAMETER = 0.0225;
const float METERS_PER_TICK = PI * PULLEY_DIAMETER / 403.2;
const float RADIANS_PER_TICK = 2.0 * PI / 2400.0;

extern volatile long pendulum_ticks;
extern volatile long cart_ticks;
extern int ENA;

float read_position();
float read_angle();
void update_motor(float force);
void update_motor_directly();
void coast_motor();
void hardware_setup();
