#pragma once
#include <Arduino.h>

#define XA_PIN 1
#define XB_PIN 2
#define PA_PIN 3
#define PB_PIN 9
#define IN_PIN 5
#define ENA_PIN 7
#define LIMIT_L_PIN 10
#define LIMIT_R_PIN 11

const float CART_MASS = 1.54915;
const float TAU = 0.102;
const float V_SS = 0.93;

// const float MAX_FORCE = 7.7;
const float PULLEY_DIAMETER = 0.0225;
const float METERS_PER_TICK = PI * PULLEY_DIAMETER / 403.2;
const float RADIANS_PER_TICK = 2.0 * PI / 2400.0;

extern volatile long pendulum_ticks;
extern volatile long cart_ticks;
extern int ENA;

// Tick-timing velocity (period measurement)
extern volatile float cart_tick_vel;
extern volatile float pendulum_tick_vel;
extern volatile unsigned long cart_last_tick_us;
extern volatile unsigned long pendulum_last_tick_us;

// Returns 0 if no tick received within stale_us microseconds
float read_cart_velocity(unsigned long stale_us = 100000);
float read_pendulum_velocity(unsigned long stale_us = 100000);

float read_position();
float read_angle();
void update_motor(float force, float xdot);
void update_motor_directly();
void coast_motor();
void hardware_setup();
