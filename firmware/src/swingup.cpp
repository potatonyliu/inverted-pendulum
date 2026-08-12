// Energy-pumping swing-up. Algorithm is the same as the standalone
// `firmware/swingup` file from the swing-up branch. Motor drive goes
// through update_motor_directly() (analogWrite-based PWM) instead of
// raw digitalWrite on ENA — the original could use digitalWrite because
// its coast_motor() also used digitalWrite, so ENA_PIN stayed in SIO
// mode. Main's coast_motor() uses analogWrite(0), which leaves the pin
// in PWM hardware-function, and mixing digitalWrite calls on top of
// that produces asymmetric drive. Active only when currentState ==
// SWINGUP. Hand-off to LQR happens in main.cpp when |phi| < 0.2.

#include <Arduino.h>
#include "hardware.h"
#include "states.h"

void motor_forward(int ena_amount) { //static void motor_forward() {
    ENA = ena_amount;
    update_motor_directly();
}

void motor_backward(int ena_amount) { //static void motor_backward() {
    ENA = -ena_amount;
    update_motor_directly();
}

void swingup_enter() {
    // No-op: statics inside swingup_tick persist exactly like the
    // original program-statics.
}

void swingup_tick(float swingup_phi, float swingup_phidot, float new_max_speed = 150) {
    if (currentState != SWINGUP) return;

    static float motor_speed_max = 150;
    static float motor_speed_start = motor_speed_max;
    static float motor_speed = motor_speed_start;
    static float motor_speed_increase_per_tick = 0.05;
    static int direction = 0;
    static bool direction_change = false;
    static bool direction_locked = true;

    motor_speed_max = new_max_speed;
    motor_speed = motor_speed_max;


    if (subState == INITIALIZING) {
        if (fabsf(swingup_phi) > 3.12 &&    //if we're near the bottom, near abs 3.14
            fabsf(swingup_phidot) < 0.5f) { //and we're almost still
            subState = DOWN_AND_STILL;
            Serial.print("subState from: INITIALIZING to: DOWN_AND_STILL");
        }
    } else if (subState == DOWN_AND_STILL) {
        Serial.print("PHI: "); Serial.println(swingup_phi, 6);
        if (fabsf(swingup_phi) < 3.12 &&
            fabsf(swingup_phidot) < 0.05f) {
                subState = LOW_SWING;
                Serial.println("Setting subState to: LOW_SWING");
            }
        else {
            direction = 1;
            motor_forward(motor_speed);
        }
    } else if (subState == LOW_SWING) {
        if (fabsf(swingup_phi) < 2.00) {
            subState = HIGH_SWING;
            Serial.println("Setting subState to: HIGH_SWING");
        }

        if (swingup_phidot > 0.00f) {
            // if (direction == 1) {Serial.print("LOW_SWING: motor_forward() at PHI: "); Serial.print(swingup_phi, 6); Serial.print(" PHIDOT: "); Serial.println(swingup_phidot, 6);}
            direction = -1;
        } else {
            // if (direction == -1) {Serial.print("LOW_SWING: motor_backward() at PHI: "); Serial.print(swingup_phi, 6); Serial.print(" PHIDOT: "); Serial.println(swingup_phidot, 6);}
            direction = 1;
        }

        if (direction == 1) {
            motor_forward(motor_speed);
        }
        else if (direction == -1) {
            motor_backward(motor_speed);
        }
    } else if (subState == HIGH_SWING) {
        // Serial.print(swingup_phidot, 6); Serial.print(","); Serial.print(direction); Serial.print(","); Serial.println(swingup_phi, 6);

        if (fabsf(swingup_phi) < 1.00) {
            // subState = ALMOST_THERE;
        }

        if (fabsf(swingup_phi) < 2.00 && fabsf(swingup_phidot) < 0.5f) { //if we're away from the bottom, and slowing down
            direction_locked = false;
        }

        if (fabsf(swingup_phi) > 3.00 && fabsf(swingup_phidot) > 2.00 && direction_locked == false) { //if we're close to the bottom, swinging fast, and unlocked
            direction *= -1;
            Serial.print("Changing Direction at PHI: "); Serial.println(swingup_phi, 6);
            direction_locked = true;
        }

        if (direction == 1) {
            motor_forward(motor_speed);
        }
        else if (direction == -1) {
            motor_backward(motor_speed);
        }
    } else if (subState == ALMOST_THERE) {
        if (fabsf(swingup_phi) < 2.00 && fabsf(swingup_phidot) < 0.5f) { //if we're away from the bottom, and slowing down
            direction_locked = false;
        }

        if (fabsf(swingup_phi) > 3.00 && fabsf(swingup_phidot) > 2.00 && direction_locked == false) { //if we're close to the bottom, swinging fast, and unlocked
            direction *= -1;
            Serial.print("Changing Direction at PHI: "); Serial.println(swingup_phi, 6);
            direction_locked = true;
        }

        if (direction == 1) {
            motor_forward(motor_speed*.995); //slow down the motor as we get closer and closer to the top
        }
        else if (direction == -1) {
            motor_backward(motor_speed*0.995);
        }
    }





    // if (fabsf(swingup_phi) > 3.12) { //if we're near the bottom, near abs 3.14
    //     direction_change = false; //reset direction change at the bottom of every swing
    //     if (fabsf(swingup_phidot) < 1.0f) { //if we're nearly still
    //         motor_forward(motor_speed);
    //     }
    //     else {
    //         coast_motor();
    //     }
    // } else if (fabsf(swingup_phi) > 2.5) { //we're away from the bottom, but just a bit
    //     if (swingup_phidot < 0) {
    //         motor_forward(motor_speed);
    //         direction = 1;
    //         motor_speed = motor_speed_start;
    //     }
    //     else if (swingup_phidot > 0) {
    //         motor_backward(motor_speed);
    //         direction = -1;
    //         motor_speed = motor_speed_start;
    //     }
    // } else if (fabsf(swingup_phi) > 1.57) { //we're far away from the bottom, but not yet over half way up
    //     if (fabsf(swingup_phidot) < 0.05 && direction_change == false) {
    //         //keep driving the same direction until the top of the swing
    //         //note that we're about to change direction, but don't change just yet
    //         direction_change = true;
    //     }
        
    //     if (fabsf(swingup_phidot) > 2.5 && direction_change == true) {
    //         //once we crest the top of the arc and increase velocity on the way back down
    //         //then start driving in the other direction.
    //         direction *= -1;
    //         motor_speed = motor_speed_start;
    //         direction_change = false;
    //     }

    //     if (direction == 1) {
    //         motor_forward(motor_speed);
    //     }
    //     else if (direction == -1) {
    //         motor_backward(motor_speed);
    //     }
    // }

    // if (motor_speed <= motor_speed_max) {
    //     motor_speed += motor_speed_increase_per_tick;
    // }
    //  else if (fabsf(swingup_phi) > 0.2) { //we're over half way up
    //     Serial.print("swingup_phi: "); Serial.println(swingup_phi, 6);
    //     if (fabsf(swingup_phidot) < 0.02 && direction_change == false) { //keep driving the same direction until the top of the swing
    //         direction *= -1;
    //         direction_change = true;
    //     }
        
    //     if (direction == 1) {
    //         motor_forward(motor_speed_start);
    //     }
    //     else if (direction == -1) {
    //         motor_backward(motor_speed_start);
    //     }
    // }

}

void swingup_tick_without_state(float swingup_phi, float swingup_phidot) {
    if (currentState != SWINGUP) return;

    static float motor_speed_max = 220;
    static float motor_speed_start = 220;
    static float motor_speed = motor_speed_start;
    static float motor_speed_increase_per_tick = 0.05;
    static int direction = 0;
    static bool direction_change = false;


    if (fabsf(swingup_phi) > 3.12) { //if we're near the bottom, near abs 3.14
        direction_change = false; //reset direction change at the bottom of every swing
        if (fabsf(swingup_phidot) < 1.0f) { //if we're nearly still
            motor_forward(motor_speed);
        }
        else {
            coast_motor();
        }
    } else if (fabsf(swingup_phi) > 2.5) { //we're away from the bottom, but just a bit
        if (swingup_phidot < 0) {
            motor_forward(motor_speed);
            direction = 1;
            motor_speed = motor_speed_start;
        }
        else if (swingup_phidot > 0) {
            motor_backward(motor_speed);
            direction = -1;
            motor_speed = motor_speed_start;
        }
    } else if (fabsf(swingup_phi) > 1.57) { //we're far away from the bottom, but not yet over half way up
        if (fabsf(swingup_phidot) < 0.05 && direction_change == false) {
            //keep driving the same direction until the top of the swing
            //note that we're about to change direction, but don't change just yet
            direction_change = true;
        }
        
        if (fabsf(swingup_phidot) > 2.5 && direction_change == true) {
            //once we crest the top of the arc and increase velocity on the way back down
            //then start driving in the other direction.
            direction *= -1;
            motor_speed = motor_speed_start;
            direction_change = false;
        }

        if (direction == 1) {
            motor_forward(motor_speed);
        }
        else if (direction == -1) {
            motor_backward(motor_speed);
        }
    }

    if (motor_speed <= motor_speed_max) {
        motor_speed += motor_speed_increase_per_tick;
    }
    //  else if (fabsf(swingup_phi) > 0.2) { //we're over half way up
    //     Serial.print("swingup_phi: "); Serial.println(swingup_phi, 6);
    //     if (fabsf(swingup_phidot) < 0.02 && direction_change == false) { //keep driving the same direction until the top of the swing
    //         direction *= -1;
    //         direction_change = true;
    //     }
        
    //     if (direction == 1) {
    //         motor_forward(motor_speed_start);
    //     }
    //     else if (direction == -1) {
    //         motor_backward(motor_speed_start);
    //     }
    // }

}

void old_swingup_tick(float swingup_phi, float swingup_phidot, int new_direction = 0) {
    if (currentState != SWINGUP) return;

    static float speed = 220;

    static int swingup_direction = 0;

    if (new_direction != 0) {
        swingup_direction = new_direction;
        Serial.print("Direction Change at PHI: "); Serial.println(swingup_phi, 6);
        Serial.print("Direction: "); Serial.println(swingup_direction);
        speed = 220;
    }

    if (fabsf(swingup_phi) > 3.12) { //if we're near the bottom, near abs 3.14
        if (fabsf(swingup_phidot) < 1.0f) { //if we're nearly still
            // Serial.print("PHIDOT NEAR BOTTOM:"); Serial.println(phidot, 4);
            static bool          kick_dir   = true;
            static unsigned long kick_timer = 0;

            motor_forward();
        }
        else {
            coast_motor();
        }
    }
    else { //we're away from the bottom, away from abs 3.14
        if (swingup_direction == -1) {
            // motor_forward(fabsf(speed));
            motor_forward();
        }
        else if (swingup_direction == 1) {
            // motor_backward(fabsf(speed));
            motor_backward();
        }
    }
}

void recenter_tick(float current_x) {
    // if (currentState != RECENTER) return;

    if (current_x > 0.01) {
        motor_forward();
    }  
    else if (current_x < -0.01) {
        motor_backward();
    }
    else {
        motor_forward(0);
    }
}
