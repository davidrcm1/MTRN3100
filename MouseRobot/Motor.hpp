#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {


// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm1, uint8_t dir1, uint8_t pwm2, uint8_t dir2) :  pwm_pin1(pwm1), dir_pin1(dir1) , pwm_pin2(pwm2), dir_pin2(dir2) {
        pinMode(pwm1, OUTPUT);
        pinMode(dir1, OUTPUT);
        pinMode(pwm2, OUTPUT);
        pinMode(dir2, OUTPUT);

    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm1, int16_t pwm2) {
        
      // TODO: Output PWM signal between 0 - 255.
        analogWrite(pwm_pin1, abs(pwm1));
        analogWrite(pwm_pin2, abs(pwm2));

      // TODO: Output digital direction pin based on if input signal is positive or negative.
        //TODO: Inverted for motors:

        // left
        digitalWrite(dir_pin1, (pwm1 >= 0 ? LOW : HIGH));
        // right
        digitalWrite(dir_pin2, (pwm2 >= 0 ? HIGH : LOW));

    }

private:
    const uint8_t pwm_pin1;
    const uint8_t pwm_pin2;
    const uint8_t dir_pin1;
    const uint8_t dir_pin2;
};

}  // namespace mtrn3100
