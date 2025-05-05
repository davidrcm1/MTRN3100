
#pragma once
#pragma once

#include <math.h>

namespace mtrn3100 {

    class PIDController {
    public:
        PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

        // Compute the output signal required from the current/actual value.
        float compute(float input) {

            curr_time = micros();
            dt = static_cast<float>(curr_time - prev_time) / 1e6;
            prev_time = curr_time;

            error = setpoint - (input - zero_ref);
            /*Serial.print("Target: ");
            Serial.print(setpoint);
            Serial.print(", ");
            Serial.print("Input: ");
            Serial.print(input);
            Serial.print(", ");
            Serial.print("Error: ");
            Serial.print(error);
            Serial.print(", ");*/
            
            //if(abs(error) < tolerance) return 0;

            integral = integral + error * dt;
            derivative = (error - prev_error) / dt;
            //P control:
            //output = kp * error;

            //PID control:
            output = (kp * error) + (ki * integral) + (kd * derivative);
            if(output < 0) {
              if(output < -125) output = -125;
            } else {
              if(output > 125) output = 125;
            }
            /*Serial.print("Output: ");
            Serial.println(output);*/

            prev_error = error;
            
            
            return output;
        }

        

        // Function used to return the last calculated error. 
        // The error is the difference between the desired position and current position. 
        void tune(float p, float i, float d) {
            kp = p;
            ki = i;
            kd = d;
        }

        float getError() {
            return error;
        }

        // This must be called before trying to achieve a setpoint.
        // The first argument becomes the new zero reference point.
        // Target is the setpoint value.
        void zeroAndSetTarget(float zero, float target) {
            prev_time = micros();
            zero_ref = zero;
            setpoint = target;
        }

    public:
        uint32_t prev_time, curr_time = micros();
        float dt;

    private:
        float kp, ki, kd;
        float error, derivative, integral, output;
        float prev_error = 0;
        float setpoint = 0;
        float zero_ref = 0;
        float tolerance = 0.05;

        //LiDar setpoints
        int LR_setpoint = 15;
        int centre_setpoint = 43;
        
    };

}  // namespace mtrn3100
