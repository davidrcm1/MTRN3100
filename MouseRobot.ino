#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
//#include "IMUOdometry.hpp"
#include "Wire.h"
#include "Motor.hpp"
#include "PIDController.hpp"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

#define PWM_L 11
#define DIR_L 12
#define PWM_R 9
#define DIR_R 10


mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::Motor motors(PWM_L, DIR_L, PWM_R, DIR_R);
mtrn3100::EncoderOdometry encoder_odometry(16,100);
mtrn3100::PIDController pidL(25, 0, 0);
mtrn3100::PIDController pidR(30, 0, 0);




void setup() {
    Serial.begin(115200);
    Wire.begin();

    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
    pidL.zeroAndSetTarget(encoder.getLeftRotation(), 2.0 * PI);
    pidR.zeroAndSetTarget(encoder.getRightRotation(), 2.0 * PI);
}

void rotateToAngle(float targetAngleDegrees) {
    float targetAngle = targetAngleDegrees * (PI / 180.0); // Convert degrees to radians
    float currentAngle = 0;
    float initialAngle = 0;
    unsigned long lastTime = millis();
    float turnRate = 1.0; // Adjust this to control the speed of rotation

    // Get the initial angle from the IMU
    mpu.update();
    initialAngle = mpu.getAngleZ() * (PI / 180.0); // Ensure the IMU angle is in radians

    // Calculate the shortest direction to rotate
    float angleDifference = targetAngle - initialAngle;
    if (angleDifference > PI) {
        angleDifference -= 2 * PI;
    } else if (angleDifference < -PI) {
        angleDifference += 2 * PI;
    }
    
    int rotationDirection = (angleDifference >= 0) ? 1 : -1; // 1 for right, -1 for left

    while (abs(currentAngle) < abs(angleDifference)) {
        unsigned long now = millis();
        float deltaTime = (now - lastTime) / 1000.0;
        lastTime = now;

        // Apply rotation by setting motors to rotate in the appropriate direction
        motors.setPWM(rotationDirection * 100, -rotationDirection * 100); // Adjust the speed values as necessary

        // Update current angle based on encoder readings
        float leftRotation = encoder.getLeftRotation();
        float rightRotation = encoder.getRightRotation();
        float deltaH = (leftRotation - rightRotation) * turnRate * deltaTime * rotationDirection;
        currentAngle += deltaH;

        // Update IMU and correct angle
        mpu.update();
        float imuAngle = (mpu.getAngleZ() * (PI / 180.0)) - initialAngle; // Ensure the IMU angle is in radians

        // Correct the current angle using IMU data
        currentAngle = imuAngle;

        // Print the current angle for debugging
        Serial.print("Current Angle: ");
        Serial.println(currentAngle * (180.0 / PI)); // Print angle in degrees for readability

        delay(10);
    }

    // Stop the motors after completing the rotation
    motors.setPWM(0, 0);
}




void loop() {
    delay(50);
    //motors.setPWM(pidL.compute(encoder.getLeftRotation()), pidR.compute(encoder.getRightRotation()));
    
    motors.setPWM(pidR.compute(encoder.getRightRotation()), pidR.compute(encoder.getRightRotation()));
   

    /*Serial.print("ODOM:\t\t");
    Serial.print(encoder_odometry.getX());
    Serial.print(",\t\t");
    Serial.print(encoder_odometry.getY());
    Serial.print(",\t\t");
    Serial.print(encoder_odometry.getH());
    Serial.println();*/
    
}
