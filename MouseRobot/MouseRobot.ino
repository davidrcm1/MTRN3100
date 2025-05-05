#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
//#include "IMUOdometry.hpp"
#include "Wire.h"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Lidar.hpp"
#include "Display.hpp"
#include <MPU6050_light.h>


// Chech MPU reset**


MPU6050 mpu(Wire);

#define EN_1_A 2 // These are the pins for the PCB encoder
#define EN_1_B 7 // These are the pins for the PCB encoder
#define EN_2_A 3 // These are the pins for the PCB encoder
#define EN_2_B 8 // These are the pins for the PCB encoder

#define PWM_L 11
#define DIR_L 12
#define PWM_R 9
#define DIR_R 10

#define MAXpwm 100 // MAX Motor speed for PID

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::Motor motors(PWM_L, DIR_L, PWM_R, DIR_R);
mtrn3100::EncoderOdometry encoder_odometry(16, 100);
mtrn3100::PIDController pidL(4, 0 , 0);
mtrn3100::PIDController pidR(4, 0, 0);
mtrn3100::PIDController pidIMU(5.2, 0.25, 0.3);
mtrn3100::PIDController pidIMUStraight(8.7, 1.3, 4.1); // Change D to 3.2 (Last check in MakerSpace)
mtrn3100::PIDController pidLidar(1.0, 0.1, 0.01); // Adjust PID gains as needed
mtrn3100::Lidar lidar(A1, A0, A2);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Set up the IMU
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // Stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);
  Serial.println("Done!\n");
  pidL.zeroAndSetTarget(encoder.getLeftRotation(), 2.0 * PI);
  pidR.zeroAndSetTarget(encoder.getRightRotation(), 2.0 * PI);

  // Set up Lidar
  Serial.println("Setting up Lidar...");
  lidar.setup();
  Serial.println("Lidar setup complete.");
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// f forward
// b back.
// USE THIS FOR WEEK 8

void move(char direction) {
  double pwmValue = 50; // Max motor speed

  while (true) {
    // Read the front Lidar sensor value
    int frontLidarReading = lidar.readFront();

    // Check if the front Lidar reading is less than 60
    if (frontLidarReading < 60) {
      Serial.println("Obstacle detected! Stopping the robot.");
      break;
    }

    // Set motor PWM based on direction
    if (direction == 'f') {
      motors.setPWM(pwmValue + 1.5, pwmValue); // Move forward
    } else if (direction == 'b') {
      motors.setPWM(-pwmValue - 1.5, -pwmValue); // Move backward
    } else {
      Serial.println("Invalid direction! Use 'f' for forward or 'b' for backward.");
      break;
    }

    delay(10); // Small delay to prevent overwhelming the loop
  }

  motors.setPWM(0, 0); // Stop the motors
}


// r & l for direction

void rotate(char dir) {
  //reset IMU
  mpu.calcOffsets(true, true);

  //Get a Multiple Flag
  int mul = (dir == 'l') ? -1 : 1;
  if (dir == 'l') Serial.println("Turning Left");

  //Zero the PID + Calculate Error
  mpu.update();
  pidIMU.zeroAndSetTarget(mpu.getAngleZ(), -mul * (90.0));
  auto setpoint = (dir == 'l') ? -90 : 90;
  auto spd = pidIMU.compute(mpu.getAngleZ());

  
  while (abs(pidIMU.getError()) > 0.6) {
    spd = pidIMU.compute(mpu.getAngleZ());
    spd *= (pidIMU.getError() < 3.0) ? 4 : 1;
    motors.setPWM(mul * abs(spd), -mul * abs(spd));
    mpu.update();
    ///if(abs(pidIMU.getError() < 0.6) break;

    Serial.print(mpu.getAngleZ());
    Serial.print(", ");
    Serial.print(spd);
    Serial.print(", ");
    Serial.println(pidIMU.getError());

  }
  motors.setPWM(0, 0);
}
// Check why the left turn speed is different

void chainTask(const char chainString[]) {
  //Complete 8 movements given in a string
  //Example: Start at (1,1,S) given "fflfrflf"
  Serial.println("Starting Chaining");
  delay(1000);
  int length = strlen(chainString);
  for (int i = 0; i < length; i++) {
    delay(100);
    Serial.print("Current character: ");
    Serial.println(chainString[i]);

    if (chainString[i] == 'f') {
      //Move forward one cell
      drive_forward_cell_PID('f');
      Serial.println("Moving forward 1");
      //Small Delay
      delay(1000);
    } else if (chainString[i] == 'l') {
      //Turn 90 degrees counter-clockwise
      // rotate90(true); //True for left
      rotate('l');
      Serial.println("Turning left 90");
      //Small Delay
      delay(1000);
    } else if (chainString[i] == 'r') {
      //Turn 90 degrees clockwise
      //rotate90(false); //False for right
      rotate('r');
      Serial.println("Turning right 90");
      //Small Delay
      delay(1000);
    }
  }
}

// FOR THE WEEK 8 TASK
// Rotate in a 360, stopping each 90 degrees for 1 sec
// PARAMS:
//  char direction - pass in 'r' for right turn (clockwise)
//                   pass in 'l' for left turn (counter clockswise)
// RETURN:
//  N/A
void rotate_full(char direction) {
  rotate(direction);
  delay(1000);
  rotate(direction);
  delay(1000);
  rotate(direction);
  delay(1000);
  rotate(direction);
  delay(1000);

}

// FOR THE WEEK 8 TASK
// Drive straight until the lidar is 7.5cm away from the wall
// PARAMS:
//  char direction - pass in 'f' for forwards
//                   pass in 'b' for backwards
// RETURN:
//  N/A
//
// NOTE: if it is drifting left, reduce frequency var, if it is drifting right, increase frequency var
//
void drive_straight(char direction) {
  double pwmValue = 50; // Max motor speed

  int round_count = 0; // when this is 5, increase left motor to stop drift?
  int frequency = 10; // every [frequency]th round of the while loop, increase the
  // motor speed of the left wheel to account for left drift.

  while (true) {
    // Read the front Lidar sensor value
    int frontLidarReading = lidar.readFront();

    // Check if the front Lidar reading is less than 60
    // MODIFYING to 75 as the cells are 25cm, robot is 9cmx9cm, wall is 0.5cm, so to be in
    // the middle of the final cell we need to stop about 7.5 cm away from the wall
    if (frontLidarReading < 75) {
      Serial.println("Obstacle detected! Stopping the robot.");
      break;
    }

    int offs = (round_count % frequency == 0) * 1;
    // Set motor PWM based on direction
    if (direction == 'f') {
      motors.setPWM(pwmValue + offs, pwmValue); // Move forward
    } else if (direction == 'b') {
      motors.setPWM(-pwmValue, -pwmValue); // Move backward
    } else {
      Serial.println("Invalid direction! Use 'f' for forward or 'b' for backward.");
      break;
    }
    /*
      - when the pwm are the same, the robot drift slightly to the left, but not enough that
      an integer difference on 50 seems to straighten is
      - can increase the left once every 10 loops or something to combat it?
    */

    delay(10); // Small delay to prevent overwhelming the loop
    round_count++;
  }

  motors.setPWM(0, 0); // Stop the motors
}

// FOR THE WEEK 8 TASK
// Drive forward one cell. Uses a time to work out how long a cell has been, or if we overshot the time,
// the lidar stops when it is 7.5cm from a wall
// PARAMS:
//  char direction - pass in 'f' for forwards
//                   pass in 'b' for backwards
// RETURN:
//  N/A
//
// NOTE: if it is drifting left, reduce frequency var, if it is drifting right, increase frequency var
//
void drive_forward_cell(char direction) {

  double pwmValue = 50; // Max motor speed

  int round_count = 0; // when this is 5, increase left motor to stop drift?
  int frequency = 10;

  unsigned long startTime = millis();
  unsigned long timeout = 4500; // 4200 seconds timeout

  while (millis() - startTime < timeout && lidar.readFront() >= 76) { // 7.6cm from wall

    int offs = (round_count % frequency == 0) * 1;
    // Set motor PWM based on direction
    if (direction == 'f') {
      motors.setPWM(pwmValue + offs, pwmValue); // Move forward
    }
    /*
      - when the pwm are the same, the robot drift slightly to the left, but not enough that
      an integer difference on 50 seems to straighten is
      - can increase the left once every 5 loops or something to combat it?
    */

    delay(10); // Small delay to prevent overwhelming the loop
    round_count++;
  }

  motors.setPWM(0, 0); // Stop the motors
}

//IMU-PID Version of the code above:
//Uses mpu to adjust left motor only since that one is the one that is unpowered.
void drive_forward_cell_PID(char direction) {
  //Reset IMU:
  mpu.calcOffsets(true, true);
  mpu.update();
  pidIMUStraight.zeroAndSetTarget(mpu.getAngleZ(), 0);

  double pwmValue = 50; // Max motor speed

  unsigned long startTime = millis();
  unsigned long timeout = 4200; // 4200 seconds timeout

  while (millis() - startTime < timeout && lidar.readFront() >= 76) { // 7.6cm from wall

    // Set motor PWM based on direction
    if (direction == 'f') {
      mpu.update();
      Serial.print(pidIMUStraight.compute(mpu.getAngleZ()));
      Serial.print(", ");
      Serial.println(pidIMUStraight.getError());
      motors.setPWM(pwmValue - pidIMUStraight.compute(mpu.getAngleZ()), pwmValue); // Move forward
    }
    delay(10); // Small delay to prevent overwhelming the loop
  }

  motors.setPWM(0, 0); // Stop the motors
}


void loop() {
  chainTask("rl");
  //rotate_full('r');


  exit(1);


}
