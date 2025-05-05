#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h> // Ensure you include the correct library

namespace mtrn3100 {
  class Lidar {
  public:
      Lidar(int frontPin, int leftPin, int rightPin)
          : lidarFront_pin(frontPin), lidarLeft_pin(leftPin), lidarRight_pin(rightPin) {}

      Lidar() {}

      void setup()
      {
          // Wire.begin();
          // Serial.begin(115200);

          pinMode(lidarFront_pin, OUTPUT);
          pinMode(lidarLeft_pin, OUTPUT);
          pinMode(lidarRight_pin, OUTPUT);

          digitalWrite(lidarFront_pin, LOW);
          digitalWrite(lidarLeft_pin, LOW);
          digitalWrite(lidarRight_pin, LOW);

          // Initialize front sensor
          digitalWrite(lidarFront_pin, HIGH);
          delay(50);
          sensor1.init();
          sensor1.configureDefault();
          sensor1.setTimeout(250);
          sensor1.setAddress(0x54); // New I2C address for sensor 1
          delay(50);

          // Initialize left sensor
          digitalWrite(lidarLeft_pin, HIGH);
          delay(50);
          sensor2.init();
          sensor2.configureDefault();
          sensor2.setTimeout(250);
          sensor2.setAddress(0x56); // New I2C address for sensor 2
          delay(50);

          // Initialize right sensor
          digitalWrite(lidarRight_pin, HIGH);
          delay(50);
          sensor3.init();
          sensor3.configureDefault();
          sensor3.setTimeout(250);
          sensor3.setAddress(0x58); // New I2C address for sensor 3
      }

      int readFront()
      {
          if (sensor1.timeoutOccurred())
          {
              Serial.println("Front sensor timeout!");
              return -1;
          }
          return sensor1.readRangeSingleMillimeters();
      }

      int readLeft()
      {
          if (sensor2.timeoutOccurred())
          {
              Serial.println("Left sensor timeout!");
              return -1;
          }
          return sensor2.readRangeSingleMillimeters();
      }

      int readRight()
      {
          if (sensor3.timeoutOccurred())
          {
              Serial.println("Right sensor timeout!");
              return -1;
          }
          return sensor3.readRangeSingleMillimeters();
      }

      void printReadings()
      {
          Serial.print(readFront());
          Serial.print(" | ");
          Serial.print(readLeft());
          Serial.print(" | ");
          Serial.print(readRight());
          Serial.println();
      }

  private:
      VL6180X sensor1;
      VL6180X sensor2;
      VL6180X sensor3;

      const int lidarFront_pin = A1;
      const int lidarLeft_pin = A0;
      const int lidarRight_pin = A2;
  };
}
