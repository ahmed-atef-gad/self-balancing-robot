#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#define enA 3
#define in1 4
#define in2 5

#define enB 9
#define in3 10
#define in4 11

#define INTERRUPT_PIN 2  

MPU6050 mpu6050(Wire);
void initMpu();

void setup() {
  Serial.println("Initializing>>>>");

  Serial.begin(9600);
  Wire.begin();
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  analogWrite(enA, 100);
  analogWrite(enB, 100);

  initMpu();
}

void loop() {
  mpu6050.update();
  float angleY = mpu6050.getAngleY();
  
  Serial.print("y: ");
  Serial.println(angleY);
  
}

void initMpu()
{
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("\nRobot is ready to go  (●'◡'●)");
}