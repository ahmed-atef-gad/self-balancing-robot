#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v1.h>
// #include "MPU6050.h"

#define enA 3
#define in1 4
#define in2 5

#define enB 9
#define in3 10
#define in4 11

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

MPU6050 mpu6050(Wire);

// PID parameters
double Kp = 20.0;  // Proportional constant
double Ki = 0.0;   // Integral constant
double Kd = 0.0;   // Derivative constant
double output = 0;
double angleY = 0;
double setpoint = 0; // Target angle (upright position)

PID pid(&angleY, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// float previousError = 0;
// float integral = 0;

unsigned long previousTime = 0;

float offset = 0;
void initMpu();
void motorControl(float speed, int direction);

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
  // setpoint = getAngleY();
  setpoint = mpu6050.getAngleY();
  // Serial.println("Robot is ready to go  (●'◡'●)");
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
  offset = 180 - setpoint;
  setpoint += offset;
  Serial.print("Setpoint: ");
  Serial.println(setpoint);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(5);
  pid.SetOutputLimits(-255, 255);
}

void loop() {
  mpu6050.update();
  // update();
  angleY = mpu6050.getAngleY() + offset;
  pid.Compute();
  // float angleY = getAngleY();
  // Serial.print("AngleY: ");
  // Serial.println(angleY);

  // unsigned long currentTime = millis();
  // float elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  // previousTime = currentTime;

  // PID calculations
  
  float error = setpoint - angleY;
  // integral += error * elapsedTime;
  // float derivative = (error - previousError) / elapsedTime;
  // float output = Kp * error + Ki * integral + Kd * derivative;

  // previousError = error;

  // Motor control
  if (output > 0) {
    // Robot tilts forward, move motors forward
    motorControl(output, FORWARD);
    Serial.print("error: ");
    Serial.println(error);
  } else if (output < 0) {
    // Robot tilts backward, move motors backward
    motorControl(-output, BACKWARD);
    Serial.print("error: ");
    Serial.println(error);
  } else {
    // Stop motors
    motorControl(0, STOP);
  }

  // Debugging
  Serial.print("AngleY: ");
  Serial.print(angleY);
  Serial.print(" | PID Output: ");
  Serial.println(output);
}

void motorControl(float speed, int direction) {
  
  int motorSpeed = constrain(speed, 20, 255); // Limit speed to valid PWM range (0-255)
  // int motorSpeed = speed;
  Serial.print("Speed: ");
  Serial.println(motorSpeed);
  if (direction == FORWARD) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, motorSpeed);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, motorSpeed);
  } else if (direction == BACKWARD) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, motorSpeed);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, motorSpeed);
  } else {
    // Stop motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
  }
}

void initMpu() {
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  // setpoint = mpu6050.getAngleY();
  Serial.println("\nRobot is ready to go  (●'◡'●)");
}
