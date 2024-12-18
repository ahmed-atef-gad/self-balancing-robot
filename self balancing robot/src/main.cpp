#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v1.h>
// #include "MPU6050.h"

#define enA 3
#define in1 4
#define in2 5

#define enB 9
#define in3 11
#define in4 10

#define FORWARD 1
#define BACKWARD -1
#define STOP 0
#define Debugging 1

MPU6050 mpu6050(Wire);

// PID parameters
double Kp = 50.0; // Proportional constant
double Ki = 1.5;  // Integral constant
double Kd = 2.0;  // Derivative constant
double output = 0;
double angleY = 0;
double setpoint = 0;       // Target angle (upright position)
double filteredAngleY = 0; // Complementary filter result

// PID pid(&angleY, &output, &setpoint, Kp, Ki, Kd, DIRECT);
float previousError = 0;
float integral = 0;

unsigned long previousTime = 0;

float offset = 0;
void initMpu();
void motorControl(float speed, int direction);

void setup()
{
#if Debugging == 1
  Serial.begin(9600);
  Serial.println("Initializing>>>>");
#endif
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
#if Debugging == 1
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
#endif
  // offset = 180 - setpoint;
  setpoint += offset;
#if Debugging == 1
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
#endif

  // pid.SetMode(AUTOMATIC);
  // pid.SetSampleTime(20);
  // pid.SetOutputLimits(-255, 255);
}

void loop()
{
  mpu6050.update();
  // update();
  float accelAngleY = mpu6050.getAccAngleY();
  float gyroRateY = mpu6050.getGyroY();

  // Complementary filter
  filteredAngleY = 0.98 * (filteredAngleY + gyroRateY * 0.01) + 0.02 * accelAngleY;

  // angleY = mpu6050.getAngleY() + offset;
  angleY = filteredAngleY + offset;
  // pid.Compute();

  // float angleY = getAngleY();
  // Serial.print("AngleY: ");
  // Serial.println(angleY);

  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  // PID calculations

  float error = setpoint - angleY;
  integral += Ki * error * elapsedTime;
  integral = constrain(integral, -60, 60);
  float derivative = Kd * (error - previousError)  / elapsedTime;
  float output = Kp * error + integral + derivative;
  // Serial.print("integral: ");
  // Serial.println(integral);
  // Serial.print("derivative: ");
  // Serial.println(derivative);

  previousError = error;

  // Motor control
  if (abs(setpoint - angleY) < 1.5)
  { // Deadband of ±2 degrees
    motorControl(0, STOP);
    integral = 0;
  }
  else if (output > 0)
  {
    motorControl(output, FORWARD);
  }
  else
  {
    motorControl(-output, BACKWARD);
  }

#if Debugging == 1
  // Debugging
  Serial.print("AngleY: ");
  Serial.print(angleY);
  Serial.print(" | PID Output: ");
  Serial.println(output);
  Serial.print("error: ");
  Serial.println(error);
  // delay(500);
#endif
}

void motorControl(float speed, int direction)
{

  int motorSpeed = constrain(speed, 20, 255); // Limit speed to valid PWM range (0-255)
// int motorSpeed = speed;
#if Debugging == 1
  Serial.print("Speed: ");
  Serial.println(motorSpeed);
#endif
  if (direction == FORWARD)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, motorSpeed);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, motorSpeed);
  }
  else if (direction == BACKWARD)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, motorSpeed);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, motorSpeed);
  }
  else
  {
    // Stop motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
  }
}

void initMpu()
{
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
// setpoint = mpu6050.getAngleY();
#if Debugging == 1
  Serial.println("\nRobot is ready to go  (●'◡'●)");
#endif
}
