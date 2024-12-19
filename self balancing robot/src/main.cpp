#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// Motor control pins
#define enA 3
#define in1 5
#define in2 4

#define enB 9
#define in3 11
#define in4 10

// Motor control directions
#define FORWARD 1
#define BACKWARD -1
#define STOP 0

// Debugging flag
#define Debugging 1

// MPU6050 sensor object
MPU6050 mpu6050(Wire);

// PID parameters
double Kp = 70.0; // Proportional constant
double Ki = 1.5;  // Integral constant
double Kd = 3.0;  // Derivative constant
double output = 0;
double angleY = 0;
double setpoint = 0;       // Target angle (upright position)
double filteredAngleY = 0; // Complementary filter result

float previousError = 0;
float integral = 0;

unsigned long previousTime = 0;

// Function declarations
void initMpu();
void motorControl(float speed, int direction);

void setup()
{
  // Initialize serial communication if debugging is enabled
  #if Debugging == 1
    Serial.begin(9600);
    Serial.println("Initializing>>>>");
  #endif

  // Initialize I2C communication
  Wire.begin();

  // Set motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set initial motor speeds
  analogWrite(enA, 100);
  analogWrite(enB, 100);

  // Initialize the MPU6050 sensor
  initMpu();
  
  // Set the target angle to the current Y angle
  setpoint = mpu6050.getAngleY();

  // Print the setpoint if debugging is enabled
  #if Debugging == 1
    Serial.print("Setpoint: ");
    Serial.println(setpoint);
  #endif
}

void loop()
{
  // Update MPU6050 sensor data
  mpu6050.update();
  float accelAngleY = mpu6050.getAccAngleY();
  float gyroRateY = mpu6050.getGyroY();

  // Complementary filter to calculate the filtered angle
  filteredAngleY = 0.98 * (filteredAngleY + gyroRateY * 0.01) + 0.02 * accelAngleY;

  // Use the filtered angle for PID calculations
  angleY = filteredAngleY;

  // Calculate elapsed time
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  // PID calculations
  float error = setpoint - angleY;
  integral += Ki * error * elapsedTime;
  integral = constrain(integral, -60, 60); // Constrain integral to prevent windup
  float derivative = Kd * (error - previousError) / elapsedTime;
  float output = Kp * error + integral + derivative;

  previousError = error;

  // Motor control based on PID output
  if (abs(setpoint - angleY) < 1.5)
  { // Deadband of ±1.5 degrees
    motorControl(0, STOP);
    integral = 0; // Reset integral term when within deadband
  }
  else if (output > 0)
  {
    motorControl(output, FORWARD);
  }
  else
  {
    motorControl(-output, BACKWARD);
  }

  // Print debugging information if debugging is enabled
  #if Debugging == 1
    Serial.print("AngleY: ");
    Serial.print(angleY);
    Serial.print(" | PID Output: ");
    Serial.println(output);
    Serial.print("error: ");
    Serial.println(error);
  #endif
}

void motorControl(float speed, int direction)
{
  // Constrain motor speed to valid PWM range (0-255)
  int motorSpeed = constrain(speed, 20, 255);

  // Print motor speed if debugging is enabled
  #if Debugging == 1
    Serial.print("Speed: ");
    Serial.println(motorSpeed);
  #endif

  // Control motors based on direction
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
  // Initialize MPU6050 sensor
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); // Calculate gyro offsets

  // Print a message if debugging is enabled
  #if Debugging == 1
    Serial.println("\nRobot is ready to go  (●'◡'●)");
  #endif
}