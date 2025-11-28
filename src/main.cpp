#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <avr/wdt.h>

// Motor control pins
#define enA 3
#define in1 4
#define in2 5

#define enB 9
#define in3 11
#define in4 10

// Motor control directions
#define FORWARD 1
#define BACKWARD -1
#define STOP 0

#define LED 12

// Debugging flag
#define Debugging 0

// MPU6050 sensor object
MPU6050 mpu6050(Wire);

// PID parameters
double Kp = 40.0; // Proportional constant
double Ki = 0.25;  // Integral constant
double Kd = 3.5;  // Derivative constant
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
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
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

  for (uint8_t i = 0; i < 5; i++)
  {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }


  // Initialize the MPU6050 sensor
  initMpu();

  // Set the target angle to the current Y angle
  setpoint = mpu6050.getAngleY();
  filteredAngleY = setpoint;

  // Print the setpoint if debugging is enabled
#if Debugging == 1
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
#endif
  digitalWrite(LED, HIGH);

  // Enable watchdog timer with 1 second timeout to prevent  MPU6050 stucking issue
  wdt_enable(WDTO_1S);

}

void loop()
{
  // Reset watchdog timer
  wdt_reset();

#if Debugging == 0
  delay(10); // Loop delay for stability
#endif
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
  output = constrain(output, -255, 255); // Constrain output to motor PWM range

  previousError = error;

  // Motor control based on PID output
  if (abs(setpoint - angleY) < 2 || error >= 45 || error <= -45)
  { // Deadband of ±2 degrees and safety cutoff at ±45 degrees
    motorControl(0, STOP);
    integral = 0; // Reset integral term when within deadband
    output = 0;
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
  // output in volt
  output = output * (5.0 / 255.0);
  if (output < 0) output = -output;
  Serial.print("   AngleY:");
  Serial.print(angleY);
  Serial.print("   Setpoint:");
  Serial.print(setpoint);
  Serial.print("   Output(V):");
  Serial.println(output);
#endif
}

void motorControl(float speed, int direction)
{
  // Constrain motor speed to valid PWM range (0-255)
  int motorSpeed = constrain(speed, 20, 255);


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