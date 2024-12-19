# Self-Balancing Robot

This project is a self-balancing robot using an Arduino Uno, MPU6050 sensor, and PID control.

## Project Structure

## Hardware Components

- Arduino Uno
- MPU6050 sensor
- Motor driver
- Motors
- Wheels

## Software Components

- Arduino framework
- MPU6050_tockn library


## Setup

1. Clone the repository.
2. Open the project in PlatformIO.
3. Connect the hardware components as per the pin configuration in `main.cpp`.

## Pin Configuration

- Motor control pins:
  - `enA` - Pin 3
  - `in1` - Pin 5
  - `in2` - Pin 4
  - `enB` - Pin 9
  - `in3` - Pin 11
  - `in4` - Pin 10

## PID Parameters

- Proportional constant (`Kp`): 70.0
- Integral constant (`Ki`): 1.5
- Derivative constant (`Kd`): 3.0

## How It Works

1. The MPU6050 sensor measures the angle of the robot.
2. The PID controller calculates the required motor speed to balance the robot.
3. The motors are controlled based on the PID output to maintain the upright position.

## Debugging

- Serial communication is used for debugging.
- Set `Debugging` flag to `1` to enable debugging messages.

## Running the Code

1. Upload the code to the Arduino Uno.
2. Open the Serial Monitor to view debugging messages.
3. Place the robot on a flat surface and observe the balancing behavior.

## License

This project is licensed under the MIT License.self balancing
