# SoloBrushlessMotor

This repository contains the `SoloBrushlessMotor` class, which interfaces with the SOLO motor controller to drive brushless DC motors. The motor is controlled using UART communication and can be configured to manage the motor’s current, torque, and speed with options for automatic calibration and real-time feedback.

## Features

- Supports two baud rate options: **115200** and **937500**.
- Configurable motor current limit and number of poles.
- Real-time current and voltage feedback from the motor.
- Automatic calibration for current loop Kp and Ki.
- Easy-to-use API for setting motor reference current, obtaining motor status, and controlling the motor with PWM.

## Class Overview

### Constructor

The `SoloBrushlessMotor` constructor initializes the motor controller with the following parameters:

```cpp
SoloBrushlessMotor(
    unsigned char _deviceAddress = 0, 
    HardwareSerial &_serial = HWSERIAL, 
    BaudRate _baudRate = BaudRate::RATE_115200, 
    long _millisecondsTimeout = 50, 
    int _packetFailureTrialAttempts = 5
);
```

- `_deviceAddress`: Address of the SOLO controller (range: 0-255).
- `_serial`: UART serial port for communication.
- `_baudRate`: Baud rate for communication (115200 or 937500).
- `_millisecondsTimeout`: Timeout in milliseconds for serial communication.
- `_packetFailureTrialAttempts`: Number of retries on packet failure.

### Key Methods

- **`init(float myCurrentLimit, long myNumberOfPoles)`**  
  Initializes the motor with the specified current limit and number of poles.

- **`setCurrent(double current)`**  
  Sets the reference current for the motor.

- **`calibration()`**  
  Runs auto-calibration for current loop Kp and Ki.

- **`getInformation()`**  
  Prints the motor's current readings and status.

- **`getQuadratureCurrentIq()`**, **`getTorqueReferenceIq()`**  
  Retrieves the quadrature current (Iq) or torque reference current (Iq).

- **`getPhaseACurrent()`**, **`getPhaseBCurrent()`**  
  Retrieves phase A or phase B current.

- **`getPhaseAVoltage()`**, **`getPhaseBVoltage()`**  
  Retrieves phase A or phase B voltage.

## Example Usage

```cpp
#include "SoloBrushlessMotor.h"

SoloBrushlessMotor motor(1, Serial1, SoloBrushlessMotor::BaudRate::RATE_115200);

void setup() {
    Serial.begin(115200);
    motor.init(6, 4);  // Initialize motor with 6A current limit and 4 poles
    motor.calibration();  // Auto-calibrate the motor
}

void loop() {
    motor.setCurrent(3.5);  // Set motor reference current to 3.5A
    Serial.println(motor.getQuadratureCurrentIq());  // Print the quadrature current
}
```

## Requirements

- **Microcontroller**: Any microcontroller supporting UART communication (e.g., Arduino, STM32).
- **Libraries**: 
  - `SOLOMotorControllersUart.h` for communication with the SOLO motor controller.

## Installation

1. Clone this repository to your local machine.
2. Include the necessary library for UART communication with SOLO motor controllers.
3. Connect your microcontroller’s hardware UART to the SOLO motor controller.
4. Upload your code to the microcontroller.

## License

This project is licensed under the MIT License.
