# Real-time Monitoring and Control Platform for Intravenous Drip Based on ESP32

[中文版本](#基于-esp32-的医院静脉点滴实时监控与控制平台)

## Project Overview

The "Real-time Monitoring and Control Platform for Intravenous Drip Based on ESP32" is mainly applied in the medical field, aiming to address safety hazards and management challenges during clinical intravenous infusion. This system monitors the infusion rate and status in real-time, promptly detects abnormal situations, issues alarms, and automatically controls the infusion tube blockage to ensure patient safety.

## Features

- **Real-time Monitoring**: High-precision sensors monitor the infusion rate and status in real-time, promptly detecting completion, blockage, tube disconnection, and other abnormalities.
- **Instant Alarm**: When an abnormal situation or infusion completion is detected, the buzzer sounds an alarm, and the digital tube displays the bed number of the abnormal drip, facilitating quick location by the nurse.
- **Automatic Blocking**: When an abnormal situation or infusion completion is detected, the servo automatically blocks the infusion tube to prevent further dripping or backflow of blood.
- **Centralized Management**: Multiple infusion monitoring devices and nurse terminals can be connected via a local area network to achieve centralized management and real-time monitoring of multiple beds.

## Hardware Components

- ESP-c32 Super Mini Controller Board
- SD01 Drip Detection Sensor
- MG90S Servo
- HS-F14P Four-digit LED Digital Tube
- 3D Printed Casing
- TP4056 Charging Chip
- Custom Circuit Board
- Soft Pack Lithium Battery
- Buzzer

## Technical Solution

### 1. Drip Detection Sensor (SD01)
Uses the conductivity between two infrared diodes to detect drip drops. Each drop generates a pulse signal, causing a change in the signal, thus monitoring the infusion status in real-time.

### 2. Servo Control (MG90S)
Receives PWM signals to control the servo's movement, achieving precise control of the infusion tube. The servo quickly blocks the infusion tube in case of abnormalities to prevent further dripping.

### 3. Digital Tube Display (HS-F14P)
Displays the bed number corresponding to the alarm using I2C communication and the TM1650 driver, facilitating identification and handling by the nurse.

### 4. Network Communication
Uses a local area network to achieve data transmission and remote control between multiple devices. Devices communicate via static IP addresses and port numbers to ensure real-time and reliable information transmission.

## Performance Testing

- **Infusion Rate Monitoring Accuracy**: Error within ±1%
- **Alarm Sound Pressure Level**: Average sound pressure level of 85dB measured 1 meter away
- **Data Transmission Real-time Performance**: Delay less than 500ms
- **Battery Life**: Continuous operation for more than 2 hours

## Application Prospects

- **Medical Safety**: Real-time monitoring of the infusion process, timely warning of abnormalities, improving the safety of intravenous infusion, and reducing medical accidents.
- **Management Efficiency**: Simplifies the workflow of medical staff, improving management efficiency, particularly suitable for small and medium-sized hospitals.
- **Scalability**: Modular design with universal interfaces, easy to expand and promote, adapting to more scenarios and needs.

## Usage Instructions

### Hardware Connection

1. Connect the SD01 Drip Detection Sensor to the ESP32 controller board.
2. Connect the MG90S Servo, ensuring the PWM signal is correctly input.
3. Connect the HS-F14P Four-digit LED Digital Tube to the controller board, ensuring I2C communication is normal.
4. Install the 3D printed casing to secure all components.
5. Connect the TP4056 Charging Chip and the soft pack lithium battery to power the system.

### Software Setup

1. Download the code locally, open and compile it.
2. Upload the code to the ESP32 controller board.
3. Configure the local area network to ensure all devices are properly connected.
4. Start the system to begin real-time monitoring of the infusion process.

### Code Files

- `SEVER_LATEST.zip`: Code deployed on the nurse's receiver, responsible for driving the digital tube, controlling the buzzer, and connecting to the local area network.
- `CLIENT_LATEST.zip`: Code deployed on the drip detection alarm, responsible for detecting drip drops and controlling the servo.

## Contribution

Contributions to this project are welcome! If you have any suggestions or find any issues, please submit an issue or pull request.

## License

This project is licensed under the MIT License. For details, please see the LICENSE file.
