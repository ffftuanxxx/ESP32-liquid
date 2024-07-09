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

- `detector_client`: Code deployed on the nurse's receiver, responsible for driving the digital tube, controlling the buzzer, and connecting to the local area network.
- `detector_client`: Code deployed on the drip detection alarm, responsible for detecting drip drops and controlling the servo.
- `Shell_model_file`: Contain all the 3D files need to be printed in this project.

### Notification
- **build** is needed in both `detector_client` and `detector_client`.


## How to use example

### Configure the project

Open the project configuration menu (`idf.py menuconfig`).

In the `Example Configuration` menu:

* Set the Wi-Fi configuration.
    * Set `WiFi SSID`.
    * Set `WiFi Password`.

Optional: If you need, change the other options according to your requirements.

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for all the steps to configure and use the ESP-IDF to build projects.

* [ESP-IDF Getting Started Guide on ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-S2](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)

## Example Output
Note that the output, in particular the order of the output, may vary depending on the environment.

Console output if station connects to AP successfully:
```
I (589) wifi station: ESP_WIFI_MODE_STA
I (599) wifi: wifi driver task: 3ffc08b4, prio:23, stack:3584, core=0
I (599) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (599) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (629) wifi: wifi firmware version: 2d94f02
I (629) wifi: config NVS flash: enabled
I (629) wifi: config nano formating: disabled
I (629) wifi: Init dynamic tx buffer num: 32
I (629) wifi: Init data frame dynamic rx buffer num: 32
I (639) wifi: Init management frame dynamic rx buffer num: 32
I (639) wifi: Init management short buffer num: 32
I (649) wifi: Init static rx buffer size: 1600
I (649) wifi: Init static rx buffer num: 10
I (659) wifi: Init dynamic rx buffer num: 32
I (759) phy: phy_version: 4180, cb3948e, Sep 12 2019, 16:39:13, 0, 0
I (769) wifi: mode : sta (30:ae:a4:d9:bc:c4)
I (769) wifi station: wifi_init_sta finished.
I (889) wifi: new:<6,0>, old:<1,0>, ap:<255,255>, sta:<6,0>, prof:1
I (889) wifi: state: init -> auth (b0)
I (899) wifi: state: auth -> assoc (0)
I (909) wifi: state: assoc -> run (10)
I (939) wifi: connected with #!/bin/test, aid = 1, channel 6, BW20, bssid = ac:9e:17:7e:31:40
I (939) wifi: security type: 3, phy: bgn, rssi: -68
I (949) wifi: pm start, type: 1

I (1029) wifi: AP's beacon interval = 102400 us, DTIM period = 3
I (2089) esp_netif_handlers: sta ip: 192.168.77.89, mask: 255.255.255.0, gw: 192.168.77.1
I (2089) wifi station: got ip:192.168.77.89
I (2089) wifi station: connected to ap SSID:myssid password:mypassword
```

Console output if the station failed to connect to AP:
```
I (589) wifi station: ESP_WIFI_MODE_STA
I (599) wifi: wifi driver task: 3ffc08b4, prio:23, stack:3584, core=0
I (599) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (599) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (629) wifi: wifi firmware version: 2d94f02
I (629) wifi: config NVS flash: enabled
I (629) wifi: config nano formating: disabled
I (629) wifi: Init dynamic tx buffer num: 32
I (629) wifi: Init data frame dynamic rx buffer num: 32
I (639) wifi: Init management frame dynamic rx buffer num: 32
I (639) wifi: Init management short buffer num: 32
I (649) wifi: Init static rx buffer size: 1600
I (649) wifi: Init static rx buffer num: 10
I (659) wifi: Init dynamic rx buffer num: 32
I (759) phy: phy_version: 4180, cb3948e, Sep 12 2019, 16:39:13, 0, 0
I (759) wifi: mode : sta (30:ae:a4:d9:bc:c4)
I (769) wifi station: wifi_init_sta finished.
I (889) wifi: new:<6,0>, old:<1,0>, ap:<255,255>, sta:<6,0>, prof:1
I (889) wifi: state: init -> auth (b0)
I (1889) wifi: state: auth -> init (200)
I (1889) wifi: new:<6,0>, old:<6,0>, ap:<255,255>, sta:<6,0>, prof:1
I (1889) wifi station: retry to connect to the AP
I (1899) wifi station: connect to the AP fail
I (3949) wifi station: retry to connect to the AP
I (3949) wifi station: connect to the AP fail
I (4069) wifi: new:<6,0>, old:<6,0>, ap:<255,255>, sta:<6,0>, prof:1
I (4069) wifi: state: init -> auth (b0)
I (5069) wifi: state: auth -> init (200)
I (5069) wifi: new:<6,0>, old:<6,0>, ap:<255,255>, sta:<6,0>, prof:1
I (5069) wifi station: retry to connect to the AP
I (5069) wifi station: connect to the AP fail
I (7129) wifi station: retry to connect to the AP
I (7129) wifi station: connect to the AP fail
I (7249) wifi: new:<6,0>, old:<6,0>, ap:<255,255>, sta:<6,0>, prof:1
I (7249) wifi: state: init -> auth (b0)
I (8249) wifi: state: auth -> init (200)
I (8249) wifi: new:<6,0>, old:<6,0>, ap:<255,255>, sta:<6,0>, prof:1
I (8249) wifi station: retry to connect to the AP
I (8249) wifi station: connect to the AP fail
I (10299) wifi station: connect to the AP fail
I (10299) wifi station: Failed to connect to SSID:myssid, password:mypassword
```

## Contribution

Contributions to this project are welcome! If you have any suggestions or find any issues, please submit an issue or pull request.

## License

This project is licensed under the MIT License. For details, please see the LICENSE file.

---

# 基于 ESP32 的医院静脉点滴实时监控与控制平台

[English Version](#real-time-monitoring-and-control-platform-for-intravenous-drip-based-on-esp32)

## 项目概述

"基于 ESP32 的医院静脉点滴实时监控与控制平台" 主要应用于医疗领域，致力于解决临床静脉输液过程中存在的安全隐患和管理难题。该系统通过实时监控输液速率和状态，及时发现异常情况并发出警报，同时自动控制输液管阻断，以确保患者安全。

## 功能特点

- **实时监控**：高精度传感器实时监测输液速率和状态，及时发现输液完成、堵塞、管脱落等异常情况。
- **即时警报**：异常情况或输液完成时，蜂鸣器发出声音警报，数码管显示异常点滴的床位号，方便护士快速定位。
- **自动阻断**：异常情况或输液完成时，舵机自动阻断输液管，防止药液继续滴漏或血液倒流。
- **集中管理**：多个输液监控设备和护士终端可以通过局域网连接，实现对多个病床的集中管理和实时监控。

## 硬件组成

- ESP-c32 Super Mini 主控板
- SD01 液滴检测传感器
- MG90S 舵机
- HS-F14P 四位 LED 数码管
- 3D 打印外壳
- TP4056 充电芯片
- 定制电路板
- 软包锂电池
- 蜂鸣器

## 技术方案

### 1. 液滴检测传感器 (SD01)
利用红外二极管的导通性检测液滴滴落情况，每一滴液体通过时产生脉冲信号，引起信号变化，实时监测输液状态。

### 2. 舵机控制 (MG90S)
接收 PWM 信号控制舵机运动，实现对输液管的精确控制。舵机在异常情况下快速阻断输液管，防止药液滴漏。

### 3. 数码管显示 (HS-F14P)
通过 I2C 通信和 TM1650 驱动数码管，显示警报所对应的床位号，方便护士识别和处理。

### 4. 网络通信
利用局域网实现多设备间的数据传输和远程控制，设备通过静态 IP 地址和端口号进行通信，确保信息传输的实时性和可靠性。

## 性能测试

- **输液速率监测精度**：误差在 ±1% 以内
- **警报声压级**：1 米处测量平均声压级为 85dB
- **数据传输实时性**：延迟小于 500ms
- **电池续航能力**：连续工作 2 小时以上

## 应用前景

- **医疗安全**：实时监控输液过程，及时预警异常情况，提高静脉输液安全性，减少医疗事故。
- **管理效率**：简化医护人员的工作流程，提高管理效率，特别适用于中小型医院。
- **可扩展性**：模块化设计，接口通用，易于扩展和推广，适应更多场景和需求。

## 使用说明

### 硬件连接

1. 将 SD01 液滴检测传感器与 ESP32 主控板连接。
2. 连接 MG90S 舵机，确保 PWM 信号正确输入。
3. 将 HS-F14P 四位 LED 数码管与主控板连接，确保 I2C 通信正常。
4. 安装 3D 打印外壳，固定所有组件。
5. 连接 TP4056 充电芯片和软包锂电池，为系统供电。

### 软件设置

1. 将代码下载到本地，打开并编译。
2. 将代码上传到 ESP32 主控板。
3. 配置局域网，确保所有设备正常连接。
4. 启动系统，开始实时监控输液过程。

### 代码文件

- `SEVER_LATEST.zip`: 部署在护士端的接收器代码，负责数码管驱动、蜂鸣器控制和局域网连接。
- `CLIENT_LATEST.zip`: 部署在液滴检测警报器上的代码，负责液滴检测和舵机控制。

## 贡献

欢迎对本项目进行贡献！如果有任何建议或发现问题，请提交 issue 或 pull request。

## 许可证

本项目使用 MIT 许可证，详情请参见 LICENSE 文件。
