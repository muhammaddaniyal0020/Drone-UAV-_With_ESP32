<div align="center">
  <h1>🚁 ESP32-S3 Drone UAV Flight Controller</h1>
  <p>
    <img src="https://img.shields.io/badge/platform-ESP32--S3-blue?style=for-the-badge" alt="Platform" />
    <img src="https://img.shields.io/badge/framework-Arduino%20Core-2ca5e0?style=for-the-badge" alt="Framework" />
    <img src="https://img.shields.io/badge/RTOS-FreeRTOS-f19b2c?style=for-the-badge" alt="RTOS" />
    <img src="https://img.shields.io/badge/license-MIT-green?style=for-the-badge" alt="License" />
    <img src="https://img.shields.io/badge/version-1.0.0-orange?style=for-the-badge" alt="Version" />
  </p>
</div>

## Professional Overview

A high-performance, single-file quadcopter flight controller engineered specifically for the **ESP32-S3 DevKitC-1 N16R8**. Built on FreeRTOS, it leverages dual-core processing to strictly separate the **250Hz flight stabilization loop** (Core 0) from the WebSocket telemetry and WiFi hotspot management (Core 1).

This project includes a beautifully designed, embedded **Ground Control Station (GCS)** accessible from any web browser without needing an external internet connection. It serves as an all-in-one, highly integrated foundation for custom UAV development, providing real-time telemetry, live PID tuning, and virtual joystick controls out of the box.

---

## Table of Contents

- [Professional Overview](#professional-overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Complete BOM](#complete-bom)
- [Wiring Guide](#wiring-guide)
- [Software Setup](#software-setup)
- [Dashboard Documentation](#dashboard-documentation)
- [WebSocket API](#websocket-api)
- [PID Tuning](#pid-tuning)
- [Safety Checklist](#safety-checklist)
- [Troubleshooting](#troubleshooting)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)
- [Author](#author)

---

## Features

- **WiFi Access Point (Hotspot):** ESP32 creates its own network (no router needed).
- **Web GCS Dashboard:** Accessible at `http://192.168.4.1`, features a premium dark UI with virtual joysticks, attitude indicator, compass, and telemetry gauges.
- **Dual-core FreeRTOS:** Flight loop isolated on Core 0; Web server and telemetry isolated on Core 1.
- **PID Stabilization:** Roll/Pitch in angle mode, Yaw in rate mode.
- **Complementary Filter:** Gyro + Accelerometer sensor fusion at 250 Hz.
- **BMP280 Barometer:** Real-time altitude and temperature readings.
- **Battery Monitoring:** Voltage and percentage calculation via ADC voltage divider.
- **Live PID Tuning:** Adjust all 9 gains directly from the GCS without reflashing the firmware.
- **Auto-disarm Safety:** Automatically cuts motors if the GCS WebSocket signal is lost for >2 seconds.

---

## System Architecture

![System Architecture Placeholder](./images/architecure_diagram.png)

*Above: High-level overview of the hardware and software subsystems.*

---

## Complete BOM

| Component | Specifications | Est. Price (PKR) |
|-----------|----------------|------------------|
| **ESP32-S3 DevKitC-1 N16R8** | Main flight controller | 1,870 |
| **F450 Quadcopter Frame** | 450mm wheelbase, X-frame | 2,000-2,500 |
| **4x A2212 1000KV Motor** | Brushless, standard 450 size | 3,000 (set) |
| **4x 30A BLHELI_S ESC** | PWM input, 2-4S LiPo | 2,500 (set) |
| **MPU-6050 IMU Module** | I2C gyro + accelerometer | 300-500 |
| **BMP280 Barometer Module** | I2C pressure + temperature | 200-400 |
| **3S LiPo 2200mAh** | 11.1V nominal, XT60 connector | 2,500-3,500 |
| **10x4.5 Propellers** | 2x CW + 2x CCW pairs | 500 |
| **XT60 Battery Connector** | Power distribution plug | 200 |
| **47kΩ + 10kΩ Resistors** | Battery voltage divider | 50 |
| **Power Distribution Board** | Optional, for ESC wiring | 400 |
| **Dupont wires / headers** | Sensor connections | 200 |
| **TOTAL (estimated)** | | **~13,000-17,000 PKR** |

---

## Wiring Guide

### I2C Sensors (MPU-6050 + BMP280)

Both sensors share the same I2C bus (run both on same SDA/SCL lines):

| Signal | ESP32-S3 Pin | MPU-6050 | BMP280 |
|--------|-------------|----------|--------|
| **SDA** | GPIO 8 | SDA | SDA |
| **SCL** | GPIO 9 | SCL | SCL |
| **VCC** | 3.3V | VCC | VCC |
| **GND** | GND | GND | GND |
| **AD0** | — | GND (addr=0x68) | — |
| **SDO** | — | — | GND (addr=0x76) |

> ⚠️ **WARNING:** Both sensors use 3.3V logic. Do NOT connect to 5V!

### ESC Motor Connections

```
GPIO 4 (PIN_M1)  --> Front-Right ESC (Motor 1, CW)
GPIO 5 (PIN_M2)  --> Rear-Right ESC (Motor 2, CCW)
GPIO 6 (PIN_M3)  --> Rear-Left ESC (Motor 3, CW)
GPIO 7 (PIN_M4)  --> Front-Left ESC (Motor 4, CCW)
GND              --> All ESC GND (Must share common ground)
```

### Motor Layout (Top View)

```
      FRONT
M4(FL,CCW) --- M1(FR,CW)
     \\         /
M3(RL,CW) --- M2(RR,CCW)
      REAR
```

### Battery Voltage Divider

```
Battery(+) ---[47k Ohm]--- GPIO1 (PIN_VBAT) ---[10k Ohm]--- GND
```

---

## Software Setup

### 1. Arduino IDE Board Settings

1. Add ESP32 Boards URL in Preferences: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Install **esp32** by Espressif Systems in Boards Manager.
3. Select board settings:
   - **Board:** ESP32S3 Dev Module
   - **Flash Size:** 16MB (128Mb)
   - **Partition Scheme:** Default 4MB with spiffs
   - **PSRAM:** OPI PSRAM
   - **Upload Speed:** 921600

### 2. Install Required Libraries

Install via Library Manager or Download ZIP:
- **ESPAsyncWebServer** (by me-no-dev)
- **AsyncTCP** (by me-no-dev)
- **ArduinoJson** (by Benoit Blanchon, v6 or v7)

### 3. Flash & Boot

1. Open `drone_uav/drone_uav.ino`.
2. Hold **BOOT** button on ESP32, press **RESET**, release **BOOT**.
3. Click Upload.
4. Open Serial Monitor at `115200 baud` to verify initialization and gyro calibration.
5. Connect your device to the `DroneAP-ESP32S3` WiFi network (Password: `drone1234`).

---

## Dashboard Documentation

![GCS Dashboard Screenshot](https://via.placeholder.com/1000x500/07080f/00e5d4?text=Ground+Control+Station+UI+Screenshot)

Navigate to `http://192.168.4.1` on any browser connected to the drone's WiFi network.

| Panel | Description |
|-------|-------------|
| **Attitude Indicator** | Animated artificial horizon showing real-time roll and pitch. |
| **Compass** | Heading gauge integrating yaw angle data. |
| **Altitude & Temp** | Vertical bar gauges for BMP280 barometer telemetry. |
| **Motor Outputs** | Four progress bars displaying ESC pulse widths in microseconds. |
| **Battery Monitor** | Dynamic battery level and voltage readout. |
| **Joysticks** | Touch/mouse enabled virtual sticks for Thrust/Yaw and Pitch/Roll. |
| **ARM/DISARM** | Safety interlock button. Hardware will not spin motors until armed. |
| **PID Tuning** | Expandable panel for live adjustments of Kp, Ki, and Kd. |

---

## WebSocket API

The GCS communicates with the drone over `ws://192.168.4.1/ws`. 

### Sending Commands to Drone
```json
// Arm / Disarm
{"cmd": "arm"}
{"cmd": "disarm"}

// Flight Control (thr: 0-100%, angles: -100 to 100%)
{"cmd": "ctrl", "thr": 55, "roll": 0.0, "pitch": -5.5, "yaw": 10.0}

// Update PID Gains
{"cmd": "pid", "kp_r": 1.4, "ki_r": 0.03, "kd_r": 18.0}
```

### Telemetry from Drone (10 Hz)
```json
{
  "roll": 1.2, "pitch": -0.8, "yaw": 45.0, 
  "alt": 1.5, "temp": 28.4, "vbat": 11.8, 
  "m1": 1350, "m2": 1380, "m3": 1340, "m4": 1360,
  "armed": true, "rssi": -42
}
```

---

## PID Tuning

**Default Gains (F450 Frame with 1000KV motors):**
| Axis | Kp | Ki | Kd |
|------|----|----|----|
| **Roll** | 1.40 | 0.03 | 18.0 |
| **Pitch** | 1.40 | 0.03 | 18.0 |
| **Yaw** | 3.50 | 0.02 | 0.0 |

**Tuning Process (Props OFF for safety):**
1. Set `Ki=0`, `Kd=0`.
2. Increase `Kp` until oscillations occur, then reduce by 30%.
3. Increase `Kd` to dampen oscillations (typically 15-25).
4. Add a small `Ki` (0.01-0.05) to correct steady-state drift.
5. Repeat for Pitch. Yaw generally requires higher `Kp` and zero/low `Kd`.

---

## Safety Checklist

- [ ] **PROPS OFF** during all bench testing and development.
- [ ] Check motor rotation direction matches CW/CCW layout before mounting props.
- [ ] Calibrate ESCs via standard receiver/FC pass-through before first flight.
- [ ] Ensure 3S battery is fully charged (12.6V) before flying.
- [ ] Verify Failsafe: Disconnect WiFi and ensure motors disarm automatically.
- [ ] Keep first hover attempts low (20-30cm) in an open, outdoor area.
- [ ] Land immediately if battery voltage drops below 10.5V.

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| `MPU-6050 FAIL` in Serial | I2C Wiring | Verify SDA=GPIO8, SCL=GPIO9, 3.3V Power. |
| `BMP280 FAIL` in Serial | I2C Address | Pull SDO pin to GND to set address `0x76`. |
| GCS does not load | WiFi connection | Ensure device is connected to `DroneAP-ESP32S3`. |
| Motors do not spin | Interlock | Must click ARM in GCS and raise throttle joystick. |
| Drone oscillates rapidly | PID `Kp` too high | Reduce Roll/Pitch `Kp` by 20% in GCS. |
| Battery reads 0V | ADC Wiring | Check the 47kΩ/10kΩ divider at GPIO 1. |

---

## Roadmap

- [ ] Implement BLE Gamepad (PS4/Xbox) support for physical control.
- [ ] Add GPS Waypoint navigation and Return-to-Home (RTH).
- [ ] Barometer-assisted Altitude Hold mode.
- [ ] SD Card flight data logging.
- [ ] Integrate HMC5883L Magnetometer for absolute heading.

---

## Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

---

## License

Distributed under the MIT License. See `LICENSE` for more information.

---

## Author

Built with ❤️ by **Muhammad**

*If you found this project helpful, don't forget to star the repository!*
