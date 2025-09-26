![Alt text](phyphox_inclination_sensor/qr.png)

# phyphox_inclination_sensor

# ESP32 Phyphox Kalman Buzzer

This project demonstrates how to:
- Receive IMU sensor data (Accel/Gyro/Mag) via BLE from the Phyphox app
- Process data with a **Kalman Filter** for pitch/roll estimation
- Trigger a **buzzer alert** if roll angle deviates more than ±12° from vertical
- Auto-fallback to zero values if experiment is stopped

## Features
- BLE GATT server for data transfer from Phyphox
- Custom Kalman filter implementation (no external libraries)
- Non-blocking buzzer control using `millis()`
- Timeout handling if no data packets are received

## Hardware
- ESP32 DevKit
- Active buzzer connected to GPIO 32

## Usage
1. Open project in Arduino IDE (or PlatformIO).
2. Flash to ESP32.
3. Connect to `ESP32_Phyphox_Output` from Phyphox app.
4. Run experiment → pitch/roll calculated and buzzer toggles when condition met.

## Example Output
Roll: 95.3 deg, Pitch: -2.4 deg

Buzzer ON (roll deviation > 12°)

## License
MIT
