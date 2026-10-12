# rp2040-imu-monitor

A magnetometer/accelerometer for remote monitoring of an antenna rotator, built using an RP2040-Zero and LSM303 chip

## Parts

- RP2040-Zero
- LSM303DLH IMU breakout board (any LSM303 variant works: DLH, DLM, DLHC, D, AGR)

## Connections

- I2C-SDA = GP26
- I2C-SCL = GP27
- VCC/VIN = 3V3
- GND = GND

## Guide

1. Solder the PCBs together directly, using bodge wire as necessary.
2. Flash firmware (see [firmware/README.md](firmware/README.md)).
3. Open serial terminal to view output (JSONL, ~4 lines/second).
