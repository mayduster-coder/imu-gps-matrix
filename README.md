# IMU / GPS / LED-Matrix Telemetry System

A real-time embedded application on an **STM32F756ZG (ARM Cortex-M7)** that reads an **IMU** and a **GPS** receiver, visualizes device orientation on an **8×8 LED matrix**, and streams telemetry over **Ethernet**. Built on **FreeRTOS** with a clean, modular task architecture.

---

## Overview

The system fuses inertial and positioning data in real time:

- Reads acceleration and angular rate from an **MPU6050 IMU** over **I²C**.
- Computes tilt (pitch / roll) using a **complementary filter** that fuses the accelerometer and gyroscope.
- Maps the tilt direction to one of 8 compass directions and displays an arrow (or a smiley when level) on a **MAX7219 LED matrix** over **SPI**.
- Parses **NMEA (GGA)** sentences from a **u-blox GPS** over **UART with DMA**.
- Sends telemetry (orientation + position) over **Ethernet (UDP)** using the **lwIP** stack.

## Hardware

| Component | Interface | Pins / Peripheral |
|---|---|---|
| STM32F756ZG (Nucleo-144) | — | ARM Cortex-M7 @ up to 216 MHz |
| MPU6050 IMU | I²C1 | — |
| MAX7219 LED matrix | SPI2 | — |
| u-blox GPS | USART2 (DMA) | NMEA GGA |
| Ethernet (telemetry) | lwIP | UDP |

## Software Architecture

The application runs under **FreeRTOS (CMSIS-RTOS v1)**, organized into independent modules — not a monolithic `main`:

- **Sensor task** — samples the IMU at a fixed rate using `vTaskDelayUntil` for an exact period.
- **Orientation** — complementary filter (`angle = α·(angle + gyro·dt) + (1-α)·accel_angle`) for drift-free, smooth tilt.
- **Display task** — drives the LED matrix over SPI based on the computed direction.
- **GPS task** — DMA-based reception of NMEA sentences, parsed for latitude/longitude.
- **Telemetry** — lwIP UDP transmission of the fused data.

Inter-task communication uses FreeRTOS **queues** and a shared, **mutex-protected** logger.

## Key Concepts Demonstrated

- Sensor fusion (complementary filter) on a resource-constrained MCU
- Multiple concurrent peripherals: I²C, SPI, UART/DMA, Ethernet
- FreeRTOS task design, priorities, queues, mutexes, and periodic scheduling
- Interrupt- and DMA-driven I/O for efficient, non-blocking data acquisition
- TCP/IP networking on embedded (lwIP)

## Toolchain

- **STM32CubeIDE** / CubeMX (HAL, FreeRTOS, lwIP)
- **GCC** for ARM (arm-none-eabi)
- Language: **C**

## Author

**May Duster** — Embedded Software Engineer
[LinkedIn](https://www.linkedin.com/in/may-duster-201914241) · [GitHub](https://github.com/mayduster-coder)
