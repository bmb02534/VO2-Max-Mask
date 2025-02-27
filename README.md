# VO2-Max-Mask

This repository contains the firmware and design documentation for a temperature monitoring system that combines analog signal conditioning, and noise filtering. The system uses a negative temperature coefficient (NTC) thermistor linearized with a parallel resistor, amplified by an INA121 instrumentation amplifier, and digitized by an ESP32 microcontroller. A Kalman filter smooths measurements, and results are transmitted via Wi-Fi for remote monitoring.
