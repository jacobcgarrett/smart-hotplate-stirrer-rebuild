# Smart Hot Plate & Magnetic Stirrer Rebuild

A full rebuild of a broken analog laboratory hot plate/stirrer into a high-performance, digitally controlled, network-connected device.

This project upgrades a legacy ceramic hotplate with:

- Fully digital control system
- Phase-angle modulated heater & stirrer 
- Wireless communication (Bluetooth implemented, Wi-Fi capable)
- OLED display
- Fast, stable PID temperature control
- Onboard data logging via SD and USB
- Robust safety features & AC shielding

Built around a dual-MCU architecture:
- Teensy 4.1 handles fast control loops, PID, PWM, encoders, switches, data logging
- ESP32-S3 manages wireless communication, OLED display

Hardware features

- TRIAC-based phase control for both heater and stirrer motor (via 2x Krida dimmer modules https://www.tindie.com/products/bugrovs2012/pwm-8a-ac-light-dimmer-module-50hz-60hz-tasmota/ )
- Thermocouple sensing (top surface + external probe) with cold-junction compensation (MAX6675)
- Rotary encoder + OLED (SSD1306) UI with stirring control and choice between probe or hot-top temperature setpoint control
- Bluetooth interface for remote monitoring & control (ESP32S3)
- SD card logging, micro USB passthrough (Teensy 4.1)
- Isolated DC compartment
- Safety features incl. overtemp shutoff and power limits
- EMI filter, redundant fuses, and AC switch integrated in Schaffner FN9290B
