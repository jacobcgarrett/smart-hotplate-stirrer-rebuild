# Smart Hot Plate & Magnetic Stirrer Rebuild

A full rebuild of a broken analog laboratory hot plate/stirrer (Corning PC-420) into a high-performance, digitally controlled, network-connected device.

This project upgrades a legacy ceramic hotplate with:

- Fully digital control system  
- Phase-angle modulated heater & stirrer  
- Wireless communication (Bluetooth implemented, Wi-Fi capable)  
- OLED user interface  
- Fast, stable PID temperature control  
- Onboard data logging via SD card and USB  
- Robust safety features & AC shielding  

**Built around a dual-MCU architecture:**
- **Teensy 4.1** handles control loops, PID, PWM, encoders, switches, and SD logging  
- **ESP32-S3** manages wireless communication and the OLED UI

---

## 🔧 Hardware Features

- **TRIAC-based phase control** for heater and stirrer motors via 2× [Krida dimmer modules](https://www.tindie.com/products/bugrovs2012/pwm-8a-ac-light-dimmer-module-50hz-60hz-tasmota/)
- **Dual thermocouple sensing** (top surface + external probe via RCA jack) with cold-junction compensation using the MAX6675
- **Rotary encoder + OLED (SSD1306)** interface with stirring control and switchable temperature source (hot top or probe)
- **Bluetooth interface** for remote control (ESP32-S3; Wi-Fi logic also supported)
- **SD card logging** and **USB passthrough** (Teensy 4.1)
- Fully **isolated DC compartment**
- **Safety features:** overtemperature shutoff, power limiting, redundant fusing
- **Integrated EMI filter, fused AC inlet, and rocker switch** (Schaffner FN9290B)

---

## 💡 Why Rebuild a Hot Plate Stirrer?

Commercial hot plates are often discarded due to simple analog circuit failures, despite having high-quality ceramic tops and induction motors still in perfect condition. This project offers a blueprint to **modernize those undervalued tools** using open microcontrollers and affordable sensors — making precision lab equipment more accessible.

It also serves as an extensible platform for:
- Custom automation or reaction monitoring
- Decoupling detection
- Remote control of thermal processes

All with minimal extra labor or hardware cost.

---

## 🛠️ Technology Stack

- **Teensy 4.1**: fast real-time control, PID loops, SD logging  
- **ESP32-S3**: BLE, Wi-Fi, OLED UI  
- **MAX6675**: thermocouple amplifier  
- **Krida dimmers**: isolated, TRIAC-based AC power modules  
- **SSD1306 OLED**, **SD card**, **USB passthrough**  
- Enclosure cut by hand (CNC recommended)

---

## 🔒 Safety

- All AC mains wiring is shielded, fused, and fully isolated  
- Grounded enclosure; AC ground connected to case  
- All load-carrying wires are **14 AWG enameled copper**, soldered and crimped  
- EMI filtering and fused switch via **Schaffner FN9290B** power entry module

---

## 📜 License

MIT License.  
**Use at your own risk.** Safety measures have been implemented, but I can't guarantee compatibility with other builds or modifications.
