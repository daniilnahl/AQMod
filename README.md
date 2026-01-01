&#x20;

## Table of Contents

- [Introduction](#aqmod-aka-air-quality-module)
- [Prerequisites](#prerequisites)
- [Hardware Setup](#hardware-setup)
- [Software Installation](#software-installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

# AQMod aka Air Quality Module

**This project is a system built on top of ESP32-S3 integrating BMP 280, MQ-9 and SEN54 SDN-T sensors to track air quality.**

### What does it track?

- Temperature
- Humidity
- Pressure
- Particulate Matter (<1µm)
- Particulate Matter (<2.5µm)
- Particulate Matter (<4µm)
- Particulate Matter (<10µm)
- Voc Index
- Methane
- Altitude

### How does it track it?

- MQ-9 is responsible for tracking methane.
- BMP 280 is responsible for tracking temperature, pressure and altitude.
- SEN 54 is responsible for tracking particulate matters, humidity, temperature and voc index.

### Demonstration
[![Demonstration video 1](https://media0.giphy.com/media/v1.Y2lkPTc5MGI3NjExeG52dTAweGhvN3hzNnFsNTk2dHJxdTlwNnhpNXB1OXNjdHJkc2RsdSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/WigBvEbSp0v5ljpwq8/giphy.gif)](https://youtu.be/7I6uvrYwucg)
[![Demonstration video 2](https://media0.giphy.com/media/v1.Y2lkPTc5MGI3NjExeG52dTAweGhvN3hzNnFsNTk2dHJxdTlwNnhpNXB1OXNjdHJkc2RsdSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/WigBvEbSp0v5ljpwq8/giphy.gif)](https://youtu.be/7I6uvrYwucg)

---
## Project Milestones

1. ~~First working prototype~~ COMPLETED on 07/17/2025
2. ~~RTOS Integration~~ COMPLETED on 08/10/2025
3. ~~Bluetooth Connectivity~~ COMPLETED on 08/15/2025
4. ~~TinyML Real Time Classification~~ COMPLETED in Nov. of 2025

## Prerequisites

**Hardware:**

- ESP32-S3 dev board
- BMP280 breakout board
- MQ-9 breakout board
- SEN54 SDN-T
- Breadboard & jumper wires
- Elegoo Power Spply module

**Software:**

- Arduino IDE (with ESP32-S3 support)
- Required libraries:
  - `Adafruit BMP280 Library`
  - `MQUnifiedsensor`
  - `Sensirion I2C SEN5X`
  - `AQMod_inferencing`

> **All these libraries can be downloaded through Arduino IDE library manager (besides AQMod_inferencing. You can add it following these instructions: Arduino IDE -> Sketch -> Include Library -> Add .ZIP Library...**

---

## Hardware Setup
![Schematic](misc/images/AQMod_Schematic.png)

---

## Software Installation

1. Clone the repo:
   ```bash
   git clone https://github.com/yourusername/AQMod.git
   cd AQMod
   ```
2. Install dependencies:
   - In Arduino IDE: Sketch → Include Library → Manage Libraries…
   - Search for and install the listed libraries above.
3. Open and upload the sketch:
   - Via Arduino IDE: open `main/main.ino` and select the ESP32-S3 board.
---

## Usage

1. Upload firmware to ESP32-S3.
2. Open serial monitor at `115200 baud`.
3. Observe output.
---

## Project Structure
```
AQMod
├── Edge Impulse Model
│   ├── model example code
│   │   └── static_buffer
│   │       └── static_buffer.ino
│   └── ei-aqmod-arduino-1.0.2.zip (library and the model)
├── main
│   └── main.ino
├── misc
│   ├── AQMod schematic (KiCad)
│   ├── data collection
│   │   ├── data collected (csv files folder)
│   │   ├── data_collection.ino
│   │   ├── data_collection_with_TinyML.ino
│   │   ├── sw_listener (env folder of below file)
│   │   └── serial listener.py
│   ├── images
│   └── notes
├── LICENSE
└── README.md
```
---

## Contributing

Please open issues or pull requests! To contribute:

1. Fork the repo.
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Commit changes and push: `git push origin feature/your-feature`
4. Open a PR against `develop` branch.

---

## License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

---

*Developed by Daniil Nahliuk(@daniilnahl)*










