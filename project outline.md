## AQMod
**AQMod (Air Quality Module)** an ESP32-S3 based system tracking air quality metrics through 4 different sensors utlizing RTOS.

### Objectives
- Utilize an RTOS to gather data from all 4 sensors in set time intervals (start with one “Hello RTOS” LED-blink task, then add sensors incrementally).
- Analyze received data and score the air quality using hard-coded thresholds first.
- Transmit the analysis + raw value over Bluetooth (prototype with “AQMod Ready” string → JSON telemetry → custom iOS app).
- After collecting baseline data, train and deploy a TinyML anomaly-detection model via a zero-code tool (e.g. Edge Impulse).

### Scope of Work
1. **Hardware & Electrical**
- ~~Breadboard prototyping with each sensor connected.~~
- ~~Verify each sensor (BMP280, MQ-9, SEN54, MS-1100) with simple scripts.~~
- ~~Calibrate sensors.~~
- ~~Setup power module to supply voltage to esp32.~~

2. **RTOS Integration**
- ~~setup FreeRTOS task toggling an LED to confirm scheduler and toolchain.~~
- ~~Add one sensor per task, use vTaskDelay() for timing; queue all readings to a single processor task.~~
- ~~Setup Single FreeRTOS queue(fifo inter task comm.) for sensor→analysis; analysis function prints the data to serial with labels, after checking each value.~~
- ~~Enable watchdog to auto-reset on deadlock.~~

3. **Data Analysis & Scoring**
- ~~Determine Green/Yellow/Red thresholds~~
- ~~Hard-code using one sensor (e.g. MQ-9 CO) based on datasheet/EPA first.~~

4. **Bluetooth**
- ~~Follow the tutorial in notes to setup bluetooth connection.~~
- ~~fix that when I disconnect from ble on my phone I cant find the device anymore.~~

4.5 **Required Fixes before TinyML**
- Fix Sen54 sensor reporting junk data.

5. **TinyML Anomaly Detection**
- Run RTOS+BLE loop overnight → log “normal” CSV.
- Upload CSV to Edge Impulse, train an anomaly detector, export Arduino library.
- Trigger inference manually (button press) before scheduling per read.
- Inject known gas spikes to measure false positives/negatives.

