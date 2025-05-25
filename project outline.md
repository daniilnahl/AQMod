## AQMod
**AQMod (Air Quality Module)** an ESP32-S3 based system tracking air quality metrics through 4 different sensors utlizing RTOS.

### Objectives
- Utilize an RTOS to gather the necessary data from all 4 sensors in set time intervals.
- Analyze received data and score the air quality.
- Transmit the analysis with data to my iphone through bluetooth or wifi. 

### Scope of Work
- Implementing Sensors
    - BMP 280: retrieve pressure metric.
    - MQ-9: retrieve carbon monoxide metric.
    - MS-1100: retrieve carbon dioxide metric.
    - SEN54: retrieve PM, VOC, humidity and temperature metrics.
- Implementing RTOS
    - Set up threads for each sensors with this priorty: sen54, bmp280, ms-1100 and mq-9.
- Implementing Data Analysis
    - Display green, yellow or red zone based on value.
    - Research the "green, yellow, red" for each measured chemical.
- Implementing Wifi/Bluetooth connection
    - Create a connection from board to phone using serial bluetooth terminal.