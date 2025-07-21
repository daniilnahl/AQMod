#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_BMP280.h>
#include <MQUnifiedsensor.h>

//MQ-9 parameters
#define         Board                   ("Arduino UNO")
#define         Pin                     (6) 
#define         Type                    ("MQ-9") 
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (12) 
#define         RatioMQ9CleanAir        (9.6)

//Sen 54 buffer reqs
//Explanation: The used commands use up to 48 bytes. On some Arduino's the default buffer space is not large enough.
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

//Sensor initialization
Adafruit_BMP280 bmp; 
SensirionI2CSen5x sen5x;
MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

//setup functions


//task functions
void vMainGetDataSen54()
void vMainGetDataMq9()
void vMainGetDataBmp280()

void setup() {
  Serial.begin(115200);
  Wire.begin();

  //sensor setup
  setupBMP280();
  setupMQ9();
  setupSen54();

  //task setup
  xTaskCreate(TaskFunction_t task1, //function name
  const char *const "Task 1",       //task name
  const uint32_t 1000,              //stack size
  void *const NULL,                 //task paramaters
  UBaseType_t 1,                    //priority
  TaskHandle_t *const NULL)         //task handle
}

void loop() {
}
