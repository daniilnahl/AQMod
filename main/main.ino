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
void initBmp280(){
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
void initMq9(){
  MQ9.setRegressionMethod(1); //sets math model to calculate PPM concentration

  MQ9.init(); 

  MQ9.setRL(10);

  //Calibration
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ9.update(); // Update data, the board will read the voltage from the analog pin
    calcR0 += MQ9.calibrate(RatioMQ9CleanAir);
    Serial.print(".");
  }
  MQ9.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  Serial.println("** Values from MQ-9 ****");
  Serial.println("  CH4  ");  
}
void initSen54(){
    sen5x.begin(Wire);

    uint16_t error;
    char errorMessage[256];
    error = sen5x.deviceReset();
    if (error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Temperature Offset set to ");
        Serial.print(tempOffset);
        Serial.println(" deg. Celsius (SEN54/SEN55 only");
    }

    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
}

//task functions - right now only prints values into serial monitor.
void vMainGetDataSen54(void* parameters){
  initSen54();
  vTaskDelay(1500 / portTICK_PERIOD_MS);

  for(;;){
        Serial.print("\nSEN 54\n==========================================\n");
    uint16_t error;
    char errorMessage[256];

    vTaskDelay(1000 / portTICK_PERIOD_MS); //expressed in ticks, but converted into seconds based on my esp32's clock speed

    // Read Measurement
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;

    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex);

    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("MassConcentrationPm1p0: ");
        Serial.print(massConcentrationPm1p0);
        Serial.print("\t");
        Serial.print("\nMassConcentrationPm2p5: ");
        Serial.print(massConcentrationPm2p5);
        Serial.print("\t");
        Serial.print("\nMassConcentrationPm4p0: ");
        Serial.print(massConcentrationPm4p0);
        Serial.print("\t");
        Serial.print("\nMassConcentrationPm10p0: ");
        Serial.print(massConcentrationPm10p0);
        Serial.print("\t");
        Serial.print("\nAmbientHumidity: ");
        if (isnan(ambientHumidity)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientHumidity);
        }
        Serial.print("\t");
        Serial.print("\nAmbientTemperature: ");
        if (isnan(ambientTemperature)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientTemperature);
        }
        Serial.print("\t");
        Serial.print("\nVocIndex: ");
        if (isnan(vocIndex)) {
            Serial.print("n/a");
        } else {
            Serial.print(vocIndex);
        }
        Serial.print("\t\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); //expressed in ticks, but converted into seconds based on my esp32's clock speed
    //measures how many bytes the task is using
    UBaseType_t highWater = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("High-water mark: %u bytes\n", highWater * sizeof(StackType_t));
  }

}
void vMainGetDataMq9(void* parameters){
  initMq9();
  vTaskDelay(1500 / portTICK_PERIOD_MS);

  for(;;){
  Serial.print("\nMQ-9\n==========================================");
  MQ9.update();

  MQ9.setA(4269.6); MQ9.setB(-2.648); //methane values - CH4     | 4269.6 | -2.648    5v supply.
  float CH4 = MQ9.readSensor(); // reads PPM concentration using the model, a and b values set previously or from the setup
  
  Serial.print("\nMethane: "); Serial.print(CH4); Serial.print("\n");
  vTaskDelay(500 / portTICK_PERIOD_MS); //expressed in ticks, but converted into seconds based on my esp32's clock speed
  //measures how many bytes the task is using
  UBaseType_t highWater = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("\nHigh-water mark: %u bytes\n", highWater * sizeof(StackType_t));
  }
}
void vMainGetDataBmp280(void* parameters){
  initBmp280();
  vTaskDelay(1500 / portTICK_PERIOD_MS);

  for (;;){
  Serial.print("\nBMP 280\n==========================================\n");
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1020)); //approx. bonney lake QNH -  current local sea-level pressure (in hPa) 
  Serial.println(" m");

  Serial.println();
  vTaskDelay(500 / portTICK_PERIOD_MS); //expressed in ticks, but converted into seconds based on my esp32's clock speed
  //measures how many bytes the task is using
  UBaseType_t highWater = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("High-water mark: %u bytes\n", highWater * sizeof(StackType_t));
  }
}

void setup() { 
  Serial.begin(115200);
  Wire.begin();

  //tasks
  xTaskCreate(vMainGetDataSen54,  //function name
  "Get Data from Sen54",          //task name
  4096,                           //stack size
  NULL,                           //task paramaters
  3,                              //priority
  NULL);                          //task handle

  xTaskCreate(vMainGetDataBmp280, //function name
  "Get Data from BMP 280",        //task name
  2048,                           //stack size
  NULL,                           //task paramaters
  2,                              //priority
  NULL);                          //task handle

  xTaskCreate(vMainGetDataMq9,  //function name
  "Get Data from MQ-9",         //task name
  2048,                         //stack size
  NULL,                         //task paramaters
  1,                            //priority
  NULL);                        //task handle
}

void loop() {

}
