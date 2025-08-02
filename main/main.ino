#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_BMP280.h>
#include <MQUnifiedsensor.h>
#include "esp_task_wdt.h" //watchdog :)))))))))))))

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

//global variables
char buffer[512];

const esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms      = 5000,                      // 5 seconds
    .idle_core_mask  = 0,  
    .trigger_panic   = true,
};

//task handlers
TaskHandle_t task_i2c_sensors_handle = NULL;
TaskHandle_t task_mq9_handle = NULL;
TaskHandle_t task_ble_conn_handle = NULL;
TaskHandle_t task_analysis_handle = NULL;

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
void initBLE(){
}

//task functions - right now only prints values into serial monitor.
void vMainGetDataI2cSensors(void* parameters){
  initSen54();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  initBmp280();
  vTaskDelay(1000 / portTICK_PERIOD_MS);  

  for(;;){
    uint16_t error;
    char err_msg[256];

    //data vars
    float mass_con_pm1, mass_con_pm2p5, mass_con_pm4, mass_con_pm10, hum, temp, voc, nox, pressure, alt;

    //gets values and auto fills error msg if any errors come up
    error = sen5x.readMeasuredValues(mass_con_pm1, mass_con_pm2p5, mass_con_pm4, mass_con_pm10, hum, temp, voc, nox);

    if (error) {
        errorToString(error, err_msg, sizeof(err_msg));
        snprintf(buffer, sizeof(buffer), "SEN 54\nError trying to execute readMeasuredValues(): %s", err_msg);
    } else {
        //if humidity is n/a
        if (isnan(hum)) hum = -1.0;
        //if temp is n/a
        if (isnan(temp)) temp = -1.0;
        //if vocIndex is n/a     
        if (isnan(voc)) voc = -1.0;

        pressure = bmp.readPressure();
        alt = bmp.readAltitude(1020); //approx. bonney lake QNH -  current local sea-level pressure (in hPa) 

        snprintf(buffer, sizeof(buffer), "SEN 54\nmass concentration pm 1um: %0.01f \nmass concentration pm 2.5um: %0.01f \nmass concentration pm 4um: %0.01f \nmass concentration pm 10um: %0.01f \nambient humidity: %0.01f \nambient temperature: %0.01f \nvoc index: %0.01f \n\nBMP 280\nPressure = %0.01f Pa\nApprox altitude = %0.01f m\n\n",
        mass_con_pm1, mass_con_pm2p5, mass_con_pm4, mass_con_pm10, hum, temp, voc, pressure, alt);
    }

    //print final message here
    Serial.print(buffer);
    
    ESP_ERROR_CHECK(esp_task_wdt_reset());
    vTaskDelay(3000 / portTICK_PERIOD_MS); //expressed in ticks, but converted into seconds based on my esp32's clock speed
      
  /***measures how many bytes are free from the stack size allocated
    UBaseType_t free_bytes = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("%u bytes\n\n\n", free_bytes);
    **/
    
  }
}
void vMainGetDataMq9(void* parameters){
  initMq9();
  vTaskDelay(1500 / portTICK_PERIOD_MS);

  for(;;){
  MQ9.update();
  MQ9.setA(4269.6); MQ9.setB(-2.648); //methane values - CH4     | 4269.6 | -2.648    5v supply.
  float methane = MQ9.readSensor(); // reads PPM concentration using the model, a and b values set previously or from the setup
  
  snprintf(buffer, sizeof(buffer), "MQ-9\nMethane: %0.01f\n\n", methane);
  Serial.print(buffer);

  ESP_ERROR_CHECK(esp_task_wdt_reset());

  vTaskDelay(3000 / portTICK_PERIOD_MS); //expressed in ticks, but converted into seconds based on my esp32's clock speed
  
  /***measures how many bytes are free from the stack size allocated 
  UBaseType_t free_bytes = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("\nMq-9: %u bytes\n\n\n", free_bytes);
  **/
  }
}
void vMainDoAnalyis(void* parameters){
}
void vMainConnBLE(void* parameters){
}

void setup() { 
  Serial.begin(115200);
  Wire.begin();
  
  //init watchdog
  static bool wdt_inited = false;

  if (!wdt_inited){
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_cfg));
    wdt_inited = true;
  }
  

  //tasks - increase stack allocation if running free bytes script
  xTaskCreate(vMainGetDataI2cSensors,  //function name
  "Get Data from Sen54",          //task name
  2500,                           //stack size
  NULL,                           //task paramaters
  3,                              //priority
  &task_i2c_sensors_handle);             //task handle

  xTaskCreate(vMainGetDataMq9,  //function name
  "Get Data from MQ-9",         //task name
  2048,                         //stack size
  NULL,                         //task paramaters
  1,                            //priority
  &task_mq9_handle);            //task handle

  /**
  xTaskCreate(vMainDoAnalyis, 
  "Do data analysis",        
  2400,                          
  NULL,                          
  4,                            
  &task_analysis_handle);          

  xTaskCreate(vMainConnBLE,  
  "Connect BLE",         
  2048,                            
  NULL,                           
  5,                                 
  &task_ble_conn_handle);            
  **/

  //subscribing tasks to watchdog
  ESP_ERROR_CHECK( esp_task_wdt_add(task_i2c_sensors_handle));
  ESP_ERROR_CHECK( esp_task_wdt_add(task_mq9_handle));

  /***
  ESP_ERROR_CHECK( esp_task_wdt_add(task_ble_conn_handle));
  ESP_ERROR_CHECK( esp_task_wdt_add(task_analysis_handle));
  **/
}

void loop() {
}
