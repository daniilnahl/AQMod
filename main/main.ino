#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_BMP280.h>
#include <MQUnifiedsensor.h>
#include "esp_task_wdt.h" //watchdog :)))))))))))))
#include "queue.h" //mister queue
#include <BLEDevice.h> 
#include <BLEUtils.h>
#include <BLEServer.h>

//BLE init
#define SERVICE_UUID        "693aa0e4-7b2f-4350-bf38-e3d73f2b2a8f" //uniquely generated
#define CHARACTERISTIC_UUID "6d11c04a-78fd-4ae7-8b6e-2a9527d4380e"

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

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

//Sensor init
Adafruit_BMP280 bmp; 
SensirionI2CSen5x sen5x;
MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

//global variables
char buffer[512];
float queue_metrics[10];
volatile bool device_connected = false;
volatile bool old_device_connected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("BLE device connected!");
    device_connected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("BLE device disconnected!\nRestarting advertising...");
    device_connected = false;
  }
};

struct Range { float a, b, c; bool has; };// 3 thresholds, or 'has=false' if N/A

static constexpr Range RANGES[10] = {
    { 0,  15,   25,   true },  // pm1
    { 0,  15,   25,   true },  // pm2.5
    { 0,  45,   75,   true },  // pm4
    { 0,  45,   75,   true },  // pm10
    {30,  50,   60,   true },  // humidity
    { 0,   0,    0,  false },  // temp: special-case (person-dependent)
    { 0, 300, 1000,   true },  // voc
    {100000, 102500, 103500, true }, // pressure
    { 0, 2500, 3500,  true },  // altitude
    { 0,   10, 1000,  true },  // methane
};

static constexpr const char* METRIC[10] = {
    "Mass concentration pm 1um: ",
    "Mass concentration pm 2.5um: ",
    "Mass concentration pm 4um: ",
    "Mass concentration pm 10um: ",
    "Ambient humidity (%): ",
    "Ambient temperature (C): ",
    "VOC index: ",
    "Pressure (Pa): ",
    "Approx. altitude (m): ",
    "Methane: "
};

const esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms      = 10000,                      // 5 seconds
    .idle_core_mask  = 0,  
    .trigger_panic   = true,
};
//handlers
TaskHandle_t task_sensors_handle = NULL;
TaskHandle_t task_ble_conn_handle = NULL;
TaskHandle_t task_analysis_handle = NULL;

xQueueHandle data_queue = NULL;

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
  Serial.print("MQ 9: Calibrating please wait.");
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
        //Serial.print("Temperature Offset set to ");
        //Serial.print(tempOffset);
        //Serial.println(" deg. Celsius (SEN54/SEN55 only");
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
  BLEDevice::init("AQMod-Server");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ);

  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ); //enforces read only access
  pCharacteristic->setValue("Air quality data and analysis.");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
}

void vMainGetDataSensors(void* parameters){
  for(;;){
    MQ9.update();
    MQ9.setA(4269.6); MQ9.setB(-2.648); //methane values - CH4 | 4269.6 | -2.648    5v supply.

    uint16_t error;
    char err_msg[256];
    float mass_con_pm1, mass_con_pm2p5, mass_con_pm4, mass_con_pm10, hum, temp, voc, nox, pressure, alt, methane;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
        methane = MQ9.readSensor(); // reads PPM concentration using the model, a and b values set previously or from the setup
        
        
        float temp_data[10] = {mass_con_pm1, mass_con_pm2p5, mass_con_pm4, mass_con_pm10, hum, temp, voc, pressure, alt, methane};
        //send it to the queue
        xQueueSend(data_queue, temp_data, portMAX_DELAY);
    }
  /***
  measures how many bytes are free from the stack size allocated
  UBaseType_t free_bytes = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("%u bytes\n\n\n", free_bytes);
  ***/ 
    ESP_ERROR_CHECK(esp_task_wdt_reset());
    vTaskDelay(3000 / portTICK_PERIOD_MS); //expressed in ticks, but converted into seconds based on my esp32's clock speed
  }
}

void vMainDoAnalyis(void* parameters){

  for (;;){
    if (xQueueReceive(data_queue, queue_metrics, portMAX_DELAY) == pdTRUE){
    size_t used = 0;

    for (int i = 0; i < 10; i++){
        const char* quality = nullptr;

        if (!RANGES[i].has) { //metrics that dont have threshold aka only temp
            int n = snprintf(buffer + used, (used < sizeof(buffer)) ? sizeof(buffer) - used : 0,
                             "%s%.2f\n", METRIC[i], queue_metrics[i]);
            used += (n > 0) ? (size_t)n : 0;
            continue;
        }

        if (queue_metrics[i] >= RANGES[i].c)      quality = "Poor";
        else if (queue_metrics[i] >= RANGES[i].b) quality = "Fair";
        else if (queue_metrics[i] >= RANGES[i].a) quality = "Good";
        else                          quality = "Fair/Poor";

        int n = snprintf(buffer + used, (used < sizeof(buffer)) ? sizeof(buffer) - used : 0,
                         "%s%.2f - %s\n", METRIC[i], queue_metrics[i], quality);
        used += (n > 0) ? (size_t)n : 0;
      }
      snprintf(buffer + used, (used < sizeof(buffer)) ? sizeof(buffer) - used : 0, "\n\n");

      Serial.println(buffer);

      if (device_connected){ //connected
        pCharacteristic->setValue((uint8_t*)buffer, strnlen(buffer, sizeof(buffer)));
      }
      
      if (!device_connected && old_device_connected){ //disconected
        old_device_connected = device_connected;
      }

      if (device_connected && !old_device_connected) { //connecting
        old_device_connected = device_connected;
      }
    }

    ESP_ERROR_CHECK(esp_task_wdt_reset());
    vTaskDelay(3000 / portTICK_PERIOD_MS); 
    }
  }

void setup() { 
  Serial.begin(115200);
  Wire.begin();
  
  static bool sensors_inited = false;
  static bool wdt_inited = false;
  static bool ble_inited = false;

  if (!ble_inited){
    initBLE();
    delay(500);
    Serial.println("BLE initialized.");
  }

  if (!sensors_inited){
    initSen54();
    delay(60000);
    Serial.println("SEN 54 warm up complete.");
    initBmp280();
    delay(1000);
    Serial.println("BMP 280 warm up complete.");
    initMq9();
    delay(1000);
    Serial.println("MQ 9 warm up complete.");
    sensors_inited = true;
  }

  if (!wdt_inited){
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_cfg));
    wdt_inited = true;
  }

  data_queue = xQueueCreate(3, 10 * sizeof(float));

  //tasks - increase stack allocation if running free bytes script
  xTaskCreate(vMainGetDataSensors,  //function name
  "Get sensors data",               //task name
  2600,                             //stack size
  NULL,                             //task paramaters
  1,                                //priority
  &task_sensors_handle);            //task handle

  xTaskCreate(vMainDoAnalyis, 
  "Do data analysis",        
  2200,                          
  NULL,                          
  2,                            
  &task_analysis_handle);          

  //subscribing tasks to watchdog
  ESP_ERROR_CHECK( esp_task_wdt_add(task_sensors_handle));
  ESP_ERROR_CHECK( esp_task_wdt_add(task_analysis_handle));
}

void loop() {
}
