#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_BMP280.h>
#include <MQUnifiedsensor.h>
#include <AQMod_inferencing.h>

//MQ-9 parameters
#define         Board                   ("Arduino UNO")
#define         Pin                     (6) 
#define         Type                    ("MQ-9") 
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (12) 
#define         RatioMQ9CleanAir        (9.6)

//Sen 54 buffer reqs
//Explanation: The used commands use up to 48 bytes. On some Arduino's the default buffer space is not large enough.
// #define MAXBUF_REQUIREMENT 48

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


//edge impulse stuff
float features[6];

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void print_inference_result(ei_impulse_result_t result);
void inference();


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

  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void initMq9(){
  MQ9.setRegressionMethod(1); //sets math model to calculate PPM concentration
  MQ9.setA(4269.6); MQ9.setB(-2.648); //methane values - CH4 | 4269.6 | -2.648    5v supply.
  MQ9.init(); 
  MQ9.setRL(10);

  //Calibration
  Serial.print("MQ 9: Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i++)
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
    } 

    error = sen5x.startMeasurement();// Start Measurement
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    error = sen5x.setFanAutoCleaningInterval(43200);// fan cleaning interval
    if (error) {
        Serial.print("Error trying to execute setFanAutoCleaningInterval(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    error = sen5x.startFanCleaning();// fan cleaning 
    if (error) {
        Serial.print("Error trying to execute startFanCleaning(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
}

void inference(){
    ei_printf("Edge Impulse standalone inferencing (Arduino)\n");

    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        delay(1000);
        return;
    }

    ei_impulse_result_t result = { 0 };

    // the features are stored into flash, and we don't want to load everything into RAM
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;

    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

    // print inference return code
    ei_printf("run_classifier returned: %d\r\n", res);
    print_inference_result(result);

}

void print_inference_result(ei_impulse_result_t result) {

    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    // Print the prediction results (object detection)
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif

}


void setup() { 
  Serial.begin(115200);
  Wire.begin();
  
  static bool sensors_inited = false;

  if (!sensors_inited){
    initSen54();
    delay(63000);
    Serial.println("SEN 54 warm up complete.");
    initBmp280();
    delay(1000);
    Serial.println("BMP 280 warm up complete.");
    initMq9();
    delay(1000);
    Serial.println("MQ 9 warm up complete.");
    sensors_inited = true;
  }
}

void loop() {      
    MQ9.update();

    uint16_t error;
    char err_msg[256];
    float mass_con_pm1, mass_con_pm2p5, mass_con_pm4, mass_con_pm10, hum, temp, voc, nox, pressure, alt, methane;

    delay(1000);
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
        //mass concentration pm 1um -> mass concentration pm 2.5um -> mass concentration pm 4um -> mass concentration pm 10um  -> ambient humidity -> ambient temperature -> voc index -> Methane
        snprintf(buffer, sizeof(buffer), "%0.01f,%0.01f,%0.01f,%0.01f,%0.01f,%0.01f,%0.01f,%0.01f\n",
        mass_con_pm1, mass_con_pm2p5, mass_con_pm4, mass_con_pm10, hum, temp, voc, methane);
        Serial.println(buffer);

        features[0] = mass_con_pm1;
        features[1] = mass_con_pm2p5;
        features[2] = mass_con_pm4;
        features[3] = mass_con_pm10;
        features[4] = voc;
        features[5] = methane;
        inference();
        delay(1000);
      }
}
 

