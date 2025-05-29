#include <MQUnifiedsensor.h>
/************************Hardware Related Macros************************************/
#define         Board                   ("Arduino UNO")
#define         Pin                     (4)  //Analog input 4 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-9") //MQ9
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (12) 
#define         RatioMQ9CleanAir        (9.6) //RS / R0 = 60 ppm 
/*****************************Globals***********************************************/

MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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

void loop() {
  MQ9.update();
    /*
  Exponential regression:
  GAS     | a      | b
  LPG     | 1000.5 | -2.186    5v supply.
  CH4     | 4269.6 | -2.648    5v supply.
  CO      | 599.65 | -2.244    for co I must switch between 5v and 1.5v being supplied to the sensor.
  */
  MQ9.setA(4269.6); MQ9.setB(-2.648); //methane values
  float CH4 = MQ9.readSensor(); // reads PPM concentration using the model, a and b values set previously or from the setup
  
  Serial.print("\n"); Serial.print(CH4);

  delay(500); //Sampling frequency
}
