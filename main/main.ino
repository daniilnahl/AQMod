#define RGB_BRIGHTNESS 100
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(38, HIGH);  // Turn the RGB LED white
  delay(1000);
  digitalWrite(38, LOW);  // Turn the RGB LED off
  delay(1000);

  rgbLedWrite(38, RGB_BRIGHTNESS, 0, 0);  // Red
  delay(1000);
  rgbLedWrite(38, 0, RGB_BRIGHTNESS, 0);  // Green
  delay(1000);
  rgbLedWrite(38, 0, 0, RGB_BRIGHTNESS);  // Blue
  delay(1000);
  rgbLedWrite(38, 0, 0, 0);  // Off / black
  delay(1000);


}
