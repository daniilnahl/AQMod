 #### 07/08/2025
 Was troubleshooting the bme280 sensors. Tested all the sensors I had and no one of them were giving me and I2C address when I was scanning for it while my 1602 lcd did give one. Conclusion I got faulty sensors. Will try to return them and order news ones ASAP.

 #### 07/10/2025
 Ordered new bmp280s, they are coming in tomorrow. Today I setup sen54 to work and its perfect. I modularized code for both sen54 and mq9 and now only bmp280 is left. Updated my outline and now its more clear as to what I need to do. 

 #### 07/12/2025
Soldered bmp280 pins. First time soldering, it was pretty cool. Then started setting up all three sensors, but now I am running into an issue with bmp280 and sen54 being on the same i2c bus, and I need the voltage from sen54 to drop down to 3.3 when it goes the sensor, if it doesnt it will damage the sensor and then damage my microcontroller.