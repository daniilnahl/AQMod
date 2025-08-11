 #### 07/08/2025
 Was troubleshooting the bme280 sensors. Tested all the sensors I had and no one of them were giving me and I2C address when I was scanning for it while my 1602 lcd did give one. Conclusion I got faulty sensors. Will try to return them and order news ones ASAP.

 #### 07/10/2025
 Ordered new bmp280s, they are coming in tomorrow. Today I setup sen54 to work and its perfect. I modularized code for both sen54 and mq9 and now only bmp280 is left. Updated my outline and now its more clear as to what I need to do. 

 #### 07/12/2025
Soldered bmp280 pins. First time soldering, it was pretty cool. Then started setting up all three sensors, but now I am running into an issue with bmp280 and sen54 being on the same i2c bus, and I need the voltage from sen54 to drop down to 3.3 when it goes the sensor, if it doesnt it will damage the sensor and then damage my microcontroller.

#### 07/22/2025
- **ISSUE** with allocation enough memory for each task due to that running a lot of stack size tests and trying to find the right amount for each task(ERROR FIXED).
- Must allocate enough stack size (in bytes for esp32) for tasks or it will keep crashing and giving me panic errors: "Guru Meditation Error: Core  0 panic'ed (Unhandled debug exception). Debug exception reason: Stack canary watchpoint triggered (Get Data from B)"
- init functions for each sensors must be within the task function but before the loop because otherwise watchdog will keep getting activated and trying to reboot esp32(infinite error loop).
- **FOR NEXT SESSION**: check last chats in chatgpt and implement fixes for sen54 messages leaking into other sensors' messages.

#### 07/23/2025
- Need to create a main message buffer for all tasks to use then keep the error message buffer within the sen 54 task as its own entity.
- Today started fixing the fragmented message errors, using snprintf again. Revisiting the old stuff!! I forgot how it fully works plus I am woring on C++ this time, not C so must convert one style to another. Chatgpt and google helping me out.


#### 07/28/2025
- Created placeholder tasks for data analysis and bluetooth.
- Started learning about watchdog.

#### 07/30-08/01
- Added a bunch of notes on watchdog and TWDT, turns out its default configured even before I run setup(), the configs happens out of the box, thus I had to reconfig to fit my criterias.
- Set up watchdog to only watch defined tasks excluding idle tasks because it was simpler. I did it by telling watchdog through config to not subscribe to any of the cores' idle tasks. 
- Next on agenda is to make a single task for i2c sensors to ensure sensors are not trying to send data through i2c to mcu at the same time which can potentially corrupt data received.

#### 08/07
- Combined sensor tasks into one reducing stack memory used by tasks from 7000 down to 2800.
- Created basic algorithm to clasify the quality of each metric based on their value. Simple algorithm, there's defintely a way to improve it. !!!!For now use it and when tinyML is implemented comeback and improve!!!!
- SEN54 readings are being wack, I need to let the sensor warm up for 60 seconds after running startMeasure...() so it actually gets the pm values. I am thinking of doing moving init functions into a separate task and running them only once using a simple bool flag like the one I have for watchdog init.
- also run command from sen54 to clean the sensor.

#### 08/08
- run sen54 cleaning protocol then the debugger code. Otherwise didn't do anything of significance.

#### 08/10
- pm values are still not working. Tomorrow before beginning work on bluetooth try running the sen54 library command to get pm values as integers and implement fan cleaning each time setup() is ran. 
- Otherwise, queue is working perfectly. freeRTOS part is officially complete, next fixing sen54 and only then setting up bluetooth.