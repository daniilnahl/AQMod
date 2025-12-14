#python -m serial.tools.list_ports  --- shows the serial ports
import serial
import csv
import time
import os
import re
ser = serial.Serial(port='COM8',baudrate=115200)
time.sleep(2)
csv_file_path = 'meth.csv' #change for each class
field_names = ['time','ms 1 um', 'ms 2.5 um','ms 4 um','ms 10 um','hum','temp','voc','methane']
time_counter = 0
def write_to_csv(data):
    file_exists = os.path.isfile(csv_file_path)
    
    with open(csv_file_path, mode='a', newline='') as csvfile:
        writer = csv.writer(csvfile)
    
        if not file_exists:
            writer.writerow(field_names)
            
        writer.writerow(data)        
      
try:  
    while True:      
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        
        #listen to values
        value_raw = ser.readline()
        value_string = str(value_raw, 'UTF-8')
        data = [timestamp]
        data += re.findall(r"\d+\.\d+", value_string) #\d+ matches digits + . for decimal point and then \d+ for digits after decimal. The + necessary to collect more than a single digit.
        
        write_to_csv(data)
        print(f"Logged: {data}")
        time.sleep(1)
        time_counter+=1
        
        if time_counter == 300:
            print("5 minute passed, continue data collection? y/n: ")
            user_input = input()
            if (user_input == 'n'):
                print("User stopped data collection\nSerial port {ser.name} closed.")
                ser.close()
                break
            else: 
                print("User continued data collection")
                time_counter = 0

            
        
except KeyboardInterrupt:
    print("\nData logging stopped.")

        
        
    

