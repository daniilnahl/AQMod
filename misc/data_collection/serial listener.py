#python -m serial.tools.list_ports  --- shows the serial ports
import serial
import csv
import time
import os

ser = serial.Serial(port='[PORT NAME]',baudrate=115200)
csv_file_path = 'idle.csv' #change for each class
field_names = ['ms 1 um', 'ms 2.5 um','ms 4 um','ms 10 um','hum','temp','voc','methane']

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
        #need to break down the string by space into separate values
        data = [] #fill this up
        print(value_string)
        
        write_to_csv(data)
        print(f"Logged: {data}")
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\nData logging stopped.")

        
        
    

