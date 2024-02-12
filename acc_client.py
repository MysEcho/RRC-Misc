'''
This script has been written to obtain the velocity dataset when the Intel RealSense D455 is mounted on the wheelchair and moved around. The dataset is going to be used 
in an RL Model in development.
Author: Antareep Singha
Date: 12/02/2024
'''
import websocket
import json
from datetime import datetime
import time
import os
import csv

prev_timestamp = None
prev_x = None
prev_y = None
prev_z = None

def calculate_velocity(prev_value, current_value, time_diff):
    #Area of Trapezium rule used for calculating area under acceleration-time graph
    velocity = (prev_value + current_value) / 2 * time_diff
    return velocity

def on_message(ws, message):
    global prev_timestamp, prev_x, prev_y, prev_z
    
    data = json.loads(message)
    #Accelerometer Data Dump
    #print(data)
    #Time stamp data from accelerometer
    timestamp = data['timestamp'] / 1e9
    #Unix Time from Linux
    unix_time= int(time.time())

    values = data['values']
    x = values[0]
    y = values[1]
    z = values[2]
    
    if prev_timestamp is not None:
        time_diff = timestamp - prev_timestamp
    else:
        time_diff = 0
    if prev_x is not None:
        velocity_x = calculate_velocity(prev_x, x, time_diff)
        velocity_y = calculate_velocity(prev_y, y, time_diff)
        velocity_z = calculate_velocity(prev_z, z, time_diff)
    else:
        velocity_x = velocity_y = velocity_z = None
    
    prev_timestamp = timestamp
    prev_x = x
    prev_y = y
    prev_z = z
    #Dumping to CSV File
    csv_file_path = 'data.csv'
    mode = 'a' if os.path.exists(csv_file_path) else 'w'
    with open(csv_file_path, mode, newline='') as file:
        writer = csv.writer(file)
        if mode == 'w':
            writer.writerow(['x', 'y', 'z','time'])
        writer.writerow([velocity_x,velocity_y,velocity_z,unix_time])

def on_error(ws, error):
    print("error occurred ", error)
    
def on_close(ws, close_code, reason):
    print("connection closed : ", reason)
    
def on_open(ws):
    print("connected")
    

def connect(url):
    ws = websocket.WebSocketApp(url,
                              on_open=on_open,
                              on_message=on_message,
                              on_error=on_error,
                              on_close=on_close)

    ws.run_forever()
 
  
connect("ws://172.16.17.47:8080/sensor/connect?type=android.sensor.accelerometer") 
