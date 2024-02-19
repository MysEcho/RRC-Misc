import websocket
import json
from datetime import datetime
import time
import os
import csv
import math

prev_timestamp = None
zero_time = 0
prev_x = None
prev_y = None
prev_z = None


def calculate_velocity(prev_value, current_value, time_diff):
    # Area of Trapezium rule used for calculating area under acceleration-time graph
    velocity = format((prev_value + current_value) / 2 * time_diff, ".4f")
    return velocity


def calculate_acceleration(acceleration_x, acceleration_y, acceleration_z):
    acceleration_magnitude = math.sqrt(
        acceleration_x**2 + acceleration_y**2 + acceleration_z**2
    )
    return acceleration_magnitude


def calculate_angle(acceleration_x, acceleration_y, acceleration_z):
    # Calculate the angle using arctangent
    angle = math.atan2(math.sqrt(acceleration_y**2 + acceleration_x**2), acceleration_z)
    return math.degrees(angle)


def on_message(ws, message):
    global prev_timestamp, prev_x, prev_y, prev_z, zero_time

    data = json.loads(message)
    # Accelerometer Data Dump
    # print(data)
    # Time stamp data from accelerometer
    timestamp = data["timestamp"] / 1e9
    # Unix Time from Linux
    unix_time = int(time.time())

    values = data["values"]
    x = values[0]
    y = values[1]
    z = values[2]

    ego_acceleration = calculate_acceleration(x, y, z)
    heading_angle = calculate_angle(x, y, z)

    if prev_timestamp is not None:
        time_diff = timestamp - prev_timestamp
        zero_time = zero_time + time_diff
    else:
        time_diff = 0
    if prev_x is not None:
        velocity_x = calculate_velocity(prev_x, x, time_diff)
        velocity_y = calculate_velocity(prev_y, y, time_diff)
        velocity_z = calculate_velocity(prev_z, z, time_diff)
        # ego_velocity = math.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
    else:
        velocity_x = velocity_y = velocity_z = None

    prev_timestamp = timestamp
    prev_x = x
    prev_y = y
    prev_z = z
    print(zero_time)
    # Dumping to CSV File
    csv_file_path = "data.csv"
    mode = "a" if os.path.exists(csv_file_path) else "w"
    with open(csv_file_path, mode, newline="") as file:
        writer = csv.writer(file)
        if mode == "w":
            writer.writerow(["vel(x)", "vel(y)", "vel(z)", "elapsed time"])
        writer.writerow(
            [
                velocity_x,
                velocity_y,
                velocity_z,
                # ego_acceleration,
                # heading_angle,
                zero_time,
            ]
        )


def on_error(ws, error):
    print("error occurred ", error)


def on_close(ws, close_code, reason):
    print("connection closed : ", reason)


def on_open(ws):
    print("connected")


def connect(url):
    ws = websocket.WebSocketApp(
        url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )

    ws.run_forever()


connect("ws://10.2.138.77:8080/sensor/connect?type=android.sensor.accelerometer")
