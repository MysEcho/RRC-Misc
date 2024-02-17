import random
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO
import time
import csv
import os
from realsense_depth import *

#frame_pos=(300,300)
count =0

def dump_to_csv(msg,timestamp):
    csv_file_path = 'data.csv'
    mode = 'a' if os.path.exists(csv_file_path) else 'w'
    with open(csv_file_path, mode, newline='') as file:
        writer = csv.writer(file)
        if mode == 'w':
            writer.writerow(['Status','Timestamp'])
        writer.writerow([msg,timestamp])

#Object Listings
my_file = open("coco.txt", "r")
data = my_file.read()
class_list = data.split("\n")
my_file.close()

# Generate random colors for class list
detection_colors = []
for i in range(len(class_list)):
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    detection_colors.append((b, g, r))

# Pre-trained YOLOv8n model
model = YOLO("antareep_test/yolov8n.pt", "v8")

# Vals to resize video frames | small frame optimise the run
'''frame_wid = 640
frame_hyt = 480'''
###############################################################

#cap = cv2.VideoCapture("python_scripts/inference/videos/people.MP4") #Import Video
#cap = cv2.VideoCapture(0) # Webcam/RealSense
cap = DepthCamera()


while True:
    # Capture frame-by-frame
    ret, depth_frame, color_frame = cap.get_frame()

    # Predict on image
    detect_params = model.predict(source=[color_frame], conf=0.75, save=False)

    # Convert tensor array to numpy
    DP = detect_params[0].cpu().numpy()
    #print(DP)

    if len(DP) != 0:
        msg="Object Detected"
        timestamp=time.time()
        dump_to_csv(msg,timestamp)
        
        #Save Obstacle Frames

        #cv2.imwrite("frames/Obs_frame%d.jpg" % count, frame)
        #count = count +1
        
        for i in range(len(detect_params[0])):
            #print(i)

            #CHECK FOR CHANGE
###############################################################
            boxes = detect_params[0].boxes
            box = boxes[i]  # returns one box
            clsID = box.cls.cpu().numpy()[0]
            conf = box.conf.cpu().numpy()[0]
            bb = box.xyxy.cpu().numpy()[0]
            print(bb)
            cv2.rectangle(
                color_frame,
                (int(bb[0]), int(bb[1])),
                (int(bb[2]), int(bb[3])),
                detection_colors[int(clsID)],
                3,
            )
            #cv2.imwrite("frames/frame%d.jpg" % count, frame)
            #count = count +1
            #CHECK FOR DISPLAYING TEXT
#################################################################
            
    # Display the resulting frame
    
    cv2.imshow("ObjectDetection", color_frame)

    # Terminate run when "Q" pressed
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()