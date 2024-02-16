import random
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO
import time
import csv
import os

#frame_pos=(300,300)
count =0

def dump_to_csv(msg,timestamp):
    csv_file_path = 'data.csv'
    mode = 'a' if os.path.exists(csv_file_path) else 'w'
    with open(csv_file_path, mode, newline='') as file:
        writer = csv.writer(file)
        if mode == 'w':
            writer.writerow(['a','b'])
        writer.writerow([msg,timestamp])

#Object Listings
my_file = open("python_scripts/utils/coco.txt", "r")
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
model = YOLO("python_scripts/weights/yolov8n.pt", "v8")

# Vals to resize video frames | small frame optimise the run
frame_wid = 640
frame_hyt = 480
###############################################################
#FOR REALSENSE DEPTH CAMERA
'''pipe =rs.pipeline()
cfg =rs.config()

cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
pipe.start(cfg)'''
###############################################################

cap = cv2.VideoCapture("python_scripts/inference/videos/people.MP4") #Import Video
#cap = cv2.VideoCapture(0) # Webcam/RealSense

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    #REALSENSE CONFIGURATION BLOCK(When implementing , dump to csv)
    ######################################################
    '''frame=pipe.wait_for_frames()
    depth_frame=frame.get_depth_frame()
    color_frame=frame.get_color_frame()

    depth_image= np.asanyarray(depth_frame.get_data())
    color_image= np.asanyarray(color_frame.get_data())
    cv2.circle(color_frame, frame_pos, 4,(0,0,255))
    distance_to_object= depth_frame[frame_pos[1],frame_pos[0]]
    cv2.putText(color_frame,"{}mm".format(distance_to_object),(frame_pos[0],fram_pos[1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)'''

    ######################################################
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # resize the frame | small frame optimise the run
    # frame = cv2.resize(frame, (frame_wid, frame_hyt))

    # Predict on image
    detect_params = model.predict(source=[frame], conf=0.75, save=False)

    # Convert tensor array to numpy
    DP = detect_params[0].numpy()
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
            clsID = box.cls.numpy()[0]
            conf = box.conf.numpy()[0]
            bb = box.xyxy.numpy()[0]
            print(bb)
            cv2.rectangle(
                frame,
                (int(bb[0]), int(bb[1])),
                (int(bb[2]), int(bb[3])),
                detection_colors[int(clsID)],
                3,
            )
            cv2.imwrite("frames/frame%d.jpg" % count, frame)
            count = count +1
            #CHECK FOR DISPLAYING TEXT
#################################################################
            # Display class name and confidence
            '''font = cv2.FONT_HERSHEY_COMPLEX
            cv2.putText(
                frame,
                class_list[int(clsID)] + " " + str(round(conf, 3)) + "%",
                (int(bb[0]), int(bb[1]) - 10),
                font,
                1,
                (255, 255, 255),
                2,
            )'''
##################################################################
            
    # Display the resulting frame
    
    cv2.imshow("ObjectDetection", frame)
    

    #DISPLAY REALSENSE FRAMES
    #cv2.imshow('rgb',color_image)

    # Terminate run when "Q" pressed
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()