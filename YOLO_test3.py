import cv2
import pyrealsense2 as rs
from realsense_depth import *

#Method to show depth w.r.t mouse pointer
frame_pos=(400,400)
def mouse_distance(event,x,y,args,params):
    global frame_pos
    print(x,y)

dc = DepthCamera()

#Mouse Event(if required)

cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", mouse_distance)

while True:
    ret, depth_frame, color_frame = dc.get_frame()
    cv2.circle(color_frame, frame_pos, 4,(0,0,255))
    distance_to_object= depth_frame[frame_pos[1],frame_pos[0]]
    #print(distance_to_object)
    cv2.putText(color_frame,"{}mm".format(distance_to_object),(frame_pos[0],fram_pos[1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Color frame", color_frame)
    key = cv2.waitKey(1)
    if key==27:
        break
