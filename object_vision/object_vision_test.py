from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import numpy as np
import math
import cv2
import datetime
from time import sleep
import os
thres = 0.53  # Threshold to detect object


kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color)
kinect_depth = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)


sleep(2)
colorFrame = kinect.get_last_color_frame()
colorFrame = colorFrame.reshape((1080, 1920 ,4)) #.astype(np.uint8) # pictures are 1920 columns x 1080 rows with 4 bytes (BGRA) per pixel
img = colorFrame[:,580:1660, :3]
img = cv2.resize(img,(512,424))
img = cv2.rotate(img, cv2.cv2.ROTATE_90_CLOCKWISE)
cv2.waitKey(1)
sleep(2)

# checking for depth frame
frame = kinect_depth.get_last_depth_frame() # retrieving depth frame

frame = frame.astype(np.uint8)
frame = np.reshape(frame, (424, 512))
frame = cv2.rotate(frame,cv2.cv2.ROTATE_90_CLOCKWISE)
frameD = kinect_depth._depth_frame_data  # retrieving depth frame data
frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)


classNames = []
classFile = "coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

    configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    weightsPath = "frozen_inference_graph.pb"

    net = cv2.dnn_DetectionModel(weightsPath, configPath)
    net.setInputSize(320, 320)
    net.setInputScale(1.0 / 127.5)
    net.setInputMean((127.5, 127.5, 127.5))
    net.setInputSwapRB(True)

    classIds, confs, bbox = net.detect(img, confThreshold=thres)

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            img = np.array(img)
            x = int((((box[0]) + box[2])+box[0])/2)
            y = int((((box[1]) + box[3])+box[1])/2)
            print(classNames[classId - 1]) #edit here


            Pixel_Depth = kinect_depth._depth_frame_data[(y + (x * 512))] #<ctypes.wintypes.LP_c_ushort object at 0x000001A698686648>
            Pixel_Depth = math.sqrt((Pixel_Depth**2) - (475**2))
            z = (635 - Pixel_Depth) / 1000  #665 gives us front of ball
            print("Distance from Robot" , z)
            if (x<220):
                print(220-x)

            if (x>220):
                y = x-220
                print(224 - y)

            cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
            cv2.putText(img, classNames[classId - 1].upper() + " CFD: " +str(round(confidence * 100, 2)), (box[0] , box[1] ),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(img, str(Pixel_Depth), (int((((box[0]) + box[2])+box[0])/2), int((((box[1]) + box[3])+box[1])/2)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("Output", img)
    cv2.imshow("depth",frame)
    cv2.waitKey(0)

#sleep(20)