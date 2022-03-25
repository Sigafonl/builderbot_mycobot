from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import numpy as np
import cv2
import datetime
from time import sleep
import os
thres = 0.45  # Threshold to detect object

kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color)


sleep(2)
colorFrame = kinect.get_last_color_frame()       # PyKinect2 returns a color frame in a linear array of size (8294400,)
colorFrame = colorFrame.reshape((1080, 1920 ,4))#.astype(np.uint8) # pictures are 1920 columns x 1080 rows with 4 bytes (BGRA) per pixel
now = datetime.datetime.now()                    # You know it
cv2.imwrite(now.strftime("%Y-%m-%d-%H-%M") + '.jpg',colorFrame) # saves the pictures
img = colorFrame[:, :, :3]
cv2.imshow("temp",img)
cv2.waitKey(1)
sleep(2)

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




    classIds, confs, bbox = net.detect(img, confThreshold=thres) # problem is here
    print(classIds, bbox)

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            img = np.array(img)
            cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
            cv2.putText(img, classNames[classId - 1].upper(), (box[0] + 10, box[1] + 30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(img, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Output", img)
    cv2.waitKey(1)

sleep(20)