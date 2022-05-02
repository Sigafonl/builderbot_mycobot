from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import numpy as np
import cv2

kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth) # creating a kinect object

while True:
    # --- Getting frames and drawing
    if kinect.has_new_depth_frame(): # checking for depth frame
        frame = kinect.get_last_depth_frame() # retrieving depth frame
        frameD = kinect._depth_frame_data # retrieving depth frame data
        frame = frame.astype(np.uint8)
        frame = np.reshape(frame, (424, 512))

        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        def click_event(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                print(x, y)
            if event == cv2.EVENT_RBUTTONDOWN:
                Pixel_Depth = frameD[(y + (x * 512))]
                print(Pixel_Depth)
        ##output = cv2.bilateralFilter(output, 1, 150, 75)
        frame = cv2.rotate(frame, cv2.cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow('KINECT Video Stream', frame)
        cv2.setMouseCallback('KINECT Video Stream', click_event)
        output = None

    key = cv2.waitKey(1)
    if key == 27: break