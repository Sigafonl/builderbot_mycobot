import cv2
from kinect_camera import *
from mask_rcnn import *

rs = MicrosoftKinect()
mrcnn = MaskRCNN()

while True:
    ret, bgr_frame, depth_frame = rs.get_frame_stream()

    print(depth_frame)
    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("BGR frame", bgr_frame)