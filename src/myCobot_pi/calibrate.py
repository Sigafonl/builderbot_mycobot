#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from mycobot_communication.msg import MycobotAngles

from pymycobot import PI_PORT, PI_BAUD # For raspberry pi version of mycobot.

from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import Coord

# Connect to the mycobot
print("Try connect real mycobot...")
port = rospy.get_param("~port", "/dev/ttyAMA0")
baud = rospy.get_param("~baud", 115200)
print("port: {}, baud: {}\n".format(port, baud))

try:
    mc = MyCobot(port, baud)
    # Detect whether the robot arm can burn into the program
    if mc.is_controller_connected() != 1:
        print("Connect the robot arm correctly for program writing")
        exit(0)

    # Fine-tune the robot arm to ensure that all snaps in the adjusted position are aligned
    # Based on the alignment of the robot arm port, only a case is given here
    mc.send_angles([0, 0, 0, 90, -180, 0], 20)

    for i in range(1, 7):
        mc.set_servo_calibration(i)

    rospy.spin()

except Exception as e:
    print(e)
    print(
        """\
        \rTry connect mycobot failed!
        \rPlease check wether connected with mycobot.
        \rPlease check wether the port or baud is right. ( port: /dev/ttyAMA0   baud: 115200 )
    """
    )
    exit(1)