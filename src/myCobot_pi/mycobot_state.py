#!/usr/bin/env python2
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_communication.msg import MycobotAngles

from pymycobot import PI_PORT, PI_BAUD # For raspberry pi version of mycobot.

from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import Coord

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)
    mycobot.send_angle(Angle.J2.value, 10, 50)

def listener():
    global mycobot
    rospy.init_node("control_joint", anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)

    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)

    mycobot = MyCobot(port, baud)

    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    listener()