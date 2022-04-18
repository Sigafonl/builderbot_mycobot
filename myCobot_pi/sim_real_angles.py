#!/usr/bin/env python

from __future__ import print_function

from requests import get

import rospy

import time
import sys
import copy
import math
import moveit_commander
import actionlib
import math
import rospy
import geometry_msgs.msg
import moveit_msgs.msg

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from mycobot_communication.msg import MycobotAngles

from pymycobot import PI_PORT, PI_BAUD # For raspberry pi version of mycobot.
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import Coord

from builderbot_mycobot.msg import MyCobotMoveitJoints
from builderbot_mycobot.srv import MoverService, MoverServiceResponse


joints = []

def radianToEuler(radian):
    joint = radian * (180/math.pi)
    return round(joint, 2)

def get_joints(data):
    global joints
    rospy.loginfo(rospy.get_caller_id() + "\nIn Sub:\n%s", data)
    
    joints.append(radianToEuler(data.joints[0]))
    joints.append(radianToEuler(data.joints[1]))
    joints.append(radianToEuler(data.joints[2]))
    joints.append(radianToEuler(data.joints[3]))
    joints.append(radianToEuler(data.joints[4]))
    joints.append(radianToEuler(data.joints[5]))

    rospy.loginfo("\nNew joints:\n%s", joints)

    return joints
    

def pub_callback(data):
    rate = rospy.Rate(30)  # 30hz

    data = rospy.Subscriber("/mycobot_joints", MyCobotMoveitJoints, queue_size=10)
    joints = get_joints(data)

    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        angles = mycobot.get_radians()
        data_list = []
        for index, value in enumerate(angles):
            data_list.append(value)

        rospy.loginfo('Current position: {}'.format(data_list))
        joint_state_send.position = data_list

        pub.publish(joint_state_send)
        coords = mycobot.get_coords()
        print(coords)

        mycobot.send_angles(joints, 20)

        # marker position initial
        # print(coords)
        if not coords:
            coords = [0, 0, 0, 0, 0, 0]
            rospy.loginfo("error [101]: can not get coord values")

        rate.sleep()


def talker():
    global mycobot
    global pub

    # Connect to the mycobot
    print("Try connect real mycobot...")
    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))

    try:
        mycobot = MyCobot(port, baud)

        # Joint nodes 
        rospy.init_node("control_joint", anonymous=True)
        pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        
        mycobot.send_angles([0, 0, 0, 0, 0, 0], 20) # set the robot position to the default 
        print("::get_coords() ==> coords {}\n".format(mycobot.get_coords()))
        rospy.sleep(5) # wait 5 seconds

        pub_callback()

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

    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass