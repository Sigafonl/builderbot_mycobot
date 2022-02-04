#!/usr/bin/env python2
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from mycobot_communication.msg import MycobotAngles

from pymycobot import PI_PORT, PI_BAUD # For raspberry pi version of mycobot.

from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import Coord

def callback():
    rate = rospy.Rate(30)  # 30hz

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

    # Initialize marker for rviz
    marker_ = Marker()
    marker_.header.frame_id = "/joint1"
    marker_.ns = "my_namespace"

    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        angles = mycobot.get_radians()
        data_list = []
        for index, value in enumerate(angles):
            data_list.append(value)

        #mycobot.send_angle(Angle.J2.value, 0, 50)

        mycobot.send_angles([0, 0, 0, 90, -180, 0], 20)
        # rospy.loginfo('{}'.format(data_list))
        #joint_state_send.position = data_list

        #pub.publish(joint_state_send)

        coords = mycobot.get_coords()

        # marker
        marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.SPHERE
        marker_.action = marker_.ADD
        marker_.scale.x = 0.04
        marker_.scale.y = 0.04
        marker_.scale.z = 0.04

        # marker position initial
        # print(coords)
        if not coords:
            coords = [0, 0, 0, 0, 0, 0]
            rospy.loginfo("error [101]: can not get coord values")

        marker_.pose.position.x = coords[1] / 1000 * -1
        marker_.pose.position.y = coords[0] / 1000
        marker_.pose.position.z = coords[2] / 1000

        marker_.color.a = 1.0
        marker_.color.g = 1.0
        pub_marker.publish(marker_)

        rate.sleep()

       

def listener():
    global mycobot
    global pub
    global pub_marker

    # Display node 
    #rospy.init_node("display", anonymous=True)

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
        pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)

        callback()

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
        listener()
    except rospy.ROSInterruptException:
        pass