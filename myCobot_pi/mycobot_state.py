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

    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        angles = mycobot.get_radians()
        data_list = []
        for index, value in enumerate(angles):
            data_list.append(value)


        #mycobot.send_angles([0, 0, 0, 0, 0, 0], 20)
        #mycobot.send_angle(Angle.J3.value, 30, 20)
        #mycobot.send_angle(Angle.J4.value, -20, 20)
        #mycobot.send_angle(Angle.J5.value, 50, 20)

        rospy.loginfo('{}'.format(data_list))
        joint_state_send.position = data_list

        pub.publish(joint_state_send)
        coords = mycobot.get_coords()
        print(coords)
        
        # mycobot.send_coords([90, 90, 300, -135.57, 19.98, 102.03], 20, 0)
        # mycobot.send_coords([-23.1, -132.4, 310.7, -82.79, -1.57, -53.11], 20, 0)
        # mycobot.send_coords([90, 90, 300, -135.57, 19.98, 102.03], 20, 0)
        # mycobot.send_coords([-23.1, -132.4, 310.7, -82.79, -1.57, -53.11], 20, 0)
        mycobot.send_coords([55, -88, 500, -60, 0.4, -88], 20, 0)

        #mycobot.send_angles( [0, 87, -110.45, 12, -16.42, 0], 20)
        #rospy.sleep(3) # wait 2 seconds between movements
        #mycobot.send_coords([-93.9, -118.5, 306.1, -89.08, -8.63, 31.55], 20, 0)
        # mycobot.send_coords([-126.1, -124.1, 306.5, -89.92, 0.43, 39.55], 20, 0)
        # mycobot.send_coords([-99.4, -121.6, 301.6, 88.98, -30, -150.29], 20, 0)  
        # mycobot.send_coords([-99.4, -121.6, 301.6, -89.37, -6.12, 32.04 ], 20, 0)  

        # marker position initial
        # print(coords)
        if not coords:
            coords = [0, 0, 0, 0, 0, 0]
            rospy.loginfo("error [101]: can not get coord values")

        rate.sleep()

       
def talker():
    global mycobot
    global pub

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
        
        mycobot.send_angles([0, 0, 0, 0, 0, 0], 20) # set the robot position to the default 
        print("::get_coords() ==> coords {}\n".format(mycobot.get_coords()))
        rospy.sleep(5) # wait 5 seconds

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
        talker()
    except rospy.ROSInterruptException:
        pass