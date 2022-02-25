#!/usr/bin/env python2
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_communication.msg import MycobotAngles

def callback(data):
    """`mycobot/angles_real` subscriber callback method.

        Args:
            data (MycobotAngles): callback argument.
    """
    
    rate = rospy.Rate(30)  # 30hz
    
    while not rospy.is_shutdown():
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
        joint_state_send.header.stamp = rospy.Time.now()

        # process callback data.
        radians_list = [
            data.joint_1 * (math.pi / 180),
            data.joint_2 * (math.pi / 180),
            data.joint_3 * (math.pi / 180),
            data.joint_4 * (math.pi / 180),
            data.joint_5 * (math.pi / 180),
            data.joint_6 * (math.pi / 180),
        ]
        rospy.loginfo("res: {}".format(radians_list))

        joint_state_send.position = radians_list
        pub.publish(joint_state_send)    

def main():
    global pub, sub

    rospy.loginfo("start ...")
    rospy.init_node("real_listener", anonymous=True)
    
    # init publisher.
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    # init subscriber.
    sub = rospy.Subscriber("mycobot/angles_real", MycobotAngles, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass