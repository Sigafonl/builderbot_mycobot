#!/usr/bin/env python
"""
    Subscribes to SourceDestination topic.
    Uses MoveIt to compute a trajectory from the target to the destination.
    Trajectory is then published to PickAndPlaceTrajectory topic.
"""
import rospy
import math

from builderbot_mycobot.msg import MyCobotMoveitJoints, EulerJoints
from moveit_msgs.msg import RobotTrajectory

pub = rospy.Publisher("/euler_joints", EulerJoints, queue_size=10)

def radianToEuler(radian):
    joint = radian * (180/math.pi)
    return round(joint, 2)

def get_joints(data):
    joints = [] 
    joints.append(radianToEuler(data.joints[0]))
    joints.append(radianToEuler(data.joints[1]))
    joints.append(radianToEuler(data.joints[2]))
    joints.append(radianToEuler(data.joints[3]))
    joints.append(radianToEuler(data.joints[4]))
    joints.append(radianToEuler(data.joints[5]))

    return joints

def listen(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard:\n%s", data)
    joints = get_joints(data)
    pub.publish(joints)
    rospy.loginfo(rospy.get_caller_id() + "\nRobot joint angles:\n%s", joints)
    

def talker():
    rospy.init_node('Joints', anonymous=True)
    rospy.Subscriber("/mycobot_joints", MyCobotMoveitJoints, listen)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    talker()
