#!/usr/bin/env python
"""
    Subscribes to SourceDestination topic.
    Uses MoveIt to compute a trajectory from the target to the destination.
    Trajectory is then published to PickAndPlaceTrajectory topic.
"""
import rospy

from builderbot_mycobot.msg import MyCobotMoveitJoints
from moveit_msgs.msg import RobotTrajectory


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard:\n%s", data)


def listener():
    rospy.init_node('Trajectory', anonymous=True)
    # rospy.Subscriber("/mycobot_joints", MyCobotMoveitJoints, callback)
    rospy.Publisher("/mycobot_joints", MyCobotMoveitJoints, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()