#!/usr/bin/env python
# coding: UTF-8

# Script credit: https://note.com/npaka/n/na3ec9819ac2d, Unity Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub 

from __future__ import print_function
from urllib import response

import rospy

import sys
import copy
import moveit_commander

import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
import moveit_commander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from builderbot_mycobot.srv import MoverService, MoverServiceResponse

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

class MoveGroup(object):
    def __init__(self):
        super(MoveGroup, self).__init__()

        # MoveGroupの生成 - MoveGroup generation
        group_name = "arm_group" # found the name in ~/catkin_ws/src/mycobot_ros/mycobot320/mycobot_320_moveit/config/firefighter.yaml 
        move_group = moveit_commander.MoveGroupCommander(group_name)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        #these names have to be the same as the move group (found from robot_info.py and mycobot 320 moveit config files)
        joint_names = [
                "joint2_to_joint1",
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
                "joint6output_to_joint6",
            ]

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

         # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


        # Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
        if sys.version_info >= (3, 0):
            def planCompat(plan):
                return plan[1]
        else:
            def planCompat(plan):
                return plan

    # モーションプランニング - Motion planning: Given the start angles 
    def go_to_pose_goal(self):
        move_group = self.move_group
        
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.3
        pose_goal.position.y = 0.02
        pose_goal.position.z = 0.25

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)

        # 後処理 - Post Processing
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        # For testing 
        current_pose = move_group.get_current_pose().pose
        joint_goal = move_group.get_current_joint_values()
        print("\npose target sent to moveit:\n", pose_goal)
        print("\nJoints calculated: \n", joint_goal)
        print("\nCurrent goal: \n",current_pose)

        return plan

    def plan_cartesian_path(self, scale=1):
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        move_group = self.move_group
        
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

# サーバーのリクエスト受信時に呼ばれる - Called when we recieve a server request
def receive_request(req):
    # MoveGroupの生成 - MoveGroup generation
    mover = MoveGroup()

    #TODO: verify that coord system is right, look what is handed back, look what our input is
    print("goal pose: ", req.goal_pose) # currently going in the opposite direction
    print("joints before: ", req.joints_input.joints)

    # モーションプランニング - Motion planning
    mover.go_to_pose_goal()
    cartesian_plan, fraction = mover.plan_cartesian_path()

    # If the trajectory has no points, planning has failed and we return an empty response
    # レスポンスの生成 - Response generation
    response = MoverServiceResponse()
    if cartesian_plan.joint_trajectory.points:
        print("success>>>")
        response.trajectory = cartesian_plan
    else:
        print("fail>>>")

    # what are we putting in and what are we getting out -> give 000-> what do we get back?

    # print("plan points: ", len(response.trajectory.joint_trajectory.points))
    # print("plan points: ", response.trajectory.joint_trajectory.points[0])

    print("joints after: ", req.joints_input.joints)

    return response
    

# メイン - Main
def main():
    # MoveItの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの初期化 - Initialize Movement
    rospy.init_node('mover_server')

    # サーバーの生成 - Initialize server
    rospy.Service('builderbot_moveit', MoverService, receive_request)
    print("Ready to plan")

    # ノード終了まで待機 - Wait until the node finishes 
    rospy.spin()

    # 

if __name__ == "__main__":
    main()