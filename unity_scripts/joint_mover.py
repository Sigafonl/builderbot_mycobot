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
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from builderbot_mycobot.srv import MoverService, MoverServiceResponse

#these names have to be the same as the move group (found from robot_info.py and mycobot 320 moveit config files)
joint_names = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
    ]

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan

# サーバーのリクエスト受信時に呼ばれる - Called when we recieve a server request
def receive_request(req):
    # MoveGroupの生成 - MoveGroup generation
    group_name = "arm_group" # found the name in ~/catkin_ws/src/mycobot_ros/mycobot320/mycobot_320_moveit/config/firefighter.yaml 
    move_group = moveit_commander.MoveGroupCommander(group_name)
    robot = moveit_commander.RobotCommander()

    #TODO: verify that coord system is right, look what is handed back, look what our input is
    print("goal pose: ", req.goal_pose) # currently going in the opposite direction
    print("joints: ", req.joints_input.joints)

    # モーションプランニング - Motion planning
    plan = go_to_pose_goal(
        move_group, req.goal_pose, req.joints_input.joints)

    

    # If the trajectory has no points, planning has failed and we return an empty response
    # レスポンスの生成 - Response generation
    response = MoverServiceResponse()
    if plan.joint_trajectory.points:
        print("success>>>")
        response.trajectory = plan
    else:
        print("fail>>>")

    # what are we putting in and what are we getting out -> give 000-> what do we get back?

    # print("plan points: ", len(response.trajectory.joint_trajectory.points))
    # print("plan points: ", response.trajectory.joint_trajectory.points[0])

    print("joints: ", req.joints_input.joints)
    print()

    return response
    
# モーションプランニング - Motion planning: Given the start angles 
def go_to_pose_goal(move_group, pose_target, start_joints):
    # スタート状態の指定 - Specifying the start state
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state.position = start_joints

    # Specifying the robot state 
    robot_state = RobotState()
    robot_state.joint_state = joint_state
    move_group.set_start_state(robot_state)

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
    print("\nJoints: \n", joint_goal)
    print("\nCurrent goal: \n",current_pose)

    print("\nPlan: \n", plan)
    return plan


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