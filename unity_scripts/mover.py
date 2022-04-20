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
    plan = plan_trajectory(
        move_group, req.goal_pose, req.joints_input.joints)


    # 後処理 - Post Processing
    move_group.stop()
    move_group.clear_pose_targets()

    # If the trajectory has no points, planning has failed and we return an empty response
    # レスポンスの生成 - Response generation
    response = MoverServiceResponse()
    if plan.joint_trajectory.points:
        print("success>>>")
        response.trajectory = plan
    else:
        print("fail>>>")


    # print("plan points: ", len(response.trajectory.joint_trajectory.points))
    # print("plan points: ", response.trajectory.joint_trajectory.points[0])

    print("joints: ", req.joints_input.joints)

    return response
    

# モーションプランニング - Motion planning: Given the start angles 
def plan_trajectory(move_group, pose_target, start_joints):
    # スタート状態の指定 - Specifying the start state
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state.position = start_joints

    # Specifying the robot state 
    robot_state = RobotState()
    robot_state.joint_state = joint_state
    move_group.set_start_state(robot_state)

    # TODO: maybe get rid of this?? replace w/ pose_target
    # ゴール状態の指定 - Specifying the goal state
    pose_goal = geometry_msgs.msg.Pose()
    
    pose_goal.position = pose_target.position
    pose_goal.position.x = -pose_goal.position.x
    pose_goal.position.y = pose_goal.position.y
    pose_goal.position.z = pose_goal.position.z

    # pose_goal.position.x = pose_target.position.x
    # pose_goal.position.y = -pose_target.position.y
    # pose_goal.position.z = -pose_target.position.z
    pose_goal.orientation = pose_target.orientation
    # pose_goal.orientation.w = 1.0

    # print("\npose target:\n", pose_goal)
    # print("\n")

    # move_group.set_goal_orientation_tolerance(0.5)
    # move_group.set_goal_position_tolerance(0.5)
    move_group.set_planning_time(10);


    # プランの作成 - Creating the plan
    move_group.set_joint_value_target(pose_goal, True)
    # move_group.set_pose_target(pose_goal)
    # plan = move_group.go(wait=True)
    plan = move_group.plan()

    # # print("\nplan points: \n", plan)

    # If the plan does not work, throw an exception 
    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(pose_target, pose_target)
        raise Exception(exception_str)
    
    # プランの作成
    # return move_group.plan()

    return planCompat(plan)


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


if __name__ == "__main__":
    main()