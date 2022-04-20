#!/usr/bin/env python
# coding: UTF-8
import sys
import moveit_commander
import rospy


def main():
    # moveit_commanderの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの初期化
    rospy.init_node('robot_info')

    # RobotCommanderのインスタンス化
    robot = moveit_commander.RobotCommander()

    # MoveGroupCommanderのインスタンス化
    move_group = moveit_commander.MoveGroupCommander("arm_group")

    # ロボットのプランニングフレーム名の取得
    planning_frame = move_group.get_planning_frame()
    print ("planning_frame: %s" % planning_frame)

    # エンドエフェクタリンク名の取得
    end_effector_link = move_group.get_end_effector_link()
    print ("end_effector_link: %s" % end_effector_link)

    # ロボット内のグループ名のリストの取得
    group_names = robot.get_group_names()
    print ("group_names:", group_names)

    # ロボットの現在の状態の取得
    current_state = robot.get_current_state()
    print ("\n", current_state)

    # ノード終了まで待機
    rospy.spin()

if __name__ == "__main__":
    main()
