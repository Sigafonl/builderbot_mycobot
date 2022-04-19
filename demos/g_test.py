#!/usr/bin/env python2
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
from pymycobot.mycobot import MyCobot
import time


def gripper_test(mc):
    print("Start check IO part of api\n")
    # Check if the gripper is moving
    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)

   

    # Set the state of gripper, Let it open its paws quickly at 50 speeds
    mc.set_gripper_state(0, 70)
    print(mc.get_gripper_value())
    time.sleep(2)
    # Set the state of gripper, Let it close its paws quickly at 50 speeds
    mc.set_gripper_state(1, 70)
    print(mc.get_gripper_value())
    time.sleep(2)

    # Get the value of gripper
    print("")
    print(mc.get_gripper_value())

if __name__ == "__main__":
    # Initialize a MyCobot object
    mc = MyCobot(PI_PORT, 115200)
    gripper_test(mc)