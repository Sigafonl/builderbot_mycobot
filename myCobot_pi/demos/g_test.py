#!/usr/bin/env python2
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
from pymycobot.mycobot import MyCobot
from port_setup import setup
import time


def gripper_test(mc):
    print("Start check IO part of api\n")
    #Check if the gripper is moving
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

def g_test(mycobot):
    # mycobot = setup()

    print("opening")
    mycobot.set_gripper_state(0,70) # open gripper
    time.sleep(2)
    print("g value: ", mycobot.get_gripper_value())
    print("is moving: ", mycobot.is_gripper_moving())

    time.sleep(3)
    print("closing")
    mycobot.set_gripper_state(1,70) # close gripper
    time.sleep(2)
    print("g value: ", mycobot.get_gripper_value())
    print("is moving: ", mycobot.is_gripper_moving())
    

if __name__ == "__main__":
    # Initialize a MyCobot object
    mc = MyCobot(PI_PORT, 115200)
    # gripper_test(mc)
    g_test(mc)