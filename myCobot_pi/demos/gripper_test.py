from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
from pymycobot.mycobot import MyCobot
import time
import os
import sys
from port_setup import setup


def gripper_test(mc):
    print("Start check IO part of api\n")
    # Check if the gripper is moving
    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)

    # Set the current position to (2048).
    # Use it when you are sure you need it.
    # Gripper has been initialized for a long time. Generally, there
    # is no need to change the method.
    # mc.set_gripper_ini()

    # Set the gripper rotate to the position 2048
    mc.set_encoder(7, 2048)
    time.sleep(3)
    # Set the gripper rotate to the position 1300
    mc.set_encoder(7, 1300)
    time.sleep(3)

    # Set the gripper to the state 2048 at 70 speeds.
    mc.set_gripper_value(2000, 70)
    time.sleep(3)
    # Set the gripper to the state 1500 at 70 speeds.
    mc.set_gripper_value(1500, 70)
    time.sleep(3)

    mc.set_gripper_state(0, 70)
    time.sleep(3)
    
    mc.set_gripper_state(1, 70)
    time.sleep(3)

    # Get the value of gripper
    print("")
    print(mc.get_gripper_value())


if __name__ == "__main__":
    # Initialize a MyCobot object
    mc = MyCobot(PI_PORT, 115200)
    gripper_test(mc)