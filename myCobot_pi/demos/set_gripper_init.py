#!/usr/bin/env python2

from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
from pymycobot.mycobot import MyCobot
from port_setup import setup
import time

mycobot = setup()
mycobot.set_gripper_ini()