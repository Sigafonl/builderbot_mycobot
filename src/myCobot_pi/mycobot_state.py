#!/usr/bin/env python2
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_communication.msg import MycobotAngles

from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import Coord


mycobot = MyCobot('/dev/ttyAMA0')
mycobot.send_angle(Angle.J2.value, 10, 50)