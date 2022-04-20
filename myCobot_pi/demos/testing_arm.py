from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
import time

# Initialize a MyCobot object
mc = MyCobot(PI_PORT, 115200)
# Determine whether the robot arm is powered or not, and if there is no power supply, it needs to be powered first
if not mc.is_power_on():
    # Power the robot arm
    mc.power_on()
    print("power is on")

# Check that the six joints are working properly
# You can also use is_servo_enable(servo_id) to change single calibration
if mc.is_all_servo_enable():
    # Power off the robot arm
    mc.power_off()
    # Check whether the robot arm is power off
    if mc.is_all_servo_enable() == -1:
        print("power is off")