from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
import time

# Initialize a MyCobot object
mc = MyCobot(PI_PORT, 115200)
# Determine whether the robot arm is powered or not, and if there is no power supply, it needs to be powered first
if not mc.is_power_on():
    # Power the robot arm
    mc.power_on()
# The robot arm arrive at position[0,0,0,0,0,0] at 30 speeds
mc.send_angles([0, 0, 0, 0, 0, 0], 30)
# Get the current time
start = time.time()
# Check whether the robot arm has arrived at the position [0,0,0,0,0,0] or not
while not mc.is_in_position([0, 0, 0, 0, 0, 0], 0):
    # Restore the robot arm movement
    mc.resume()
    # Let the robot arm move 0.5s
    time.sleep(0.5)
    # Pause the robot arm movement
    mc.pause()
    # Determine if the movement timed out
    if time.time() - start > 9:
        # Stop movement of the robot arm 
        mc.stop()
        break
# Get the current time
start = time.time()
# The robot arm arrive at position[88.68, -138.51, 155.65, -128.05, -9.93, -15.29] at 30 speeds
mc.send_angles([0, 87, -110.45, 12, -16.42, 0], 30)
# Check whether the robot arm has arrived at the position [88.68, -138.51, 155.65, -128.05, -9.93, -15.29] or not
while not mc.is_in_position([0, 87, -110.45, 12, -16.42, 0], 0):
    # Restore the movement of the robot arm
    mc.resume()
    # Let the robot arm move 0.5s
    time.sleep(0.5)
    # Pause the movement of the robot arm You can use is_paused() to check whether the robot arm is in a pause state or not.
    mc.pause()
    # Check if the movement timed out
    if time.time() - start > 9:
        mc.stop()
        # Stop the movement of the robot arm
        break
# Power off the robot arm after operation
mc.power_off()