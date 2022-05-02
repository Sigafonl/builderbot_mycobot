from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
from pymycobot.mycobot import MyCobot 
import time
import rospy

if __name__ == '__main__':
    # Initialize a MyCobot object
    print("Try connect real mycobot...")
    mc = MyCobot(PI_PORT, 115200)

    if(mc.is_all_servo_enable() == 0):
        mc.focus_servo(1)
        mc.focus_servo(2)
        mc.focus_servo(3)
        mc.focus_servo(4)
        mc.focus_servo(5)
        mc.focus_servo(6)

    print("sending angles: [73.7, 135.6, -132.5, 47, -76.8, 93.5]")
    mc.send_angles([-90, 130.6, -130.5, -35, -90, 0], 30)

    # Check whether it move to the specified positon
    while not mc.is_in_position([-90, 130.6, -130.5, -35, -90, 0], 0):
        print("restore arm pose")
        # Restore the movement of the robot arm
        mc.resume()

    time.sleep(1)