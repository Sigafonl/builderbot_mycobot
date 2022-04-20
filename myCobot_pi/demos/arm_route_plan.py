from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
import time

# Initialize a mycobot object
mc = MyCobot(PI_PORT, 115200)

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

# Gets the angle and posture of the current head
coords = mc.get_coords()
print(coords)
# Intelligently plan your route, Let the head linearly arrive at the coordinate[59.9,-65.8,250.7], and keep[-50.99,83.14,-52.42]this position.
#mc.send_coords([59.9, -165.8, 250.7, -50.99, 83.14, -52.42], 20, 1)
#[-147.1, -144.8, 257.7, -89.36, 8.85, 95.41]
mc.send_coords([73.1, -90.0, 385.0, -80.68, 0.52, -89.91], 80, 1)

# Set the wait time
time.sleep(3)
print(mc.get_coords())

# Intelligently plan your route, Let the head linearly arrive at the coordinate[59.9,-65.8,350.7], and keep[-50.99,83.14,-52.42]this position
#mc.send_coords([59.9, -165.8, 350.7, -50.99, 83.14, -52.42], 20, 1)
mc.send_coords([-1.0, -90.2, 459.5,  -80.68, 0.52, -89.91], 80, 1)

# Set the wait time
time.sleep(3)
# Change only the x coordinates of the head,Set the x coordinate of the head to -40. Let it intelligently plan the route to move the head to the changed position
mc.send_coord(Coord.X.value, -40, 70)
print(mc.get_coords())
time.sleep(3)