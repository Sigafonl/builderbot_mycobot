from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
import time

# Initialize a mycobot object
mc = MyCobot(PI_PORT, 115200)
# Gets the angle and posture of the current head
coords = mc.get_coords()
print(coords)
# Intelligently plan your route, Let the head linearly arrive at the coordinate[59.9,-65.8,250.7], and keep[-50.99,83.14,-52.42]this position.
mc.send_coords([59.9, -65.8, 250.7, -50.99, 83.14, -52.42], 80, 1)
# Set the wait time
time.sleep(1.5)
# Intelligently plan your route, Let the head linearly arrive at the coordinate[59.9,-65.8,350.7], and keep[-50.99,83.14,-52.42]this position
mc.send_coords([59.9, -65.8, 350.7, -50.99, 83.14, -52.42], 80, 1)
# Set the wait time
time.sleep(1.5)
# Change only the x coordinates of the head,Set the x coordinate of the head to -40. Let it intelligently plan the route to move the head to the changed position
mc.send_coord(Coord.X.value, -40, 70)