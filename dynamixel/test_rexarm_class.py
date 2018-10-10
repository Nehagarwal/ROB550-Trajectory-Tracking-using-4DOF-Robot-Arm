
import os
import time
import numpy as np
os.sys.path.append('dynamixel/')             # Path setting
from dynamixel_XL import *
from dynamixel_AX import *
from dynamixel_MX import *
from dynamixel_bus import *
from rexarm import Rexarm
from trajectory_planner import TrajectoryPlanner

BAUDRATE   = 1000000
DEVICENAME = "/dev/ttyACM1".encode('utf-8')

dxlbus = DXL_BUS(DEVICENAME, BAUDRATE)
port_num = dxlbus.port()

base = DXL_MX(port_num, 0)
shld = DXL_MX(port_num, 1)
elbw = DXL_MX(port_num, 2)
wrst = DXL_AX(port_num, 3)
wrst2 = DXL_XL(port_num, 4)
grip = DXL_XL(port_num, 5)

rexarm = Rexarm((base,shld,elbw,wrst,wrst2), grip)
rexarm.set_speeds_normalized_global(0.15)
rexarm.initialize()
rexarm.open_gripper()
tp = TrajectoryPlanner(rexarm)

time.sleep(1)

waypoints = [[1.0,0.8,1.0,1.0,1.0],
             [-1.0,-0.8,-1.0,-1.0,-1.0],
             [-1.0,0.8,1.0,1.0,1.0],
             [1.0,-0.8,-1.0,-1.0,-1.0],
             [0.0,0.0,0.0,0.0,0.0]]

for wp in waypoints:
    goal_wp = wp
    tp.set_initial_wp()
    tp.set_final_wp(goal_wp)
    tp.go()
    rexarm.toggle_gripper()

time.sleep(1)

#rexarm.disable_torque()

dxlbus.close()


