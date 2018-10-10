import numpy as np
import time
from rexarm import *
"""
TODO: build a trajectory generator and waypoint planner
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""


class TrajectoryPlanner:
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0] * self.num_joints
        self.final_wp = [0.0] * self.num_joints
        self.dt = 0.5  # command rate
        self.time = 1.5  # NIC 10/4 time between points

    def set_initial_wp(self, wp ):
        self.initial_wp = wp

    def set_final_wp(self, wp=None):
        self.final_wp = wp

    def go(self, point, speed):
        self.rexarm.set_speeds(speed)
        self.rexarm.set_positions(point)
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def stop(self):
        pass

    '''in calc_time_from_waypoints() the max_speed will determine the total time that is required to complete traj from
     the initial to final wp we have assumed it as 3 for the time being

     def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
     #self.time = (self.final_wp-self.initial_wp)/max_speed
     pass'''

    def generate_cubic_spline(self, initial_wp, final_wp,l):
      p = np.empty(shape = (l, self.num_joints)) # NIC 10/4
      v = np.empty(shape = (l, self.num_joints)) # NIC 10/4
      t = np.arange(0,self.time+1,.03)
      t = np.transpose(t)
      for i in range(self.num_joints-1): # NIC 10/4
        ini = initial_wp[i]
        fin = final_wp[i]
        c = np.array([[ini], [fin], [0.1], [0.1]]) 
        b = np.array([[1, 0, 0, 0], [1, self.time, self.time*self.time, self.time*self.time*self.time], [0, 1, 0, 0], [0, 1, 2*self.time, 3*self.time*self.time]])
        '''taking the time taken to go from initial to final waypoint as 3s'''
        #b = np.linalg.pinv(b)
        a = np.linalg.solve(b, c) 
        for j in range(l):
          p[j, i] = (a[0] + (a[1] * t[j]) + (a[2] * t[j] * t[j]) + (a[3] * t[j] * t[j] * t[j]))
          v[j, i] = 1 + (a[1]) + 2 * (a[2] * t[j]) + 3 * (a[3] * t[j] ** 2)
          p[j, 4] = self.rexarm.joint_angles_fb[4] #-np.pi/2 # NIC 10/4 - adding orientation for wrst2 self.rexarm.joint_angles_fb[4] 
          v[j, 4] = 0.2 # NIC 10/4 - adding velocity for wrst2

      return p, v
     

    def execute_plan(self, plan, look_ahead=8):
        self.set_initial_wp(plan[0])
        self.set_final_wp(plan[1])
        lh = look_ahead

        l = int(self.time / .03)
        p, v = self.generate_cubic_spline(self.initial_wp, self.final_wp,l)
        for j in range(l):
          if (j + lh) <= l-1:
            self.go(point = p[j+ lh ], speed=v[j])
          else:
            self.go(point = p[l-1], speed=v[j])

