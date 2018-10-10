#rexarm.py
import numpy as np
from kinematics import *
import time

""" 
TODO:

Implement the missing functions
add anything you see fit

"""

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

class Rexarm():
    def __init__(self, joints, gripper):
        self.joints = joints
        self.gripper = gripper
        self.gripper_open_pos = np.deg2rad(30.0) # NIC 10/4
        self.gripper_closed_pos = np.deg2rad(-80.0) # NIC 10/4
        self.gripper_state = True
        self.estop = False
        """TODO: Find the physical angle limits of the Rexarm"""
        self.angle_limits = np.array([
                            [-30, 210],
                            [-120, 120],
                            [-123, 123],
                            [-180, 179.99]], dtype=np.float)*D2R

        """ Commanded Values """
        self.num_joints = len(joints)
        self.position = [0.0] * self.num_joints     # degrees
        self.speed = [1.0] * self.num_joints        # 0 to 1
        self.max_torque = [1.0] * self.num_joints   # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # degrees
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius
        self.move_fb = [0] *  self.num_joints

    def initialize(self):
        for joint in self.joints:
            joint.enable_torque()
            joint.set_position(0.0)
            joint.set_torque_limit(0.5)
            joint.set_speed(0.25)
        if(self.gripper != 0):
            self.gripper.set_torque_limit(0.5)#1.0)
            self.gripper.set_speed(0.8)
            self.close_gripper()

    def open_gripper(self,update_now = True): #NIC 10/4
        self.gripper_state = False     
        pass

    def close_gripper(self, update_now = True): #NIC 10/4
        self.gripper_state = True
        #startT = time.time()
        #if (time.time() - startT > 0.5):
        #    self.gripper.set_torque_limit(0.0)
        pass

    def toggle_gripper(self,gripper_state): #NIC 10/4
        if (self.gripper_state == False):
            self.gripper.set_position(self.gripper_open_pos)
        else:
            self.gripper.set_position(self.gripper_closed_pos)
        pass

    def set_positions(self, joint_angles, update_now = True):
        self.clamp(joint_angles)
        for i,joint in enumerate(self.joints):
            self.position[i] = joint_angles[i]
            if(update_now):
                joint.set_position(joint_angles[i])
                
    def set_speeds_normalized_global(self, speed, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speed
            if(update_now):
                joint.set_speed(speed)

    def set_speeds_normalized(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            if(update_now):
                joint.set_speed(speeds[i])

    def set_speeds(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            speed_msg = abs(speeds[i]/joint.max_speed)
            if (speed_msg < 3.0/1023.0):
                speed_msg = 3.0/1023.0
            if(update_now):
                joint.set_speed(speed_msg)
    
    def set_torque_limits(self, torques, update_now = True):
        for i,joint in enumerate(self.joints):
            self.max_torque[i] = torques[i]
            if(update_now):
                joint.set_torque_limit(torques[i])

    def send_commands(self):
        self.set_positions(self.position)
        self.set_speeds_normalized(self.speed)
        self.set_torque_limits(self.max_torque)
        self.toggle_gripper(self.gripper_state) # NIC 10/4

    def enable_torque(self):
        for joint in self.joints:
            joint.enable_torque()

    def disable_torque(self):
        for joint in self.joints:
            joint.disable_torque()

    def get_positions(self):
        for i,joint in enumerate(self.joints):
            self.joint_angles_fb[i] = joint.get_position()
        return self.joint_angles_fb

    def get_speeds(self):
        for i,joint in enumerate(self.joints):
            self.speed_fb[i] = joint.get_speed()
        return self.speed_fb

    def get_loads(self):
        for i,joint in enumerate(self.joints):
            self.load_fb[i] = joint.get_load()
        return self.load_fb

    def get_temps(self):
        for i,joint in enumerate(self.joints):
            self.temp_fb[i] = joint.get_temp()
        return self.temp_fb

    def get_moving_status(self):
        for i,joint in enumerate(self.joints):
            self.move_fb[i] = joint.is_moving()
        return self.move_fb

    def get_feedback(self):
        self.get_positions()
        self.get_speeds()
        self.get_loads()
        self.get_temps()
        self.get_moving_status()

    def pause(self, secs):
        time_start = time.time()
        while((time.time()-time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if(self.estop == True):
                break

    def clamp(self, joint_angles):
        """TODO"""
	#joint_angles = joint_angles*R2D
	if joint_angles[1]>(210*D2R):
		joint_angles[1]=210*D2R
	elif joint_angles[1]<(-120*D2R):
		joint_angles[1]=-120*D2R
	if joint_angles[2]>(120*D2R):
		joint_angles[2]=120*D2R
	elif joint_angles[2]<(-120*D2R):
		joint_angles[2]=-120*D2R
	if joint_angles[3]>(123*D2R):
		joint_angles[3]=123*D2R
	elif joint_angles[3]<(-123*D2R):
		joint_angles[3]=-123*D2R
	if joint_angles[0]>(180*D2R):
		joint_angles[0]=180*D2R
	elif joint_angles[0]<(-179.99*D2R):
		joint_angles[0]=-179.99*D2R
	
        pass
    
    def get_wrist_pose(self):
        """TODO"""
	#joint_angles = self.get_positions()       #to get the joint angles continuously
	T = FK_dh(self.joint_angles_fb) 			 # get transformation matrix using DH table form Kinematics.py	
	PoX_coord= FK_pox(self.joint_angles_fb)
	pos = PoX_coord
        return pos						#[T[0,3],T[1,3],T[2,3],np.arccos(T[1,2])]  #pos = [x,y,z,phi(cos inverse of T(0,0))] 
