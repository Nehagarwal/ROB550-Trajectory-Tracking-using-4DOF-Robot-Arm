import time
import numpy as np
from numpy import genfromtxt
import csv
from kinematics import *
from ui import Ui_MainWindow

"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"


    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
	    if(self.next_state == "execute"):
		self.execute()
	    if(self.next_state == "record"):
		self.record()
	    if(self.next_state == "play"):
		self.play()
            if(self.next_state == "execute_plan"):
		self.execute_plan()
	    if(self.next_state == "task_1"):
  		self.task_1()
	    if(self.next_state == "task_2"):
  		self.task_2()
	    if(self.next_state == "task_5"):
  		self.task_5()
                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop() 

        if(self.current_state == "execute"):
	    if(self.next_state == "estop"):
		self.estop() 
	    if(self.next_state == "execute"):
		self.execute();
	    if(self.next_state == "idle"):
		self.idle()

	if(self.current_state == "record"):
	    if(self.next_state == "estop"):
		self.estop() 
	    if(self.next_state == "record"):
		self.record()
	    if(self.next_state == "idle"):
		self.idle()

	if(self.current_state == "play"):
	    if(self.next_state == "estop"):
		self.estop() 
	    if(self.next_state == "play"):
		self.play()
	    if(self.next_state == "idle"):
		self.idle()

        if(self.current_state == "execute_plan"):
	    if(self.next_state == "estop"):
		self.estop() 
	    if(self.next_state == "execute_plan"):
		self.execute_plan()
	    if(self.next_state == "idle"):
		self.idle()
	    
	if(self.current_state == "task_1"):
	    if(self.next_state == "estop"):
		self.estop() 
	    if(self.next_state == "task_1"):
		self.task_1()
	    if(self.next_state == "idle"):
		self.idle()

	if(self.current_state == "task_2"):
	    if(self.next_state == "estop"):
		self.estop() 
	    if(self.next_state == "task_2"):
		self.task_1()
	    if(self.next_state == "idle"):
		self.idle()
	
	if(self.current_state == "task_5"):
	    if(self.next_state == "estop"):
		self.estop() 
	    if(self.next_state == "task_5"):
		self.task_1()
	    if(self.next_state == "idle"):
		self.idle()

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
               

    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
	print self.rexarm.joint_angles_fb
	#joint_angles = self.rexarm.get_positions()
	#self.rexarm.get_wrist_pose()  
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()
        self.rexarm.send_commands()
	#print self.kinect.currentDepthFrame[0][0]
	#self.kinect.blockDetector()

    def execute(self):
        self.status_message = "State: Execute - Go to waypoints"
        self.current_state = "execute"
	waypoints = np.array([[ 0.0, 0.0, 0.0, 0.0],
 			     [ 1.0, 0.8, 1.0, 1.0],
 			     [-1.0,-0.8,-1.0,-1.0],
 			     [-1.0, 0.8, 1.0, 1.0],
 			     [1.0, -0.8,-1.0,-1.0],
 			     [ 0.0, 0.0, 0.0, 0.0]])
	for i in range(len(waypoints)):
	     self.rexarm.set_speeds([0.05,0.10,0.15,0.25])
	     self.rexarm.set_positions(waypoints[i])
	     self.rexarm.send_commands()
	     self.rexarm.get_feedback()
	     self.rexarm.pause(3)
	     
	self.next_state = "idle"

    def record(self):
        self.status_message = "State: Record waypoints"
        self.current_state = "record"
	self.rexarm.disable_torque()
	wp = np.array(self.rexarm.get_positions()) #wp is the joint angles at each waypoint
	with open('WP.csv','a') as fd:
	    writer = csv.writer(fd)
	    writer.writerow(wp)
		    
	fd.close()
	self.next_state = "idle"

    def play(self):
        self.status_message = "State: Repeating waypoints"
        self.current_state = "play"
	wpreader = csv.reader(open("WP.csv","rb"),delimiter=',')
	x = list(wpreader)
	waypoints = np.array(x).astype("float")
	self.rexarm.enable_torque()
	for i in range(len(waypoints)):
	     self.rexarm.set_speeds([0.05,0.10,0.15,0.25])	
	     self.rexarm.set_positions(list(waypoints[i]))
	     self.rexarm.send_commands()
	     self.rexarm.get_feedback()
	     self.rexarm.pause(1)
	self.next_state = "idle"
    
    def pick(self,color,t_phi):
	cx1,cy1 = self.kinect.blockDetector(color)
	start_d = self.kinect.currentDepthFrame[cy1][cx1]
	start_xy = self.kinect.pixeltoworldcoordinates([cx1,cy1,1], start_d)
	start_z = self.kinect.depthcalibration(start_d)
	start = [start_xy[0],start_xy[1],start_z+50,t_phi]
   	grab = [start_xy[0],start_xy[1],start_z+20,t_phi]
	start_wp = IK(start)
	grab_wp = IK(grab)
	idle = (0.0,0.0,0.0,0.0)
	blockorient = self.kinect.blockangle
	
	waypoints = np.array([idle,start_wp])
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
        self.tp.execute_plan(waypoints[0:2])
	
	#if t_phi == -90:
	if start_wp[0] < 0:
	   theta4 = 1.57 + start_wp[0]-blockorient
	else:
	   theta4 = start_wp[0]-blockorient - 1.57
	   #self.rexarm.set_positions(np.append(start_wp,theta4))
	#else:
	    #self.rexarm.set_positions(np.append(start_wp,-np.pi/2))
	joint_angle = np.append(start_wp,theta4)
	self.rexarm.set_positions(joint_angle)
	self.rexarm.pause(1)
	waypoints = np.array([start_wp,grab_wp])
	self.tp.execute_plan(waypoints[0:2])
	self.rexarm.gripper_state = True
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	self.rexarm.pause(1.5)
	waypoints = np.array([grab_wp,start_wp,idle])
	for i in range(len(waypoints)-1):
            self.tp.execute_plan(waypoints[i:i+2])
	start = [start_xy[0],start_xy[1],start_z,t_phi]
	return start
  
    def place(self,start):
	end = [start[0],start[1],start[2]+20,start[3]]
	end_wp = IK(end)
	end1 = [start[0],start[1],start[2]+60,start[3]]
	end_wp1 = IK(end1)
	idle = (0.0,0.0,0.0,0.0)#,0.0]
	waypoints = np.array([idle,end_wp])
        self.tp.execute_plan(waypoints[0:2])

	joint_angle = np.append(end_wp,end_wp[0])#+self.kinect.blockangle-np.pi/2)
	self.rexarm.set_positions(joint_angle)
	self.rexarm.pause(1)
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	self.rexarm.pause(1)
	waypoints = np.array([end_wp,end_wp1])
        self.tp.execute_plan(waypoints[0:2])
	self.rexarm.pause(1)
	self.rexarm.set_positions([0.0,0.0,0.0,0.0,0.0])	

    def place2(self,start):
	end = [start[0],start[1],start[2]+20,start[3]]
	end_wp = IK(end)
	end1 = [start[0]+100,start[1]+50,start[2]+20,start[3]]
	end_wp1 = IK(end1)
	idle = (0.0,0.0,0.0,0.0)#,0.0]
	waypoints = np.array([idle,end_wp])
        self.tp.execute_plan(waypoints[0:2])

	joint_angle = np.append(end_wp,end_wp[0])#+self.kinect.blockangle-np.pi/2)
	self.rexarm.set_positions(joint_angle)
	self.rexarm.pause(1)
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	self.rexarm.pause(1)
	waypoints = np.array([end_wp,end_wp1])
        self.tp.execute_plan(waypoints[0:2])
	self.rexarm.pause(1)
	self.rexarm.set_positions([0.0,0.0,0.0,0.0,0.0])	

	
    def task_1(self):
        self.status_message = "State: Executing the 1st Task"
        self.current_state = "task_1"
	# detect red block
	t_phi = -90
	start = self.pick(7,t_phi)
	start = [-start[0],start[1],start[2],start[3]]
	self.place(start)
	start_blue = self.pick(6,t_phi)
        start_blue= [-start_blue[0],start_blue[1],start_blue[2],start_blue[3]]
	self.place(start_blue)
	start_green = self.pick(5,t_phi)
        start_green= [-start_green[0],start_green[1],start_green[2],start_green[3]]
	self.place(start_green)
	"""cx1,cy1 = self.kinect.blockDetector(7)
	start_d = self.kinect.currentDepthFrame[cy1][cx1]
	start_xy = self.kinect.pixeltoworldcoordinates([cx1,cy1,1], start_d)
	start_z = self.kinect.depthcalibration(start_d)
	start = [start_xy[0],start_xy[1],start_z+30,t_phi]
	end = [-start_xy[0],start_xy[1],start_z+20,t_phi]
	end1 = [-start_xy[0],start_xy[1],start_z+40,t_phi]
	grab = [start_xy[0],start_xy[1],start_z+10,t_phi]
	start_wp = IK(start)
	end_wp = IK(end)
	end_wp1 = IK(end1)
	grab_wp = IK(grab)
	idle = [0.0,0.0,0.0,0.0]
	waypoints = np.array([idle,start_wp])
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
        self.tp.execute_plan(waypoints[0:2])
	#print self.rexarm.joint_angles_fb[4]
	blockorient = self.kinect.blockangle
	print "base angle", start_wp[0]*(180.0/3.14)
	print "state machine", blockorient
	if start_wp[0] < 0:
	   theta4 = 1.57 + start_wp[0]-blockorient
	else:
	   theta4 = start_wp[0]-blockorient - 1.57
	self.rexarm.set_positions(np.append(start_wp,theta4))
	print "grip angle",theta4*(180.0/3.14)
        self.rexarm.send_commands()
	waypoints = np.array([start_wp,grab_wp])
	self.tp.execute_plan(waypoints[0:2])

	self.rexarm.gripper_state = True
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	self.rexarm.pause(1.5)
	waypoints = np.array([grab_wp,start_wp,idle])
	for i in range(len(waypoints)-1):
            self.tp.execute_plan(waypoints[i:i+2])
        waypoints = np.array([idle,end_wp])
        self.tp.execute_plan(waypoints[0:2])
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	self.rexarm.pause(1)
	waypoints = np.array([end_wp,end_wp1])
        self.tp.execute_plan(waypoints[0:2])

	self.rexarm.set_positions([0.0,0.0,0.0,0.0,0.0])

	# detect blue block
	cx2,cy2 = self.kinect.blockDetector(6)
	start_d = self.kinect.currentDepthFrame[cy2][cx2]
	start_xy = self.kinect.pixeltoworldcoordinates([cx2,cy2,1], start_d)
	start_z = self.kinect.depthcalibration(start_d)
	start = [start_xy[0],start_xy[1],start_z+30,t_phi]
	end = [-start_xy[0],start_xy[1],start_z+20,t_phi]
	end1 = [-start_xy[0],start_xy[1],start_z+40,t_phi]
	grab = [start_xy[0],start_xy[1],start_z+10,t_phi]
	start_wp = IK(start)
	end_wp = IK(end)
	end_wp1 = IK(end1)
	grab_wp = IK(grab)
	idle = [0.0,0.0,0.0,0.0]
	waypoints = np.array([idle,start_wp])
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
        self.tp.execute_plan(waypoints[0:2])
	blockorient = self.kinect.blockangle
	print "base angle", start_wp[0]*(180.0/3.14)
	print "state machine", blockorient
	if start_wp[0] < 0:
	   theta4 = 1.57 + start_wp[0]-blockorient
	else:
	   theta4 = start_wp[0]-blockorient - 1.57
	self.rexarm.set_positions(np.append(start_wp,theta4))
	print "grip angle",theta4*(180.0/3.14)
        self.rexarm.send_commands()
	waypoints = np.array([start_wp,grab_wp])
        
	self.tp.execute_plan(waypoints[0:2])
	self.rexarm.gripper_state = True
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	waypoints = np.array([grab_wp,start_wp,idle])
	self.rexarm.pause(1.5)
	for i in range(len(waypoints)-1):
            self.tp.execute_plan(waypoints[i:i+2])
        waypoints = np.array([idle,end_wp])
        self.tp.execute_plan(waypoints[0:2])
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	self.rexarm.pause(1)
	waypoints = np.array([end_wp,end_wp1])
        self.tp.execute_plan(waypoints[0:2])

	self.rexarm.set_positions([0.0,0.0,0.0,0.0,0.0])"""
        self.next_state = "idle"  

    def task_2(self):
	self.status_message = "State: Executing the 2nd Task"
        self.current_state = "task_2"
	
	t_phi = -90
	start = self.pick(7,t_phi)
	print "start",start
	end = [-60,-110,start[2],t_phi]
	print "end",end
	self.place(end)
  	start_blue = self.pick(6,t_phi)
	print "start_blue",start_blue
  	end = [end[0],end[1],end[2]+start_blue[2],end[3]]
	print "end new",end
  	self.place(end)
	start_green = self.pick(5,t_phi)
	end = [end[0],end[1],end[2]+start_green[2],end[3]]
	self.place2(end)
	self.next_state = "idle" 
	pass 

    def task_5(self):
	self.status_message = "State: Executing the 5th Task"
        self.current_state = "task_5"
	end_points = [[55,-125],[0,-125],[-55,-125]]
	end_points2 = [[30,-125],[-30,-125]]
	end_points3 = [0,-125]
	t_phi = -90
	start = self.pick(7,t_phi)
	end = [end_points[0][0],end_points[0][1],start[2],t_phi]
	self.place(end)
  	start_blue = self.pick(6,t_phi)
  	end = [end_points[1][0],end_points[1][1],start[2],t_phi]
  	self.place(end)
	start_green = self.pick(5,t_phi)
  	end = [end_points[2][0],end_points[2][1],start[2],t_phi]
	self.place(end)
	start_yellow = self.pick(0,t_phi)
	end = [end_points2[0][0],end_points[0][1],2*start[2],t_phi]
	self.place(end)
	start_orange = self.pick(1,t_phi)
	end = [end_points2[1][0],end_points[1][1],2*start[2],t_phi]
	self.place(end)
	start_purple = self.pick(4,t_phi)
	end = [end_points3[0],end_points3[1],3*start[2],t_phi]
	self.place2(end)
	self.next_state = "idle" 
	pass 

    def execute_plan(self):
        self.status_message = "State: Executing the plan according to the traj plan"
        self.current_state = "execute_plan"
	start = []
	end = []
	
	"""location_strings = ["Start",
                            "End"]
        i = 0
        for j in range(2):
            self.status_message = "Start - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.tp_click_points[i] = self.kinect.last_click.copy()
		    
                    i = i + 1
                    self.kinect.new_click = False        
	
	print self.kinect.tp_click_points
	self.kinect.tp_click_points = np.array([[220,336],[200,300]])	
	
        t_phi = -90 # NIC 10/4 - Remember, phi is in degrees, negative is tool down position!
	start_d = self.kinect.currentDepthFrame[self.kinect.tp_click_points[0][1]][self.kinect.tp_click_points[0][0]]
	end_d = self.kinect.currentDepthFrame[self.kinect.tp_click_points[1][1]][self.kinect.tp_click_points[1][0]]
	start_xy = self.kinect.pixeltoworldcoordinates([self.kinect.tp_click_points[0][0],self.kinect.tp_click_points[0][1],1], start_d)
	start_z = self.kinect.depthcalibration(start_d)
	start = [start_xy[0],start_xy[1],start_z+40,t_phi] # NIC 10/4
	end_xy = self.kinect.pixeltoworldcoordinates([self.kinect.tp_click_points[1][0],self.kinect.tp_click_points[1][1],1], end_d)
	end_z = self.kinect.depthcalibration(end_d)
	#end = [end_xy[0],end_xy[1],end_z+40,t_phi] # NIC 10/4
	end1 = [end_xy[0],end_xy[1],end_z+60,t_phi]
	end2 = [end_xy[0],end_xy[1],end_z+30,t_phi]
 	print start,end
	start_wp = self.rexarm.joint_angles_fb
	newstart_wp = IK(start)
	end_wp1 = IK(end1)
	end_wp2 = IK(end2)
	#print newstart_wp,end_wp
	#waypoints = np.array([start_wp,newstart_wp,end_wp])
	waypoints = np.array([start_wp,end_wp1,end_wp2])"""

	# to test with block detector - Neha 10.6
	t_phi = -90
	#self.rexarm.joint_angles_fb = 0.0
	cx1,cy1 = self.kinect.blockDetector(6)
	start_d = self.kinect.currentDepthFrame[cy1][cx1]
	start_xy = self.kinect.pixeltoworldcoordinates([cx1,cy1,1], start_d)
	start_z = self.kinect.depthcalibration(start_d)
	start = [start_xy[0],start_xy[1],start_z+70,t_phi]
	end = [-start_xy[0],start_xy[1],start_z+30,t_phi]
	grab = [start_xy[0],start_xy[1],start_z+30,t_phi]
	#start_wp = self.rexarm.joint_angles_fb
	start_wp = IK(start)
	end_wp = IK(end)
	grab_wp = IK(grab)
	idle = [0.0,0.0,0.0,0.0]
	waypoints = np.array([idle,start_wp])#,end_wp])
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
        self.tp.execute_plan(waypoints[0:2])
	#print self.rexarm.joint_angles_fb[4]
	blockorient = self.kinect.blockangle
	print "base angle", start_wp[0]*(180.0/3.14)
	print "state machine", blockorient
	if t_phi == -90:
	   if start_wp[0] < 0:
	      theta4 = 1.57 + start_wp[0]-blockorient
	   else:
	      theta4 = start_wp[0]-blockorient - 1.57
	else:
	    theta4 = 1.57
	self.rexarm.set_positions(np.append(start_wp,theta4))
	print "grip angle",theta4*(180.0/3.14)
        self.rexarm.send_commands()
	waypoints = np.array([start_wp,grab_wp])
        self.rexarm.pause(1)
	self.tp.execute_plan(waypoints[0:2])
	self.rexarm.gripper_state = True
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	waypoints = np.array([grab_wp,start_wp,end_wp])
	for i in range(len(waypoints)-1):
            self.tp.execute_plan(waypoints[i:i+2])
	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
        waypoints = np.array([end_wp,idle])
 	self.tp.execute_plan(waypoints[0:2])
	self.rexarm.gripper_state = True
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	"""self.rexarm.enable_torque()

	self.rexarm.gripper_state = False 
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
	#self.tp.execute_plan(waypoints[0:2])
        for i in range(len(waypoints)-1):
            self.tp.execute_plan(waypoints[i:i+2])
	
	self.rexarm.gripper_state = True
	self.rexarm.toggle_gripper(self.rexarm.gripper_state)
        self.rexarm.send_commands()# NIC 10/4
	self.rexarm.gripper_state = True"""
        self.next_state = "idle"
    

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        #self.tp.go(max_speed=2.0)
	#self.kinect.loadCameraCalibration()
	#print self.kinect.intrinsic
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        #print self.kinect.rgb_click_points
        #print self.kinect.depth_click_points
	

	"""TODO Perform camera calibration here"""

	#self.kinect.loadCameraCalibration()
	self.kinect.depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.rgb_click_points,self.kinect.depth_click_points)
	self.kinect.pixeltoworld = self.kinect.getAffineTransform(self.kinect.worldcoordinate,self.kinect.rgb_click_points)
	self.kinect.kinectCalibrated = True
	#print self.kinect.pixeltoworld
	self.kinect.transform = np.matmul(np.linalg.pinv(self.kinect.intrinsic),np.append(self.kinect.pixeltoworld,[[0,0,1]],axis = 0))

        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

