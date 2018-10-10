# ROB550-Trajectory-Tracking-using-4DOF-Robot-Arm
Armlab- For block detection and trajectory tracking of 4DOF Robot Arm

This project comprised of three parts:
1. Acting
  6-DOF rigid-body coordinate transforms using homogeneous coordinate transforms
  Forward kinematics modeling of a manipulator
  Inverse kinematics modeling of a manipulator
  Grasping

2. Sensing
  3D image/workspace calibration
  Object detection with OpenCV
  Depth camera sensors

3. Reasoning
  Path planning & path smoothing
  State machines 
 
control_station.py- Main program.  Sets up threads and callback functions.

dynamixel- This is a library of functions to interface with the official DynamixelSDK from Robotis using python.  Files define the same functions for each type of joint.  These joints are passed to the Rexarm class where member functions are provided to handle most relevant functionality.

rexarm.py- Implements the Rexarm class.This class contains: last joint command, last feedback from joints, functions to command the joints, functions to get feedback from joints,functions to do FK and IK  

kinect.py- Implements Kinect class. This class contains: functions to capture and convert frames, functions to load camera calibration data, functions to find and  perform 2D transforms, functions to perform world->cam and cam->world transforms
functions to detect blocks in the depth and rgb frames

state_machine.py- Implements the StateMachine class.The state machine is the heart of the control 

trajectory.py- Implements the TrajectoryPlanner class

kinematics.py- Implements functions for forward and inverse kinematics.

mainWindow.ui- This file defines the GUI interface, created using QtCreator. 
To compile a new ui.py file, run:

pyuic4 mainWindow.ui -o ui.py
ui.py
Output from QtCreator with GUI implementation in Python. 

camera_cal.py- Standalone program to generate camera distortion parameters and camera intrinsic matrix after calibrating with a checkerboard. Outputs the camera calibration into the calibration.cfg file. 

