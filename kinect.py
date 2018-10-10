import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import ConfigParser as configparser

inch2mm = 25.4
D2R = 3.141592/180.0
R2D = 180.0/3.141592
def mouse_to_raw(x,y):
    return int((y)*(1280.0/640.0)), int((x)*(960.0/480.0))

def raw_to_mouse(x,y):
    return int(y*(480.0/960.0)), int(x*(640.0/1280.0))
    
class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
	self.tp_click_points = np.zeros((2,2),int)
        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])
	self.blockangle = 0
        """ block info """
        self.block_contours = np.array([])
	self.intrinsic = np.array([[ 522.17756161,0.,322.92436352],[0.,519.93449831,254.8361581],[0.,0.,1.]])   # defined to store intrinsic matrix
	self.transform = np.array([])	# deifined to store complete transformation matrix (intrinsic^(-1)*[[a,b,c],[d,e,f],[0,0,1]]
	self.worldcoordinate = [[301.6,298.45],[279.4,-327.0],[-304.8,-306.38],[-300,301.6],[0.000,0.000]] #defined to store world coordinate
	self.pixeltoworld = np.array([])  # defined to store pixel to world affine transformation

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
	coord1_copy = np.array(coord1)
	if (np.array(coord1)).shape[1] == 2:
	    coord1 = np.c_[coord1,np.ones((np.array(coord1)).shape[0])]
	    coord2 = np.c_[coord2,np.ones((np.array(coord2)).shape[0])]


	A = np.zeros((2*((coord1).shape[0]),6))
	#A = np.zeros((2*coord1.shape[0], 6))
	A[::2, 0:3] = coord2
	A[1::2, 3:6] = coord2
	b = np.reshape(coord1_copy,(coord1_copy.size,1))
        x = np.matmul(np.linalg.pinv(A),b)
	#x = np.linalg.solve(A,b)
        #print x
	"""A[::2, 0:2] = coord2
        A[1::2, 3:5] = coord2
        b = np.reshape(coord1, (coord1.size, 1))"""
	"""pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print cv2.getAffineTransform(pts1,pts2)
        return cv2.getAffineTransform(pts1,pts2)"""
        return np.reshape(x, (2, 3))

    def registerDepthFrame(self, frame):
        """
        TODO:
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
	x = cv2.warpAffine(frame, self.depth2rgb_affine, (640,480))
	return x
        pass

    def loadCameraCalibration(self):
        """
        TODO:
        Load camera intrinsic matrix from file.
        """
	path = r'util/calibration.cfg'
        config = configparser.ConfigParser()
        config.read(path)
        cf = config.get('intrinsic', 'intrinsic matrix')
	#print cf
	#print cf.split()
        self.intrinsic = np.reshape(np.array([float(i) for i in cf.split()]), (3, 3))
	#print self.intrinsic

	
        pass
    
    def blockDetector(self,color):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
	# color is the index for selecting the color range, 0: yellow, 1: orange, 2: pink, 3: black, 4: purple, 5: green, 6: blue, 7: red
	
	img = cv2.cvtColor(freenect.sync_get_video()[0], cv2.COLOR_RGB2BGR)
    	hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    	kernel = np.ones((3, 3), np.uint8)
    	yellow_lower_range = np.array([23,200,200])
    	yellow_upper_range = np.array([27,255,255])
    	orange_lower_range = np.array([6,100,100])
    	orange_upper_range = np.array([14,255,255])
    	pink_lower_range = np.array([168, 100, 100])
    	pink_upper_range = np.array([172, 255, 255])
    	black_lower_range = np.array([6, 100, 100])  # a lot of the image is black so TODO find the least area and that will be the block
    	black_upper_range = np.array([14, 255, 255])
   	purple_lower_range = np.array([155, 90, 100])
    	purple_upper_range = np.array([165, 255, 255])
    	green_lower_range = np.array([50, 45, 100])
    	green_upper_range = np.array([70, 255, 255])
    	blue_lower_range = np.array([110, 80, 100])
    	blue_upper_range = np.array([117, 255, 255])
    	red_lower_range = np.array([155, 90, 100])
    	red_upper_range = np.array([165, 255, 255])
	color_range = np.array([[29,100,200],[32,255,255],[6,200,100],[14,255,255],[167, 100, 100],[170, 200, 255],[0, 0, 0],[180, 255, 50],[140, 100, 100],[145, 255, 255],[50, 70, 100],[65, 200, 255],
				[108, 80, 100],[117, 255, 255],[170, 190, 100],[180, 230, 190]])
    	mask = cv2.inRange(hsv,color_range[color*2],color_range[color*2+1])
    	cv2.imwrite('thres.png',mask)
    	mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
    	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    	_,contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    	res = cv2.bitwise_and(img,img,mask=mask)
    	max_area = 0
    	r = 1
    	best_cnt = 0
	cx = 0
	cy = 0
	flag = 0
	a = []
    	for cnt in contours:
            cnt_area = cv2.contourArea(cnt)
	    moments = cv2.moments(cnt)
	    if moments['m00']!=0:
               cx = int(moments['m10']/moments['m00'])
               cy = int(moments['m01']/moments['m00'])
	    rect = cv2.minAreaRect(cnt)
	    #a = np.append(a,abs(rect[2]))
	    #flag = flag + 1
      	    self.blockangle = abs(rect[2])*D2R
	    box = cv2.boxPoints(rect)
	    box = np.int0(box)
            im = cv2.drawContours(img,[box],0,(0,0,255),2)      

    	#cv2.drawContours(img,contours,-1,(255,255,255),2)

	print "kinect",self.blockangle
    	return cx,cy
	

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        pass

    def pixeltoworldcoordinates(self, pixels, d):
        
	 # question is where is the depth variable in the gui and how to multiply depth to rgb affine to the transform
         zw = self.depthcalibration(d)
	 uv1 = np.array(pixels)
         #uv1 = np.append(pixels, np.ones((pixels.shape[0], 1)))   # uv1 is 1x3 matrix
	 #print uv1
         xy1 = np.matmul(np.vstack((self.pixeltoworld,[[0, 0, 1]])), uv1.T) # pixeltoworld is 2x3 matrix so uv1 is transposed
         #xyz = np.vstack((xy1[0:2, :], zw.T))  # xw and yw from affine transform and zw using depth calibration function
         return xy1



    def depthcalibration(self,d):
	return 932 - 123.6*np.tan(d/2842.5 + 1.1863)
	
    
