import cv2
import freenect
import numpy as np

font = cv2.FONT_HERSHEY_SIMPLEX

def mouse_callback(event,x,y,flags,param):
    r = img[y][x][2]
    g = img[y][x][1]
    b = img[y][x][0]
    h = hsv[y][x][0]
    s = hsv[y][x][1]
    v = hsv[y][x][2]
    output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
    output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
    tmp = img.copy()
    cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (255,255,255))
    cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (255,255,255))
    cv2.imshow('window',tmp)
    if event == cv2.EVENT_LBUTTONDOWN:
        print "bgr: (%d,%d,%d) \n hsv: (%d,%d,%d)" % (b,g,r,h,s,v)


if(freenect.sync_get_depth() == None):
    kinectConnected = False
else:
    kinectConnected = True

if(kinectConnected):
    rgb_frame = freenect.sync_get_video()[0]
    
    img = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
    cv2.imshow('window', img)
    yellow_lower_range = np.array([23,200,200])
    yellow_upper_range = np.array([27,255,255])
    orange_lower_range = np.array([6,100,100])
    orange_upper_range = np.array([14,255,255])
    pink_lower_range = np.array([167, 100, 100])
    pink_upper_range = np.array([170, 200, 255])
    black_lower_range = np.array([0, 0, 0])  # a lot of the image is black so TODO find the least area and that will be the block
    black_upper_range = np.array([180, 255, 50])
    purple_lower_range = np.array([155, 90, 100])
    purple_upper_range = np.array([165, 255, 255])
    green_lower_range = np.array([50, 45, 100])
    green_upper_range = np.array([70, 255, 255])
    blue_lower_range = np.array([110, 80, 100])
    blue_upper_range = np.array([117, 255, 255])
    red_lower_range = np.array([172, 180, 100])
    red_upper_range = np.array([176, 255, 255])
    color_range = np.array([[29,100,200],[32,255,255],[6,200,100],[14,255,255],[167, 100, 100],[170, 200, 255],[0, 0, 0],[180, 255, 50],[140, 100, 100],[145, 255, 255],[50, 70, 100],[65, 200, 255],
				[108, 80, 100],[117, 255, 255],[170, 190, 100],[180, 230, 190]])
    mask = cv2.inRange(hsv,color_range[14],color_range[15])
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
    for cnt in contours:
        cnt_area = cv2.contourArea(cnt)
	moments = cv2.moments(cnt)
	if moments['m00']!=0:
           cx = int(moments['m10']/moments['m00'])
           cy = int(moments['m01']/moments['m00'])
	#print cx,cy

        rect = cv2.minAreaRect(cnt)
	print rect[2]
	box = cv2.boxPoints(rect)
	print box
	box = np.int0(box)
        im = cv2.drawContours(img,[box],0,(0,0,255),2)      
    cv2.drawContours(img,contours,-1,(255,255,255),2)
    cv2.imwrite('detected.png',mask)
    cv2.setMouseCallback("window",mouse_callback)
    while True:
        ch = 0xFF & cv2.waitKey(10)
        if ch == 0x1B:
           break
    cv2.destroyAllWindows()
