import sys
import cv2
import numpy as np
import freenect

font = cv2.FONT_HERSHEY_SIMPLEX


"""def mouse_callback(event,x,y,flags,param):
    r = img[y][x][2]
    g = img[y][x][1]
    b = img[y][x][0]
    h = hsv[y][x][0]
    s = hsv[y][x][1]
    v = hsv[y][x][2]
    output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
    output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
    tmp = img.copy()
    cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
    cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
    cv2.imshow('window',tmp)
    if event == cv2.EVENT_LBUTTONDOWN:
        print "bgr: (%d,%d,%d) \n hsv: (%d,%d,%d)" % (b,g,r,h,s,v)"""



rgb_frame = freenect.sync_get_video()[0]
img = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
kernel = np.ones((5, 5), np.uint8)
cv2.namedWindow("window",1)
cv2.imshow('window',img)
yellow_lower_range = np.array([18,100,100])
yellow_upper_range = np.array([24,255,255])
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
mask = cv2.inRange(hsv,np.array([175,50,100]),np.array([180,255,255]))
cv2.imwrite('thres.png',mask)
mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
"""erosion = cv2.erode(mask,kernel,iterations = 1)
cv2.imwrite('erode.png',erosion)
dilation = cv2.dilate(erosion,kernel,iterations = 2)
cv2.imwrite('dilate.png',dilation)"""
_,contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
res = cv2.bitwise_and(img,img,mask=mask)
max_area = 0
r = 1
"""for cnt in contours:
        cnt_area = cv2.contourArea(cnt)
        if cnt_area > max_area:
            max_area = cnt_area
            best_cnt = cnt
            r=1


    if r==1:
        moments = cv2.moments(best_cnt)
        if moments['m00']!=0:
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])"""

cv2.drawContours(rgb_frame,contours,-1,(255,255,255),1)
#print cx,cy
#cv2.circle(res,(cx,cy),5,(255,255,255),-1)
#cv2.setMouseCallback("window",mouse_callback)

while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 27:
       break
cv2.destroyAllWindows()


