
import cv2
import numpy as np
import freenect

depth_frame = freenect.sync_get_depth()[0]

np.clip(depth_frame,0,2**10 - 1,depth_frame)
depth_frame >>= 2
depth_frame = depth_frame.astype(np.uint8)

cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
cv2.imshow('window', depth_frame)

while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 0x1B:
            break
cv2.destroyAllWindows()

