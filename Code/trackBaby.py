import cv2
import numpy as np
from rigid_transform_3D import rigid_transform_3D

cap = cv2.VideoCapture(r'C:\Users\ruhan\Desktop\Xisys\Drone_Project\Drone_Project\ravi_block_150.MP4')
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 50,
                       qualityLevel = 0.2,
                       minDistance = 3,
                       blockSize = 3 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (30,30),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))
# Take first frame and find corners in it
ret, old_frame = cap.read()
old_frame = cv2.resize(old_frame, (0, 0), fx=0.5, fy=0.5)

height = len(old_frame)
width = len(old_frame[0,:])

# mask[int(height/2 - height/4): int(height/2+height/4), int(width/2 - width/4): int(width/2 + width/4), :] = 1
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
mask1 = np.zeros_like(old_gray)
roi = [300, 600, 300, 600]
mid_x = (600-300)/2
mid_y = (600-300)/2
center = [mid_x, mid_y, 1]
center = np.array(center)
# center = np.reshape(center, (1, 3))
mask1[roi[0]:roi[1], roi[2]:roi[3]] = 255
p0 = cv2.goodFeaturesToTrack(old_gray, mask = mask1, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)
frame_no= 1

while(1):
    # Skip n frames = 30
    for i in range(1):
        ret, frame = cap.read()
    frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if int(frame_no)% 600 == 0:
        print("HERE")
        p0 = cv2.goodFeaturesToTrack(old_gray, mask = mask1, **feature_params)
        frame_no = 1
        mid_x = (600 - 300) / 2
        mid_y = (600 - 300) / 2
        center = [mid_x, mid_y, 1]
        center = np.array(center)
    # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]
    good_new_t = np.transpose(good_new)
    good_old_t = np.transpose(good_old)
    temp = np.ones((len(good_new),), dtype=int)
    good_new_t = np.vstack([good_new_t,temp])
    good_old_t = np.vstack([good_old_t,temp])
    # good_new_t = np.transpose(good_new_t)
    # good_old_t = np.transpose(good_old_t)
    # draw the tracks
    ret_R, ret_t = rigid_transform_3D(good_old_t, good_new_t)
    center = ret_R@center + np.transpose(ret_t)
    center = np.reshape(center, (3,))
    # M = cv2.getAffineTransform(good_old_t, good_new_t)
    for i,(new,old) in enumerate(zip(good_new,good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        # mask = cv2.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)

        # frames_draw = cv2.rectangle(frames_draw, (xmin, ymin), (xmin + boxw, ymin + boxh), (255, 0, 0), 2)
        # frame = cv2.circle(frame,(int(a),int(b)),5,color[i].tolist(),2)
    frame = cv2.circle(frame, (int(center[0]), int(center[1])), 5, color[i].tolist(), 2)
    img = cv2.add(frame,mask)
    cv2.imshow('frame',img)
    k = cv2.waitKey(20) & 0xff
    if k == 27:
        break
    # Now update the previous frame and previous points


    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1,1,2)
    frame_no = frame_no + 1
cv2.destroyAllWindows()
cap.release()
