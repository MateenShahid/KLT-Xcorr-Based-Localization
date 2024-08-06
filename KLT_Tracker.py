import numpy as np
import cv2
import time
import math
from rigid_transform_3D import rigid_transform_3D


class KltTracker:

    def __init__(self, objectframe):
        # params for ShiTomasi corner detection
        self.feature_params = dict(maxCorners=50,
                                   qualityLevel=0.2,
                                   minDistance=3,
                                   blockSize=3)
        # Parameters for lucas Kanade optical flow
        self.lk_params = dict(winSize=(30, 30),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.reg_length = (len(objectframe[:, 1])) / 2
        self.reg_width = (len(objectframe[1, :])) / 2
        self.reg_top_left_y = self.reg_length - (self.reg_length / 2)
        self.reg_top_left_x = self.reg_width - (self.reg_width / 2)
        self.reg_center_x = self.reg_top_left_x + (self.reg_width / 2)
        self.reg_center_y = self.reg_top_left_y + (self.reg_length / 2)
        self.objectRegion = [self.reg_top_left_x, self.reg_top_left_y, self.reg_length, self.reg_width]
        # self.bboxPoints = bbox2points(self.objectRegion(1,:));
        self.mask = np.zeros_like(objectframe)
        roi = [int(self.reg_top_left_y), int(self.reg_top_left_y + self.reg_length), int(self.reg_top_left_x),
               int(self.reg_top_left_x + self.reg_width)]

        self.center = [self.reg_center_x, self.reg_center_y, 1]
        self.center = np.array(self.center)
        # center = np.reshape(center, (1, 3))
        self.mask[roi[0]:roi[1], roi[2]:roi[3]] = 255
        self.old_points = cv2.goodFeaturesToTrack(objectframe, mask=self.mask, **self.feature_params)
        self.old_gray = objectframe.copy()

    def step(self, frame_gray):
        """
        This method find the transformation of points between previous frame and current frame. Then it applies it to
        the center of the frame and determines how much it has moved in pixels in x and y directions

        :return: returns the x, y displacement
        """

        # Calculate optical flow between the old points and new frame
        new_points, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.old_points, None,
                                                       **self.lk_params)
        good_new = new_points[st == 1]
        good_old = self.old_points[st == 1]
        good_new_t = np.transpose(good_new)
        good_old_t = np.transpose(good_old)
        temp = np.ones((len(good_new),), dtype=int)
        good_new_t = np.vstack([good_new_t, temp])
        good_old_t = np.vstack([good_old_t, temp])
        # mask = np.zeros_like(frame_gray)

        # Find translation and rotation between the two set of point (Old and New)
        ret_r, ret_t = rigid_transform_3D(good_old_t, good_new_t)

        # Rotate and translate the center point accoring to the matrices
        center = ret_r @ self.center + np.transpose(ret_t)
        center = np.reshape(center, (3,))

        # Just for Plotting
        #color = np.random.randint(0, 255, (100, 3))
        #frame = frame_gray.copy()
        #frame = cv2.circle(frame, (int(center[0]), int(center[1])), 5, np.array([255, 255, 255]).tolist(), 2)
        #frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
        #cv2.imshow('frame', frame)
        #k = cv2.waitKey(20) & 0xff
        #if k == 27:
         #    return

        # Now update the previous frame and previous points
        disp_x = center[0] - self.reg_center_x
        disp_y = center[1] - self.reg_center_y
        self.old_gray = frame_gray.copy()
        self.old_points = good_new.reshape(-1, 1, 2)
        self.center = center

        return disp_x, disp_y

    def update_feature_to_track(self, frame):
        """
        This method finds and stores points to be matched in the frames coming ahead

        :return: returns the x, y displacement
        """
        self.old_points = cv2.goodFeaturesToTrack(frame, mask=self.mask, **self.feature_params)
        self.center = [self.reg_center_x, self.reg_center_y, 1]
        self.center = np.array(self.center)
