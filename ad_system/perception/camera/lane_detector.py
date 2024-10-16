import numpy as np
import cv2
import logging
from collections import defaultdict
from heapq import heapify, heappop, heappush
import math
import matplotlib.pyplot as plt 


class LaneDetector: 

    GRAD_THRESHOLD = 0.8
    HISTOGRAM_PEAK_THRESHOLD = 0
    PIXEL_FACTOR = 20.0

    def __init__(self, intrinsic_parameter): 
        self.intrinsic_parameter = intrinsic_parameter

    def get_lanes(self, rgb_img, depth_img): 
        # gaussian blurring, canny edge detection
        edges = self._detect_edges(rgb_img)

        # extract the lines in ROI
        roi_edges = self._region_of_interest(edges)

        # find intercept, slope of lines in the coordinates of vehicle 
        left_lane_image_coord, right_lane_image_coord = self._detect_line_segments(roi_edges) 

        if (left_lane_image_coord is None) or (right_lane_image_coord is None):
            return None, None

        # transform
        left_lane_camera_coord, right_lane_camera_coord, line_image = self._transform_to_camera_coordinate(
                                                                                        left_lane_image_coord, 
                                                                                        right_lane_image_coord,
                                                                                        rgb_img, depth_img)

        # return line_image
        lane_camera_coord = left_lane_camera_coord + right_lane_camera_coord 

        if lane_camera_coord:
            lane_camera_coord.sort(key=lambda x: (-x[1] / x[0])) 

        return line_image, lane_camera_coord  


    def _detect_edges(self, frame): 
        # Gaussian blur
        frame = cv2.GaussianBlur(frame,(5,5),0)

        # white color extraction
        lower_white = np.array([230, 230, 230])
        upper_white = np.array([255, 255, 255]) 
        mask = cv2.inRange(frame, lower_white, upper_white)

        # detect edges
        edges = cv2.Canny(mask, 200, 400)
        return edges
    

    def _region_of_interest(self, edges): 
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height*(1/2)),
            (width, height*(1/2)),
            (width, height),
            (0, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)


        masked_image = cv2.bitwise_and(edges, mask)
        return masked_image 

    def _detect_line_segments(self, roi_edges): 
        
        rho = 2  # precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # degree in radian, i.e. 1 degree
        min_threshold = 50  # minimal of votes

        line_segments = cv2.HoughLinesP(roi_edges, rho, angle, min_threshold, np.array([]), minLineLength=40, maxLineGap=30) 

        left = [] # [[slope, intercept], [slope, intercept] .... ]
        right = []

        if line_segments is not None:
            for line_segment in line_segments:
                for x1, y1, x2, y2 in line_segment:

                    # camera_to_world((x1,y1),(x2,y2),_depth_img)
                    params = np.polyfit((x1, x2), (y1, y2), 1)
                    if abs(x1-x2) <= 1:
                        continue
                    slope = params[0]
                    y_intercept = params[1]
                    if slope < 0:
                        left.append((slope, y_intercept))  # Negative slope = left lane
                    else:
                        right.append((slope, y_intercept))

        left.sort(key = lambda x : (x[0]))
        right.sort(key = lambda x : (x[0]))

        if (len(left) == 0) or (len(right) == 0):
            return None, None

        left_cur_slope = left[0][0]
        right_cur_slope = right[0][0]
        new_left = [left[0]]
        new_right = [right[0]]

        threshold_interval = 0.3

        for slope_l, intercept_l in left:
            if abs(slope_l - left_cur_slope) <= threshold_interval: # 같은 line으로 분류
                continue
            else: # 다른 line으로 분류
                new_left.append((slope_l,intercept_l))
                left_cur_slope = slope_l

        for slope_r, intercept_r in right:
            if abs(slope_r - right_cur_slope) <= threshold_interval: # 같은 line으로 분류
                continue
            else: # 다른 line으로 분류
                new_right.append((slope_r,intercept_r))
                right_cur_slope = slope_r

        return new_left, new_right

    def _transform_to_camera_coordinate(self, left, right, rgb_img, depth_img): 
        h, w, _ = rgb_img.shape
        line_image = np.zeros_like((rgb_img))

        left_camera = []
        right_camera = []

        y1, y2 = h, int(h * (3 / 5))
        for slope, intercept in left:
            if abs(slope) <= 0.2 or abs(slope) >= 8:
                continue
            x1, x2 = int((y1-intercept)/slope), int((y2-intercept)/slope)

            left_camera_line_info = self._get_camera_coordinates(slope, intercept, depth_img)
            left_camera.append(left_camera_line_info)

            cv2.line(line_image, (x1, y1), (x2, y2), (255,255,255), 1)

        for slope, intercept in right:
            if abs(slope) <= 0.2 or abs(slope) >= 8:
                continue
            x1, x2 = int((y1-intercept)/slope), int((y2-intercept)/slope) 

            right_camera_line_info = self._get_camera_coordinates(slope, intercept, depth_img) 
            right_camera.append(right_camera_line_info)

            cv2.line(line_image, (x1, y1), (x2, y2), (255,255,255), 1)

        return left_camera, right_camera, line_image
    
    def _get_camera_coordinates(self, slope, intercept, depth_img): 
        h, w= depth_img.shape
        cx, cy, f = self.intrinsic_parameter
        
        y1, y2 = int(h*(3/5)), int(h*(3.5/5))
        x1, x2 = int((y1 - intercept) / slope), int((y2 - intercept) / slope)

        if x1 >=1000:
            x1 = 999
        if x2 >= 1000:
            x2 = 999

        d1, d2 = depth_img[y1][x1], depth_img[y2][x2]

        # image coordinate
        y11, x11 = cy - y1, cx - x1
        y22, x22 = cy - y2, cx - x2

        # camera coordinate
        X1, Y1, Z1 = -x11*d1/f, y11*d1/f, d1 # 종방향 거리
        X2, Y2, Z2 = -x22 * d2 / f, y22 * d2 / f, d2  # 종방향 거리

        slope, intercept = np.polyfit((X1,X2),(Z1,Z2),1)
        return [slope, intercept]

    
     