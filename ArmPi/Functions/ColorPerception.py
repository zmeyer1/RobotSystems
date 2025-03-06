#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import numpy as np
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


# Central Loop is:
# if started
# reduce to (640, 480) and blur k=11
# for all the target colors, mask for that color (in LAB) (then close, open), and find max contours
# only if the contour has 2500+ area, draw a bounding box and calculate center pt
# draw on world transformed points & store em for distance calc
# distance count is used to account for discontinuities
# keeps track of all the old centers
# after a second, average out the position and reset the center list -- long term position

class ColorTracking:

    resized_shape = (640, 480)

    def __init__(self, target_colors):
        self.target_colors = target_colors
        self._is_running = False
        self.reset()

    def reset(self):
        self.start_pick_up = False
        self.last_x = 0
        self.last_y = 0
        self.world_x = 0
        self.world_y = 0
        self.persistant_start = None
        self.distance = np.inf
        self.center_list = []

    def start(self):
        self._is_running = True

    def stop(self):
        self._is_running = False

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Iterate through all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only when the area is greater than 300, the contour of the largest area is valid to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the largest contour

    def subsample_image(self, img: np.array) -> np.array:
        img_copy = img.copy()
        
        if not self._is_running:
            return img
        resized = cv2.resize(img_copy, self.resized_shape, interpolation=cv2.INTER_NEAREST)
        return cv2.GaussianBlur(resized, (11, 11), 11)
        

    def find_target_color(self, img: np.array) -> np.array:

        if not self._is_running:
            return img
        
        img_b = self.subsample_image(img)
        
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_lab = cv2.cvtColor(img_b, cv2.COLOR_BGR2LAB)  # Convert the image to LAB space

        area_max = 0
        areaMaxContour = 0


        for i in color_range:
            if i in self.target_colors:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # Bitwise operation on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Close operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find contours
                areaMaxContour, area_max = self.getAreaMaxContour(contours)  # Find the largest contour
                if area_max > 2500:  # Found the largest area
                    rect = cv2.minAreaRect(areaMaxContour)
                    box = np.int0(cv2.boxPoints(rect))

                    roi = getROI(box) # Get the ROI area
                    get_roi = True

                    img_centerx, img_centery = getCenter(rect, roi, self.resized_shape, square_length)  # Get the center coordinates of the block
                    self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.resized_shape) # Convert to real-world coordinates
                    
                    cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
                    cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) # Draw the center point
                    self.distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2)) # Compare with the last coordinates to determine whether it moved
                    self.last_x, self.last_y = self.world_x, self.world_y
                    break

        return img


    def find_persistent_location(self, img: np.array) -> np.array:

        display = self.find_target_color(img)
        if self.distance < 0.5:
            self.center_list.append((self.world_x, self.world_y))

            if self.persistant_start is None:
                self.persistant_start = time.time()
            elif self.persistant_start - time.time() > 1:
                self.start_pick_up = True
                self.world_X, self.world_Y = np.mean(self.center_list, axis=0)
                self.center_list = []
            
        else:
            self.persistant_start = None
            self.center_list = []
        return display

            

    def move_block(self):
        pass

if __name__ == "__main__":
    tracker = ColorTracking(('red','green', 'blue'))
    tracker.start()

    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            disp = tracker.find_persistent_location(img)
            cv2.imshow('Frame', disp)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()

