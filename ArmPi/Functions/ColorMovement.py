#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import numpy as np
import time
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from ColorPerception import range_rgb, ColorTracking


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

sorting_coordinates = {
    'red':   (-15 + 0.5, 12 - 0.5, 1.5),
    'green': (-15 + 0.5, 6 - 0.5,  1.5),
    'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
}

stacking_coordinates = {
    'red':   (-15 + 1, -7 - 0.5, 1),
    'green': (-15 + 1, -7 - 0.5, 1),
    'blue':  (-15 + 1, -7 - 0.5, 1),
}

dz = 2.5

GRIPPER_CLOSED_ANGLE = 500
GRIPPER_OPEN_ANGLE = GRIPPER_CLOSED_ANGLE - 280


class ColorMover:

    def __init__(self):
        self.AK = ArmIK()
        self.reset()
        self.z_stack = 0
   
    def reset(self):
        Board.setBusServoPulse(1, GRIPPER_CLOSED_ANGLE - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def pick_up_block_at_position(self, position, rotation_angle):
        # Do not fill in the runtime parameter, adapt the runtime
        result = self.AK.setPitchRangeMoving((position[0], position[1] - 2, 5), -90, -90, 0)
        if not bool(result):
            # not reachable, abort
            self.reset()
            return False
        time.sleep(result[2]/1000)
        Board.setBusServoPulse(1, GRIPPER_OPEN_ANGLE, 300)
        servo2_angle = getAngle(position[0], position[1], rotation_angle)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.8)
        self.AK.setPitchRangeMoving((position[0], position[1], 1.5), -90, -90, 0, 1000)  # Lower the height
        time.sleep(1)
        Board.setBusServoPulse(1, GRIPPER_CLOSED_ANGLE, 500)  # Gripper closes
        time.sleep(1)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((position[0], position[1], 12), -90, -90, 0, 1000)  # Lift the robotic arm
        time.sleep(1)
        return True


    def drop_off_at_position(self, pos_3d):
        posx, posy, posz = pos_3d
        result = self.AK.setPitchRangeMoving((posx, posy, 12), -90, -90, 0)   
        time.sleep(result[2]/1000)
        servo2_angle = getAngle(posx, posy, -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)
        self.AK.setPitchRangeMoving((posx, posy, posz + 3), -90, -90, 0, 500)
        time.sleep(0.5)
        self.AK.setPitchRangeMoving(pos_3d, -90, -90, 0, 1000)
        time.sleep(0.3)
        Board.setBusServoPulse(1, GRIPPER_OPEN_ANGLE, 500)  # Gripper opens, place the object down
        time.sleep(0.8)
        self.AK.setPitchRangeMoving((posx, posy, 12), -90, -90, 0, 800)
        time.sleep(0.8)
        self.reset()
        time.sleep(2)

if __name__ == "__main__":
    
    MODE = "PALLETIZE"
    # MODE = "SORT"
    
    tracker = ColorTracking(('red','green', 'blue'))
    tracker.start()
    mover = ColorMover()

    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            disp = tracker.find_persistent_location(img)
            if tracker.world_X and tracker.world_Y:
                reached = mover.pick_up_block_at_position(
                    position = (tracker.world_X, tracker.world_y),
                    rotation_angle = tracker.rect[2],
                    )
                if not reached:
                    continue
                if MODE == "SORT":
                    color = tracker.detect_color
                    mover.drop_off_at_position(sorting_coordinates[color])
                    tracker.reset()
                elif MODE == "PALLETIZE":
                    color = tracker.detect_color
                    posx,posy,z_base = stacking_coordinates[color]
                    mover.drop_off_at_position((posx,posy,mover.z_stack+z_base))
                    mover.z_stack += dz
                    tracker.reset()

                else:
                    print("You broke something. I can break stuff too.")
                    break


            cv2.imshow('Frame', disp)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()