import cv2, os
from vilib import Vilib
import picarx_improved as pcx
from sensor_control import SensorController
import numpy as np


def find_line(im):

    im_grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(im_grey, (9, 9), 0)

    # mask = cv2.Canny(blurred, 10, 20)
    _, mask = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY_INV)
    # mask = cv2.adaptiveThreshold(blurred,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
    #         cv2.THRESH_BINARY,11,2)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pt = ()
    if len(contours) > 0:
        line = max(contours, key = cv2.contourArea)
        M = cv2.moments(line)
        pt = (int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"]))

    # https://github.com/tprlab/pitanq-dev/blob/master/selfdrive/follow_line/README.md
    

    return pt


def steer_with_camera(car, controller, display=True):

    car.forward(25)

    while True:

        frame = Vilib.img

        frame = frame[frame.shape[0]//2:, :frame.shape[1], :]
        
        pt = find_line(frame)

        midpt = frame.shape[1]//2

        if len(pt) > 1:
            control = (pt[0] - midpt) / frame.shape[1]
            cv2.circle(frame, pt, radius = 5, color=(255,0,0), thickness=2)

            controller.sensor_steer(control)

        if display:

            cv2.imshow("window", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Press 'q' to quit
                car.stop()
                Vilib.camera_close()
                break

    if display:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    
    
    Vilib.camera_start()

    car = pcx.Picarx()

    car.set_cam_tilt_angle(-35)

    cont = SensorController(car)

    steer_with_camera(car, cont)

    

        