import cv2, time
from vilib import Vilib
from sensor_control import SensorController
import numpy as np
import atexit


class CameraSensor:

    def __init__(self):
        Vilib.camera_start()
        atexit.register(Vilib.camera_close)
        time.sleep(0.5)

    def read(self):
        return Vilib.img
    
    def close(self):
        Vilib.camera_close()


class CameraInterpreter:

    def __init__(self):
        self.threshold=10
        self.max_value=255


    def interpret(self, reading: np.array, display=False):

        if reading is None:
            return None

        im_grey = cv2.cvtColor(reading, cv2.COLOR_BGR2GRAY)

        blurred = cv2.GaussianBlur(im_grey, (9, 9), 0)
        _, mask = cv2.threshold(blurred, self.threshold, self.max_value, cv2.THRESH_BINARY_INV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        control = 0
        if len(contours) > 0:
            line = max(contours, key = cv2.contourArea)
            M = cv2.moments(line)
            pt = (int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"]))
            midpt = reading.shape[1]//2
            control = (pt[0] - midpt) / reading.shape[1]

            if display:
                cv2.circle(reading, pt, radius = 5, color=(255,0,0), thickness=2)
                cv2.imshow("window", reading)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    cv2.destroyAllWindows()
                    return None
        
            
        return control


def steer_with_camera():


    sensor = CameraSensor()

    interpreter = CameraInterpreter()

    cont = SensorController()

    while True:

        frame = sensor.read()


        control = interpreter.interpret(frame)

        if control is None:
            break

        cont.sensor_steer(control)

    sensor.close()

if __name__ == "__main__":
    
    
    steer_with_camera()

    

        