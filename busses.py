from readerwriterlock import rwlock
from camera_control import CameraSensor, CameraInterpreter
from sensor_control import SensorController
import picarx_improved as pcx
import time

import concurrent.futures

class Bus:

    def __init__(self):
        self._message = None
        self.lock = rwlock.RWLockWriteD()

    def write(self, msg):
        with self.lock.gen_wlock():
            self._message = msg

    def read(self):
        with self.lock.gen_rlock():
            msg = self._message
        return msg
    
    
def sensor_producer(sensor_bus, delay):
    sense = CameraSensor()
    time.sleep(0.5)
    while True:
        sensor_bus.write(sense.read())
        time.sleep(delay)

def interpreter_producer_consumer(sensor_bus, control_bus, delay):
    interpret = CameraInterpreter()
    while True:
        control_bus.write(interpret.interpret(sensor_bus.read()))
        time.sleep(delay)

def controller_consumer(car, control_bus, delay):
    control = SensorController(car)
    car.forward(25)
    while True:
        control.sensor_steer(control_bus.read())
        time.sleep(delay)


def steer_with_camera(car):

    car.set_cam_tilt_angle(-35)

    sensor_bus = Bus()
    control_bus = Bus()

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor_producer, sensor_bus, 0.1)
        eInterpreter = executor.submit(interpreter_producer_consumer, sensor_bus, control_bus, 0.1)
        eController = executor.submit(controller_consumer, car, control_bus, 0.1)


if __name__ == "__main__":

    car = pcx.Picarx()
    steer_with_camera(car)
        