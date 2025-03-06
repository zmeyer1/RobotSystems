import RossROS
from camera_control import CameraInterpreter
from sensor_control import SensorController
from vilib import Vilib
import time, atexit

Vilib.camera_start()
Vilib.display()
atexit.register(Vilib.camera_close)
time.sleep(0.3)
control = SensorController()

def img_sensor():
    return Vilib.img

def img_interpreter(sensor_reading):
    interpret = CameraInterpreter()
    return interpret.interpret(sensor_reading)

def controller(control_val):
    control.sensor_steer(control_val)

def ultrasonic_stopper():
    dist = control.car.ultrasonic.read()
    if dist > 0 and dist < 5:
        control.car.stop()
        control.car.set_dir_servo_angle(0)
    else:
        control.car.forward(25)

def set_up_consumers():

    prod_cons_list = []

    sensor_bus = RossROS.Bus(None, name="Sensor Bus")
    control_bus = RossROS.Bus(None, name="Control Bus")
    timer_bus = RossROS.Bus(None, name="Timer Bus")

    prod_cons_list.extend([
        RossROS.Timer(timer_bus, duration = 15, termination_buses=(timer_bus,)),
        RossROS.Producer(ultrasonic_stopper, (), delay=0.1, termination_buses=(timer_bus,)),
        RossROS.Producer(img_sensor, (sensor_bus,), delay=0.1, termination_buses=(timer_bus,)),
        RossROS.ConsumerProducer(img_interpreter, (sensor_bus,), (control_bus,), delay=0.1, termination_buses=(timer_bus,)),
        RossROS.Consumer(controller, (control_bus,), delay=0.1, termination_buses=(timer_bus,)),
    ])
    return prod_cons_list

if __name__ == "__main__":

    consumer_list = set_up_consumers()
    RossROS.runConcurrently(consumer_list)
    Vilib.camera_close()