from robot_hat import ADC
import picarx_improved as pcx
from typing import List
import numpy as np

class Sensor:
    default_pins = ['A0', 'A1', 'A2']

    def __init__(self, grayscale_pins: List[str] = default_pins):
        self.ADC_pins = [ADC(pin) for pin in grayscale_pins]

    def read(self) -> List[float]:
        return [pin.read() for pin in self.ADC_pins]


class SensorInterpreter:
    def __init__(self, sensitivity: float = 10, polarity: bool = True):
        self.sensitivity = sensitivity
        self.polarity_sign = 1 if polarity else -1

    def interpret(self, reading: List[float]) -> float:

        max_reading = max(reading)

        normalized_reading = self.polarity_sign*(np.array(reading) - reading[1]) // self.sensitivity

        normalized_difference = (normalized_reading[0] - normalized_reading[2]) / (max_reading // self.sensitivity)

        return normalized_difference


class SensorController:
    def __init__(self, car, scaling_factor: float = 50):
        self.car = car
        self.scaling_factor = scaling_factor

    def sensor_steer(self, direction: float) -> float:
        angle_cmd = direction * self.scaling_factor
        self.car.set_dir_servo_angle(angle_cmd)
        return angle_cmd


def steer_with_sensors(car, sensor, interpreter, controller):

    car.forward(25)
    while True:
        controller.sensor_steer(interpreter.interpret(sensor.read()))



if __name__ == "__main__":
    car = pcx.Picarx()
    car.reset()
    sensor = Sensor()
    interpreter = SensorInterpreter()
    controller = SensorController(car)

    steer_with_sensors(car, sensor, interpreter, controller)