import picarx_improved as pcx
import enum, time

SPEED = 50

class Dir(enum.Enum):
    forwards = "forwards"
    backwards = "backwards"
    left = "left"
    right = "right"

class CarControl:

    def __init__(self):
        self.car = pcx.Picarx()
        self.car.reset()


    def steer(self, angle: float, direction: Dir, duration: float, speed: float = SPEED) -> None:
        self.car.set_dir_servo_angle(angle)
        assert direction == Dir.forwards or direction == Dir.backwards
        move = self.car.forward if direction == Dir.forwards else self.car.backward
        start = time.time()
        move(speed)
        while time.time() - start < duration:
            time.sleep(0.01)
        self.car.stop()
        self.car.set_dir_servo_angle(0)


    def parallel_park(self, direction: Dir) -> None:
        assert direction == Dir.left or direction == Dir.right
        sign = 1 if direction == Dir.right else -1
        self.steer(0, Dir.forwards, 2)
        self.steer(sign*30, Dir.backwards, 1)
        self.steer(0, Dir.backwards, 0.5)
        self.steer(sign*-30, Dir.backwards, 1)


    def k_turn(self, direction: Dir) -> None:
        assert direction == Dir.left or direction == Dir.right
        sign = 1 if direction == Dir.right else -1
        self.steer(sign*30, Dir.forwards, 1.7)
        self.steer(sign*-30, Dir.backwards, 1.7)
        self.steer(sign*30, Dir.forwards, 1.7)


if __name__ == "__main__":

    controller = CarControl()

    commands = ["steer", "kturn", "park", "ppark", "quit"]

    while True:
        user_input = input("Enter Command: ").split()
        if user_input[0].lower() not in commands:
            print(f"Invalid Command: {user_input[0]}")
            print(f"Valid inputs are {' '.join(commands)}")
            continue
        try:
            if user_input[0].lower() == "quit":
                break
            elif user_input[0].lower() == "steer":
                angle = float(user_input[1])
                direction = Dir(user_input[2])
                duration = float(user_input[3])
                controller.steer(angle, direction, duration)
            elif user_input[0].lower() == "kturn":
                direction = Dir(user_input[1])
                controller.k_turn(direction)
            elif user_input[0].lower() in ["park", "ppark"]:
                direction = Dir(user_input[1])
                controller.parallel_park(direction)

        except (IndexError, TypeError, ValueError) as e:
            print(f"Invalid Arguments to {user_input[0]}: {e}")

            


    # controller.steer(30, Dir.forwards, 3)
    # controller.parallel_park(Dir.right)
    # controller.k_turn(Dir.left)
