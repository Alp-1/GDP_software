"""Driver methods to drive rover through rpi (in MANUAL/AUTO mode, through FC interface on the low level side)"""

from rpi5_pwm import Servo
from time import sleep
import math


class Rover:

    def __init__(self, servo_left_pin=13, servo_right_pin=12):
        self.servo_right = Servo(servo_right_pin)
        self.servo_left = Servo(servo_left_pin)
        self.servo_left.value = 0
        self.servo_right.value = 0

    def drive(self, x, y, duration):
        """Move the rover for duration (seconds).
        x: forward/backward (+1/-1)
        y: left/right (+1/-1)
        """
        # clamp to -1/+1
        x = max(-1, min(x, 1))
        y = max(-1, min(y, 1))

        l, r = self.steering(x, y)

        self.servo_left.value = l
        self.servo_right.value = r

        sleep(duration)

        self.servo_left.value = 0
        self.servo_right.value = 0

    @staticmethod
    def steering(x, y):
        """Mix x,y to left, right (x,y, left, right are scaled to -1/+1)"""
        # x: forward/backward (+1/-1)
        # y: left/right (+1/-1)
        # convert to polar
        r = math.hypot(x, y)
        t = math.atan2(y, x)

        # rotate by 45 degrees
        t += math.pi / 4

        # back to cartesian
        left = r * math.cos(t)
        right = r * math.sin(t)

        # rescale the new coords
        left = left * math.sqrt(2)
        right = right * math.sqrt(2)

        # clamp to -1/+1
        left = max(-1, min(left, 1))
        right = max(-1, min(right, 1))

        return left, right


if __name__ == "__main__":
    rover = Rover()  # default pin 12,13 (PWM0,1)
    rover.drive(0.5, 0.5, 1)  # forward right
