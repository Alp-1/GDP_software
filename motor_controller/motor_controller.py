"""The main motor controller code"""

import time
import ustruct
from machine import Pin, UART
from motor_controller.motor import MotorSensing
from motor_controller.sabertooth import Sabertooth
from motor_controller.pid import PID
from user_interface.led import OnBoardLED

LEFT_MOTOR_ID = 2
LEFT_MOTOR_ENCODER_PIN = 12
LEFT_MOTOR_SM = 0
LEFT_MOTOR_CURRENT_PIN = 2

RIGHT_MOTOR_ID = 1
RIGHT_MOTOR_ENCODER_PIN = 14
RIGHT_MOTOR_SM = 1
RIGHT_MOTOR_CURRENT_PIN = 1

CENTRAL_HUB_TX_PIN = 8
CENTRAL_HUB_RX_PIN = 9

SABERTOOTH_ADDRESS = 128
SABERTOOTH_BAUDRATE = 9600
SABERTOOTH_TX_PIN = 0
SABERTOOTH_RX_PIN = 1  # Irrelevant

SIZE_OF_FLOAT = 4

# A flag to test the control loop without the current sensor
# Remove this flag for normal operation
CURRENT_SENSOR_ABSENT = False


class MotorController:
    """Controls the motor speed using the signals coming from the central hub"""

    # Some magic numbers for the motor controller
    ENABLE_COMMAND = b"\x80\x24\x21"
    DISABLE_COMMAND = b"\x8F\x21\x24"

    ENABLED = False

    THRESHOLD_CURRENT = 20  # Amps
    PID_PERIOD_MS = 10
    MAIN_LOOP_PERIOD_MS = 1

    # Speed setpoint range
    MAX_SPEED_RPM = 145
    MIN_SPEED_RPM = -145

    def __init__(
        self,
        motor_driver_serial=None,
        central_hub_interface=None,
        left_motor=None,
        right_motor=None,
        motor_driver=None,
    ):
        """Initialize the motor controller.
        The parameters are used for testing purposes,
        please use the default values for normal operation."""
        # Initialize the motors
        self.motor_driver_serial = (
            UART(
                0,
                baudrate=SABERTOOTH_BAUDRATE,
                tx=Pin(SABERTOOTH_TX_PIN),
                rx=Pin(SABERTOOTH_RX_PIN),
            )
            if motor_driver_serial is None
            else motor_driver_serial
        )
        self.left_motor = (
            MotorSensing(
                LEFT_MOTOR_ENCODER_PIN,
                LEFT_MOTOR_SM,
                LEFT_MOTOR_CURRENT_PIN,
            )
            if left_motor is None
            else left_motor
        )
        self.right_motor = (
            MotorSensing(
                RIGHT_MOTOR_ENCODER_PIN,
                RIGHT_MOTOR_SM,
                RIGHT_MOTOR_CURRENT_PIN,
            )
            if right_motor is None
            else right_motor
        )
        self.motor_driver: Sabertooth = (
            Sabertooth(
                self.motor_driver_serial,
                address=SABERTOOTH_ADDRESS,
            )
            if motor_driver is None
            else motor_driver
        )
        self.central_hub_interface = (
            UART(
                1,
                baudrate=9600,
                tx=Pin(CENTRAL_HUB_TX_PIN),
                rx=Pin(CENTRAL_HUB_RX_PIN),
            )
            if central_hub_interface is None
            else central_hub_interface
        )

        self.left_speed_command = 0
        self.right_speed_command = 0
        self.turn = 0

        self._previous_turn = 0  # Attribute to detect mode change

        self.pid_left = PID(
            Kp=0.125,
            Ki=0.05,
            Kd=0.025,
            setpoint=0,
            sample_time=self.PID_PERIOD_MS,
            output_limits=(self.MIN_SPEED_RPM, self.MAX_SPEED_RPM),
            time_fn=time.ticks_ms,
        )
        self.pid_right = PID(
            Kp=0.125,
            Ki=0.05,
            Kd=0.025,
            setpoint=0,
            sample_time=self.PID_PERIOD_MS,
            output_limits=(self.MIN_SPEED_RPM, self.MAX_SPEED_RPM),
            time_fn=time.ticks_ms,
        )

    def drive(self, left_speed: float, right_speed: float, turn=None):
        """Drive the motors at a given speed

        Parameters
        ----------
        left_speed: float
            The speed to drive the left motor at (-100: full reverse, 0: stop, 100: full forward)
        right_speed: float
            The speed to drive the right motor at (-100: full reverse, 0: stop, 100: full forward)
        turn: float
            The turn value (-100: full left, 0: straight, 100: full right)
            If this is not None, the rover will be driven in mixed mode
            using only values from left_speed and turn, i.e. the
            right_speed value will be ignored. (Assume this only happens in RC mode)
        """

        if turn is not None:
            self.motor_driver.drive_both(left_speed, turn)
        else:
            self.motor_driver.drive(LEFT_MOTOR_ID, left_speed)
            self.motor_driver.drive(RIGHT_MOTOR_ID, right_speed)

    def is_overcurrent(self, motor_id: int):
        """Check if the motor is overcurrent"""
        if motor_id == LEFT_MOTOR_ID:
            return abs(self.left_motor.current) > self.THRESHOLD_CURRENT
        elif motor_id == RIGHT_MOTOR_ID:
            return abs(self.right_motor.current) > self.THRESHOLD_CURRENT

    def overcurrent_check(self):
        """Stop the motors if overcurrent is detected. At the moment both motors will be stopped"""
        if self.is_overcurrent(LEFT_MOTOR_ID) or self.is_overcurrent(RIGHT_MOTOR_ID):
            self.drive(0, 0)
            self.ENABLED = False

    def parse_command(self, command):
        """Parse the command from the central hub. The command can be:

        - ENABLE_COMMAND
        - DISABLE_COMMAND
        - (left_speed, right_speed)
        - (left_speed, right_speed, turn)

        Parameters
        ----------
        command: bytes
            The command from the central hub

        Returns
        -------
        bool
            True if the command is valid, False otherwise
        """
        print(command)
        if command == self.ENABLE_COMMAND:
            self.ENABLED = True
            self.central_hub_interface.write(self.ENABLE_COMMAND)
            print("Enabled")
            OnBoardLED.set_period_ms(1000)
        elif command == self.DISABLE_COMMAND:
            self.ENABLED = False
            self.central_hub_interface.write(self.DISABLE_COMMAND)
            print("Disabled")
            OnBoardLED.set_period_ms(500)
        elif len(command) == 3 * SIZE_OF_FLOAT or len(command) == 2 * SIZE_OF_FLOAT:
            command = ustruct.unpack(
                "{}".format("f" * (len(command) // SIZE_OF_FLOAT)), command
            )
            self.left_speed_command, self.right_speed_command, *turn = command
            self.turn = turn[0] if turn else None
            self.detect_mode_change()

            if self.turn is not None:
                # Mixed mode drive
                self.drive(self.left_speed_command, 0, self.turn)
            else:
                self.pid_left.setpoint = self.rpm_to_setpoint(self.left_speed_command)
                self.pid_right.setpoint = self.rpm_to_setpoint(self.right_speed_command)
                self.pid_update()
                self.drive(self.left_speed_command, self.right_speed_command)
        else:
            print("Invalid command length {}".format(len(command)))
            return False
        return True

    def detect_mode_change(self):
        """Allow the class to set the timer period once per mode change"""
        if self._previous_turn is not None and self.turn is None:
            # mode change
            OnBoardLED.set_period_ms(2000)
        elif self._previous_turn is None and self.turn is not None:
            OnBoardLED.set_period_ms(1000)
        self._previous_turn = self.turn

    @staticmethod
    def rpm_to_setpoint(rpm):
        """Convert a rpm value to a setpoint within the range of -100 to 100"""
        setpoint = (rpm - MotorController.MIN_SPEED_RPM) / (
            MotorController.MAX_SPEED_RPM - MotorController.MIN_SPEED_RPM
        ) * 200 - 100
        return max(min(setpoint, 100), -100)

    def main_control_loop(self):
        """Inteprets the commands from the central hub and controls the motors"""
        OnBoardLED.on()

        while True:  # Can be changed to use async to allow other tasks to run
            # The command will latch until a new command is received
            if not CURRENT_SENSOR_ABSENT:
                self.overcurrent_check()
            if self.central_hub_interface.any() != 0:
                command = self.central_hub_interface.read()
                self.parse_command(command)
            if self.ENABLED:
                if self.turn is not None:
                    self.drive(self.left_speed_command, 0, self.turn)
                else:
                    # Run the PID control loop until a new command is received
                    self.pid_update()
            else:
                # Stop the motor if the motor is disabled
                self.drive(0, 0)

            time.sleep_ms(self.MAIN_LOOP_PERIOD_MS)

    def pid_update(self):
        """Execute one iteration of the PID control loop"""
        current_speed = [self.left_motor.rpm, self.right_motor.rpm]
        left_input_rpm = self.pid_left(current_speed[0])
        right_input_rpm = self.pid_right(current_speed[1])
        # Clip the speed command to the maximum speed
        self.left_speed_command = self.rpm_to_setpoint(left_input_rpm)
        self.right_speed_command = self.rpm_to_setpoint(right_input_rpm)
        print(
            "Left: {} Right: {}; rpm: {}".format(
                self.left_speed_command, self.right_speed_command, current_speed
            )
        )
        self.drive(self.left_speed_command, self.right_speed_command)

    def pid_until_stable(
        self,
        left_target_speed=0,
        right_target_speed=0,
        rpm_error_threshold=1,
        timeout=1000,
    ):
        """Execute the PID control loop until the speed is stable
        Use for testing purposes
        """
        self.pid_left.reset()
        self.pid_right.reset()
        last_time = time.ticks_ms()
        self.pid_left.setpoint = left_target_speed
        self.pid_right.setpoint = right_target_speed
        iteration = 0
        try:
            while time.ticks_diff(time.ticks_ms(), last_time) < timeout:
                iteration += 1
                start = time.ticks_us()
                self.pid_update()
                last_error = (self.pid_left._last_error, self.pid_right._last_error)
                print(
                    "Iteration {}: {} us".format(
                        iteration, time.ticks_diff(time.ticks_us(), start)
                    )
                )
                print("Last error at iteration {}: {}".format(iteration, last_error))
                if all(abs(error) < rpm_error_threshold for error in last_error):
                    break
        except KeyboardInterrupt:
            print("Keyboard interrupt")
        finally:
            self.drive(0, 0)
            self.drive(0, 0)
            self.drive(0, 0)


if __name__ == "__main__":
    motor_controller = MotorController()
    motor_controller.main_control_loop()
