"""The main motor controller code"""

import time
from machine import Pin, UART
from motor_controller.motor import MotorSensing
from motor_controller.sabertooth import Sabertooth
from motor_controller.pid import PID
from protocol import Commands
from user_interface.led import OnBoardLED

LEFT_MOTOR_ID = 1
LEFT_MOTOR_ENCODER_PIN = 12
LEFT_MOTOR_SM = 0
LEFT_MOTOR_CURRENT_PIN = 2

RIGHT_MOTOR_ID = 2
RIGHT_MOTOR_ENCODER_PIN = 14
RIGHT_MOTOR_SM = 1
RIGHT_MOTOR_CURRENT_PIN = 1

CENTRAL_HUB_TX_PIN = 4
CENTRAL_HUB_RX_PIN = 5

SABERTOOTH_ADDRESS = 128
SABERTOOTH_BAUDRATE = 9600
SABERTOOTH_TX_PIN = 0
SABERTOOTH_RX_PIN = 1  # Irrelevant


# A flag to test the control loop without the current sensor
# Remove this flag for normal operation
CURRENT_SENSOR_ABSENT = False


class MotorController:
    """Controls the motor speed using the signals coming from the central hub"""

    THRESHOLD_CURRENT = 15  # Amps
    OVERCURRENT_TIMEOUT_MS = 1000  # ms
    PID_PERIOD_MS = 200  # Encoder is 10ms + overhead
    MAIN_LOOP_PERIOD_MS = 1
    SENSOR_UPDATE_PERIOD_MS = 1000  # Update the sensor every 2000ms

    # Speed setpoint range
    MAX_SPEED_RPM = 165
    MIN_SPEED_RPM = -165
    # The maximum range of the PID controller
    MAX_PID_RANGE = (MIN_SPEED_RPM, MAX_SPEED_RPM)
    PID_DEADZONE = 10  # Prevent noise from causing the motor to oscillate

    MIXED = 0
    INDEPENDENT_MOTOR = 1

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
                baudrate=4800,
                tx=Pin(CENTRAL_HUB_TX_PIN),
                rx=Pin(CENTRAL_HUB_RX_PIN),
            )
            if central_hub_interface is None
            else central_hub_interface
        )

        self.left_speed_command = 0
        self.right_speed_command = 0
        self.turn = 0

        self.current_mode = self.MIXED
        self._previous_turn = 0  # Attribute to detect mode change

        self.pid_left = PID(
            Kp=0.25,
            Ki=0.001,
            Kd=0.005,
            setpoint=0,
            sample_time=self.PID_PERIOD_MS,
            output_limits=self.MAX_PID_RANGE,
            time_fn=time.ticks_ms,
        )
        self.pid_right = PID(
            Kp=0.25,
            Ki=0.001,
            Kd=0.005,
            setpoint=0,
            sample_time=self.PID_PERIOD_MS,
            output_limits=self.MAX_PID_RANGE,
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
        fault = False
        result_left = self.is_overcurrent(LEFT_MOTOR_ID)
        result_right = self.is_overcurrent(RIGHT_MOTOR_ID)
        while result_left or result_right:
            if not fault:
                start = time.ticks_ms()
                while (
                    time.ticks_diff(time.ticks_ms(), start)
                    < self.OVERCURRENT_TIMEOUT_MS
                ):
                    result_left = self.is_overcurrent(LEFT_MOTOR_ID)
                    result_right = self.is_overcurrent(RIGHT_MOTOR_ID)
                    if (not result_left) and (not result_right):
                        return
                print("Overcurrent")
                # Overcurrent over the timeout period
                OnBoardLED.set_period_ms(100)
                fault = True
            else:
                time.sleep_ms(self.OVERCURRENT_TIMEOUT_MS)
                result_left = self.is_overcurrent(LEFT_MOTOR_ID)
                result_right = self.is_overcurrent(RIGHT_MOTOR_ID)

            self.drive(0, 0)
            self.central_hub_interface.write(Commands.OVERCURRENT_COMMAND)

        if fault:
            if self.current_mode == self.MIXED:
                OnBoardLED.set_period_ms(2000)
            elif self.current_mode == self.INDEPENDENT_MOTOR:
                OnBoardLED.set_period_ms(1000)

    def send_currents(self):
        """Send the current values to the central hub"""
        currents = (self.left_motor.current, self.right_motor.current)
        out_command = Commands.generate_command(
            (
                Commands.RESP_CURRENTS,
                currents,
            )
        )
        self.central_hub_interface.write(out_command)

    def send_encoders(self):
        """Send the encoder values to the central hub"""
        encoders = (
            self.left_motor.encoder.position(),
            self.right_motor.encoder.position(),
        )
        out_command = Commands.generate_command(
            (
                Commands.RESP_ENCODERS,
                encoders,
            )
        )
        self.central_hub_interface.write(out_command)

    def execute_command(self, command):
        """Parse the command from the central hub. The command can be:

        Parameters
        ----------
        command: bytes
            The command from the central hub

        Returns
        -------
        bool
            True if the command is valid, False otherwise
        """
        valid = False
        parsed = Commands.parse_command(command)
        if parsed is None:
            return False
        else:
            valid = True
            command_type, command_value = parsed
            # print(command_type, command_value)
            if command_type == Commands.SET_SPEED_MIXED:
                self.left_speed_command, self.turn = command_value
                print("Left: {} Turn: {}".format(self.left_speed_command, self.turn))
                self.drive(self.left_speed_command, 0, self.turn)
                self.detect_mode_change()
            elif command_type == Commands.SET_SPEED_LEFT_RIGHT:
                self.turn = None
                # print("command_value", command_value)
                self.left_speed_command = command_value[0]
                self.right_speed_command = command_value[1]
                self.set_pid_setpoint(
                    self.pid_left, self.setpoint_to_rpm(self.left_speed_command)
                )
                self.set_pid_setpoint(
                    self.pid_right, self.setpoint_to_rpm(self.right_speed_command)
                )
                print(
                    "Left: {} Right: {}".format(
                        self.left_speed_command, self.right_speed_command
                    )
                )
                # self.pid_left.setpoint = self.setpoint_to_rpm(self.left_speed_command)
                # self.pid_right.setpoint = self.setpoint_to_rpm(self.right_speed_command)
                # self.pid_left.reset()
                # self.pid_right.reset()
                self.pid_update()
                self.drive(self.left_speed_command, self.right_speed_command)
                self.detect_mode_change()
            elif command_type == Commands.UNKNOWN:
                print("Unknown command")
                valid = False
            else:
                # Ignore other commands
                valid = False

        return valid

    @classmethod
    def set_pid_setpoint(cls, pid, value):
        """Disable PID controller if the value is within the deadzone. Enable otherwise"""
        if abs(value) < cls.PID_DEADZONE:
            pid.setpoint = 0
            pid.auto_mode = False
        else:
            pid.setpoint = value
            pid.auto_mode = True

    def detect_mode_change(self):
        """Allow the class to set the timer period once per mode change"""
        if self._previous_turn is not None and self.turn is None:
            # mode change
            self.current_mode = self.MIXED
            OnBoardLED.set_period_ms(2000)
        elif self._previous_turn is None and self.turn is not None:
            self.current_mode = self.INDEPENDENT_MOTOR
            OnBoardLED.set_period_ms(1000)
        self._previous_turn = self.turn

    @staticmethod
    def rpm_to_setpoint(rpm):
        """Convert a rpm value to a setpoint within the range of -100 to 100"""
        setpoint = (rpm - MotorController.MIN_SPEED_RPM) / (
            MotorController.MAX_SPEED_RPM - MotorController.MIN_SPEED_RPM
        ) * 200 - 100
        return max(min(setpoint, 100), -100)

    @staticmethod
    def setpoint_to_rpm(setpoint):
        """Convert a setpoint value within the range of -100 to 100 to a rpm value"""
        rpm = (setpoint + 100) / 200 * (
            MotorController.MAX_SPEED_RPM - MotorController.MIN_SPEED_RPM
        ) + MotorController.MIN_SPEED_RPM
        return max(
            min(rpm, MotorController.MAX_SPEED_RPM), MotorController.MIN_SPEED_RPM
        )

    def main_control_loop(self):
        """Inteprets the commands from the central hub and controls the motors"""
        OnBoardLED.on()
        prev_sensor_time = time.ticks_ms()

        while True:  # Can be changed to use async to allow other tasks to run
            # The command will latch until a new command is received
            if not CURRENT_SENSOR_ABSENT:
                self.overcurrent_check()
            if (
                time.ticks_diff(time.ticks_ms(), prev_sensor_time)
                > self.SENSOR_UPDATE_PERIOD_MS
            ):
                self.send_currents()
                self.send_encoders()
                prev_sensor_time = time.ticks_ms()
            if self.central_hub_interface.any() != 0:
                command = self.central_hub_interface.read()
                self.execute_command(command)

            if self.turn is not None:
                self.drive(self.left_speed_command, 0, self.turn)
            else:
                # Run the PID control loop until a new command is received
                self.pid_update()

            time.sleep_ms(self.MAIN_LOOP_PERIOD_MS)

    def pid_update(self):
        """Execute one iteration of the PID control loop"""
        current_speed = [self.left_motor.rpm, self.right_motor.rpm]
        left_input_rpm = 0
        right_input_rpm = 0
        if self.pid_left.auto_mode:
            left_input_rpm = self.pid_left(current_speed[0], dt=self.PID_PERIOD_MS)
        if self.pid_right.auto_mode:
            right_input_rpm = self.pid_right(current_speed[1], dt=self.PID_PERIOD_MS)
        # Clip the speed command to the maximum speed
        self.left_speed_command = self.rpm_to_setpoint(left_input_rpm)
        self.right_speed_command = self.rpm_to_setpoint(right_input_rpm)
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
        # self.pid_left.setpoint = self.set_pid_setpoint(left_target_speed, self.PID_DEADZONE)
        # self.pid_right.setpoint = self.set_pid_setpoint(right_target_speed, self.PID_DEADZONE)
        self.set_pid_setpoint(self.pid_left, left_target_speed)
        self.set_pid_setpoint(self.pid_right, right_target_speed)
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
                if rpm_error_threshold > 0 and all(
                    abs(error) < rpm_error_threshold
                    for error in last_error
                    if error is not None
                ):
                    break
                time.sleep_ms(self.PID_PERIOD_MS)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
        finally:
            self.drive(0, 0)


if __name__ == "__main__":
    motor_controller = MotorController()
    motor_controller.main_control_loop()
