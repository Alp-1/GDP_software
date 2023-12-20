"""The main motor controller code"""

import time
from machine import Pin, UART
from motor_controller.motor import MotorSensing
from motor_controller.sabertooth import Sabertooth
from motor_controller.pid import PID
from protocol import Commands
from user_interface.led import OnBoardLED

LEFT_MOTOR_ID = 2
LEFT_MOTOR_ENCODER_PIN = 12
LEFT_MOTOR_SM = 0
LEFT_MOTOR_CURRENT_PIN = 1

RIGHT_MOTOR_ID = 1
RIGHT_MOTOR_ENCODER_PIN = 14
RIGHT_MOTOR_SM = 1
RIGHT_MOTOR_CURRENT_PIN = 2

CENTRAL_HUB_TX_PIN = 4
CENTRAL_HUB_RX_PIN = 5

SABERTOOTH_ADDRESS = 128
SABERTOOTH_BAUDRATE = 9600
SABERTOOTH_TX_PIN = 0
SABERTOOTH_RX_PIN = 1  # Irrelevant


class MotorController:
    """Controls the motor speed using the signals coming from the central hub"""

    THRESHOLD_CURRENT = 15  # Amps
    # Need to be at least twice the encoders' measurement time
    PID_PERIOD_MS = 200
    MAIN_LOOP_PERIOD_MS = 1
    SENSOR_UPDATE_PERIOD_MS = 1000
    # Period before the central hub is considered disconnected
    CENTRAL_HUB_TIMEOUT_MS = 1000

    # Speed setpoint range
    MAX_SPEED_RPM = 165
    MIN_SPEED_RPM = -165
    # The maximum range of the PID controller
    MAX_PID_RANGE = (MIN_SPEED_RPM, MAX_SPEED_RPM)
    PID_DEADZONE = 5

    # Attribute to reverse the output of the motor
    # to compensate for the motor being mounted in reverse
    REVERSE_OUTPUT = True

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
                invert_current=True,
            )
            if left_motor is None
            else left_motor
        )
        self.right_motor = (
            MotorSensing(
                RIGHT_MOTOR_ENCODER_PIN,
                RIGHT_MOTOR_SM,
                RIGHT_MOTOR_CURRENT_PIN,
                invert_current=True,
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
        self.mixed_clipped_range = (-100, 100)

        self.current_mode = self.MIXED

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
        if self.REVERSE_OUTPUT:
            left_speed = -left_speed
            right_speed = -right_speed
            turn = -turn if turn is not None else None

        if turn is not None:
            left_speed = self.convert(
                left_speed,
                -100,
                100,
                self.mixed_clipped_range[0],
                self.mixed_clipped_range[1],
            )
            turn = self.convert(
                turn,
                -100,
                100,
                self.mixed_clipped_range[0],
                self.mixed_clipped_range[1],
            )
            self.motor_driver.drive_both(left_speed, turn)
        else:
            self.motor_driver.drive(LEFT_MOTOR_ID, left_speed)
            self.motor_driver.drive(RIGHT_MOTOR_ID, right_speed)

    def overcurrent_protection(self):
        """Reacts to overcurrent depending on the mode.

        If the current mode is mixed, the direct output will be clipped (both turn and speed)
        If the current mode is individual, the pid output will be reduced
        """
        if self.current_mode == self.MIXED:
            prev_range = self.mixed_clipped_range[1] - self.mixed_clipped_range[0]
            left_clipped_range = self.current_to_output_map(
                self.left_motor.current, prev_range=prev_range
            )
            right_clipped_range = self.current_to_output_map(
                self.right_motor.current, prev_range=prev_range
            )
            self.mixed_clipped_range = (
                max(left_clipped_range[0], right_clipped_range[0]),
                min(left_clipped_range[1], right_clipped_range[1]),
            )
            return self.mixed_clipped_range
        elif self.current_mode == self.INDEPENDENT_MOTOR:
            max_range = self.MAX_PID_RANGE[1] - self.MAX_PID_RANGE[0]
            left_prev_range = (
                self.pid_left.output_limits[1] - self.pid_left.output_limits[0]
            )
            right_prev_range = (
                self.pid_right.output_limits[1] - self.pid_right.output_limits[0]
            )
            self.pid_left.output_limits = self.current_to_output_map(
                self.left_motor.current,
                prev_range=left_prev_range,
                max_range=max_range,
            )
            self.pid_right.output_limits = self.current_to_output_map(
                self.right_motor.current,
                prev_range=right_prev_range,
                max_range=max_range,
            )
            return self.pid_left.output_limits, self.pid_right.output_limits

    def current_to_output_map(self, current, prev_range, max_range=200):
        """Return the clipped output range of the controller"""
        # No output cap for current less than 20% of the threshold (3A for 15A threshold)
        if abs(current) < 0.2 * self.THRESHOLD_CURRENT and prev_range == max_range:
            return (-max_range / 2, max_range / 2)
        # Generate a range from 0 to 1
        normalised_clipped_range = (
            self.THRESHOLD_CURRENT - abs(current)
        ) / self.THRESHOLD_CURRENT
        # Prevent negative values
        normalised_clipped_range = max(normalised_clipped_range, 0)
        # Slowly increase the range to prevent aggressive oscillation
        clipped_range = normalised_clipped_range * (prev_range + 10) * 1.2
        clipped_range = min(clipped_range, max_range)
        return (-clipped_range / 2, clipped_range / 2)  # Assume the output is symmetric

    @staticmethod
    def convert(x, in_min, in_max, out_min, out_max):
        # Will return a float
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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
                print(
                    "Left: {:.2f} Turn: {:.2f}".format(
                        self.left_speed_command, self.turn
                    )
                )
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
                    "Left: {:.2f} Right: {:.2f}".format(
                        self.left_speed_command, self.right_speed_command
                    )
                )
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
        if self.current_mode != self.MIXED and self.turn is not None:
            # mode change
            self.current_mode = self.MIXED
            OnBoardLED.set_period_ms(2000)
        elif self.current_mode != self.INDEPENDENT_MOTOR and self.turn is None:
            self.current_mode = self.INDEPENDENT_MOTOR
            OnBoardLED.set_period_ms(1000)

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

    def central_hub_timeout_handler(self):
        """Stop the motors when the central hub timeout"""
        self.left_speed_command = 0
        self.right_speed_command = 0
        if self.current_mode == self.MIXED:
            self.turn = 0
            self.drive(self.left_speed_command, 0, self.turn)
        elif self.current_mode == self.INDEPENDENT_MOTOR:
            self.set_pid_setpoint(self.pid_left, self.left_speed_command)
            self.set_pid_setpoint(self.pid_right, self.right_speed_command)
            self.pid_update()
        else:
            print("Unknown mode")

    def main_control_loop(self):
        """Inteprets the commands from the central hub and controls the motors"""
        OnBoardLED.on()
        prev_sensor_time = time.ticks_ms()
        last_command_time = time.ticks_ms()

        while True:  # Can be changed to use async to allow other tasks to run
            # The command will latch until a new command is received
            self.overcurrent_protection()
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
                last_command_time = time.ticks_ms()
            else:
                # Try to parse the remaining data in the buffer
                self.execute_command(b"")
                # Check if the central hub is disconnected
                if (
                    time.ticks_diff(time.ticks_ms(), last_command_time)
                    > self.CENTRAL_HUB_TIMEOUT_MS
                ):
                    self.central_hub_timeout_handler()

            if self.current_mode == self.MIXED:
                self.drive(self.left_speed_command, 0, self.turn)
            elif self.current_mode == self.INDEPENDENT_MOTOR:
                # Run the PID control loop until a new command is received
                self.pid_update()

            time.sleep_ms(self.MAIN_LOOP_PERIOD_MS)

    def pid_update(self):
        """Execute one iteration of the PID control loop"""
        current_speed = [self.left_motor.rpm, self.right_motor.rpm]
        left_input_rpm = 0
        right_input_rpm = 0
        if self.pid_left.auto_mode:
            left_input_rpm = self.pid_left(current_speed[0])
        if self.pid_right.auto_mode:
            right_input_rpm = self.pid_right(current_speed[1])
        # Clip the speed command to the maximum speed
        self.left_speed_command = self.rpm_to_setpoint(left_input_rpm)
        self.right_speed_command = self.rpm_to_setpoint(right_input_rpm)
        self.drive(self.left_speed_command, self.right_speed_command)
        return self.left_speed_command, self.right_speed_command

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
        self.current_mode = self.INDEPENDENT_MOTOR
        last_time = time.ticks_ms()
        self.set_pid_setpoint(self.pid_left, left_target_speed)
        self.set_pid_setpoint(self.pid_right, right_target_speed)
        iteration = 0
        try:
            while time.ticks_diff(time.ticks_ms(), last_time) < timeout:
                iteration += 1
                start = time.ticks_us()
                print("Limits: ", self.overcurrent_protection())
                self.pid_update()
                last_error = (self.pid_left._last_error, self.pid_right._last_error)
                print(
                    "Iteration {}: {} us".format(
                        iteration, time.ticks_diff(time.ticks_us(), start)
                    )
                )
                print("Last error at iteration {}: {}".format(iteration, last_error))
                print(
                    "Drive: {:.2f} {:.2f}".format(
                        self.left_speed_command, self.right_speed_command
                    )
                )
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
