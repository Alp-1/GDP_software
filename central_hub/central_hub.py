"""The main central hub code"""

import time
from machine import Pin
from central_hub.pwm import Flight_Controller_Input
from central_hub.receiver import RCReceiver
from central_hub.soft_uart import SoftUART
from protocol import Commands
from user_interface.led import OnBoardLED
from upy_mavlink.mav_bridge import MavLinkBridge

FRONT_WHEEL_CONTROLLER_TX_PIN = 0
FRONT_WHEEL_CONTROLLER_RX_PIN = 1

REAR_WHEEL_CONTROLLER_TX_PIN = 8
REAR_WHEEL_CONTROLLER_RX_PIN = 9

CONTROLLER_BAUDRATE = 9600


class CentralHub:
    """Inteprets data from receiver and transmit speed command accordingly"""

    # Command modes
    DIRECT_RC = 0
    FLIGHT_CONTROLLER = 1
    FULLY_AUTONOMOUS = 2

    # Motor states
    PAUSED = 2
    RUNNING = 1
    FAULT = 0

    # Define the channels function for control
    THROTTLE = RCReceiver.CHANNEL_2
    RUDDER = RCReceiver.CHANNEL_1
    OPERATION_MODE_SELECTOR = RCReceiver.CHANNEL_5
    MOTOR_STATE_SELECTOR = RCReceiver.CHANNEL_7

    # Some common commands for the motor controller
    STOP_COMMAND = Commands.generate_command((Commands.SET_SPEED_LEFT_RIGHT, (0, 0)))

    COMMAND_LOOP_PERIOD_MS = 100
    E_STOP_CHECK_PERIOD_MS = 10

    SABERTOOTH_ESTOP_PIN = 19  # Connect to S2 pin of Sabertooth

    def __init__(
        self,
        front_wheel_controller=None,
        rear_wheel_controller=None,
        rc_receiver=None,
        e_stop_pin=None,
        mavlink_bridge=None,
    ):
        """Initialize the central hub.

        Parameters
        ----------
        front_wheel_controller : UART, optional
            The UART object for talking to the front wheel controller.
        rear_wheel_controller : UART, optional
            The UART object for talking to the rear wheel controller.
        rc_receiver : RCReceiver, optional
            The RCReceiver object for talking to the RC receiver.
        """
        if front_wheel_controller is None:
            self.front_wheel_controller = SoftUART(
                0,
                1,
                baudrate=CONTROLLER_BAUDRATE,
                tx=Pin(FRONT_WHEEL_CONTROLLER_TX_PIN),
                rx=Pin(FRONT_WHEEL_CONTROLLER_RX_PIN),
            )
        else:
            self.front_wheel_controller = front_wheel_controller
        if rear_wheel_controller is None:
            self.rear_wheel_controller = SoftUART(
                2,
                3,
                baudrate=CONTROLLER_BAUDRATE,
                tx=Pin(REAR_WHEEL_CONTROLLER_TX_PIN),
                rx=Pin(REAR_WHEEL_CONTROLLER_RX_PIN),
            )
        else:
            self.rear_wheel_controller = rear_wheel_controller

        if rc_receiver is None:
            self.rc_receiver = RCReceiver(8)
        else:
            self.rc_receiver = rc_receiver

        self.e_stop_pin = (
            Pin(self.SABERTOOTH_ESTOP_PIN, Pin.OUT)
            if e_stop_pin is None
            else e_stop_pin
        )

        self.mavlink_bridge = (
            MavLinkBridge() if mavlink_bridge is None else mavlink_bridge
        )

        self.current_mode = self.DIRECT_RC  # Default
        self.update_mode()
        self.state = self.PAUSED  # The initial state is paused

    def update_mode(self):
        """Detect the mode from the switch position of mode selector switch"""
        mode = self.rc_receiver.channel_data(self.OPERATION_MODE_SELECTOR)
        if mode == 0 and self.current_mode != self.DIRECT_RC:
            self.current_mode = self.DIRECT_RC
            OnBoardLED.set_period_ms(1000)
        elif mode == 1 and self.current_mode != self.FLIGHT_CONTROLLER:
            self.current_mode = self.FLIGHT_CONTROLLER
            OnBoardLED.set_period_ms(2000)
        elif mode == 2 and self.current_mode != self.FULLY_AUTONOMOUS:
            self.current_mode = self.FULLY_AUTONOMOUS
            OnBoardLED.set_period_ms(2000)

    def request_speed_from_flight_controller(self):
        """Request the speed from the flight controller (should be between -100 to 100)"""
        throttle_left, throttle_right = Flight_Controller_Input.speed_request()
        return [throttle_left, throttle_right, throttle_left, throttle_right]

    def relay_currrent_from_motor_controllers(self):
        """Transmit the current readings from the motor controllers to the flight controller"""
        front_left_current, front_right_current = self.read_current(
            self.front_wheel_controller
        )
        rear_left_current, rear_right_current = self.read_current(
            self.rear_wheel_controller
        )
        self.mavlink_bridge.send_current_readings(
            front_left_current,
            front_right_current,
            rear_left_current,
            rear_right_current,
        )

    def read_current(self, controller):
        """Read the current from the motor controller"""
        controller.write(Commands.generate_command((Commands.GET_CURRENTS, None)))
        time.sleep_ms(100)
        if controller.any():
            for command_type, response in Commands.parse_command(controller.read()):
                if command_type == Commands.RESP_CURRENTS:
                    return response
        return 100, 100  # Return a large number if the current cannot be read

    def configure_controllers(self, enable: bool):
        """Send command to enable/disable the motor controllers

        Parameters
        ----------
        enable : bool
            True to enable the motor controllers, False to disable
        """
        for i in range(5):
            # Try to enable/disable the motor controller 5 times
            if enable:
                self.front_wheel_controller.write(Commands.ENABLE_COMMAND)
                self.rear_wheel_controller.write(Commands.ENABLE_COMMAND)
            else:
                self.front_wheel_controller.write(Commands.DISABLE_COMMAND)
                self.rear_wheel_controller.write(Commands.DISABLE_COMMAND)
            try:
                self.handshake_wait()
                break
            except Exception as e:
                print(e)
                print("Trial no. {} failed".format(i))
                time.sleep_ms(1000)
        else:
            raise Exception("Failed to configure motor controller")

    def handshake_wait(self, timeout_ms=5000):
        """Wait for the motor controller to be ready by waiting for confirmation
        from the motor controller with the same command"""
        start_time = time.ticks_ms()
        front_controller_confirmation = False
        rear_controller_confirmation = False

        while not (front_controller_confirmation and rear_controller_confirmation):
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                raise Exception("Motor controller timeout")

            if self.front_wheel_controller.any() and not front_controller_confirmation:
                for command_type, _ in Commands.parse_command(
                    self.front_wheel_controller.read()
                ):
                    if command_type == Commands.ACK:
                        front_controller_confirmation = True
                        print("Front wheel controller ACK")
                        break
                else:
                    raise Exception("Front wheel controller ACK error")

            if self.rear_wheel_controller.any() and not rear_controller_confirmation:
                for command_type, _ in Commands.parse_command(
                    self.rear_wheel_controller.read()
                ):
                    if command_type == Commands.ACK:
                        rear_controller_confirmation = True
                        print("Rear wheel controller ACK")
                        break
                else:
                    raise Exception("Rear wheel controller ACK error")

            time.sleep_ms(100)

    @property
    def state_selector(self):
        """Return the state selector switch info"""
        return self.rc_receiver.channel_data(self.MOTOR_STATE_SELECTOR)

    @property
    def emergency_stop(self):
        """Return if the emergency stop switch is triggered"""
        return self.state_selector == self.FAULT

    def stop_motor(self):
        """Stop the motor by both setting the speed to 0 and pull down the S2 pin"""
        self.e_stop_pin.value(0)
        self.front_wheel_controller.write(self.STOP_COMMAND)
        self.rear_wheel_controller.write(self.STOP_COMMAND)

    def fault_check(self):
        """Stop the motor and set the current mode to FAULT if the emergency stop switch is triggered"""
        if self.emergency_stop:
            self.stop_motor()
            return True
        self.e_stop_pin.value(1)  # Release the emergency stop if it is not triggered
        return False

    def update_command(self):
        """This method update checks the current mode and
        updates speed command to the controllers.
        Call this method in a loop to update the speed command periodically.
        """
        if self.state == self.FAULT:
            # Should not update command if the state is FAULT
            return
        self.update_mode()

        if self.current_mode == self.DIRECT_RC:
            # Inverted as the throttle is negative when pushed forward
            speed = -self.rc_receiver.channel_data(self.THROTTLE)
            # Rudder is inverted as the hardware is wired in the opposite direction
            turn = -self.rc_receiver.channel_data(self.RUDDER)
            print(self.current_mode, speed, turn)
            command = Commands.generate_command(
                (Commands.SET_SPEED_MIXED, (speed, turn))
            )
            self.front_wheel_controller.write(command)
            self.rear_wheel_controller.write(command)

        elif (
            self.current_mode == self.FLIGHT_CONTROLLER
            or self.current_mode == self.FULLY_AUTONOMOUS
        ):
            command = self.request_speed_from_flight_controller()
            command_front = Commands.generate_command(
                (Commands.SET_SPEED_LEFT_RIGHT, (command[0], command[1]))
            )
            command_rear = Commands.generate_command(
                (Commands.SET_SPEED_LEFT_RIGHT, (command[2], command[3]))
            )
            print(self.current_mode, command_front, command_rear)
            self.front_wheel_controller.write(command_front)
            self.rear_wheel_controller.write(command_rear)

    def update_state(self):
        """This method updates the state of the central hub."""

        if self.fault_check():
            if self.state != self.FAULT:
                OnBoardLED.set_period_ms(100)
            self.state = self.FAULT
            return

        if self.state == self.PAUSED and self.state_selector == self.RUNNING:
            OnBoardLED.set_period_ms(1000)
            self.configure_controllers(enable=True)
            self.state = self.RUNNING
        elif self.state == self.RUNNING and self.state_selector == self.PAUSED:
            OnBoardLED.set_period_ms(500)
            self.configure_controllers(enable=False)
            self.state = self.PAUSED
        elif self.state == self.FAULT:
            # Clear the FAULT state since the emergency stop is released
            self.state = self.state_selector

    def command_loop(self):
        """The main command loop. Call this method to update the speed command periodically."""
        last_time = time.ticks_ms()
        while self.state_selector != self.PAUSED:
            OnBoardLED.on()
            # prevents unwanted movement when the switch is not
            # in the disabled state when starting.
            pass
        OnBoardLED.set_period_ms(500)
        while True:
            self.update_state()
            if (
                self.state == self.RUNNING
                and time.ticks_diff(time.ticks_ms(), last_time)
                > self.COMMAND_LOOP_PERIOD_MS
            ):
                last_time = time.ticks_ms()
                self.update_command()

            # State machine is run more frequently than the update command
            time.sleep_ms(self.E_STOP_CHECK_PERIOD_MS)
