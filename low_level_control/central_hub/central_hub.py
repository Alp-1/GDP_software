"""The main central hub code"""

import time
import asyncio
from machine import Pin
from central_hub.pwm import Flight_Controller_Input
from central_hub.receiver import RCReceiver
from central_hub.soft_uart import SoftUART
from protocol import Commands
from user_interface.led import OnBoardLED
import upy_mavlink.mav_bridge as mav_bridge

FRONT_WHEEL_CONTROLLER_TX_PIN = 0
FRONT_WHEEL_CONTROLLER_RX_PIN = 1

REAR_WHEEL_CONTROLLER_TX_PIN = 8
REAR_WHEEL_CONTROLLER_RX_PIN = 9

CONTROLLER_BAUDRATE = 4800


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

    # Whether to reverse the channel input (Depends on the hardware wiring)
    REVERSE_THROTTLE = True
    REVERSE_RUDDER = False
    REVERSE_SERVO = False  # Use in Indepedent mode

    # Some common commands for the motor controller
    STOP_COMMAND = Commands.generate_command((Commands.SET_SPEED_LEFT_RIGHT, (0, 0)))

    COMMAND_LOOP_PERIOD_MS = 100
    E_STOP_CHECK_PERIOD_MS = 10

    SABERTOOTH_ESTOP_PIN = 22  # Connect to S2 pin of Sabertooth

    # The number of missing messages before resetting the state machine
    #  (This is to prevent the SoftUART state machine from getting stuck)
    SOFT_UART_RESET_COUNT = 50

    RC_FILTER_WINDOW_SIZE = 3
    # Each PPM packet is 20ms (Standard RC receiver)
    RC_FILTER_TIME_MS = 20

    RC_TIMEOUT_MS = 3000

    def __init__(
        self,
        controllers=None,
        rc_receiver=None,
        e_stop_pin=None,
    ):
        """Initialize the central hub.

        Parameters
        ----------
        controllers: A dict with {"front": front_wheel_controller, "rear": rear_wheel_controller}
         where each controller is a UART object for talking to the respective wheel controller.
        rc_receiver : RCReceiver, optional
            The RCReceiver object for talking to the RC receiver.
        """
        if controllers is None:
            self.controllers = {
                "front": SoftUART(
                    0,
                    1,
                    baudrate=CONTROLLER_BAUDRATE,
                    tx=Pin(FRONT_WHEEL_CONTROLLER_TX_PIN),
                    rx=Pin(FRONT_WHEEL_CONTROLLER_RX_PIN),
                ),
                "rear": SoftUART(
                    2,
                    3,
                    baudrate=CONTROLLER_BAUDRATE,
                    tx=Pin(REAR_WHEEL_CONTROLLER_TX_PIN),
                    rx=Pin(REAR_WHEEL_CONTROLLER_RX_PIN),
                ),
            }
        else:
            self.controllers = controllers

        if rc_receiver is None:
            self.rc_receiver = RCReceiver(8, rc_timeout_ms=self.RC_TIMEOUT_MS)
        else:
            self.rc_receiver = rc_receiver

        self.e_stop_pin = (
            Pin(self.SABERTOOTH_ESTOP_PIN, Pin.OUT)
            if e_stop_pin is None
            else e_stop_pin
        )

        self.current_mode = self.DIRECT_RC  # Default
        self.state = self.PAUSED  # The initial state is paused
        self.armed = False

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

    @property
    def state_selector(self):
        """Return the state selector switch info"""
        return self.rc_receiver.channel_data(self.MOTOR_STATE_SELECTOR)

    def stop_motor(self):
        """Stop the motor by both setting the speed to 0 and pull down the S2 pin"""
        print("Motor stopped")
        self.e_stop_pin.value(1)
        for controller in self.controllers.values():
            controller.write(self.STOP_COMMAND)

    async def command_action(self, position, command_type, command_payload):
        """Execute the command from the RC receiver

        Parameters
        ----------
        position : str
            The position of the controller. Either "front" or "rear"
        command_type : int
            The command type
        command_payload : Any
            The command payload
        """
        if position == "front":
            left_encoder_name = "a"
            right_encoder_name = "b"
            left_current_name = "e"
            right_current_name = "f"
        elif position == "rear":
            left_encoder_name = "c"
            right_encoder_name = "d"
            left_current_name = "g"
            right_current_name = "h"
        else:
            raise Exception("Invalid position for the controller")
        # fmt: off
        if command_type == Commands.RESP_ENCODERS:
            await mav_bridge.send_name_value_floats(left_encoder_name, command_payload[0])
            await mav_bridge.send_name_value_floats(right_encoder_name, command_payload[1])
            await mav_bridge.send_name_value_floats(left_encoder_name, command_payload[0])
            await mav_bridge.send_name_value_floats(right_encoder_name, command_payload[1])
        elif command_type == Commands.RESP_CURRENTS:
            await mav_bridge.send_name_value_floats(left_current_name, command_payload[0])
            await mav_bridge.send_name_value_floats(right_current_name, command_payload[1])
            await mav_bridge.send_name_value_floats(left_current_name, command_payload[0])
            await mav_bridge.send_name_value_floats(right_current_name, command_payload[1])
        # fmt: on
        elif command_type == Commands.OVERCURRENT:
            self.state = self.FAULT
            self.stop_motor()
        else:
            print("Unwanted command type", command_type)
            return False
        return True

    async def parse_controllers_input(self):
        """Parse the input bytes coming from the motor controller pico"""
        front_miss_count = 0
        rear_miss_count = 0
        while True:
            if self.controllers["front"].any():
                front_miss_count = 0
                received = self.controllers["front"].read()
                parsed = Commands.parse_command(received, 0)
                if parsed is not None:
                    # print("front_parsed", parsed)
                    command_type, response = parsed
                    await self.command_action("front", command_type, response)
            else:
                front_miss_count += 1
                if front_miss_count > self.SOFT_UART_RESET_COUNT:
                    self.controllers["front"].reset_sm()
                    front_miss_count = 0
            if self.controllers["rear"].any():
                rear_miss_count = 0
                received = self.controllers["rear"].read()
                parsed = Commands.parse_command(received, 1)
                if parsed is not None:
                    # print("rear_parsed", parsed)
                    command_type, response = parsed
                    await self.command_action("rear", command_type, response)
            else:
                rear_miss_count += 1
                if rear_miss_count > self.SOFT_UART_RESET_COUNT:
                    self.controllers["rear"].reset_sm()
                    rear_miss_count = 0

            await asyncio.sleep_ms(10)

    async def update_state(self):
        """Constantly checking the state of the system on the state selector switch"""
        while True:
            if self.state != self.state_selector:
                # Only update the state if the state selector switch is changed
                self.state = self.state_selector
                if self.armed:
                    self.state_action()
            await asyncio.sleep_ms(self.E_STOP_CHECK_PERIOD_MS)

    def average_filter(self, window_size, data_fn, *args):
        """Average filter for the data_fn that returns a fixed number of values"""
        avg = data_fn(*args)
        if not isinstance(avg, list):
            num_values = 1
        else:
            num_values = len(avg)
        for _ in range(window_size):
            if num_values == 1:
                avg += data_fn(*args)
            else:
                avg = [avg[i] + data_fn(*args)[i] for i in range(num_values)]
            time.sleep_ms(self.RC_FILTER_TIME_MS)
        return [x / window_size for x in avg] if num_values > 1 else avg / window_size

    def send_command(self):
        """This method reads from the receiver and sends the
        speed command to the controllers depending on the mode.
        Call this method in a loop to update the speed command periodically.
        """
        if self.state == self.FAULT:
            # Should not update command if the state is FAULT
            return

        if self.current_mode == self.DIRECT_RC:
            speed = self.average_filter(
                self.RC_FILTER_WINDOW_SIZE, self.rc_receiver.channel_data, self.THROTTLE
            ) * (-1 if self.REVERSE_THROTTLE else 1)
            turn = self.average_filter(
                self.RC_FILTER_WINDOW_SIZE, self.rc_receiver.channel_data, self.RUDDER
            ) * (-1 if self.REVERSE_RUDDER else 1)
            print("Mode: Direct RC, Speed: {:.2f}, Turn: {:.2f}".format(speed, turn))
            command = Commands.generate_command(
                (Commands.SET_SPEED_MIXED, (speed, turn))
            )
            for controller in self.controllers.values():
                controller.write(command)

        elif (
            self.current_mode == self.FLIGHT_CONTROLLER
            or self.current_mode == self.FULLY_AUTONOMOUS
        ):
            command = self.request_speed_from_flight_controller()
            if self.REVERSE_SERVO:
                command = [-x for x in command]
            command_front = Commands.generate_command(
                (Commands.SET_SPEED_LEFT_RIGHT, (command[0], command[1]))
            )
            command_rear = Commands.generate_command(
                (Commands.SET_SPEED_LEFT_RIGHT, (command[2], command[3]))
            )
            print("Mode: FC, Speed: {:.2f}, {:.2f}".format(command[0], command[1]))
            self.controllers["front"].write(command_front)
            self.controllers["rear"].write(command_rear)

    def state_action(self):
        """Configure the user interface according to the state of the system"""

        if self.state == self.PAUSED:
            OnBoardLED.set_period_ms(500)
            # Release the emergency stop if it is not triggered
            self.e_stop_pin.value(0)
        elif self.state == self.RUNNING:
            OnBoardLED.set_period_ms(1000)
            # Release the emergency stop if it is not triggered
            self.e_stop_pin.value(0)
        elif self.state == self.FAULT:
            OnBoardLED.set_period_ms(100)
            self.stop_motor()

    async def command_loop(self):
        """The main command loop. Call this method to update the speed command periodically."""
        # "Pre-arm" check
        while self.state_selector != self.PAUSED:
            await asyncio.sleep_ms(self.COMMAND_LOOP_PERIOD_MS)
            OnBoardLED.on()
            # prevents unwanted movement when the switch is not
            # in the disabled state when starting.
        self.armed = True
        OnBoardLED.set_period_ms(500)  # Exits the pre-arm check
        while True:
            if self.state == self.RUNNING:
                self.update_mode()
                self.send_command()

            await asyncio.sleep_ms(self.COMMAND_LOOP_PERIOD_MS)


async def main():
    """The main loop to run the central hub"""
    central_hub = CentralHub()
    asyncio.create_task(central_hub.command_loop())
    asyncio.create_task(mav_bridge.mavlink_main())
    asyncio.create_task(central_hub.parse_controllers_input())
    asyncio.create_task(central_hub.update_state())
    while True:
        await asyncio.sleep_ms(10)
