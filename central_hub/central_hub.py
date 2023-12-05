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
    REVERSE_THROTTLE = False
    REVERSE_RUDDER = True

    # Some common commands for the motor controller
    STOP_COMMAND = Commands.generate_command((Commands.SET_SPEED_LEFT_RIGHT, (0, 0)))

    COMMAND_LOOP_PERIOD_MS = 100
    E_STOP_CHECK_PERIOD_MS = 10

    SABERTOOTH_ESTOP_PIN = 22  # Connect to S2 pin of Sabertooth

    # The number of missing messages before resetting the state machine
    SOFT_UART_RESET_COUNT = 50

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
            self.rc_receiver = RCReceiver(8)
        else:
            self.rc_receiver = rc_receiver

        self.e_stop_pin = (
            Pin(self.SABERTOOTH_ESTOP_PIN, Pin.OUT)
            if e_stop_pin is None
            else e_stop_pin
        )

        self.current_mode = self.DIRECT_RC  # Default
        self.update_mode()
        self.state = self.PAUSED  # The initial state is paused
        self._prev_state = self.PAUSED
        self.armed = False

        self.controller_ack = {"front": False, "rear": False}

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

    @property
    def emergency_stop(self):
        """Return if the emergency stop switch is triggered"""
        return self.state_selector == self.FAULT

    def stop_motor(self):
        """Stop the motor by both setting the speed to 0 and pull down the S2 pin"""
        print("Motor stopped")
        self.e_stop_pin.value(1)
        for controller in self.controllers.values():
            controller.write(self.STOP_COMMAND)

    # fmt: off
    async def parse_command(self):
        """Parse the input bytes coming from the motor controller pico"""
        (
            front_left_encoder,
            front_right_encoder,
            rear_left_encoder,
            rear_right_encoder,
        ) = (None, None, None, None)
        (
            front_left_current,
            front_right_current,
            rear_left_current,
            rear_right_current,
        ) = (None, None, None, None)
        front_miss_count = 0
        rear_miss_count = 0
        while True:
            self._prev_state = self.state
            self.state = self.state_selector
            if self.controllers["front"].any():
                front_miss_count = 0
                received = self.controllers["front"].read()
                parsed = Commands.parse_command(received, 0)
                if parsed is not None:
                    print("front_parsed", parsed)
                    command_type, response = parsed
                    if command_type == Commands.RESP_ENCODERS:
                        front_left_encoder, front_right_encoder = response
                    elif command_type == Commands.RESP_CURRENTS:
                        front_left_current, front_right_current = response
                    elif command_type == Commands.ACK:
                        self.controller_ack["front"] = True
                    elif command_type == Commands.OVERCURRENT:
                        self.state = self.FAULT
                        self.stop_motor()
                    else:
                        print("Unwanted command type", command_type)
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
                    print("rear_parsed", parsed)
                    command_type, response = parsed
                    if command_type == Commands.RESP_ENCODERS:
                        rear_left_encoder, rear_right_encoder = response
                    elif command_type == Commands.RESP_CURRENTS:
                        rear_left_current, rear_right_current = response
                    elif command_type == Commands.ACK:
                        self.controller_ack["rear"] = True
                    elif command_type == Commands.OVERCURRENT:
                        self.state = self.FAULT
                        self.stop_motor()
                    else:
                        print("Unwanted command type", command_type)
            else:
                rear_miss_count += 1
                if rear_miss_count > self.SOFT_UART_RESET_COUNT:
                    self.controllers["rear"].reset_sm()
                    rear_miss_count = 0

            if self.armed:
                self.state_action()

            if (
                front_left_encoder is not None
                and front_right_encoder is not None
            ):
                await mav_bridge.send_name_value_floats("a", front_left_encoder)
                await mav_bridge.send_name_value_floats("b", front_right_encoder)
                await mav_bridge.send_name_value_floats("a", front_left_encoder)
                await mav_bridge.send_name_value_floats("b", front_right_encoder)
                front_left_encoder, front_right_encoder = None, None
            if (
                rear_left_encoder is not None
                and rear_right_encoder is not None
            ):
                await mav_bridge.send_name_value_floats("c", rear_left_encoder)
                await mav_bridge.send_name_value_floats("d", rear_right_encoder)
                await mav_bridge.send_name_value_floats("c", rear_left_encoder)
                await mav_bridge.send_name_value_floats("d", rear_right_encoder)
                rear_left_encoder, rear_right_encoder = None, None

            if (
                front_left_current is not None
                and front_right_current is not None
            ):
                await mav_bridge.send_name_value_floats("e", front_left_current)
                await mav_bridge.send_name_value_floats("f", front_right_current)
                await mav_bridge.send_name_value_floats("e", front_left_current)
                await mav_bridge.send_name_value_floats("f", front_right_current)
                front_left_current, front_right_current = None, None

            if (
                rear_left_current is not None
                and rear_right_current is not None
            ):
                await mav_bridge.send_name_value_floats("g", rear_left_current)
                await mav_bridge.send_name_value_floats("h", rear_right_current)
                await mav_bridge.send_name_value_floats("g", rear_left_current)
                await mav_bridge.send_name_value_floats("h", rear_right_current)
                rear_left_current, rear_right_current = None, None
            await asyncio.sleep_ms(10)
    # fmt: on

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
            speed = self.rc_receiver.channel_data(self.THROTTLE) * (
                -1 if self.REVERSE_THROTTLE else 1
            )
            turn = self.rc_receiver.channel_data(self.RUDDER) * (
                -1 if self.REVERSE_RUDDER else 1
            )
            print(self.current_mode, speed, turn)
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
            command_front = Commands.generate_command(
                (Commands.SET_SPEED_LEFT_RIGHT, (command[0], command[1]))
            )
            command_rear = Commands.generate_command(
                (Commands.SET_SPEED_LEFT_RIGHT, (command[2], command[3]))
            )
            print(self.current_mode, command[0], command[1])
            self.controllers["front"].write(command_front)
            self.controllers["rear"].write(command_rear)

    def state_action(self):
        """Configure the user interface according to the state of the system"""

        if self.state == self.PAUSED and self._prev_state != self.PAUSED:
            OnBoardLED.set_period_ms(500)
            # Release the emergency stop if it is not triggered
            self.e_stop_pin.value(0)
        elif self.state == self.RUNNING and self._prev_state != self.RUNNING:
            OnBoardLED.set_period_ms(1000)
            # Release the emergency stop if it is not triggered
            self.e_stop_pin.value(0)
        elif self.state == self.FAULT and self._prev_state != self.FAULT:
            OnBoardLED.set_period_ms(100)
            self.stop_motor()

    async def command_loop(self):
        """The main command loop. Call this method to update the speed command periodically."""
        last_time = time.ticks_ms()
        # "Pre-arm" check
        while self.state_selector != self.PAUSED:
            await asyncio.sleep_ms(500)
            OnBoardLED.on()
            # prevents unwanted movement when the switch is not
            # in the disabled state when starting.
            pass
        OnBoardLED.set_period_ms(500)
        self.armed = True
        while True:
            if (
                self.state == self.RUNNING
                and time.ticks_diff(time.ticks_ms(), last_time)
                > self.COMMAND_LOOP_PERIOD_MS
            ):
                last_time = time.ticks_ms()
                self.update_command()

            # State machine is run more frequently than the update command
            await asyncio.sleep_ms(self.E_STOP_CHECK_PERIOD_MS)


async def main():
    """The main loop to run the central hub"""
    central_hub = CentralHub()
    asyncio.create_task(central_hub.command_loop())
    asyncio.create_task(mav_bridge.mavlink_main())
    asyncio.create_task(central_hub.parse_command())
    while True:
        await asyncio.sleep_ms(10)
