"""Provides methods to communicate with Sabertooth 2x12 motor driver"""


class Sabertooth:
    """
    A class to control the Sabertooth 2x12 using packetized serial mode (DIP switch 1,2 low)

    https://www.dimensionengineering.com/datasheets/Sabertooth2x12.pdf
    """

    FORWARD_1 = 0x00
    REVERSE_1 = 0x01
    FORWARD_2 = 0x04
    REVERSE_2 = 0x05
    FORWARD_MIXED = 0x08
    REVERSE_MIXED = 0x09
    RIGHT_MIXED = 0x0A
    LEFT_MIXED = 0x0B

    def __init__(self, uart, address: int):
        """
        Parameters
        ----------
        uart: UART object
            The UART object to use to communicate with the Sabertooth
        address: int
            The address of the Sabertooth. Must be between 128 and 135 (Configurable using DIP switches)
        """
        self.saber = uart
        self.address = address

        if 128 > self.address > 135:
            raise Exception("Sabertooth, invalid address: {}".format(address))

    def set_baudrate(self, baudrate):
        """
        Sets the baudrate to: 2400, 9600, 19200, 38400
        """
        valid = {
            2400: 1,
            9600: 2,
            19200: 3,
            38400: 4,
        }

        if baudrate in valid:
            baud = valid[baudrate]
        else:
            raise Exception("Sabertooth, invalid baudrate {}".format(baudrate))

        # command = 15
        self.send_command(15, baud)

    def drive(self, motor_id: int, speed: float):
        """Drive a single motor at a given speed

        Parameters
        ----------
        motor_id: int
            The motor to drive. 1 or 2
        speed: float
            The speed to drive the motor at (-100: full reverse, 0: stop, 100: full forward)
        """
        speed = min(max(speed, -100), 100)

        if motor_id == 1:
            if speed < 0:
                command = self.REVERSE_1
                speed = -speed
            else:
                command = self.FORWARD_1
        elif motor_id == 2:
            if speed < 0:
                command = self.REVERSE_2
                speed = -speed
            else:
                command = self.FORWARD_2
        else:
            raise Exception("Sabertooth, invalid motor {}".format(motor_id))

        value = round(speed * 1.27)
        self.send_command(command, value)

    def drive_both(self, speed: float, turn: float):
        """
        Drive both motors at a given speed and turn

        Parameters
        ----------
        speed: float
            The speed to drive the motors at (-100: full reverse, 0: stop, 100: full forward)
        turn: float
            The turn value (-100: full left, 0: straight, 100: full right)
        """
        speed = min(max(speed, -100), 100)
        turn = min(max(turn, -100), 100)

        if speed < 0:
            drive_command = self.REVERSE_MIXED
            speed = -speed
        else:
            drive_command = self.FORWARD_MIXED

        if turn < 0:
            turn_command = self.LEFT_MIXED
            turn = -turn
        else:
            turn_command = self.RIGHT_MIXED

        # Note: it is not necessary to set both the speed and turn values if you are changing only one
        self.send_command(drive_command, round(speed * 1.27))
        self.send_command(turn_command, round(turn * 1.27))

    def send_command(self, command: int, value: int):
        """Send a packetized serial command to the Sabertooth

        Parameters
        ----------
        command: Command to send.
                FORWARD_1 = 0x00
                REVERSE_1 = 0x01
                FORWARD_2 = 0x04
                REVERSE_2 = 0x05
                FORWARD_MIXED = 0x08
                REVERSE_MIXED = 0x09
                RIGHT_MIXED = 0x0A
                LEFT_MIXED = 0x0B
                RAMP = 0x10
        value: Command value. 0-127

        """
        checksum = (self.address + command + value) & 127
        msg = bytes([self.address, command, value, checksum])
        self.saber.write(msg)
