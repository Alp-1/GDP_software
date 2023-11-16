"""Define the communication protocol between central pico and motor controllers"""

import ustruct

HEADER = b"\x4A\xA5\xFA"
FOOTER = b"\xBA\xA6\xAA"


class Commands:
    """Store a list of commands that can be sent to the motor controllers"""

    # One-byte command types
    ENABLE = b"\x80"
    DISABLE = b"\x82"
    ACK = b"\x83"
    GET_CURRENTS = b"\x01"
    GET_ENCODERS = b"\x02"  # Unused
    SET_SPEED_MIXED = b"\x10"
    SET_SPEED_LEFT_RIGHT = b"\x11"
    RESP_CURRENTS = b"\x21"
    UNKNOWN = b"\xFF"

    def __init__(self):
        pass

    @staticmethod
    def parse_command(command):
        """Parse a command into list of tuples of the form (command_type, command_data)"""

        while HEADER in command:
            # Extract content between header and footer
            command = command[command.index(HEADER) + len(HEADER) :]
            current_command = command[: command.index(FOOTER)]
            print(current_command)
            command = command[len(current_command) + len(FOOTER) :]
            command_type = bytes([current_command[0]])

            # Check if command is valid
            if command_type == Commands.ENABLE:
                yield (Commands.ENABLE, None)
            elif command_type == Commands.DISABLE:
                yield (Commands.DISABLE, None)
            elif command_type == Commands.ACK:
                yield (Commands.ACK, None)
            elif command_type == Commands.GET_CURRENTS:
                yield (Commands.GET_CURRENTS, None)
            elif command_type == Commands.GET_ENCODERS:
                yield (Commands.GET_ENCODERS, None)
            elif command_type == Commands.SET_SPEED_MIXED:
                speed_command, turn = ustruct.unpack("ff", current_command[1:])
                yield (Commands.SET_SPEED_MIXED, (speed_command, turn))
            elif command_type == Commands.SET_SPEED_LEFT_RIGHT:
                left, right = ustruct.unpack("ff", current_command[1:])
                yield (Commands.SET_SPEED_LEFT_RIGHT, (left, right))
            elif command_type == Commands.RESP_CURRENTS:
                left, right = ustruct.unpack("ff", current_command[1:])
                yield (Commands.RESP_CURRENTS, (left, right))
            else:
                yield (Commands.UNKNOWN, None)

    @staticmethod
    def generate_command(command: tuple):
        """Send a command to the motor controllers"""
        command_type = command[0]

        if command_type == Commands.ENABLE:
            return bytearray(HEADER + Commands.ENABLE + FOOTER)
        elif command_type == Commands.DISABLE:
            return bytearray(HEADER + Commands.DISABLE + FOOTER)
        elif command_type == Commands.ACK:
            return bytearray(HEADER + Commands.ACK + FOOTER)
        elif command_type == Commands.GET_CURRENTS:
            return bytearray(HEADER + Commands.GET_CURRENTS + FOOTER)
        elif command_type == Commands.GET_ENCODERS:
            return bytearray(HEADER + Commands.GET_ENCODERS + FOOTER)
        elif command_type == Commands.SET_SPEED_MIXED:
            speed_command, turn = command[1]
            return bytearray(
                HEADER
                + Commands.SET_SPEED_MIXED
                + ustruct.pack("ff", speed_command, turn)
                + FOOTER
            )
        elif command_type == Commands.SET_SPEED_LEFT_RIGHT:
            left, right = command[1]
            return bytearray(
                HEADER
                + Commands.SET_SPEED_LEFT_RIGHT
                + ustruct.pack("ff", left, right)
                + FOOTER
            )
        elif command_type == Commands.RESP_CURRENTS:
            left, right = command[1]
            return bytearray(
                HEADER
                + Commands.RESP_CURRENTS
                + ustruct.pack("ff", left, right)
                + FOOTER
            )
        else:
            return bytearray()


# Commonly used commands
Commands.ACK_COMMAND = Commands.generate_command((Commands.ACK, None))
Commands.ENABLE_COMMAND = Commands.generate_command((Commands.ENABLE, None))
Commands.DISABLE_COMMAND = Commands.generate_command((Commands.DISABLE, None))
