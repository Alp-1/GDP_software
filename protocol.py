"""Define the communication protocol between central pico and motor controllers"""

import ustruct

HEADER = b"\x4A\xA5\xFA"
FOOTER = b"\xBA\xA6\xAA"


class Commands:
    """Store a list of commands that can be sent to the motor controllers"""

    # One-byte commnad types
    ENABLE = b"\x80"
    DISABLE_COMMAND = b"\x82"
    ACK = b"\x83"
    GET_CURRENTS = b"\x01"
    GET_ENCODERS = b"\x02"  # Unused
    SET_SPEED_MIXED = b"\x10"
    SET_SPEED_LEFT_RIGHT = b"\x11"
    RESP_CURRENTS = b"\x21"

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
                yield ("enable", None)
            elif command_type == Commands.DISABLE_COMMAND:
                yield ("disable", None)
            elif command_type == Commands.ACK:
                yield ("ack", None)
            elif command_type == Commands.GET_CURRENTS:
                yield ("get_currents", None)
            elif command_type == Commands.GET_ENCODERS:
                yield ("get_encoders", None)
            elif command_type == Commands.SET_SPEED_MIXED:
                speed_command, turn = ustruct.unpack("ff", current_command[1:])
                yield ("set_speed_mixed", (speed_command, turn))
            elif command_type == Commands.SET_SPEED_LEFT_RIGHT:
                left, right = ustruct.unpack("ff", current_command[1:])
                yield ("set_speed_left_right", (left, right))
            elif command_type == Commands.RESP_CURRENTS:
                left, right = ustruct.unpack("ff", current_command[1:])
                yield ("resp_currents", (left, right))
            else:
                yield ("unknown", None)

    @staticmethod
    def generate_command(command: tuple):
        """Send a command to the motor controllers"""
        command_type = command[0]

        if command_type == "enable":
            return bytearray(HEADER + Commands.ENABLE + FOOTER)
        elif command_type == "disable":
            return bytearray(HEADER + Commands.DISABLE_COMMAND + FOOTER)
        elif command_type == "ack":
            return bytearray(HEADER + Commands.ACK + FOOTER)
        elif command_type == "get_currents":
            return bytearray(HEADER + Commands.GET_CURRENTS + FOOTER)
        elif command_type == "get_encoders":
            return bytearray(HEADER + Commands.GET_ENCODERS + FOOTER)
        elif command_type == "set_speed_mixed":
            speed_command, turn = command[1]
            return bytearray(
                HEADER
                + Commands.SET_SPEED_MIXED
                + ustruct.pack("ff", speed_command, turn)
                + FOOTER
            )
        elif command_type == "set_speed_left_right":
            left, right = command[1]
            return bytearray(
                HEADER
                + Commands.SET_SPEED_LEFT_RIGHT
                + ustruct.pack("ff", left, right)
                + FOOTER
            )
        elif command_type == "resp_currents":
            left, right = command[1]
            return bytearray(
                HEADER
                + Commands.RESP_CURRENTS
                + ustruct.pack("ff", left, right)
                + FOOTER
            )
        else:
            return bytearray()
