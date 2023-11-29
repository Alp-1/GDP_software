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
    SET_SPEED_MIXED = b"\x10"
    SET_SPEED_LEFT_RIGHT = b"\x11"
    RESP_CURRENTS = b"\x21"
    RESP_ENCODERS = b"\x22"
    OVERCURRENT = b"\x23"
    UNKNOWN = b"\xFF"

    buffer = [bytearray(), bytearray()]
    MAX_BUFFER_SIZE = 16

    def __init__(self):
        pass

    @staticmethod
    def accumulate_command(new_data, id):
        """Try to find the header and footer of a command in a string"""

        Commands.buffer[id] += new_data
        # print("buffer ", id, Commands.buffer[id])
        header_idx = Commands.buffer[id].find(HEADER)
        if header_idx == -1:
            Commands.buffer[id] = bytearray()
            return None
        else:
            Commands.buffer[id] = Commands.buffer[id][header_idx:]

        footer_idx = Commands.buffer[id].find(FOOTER)
        if footer_idx != -1:
            command = Commands.buffer[id][: footer_idx + len(FOOTER)]
            Commands.buffer[id] = Commands.buffer[id][footer_idx + len(FOOTER) :]
            return command
        else:
            if len(Commands.buffer[id]) > Commands.MAX_BUFFER_SIZE:
                Commands.buffer[id] = bytearray()
            return None

    @staticmethod
    def parse_command(command, id=0):
        """Parse a command and return the next command in the form (command_type, command_data)"""

        command = Commands.accumulate_command(command, id)
        if command is None:
            return None  # No valid command found, wait for more data
        # print("protocol", command)
        # Trim the header and footer
        trimmed_command = command[len(HEADER) : -len(FOOTER)]
        command_type = bytes([trimmed_command[0]])

        try:
            # Check if command is valid
            if command_type == Commands.ENABLE:
                return (Commands.ENABLE, None)
            elif command_type == Commands.DISABLE:
                return (Commands.DISABLE, None)
            elif command_type == Commands.ACK:
                return (Commands.ACK, None)
            elif command_type == Commands.SET_SPEED_MIXED:
                speed_command, turn = ustruct.unpack("ff", trimmed_command[1:])
                return (Commands.SET_SPEED_MIXED, (speed_command, turn))
            elif command_type == Commands.SET_SPEED_LEFT_RIGHT:
                left, right = ustruct.unpack("ff", trimmed_command[1:])
                return (Commands.SET_SPEED_LEFT_RIGHT, (left, right))
            elif command_type == Commands.RESP_CURRENTS:
                left, right = ustruct.unpack("ff", trimmed_command[1:])
                return (Commands.RESP_CURRENTS, (left, right))
            elif command_type == Commands.RESP_ENCODERS:
                left, right = ustruct.unpack("ii", trimmed_command[1:])
                return (Commands.RESP_ENCODERS, (left, right))
            elif command_type == Commands.OVERCURRENT:
                return (Commands.OVERCURRENT, None)
            else:
                return (Commands.UNKNOWN, None)
        except ValueError:
            print("ValueError for type", command_type)
            print("trimmed_command", trimmed_command)
            return (Commands.UNKNOWN, None)

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
        elif command_type == Commands.RESP_ENCODERS:
            left, right = command[1]
            return bytearray(
                HEADER
                + Commands.RESP_ENCODERS
                + ustruct.pack("ii", left, right)
                + FOOTER
            )
        elif command_type == Commands.OVERCURRENT:
            return bytearray(HEADER + Commands.OVERCURRENT + FOOTER)
        else:
            return bytearray()


# Commonly used commands
Commands.ACK_COMMAND = Commands.generate_command((Commands.ACK, None))
Commands.ENABLE_COMMAND = Commands.generate_command((Commands.ENABLE, None))
Commands.DISABLE_COMMAND = Commands.generate_command((Commands.DISABLE, None))
Commands.OVERCURRENT_COMMAND = Commands.generate_command((Commands.OVERCURRENT, None))
