from motor_controller.sabertooth import Sabertooth
import pytest
from unittest.mock import Mock, call


@pytest.fixture
def sabertooth():
    """Create a Sabertooth object"""
    # Create a mock UART object
    uart = Mock()
    return Sabertooth(uart, address=128)


@pytest.mark.parametrize("baudrate", [1000, 2400, 9600, 19200, 38400, 57600])
def test_set_baudrate(baudrate, sabertooth):
    """Test the set_baudrate method"""

    # Map the baudrate to the correct value

    if baudrate < 2400 or baudrate > 38400:
        with pytest.raises(
            Exception, match="Sabertooth, invalid baudrate {}".format(baudrate)
        ):
            sabertooth.set_baudrate(baudrate)
        return

    sabertooth.set_baudrate(baudrate)

    # Check that the correct command was sent
    valid = {
        2400: 1,
        9600: 2,
        19200: 3,
        38400: 4,
    }
    baudrate_code = valid[baudrate]
    checksum = (128 + 15 + baudrate_code) & 127
    sabertooth.saber.write.assert_called_once_with(
        bytes([128, 15, baudrate_code, checksum])
    )
    assert sabertooth.saber.write.call_count == 1


@pytest.mark.parametrize("motor_id", [1, 2, 3])
@pytest.mark.parametrize("speed", [-150, -100, -50, 0, 50, 100, 150])
def test_drive(motor_id, speed, sabertooth):
    """Test the drive method. The command are recalculated in order to cross-check the
    checksum calculation
    """
    if abs(speed) > 100:
        with pytest.raises(
            Exception, match="Sabertooth, invalid speed {}".format(speed)
        ):
            sabertooth.drive(motor_id, speed)
        return

    # Check that the correct command was sent
    if motor_id == 1:
        if speed < 0:
            command = sabertooth.REVERSE_1
        else:
            command = sabertooth.FORWARD_1
    elif motor_id == 2:
        if speed < 0:
            command = sabertooth.REVERSE_2
        else:
            command = sabertooth.FORWARD_2
    else:
        with pytest.raises(
            Exception, match="Sabertooth, invalid motor {}".format(motor_id)
        ):
            sabertooth.drive(motor_id, speed)
        return

    sabertooth.drive(motor_id, speed)
    value = abs(round(speed * 1.27))
    checksum = (128 + command + value) & 127
    sabertooth.saber.write.assert_called_once_with(
        bytes([128, command, value, checksum])
    )
    assert sabertooth.saber.write.call_count == 1


@pytest.mark.parametrize("speed", [-150, -100, -50, 0, 50, 100, 150])
@pytest.mark.parametrize("turn", [-150, -100, -50, 0, 50, 100, 150])
def test_drive_both(speed, turn, sabertooth):
    """Test the drive_both method"""
    if abs(speed) > 100:
        with pytest.raises(
            Exception, match="Sabertooth, invalid speed {}".format(speed)
        ):
            sabertooth.drive_both(speed, turn)
        return
    if abs(turn) > 100:
        with pytest.raises(Exception, match="Sabertooth, invalid turn {}".format(turn)):
            sabertooth.drive_both(speed, turn)
        return

    # Check that the correct command was sent
    if speed < 0:
        drive_command = sabertooth.REVERSE_MIXED
    else:
        drive_command = sabertooth.FORWARD_MIXED

    if turn < 0:
        turn_command = sabertooth.LEFT_MIXED
    else:
        turn_command = sabertooth.RIGHT_MIXED

    sabertooth.drive_both(speed, turn)
    drive_value = abs(round(speed * 1.27))
    turn_value = abs(round(turn * 1.27))
    drive_checksum = (128 + drive_command + drive_value) & 127
    turn_checksum = (128 + turn_command + turn_value) & 127
    sabertooth.saber.write.assert_has_calls(
        [
            call(bytes([128, drive_command, drive_value, drive_checksum])),
            call(bytes([128, turn_command, turn_value, turn_checksum])),
        ]
    )

    assert sabertooth.saber.write.call_count == 2
