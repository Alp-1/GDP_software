"""Test the motor controller module. 
Mainly the logic to interact with the Sabertooth motor driver
and the central hub.
"""

import pytest
import struct
from unittest.mock import Mock, patch, call

with patch.dict(
    "sys.modules",
    machine=Mock(),
    ustruct=struct,
    rp2=Mock(),
    time=Mock(),
):
    from motor_controller.motor_controller import MotorController
    import motor_controller.motor_controller as motor_controller_module


@pytest.fixture
def motor_controller():
    """Create a MotorController object"""
    # Create mock objects
    motor_driver = Mock()
    left_motor = Mock()
    right_motor = Mock()
    central_hub_interface = Mock()
    motor_driver_serial = Mock()
    mock_motor_controller = MotorController(
        motor_driver_serial=motor_driver_serial,
        central_hub_interface=central_hub_interface,
        left_motor=left_motor,
        right_motor=right_motor,
        motor_driver=motor_driver,
    )
    # mock_motor_controller.drive = Mock()
    return mock_motor_controller


def test_drive(motor_controller):
    """Test the drive method"""
    # Mock the moving average filter
    motor_controller.moving_average = Mock()
    motor_controller.moving_average.side_effect = lambda x: x

    motor_controller.drive(1, 2, 3)
    motor_controller.motor_driver.drive_both.assert_called_once_with(1, 3)
    assert motor_controller.motor_driver.drive_both.call_count == 1

    motor_controller.drive(4, 5)
    motor_controller.motor_driver.drive.assert_has_calls(
        [
            call(motor_controller_module.LEFT_MOTOR_ID, 4),
            call(motor_controller_module.RIGHT_MOTOR_ID, 5),
        ]
    )

    assert motor_controller.motor_driver.drive.call_count == 2


@pytest.mark.parametrize(
    "command",
    [
        (MotorController.ENABLE_COMMAND),
        (MotorController.DISABLE_COMMAND),
        ((10, 0, -50)),
        ((80, -20)),
        # Invalid commands
        (b""),  # Empty
        (b"\x80\x24"),  # Single int (invalid)
        (b"\x80\x24\x21\x80\x24\x21\x22\x23\x24\x25\x26\x27\x28\x29\x30"),  # Too long
    ],
)
def test_parse_command(motor_controller: MotorController, command):
    """Test the parse_command method"""

    # We are not testing these components here
    motor_controller.drive = Mock()
    motor_controller.pid_left = Mock()
    motor_controller.pid_right = Mock()
    motor_controller.pid_update = Mock()
    motor_controller.rpm_to_setpoint = Mock()

    byte_command = (
        # b"".join([to_short_bytes(x) for x in command])
        struct.pack("f" * len(command), *command)
        if isinstance(command, tuple)
        else command
    )
    valid = motor_controller.parse_command(byte_command)

    match command:
        case MotorController.ENABLE_COMMAND:
            motor_controller.central_hub_interface.write.assert_called_once_with(
                MotorController.ENABLE_COMMAND
            )
            assert motor_controller.central_hub_interface.write.call_count == 1
            assert motor_controller.drive.call_count == 0
            assert motor_controller.ENABLED == True
            assert valid == True
        case MotorController.DISABLE_COMMAND:
            motor_controller.central_hub_interface.write.assert_called_once_with(
                MotorController.DISABLE_COMMAND
            )
            assert motor_controller.central_hub_interface.write.call_count == 1
            assert motor_controller.drive.call_count == 0
            assert motor_controller.ENABLED == False
            assert valid == True
        case (left_speed, right_speed):
            motor_controller.drive.assert_called_once_with(left_speed, right_speed)
            assert motor_controller.drive.call_count == 1
            assert motor_controller.central_hub_interface.write.call_count == 0
            assert valid == True
        case (left_speed, right_speed, turn):
            motor_controller.drive.assert_called_once_with(
                left_speed, right_speed, turn
            )
            assert motor_controller.drive.call_count == 1
            assert motor_controller.central_hub_interface.write.call_count == 0
            assert valid == True
        case _:
            assert valid == False


@pytest.mark.parametrize(
    "input_rpm, expected_setpoint",
    [
        (0, 0),
        (MotorController.MAX_SPEED_RPM, 100),
        (MotorController.MIN_SPEED_RPM, -100),
        (MotorController.MAX_SPEED_RPM / 2, 50),
        (MotorController.MIN_SPEED_RPM / 2, -50),
    ],
)
def test_rpm_to_setpoint(input_rpm, expected_setpoint):
    """Test the rpm_to_setpoint method"""
    assert MotorController.rpm_to_setpoint(input_rpm) == expected_setpoint


@pytest.mark.parametrize(
    "motor_id",
    [motor_controller_module.LEFT_MOTOR_ID, motor_controller_module.RIGHT_MOTOR_ID],
)
@pytest.mark.parametrize(
    "current_left",
    [
        0,
        MotorController.THRESHOLD_CURRENT,
        MotorController.THRESHOLD_CURRENT + 0.2,
        -MotorController.THRESHOLD_CURRENT,
        -MotorController.THRESHOLD_CURRENT - 0.2,
    ],
)
@pytest.mark.parametrize(
    "current_right",
    [
        0,
        MotorController.THRESHOLD_CURRENT,
        MotorController.THRESHOLD_CURRENT + 0.2,
        -MotorController.THRESHOLD_CURRENT,
        -MotorController.THRESHOLD_CURRENT - 0.2,
    ],
)
def test_is_overcurrent(
    motor_controller,
    motor_id,
    current_left,
    current_right,
):
    """Test the over current detection"""
    motor_controller.left_motor.current = current_left
    motor_controller.right_motor.current = current_right
    expected_result = False
    if (
        motor_id == motor_controller_module.LEFT_MOTOR_ID
        and (
            current_left > MotorController.THRESHOLD_CURRENT
            or current_left < -MotorController.THRESHOLD_CURRENT
        )
    ) or (
        motor_id == motor_controller_module.RIGHT_MOTOR_ID
        and (
            current_right > MotorController.THRESHOLD_CURRENT
            or current_right < -MotorController.THRESHOLD_CURRENT
        )
    ):
        expected_result = True
    assert motor_controller.is_overcurrent(motor_id) == expected_result
