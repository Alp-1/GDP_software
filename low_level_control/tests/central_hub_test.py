"""Test the handshake and protocols in central_hub"""

from unittest.mock import patch, Mock
import struct
import pytest


with patch.dict(
    "sys.modules",
    {
        "machine": Mock(),
        "ustruct": struct,
        "time": Mock(),
        "rp2": Mock(),
        "upy_mavlink.mav_bridge": Mock(),
    },
):
    from protocol import Commands
    from central_hub.central_hub import CentralHub


@pytest.fixture
def central_hub():
    """Create a CentralHub object"""
    uart_front = Mock()
    uart_rear = Mock()
    rc_receiver = Mock()
    return CentralHub(
        controllers={
            "front": uart_front,
            "rear": uart_rear,
        },
        rc_receiver=rc_receiver,
    )


@pytest.mark.parametrize(
    "mode, commands, expected_front, expected_rear",
    [
        (
            CentralHub.DIRECT_RC,
            [80, 50],
            Commands.generate_command(
                (
                    Commands.SET_SPEED_MIXED,
                    (
                        (-1 if CentralHub.REVERSE_THROTTLE else 1) * 80,
                        (-1 if CentralHub.REVERSE_RUDDER else 1) * 50,
                    ),
                )
            ),
            Commands.generate_command(
                (
                    Commands.SET_SPEED_MIXED,
                    (
                        (-1 if CentralHub.REVERSE_THROTTLE else 1) * 80,
                        (-1 if CentralHub.REVERSE_RUDDER else 1) * 50,
                    ),
                )
            ),
        ),
        (
            CentralHub.FLIGHT_CONTROLLER,
            [70, 50, 80, 50],
            Commands.generate_command(
                (
                    Commands.SET_SPEED_LEFT_RIGHT,
                    (
                        (-1 if CentralHub.REVERSE_SERVO else 1) * 70,
                        (-1 if CentralHub.REVERSE_SERVO else 1) * 50,
                    ),
                )
            ),
            Commands.generate_command(
                (
                    Commands.SET_SPEED_LEFT_RIGHT,
                    (
                        (-1 if CentralHub.REVERSE_SERVO else 1) * 80,
                        (-1 if CentralHub.REVERSE_SERVO else 1) * 50,
                    ),
                )
            ),
        ),
        (
            CentralHub.FULLY_AUTONOMOUS,
            [-50, -60, -30, -40],
            Commands.generate_command(
                (
                    Commands.SET_SPEED_LEFT_RIGHT,
                    (
                        (-1 if CentralHub.REVERSE_SERVO else 1) * -50,
                        (-1 if CentralHub.REVERSE_SERVO else 1) * -60,
                    ),
                )
            ),
            Commands.generate_command(
                (
                    Commands.SET_SPEED_LEFT_RIGHT,
                    (
                        (-1 if CentralHub.REVERSE_SERVO else 1) * -30,
                        (-1 if CentralHub.REVERSE_SERVO else 1) * -40,
                    ),
                )
            ),
        ),
    ],
)
def test_send_command(central_hub, mode, commands, expected_front, expected_rear):
    """Test if the command written to the controllers is correct using hard coded values of the receiver output"""

    central_hub.update_mode = Mock()
    central_hub.update_mode.return_value = None
    central_hub.current_mode = mode

    def average_filter(window_size, data_fn, *args):
        """Remove the average filter from this test"""
        return data_fn(*args)
    central_hub.average_filter = average_filter


    if mode == CentralHub.DIRECT_RC:
        central_hub.rc_receiver.channel_data.side_effect = commands
    elif mode == CentralHub.FLIGHT_CONTROLLER or mode == CentralHub.FULLY_AUTONOMOUS:
        central_hub.request_speed_from_flight_controller = Mock()
        central_hub.request_speed_from_flight_controller.return_value = commands
    else:
        raise Exception("Unexpected mode for this test")

    central_hub.send_command()

    if mode in [
        CentralHub.DIRECT_RC,
        CentralHub.FLIGHT_CONTROLLER,
        CentralHub.FULLY_AUTONOMOUS,
    ]:
        # In DIRECT_RC mode, the command should be the same as the RC receiver
        central_hub.controllers["front"].write.assert_called_with(expected_front)
        central_hub.controllers["rear"].write.assert_called_with(expected_rear)
    else:
        raise Exception("Unexpected mode for this test")
