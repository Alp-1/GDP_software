"""Test the handshake and protocols in central_hub"""

from unittest.mock import patch, Mock
import struct
import pytest


with patch.dict("sys.modules", machine=Mock(), ustruct=struct, time=Mock()):
    import time
    from central_hub.central_hub import CentralHub


@pytest.fixture
def central_hub():
    """Create a CentralHub object"""
    uart_front = Mock()
    uart_rear = Mock()
    rc_receiver = Mock()
    return CentralHub(
        front_wheel_controller=uart_front,
        rear_wheel_controller=uart_rear,
        rc_receiver=rc_receiver,
    )


@pytest.mark.parametrize(
    "mode, commands, expected",
    [
        (CentralHub.DIRECT_RC, [80, 50], [-80, -80, 50]),
        (CentralHub.FLIGHT_CONTROLLER, [70, 50, 80, 50], ([70, 50], [80, 50])),
        (CentralHub.FULLY_AUTONOMOUS, [-50, -60, -30, -40], ([-50, -60], [-30, -40])),
    ],
)
def test_update_command(central_hub, mode, commands, expected):
    """Test if the update_command method writes the correct command depends on the mode"""

    central_hub.update_mode = Mock()
    central_hub.update_mode.return_value = None
    central_hub.current_mode = mode

    if mode == CentralHub.DIRECT_RC:
        central_hub.rc_receiver.channel_data.side_effect = commands
    elif mode == CentralHub.FLIGHT_CONTROLLER or mode == CentralHub.FULLY_AUTONOMOUS:
        central_hub.request_speed_from_flight_controller = Mock()
        central_hub.request_speed_from_flight_controller.return_value = commands
    else:
        raise Exception("Unexpected mode for this test")

    central_hub.update_command()

    if mode == CentralHub.DIRECT_RC:
        # In DIRECT_RC mode, the command should be the same as the RC receiver
        central_hub.front_wheel_controller.write.assert_called_with(
            struct.pack("fff", *expected)
        )
        central_hub.rear_wheel_controller.write.assert_called_with(
            struct.pack("fff", *expected)
        )
    elif mode == CentralHub.FLIGHT_CONTROLLER or mode == CentralHub.FULLY_AUTONOMOUS:
        # In FLIGHT_CONTROLLER mode, the command should be the same as the flight controller
        central_hub.front_wheel_controller.write.assert_called_with(
            struct.pack("ff", *expected[0])
        )
        central_hub.rear_wheel_controller.write.assert_called_with(
            struct.pack("ff", *expected[1])
        )
    else:
        raise Exception("Unexpected mode for this test")


def test_handshake_wait(central_hub):
    """Test the handshake_wait method with random data"""

    # Timeout is irrelevant here as we are confirmed to have a mocked reply
    time.ticks_ms.return_value = 0
    time.ticks_diff.return_value = 0

    # The handshake_command is offset by 1
    handshake_command = bytes([1, 2, 3, 4, 5])
    mock_read_data = bytes([0, *handshake_command]) * 10
    iter_count = 0

    def mock_read(num_bytes=0):
        """Mock the read method by yielding from mock_read_data"""
        nonlocal iter_count
        if num_bytes == 0:
            num_bytes = len(mock_read_data)
        result = mock_read_data[iter_count : iter_count + num_bytes]
        iter_count += num_bytes
        return result

    # ignore the front wheel controller
    central_hub.front_wheel_controller.any.return_value = True
    central_hub.front_wheel_controller.read.side_effect = [handshake_command]*10

    # mock the rear wheel controller
    central_hub.rear_wheel_controller.any.return_value = True
    central_hub.rear_wheel_controller.read.side_effect = mock_read

    try:
        central_hub.handshake_wait(handshake_command)
    except Exception as e:
        pytest.fail("handshake_wait raised an exception: {}".format(e))


def test_fault_transition(central_hub):
    """Test the fault transition from RUNNING and PAUSED state"""

    central_hub.state = CentralHub.RUNNING
    central_hub.fault_check = Mock()
    central_hub.fault_check.return_value = True
    central_hub.update_state()
    assert central_hub.state == CentralHub.FAULT

    central_hub.state = CentralHub.PAUSED
    central_hub.update_state()
    assert central_hub.state == CentralHub.FAULT


@pytest.mark.parametrize(
    "state_switch",
    [
        (CentralHub.RUNNING),
        (CentralHub.PAUSED),
    ],
)
def test_fault_clear(central_hub, state_switch, mocker):
    """Test the fault clear transition from FAULT state"""

    # The emergency stop is not triggered in normal operation
    mocker.patch(
        __name__ + ".CentralHub.state_selector",
        new_callable=mocker.PropertyMock,
        return_value=state_switch,
    )

    central_hub.state = CentralHub.FAULT
    central_hub.fault_check = Mock()
    central_hub.fault_check.return_value = False
    central_hub.update_state()
    assert central_hub.state == state_switch


@pytest.mark.parametrize(
    "initial_state, switch_sequence",
    [
        (CentralHub.RUNNING, [CentralHub.PAUSED, CentralHub.RUNNING]),
        (CentralHub.RUNNING, [CentralHub.RUNNING, CentralHub.PAUSED]),
        (CentralHub.RUNNING, [CentralHub.RUNNING, CentralHub.RUNNING]),
        (CentralHub.RUNNING, [CentralHub.PAUSED, CentralHub.PAUSED]),
        (CentralHub.PAUSED, [CentralHub.RUNNING, CentralHub.PAUSED]),
        (CentralHub.PAUSED, [CentralHub.PAUSED, CentralHub.RUNNING]),
        (CentralHub.PAUSED, [CentralHub.PAUSED, CentralHub.PAUSED]),
        (CentralHub.PAUSED, [CentralHub.RUNNING, CentralHub.RUNNING]),
    ],
)
def test_state_machine_transition(central_hub, initial_state, switch_sequence, mocker):
    """Test the state machine transitions"""

    central_hub.enable_controllers = Mock()
    central_hub.disable_controllers = Mock()
    central_hub.fault_check = Mock()
    central_hub.fault_check.return_value = False

    def reset_mocks():
        """Reset the mocks"""
        central_hub.enable_controllers.reset_mock()
        central_hub.disable_controllers.reset_mock()

    mocker.patch(
        __name__ + ".CentralHub.state_selector",
        new_callable=mocker.PropertyMock,
        side_effect=switch_sequence,
    )

    previous_switch_state = switch_sequence[0]
    central_hub.state = initial_state
    previous_state = central_hub.state

    for current_switch_state in switch_sequence:
        central_hub.update_state()
        match (previous_state, previous_switch_state, current_switch_state):
            case (CentralHub.RUNNING, CentralHub.PAUSED, CentralHub.PAUSED):
                central_hub.enable_controllers.assert_not_called()
                central_hub.disable_controllers.assert_called_once()
                assert central_hub.state == CentralHub.PAUSED
            case (CentralHub.RUNNING, CentralHub.PAUSED, CentralHub.RUNNING):
                central_hub.enable_controllers.assert_called_once()
                central_hub.disable_controllers.assert_not_called()
                assert central_hub.state == CentralHub.RUNNING
            case (CentralHub.RUNNING, CentralHub.RUNNING, CentralHub.PAUSED):
                central_hub.enable_controllers.assert_not_called()
                central_hub.disable_controllers.assert_called_once()
                assert central_hub.state == CentralHub.PAUSED
            case (CentralHub.RUNNING, CentralHub.RUNNING, CentralHub.RUNNING):
                central_hub.enable_controllers.assert_not_called()
                central_hub.disable_controllers.assert_not_called()
                assert central_hub.state == CentralHub.RUNNING
            case (CentralHub.PAUSED, CentralHub.PAUSED, CentralHub.PAUSED):
                central_hub.enable_controllers.assert_not_called()
                central_hub.disable_controllers.assert_not_called()
                assert central_hub.state == CentralHub.PAUSED
            case (CentralHub.PAUSED, CentralHub.PAUSED, CentralHub.RUNNING):
                central_hub.enable_controllers.assert_called_once()
                central_hub.disable_controllers.assert_not_called()
                assert central_hub.state == CentralHub.RUNNING
            case (CentralHub.PAUSED, CentralHub.RUNNING, CentralHub.PAUSED):
                central_hub.enable_controllers.assert_not_called()
                central_hub.disable_controllers.assert_called_once()
                assert central_hub.state == CentralHub.PAUSED
            case (CentralHub.PAUSED, CentralHub.RUNNING, CentralHub.RUNNING):
                central_hub.enable_controllers.assert_called_once()
                central_hub.disable_controllers.assert_not_called()
                assert central_hub.state == CentralHub.RUNNING
            case _:
                raise Exception("Unexpected state for this test")

        previous_switch_state = current_switch_state
        previous_state = central_hub.state
        reset_mocks()
