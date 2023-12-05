"""Tests for the logic of the RC receiver"""

import pytest
from unittest.mock import Mock, patch

with patch.dict("sys.modules", machine=Mock()):
    from central_hub.receiver import RCReceiver


@pytest.fixture
def rc_receiver():
    """Create an RCReceiver object"""
    # Create a mock PpmReader object
    ppm_reader = Mock()
    ppm_reader.max_value = 2000
    ppm_reader.min_value = 1000
    return RCReceiver(8, rc_interface=ppm_reader)


@pytest.mark.parametrize(
    "raw_data, expected",
    [
        (950, 0),
        (1000, 0),
        (1250, 0),
        (1251, 1),
        (1500, 1),
        (1750, 1),
        (1751, 2),
        (2000, 2),
        (2050, 2),
    ],
)
def test_three_way_switch_channel_data(rc_receiver, raw_data, expected):
    """Test the three_way_switch_channel_data method"""
    rc_receiver.rc_interface.get_raw_value.return_value = raw_data
    assert rc_receiver.three_way_switch_channel_data(rc_receiver.CHANNEL_5) == expected


@pytest.mark.parametrize(
    "raw_data, expected",
    [
        (950, 0),
        (1000, 0),
        (1500, 0),
        (1501, 1),
        (2000, 1),
        (2050, 1),
    ],
)
def test_button_channel_data(rc_receiver, raw_data, expected):
    """Test the button_channel_data method"""
    rc_receiver.rc_interface.get_raw_value.return_value = raw_data
    assert rc_receiver.button_channel_data(rc_receiver.CHANNEL_6) == expected


@pytest.mark.parametrize(
    "bi_data, expected",
    [
        (-1.1, -100),
        (-1, -100),
        (-0.858, -85.8),
        (-0.5562, -55.62),
        (0, 0),
        (0.5, 50),
        (0.8, 80),
        (1, 100),
        (1.1, 100),
    ],
)
def test_bi_analogue_channel_data(rc_receiver, bi_data, expected):
    """Test the bi_analogue_channel_data method"""
    rc_receiver.rc_interface.get_value_bi.return_value = bi_data
    assert rc_receiver.bi_analogue_channel_data(rc_receiver.CHANNEL_1) == pytest.approx(
        expected
    )
