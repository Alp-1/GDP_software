"""Convert the signals from RC into logic levels for the rest of the system"""

# from central_hub.picosbus import PicoSBUS
from central_hub.ppm_reader import PpmReader

RC_RECEIVER_PIN = 18


class RCReceiver:
    """Interface with the RC receiver"""

    ANALOGUE = 0
    THREE_WAY_SWITCH = 1
    BUTTON = 2

    CHANNEL_1 = 0
    CHANNEL_2 = 1
    CHANNEL_3 = 2
    CHANNEL_4 = 3
    CHANNEL_5 = 4
    CHANNEL_6 = 5
    CHANNEL_7 = 6
    CHANNEL_8 = 7

    def __init__(self, no_of_channels: int, rc_interface=None):
        """Initialise and assign the type of channels of the controller.

        Parameters
        ----------
        no_of_channels : int
            The number of channels of the RC controller
        rc_interface : PpmReader, optional
            The interface to the RC receiver, use PpmReader by default
        """
        self._channels_type = [0] * no_of_channels  # List of channels type

        # Refer to the actual RC controller
        for i in range(no_of_channels):
            if (
                i == self.CHANNEL_1
                or i == self.CHANNEL_2
                or i == self.CHANNEL_3
                or i == self.CHANNEL_4
                or i == self.CHANNEL_8
            ):
                self._channels_type[i] = self.ANALOGUE
            elif i == self.CHANNEL_5 or i == self.CHANNEL_7:
                self._channels_type[i] = self.THREE_WAY_SWITCH
            elif i == self.CHANNEL_6:
                self._channels_type[i] = self.BUTTON
            else:
                raise Exception("Invalid channel number {}".format(i + 1))

        # The actual interface to the RC receiver
        if rc_interface is None:
            self.rc_interface = PpmReader(RC_RECEIVER_PIN, no_of_channels)
        else:
            self.rc_interface = rc_interface
        self.rc_range = self.rc_interface.max_value - self.rc_interface.min_value

    def channel_data(self, channel: int) -> int:
        """Get the data from the channel"""
        if self._channels_type[channel] == self.ANALOGUE:
            return self.bi_analogue_channel_data(channel)
        elif self._channels_type[channel] == self.THREE_WAY_SWITCH:
            return self.three_way_switch_channel_data(channel)
        elif self._channels_type[channel] == self.BUTTON:
            return self.button_channel_data(channel)
        else:
            raise Exception("Invalid channel number {}".format(channel + 1))

    def bi_analogue_channel_data(self, channel: int) -> int:
        """Get the bidirectional data from the analogue channel. Range from -100 to 100"""
        bi_value = self.rc_interface.get_value_bi(channel) * 100
        return max(min(bi_value, 100), -100)  # Limit the value to between -100 and 100

    def three_way_switch_channel_data(self, channel: int) -> int:
        """Get the data from the three way switch channel"""
        BANDS = 2
        raw_value = self.get_raw_rc_data(channel)
        # Expect division of 1000-1500-2000
        interval = self.rc_range // BANDS
        # n = (raw_value - self.rc_interface.min_value) // interval
        n = 0
        while raw_value > (self.rc_interface.min_value + interval // 2):
            n += 1
            raw_value -= interval
        return n

    def button_channel_data(self, channel: int) -> int:
        """Get the data from the button channel"""
        raw_value = self.get_raw_rc_data(channel)
        if raw_value > (self.rc_interface.min_value + self.rc_range // 2):
            return 1
        else:
            return 0

    def get_raw_rc_data(self, channel: int) -> int:
        """Get the raw data from the RC receiver"""
        return self.rc_interface.get_raw_value(channel)
