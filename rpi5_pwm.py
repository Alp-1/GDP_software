"""
Wrapper around rpi-hardware-pwm to control servos.

The kernel PWM service needs to be started before use.

To set up PWM on GPIO 18/19 
    dtoverlay=pwm-2chan in config.txt 
    sudo dtoverlay pwm-2chan on command line

To set up PWM on GPIO 12/13
    dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
    sudo dtoverlay pwm-2chan  pin=12 func=4 pin2=13 func2=4

There is a service pwm to set up a single channel.
"""


from rpi_hardware_pwm import HardwarePWM

class Servo:
    """Only supports pin 12 and 13 for now. Only allows pulse between 1ms and 2ms."""
    def __init__(self, pin=None, *, initial_value=0.0, deadband=0.05):
        min_pulse_width=1/1000
        max_pulse_width=2/1000
        frame_width=20/1000
        if min_pulse_width >= max_pulse_width:
            raise ValueError('min_pulse_width must be less than max_pulse_width')
        if max_pulse_width >= frame_width:
            raise ValueError('max_pulse_width must be less than frame_width')
        self._frame_width = frame_width
        self._min_dc = min_pulse_width / frame_width
        self._dc_range = (max_pulse_width - min_pulse_width) / frame_width
        self._min_value = -1
        self._value_range = 2
        if pin is None or pin not in (12, 13):
            raise ValueError('Only pin 12 and 13 are supported')
        pwm_channel = 0 if pin == 12 else 1
        self.pwm = HardwarePWM(pwm_channel=pwm_channel, hz=int(1 / frame_width), chip=2)
        self.deadband = deadband
        self.pwm.start(0) # Start with 0% duty cycle
        
        try:
            self.value = initial_value
        except:
            self.pwm.stop()
            raise

    def dutycycle2value(self, value):
        """Convert duty cycle from 5% to 10% to -1 to 1"""
        # return (value - min_duty / max_duty-min_duty) * self._value_range + self._min_value
        return ((value - 5) / 5) * self._value_range + self._min_value
        
    
    def value2dutycycle(self, value):
        """Convert value from -1 to 1 to duty cycle of 5% to 10% (1-2ms)"""
        return ((value - self._min_value) / self._value_range) * 5 + 5

    @property
    def frame_width(self):
        """
        The time between control pulses, measured in seconds.
        """
        return self._frame_width

    @property
    def min_pulse_width(self):
        """
        The control pulse width corresponding to the servo's minimum position,
        measured in seconds.
        """
        return self._min_dc * self.frame_width

    @property
    def max_pulse_width(self):
        """
        The control pulse width corresponding to the servo's maximum position,
        measured in seconds.
        """
        return (self._dc_range * self.frame_width) + self.min_pulse_width

    @property
    def pulse_width(self):
        """
        Returns the current pulse width controlling the servo.
        """
        if self.pwm._hz is None:
            return None
        else:
            return self.pwm._duty_cycle/100 * self.frame_width

    @pulse_width.setter
    def pulse_width(self, value):
        self.pwm.change_duty_cycle(value / self.frame_width)

    def min(self):
        """
        Set the servo to its minimum position.
        """
        self.value = -1

    def mid(self):
        """
        Set the servo to its mid-point position.
        """
        self.value = 0

    def max(self):
        """
        Set the servo to its maximum position.
        """
        self.value = 1

    def detach(self):
        """
        Temporarily disable control of the servo. This is equivalent to
        setting :attr:`value` to :data:`None`.
        """
        self.value = None

    def _get_value(self):
        if self.pwm._hz is None:
            return None
        else:
            return self.dutycycle2value(self.pwm._duty_cycle)

    @property
    def value(self):
        """
        Represents the position of the servo as a value between -1 (the minimum
        position) and +1 (the maximum position). This can also be the special
        value :data:`None` indicating that the servo is currently
        "uncontrolled", i.e. that no control signal is being sent. Typically
        this means the servo's position remains unchanged, but that it can be
        moved by hand.
        """
        result = self._get_value()
        if result is None:
            return result
        else:
            # NOTE: This round() only exists to ensure we don't confuse people
            # by returning 2.220446049250313e-16 as the default initial value
            # instead of 0. The reason _get_value and _set_value are split
            # out is for descendents that require the un-rounded values for
            # accuracy
            return round(result, 14)

    @value.setter
    def value(self, value):
        if value is None or abs(value) < self.deadband:
            self.pwm.stop()
        else:
            # Limit value to -1 to 1
            self.pwm.start(self.value2dutycycle(value))

    @property
    def is_active(self):
        return self.value is not None
