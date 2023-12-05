import time
from machine import Pin, ADC

from motor_controller.encoder import Encoder


class MotorSensing:
    """Class to sense the parameters of a motor"""

    PPR = 2448  # Pulses per revolution of the encoder

    def __init__(
        self,
        encoder_out_A: int,
        sm_no: int,
        adc_in: int,
        current_fault: int = None,
    ):
        """Initialize the motor.

        Parameters
        ----------
        encoder_out_A: int
            The output A of the encoder.
            The second pin (Output B) is assumed to be encoder_out_A + 1
        sm_no: int
            The state machine number to use for the encoder pio
        adc_in: int
            The ADC pin to use for current sensing
        current_fault: int
            The pin connected to the active low current fault pin of
            ACS711EX

        """
        # Define the pins for the inputs
        ENCODER_OUT_A = Pin(encoder_out_A, Pin.IN, Pin.PULL_DOWN)
        self.encoder = Encoder(sm_no, ENCODER_OUT_A)
        self.current_sensor = ADC(adc_in)
        if current_fault is not None:
            self.fault_current = Pin(current_fault, Pin.IN, Pin.PULL_UP)

        self.no_of_samples = 256
        self.offset_u16 = 0
        self.calibrate_adc()

    def calibrate_adc(self):
        """Calibrate the ADC by finding the zero current adc val
        (i.e. the midpoint of the adc range in theory)
        """
        average_u16 = 0
        for _ in range(self.no_of_samples):
            average_u16 += self.current_sensor.read_u16()
        self.offset_u16 = average_u16 / self.no_of_samples

    @property
    def rpm(self):
        result = self.encoder.freq() * 60 / self.PPR
        # print("Freq: {}".format(self.encoder.freq()))
        # print("RPM: {}".format(result))
        return result

    @property
    def current(self):
        """Get the current of the motor (A).

        For ACS711EX (+-31A), the sensitivity is 45 mV/A for 3.3V supply.

        45mV/A is derived from 3.3V/73.3A (Refer to https://www.pololu.com/product/2453)

        Perform averaging for better accuracy.
        """

        average_u16 = 0
        for _ in range(self.no_of_samples):
            average_u16 += self.current_sensor.read_u16()
        average_u16 /= self.no_of_samples
        vout = (average_u16 - self.offset_u16) * 3.3 / 65535
        return vout / 0.045
