from machine import Pin, time_pulse_us


class Flight_Controller_Input:
    """Translates the PWM output from the flight controller into speed ranges"""

    pwm_input_pins = [Pin(14, Pin.IN), Pin(15, Pin.IN)]
    pulse_timeout = 50_000

    @staticmethod
    def speed_request():
        """Returns the speed request for the left and right motors in the range [-100,100]"""
        # count = 0
        # We need a repeated read to avoid starting in the middle of a pulse
        raw_throttle_left = time_pulse_us(
            Flight_Controller_Input.pwm_input_pins[0], 1, Flight_Controller_Input.pulse_timeout
        )
        raw_throttle_left = time_pulse_us(
            Flight_Controller_Input.pwm_input_pins[0], 1, Flight_Controller_Input.pulse_timeout
        )
        raw_throttle_right = time_pulse_us(
            Flight_Controller_Input.pwm_input_pins[1], 1, Flight_Controller_Input.pulse_timeout
        )
        raw_throttle_right = time_pulse_us(
            Flight_Controller_Input.pwm_input_pins[1], 1, Flight_Controller_Input.pulse_timeout
        )
        # print("Raw Left: {} Raw Right: {}".format(raw_throttle_left, raw_throttle_right))
        if raw_throttle_left < 0:
            raw_throttle_left = 1500
        if raw_throttle_right < 0:
            raw_throttle_right = 1500
        # Map to between -100 and 100
        raw_throttle_left = max(1000, min(2000, raw_throttle_left))
        raw_throttle_right = max(1000, min(2000, raw_throttle_right))
        normalised_throttle_left = (
            (raw_throttle_left - 1500) / 1000 * 200
        )  # Divide by previous range and multiply by new range
        normalised_throttle_right = (
            (raw_throttle_right - 1500) / 1000 * 200
        )  # Divide by previous range and multiply by new range
        # print("Raw Left: {} Raw Right: {}".format(raw_throttle_left, raw_throttle_right))
        # print("Left: {} Right: {}".format(normalised_throttle_left, normalised_throttle_right))
        return (
            (normalised_throttle_left),
            (normalised_throttle_right),
        )


if __name__ == "__main__":
    import time

    while True:
        time.sleep(0.5)
        print(Flight_Controller_Input.speed_request())
