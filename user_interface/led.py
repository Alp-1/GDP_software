"""Some basic blinking LED code"""

from machine import Pin, Timer


class OnBoardLED:
    """Define a blinking rate for onboard LED for a certain message"""

    led_instance = Pin("LED", Pin.OUT)
    led_timer = Timer()

    def __init__(self):
        pass

    @staticmethod
    def led_callback(tm):
        """Callback function to blink the LED"""
        OnBoardLED.led_instance.toggle()

    @classmethod
    def off(cls):
        """Turn off the LED"""
        cls.led_timer.deinit()
        cls.led_instance.off()

    @classmethod
    def on(cls):
        """Turn on the LED"""
        cls.led_timer.deinit()
        cls.led_instance.on()

    @classmethod
    def set_period_ms(cls, period_ms: int):
        """Blink the LED at period_ms milliseconds"""
        cls.led_timer.init(
            mode=Timer.PERIODIC, period=period_ms, callback=cls.led_callback
        )


if __name__ == "__main__":
    OnBoardLED.set_period_ms(1000)
