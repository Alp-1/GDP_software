# See https://docs.micropython.org/en/latest/library/rp2.html#rp2.asm_pio for more info on PIO ASM
# See https://docs.micropython.org/en/latest/library/rp2.StateMachine.html for more info on StateMachine
# encoder.py Uses the PIO for rapid response on RP2 chips (Pico)

# Copyright (c) 2022 Peter Hinch
# Released under the MIT License (MIT) - see LICENSE file

# PIO and SM code written by Sandor Attila Gerendi (@sanyi)
# https://github.com/micropython/micropython/pull/6894

from array import array
import rp2
import time
from machine import time_pulse_us


# Closure enables Viper to retain state. Currently (V1.17) nonlocal doesn't
# work: https://github.com/micropython/micropython/issues/8086
# so using arrays.
def make_isr(pos):
    old_x = array("i", (0,))

    @micropython.viper
    def isr(sm):
        i = ptr32(pos)
        p = ptr32(old_x)
        while sm.rx_fifo():
            v: int = int(sm.get()) & 3
            x: int = v & 1
            y: int = v >> 1
            s: int = 1 if (x ^ y) else -1
            i[0] = i[0] + (s if (x ^ p[0]) else (0 - s))
            p[0] = x

    return isr


# Args:
# StateMachine no. (0-7): each instance must have a different sm_no.
# An initialised input Pin: this and the next pin are the encoder interface.
class Encoder:
    def __init__(self, sm_no, base_pin, scale=1):
        self.scale = scale
        self.measurement_time = 10_000  # 10ms
        self._pos = array("i", (0,))  # [pos]
        self.base_pin = base_pin
        self.sm_no = sm_no
        self.sm = rp2.StateMachine(sm_no, self.pio_quadrature, in_base=base_pin)
        self.sm.irq(make_isr(self._pos))  # Instantiate the closure
        self.sm.exec("set(y, 99)")  # Initialise y: guarantee different to the input
        self.sm.active(1)

    @rp2.asm_pio()
    def pio_quadrature(in_init=rp2.PIO.IN_LOW):
        wrap_target()
        label("again")
        in_(pins, 2)
        mov(x, isr)
        jmp(x_not_y, "push_data")
        mov(isr, null)
        jmp("again")
        label("push_data")
        push()
        irq(block, rel(0))
        mov(y, x)
        wrap()

    def reset_sm(self):
        self.sm.restart()

    def is_active(self):
        """A crude way to check if the encoder is running."""
        if time_pulse_us(self.base_pin, 1, self.measurement_time) < 0:
            return False
        return True

    def position(self, value=None):
        if value is not None:
            self._pos[0] = round(value / self.scale)
        return self._pos[0] * self.scale

    def value(self, value=None):
        if value is not None:
            self._pos[0] = value
        return self._pos[0]

    def freq(self):
        """Get the frequency of the encoder (Hz)."""
        start_time = time.ticks_us()
        start_pos = self._pos[0]
        while time.ticks_diff(time.ticks_us(), start_time) < self.measurement_time:
            pass
        position_moved = self._pos[0] - start_pos
        return position_moved / time.ticks_diff(time.ticks_us(), start_time) * 1_000_000
