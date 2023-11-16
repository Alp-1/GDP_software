"""Additional UARTs for the central hub using pio state machines"""

from machine import Pin
from rp2 import PIO, StateMachine, asm_pio


@asm_pio(
    sideset_init=PIO.OUT_HIGH,
    out_init=PIO.OUT_HIGH,
    out_shiftdir=PIO.SHIFT_RIGHT,
)
def uart_tx():
    # fmt: off
    # Block with TX deasserted until data available
    pull()
    # Initialise bit counter, assert start bit for 8 cycles
    set(x, 7)  .side(0)       [7]
    # Shift out 8 data bits, 8 execution cycles per bit
    label("bitloop")
    out(pins, 1)              [6]
    jmp(x_dec, "bitloop")
    # Assert stop bit for 8 cycles total (incl 1 for pull())
    nop()      .side(1)       [6]
    # fmt: on


@asm_pio(in_shiftdir=PIO.SHIFT_RIGHT)
def uart_rx():
    # fmt: off
    label("start")
    # Stall until start bit is asserted
    wait(0, pin, 0)
    # Preload bit counter, then delay until halfway through
    # the first data bit (12 cycles incl wait, set).
    set(x, 7)                 [10]
    label("bitloop")
    # Shift data bit into ISR
    in_(pins, 1)
    # Loop 8 times, each loop iteration is 8 cycles
    jmp(x_dec, "bitloop")     [6]
    # Check stop bit (should be high)
    jmp(pin, "good_stop")
    # Either a framing error or a break. Set a sticky flag
    # and wait for line to return to idle state.
    irq(block, 4)
    wait(1, pin, 0)
    # Don't push data if we didn't see good framing.
    jmp("start")
    # No delay before returning to start; a little slack is
    # important in case the TX clock is slightly too fast.
    label("good_stop")
    push(block)
    irq(block, rel(0))
    # fmt: on


class SoftUART:
    """Returns a UART object that uses two state machines for TX and RX.

    Atm only supports one RX and one TX pin because there is only one RX buffer configured in the software class

    There are trade-offs between baudrate and maximum burst length.
    """

    def __init__(self, tx_sm_id, rx_sm_id, baudrate, tx, rx):
        """Initialize the UART."""

        # Set rx to input
        rx = Pin(rx, Pin.IN, Pin.PULL_UP)
        # Configure the RX state machine
        self.sm_rx = StateMachine(
            rx_sm_id,
            uart_rx,
            freq=8 * baudrate,
            in_base=Pin(rx),
            jmp_pin=Pin(rx),
        )
        self.sm_rx.active(1)
        self.sm_rx.irq(self.make_isr())

        # Configure the TX state machine
        self.sm_tx = StateMachine(
            tx_sm_id,
            uart_tx,
            freq=8 * baudrate,
            sideset_base=Pin(tx),
            out_base=Pin(tx),
        )
        self.sm_tx.active(1)

        self.timeout = 1  # ms
        self.buffer = bytearray()

    def make_isr(self):
        def rx_handler(sm):
            """Store the received byte in the buffer."""
            self.buffer.append(sm.get(None, 24))

        return rx_handler

    def write(self, byte_buffer):
        """Write a buffer of bytes to the UART."""
        for b in byte_buffer:
            self.sm_tx.put(b)

    def read(self):
        """Read stored data from the UART."""
        result = self.buffer if self.buffer else None
        self.buffer = bytearray()  # reset buffer
        return result

    def any(self):
        """Return the number of bytes in the RX buffer."""
        return len(self.buffer)


if __name__ == "__main__":
    UART_BAUD = 9600
    PIN_BASE = 10

    s_uart = SoftUART(0, 1, UART_BAUD, PIN_BASE, PIN_BASE + 1)
