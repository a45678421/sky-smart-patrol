"""PIO quadrature encoder reader for CircuitPython on RP2040.

The encoder count is maintained inside the PIO state machine.  read() drains
any pending cumulative counts and returns the most recent value.  This avoids
creating an asyncio task while robot.py is still being imported.
"""

import array
import adafruit_pioasm
import rp2pio


_PROGRAM = """
; OSR stores the cumulative encoder count.
; Input pins are encoder channel A and channel B.

    set y, 0
    mov osr, y
read:
    mov x, y
    in null, 32
    in pins, 2
    mov y, isr
    jmp x!=y, different
    jmp read

different:
    in x, 31
    in null, 31
    mov x, isr
    jmp !x, c1_old_zero

c1_old_not_zero:
    jmp pin, count_up
    jmp count_down

c1_old_zero:
    jmp pin, count_down

count_up:
    mov x, ~ osr
    jmp x--, fake
fake:
    mov x, ~ x
    jmp send

count_down:
    mov x, osr
    jmp x--, send

send:
    mov isr, x
    push noblock
    mov osr, x
    jmp read
"""

_ASSEMBLED = adafruit_pioasm.assemble(_PROGRAM)


class QuadratureEncoder:
    def __init__(self, first_pin, second_pin, reversed=False):
        """Create a two-channel encoder reader.

        first_pin and second_pin must be sequential GPIO pins.
        Set reversed=True if the count decreases while the wheel moves forward.
        """
        self.sm = rp2pio.StateMachine(
            _ASSEMBLED,
            frequency=0,
            first_in_pin=first_pin,
            jmp_pin=second_pin,
            in_pin_count=2,
        )
        self.reversed = reversed
        self._buffer = array.array("i", [0])
        self._latest = 0

    def read(self):
        # Each PIO FIFO item is an absolute cumulative count, so keeping only
        # the latest pending value is sufficient.
        while self.sm.in_waiting:
            self.sm.readinto(self._buffer)
            self._latest = self._buffer[0]

        if self.reversed:
            return -self._latest
        return self._latest
