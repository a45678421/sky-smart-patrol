"""Hardware definition for the RP2040 inspection robot."""

import math

import adafruit_hcsr04
import adafruit_mpu6050
import board
import busio
import digitalio
import pwmio

import pio_encoder


# ---------------- Communication ----------------
uart = busio.UART(board.GP12, board.GP13, baudrate=115200, timeout=0.1)


# ---------------- Mechanical parameters ----------------
wheel_diameter_mm = 65.0
wheel_circumference_mm = math.pi * wheel_diameter_mm
gear_ratio = 90
encoder_poles = 52

# This value must be calibrated on the actual robot.  If the encoder produces
# four counts per electrical cycle, this formula may need an additional x4.
ticks_per_revolution = encoder_poles * gear_ratio
ticks_to_mm = wheel_circumference_mm / ticks_per_revolution

wheelbase_mm = 110.0
sensor_forward_offset_mm = 20.0


# ---------------- TB6612 motor driver ----------------
motor_A1 = digitalio.DigitalInOut(board.GP18)
motor_A2 = digitalio.DigitalInOut(board.GP17)
motor_B1 = digitalio.DigitalInOut(board.GP20)
motor_B2 = digitalio.DigitalInOut(board.GP21)
motor_stby = digitalio.DigitalInOut(board.GP19)

for pin in (motor_A1, motor_A2, motor_B1, motor_B2, motor_stby):
    pin.direction = digitalio.Direction.OUTPUT

motor_stby.value = True

motor_pwm_left = pwmio.PWMOut(board.GP16, frequency=20000, duty_cycle=0)
motor_pwm_right = pwmio.PWMOut(board.GP22, frequency=20000, duty_cycle=0)

left_motor = (motor_A1, motor_A2, motor_pwm_left)
right_motor = (motor_B1, motor_B2, motor_pwm_right)
max_duty = 65535


# ---------------- Encoders ----------------
# Forward motion should make both readings increase.  If one side decreases,
# swap its reversed value between True and False.
left_encoder = pio_encoder.QuadratureEncoder(
    board.GP26, board.GP27, reversed=True
)
right_encoder = pio_encoder.QuadratureEncoder(
    board.GP28, board.GP29, reversed=False
)


# ---------------- Sensors ----------------
front_distance = adafruit_hcsr04.HCSR04(
    trigger_pin=board.GP4,
    echo_pin=board.GP5,
)

i2c0 = busio.I2C(sda=board.GP6, scl=board.GP7)
imu = adafruit_mpu6050.MPU6050(i2c0)


def _clamp(value, minimum=-1.0, maximum=1.0):
    return max(minimum, min(maximum, value))


def set_speed(motor, speed):
    """Set one motor speed from -1.0 to +1.0.

    Unlike the original code, speed=0 stops only the selected motor rather
    than calling stop() and accidentally stopping both motors.
    """
    speed = _clamp(float(speed))
    pin1, pin2, pwm = motor

    if speed > 0:
        pin1.value = True
        pin2.value = False
    elif speed < 0:
        pin1.value = False
        pin2.value = True
    else:
        pin1.value = False
        pin2.value = False

    pwm.duty_cycle = int(max_duty * abs(speed))


def set_left(speed):
    set_speed(left_motor, speed)


def set_right(speed):
    set_speed(right_motor, speed)


def set_all(speed):
    set_left(speed)
    set_right(speed)


def stop():
    set_left(0.0)
    set_right(0.0)
