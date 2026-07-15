"""Fixed-area lawn-mower patrol with obstacle bypass.

Uses:
- robot.py
- pio_encoder.py
- pid_controller.py

The robot follows an L-shaped fixed patrol route. When the front HC-SR04
detects an obstacle, it scans left/right by rotating the robot, chooses a legal
side, makes a rectangular detour, returns to the original patrol line, and
continues to the waypoint.

A single fixed front ultrasonic sensor cannot determine the full shape of an
obstacle. Use low speed and test with one stationary obstacle first.
"""

import asyncio
import json
import math
import time

import robot
from pid_controller import PIDController
from mcl_localization import MonteCarloLocalizer


# ============================================================================
# Arena and route settings (mm)
# ============================================================================
ARENA_WIDTH_MM = 1500
ARENA_HEIGHT_MM = 1500
CUTOUT_WIDTH_MM = 500
CUTOUT_HEIGHT_MM = 500

EDGE_MARGIN_MM = 150
LANE_SPACING_MM = 250

START_X_MM = EDGE_MARGIN_MM
START_Y_MM = EDGE_MARGIN_MM
START_HEADING_DEG = 0.0
REPEAT_PATROL = False


# ============================================================================
# Monte Carlo Localization settings
# ============================================================================
# Physical wall polygon measured by the HC-SR04.  This is different from the
# safety polygon below: the sonar sees the real arena walls, while the robot
# center must remain EDGE_MARGIN_MM away from those walls.
MCL_WALL_POLYGON = (
    (0.0, 0.0),
    (ARENA_WIDTH_MM - CUTOUT_WIDTH_MM, 0.0),
    (ARENA_WIDTH_MM - CUTOUT_WIDTH_MM, CUTOUT_HEIGHT_MM),
    (ARENA_WIDTH_MM, CUTOUT_HEIGHT_MM),
    (ARENA_WIDTH_MM, ARENA_HEIGHT_MM),
    (0.0, ARENA_HEIGHT_MM),
)

# Particle validity is based on the robot body radius, not the larger patrol
# safety margin.  Keeping these concepts separate avoids biasing particles
# inward when the configured start pose lies exactly on a patrol margin.
MCL_BODY_MARGIN_MM = 60.0
MCL_VALID_POLYGON = (
    (MCL_BODY_MARGIN_MM, MCL_BODY_MARGIN_MM),
    (ARENA_WIDTH_MM - CUTOUT_WIDTH_MM - MCL_BODY_MARGIN_MM,
     MCL_BODY_MARGIN_MM),
    (ARENA_WIDTH_MM - CUTOUT_WIDTH_MM - MCL_BODY_MARGIN_MM,
     CUTOUT_HEIGHT_MM + MCL_BODY_MARGIN_MM),
    (ARENA_WIDTH_MM - MCL_BODY_MARGIN_MM,
     CUTOUT_HEIGHT_MM + MCL_BODY_MARGIN_MM),
    (ARENA_WIDTH_MM - MCL_BODY_MARGIN_MM,
     ARENA_HEIGHT_MM - MCL_BODY_MARGIN_MM),
    (MCL_BODY_MARGIN_MM, ARENA_HEIGHT_MM - MCL_BODY_MARGIN_MM),
)

MCL_PARTICLE_COUNT = 96
MCL_UPDATE_INTERVAL_S = 0.40
MCL_PREDICT_MIN_WHEEL_MM = 4.0
MCL_SENSOR_SIGMA_MM = 90.0
MCL_OUTLIER_GATE_MM = 350.0


# ============================================================================
# Motion settings
# ============================================================================
# Runtime speed can be changed from the web dashboard.
# The value is a PWM ratio from 0.20 to 0.60.
SPEED_SETTING = 0.40
FORWARD_SPEED = 0.40
MIN_FORWARD_SPEED = 0.22
REVERSE_SPEED = 0.30

TURN_SPEED = 0.30
MIN_TURN_SPEED = 0.21

CONTROL_PERIOD_S = 0.01
SLOW_DOWN_DISTANCE_MM = 180.0
TURN_SLOW_DOWN_MM = 25.0


# ============================================================================
# Obstacle settings
# ============================================================================
OBSTACLE_STOP_MM = 220.0
OBSTACLE_CLEAR_MM = 300.0
OBSTACLE_BACKUP_MM = 100.0

OBSTACLE_SIDE_STEP_MM = 300.0
OBSTACLE_PASS_MM = 450.0
MIN_OBSTACLE_PASS_MM = 300.0

OBSTACLE_SCAN_ANGLE_DEG = 45.0
OBSTACLE_SCAN_SAMPLES = 4
MAX_AVOID_ATTEMPTS = 3

# Closed-loop route recovery after a detour. This uses encoder odometry to
# reduce cross-track error and restore the planned axis-aligned heading.
LINE_RECOVERY_TOLERANCE_MM = 25.0
LINE_RECOVERY_HEADING_TOLERANCE_DEG = 4.0
LINE_RECOVERY_MAX_STEP_MM = 250.0
LINE_RECOVERY_ATTEMPTS = 2

# Manual mode uses the full white arena frame with only a robot-body margin.
MANUAL_BOUNDARY_MARGIN_MM = MCL_BODY_MARGIN_MM
MANUAL_BOUNDARY_LOOKAHEAD_MM = 140.0

# AUTO straight segments use MCL pose to stay close to the current patrol line.
ROUTE_TRACKING_CROSS_KP = 0.0014
ROUTE_TRACKING_HEADING_KP = 0.0060
ROUTE_TRACKING_MAX_CORRECTION = 0.18
ROUTE_TRACKING_MAX_ERROR_MM = 220.0


STRAIGHT_PID = PIDController(
    kp=0.0030,
    ki=0.0002,
    kd=0.0004,
    d_filter_gain=0.20,
    imax=100.0,
    imin=-100.0,
)

TURN_PID = PIDController(
    kp=0.0020,
    ki=0.0,
    kd=0.0002,
    d_filter_gain=0.20,
)


pose_x_mm = START_X_MM
pose_y_mm = START_Y_MM
heading_deg = START_HEADING_DEG

# MCL-corrected live pose used by the web map and route recovery.
# The patrol planner above keeps its own command-based pose values.
odom_x_mm = START_X_MM
odom_y_mm = START_Y_MM
odom_heading_deg = START_HEADING_DEG

# Raw encoder-only pose retained for diagnostics.  This value drifts because
# it is never corrected by the ultrasonic map observation.
raw_odom_x_mm = START_X_MM
raw_odom_y_mm = START_Y_MM
raw_odom_heading_deg = START_HEADING_DEG

mcl = MonteCarloLocalizer(
    valid_polygon=MCL_VALID_POLYGON,
    wall_polygon=MCL_WALL_POLYGON,
    wheelbase_mm=robot.wheelbase_mm,
    sensor_forward_offset_mm=robot.sensor_forward_offset_mm,
    particle_count=MCL_PARTICLE_COUNT,
    sensor_sigma_mm=MCL_SENSOR_SIGMA_MM,
    outlier_gate_mm=MCL_OUTLIER_GATE_MM,
)
mcl.initialize_local(
    START_X_MM,
    START_Y_MM,
    START_HEADING_DEG,
    position_std_mm=25.0,
    heading_std_deg=3.0,
)

system_mode = "IDLE"
system_status = "READY"
manual_action = "STOP"
manual_last_command_time = 0.0
auto_task = None

MANUAL_FORWARD_SPEED = 0.40
MANUAL_REVERSE_SPEED = 0.30
MANUAL_TURN_SPEED = 0.30
MANUAL_COMMAND_TIMEOUT_S = 0.75
TELEMETRY_INTERVAL_S = 0.50

last_line_error_mm = 0.0
line_recovery_state = "IDLE"


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalise_angle(angle):
    return angle % 360.0


def shortest_turn(target_deg, current_deg):
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def apply_speed_setting(value):
    """Apply one web speed setting to automatic and manual motion."""
    global SPEED_SETTING
    global FORWARD_SPEED, MIN_FORWARD_SPEED, REVERSE_SPEED
    global TURN_SPEED, MIN_TURN_SPEED
    global MANUAL_FORWARD_SPEED, MANUAL_REVERSE_SPEED, MANUAL_TURN_SPEED
    global OBSTACLE_STOP_MM, OBSTACLE_CLEAR_MM, SLOW_DOWN_DISTANCE_MM

    value = clamp(float(value), 0.20, 1.00)
    SPEED_SETTING = value

    FORWARD_SPEED = value
    MIN_FORWARD_SPEED = clamp(value * 0.55, 0.17, 0.30)
    REVERSE_SPEED = clamp(value * 0.75, 0.18, 0.42)

    TURN_SPEED = clamp(value * 0.75, 0.20, 0.42)
    MIN_TURN_SPEED = clamp(value * 0.52, 0.17, 0.28)

    MANUAL_FORWARD_SPEED = value
    MANUAL_REVERSE_SPEED = REVERSE_SPEED
    MANUAL_TURN_SPEED = TURN_SPEED

    # Increase stopping and deceleration distance as speed rises.
    OBSTACLE_STOP_MM = 220.0 + max(0.0, value - 0.30) * 400.0
    OBSTACLE_CLEAR_MM = OBSTACLE_STOP_MM + 80.0
    SLOW_DOWN_DISTANCE_MM = max(180.0, value * 500.0)

    print(
        "Speed setting: {:.0f}%, obstacle stop: {:.0f} mm".format(
            SPEED_SETTING * 100.0, OBSTACLE_STOP_MM
        )
    )


def project_point(x_mm, y_mm, heading, distance_mm):
    angle = math.radians(heading)
    return (
        x_mm + distance_mm * math.cos(angle),
        y_mm + distance_mm * math.sin(angle),
    )


def update_pose(distance_mm):
    global pose_x_mm, pose_y_mm
    pose_x_mm, pose_y_mm = project_point(
        pose_x_mm, pose_y_mm, heading_deg, distance_mm
    )



def point_is_inside_arena_frame(x_mm, y_mm, margin_mm=0.0):
    if x_mm < margin_mm:
        return False
    if y_mm < margin_mm:
        return False
    if x_mm > ARENA_WIDTH_MM - margin_mm:
        return False
    if y_mm > ARENA_HEIGHT_MM - margin_mm:
        return False

    cutout_x = ARENA_WIDTH_MM - CUTOUT_WIDTH_MM
    cutout_y = CUTOUT_HEIGHT_MM
    if x_mm > cutout_x - margin_mm and y_mm < cutout_y + margin_mm:
        return False

    return True


def manual_point_is_safe(x_mm, y_mm):
    return point_is_inside_arena_frame(
        x_mm,
        y_mm,
        MANUAL_BOUNDARY_MARGIN_MM,
    )


def route_cross_track_error(route_heading_deg, target_line_mm):
    route_heading_deg = normalise_angle(route_heading_deg)
    if route_heading_deg in (0.0, 180.0):
        cross_error_mm = odom_y_mm - target_line_mm
        if route_heading_deg == 180.0:
            cross_error_mm = -cross_error_mm
    else:
        cross_error_mm = target_line_mm - odom_x_mm
        if route_heading_deg == 270.0:
            cross_error_mm = -cross_error_mm

    return clamp(
        cross_error_mm,
        -ROUTE_TRACKING_MAX_ERROR_MM,
        ROUTE_TRACKING_MAX_ERROR_MM,
    )
def point_is_safe(x_mm, y_mm):
    if x_mm < EDGE_MARGIN_MM:
        return False
    if y_mm < EDGE_MARGIN_MM:
        return False
    if x_mm > ARENA_WIDTH_MM - EDGE_MARGIN_MM:
        return False
    if y_mm > ARENA_HEIGHT_MM - EDGE_MARGIN_MM:
        return False

    cutout_safe_x = ARENA_WIDTH_MM - CUTOUT_WIDTH_MM - EDGE_MARGIN_MM
    cutout_safe_y = CUTOUT_HEIGHT_MM + EDGE_MARGIN_MM

    if x_mm > cutout_safe_x and y_mm < cutout_safe_y:
        return False

    return True


def segment_is_safe(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    distance = math.sqrt(dx * dx + dy * dy)
    sample_count = max(1, int(distance / 25.0))

    for index in range(sample_count + 1):
        ratio = index / sample_count
        x = x1 + dx * ratio
        y = y1 + dy * ratio
        if not point_is_safe(x, y):
            return False

    return True


def detour_is_safe(side_sign, pass_distance_mm):
    side_heading = normalise_angle(heading_deg + side_sign * 90.0)

    x0, y0 = pose_x_mm, pose_y_mm
    x1, y1 = project_point(
        x0, y0, side_heading, OBSTACLE_SIDE_STEP_MM
    )
    x2, y2 = project_point(
        x1, y1, heading_deg, pass_distance_mm
    )
    x3, y3 = project_point(
        x2,
        y2,
        normalise_angle(side_heading + 180.0),
        OBSTACLE_SIDE_STEP_MM,
    )

    return (
        segment_is_safe(x0, y0, x1, y1)
        and segment_is_safe(x1, y1, x2, y2)
        and segment_is_safe(x2, y2, x3, y3)
    )


def read_front_distance_mm():
    try:
        value_cm = robot.front_distance.distance
        if value_cm is None:
            return None
        return value_cm * 10.0
    except RuntimeError:
        return None
    except Exception as error:
        print("Ultrasonic error:", repr(error))
        return None


async def read_stable_front_distance_mm():
    readings = []

    for _ in range(OBSTACLE_SCAN_SAMPLES):
        value = read_front_distance_mm()
        if value is not None:
            readings.append(value)
        await asyncio.sleep(0.06)

    if not readings:
        return None

    readings.sort()
    return readings[len(readings) // 2]


async def wait_until_path_clear():
    robot.stop()
    print("OBSTACLE: waiting for path to clear")

    clear_samples = 0
    while clear_samples < 5:
        distance_mm = read_front_distance_mm()

        # Timeout is not considered clear.
        if distance_mm is not None and distance_mm >= OBSTACLE_CLEAR_MM:
            clear_samples += 1
        else:
            clear_samples = 0

        await asyncio.sleep(0.10)

    print("OBSTACLE: path cleared")


async def drive_distance_mm(
    target_mm,
    direction=1,
    detect_obstacle=True,
    route_heading_deg=None,
    target_line_mm=None,
):
    global last_line_error_mm
    """Return (travelled_mm, blocked)."""
    if target_mm <= 0:
        return 0.0, False

    direction = 1 if direction >= 0 else -1

    start_left = robot.left_encoder.read()
    start_right = robot.right_encoder.read()

    last_time = time.monotonic()
    last_progress_time = last_time
    last_progress_mm = 0.0

    STRAIGHT_PID.reset()
    route_tracking = (
        route_heading_deg is not None and target_line_mm is not None
    )
    blocked = False

    while True:
        now = time.monotonic()
        dt = max(now - last_time, 0.001)
        last_time = now

        left_mm = abs(robot.left_encoder.read() - start_left) * robot.ticks_to_mm
        right_mm = abs(robot.right_encoder.read() - start_right) * robot.ticks_to_mm
        travelled_mm = (left_mm + right_mm) / 2.0
        remaining_mm = target_mm - travelled_mm

        if remaining_mm <= 0:
            break

        if detect_obstacle and direction > 0:
            front_mm = read_front_distance_mm()
            if front_mm is not None and front_mm < OBSTACLE_STOP_MM:
                blocked = True
                break

        if remaining_mm < SLOW_DOWN_DISTANCE_MM:
            speed = (
                MIN_FORWARD_SPEED
                + (FORWARD_SPEED - MIN_FORWARD_SPEED)
                * remaining_mm
                / SLOW_DOWN_DISTANCE_MM
            )
        else:
            speed = FORWARD_SPEED

        if direction < 0:
            speed = min(speed, REVERSE_SPEED)

        wheel_error_mm = left_mm - right_mm
        correction = STRAIGHT_PID.calculate(wheel_error_mm, dt)
        correction = clamp(correction, -0.12, 0.12)

        if route_tracking and direction > 0:
            heading_error_deg = shortest_turn(
                route_heading_deg,
                odom_heading_deg,
            )
            cross_error_mm = route_cross_track_error(
                route_heading_deg,
                target_line_mm,
            )
            last_line_error_mm = cross_error_mm
            route_correction = (
                heading_error_deg * ROUTE_TRACKING_HEADING_KP
                - cross_error_mm * ROUTE_TRACKING_CROSS_KP
            )
            correction += clamp(
                route_correction,
                -ROUTE_TRACKING_MAX_CORRECTION,
                ROUTE_TRACKING_MAX_CORRECTION,
            )
            correction = clamp(correction, -0.22, 0.22)

        left_magnitude = clamp(
            speed - correction, MIN_FORWARD_SPEED, 1.0
        )
        right_magnitude = clamp(
            speed + correction, MIN_FORWARD_SPEED, 1.0
        )

        robot.set_left(direction * left_magnitude)
        robot.set_right(direction * right_magnitude)

        if travelled_mm >= last_progress_mm + 2.0:
            last_progress_mm = travelled_mm
            last_progress_time = now
        elif now - last_progress_time > 2.0:
            robot.stop()
            raise RuntimeError(
                "Motors or encoders stalled during straight motion"
            )

        await asyncio.sleep(CONTROL_PERIOD_S)

    robot.stop()
    await asyncio.sleep(0.15)

    left_mm = abs(robot.left_encoder.read() - start_left) * robot.ticks_to_mm
    right_mm = abs(robot.right_encoder.read() - start_right) * robot.ticks_to_mm
    travelled_mm = min(target_mm, (left_mm + right_mm) / 2.0)

    return travelled_mm, blocked


async def turn_degrees(angle_deg):
    if abs(angle_deg) < 0.5:
        return

    direction = 1.0 if angle_deg > 0 else -1.0
    target_wheel_mm = (
        math.pi * robot.wheelbase_mm * abs(angle_deg) / 360.0
    )

    start_left = robot.left_encoder.read()
    start_right = robot.right_encoder.read()

    last_time = time.monotonic()
    last_progress_time = last_time
    last_progress_mm = 0.0

    TURN_PID.reset()

    while True:
        now = time.monotonic()
        dt = max(now - last_time, 0.001)
        last_time = now

        left_mm = abs(robot.left_encoder.read() - start_left) * robot.ticks_to_mm
        right_mm = abs(robot.right_encoder.read() - start_right) * robot.ticks_to_mm
        travelled_mm = (left_mm + right_mm) / 2.0
        remaining_mm = target_wheel_mm - travelled_mm

        if remaining_mm <= 0:
            break

        if remaining_mm < TURN_SLOW_DOWN_MM:
            speed = (
                MIN_TURN_SPEED
                + (TURN_SPEED - MIN_TURN_SPEED)
                * remaining_mm
                / TURN_SLOW_DOWN_MM
            )
        else:
            speed = TURN_SPEED

        wheel_error_mm = left_mm - right_mm
        correction = TURN_PID.calculate(wheel_error_mm, dt)
        correction = clamp(correction, -0.08, 0.08)

        left_magnitude = clamp(
            speed - correction, MIN_TURN_SPEED, 1.0
        )
        right_magnitude = clamp(
            speed + correction, MIN_TURN_SPEED, 1.0
        )

        if direction > 0:
            robot.set_left(-left_magnitude)
            robot.set_right(right_magnitude)
        else:
            robot.set_left(left_magnitude)
            robot.set_right(-right_magnitude)

        if travelled_mm >= last_progress_mm + 1.0:
            last_progress_mm = travelled_mm
            last_progress_time = now
        elif now - last_progress_time > 2.0:
            robot.stop()
            raise RuntimeError("Motors or encoders stalled during turn")

        await asyncio.sleep(CONTROL_PERIOD_S)

    robot.stop()
    await asyncio.sleep(0.20)


async def turn_and_update(angle_deg):
    global heading_deg
    await turn_degrees(angle_deg)
    heading_deg = normalise_angle(heading_deg + angle_deg)


async def move_and_update(
    distance_mm,
    direction=1,
    detect_obstacle=True,
    wait_if_blocked=False,
    route_heading_deg=None,
    target_line_mm=None,
):
    remaining_mm = distance_mm
    total_travelled_mm = 0.0

    while remaining_mm > 1.0:
        travelled_mm, blocked = await drive_distance_mm(
            remaining_mm,
            direction=direction,
            detect_obstacle=detect_obstacle,
            route_heading_deg=route_heading_deg,
            target_line_mm=target_line_mm,
        )

        signed_distance = travelled_mm if direction > 0 else -travelled_mm
        update_pose(signed_distance)

        total_travelled_mm += travelled_mm
        remaining_mm -= travelled_mm

        if blocked:
            if not wait_if_blocked:
                return total_travelled_mm, True

            await wait_until_path_clear()

        if travelled_mm < 1.0 and not blocked:
            raise RuntimeError("No encoder progress")

    return total_travelled_mm, False


async def scan_left_and_right():
    print("OBSTACLE: scanning left")
    await turn_degrees(OBSTACLE_SCAN_ANGLE_DEG)
    left_mm = await read_stable_front_distance_mm()
    await turn_degrees(-OBSTACLE_SCAN_ANGLE_DEG)

    print("OBSTACLE: scanning right")
    await turn_degrees(-OBSTACLE_SCAN_ANGLE_DEG)
    right_mm = await read_stable_front_distance_mm()
    await turn_degrees(OBSTACLE_SCAN_ANGLE_DEG)

    left_print = -1 if left_mm is None else int(left_mm)
    right_print = -1 if right_mm is None else int(right_mm)
    print(
        "OBSTACLE: left={} mm, right={} mm".format(
            left_print, right_print
        )
    )

    return left_mm, right_mm


def choose_detour_side(left_mm, right_mm, pass_distance_mm):
    left_legal = detour_is_safe(+1, pass_distance_mm)
    right_legal = detour_is_safe(-1, pass_distance_mm)

    print(
        "OBSTACLE: legal left={}, right={}".format(
            left_legal, right_legal
        )
    )

    if not left_legal and not right_legal:
        return 0
    if left_legal and not right_legal:
        return +1
    if right_legal and not left_legal:
        return -1

    left_score = 0.0 if left_mm is None else left_mm
    right_score = 0.0 if right_mm is None else right_mm

    return +1 if left_score >= right_score else -1


async def turn_to_odom_heading(target_heading_deg):
    """Correct the physical heading using live encoder odometry."""
    global heading_deg

    for _ in range(2):
        error_deg = shortest_turn(target_heading_deg, odom_heading_deg)
        if abs(error_deg) <= LINE_RECOVERY_HEADING_TOLERANCE_DEG:
            break
        await turn_degrees(error_deg)
        await asyncio.sleep(0.08)

    heading_deg = normalise_angle(target_heading_deg)


async def recover_to_patrol_line(route_heading_deg, target_line_mm):
    """Return the robot to the planned horizontal or vertical patrol line.

    This reduces encoder-visible cross-track and heading error. It cannot
    correct physical wheel slip that the encoders did not measure.
    """
    global pose_x_mm, pose_y_mm, heading_deg
    global last_line_error_mm, line_recovery_state, system_status

    route_heading_deg = normalise_angle(route_heading_deg)
    line_recovery_state = "RUNNING"
    system_status = "LINE_RECOVERY"

    for attempt in range(LINE_RECOVERY_ATTEMPTS):
        if route_heading_deg in (0.0, 180.0):
            cross_error_mm = odom_y_mm - target_line_mm
            if abs(cross_error_mm) <= LINE_RECOVERY_TOLERANCE_MM:
                break
            lateral_heading = 270.0 if cross_error_mm > 0 else 90.0
        else:
            cross_error_mm = odom_x_mm - target_line_mm
            if abs(cross_error_mm) <= LINE_RECOVERY_TOLERANCE_MM:
                break
            lateral_heading = 180.0 if cross_error_mm > 0 else 0.0

        last_line_error_mm = cross_error_mm
        correction_mm = min(abs(cross_error_mm), LINE_RECOVERY_MAX_STEP_MM)

        target_x, target_y = project_point(
            odom_x_mm, odom_y_mm, lateral_heading, correction_mm
        )
        if not segment_is_safe(odom_x_mm, odom_y_mm, target_x, target_y):
            line_recovery_state = "BOUNDARY_BLOCKED"
            system_status = "RECOVERY_BOUNDARY"
            break

        print(
            "LINE RECOVERY attempt {}: error {:.1f} mm".format(
                attempt + 1, cross_error_mm
            )
        )

        await turn_to_odom_heading(lateral_heading)
        travelled_mm, blocked = await drive_distance_mm(
            correction_mm,
            direction=1,
            detect_obstacle=True,
        )
        await asyncio.sleep(0.08)

        if blocked or travelled_mm < correction_mm * 0.70:
            line_recovery_state = "OBSTACLE_BLOCKED"
            system_status = "RECOVERY_BLOCKED"
            await wait_until_path_clear()
            break

    await turn_to_odom_heading(route_heading_deg)
    await asyncio.sleep(0.08)

    if route_heading_deg in (0.0, 180.0):
        last_line_error_mm = odom_y_mm - target_line_mm
        pose_x_mm = odom_x_mm
        pose_y_mm = target_line_mm
    else:
        last_line_error_mm = odom_x_mm - target_line_mm
        pose_x_mm = target_line_mm
        pose_y_mm = odom_y_mm

    heading_deg = route_heading_deg

    if abs(last_line_error_mm) <= LINE_RECOVERY_TOLERANCE_MM:
        line_recovery_state = "RECOVERED"
        system_status = "LINE_RECOVERED"
    elif line_recovery_state == "RUNNING":
        line_recovery_state = "PARTIAL"
        system_status = "RECOVERY_PARTIAL"

    print(
        "LINE RECOVERY: state={}, remaining error={:.1f} mm".format(
            line_recovery_state, last_line_error_mm
        )
    )


async def perform_rectangular_detour(side_sign, pass_distance_mm):
    global pose_x_mm, pose_y_mm

    original_heading = heading_deg
    original_x = pose_x_mm
    original_y = pose_y_mm

    side_name = "LEFT" if side_sign > 0 else "RIGHT"
    print("OBSTACLE: bypassing on", side_name)

    await turn_and_update(side_sign * 90.0)
    await move_and_update(
        OBSTACLE_SIDE_STEP_MM,
        detect_obstacle=True,
        wait_if_blocked=True,
    )

    await turn_and_update(-side_sign * 90.0)
    await move_and_update(
        pass_distance_mm,
        detect_obstacle=True,
        wait_if_blocked=True,
    )

    await turn_and_update(-side_sign * 90.0)
    await move_and_update(
        OBSTACLE_SIDE_STEP_MM,
        detect_obstacle=True,
        wait_if_blocked=True,
    )

    await turn_and_update(side_sign * 90.0)

    # The rectangular commands are only nominal. Use live odometry to reduce
    # the cross-track error and restore the original route heading.
    target_line_mm = (
        original_y
        if original_heading in (0.0, 180.0)
        else original_x
    )
    await recover_to_patrol_line(original_heading, target_line_mm)

    print(
        "OBSTACLE: bypass complete at ({:.0f}, {:.0f}), heading {:.0f}".format(
            pose_x_mm, pose_y_mm, heading_deg
        )
    )


def distance_to_target_along_heading(target_x_mm, target_y_mm):
    if heading_deg == 0.0:
        return target_x_mm - pose_x_mm
    if heading_deg == 180.0:
        return pose_x_mm - target_x_mm
    if heading_deg == 90.0:
        return target_y_mm - pose_y_mm
    if heading_deg == 270.0:
        return pose_y_mm - target_y_mm

    raise RuntimeError("Heading is not axis aligned")


async def avoid_obstacle(target_x_mm, target_y_mm):
    remaining_before_backup = distance_to_target_along_heading(
        target_x_mm, target_y_mm
    )

    if remaining_before_backup < MIN_OBSTACLE_PASS_MM:
        print("OBSTACLE: too close to waypoint for safe bypass")
        await wait_until_path_clear()
        return False

    backup_x, backup_y = project_point(
        pose_x_mm,
        pose_y_mm,
        heading_deg,
        -OBSTACLE_BACKUP_MM,
    )

    if segment_is_safe(pose_x_mm, pose_y_mm, backup_x, backup_y):
        print("OBSTACLE: reversing {} mm".format(int(OBSTACLE_BACKUP_MM)))
        await move_and_update(
            OBSTACLE_BACKUP_MM,
            direction=-1,
            detect_obstacle=False,
        )
    else:
        print("OBSTACLE: reverse skipped near arena boundary")

    remaining_mm = distance_to_target_along_heading(
        target_x_mm, target_y_mm
    )
    pass_distance_mm = min(OBSTACLE_PASS_MM, remaining_mm)

    if pass_distance_mm < MIN_OBSTACLE_PASS_MM:
        print("OBSTACLE: insufficient route remaining for detour")
        await wait_until_path_clear()
        return False

    left_mm, right_mm = await scan_left_and_right()

    side_sign = choose_detour_side(
        left_mm, right_mm, pass_distance_mm
    )

    if side_sign == 0:
        print("OBSTACLE: both bypass paths leave the arena")
        await wait_until_path_clear()
        return False

    await perform_rectangular_detour(
        side_sign, pass_distance_mm
    )
    return True


def x_limit_for_y(y_mm):
    if y_mm < CUTOUT_HEIGHT_MM:
        return ARENA_WIDTH_MM - CUTOUT_WIDTH_MM - EDGE_MARGIN_MM
    return ARENA_WIDTH_MM - EDGE_MARGIN_MM


def build_patrol_waypoints():
    y_values = []
    y = EDGE_MARGIN_MM

    while y <= ARENA_HEIGHT_MM - EDGE_MARGIN_MM:
        y_values.append(y)
        y += LANE_SPACING_MM

    waypoints = []

    for lane_index, lane_y in enumerate(y_values):
        x_min = EDGE_MARGIN_MM
        x_max = x_limit_for_y(lane_y)

        if lane_index % 2 == 0:
            lane_start = (x_min, lane_y)
            lane_end = (x_max, lane_y)
        else:
            lane_start = (x_max, lane_y)
            lane_end = (x_min, lane_y)

        if not waypoints:
            waypoints.append(lane_start)
        else:
            current_x, current_y = waypoints[-1]

            if current_y != lane_y:
                waypoints.append((current_x, lane_y))

            if current_x != lane_start[0]:
                waypoints.append(lane_start)

        waypoints.append(lane_end)

    return waypoints


async def go_to_waypoint(target_x_mm, target_y_mm):
    global pose_x_mm, pose_y_mm, heading_deg

    dx = target_x_mm - pose_x_mm
    dy = target_y_mm - pose_y_mm

    if abs(dx) > 0.5 and abs(dy) > 0.5:
        raise ValueError("Diagonal waypoint segment is not supported")

    if abs(dx) > 0.5:
        target_heading = 0.0 if dx > 0 else 180.0
    elif abs(dy) > 0.5:
        target_heading = 90.0 if dy > 0 else 270.0
    else:
        return

    turn_angle = shortest_turn(target_heading, heading_deg)

    print(
        "TURN {:.1f} deg -> target ({:.0f}, {:.0f})".format(
            turn_angle, target_x_mm, target_y_mm
        )
    )

    await turn_and_update(turn_angle)

    avoid_attempts = 0

    while True:
        remaining_mm = distance_to_target_along_heading(
            target_x_mm, target_y_mm
        )

        if remaining_mm <= 2.0:
            break

        print("MOVE remaining {:.1f} mm".format(remaining_mm))

        target_line_mm = (
            pose_y_mm if target_heading in (0.0, 180.0) else pose_x_mm
        )

        _, blocked = await move_and_update(
            remaining_mm,
            direction=1,
            detect_obstacle=True,
            wait_if_blocked=False,
            route_heading_deg=target_heading,
            target_line_mm=target_line_mm,
        )

        if not blocked:
            break

        avoid_attempts += 1
        front_mm = read_front_distance_mm()
        front_print = -1 if front_mm is None else int(front_mm)

        print(
            "OBSTACLE detected: {} mm, attempt {}/{}".format(
                front_print,
                avoid_attempts,
                MAX_AVOID_ATTEMPTS,
            )
        )

        if avoid_attempts > MAX_AVOID_ATTEMPTS:
            print("OBSTACLE: bypass attempts exceeded; waiting")
            await wait_until_path_clear()
            avoid_attempts = 0
            continue

        await avoid_obstacle(target_x_mm, target_y_mm)

    pose_x_mm = target_x_mm
    pose_y_mm = target_y_mm
    heading_deg = normalise_angle(target_heading)


def print_route(waypoints):
    print("Patrol route:")
    for index, point in enumerate(waypoints):
        print(
            "  {:02d}: ({:.0f}, {:.0f}) mm".format(
                index, point[0], point[1]
            )
        )


async def patrol_once():
    waypoints = build_patrol_waypoints()
    print_route(waypoints)

    first_x, first_y = waypoints[0]
    if (
        abs(first_x - pose_x_mm) > 0.5
        or abs(first_y - pose_y_mm) > 0.5
    ):
        raise RuntimeError(
            "Configured start pose does not match first waypoint"
        )

    for waypoint_index, waypoint in enumerate(
        waypoints[1:], start=1
    ):
        target_x, target_y = waypoint
        print(
            "WAYPOINT {}/{}".format(
                waypoint_index, len(waypoints) - 1
            )
        )
        await go_to_waypoint(target_x, target_y)

    robot.stop()
    print(
        "PATROL COMPLETE at ({:.0f}, {:.0f}), heading {:.0f} deg".format(
            pose_x_mm, pose_y_mm, heading_deg
        )
    )


async def odometry_loop():
    """Run raw odometry plus MCL prediction/correction.

    Encoder increments first update a diagnostic dead-reckoning pose.  The
    same increments then move all MCL particles.  At a lower rate, the front
    ultrasonic reading is compared with the known wall polygon and used to
    weight/resample particles.
    """
    global odom_x_mm, odom_y_mm, odom_heading_deg
    global raw_odom_x_mm, raw_odom_y_mm, raw_odom_heading_deg
    global pose_x_mm, pose_y_mm, heading_deg

    last_left = robot.left_encoder.read()
    last_right = robot.right_encoder.read()
    pending_left_mm = 0.0
    pending_right_mm = 0.0
    last_mcl_measurement_time = time.monotonic()

    while True:
        current_left = robot.left_encoder.read()
        current_right = robot.right_encoder.read()

        left_mm = (current_left - last_left) * robot.ticks_to_mm
        right_mm = (current_right - last_right) * robot.ticks_to_mm
        last_left = current_left
        last_right = current_right

        # Raw differential-drive odometry, kept only for comparison.
        center_mm = (left_mm + right_mm) / 2.0
        delta_theta_rad = (right_mm - left_mm) / robot.wheelbase_mm
        middle_heading_rad = (
            math.radians(raw_odom_heading_deg) + delta_theta_rad / 2.0
        )

        raw_odom_x_mm += center_mm * math.cos(middle_heading_rad)
        raw_odom_y_mm += center_mm * math.sin(middle_heading_rad)
        raw_odom_heading_deg = normalise_angle(
            raw_odom_heading_deg + math.degrees(delta_theta_rad)
        )

        # Accumulate small encoder changes before particle prediction.  This
        # lowers RP2040 CPU load and avoids injecting base noise at 50 Hz.
        pending_left_mm += left_mm
        pending_right_mm += right_mm
        if max(abs(pending_left_mm), abs(pending_right_mm)) >= (
            MCL_PREDICT_MIN_WHEEL_MM
        ):
            mcl.predict(pending_left_mm, pending_right_mm)
            pending_left_mm = 0.0
            pending_right_mm = 0.0

        now = time.monotonic()
        if now - last_mcl_measurement_time >= MCL_UPDATE_INTERVAL_S:
            # Flush the last sub-millimetre motion before applying a sensor
            # correction so particles and the physical robot share one time.
            if abs(pending_left_mm) > 0.001 or abs(pending_right_mm) > 0.001:
                mcl.predict(pending_left_mm, pending_right_mm)
                pending_left_mm = 0.0
                pending_right_mm = 0.0

            front_mm = read_front_distance_mm()
            mcl.correct(front_mm)
            last_mcl_measurement_time = now

        odom_x_mm = mcl.estimate_x
        odom_y_mm = mcl.estimate_y
        odom_heading_deg = mcl.estimate_heading_deg

        # Manual control changes the planner pose too, so AUTO will refuse to
        # start unless the robot is physically returned to the configured
        # start pose and RESET_POSE is pressed.
        if system_mode == "MANUAL":
            pose_x_mm = odom_x_mm
            pose_y_mm = odom_y_mm
            heading_deg = odom_heading_deg

        await asyncio.sleep(0.02)


def compact_status(text):
    text = str(text)
    return text if len(text) <= 28 else text[:28]


def write_telemetry():
    distance_mm = read_front_distance_mm()
    payload = {
        # MCL-corrected pose.
        "x": round(odom_x_mm, 1),
        "y": round(odom_y_mm, 1),
        "z": 0.0,
        "h": round(odom_heading_deg, 1),
        # Raw encoder odometry for drift comparison.
        "ox": round(raw_odom_x_mm, 1),
        "oy": round(raw_odom_y_mm, 1),
        "oh": round(raw_odom_heading_deg, 1),
        "m": system_mode,
        "s": compact_status(system_status),
        "d": None if distance_mm is None else round(distance_mm, 1),
        "px": round(pose_x_mm, 1),
        "py": round(pose_y_mm, 1),
        "v": round(SPEED_SETTING * 100.0, 0),
        "e": round(last_line_error_mm, 1),
        "r": line_recovery_state,
    }
    try:
        robot.uart.write((json.dumps(payload) + "\n").encode("utf-8"))
    except Exception as error:
        print("Telemetry UART error:", repr(error))


async def telemetry_loop():
    while True:
        write_telemetry()
        await asyncio.sleep(TELEMETRY_INTERVAL_S)

async def mcl_debug_loop():
    while True:
        front_mm = read_front_distance_mm()

        print(
            (
                "MCL=({:.1f}, {:.1f}, {:.1f}) "
                "RAW=({:.1f}, {:.1f}, {:.1f}) "
                "range={} "
                "spread=({:.1f}mm, {:.1f}deg) "
                "Neff={:.1f} used={} reason={}"
            ).format(
                odom_x_mm,
                odom_y_mm,
                odom_heading_deg,
                raw_odom_x_mm,
                raw_odom_y_mm,
                raw_odom_heading_deg,
                "None" if front_mm is None else int(front_mm),
                mcl.position_std_mm,
                mcl.heading_std_deg,
                mcl.effective_particle_count,
                mcl.last_measurement_used,
                mcl.last_reject_reason,
            )
        )

        await asyncio.sleep(0.50)
        
def cancel_auto():
    global auto_task
    if auto_task is not None:
        try:
            auto_task.cancel()
        except Exception:
            pass
        auto_task = None


def reset_all_poses():
    global pose_x_mm, pose_y_mm, heading_deg
    global odom_x_mm, odom_y_mm, odom_heading_deg
    global raw_odom_x_mm, raw_odom_y_mm, raw_odom_heading_deg
    global last_line_error_mm, line_recovery_state

    pose_x_mm = START_X_MM
    pose_y_mm = START_Y_MM
    heading_deg = START_HEADING_DEG
    odom_x_mm = START_X_MM
    odom_y_mm = START_Y_MM
    odom_heading_deg = START_HEADING_DEG
    raw_odom_x_mm = START_X_MM
    raw_odom_y_mm = START_Y_MM
    raw_odom_heading_deg = START_HEADING_DEG
    mcl.initialize_local(
        START_X_MM,
        START_Y_MM,
        START_HEADING_DEG,
        position_std_mm=25.0,
        heading_std_deg=3.0,
    )
    last_line_error_mm = 0.0
    line_recovery_state = "IDLE"


async def automatic_patrol_runner():
    global system_mode, system_status, auto_task
    system_mode = "AUTO"
    system_status = "PATROL_RUNNING"
    robot.stop()

    try:
        await patrol_once()
        system_status = "PATROL_COMPLETE"
    except asyncio.CancelledError:
        system_status = "AUTO_CANCELLED"
        raise
    except Exception as error:
        system_status = "AUTO_ERROR"
        print("AUTO ERROR:", repr(error))
    finally:
        robot.stop()
        if system_mode == "AUTO":
            system_mode = "IDLE"
        auto_task = None


async def handle_command(command):
    global system_mode, system_status, manual_action
    global manual_last_command_time, auto_task

    command = command.upper().strip()
    print("WEB COMMAND:", command)

    if command.startswith("SPEED:"):
        try:
            value = float(command.split(":", 1)[1])
            apply_speed_setting(value)
            system_status = "SPEED_{:.0f}".format(SPEED_SETTING * 100.0)
        except (ValueError, TypeError):
            system_status = "SPEED_INVALID"
        return

    if command == "STOP":
        cancel_auto()
        manual_action = "STOP"
        system_mode = "IDLE"
        system_status = "STOPPED"
        robot.stop()
        return

    if command == "RESET_POSE":
        cancel_auto()
        manual_action = "STOP"
        system_mode = "IDLE"
        robot.stop()
        reset_all_poses()
        system_status = "POSE_RESET"
        return

    if command == "MCL_RESET":
        cancel_auto()
        manual_action = "STOP"
        system_mode = "IDLE"
        robot.stop()
        mcl.initialize_local(
            odom_x_mm,
            odom_y_mm,
            odom_heading_deg,
            position_std_mm=45.0,
            heading_std_deg=6.0,
        )
        system_status = "MCL_RESET"
        return

    if command == "AUTO":
        cancel_auto()
        manual_action = "STOP"
        robot.stop()
        system_status = "AUTO_STARTING"
        auto_task = asyncio.create_task(automatic_patrol_runner())
        return

    if command in ("FORWARD", "BACKWARD", "LEFT", "RIGHT"):
        cancel_auto()
        system_mode = "MANUAL"
        manual_action = command
        manual_last_command_time = time.monotonic()
        system_status = "MANUAL_" + command
        return

    system_status = "UNKNOWN_COMMAND"


async def uart_command_loop():
    """Receive commands forwarded by Nicla Vision."""
    rx_buffer = b""

    while True:
        try:
            waiting = robot.uart.in_waiting
            if waiting:
                data = robot.uart.read(waiting)
                if data:
                    rx_buffer += data

            while b"\n" in rx_buffer:
                line, rx_buffer = rx_buffer.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    text = line.decode("utf-8")
                except UnicodeError:
                    continue
                if text.startswith("CMD:"):
                    await handle_command(text[4:])
        except Exception as error:
            print("Command UART error:", repr(error))

        await asyncio.sleep(0.02)


def manual_boundary_is_safe(action):
    if action == "FORWARD":
        test_heading = odom_heading_deg
        test_distance_mm = MANUAL_BOUNDARY_LOOKAHEAD_MM
    elif action == "BACKWARD":
        test_heading = normalise_angle(odom_heading_deg + 180.0)
        test_distance_mm = MANUAL_BOUNDARY_LOOKAHEAD_MM
    elif action in ("LEFT", "RIGHT"):
        return manual_point_is_safe(odom_x_mm, odom_y_mm)
    else:
        return True

    if not manual_point_is_safe(odom_x_mm, odom_y_mm):
        return False

    test_x, test_y = project_point(
        odom_x_mm,
        odom_y_mm,
        test_heading,
        test_distance_mm,
    )
    return manual_point_is_safe(test_x, test_y)
async def manual_control_loop():
    global manual_action, system_mode, system_status

    while True:
        if system_mode != "MANUAL":
            await asyncio.sleep(0.03)
            continue

        if time.monotonic() - manual_last_command_time > MANUAL_COMMAND_TIMEOUT_S:
            robot.stop()
            manual_action = "STOP"
            system_mode = "IDLE"
            system_status = "MANUAL_TIMEOUT"
            await asyncio.sleep(0.03)
            continue

        if not manual_boundary_is_safe(manual_action):
            robot.stop()
            system_status = "BOUNDARY_STOP"
            await asyncio.sleep(0.03)
            continue

        if manual_action == "FORWARD":
            distance_mm = read_front_distance_mm()
            if distance_mm is not None and distance_mm < OBSTACLE_STOP_MM:
                robot.stop()
                system_status = "OBSTACLE_STOP"
            else:
                robot.set_left(MANUAL_FORWARD_SPEED)
                robot.set_right(MANUAL_FORWARD_SPEED)
                system_status = "MANUAL_FORWARD"
        elif manual_action == "BACKWARD":
            robot.set_left(-MANUAL_REVERSE_SPEED)
            robot.set_right(-MANUAL_REVERSE_SPEED)
            system_status = "MANUAL_BACKWARD"
        elif manual_action == "LEFT":
            robot.set_left(-MANUAL_TURN_SPEED)
            robot.set_right(MANUAL_TURN_SPEED)
            system_status = "MANUAL_LEFT"
        elif manual_action == "RIGHT":
            robot.set_left(MANUAL_TURN_SPEED)
            robot.set_right(-MANUAL_TURN_SPEED)
            system_status = "MANUAL_RIGHT"
        else:
            robot.stop()

        await asyncio.sleep(0.03)


async def system_main():
    global system_status

    await asyncio.sleep(1.0)
    robot.stop()
    apply_speed_setting(SPEED_SETTING)
    system_status = "READY_FOR_WEB"

    asyncio.create_task(odometry_loop())
    asyncio.create_task(telemetry_loop())
    asyncio.create_task(mcl_debug_loop())
    asyncio.create_task(uart_command_loop())
    asyncio.create_task(manual_control_loop())

    print("Web-control firmware with MCL ready")
    print("Place robot at ({}, {}) mm facing +X, then press RESET_POSE.".format(
        START_X_MM, START_Y_MM
    ))

    try:
        while True:
            await asyncio.sleep(1.0)
    finally:
        cancel_auto()
        robot.stop()


asyncio.run(system_main())

