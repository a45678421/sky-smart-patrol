"""Lightweight Monte Carlo Localization for CircuitPython robots.

The filter uses:
- differential-drive encoder motion prediction,
- a known 2-D polygonal wall map,
- one forward range sensor,
- systematic (low-variance) resampling.

It is intentionally small enough for an RP2040.  A single ultrasonic sensor
provides weak and sometimes ambiguous observations, so the filter is best used
as a local correction layer around a known start pose.  It is not equivalent
to LiDAR-based global localization.
"""

import math
import random


_EPSILON = 1.0e-9


def _normalise_angle(angle_deg):
    return angle_deg % 360.0


def _angle_difference(target_deg, current_deg):
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def _gaussian_sample(stddev):
    """Return a low-cost approximately normal random value.

    Four independent uniform samples give a compact central-limit
    approximation.  This avoids log/sqrt/trigonometric calls in the motion
    loop, which is materially faster on CircuitPython/RP2040.
    """
    if stddev <= 0.0:
        return 0.0

    centered = (
        random.random()
        + random.random()
        + random.random()
        + random.random()
        - 2.0
    )
    return stddev * 1.7320508075688772 * centered


def _point_in_polygon(x, y, polygon):
    """Odd-even point-in-polygon test."""
    inside = False
    count = len(polygon)
    j = count - 1

    for i in range(count):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        crosses = ((yi > y) != (yj > y))
        if crosses:
            denominator = yj - yi
            if abs(denominator) < _EPSILON:
                denominator = _EPSILON
            intersection_x = (xj - xi) * (y - yi) / denominator + xi
            if x < intersection_x:
                inside = not inside
        j = i

    return inside


def _ray_segment_distance(px, py, dx, dy, ax, ay, bx, by):
    """Distance from a unit ray to a segment, or None when there is no hit."""
    sx = bx - ax
    sy = by - ay
    denominator = dx * sy - dy * sx

    if abs(denominator) < _EPSILON:
        return None

    qpx = ax - px
    qpy = ay - py
    ray_distance = (qpx * sy - qpy * sx) / denominator
    segment_ratio = (qpx * dy - qpy * dx) / denominator

    if ray_distance >= 0.0 and 0.0 <= segment_ratio <= 1.0:
        return ray_distance
    return None


class MonteCarloLocalizer:
    """Particle-filter pose estimator for an L-shaped or polygonal arena."""

    def __init__(
        self,
        valid_polygon,
        wall_polygon,
        wheelbase_mm,
        sensor_forward_offset_mm=0.0,
        particle_count=96,
        sensor_sigma_mm=90.0,
        sensor_min_mm=25.0,
        sensor_max_mm=2500.0,
        outlier_gate_mm=350.0,
        resample_ratio=0.55,
    ):
        if particle_count < 20:
            raise ValueError("particle_count must be at least 20")
        if wheelbase_mm <= 0.0:
            raise ValueError("wheelbase_mm must be positive")

        self.valid_polygon = tuple(valid_polygon)
        self.wall_polygon = tuple(wall_polygon)
        self.wheelbase_mm = float(wheelbase_mm)
        self.sensor_forward_offset_mm = float(sensor_forward_offset_mm)
        self.particle_count = int(particle_count)
        self.sensor_sigma_mm = float(sensor_sigma_mm)
        self.sensor_min_mm = float(sensor_min_mm)
        self.sensor_max_mm = float(sensor_max_mm)
        self.outlier_gate_mm = float(outlier_gate_mm)
        self.resample_ratio = float(resample_ratio)

        xs = [point[0] for point in self.valid_polygon]
        ys = [point[1] for point in self.valid_polygon]
        self.min_x = min(xs)
        self.max_x = max(xs)
        self.min_y = min(ys)
        self.max_y = max(ys)

        self.x = [0.0] * self.particle_count
        self.y = [0.0] * self.particle_count
        self.heading = [0.0] * self.particle_count
        self.weight = [1.0 / self.particle_count] * self.particle_count

        self.estimate_x = 0.0
        self.estimate_y = 0.0
        self.estimate_heading_deg = 0.0
        self.position_std_mm = 9999.0
        self.heading_std_deg = 180.0
        self.effective_particle_count = float(self.particle_count)
        self.last_measurement_used = False
        self.last_reject_reason = "NOT_INITIALIZED"
        self.measurement_updates = 0
        self.resample_count = 0
        self.initialization_mode = "NONE"

    def point_is_valid(self, x, y):
        return _point_in_polygon(x, y, self.valid_polygon)

    def initialize_local(
        self,
        x_mm,
        y_mm,
        heading_deg,
        position_std_mm=35.0,
        heading_std_deg=4.0,
    ):
        """Initialize particles near a known start pose."""
        x_mm = float(x_mm)
        y_mm = float(y_mm)
        heading_deg = _normalise_angle(float(heading_deg))

        if not self.point_is_valid(x_mm, y_mm):
            raise ValueError("local MCL start pose is outside the valid polygon")

        for index in range(self.particle_count):
            accepted = False
            for _ in range(20):
                candidate_x = x_mm + _gaussian_sample(position_std_mm)
                candidate_y = y_mm + _gaussian_sample(position_std_mm)
                if self.point_is_valid(candidate_x, candidate_y):
                    accepted = True
                    break

            if not accepted:
                candidate_x = x_mm
                candidate_y = y_mm

            self.x[index] = candidate_x
            self.y[index] = candidate_y
            self.heading[index] = _normalise_angle(
                heading_deg + _gaussian_sample(heading_std_deg)
            )
            self.weight[index] = 1.0 / self.particle_count

        self.estimate_x = x_mm
        self.estimate_y = y_mm
        self.estimate_heading_deg = heading_deg
        self.position_std_mm = position_std_mm
        self.heading_std_deg = heading_std_deg
        self.effective_particle_count = float(self.particle_count)
        self.last_measurement_used = False
        self.last_reject_reason = "LOCAL_INITIALIZED"
        self.initialization_mode = "LOCAL"

    def initialize_global(self):
        """Spread particles across the map. Experimental with one sonar."""
        for index in range(self.particle_count):
            accepted = False
            for _ in range(200):
                candidate_x = self.min_x + random.random() * (
                    self.max_x - self.min_x
                )
                candidate_y = self.min_y + random.random() * (
                    self.max_y - self.min_y
                )
                if self.point_is_valid(candidate_x, candidate_y):
                    accepted = True
                    break

            if not accepted:
                candidate_x = self.valid_polygon[0][0]
                candidate_y = self.valid_polygon[0][1]

            self.x[index] = candidate_x
            self.y[index] = candidate_y
            self.heading[index] = random.random() * 360.0
            self.weight[index] = 1.0 / self.particle_count

        self._update_estimate()
        self.last_measurement_used = False
        self.last_reject_reason = "GLOBAL_INITIALIZED"
        self.initialization_mode = "GLOBAL"

    def predict(self, left_delta_mm, right_delta_mm):
        """Prediction step using differential-drive encoder increments."""
        left_delta_mm = float(left_delta_mm)
        right_delta_mm = float(right_delta_mm)

        if abs(left_delta_mm) < 0.001 and abs(right_delta_mm) < 0.001:
            return

        # Wheel-distance noise grows with travelled distance.  The small base
        # term accounts for quantization and gearbox backlash after encoder
        # increments have been accumulated to roughly 1 mm by code.py.
        left_std = 0.04 + 0.025 * abs(left_delta_mm)
        right_std = 0.04 + 0.025 * abs(right_delta_mm)

        for index in range(self.particle_count):
            noisy_left = left_delta_mm + _gaussian_sample(left_std)
            noisy_right = right_delta_mm + _gaussian_sample(right_std)

            center_mm = (noisy_left + noisy_right) / 2.0
            delta_theta_rad = (
                noisy_right - noisy_left
            ) / self.wheelbase_mm

            old_x = self.x[index]
            old_y = self.y[index]
            old_heading_rad = math.radians(self.heading[index])
            middle_heading_rad = old_heading_rad + delta_theta_rad / 2.0

            new_x = old_x + center_mm * math.cos(middle_heading_rad)
            new_y = old_y + center_mm * math.sin(middle_heading_rad)
            new_heading = _normalise_angle(
                self.heading[index] + math.degrees(delta_theta_rad)
            )

            if self.point_is_valid(new_x, new_y):
                self.x[index] = new_x
                self.y[index] = new_y
            else:
                # Keep the particle inside the legal robot-center region and
                # penalize a motion hypothesis that crosses the known wall.
                self.weight[index] *= 0.05

            self.heading[index] = new_heading

        self._normalize_weights()
        self._update_estimate()

    def expected_distance_mm(self, x_mm, y_mm, heading_deg):
        """Ray-cast the front sensor against the known physical wall map."""
        heading_rad = math.radians(heading_deg)
        dx = math.cos(heading_rad)
        dy = math.sin(heading_rad)

        sensor_x = x_mm + self.sensor_forward_offset_mm * dx
        sensor_y = y_mm + self.sensor_forward_offset_mm * dy

        nearest = self.sensor_max_mm
        count = len(self.wall_polygon)
        for index in range(count):
            ax, ay = self.wall_polygon[index]
            bx, by = self.wall_polygon[(index + 1) % count]
            distance = _ray_segment_distance(
                sensor_x, sensor_y, dx, dy, ax, ay, bx, by
            )
            if distance is not None and distance < nearest:
                nearest = distance

        return nearest

    def correct(self, measured_distance_mm):
        """Correction and optional low-variance resampling.

        Returns True when the range reading was accepted by the map model.
        A much-shorter-than-map reading is treated as an unmapped obstacle and
        is rejected so that an obstacle does not drag the pose estimate.
        """
        if measured_distance_mm is None:
            self.last_measurement_used = False
            self.last_reject_reason = "NO_RANGE"
            return False

        measured_distance_mm = float(measured_distance_mm)
        if (
            measured_distance_mm < self.sensor_min_mm
            or measured_distance_mm > self.sensor_max_mm
        ):
            self.last_measurement_used = False
            self.last_reject_reason = "RANGE_LIMIT"
            return False

        expected_at_estimate = self.expected_distance_mm(
            self.estimate_x,
            self.estimate_y,
            self.estimate_heading_deg,
        )

        # Only reject readings that are substantially shorter than the known
        # wall.  This is the usual signature of a chair/person/box not present
        # in the static map.  Longer readings are retained because sonar beam
        # geometry can miss a corner.
        adaptive_gate_mm = min(
            self.outlier_gate_mm,
            max(120.0, expected_at_estimate * 0.35),
        )
        if (
            self.initialization_mode == "LOCAL"
            and measured_distance_mm
            < expected_at_estimate - adaptive_gate_mm
        ):
            self.last_measurement_used = False
            self.last_reject_reason = "DYNAMIC_OBSTACLE"
            return False

        inverse_two_sigma_squared = 1.0 / (
            2.0 * self.sensor_sigma_mm * self.sensor_sigma_mm
        )
        total_weight = 0.0

        for index in range(self.particle_count):
            expected_mm = self.expected_distance_mm(
                self.x[index], self.y[index], self.heading[index]
            )
            error_mm = measured_distance_mm - expected_mm

            hit_probability = math.exp(
                -(error_mm * error_mm) * inverse_two_sigma_squared
            )

            # A small uniform component prevents one noisy ultrasonic sample
            # from deleting every particle.
            likelihood = 0.025 + 0.975 * hit_probability
            new_weight = self.weight[index] * likelihood
            self.weight[index] = new_weight
            total_weight += new_weight

        if total_weight <= _EPSILON:
            self._set_uniform_weights()
            self.last_measurement_used = False
            self.last_reject_reason = "WEIGHT_COLLAPSE"
            return False

        inverse_total = 1.0 / total_weight
        sum_squared = 0.0
        for index in range(self.particle_count):
            normalized = self.weight[index] * inverse_total
            self.weight[index] = normalized
            sum_squared += normalized * normalized

        self.effective_particle_count = 1.0 / max(sum_squared, _EPSILON)
        self._update_estimate()
        self.last_measurement_used = True
        self.last_reject_reason = "USED"
        self.measurement_updates += 1

        if (
            self.effective_particle_count
            < self.particle_count * self.resample_ratio
        ):
            self._systematic_resample()
            self._update_estimate()

        return True

    def _set_uniform_weights(self):
        uniform = 1.0 / self.particle_count
        for index in range(self.particle_count):
            self.weight[index] = uniform
        self.effective_particle_count = float(self.particle_count)

    def _normalize_weights(self):
        total = sum(self.weight)
        if total <= _EPSILON:
            self._set_uniform_weights()
            return

        inverse_total = 1.0 / total
        sum_squared = 0.0
        for index in range(self.particle_count):
            normalized = self.weight[index] * inverse_total
            self.weight[index] = normalized
            sum_squared += normalized * normalized
        self.effective_particle_count = 1.0 / max(sum_squared, _EPSILON)

    def _systematic_resample(self):
        """Low-variance/systematic resampling."""
        count = self.particle_count
        step = 1.0 / count
        start = random.random() * step

        new_x = [0.0] * count
        new_y = [0.0] * count
        new_heading = [0.0] * count

        cumulative = self.weight[0]
        source_index = 0

        for output_index in range(count):
            threshold = start + output_index * step
            while threshold > cumulative and source_index < count - 1:
                source_index += 1
                cumulative += self.weight[source_index]

            candidate_x = self.x[source_index] + _gaussian_sample(1.5)
            candidate_y = self.y[source_index] + _gaussian_sample(1.5)
            if not self.point_is_valid(candidate_x, candidate_y):
                candidate_x = self.x[source_index]
                candidate_y = self.y[source_index]

            new_x[output_index] = candidate_x
            new_y[output_index] = candidate_y
            new_heading[output_index] = _normalise_angle(
                self.heading[source_index] + _gaussian_sample(0.25)
            )

        self.x = new_x
        self.y = new_y
        self.heading = new_heading
        self._set_uniform_weights()
        self.resample_count += 1

    def _update_estimate(self):
        total = sum(self.weight)
        if total <= _EPSILON:
            return

        inverse_total = 1.0 / total
        mean_x = 0.0
        mean_y = 0.0
        mean_sin = 0.0
        mean_cos = 0.0

        for index in range(self.particle_count):
            normalized = self.weight[index] * inverse_total
            mean_x += normalized * self.x[index]
            mean_y += normalized * self.y[index]
            heading_rad = math.radians(self.heading[index])
            mean_sin += normalized * math.sin(heading_rad)
            mean_cos += normalized * math.cos(heading_rad)

        mean_heading = _normalise_angle(
            math.degrees(math.atan2(mean_sin, mean_cos))
        )

        position_variance = 0.0
        heading_variance = 0.0
        for index in range(self.particle_count):
            normalized = self.weight[index] * inverse_total
            dx = self.x[index] - mean_x
            dy = self.y[index] - mean_y
            position_variance += normalized * (dx * dx + dy * dy)
            angle_error = _angle_difference(
                self.heading[index], mean_heading
            )
            heading_variance += normalized * angle_error * angle_error

        self.estimate_x = mean_x
        self.estimate_y = mean_y
        self.estimate_heading_deg = mean_heading
        self.position_std_mm = math.sqrt(max(position_variance, 0.0))
        self.heading_std_deg = math.sqrt(max(heading_variance, 0.0))

    def status(self):
        """Return compact diagnostic values for telemetry."""
        return {
            "x": self.estimate_x,
            "y": self.estimate_y,
            "heading": self.estimate_heading_deg,
            "position_std": self.position_std_mm,
            "heading_std": self.heading_std_deg,
            "effective_particles": self.effective_particle_count,
            "particle_count": self.particle_count,
            "measurement_used": self.last_measurement_used,
            "reason": self.last_reject_reason,
            "updates": self.measurement_updates,
            "resamples": self.resample_count,
            "mode": self.initialization_mode,
        }
    
    def sampled_particles(self, maximum_count=24):
        """Return a compact sample of particles for visualization."""
        maximum_count = max(1, int(maximum_count))
        step = max(1, self.particle_count // maximum_count)

        particles = []

        for index in range(0, self.particle_count, step):
            particles.append(
                [
                    int(self.x[index]),
                    int(self.y[index]),
                    int(self.heading[index]),
                ]
            )

            if len(particles) >= maximum_count:
                break

        return particles
