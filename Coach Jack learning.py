# filepath: c:\Users\Watto\2026_2027\2026_2027\Coach Jack learning.py
import time
import math
import os
import random
from dataclasses import dataclass
from typing import Optional, List, Tuple

# ---------- Hardware abstraction layer (stubs) ----------

class HardwareMotor:
    def __init__(self, port: int, inverted: bool = False):
        self.port = port
        self.inverted = inverted
        self._volt = 0.0

    def set_voltage(self, volts: float):
        self._volt = -volts if self.inverted else volts

    def stop(self):
        self.set_voltage(0.0)


class HardwareGPS:
    def __init__(self, port: int):
        self.port = port

    def get_position(self) -> Tuple[float, float]:
        return (0.0, 0.0)

    def get_heading(self) -> float:
        return 0.0

    def read_field_tag(self) -> Optional[str]:
        return None


class HardwareVision:
    def __init__(self, port: int):
        self.port = port

    def get_color(self) -> Optional[str]:
        return None

    def get_apriltag(self) -> Optional[str]:
        return None


class HardwareRadio:
    def __init__(self, port: int):
        self.port = port
        # optional: set radio.team_color externally


class HardwareBump:
    def __init__(self, port: int):
        self.port = port

    def is_pressed(self) -> bool:
        return False


class HardwarePositionSensor:
    def __init__(self, port: int):
        self.port = port

    def get_position(self) -> Tuple[float, float]:
        return (0.0, 0.0)


# ---------- Utility classes ----------

@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integrator_limit: float = 1.0

    _integral: float = 0.0
    _last_error: float = 0.0

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self._integral += error * dt
        if abs(self._integral) > self.integrator_limit:
            self._integral = math.copysign(self.integrator_limit, self._integral)
        derivative = (error - self._last_error) / dt
        self._last_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class FuzzyController:
    def __init__(self, max_forward_boost: float = 1.5, max_heading_adjust: float = 2.0):
        self.max_forward_boost = max_forward_boost
        self.max_heading_adjust = max_heading_adjust

    @staticmethod
    def _tri(x, a, b, c):
        if x <= a or x >= c:
            return 0.0
        if x == b:
            return 1.0
        if x < b:
            return (x - a) / (b - a)
        return (c - x) / (c - b)

    def _distance_memberships(self, d: float):
        near = self._tri(d, 0.0, 0.05, 0.15)
        medium = self._tri(d, 0.05, 0.25, 0.45)
        far = self._tri(d, 0.2, 0.6, 1.5)
        return near, medium, far

    def _heading_memberships(self, err_deg: float):
        big_left = self._tri(err_deg, -180.0, -90.0, -30.0)
        small_left = self._tri(err_deg, -45.0, -20.0, -5.0)
        zero = self._tri(err_deg, -8.0, 0.0, 8.0)
        small_right = self._tri(err_deg, 5.0, 20.0, 45.0)
        big_right = self._tri(err_deg, 30.0, 90.0, 180.0)
        return big_left, small_left, zero, small_right, big_right

    def adjust(self, left_v: float, right_v: float, distance: float, err_heading: float):
        nl, nm, nf = self._distance_memberships(distance)
        bl, sl, z, sr, br = self._heading_memberships(err_heading)

        forward_boost = (nf * 0.8 + nm * 0.2 - nl * 0.6) * self.max_forward_boost

        heading_magnitude = (bl * 1.0 + sl * 0.6 + sr * 0.6 + br * 1.0)
        heading_sign = (sr * 1.0 + br * 1.0) - (sl * 1.0 + bl * 1.0)
        if heading_magnitude != 0:
            heading_scale = heading_sign
        else:
            heading_scale = 0.0

        turn_delta = heading_scale * min(self.max_heading_adjust, abs(err_heading) / 30.0 * self.max_heading_adjust)

        new_left = left_v + forward_boost - turn_delta
        new_right = right_v + forward_boost + turn_delta

        return new_left, new_right


# ---------- Drivetrain ----------

class Drivetrain:
    def __init__(self, motors: List[HardwareMotor], gps: HardwareGPS):
        self.array_DT = list(motors)
        if len(self.array_DT) != 4:
            raise ValueError("Drivetrain requires 4 motors: [fl, rl, fr, rr]")
        self.fl, self.rl, self.fr, self.rr = self.array_DT
        self.gps = gps

        self.fuzzy = FuzzyController()
        self.distance_pid = PID(kp=1.2, ki=0.0, kd=0.2, integrator_limit=0.5)
        self.heading_pid = PID(kp=4.0, ki=0.0, kd=0.5, integrator_limit=1.0)

        self.max_voltage = 12.0
        self.wheel_base_m = 0.30

    def stop(self):
        for m in self.array_DT:
            m.stop()

    def tank_drive_voltage(self, left_v: float, right_v: float):
        lv = max(-self.max_voltage, min(self.max_voltage, left_v))
        rv = max(-self.max_voltage, min(self.max_voltage, right_v))
        self.array_DT[0].set_voltage(lv)
        self.array_DT[1].set_voltage(lv)
        self.array_DT[2].set_voltage(rv)
        self.array_DT[3].set_voltage(rv)

    def drive_to(self, target_x: float, target_y: float, timeout: float = 10.0):
        start = time.time()
        self.distance_pid.reset()
        self.heading_pid.reset()
        prev_time = start

        while True:
            now = time.time()
            dt = now - prev_time
            prev_time = now

            if now - start > timeout:
                break

            x, y = self.gps.get_position()
            heading = self.gps.get_heading()

            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)
            if distance < 0.05:
                break

            desired_heading = math.degrees(math.atan2(dy, dx)) % 360
            err_heading = desired_heading - heading
            if err_heading > 180:
                err_heading -= 360
            if err_heading < -180:
                err_heading += 360

            forward_cmd = self.distance_pid.update(distance, dt)
            turn_cmd = self.heading_pid.update(err_heading, dt)

            left_voltage = forward_cmd - (turn_cmd * self.wheel_base_m / 2.0)
            right_voltage = forward_cmd + (turn_cmd * self.wheel_base_m / 2.0)

            left_voltage, right_voltage = self.fuzzy.adjust(left_voltage, right_voltage, distance, err_heading)
            self.tank_drive_voltage(left_voltage, right_voltage)

            time.sleep(0.02)

        self.stop()


# ---------- Robot ----------

class VexRobot:
    def __init__(self):
        self.motor_fl = HardwareMotor(port=1, inverted=False)
        self.motor_rl = HardwareMotor(port=2, inverted=False)
        self.motor_fr = HardwareMotor(port=11, inverted=True)
        self.motor_rr = HardwareMotor(port=12, inverted=True)

        self.array_DT = [self.motor_fl, self.motor_rl, self.motor_fr, self.motor_rr]
        self.position_sensor = HardwarePositionSensor(port=20)

        self.radio = HardwareRadio(port=19)
        self.gps = HardwareGPS(port=6)
        self.vision = HardwareVision(port=7)

        self.bump_fl = HardwareBump(port=8)
        self.bump_fr = HardwareBump(port=9)
        self.bump_rl = HardwareBump(port=13)
        self.bump_rr = HardwareBump(port=14)

        self.drive = Drivetrain(self.array_DT, self.gps)

    def select_pin_color(self, timeout: float = 2.0) -> str:
        start = time.time()
        while time.time() - start < timeout:
            color = self.vision.get_color()
            if color in ("red", "blue"):
                return color
            time.sleep(0.05)
        return "unknown"

    def any_bump_pressed(self) -> bool:
        return any([
            self.bump_fl.is_pressed(),
            self.bump_fr.is_pressed(),
        ...existing code...
