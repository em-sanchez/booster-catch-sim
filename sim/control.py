"""Attitude control, actuator allocation (gimbal, grid fins, RCS) and translation loops."""
from __future__ import annotations

import math

import numpy as np

from .dynamics import quat_to_mat
from .environment import GRAVITY
from .vehicle import Vehicle


def clamp_tilt(direction: np.ndarray, max_tilt: float) -> np.ndarray:
    """Limit the angle between a unit direction and world up."""
    d = direction / np.linalg.norm(direction)
    tilt = math.acos(min(max(d[2], -1.0), 1.0))
    if tilt <= max_tilt:
        return d
    h = d[:2]
    nh = float(np.linalg.norm(h))
    if nh < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    return np.array([*(h / nh * math.sin(max_tilt)), math.cos(max_tilt)])


class AttitudeController:
    """Saturation-aware pointing control of the thrust axis.

    For small errors this is PD (tau = I (kp e - kd w)). For large errors the commanded
    turn rate is capped by a braking curve sqrt(2 a_max |e|), where a_max is the angular
    acceleration the actuators can actually deliver, so the vehicle never builds up more
    angular momentum than it can stop.
    """

    def __init__(self, vehicle: Vehicle, kp: float = 2.0, kd: float = 2.6, kd_roll: float = 1.0,
                 w_max: float = math.radians(10.0)):
        self.v = vehicle
        self.kp, self.kd, self.kd_roll, self.w_max = kp, kd, kd_roll, w_max

    def torque(self, q: np.ndarray, w: np.ndarray, b_des: np.ndarray, mass: float,
               alpha_max: float = 1.0) -> np.ndarray:
        R = quat_to_mat(q)
        b = R[:, 2]
        axis = np.cross(b, b_des)
        s = float(np.linalg.norm(axis))
        c = float(b @ b_des)
        ang = math.atan2(s, c)
        inertia = self.v.inertia(mass)
        if s > 1e-9:
            e_body = R.T @ (axis / s)
            rate = min(self.kp / self.kd * ang, math.sqrt(2 * 0.45 * max(alpha_max, 1e-4) * ang), self.w_max)
            w_des = e_body * rate
        else:
            w_des = np.zeros(3)
        tau = inertia * self.kd * (w_des - w)
        tau[2] = -inertia[2] * self.kd_roll * w[2]
        return tau


def gimbal_for_torque(tau: np.ndarray, thrust_gimballed: float, h_cg: float) -> np.ndarray:
    """Common gimbal angles (dx, dy) producing pitch/yaw torque about the CG.

    Lateral force F at the engine plane, CG a height h above it: tau_x = h*Fy, tau_y = -h*Fx.
    """
    if thrust_gimballed < 1.0:
        return np.zeros(2)
    fx = -tau[1] / h_cg
    fy = tau[0] / h_cg
    return np.array([math.atan(fx / thrust_gimballed), math.atan(fy / thrust_gimballed)])


def gimbal_torque(gimbal: np.ndarray, thrust_gimballed: float, h_cg: float) -> np.ndarray:
    fx = thrust_gimballed * math.tan(gimbal[0])
    fy = thrust_gimballed * math.tan(gimbal[1])
    return np.array([h_cg * fy, -h_cg * fx, 0.0])


class GridFins:
    """Four grid fins treated as torque actuators whose authority scales with dynamic pressure."""

    def __init__(self, vehicle: Vehicle):
        self.v = vehicle
        self.defl = np.zeros(len(vehicle.gridfin_azimuth))
        self.phi = np.array(vehicle.gridfin_azimuth)

    def effectiveness(self, qbar: float, h_cg: float) -> np.ndarray:
        k = qbar * self.v.gridfin_area * self.v.gridfin_cl_delta
        a = self.v.h_gridfin - h_cg
        return k * np.vstack([-a * np.cos(self.phi), -a * np.sin(self.phi),
                              self.v.radius * np.ones_like(self.phi)])

    def max_torque(self, qbar: float, h_cg: float) -> float:
        """Conservative pitch/yaw torque available from the fins."""
        k = qbar * self.v.gridfin_area * self.v.gridfin_cl_delta * self.v.gridfin_max
        return 2.0 * k * (self.v.h_gridfin - h_cg)

    def step(self, dt: float, tau_cmd: np.ndarray, qbar: float, h_cg: float) -> np.ndarray:
        A = self.effectiveness(qbar, h_cg)
        if qbar < 50.0:
            cmd = np.zeros_like(self.defl)
        else:
            cmd = np.linalg.pinv(A) @ tau_cmd
            peak = float(np.max(np.abs(cmd)))
            if peak > self.v.gridfin_max:
                cmd *= self.v.gridfin_max / peak
        d = np.clip(cmd - self.defl, -self.v.gridfin_rate * dt, self.v.gridfin_rate * dt)
        self.defl = self.defl + d
        return A @ self.defl


def roll_rcs(tau_z: float, vehicle: Vehicle) -> float:
    return float(np.clip(tau_z, -vehicle.rcs_roll_torque, vehicle.rcs_roll_torque))


def track_plan(r_ref, v_ref, u_ref, r, v, kp=0.25, kd=0.9) -> np.ndarray:
    """Thrust acceleration command: guidance feed-forward plus PD on the trajectory error."""
    return u_ref + kp * (r_ref - r) + kd * (v_ref - v)


def terminal_accel(r: np.ndarray, v: np.ndarray, r_target: np.ndarray, v_down: float,
                   drag: np.ndarray) -> np.ndarray:
    """Final metres: hold the centre of mass over the catch axis and sink at v_down.

    Gentle lateral gains on purpose: with a long, bottom-heavy booster, every gimbal push used to
    rotate it also shoves it sideways, so aggressive lateral loops wobble near hover thrust.
    """
    a = np.zeros(3)
    a[:2] = 0.12 * (r_target[:2] - r[:2]) + 0.6 * (0.0 - v[:2])
    a[2] = 1.5 * (-v_down - v[2])
    return a - GRAVITY - drag


def coast_attitude(v: np.ndarray, wind: np.ndarray, a_lat_des: np.ndarray, qbar: float,
                   mass: float, vehicle: Vehicle, max_aoa: float) -> np.ndarray:
    """Grid-fin phase: choose an angle of attack so body lift gives the wanted lateral accel.

    Falling engines-first, tilting the nose one way pushes the vehicle the other way.
    """
    vrel = v - wind
    sp = float(np.linalg.norm(vrel))
    if sp < 1.0:
        return np.array([0.0, 0.0, 1.0])
    flow_axis = -vrel / sp
    a_perp = a_lat_des - (a_lat_des @ flow_axis) * flow_axis
    na = float(np.linalg.norm(a_perp))
    if na < 1e-6 or qbar < 50.0:
        return flow_axis
    lift_per_rad = qbar * vehicle.ref_area * vehicle.cn_alpha / mass
    aoa = min(na / lift_per_rad, max_aoa)
    return math.cos(aoa) * flow_axis - math.sin(aoa) * a_perp / na


class LateralLQR:
    """Joint translation + attitude control of each lateral axis by engine gimbal.

    Small-angle model per world axis (x or y), state s = [position, velocity, tilt, tilt rate]:
        v'     = a_T (tilt + delta) - n_a * alpha       thrust + gimbal side-force + aero normal force
        tilt'' = -b_T delta - m_a * alpha               gimbal torque + aero restoring moment
        alpha  = tilt + v / V                           angle of attack (falling engines-first)
    a_T = T/m, b_T = T_gimballed h_cg / I, n_a = q S C_N_alpha / m, m_a = q S C_N_alpha l_cp / I.
    The side-force term matters: on a long booster with a low centre of mass, the push used to
    rotate it is also a large sideways shove. The aero terms matter early in the burn, when the
    aerodynamic stiffness is about as strong as the gimbal. Gains come from the continuous
    algebraic Riccati equation, recomputed as thrust, mass and dynamic pressure change.
    """

    def __init__(self, pos: float = 4.0, vel: float = 2.0, tilt: float = 0.08, rate: float = 0.06,
                 gimbal: float = 0.12):
        self.Q = np.diag([1 / pos ** 2, 1 / vel ** 2, 1 / tilt ** 2, 1 / rate ** 2])
        self.R = np.array([[1 / gimbal ** 2]])
        self.K = None
        self._age = 1e9

    def update(self, a_t: float, b_t: float, n_a: float, m_a: float, speed: float, dt: float,
               every: float = 0.05) -> None:
        self._age += dt
        if self.K is not None and self._age < every:
            return
        from scipy.linalg import solve_continuous_are
        a_t, b_t = max(a_t, 0.5), max(b_t, 0.01)
        V = max(speed, 1.0)
        A = np.array([[0, 1, 0, 0],
                      [0, -n_a / V, a_t - n_a, 0],
                      [0, 0, 0, 1],
                      [0, -m_a / V, -m_a, 0]], float)
        B = np.array([[0], [a_t], [0], [-b_t]], float)
        P = solve_continuous_are(A, B, self.Q, self.R)
        self.K = (np.linalg.inv(self.R) @ B.T @ P).ravel()
        self._age = 0.0

    def command(self, e_pos: np.ndarray, e_vel: np.ndarray, tilt: np.ndarray, tilt_ref: np.ndarray,
                tilt_rate: np.ndarray, delta_max: float, pos_clip: float = 6.0,
                vel_clip: float = 4.0) -> np.ndarray:
        """World-frame gimbal deflection (x, y) [rad]."""
        K = self.K
        ep = np.clip(e_pos, -pos_clip, pos_clip)
        ev = np.clip(e_vel, -vel_clip, vel_clip)
        d = np.array([-(K @ np.array([ep[j], ev[j], tilt[j] - tilt_ref[j], tilt_rate[j]]))
                      for j in range(2)])
        n = float(np.linalg.norm(d))
        if n > delta_max:
            d *= delta_max / n
        return d
