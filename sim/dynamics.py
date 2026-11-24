"""6-DOF rigid-body dynamics with variable mass.

State x (14): r[0:3] CG position (world), v[3:6] velocity (world), q[6:10] attitude quaternion
(w, x, y, z) rotating body -> world, w[10:13] body angular rate, m[13] mass.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .environment import GRAVITY, density
from .vehicle import Vehicle


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    s = math.sin(angle / 2) / n
    return np.array([math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s])


def body_axis(q: np.ndarray) -> np.ndarray:
    """World direction of the body +z axis (engine plane towards nose)."""
    w, x, y, z = q
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


@dataclass
class Actuation:
    """Actuator outputs held constant over one integration step."""
    force_body: np.ndarray      # engine thrust, body frame [N]
    torque_body: np.ndarray     # engines + grid fins + RCS about CG, body frame [N m]
    mdot: float                 # propellant flow [kg/s] (positive = mass decreasing)
    thrusting: bool
    wind: np.ndarray            # world wind velocity [m/s]


def aero(vehicle: Vehicle, r: np.ndarray, v: np.ndarray, R: np.ndarray, w_body: np.ndarray,
         wind: np.ndarray, h_cg: float, thrusting: bool) -> tuple[np.ndarray, np.ndarray]:
    """Aerodynamic force (world) and torque about the CG (body)."""
    vrel = v - wind
    speed = float(np.linalg.norm(vrel))
    if speed < 1e-3:
        return np.zeros(3), np.zeros(3)
    b = R[:, 2]
    rho = density(r[2])
    va = float(vrel @ b) * b
    vn = vrel - va
    sin_a = float(np.linalg.norm(vn)) / speed
    cd = vehicle.cd_axial * (vehicle.plume_drag_factor if thrusting else 1.0)
    cn = vehicle.cn_alpha + vehicle.cn_cross * sin_a
    k = 0.5 * rho * vehicle.ref_area * speed
    f_axial = -k * cd * va
    f_normal = -k * cn * vn
    arm = (vehicle.h_cp - h_cg) * b
    torque_world = np.cross(arm, f_normal)
    torque_body = R.T @ torque_world
    # pitch/yaw damping
    damp = vehicle.cm_damp * 0.5 * rho * speed * vehicle.ref_area * vehicle.length ** 2
    torque_body[0] -= damp * w_body[0]
    torque_body[1] -= damp * w_body[1]
    return f_axial + f_normal, torque_body


def derivative(vehicle: Vehicle, x: np.ndarray, act: Actuation) -> np.ndarray:
    r, v, q, w, m = x[0:3], x[3:6], x[6:10], x[10:13], x[13]
    R = quat_to_mat(q)
    h_cg = vehicle.cg_height(m)
    f_aero, t_aero = aero(vehicle, r, v, R, w, act.wind, h_cg, act.thrusting)
    acc = GRAVITY + (R @ act.force_body + f_aero) / m
    inertia = vehicle.inertia(m)
    tau = act.torque_body + t_aero
    wdot = (tau - np.cross(w, inertia * w)) / inertia
    qdot = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
    out = np.empty(14)
    out[0:3] = v
    out[3:6] = acc
    out[6:10] = qdot
    out[10:13] = wdot
    out[13] = -act.mdot
    return out


def rk4_step(vehicle: Vehicle, x: np.ndarray, act: Actuation, dt: float) -> np.ndarray:
    k1 = derivative(vehicle, x, act)
    k2 = derivative(vehicle, x + 0.5 * dt * k1, act)
    k3 = derivative(vehicle, x + 0.5 * dt * k2, act)
    k4 = derivative(vehicle, x + dt * k3, act)
    xn = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    xn[6:10] /= np.linalg.norm(xn[6:10])
    return xn


def make_state(r, v, q=(1.0, 0.0, 0.0, 0.0), w=(0.0, 0.0, 0.0), m=0.0) -> np.ndarray:
    x = np.zeros(14)
    x[0:3] = r
    x[3:6] = v
    x[6:10] = q
    x[10:13] = w
    x[13] = m
    return x
