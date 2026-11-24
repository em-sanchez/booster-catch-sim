"""Engine cluster: per-engine ignition/shutdown, throttle response, common gimbal."""
from __future__ import annotations

import math

import numpy as np

from .environment import P0
from .vehicle import G0, GIMBAL_ENGINES, Vehicle, engine_layout


class EngineCluster:
    def __init__(self, vehicle: Vehicle, thrust_scale: float = 1.0, isp_scale: float = 1.0):
        self.v = vehicle
        self.spec = vehicle.engine
        layout = engine_layout()
        self.n = len(layout)
        self.pos = np.array([[x, y, 0.0] for x, y, _ in layout])
        self.ring = np.array([ring for _, _, ring in layout])
        self.gimballed = np.zeros(self.n, dtype=bool)
        self.gimballed[GIMBAL_ENGINES] = True
        self.thrust_scale = thrust_scale
        self.isp_scale = isp_scale
        self.lit = np.zeros(self.n, dtype=bool)
        self.level = np.zeros(self.n)          # actual thrust fraction of rated
        self.ign_time = np.full(self.n, -1e9)
        self.gimbal = np.zeros(2)              # (dx, dy) rad, shared by gimballed engines
        self.throttle = 0.0                    # last commanded throttle (clamped)

    # ---- commands ---------------------------------------------------------------------
    def ignite(self, idx, t: float) -> None:
        for i in idx:
            if not self.lit[i]:
                self.lit[i] = True
                self.ign_time[i] = t

    def shutdown(self, idx) -> None:
        for i in idx:
            self.lit[i] = False

    @property
    def n_lit(self) -> int:
        return int(self.lit.sum())

    @property
    def lit_mask(self) -> int:
        return int(sum(1 << i for i in range(self.n) if self.lit[i]))

    def rated_thrust(self) -> float:
        return self.spec.thrust_sl * self.thrust_scale

    def clamp_throttle(self, thr: float) -> float:
        return min(max(thr, self.spec.throttle_min), self.spec.throttle_max)

    def step(self, dt: float, t: float, throttle_cmd: float, gimbal_cmd: np.ndarray) -> None:
        s = self.spec
        self.throttle = self.clamp_throttle(throttle_cmd)
        ramp = np.clip((t - self.ign_time) / s.startup_time, 0.0, 1.0)
        target = np.where(self.lit, self.throttle * ramp, 0.0)
        tau = np.where(self.lit, s.throttle_tau, s.shutdown_tau)
        self.level += (target - self.level) * (1.0 - np.exp(-dt / tau))
        self.level[self.level < 1e-4] = 0.0
        # gimbal: rate then magnitude limit
        d = gimbal_cmd - self.gimbal
        dmax = s.gimbal_rate * dt
        nd = float(np.linalg.norm(d))
        if nd > dmax:
            d *= dmax / nd
        g = self.gimbal + d
        ng = float(np.linalg.norm(g))
        if ng > s.gimbal_max:
            g *= s.gimbal_max / ng
        self.gimbal = g

    # ---- outputs ------------------------------------------------------------------------
    def thrust_per_engine(self, p_ambient: float) -> np.ndarray:
        rated = self.rated_thrust()
        t = self.level * rated + np.where(self.level > 0, (P0 - p_ambient) * self.spec.exit_area, 0.0)
        return t

    def directions(self) -> np.ndarray:
        dx, dy = self.gimbal
        dg = np.array([math.tan(dx), math.tan(dy), 1.0])
        dg /= np.linalg.norm(dg)
        d = np.tile([0.0, 0.0, 1.0], (self.n, 1))
        d[self.gimballed] = dg
        return d

    def forces(self, h_cg: float, p_ambient: float) -> tuple[np.ndarray, np.ndarray, float, float]:
        """Return (force_body, torque_body about CG, mdot, total thrust magnitude)."""
        thrust = self.thrust_per_engine(p_ambient)
        f_each = self.directions() * thrust[:, None]
        arm = self.pos - np.array([0.0, 0.0, h_cg])
        torque = np.cross(arm, f_each).sum(axis=0)
        force = f_each.sum(axis=0)
        mdot = float((self.level * self.rated_thrust()).sum() / (self.spec.isp_sl * self.isp_scale * G0))
        return force, torque, mdot, float(thrust.sum())

    def gimballed_thrust(self, p_ambient: float) -> float:
        return float(self.thrust_per_engine(p_ambient)[self.gimballed].sum())
