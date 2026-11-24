"""Gravity, exponential atmosphere and wind. Frame: x east (offshore), y north, z up [m]."""
from __future__ import annotations

import math

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])   # flat Earth is fine over a few km
RHO0 = 1.225        # kg/m^3
P0 = 101325.0       # Pa
SCALE_HEIGHT = 8500.0


def density(z: float) -> float:
    return RHO0 * math.exp(-max(z, 0.0) / SCALE_HEIGHT)


def pressure(z: float) -> float:
    return P0 * math.exp(-max(z, 0.0) / SCALE_HEIGHT)


class Wind:
    """Power-law mean wind plus Ornstein-Uhlenbeck gusts.

    speed is the mean wind at 100 m; heading_deg is the direction the wind blows towards,
    measured from +x (east) counter-clockwise.
    """

    def __init__(self, speed: float = 0.0, heading_deg: float = 0.0, gust: float = 0.0,
                 gust_tau: float = 3.0, seed: int = 0):
        h = math.radians(heading_deg)
        self.dir = np.array([math.cos(h), math.sin(h), 0.0])
        self.speed = speed
        self.gust_sigma = gust
        self.gust_tau = gust_tau
        self.rng = np.random.default_rng(seed)
        self.gust_state = np.zeros(3)

    def step(self, dt: float) -> None:
        if self.gust_sigma <= 0:
            return
        a = math.exp(-dt / self.gust_tau)
        noise = self.rng.standard_normal(3) * np.array([1.0, 1.0, 0.3])
        self.gust_state = a * self.gust_state + self.gust_sigma * math.sqrt(1 - a * a) * noise

    def mean_at(self, z: float) -> np.ndarray:
        if self.speed <= 0:
            return np.zeros(3)
        return self.dir * self.speed * (max(z, 2.0) / 100.0) ** (1.0 / 7.0)

    def at(self, z: float) -> np.ndarray:
        g = self.gust_state * min(1.0, max(z, 0.0) / 30.0 + 0.2)
        return self.mean_at(z) + g
