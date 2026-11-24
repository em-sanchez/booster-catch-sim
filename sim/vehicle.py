"""Booster parameters.

Every number here is an approximate public figure or an engineering estimate chosen to be
plausible. None of it is official data. Body frame: origin at the centre of the engine plane,
+z along the vehicle axis towards the nose.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G0 = 9.80665


@dataclass(frozen=True)
class EngineSpec:
    thrust_sl: float = 2.26e6                  # N, ~230 tf sea-level (widely reported, approx.)
    throttle_min: float = 0.35                 # estimate; deep throttling lets 3 engines hover
    throttle_max: float = 1.0
    isp_sl: float = 327.0                      # s, sea-level Isp (reported, approx.)
    gimbal_max: float = math.radians(15.0)     # estimate
    gimbal_rate: float = math.radians(45.0)    # rad/s, estimate
    throttle_tau: float = 0.20                 # s, first-order throttle response (estimate)
    startup_time: float = 0.8                  # s, ignition to commanded thrust (estimate)
    shutdown_tau: float = 0.12                 # s
    exit_diameter: float = 1.15                # m nozzle exit (estimate)

    @property
    def exit_area(self) -> float:
        return math.pi * (self.exit_diameter / 2) ** 2


# Engine rings: 0 = centre 3 (gimbal), 1 = inner 10 (gimbal), 2 = outer 20 (fixed).
RING_RADIUS = (1.05, 2.55, 3.90)
RING_COUNT = (3, 10, 20)


def engine_layout() -> list[tuple[float, float, int]]:
    out: list[tuple[float, float, int]] = []
    offsets = (math.pi / 2, math.pi / 10, 0.0)
    for ring, (r, n, off) in enumerate(zip(RING_RADIUS, RING_COUNT, offsets)):
        for i in range(n):
            a = off + 2 * math.pi * i / n
            out.append((r * math.cos(a), r * math.sin(a), ring))
    return out


CENTER_ENGINES = list(range(0, 3))
INNER_ENGINES = list(range(3, 13))
OUTER_ENGINES = list(range(13, 33))
GIMBAL_ENGINES = CENTER_ENGINES + INNER_ENGINES


@dataclass(frozen=True)
class Vehicle:
    length: float = 71.0              # m
    diameter: float = 9.0             # m
    dry_mass: float = 255e3           # kg, estimate (public estimates span ~200-275 t)
    h_cg_dry: float = 27.0            # m above engine plane, empty (engines pull CG low)
    h_prop: float = 12.0              # m, CG of residual landing propellant (estimate)
    h_hardpoint: float = 64.0         # m, catch pins that rest on the tower arms
    h_gridfin: float = 66.5           # m
    h_cp: float = 44.0                # m, centre of pressure falling engines-first (estimate)
    cd_axial: float = 1.1             # axial drag coefficient, engines-first (estimate)
    cn_alpha: float = 2.0             # normal-force slope per rad (slender-body theory)
    cn_cross: float = 10.0            # extra crossflow normal force at larger angles
    cm_damp: float = 0.08             # aerodynamic pitch damping coefficient
    plume_drag_factor: float = 0.6    # drag reduction while the plume shields the base
    gridfin_area: float = 6.0         # m^2 each
    gridfin_cl_delta: float = 2.0     # per rad
    gridfin_max: float = math.radians(25.0)
    gridfin_rate: float = math.radians(60.0)
    gridfin_azimuth: tuple[float, ...] = (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)
    rcs_roll_torque: float = 3.0e5    # N m
    engine: EngineSpec = field(default_factory=EngineSpec)

    @property
    def radius(self) -> float:
        return self.diameter / 2

    @property
    def ref_area(self) -> float:
        return math.pi * self.radius ** 2

    def cg_height(self, mass: float) -> float:
        prop = max(mass - self.dry_mass, 0.0)
        return (self.dry_mass * self.h_cg_dry + prop * self.h_prop) / (self.dry_mass + prop)

    def inertia(self, mass: float) -> np.ndarray:
        """Diagonal inertia about the CG (thin-walled tank approximation)."""
        r, L = self.radius, self.length
        lat = 0.85 * mass * (3 * r * r + L * L) / 12.0
        roll = 0.9 * mass * r * r / 2.0
        return np.array([lat, lat, roll])

    def to_meta(self) -> dict:
        return {
            "length": self.length,
            "diameter": self.diameter,
            "dryMass": self.dry_mass,
            "hHardpoint": self.h_hardpoint,
            "hGridfin": self.h_gridfin,
            "engineThrust": self.engine.thrust_sl,
            "throttleMin": self.engine.throttle_min,
            "gimbalMaxDeg": math.degrees(self.engine.gimbal_max),
            "engineExitDiameter": self.engine.exit_diameter,
            "engines": [{"x": round(x, 4), "y": round(y, 4), "ring": ring} for x, y, ring in engine_layout()],
            "gridfinAzimuthDeg": [math.degrees(a) for a in self.gridfin_azimuth],
        }
