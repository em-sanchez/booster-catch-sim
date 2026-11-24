"""Scenario definitions: start state, site geometry, wind and dispersions."""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from .vehicle import Vehicle


@dataclass(frozen=True)
class Site:
    """Launch-site geometry (sim frame: x east/offshore, y north, z up; catch axis at x=y=0)."""
    arm_height: float = 98.0          # top of the arm rails the hardpoints settle on [m]
    tower_x: float = -24.0            # tower centre
    tower_width: float = 12.0
    tower_height: float = 146.0
    arm_hinge_x: float = -18.0
    arm_hinge_y: float = 7.0
    arm_length: float = 32.0
    arm_open_deg: float = 40.0
    mount_height: float = 24.0        # launch-mount table
    mount_radius: float = 8.5
    shoreline_x: float = 260.0

    def to_meta(self) -> dict:
        return {
            "armHeight": self.arm_height, "towerX": self.tower_x, "towerWidth": self.tower_width,
            "towerHeight": self.tower_height, "armHingeX": self.arm_hinge_x,
            "armHingeY": self.arm_hinge_y, "armLength": self.arm_length,
            "armOpenDeg": self.arm_open_deg, "mountHeight": self.mount_height,
            "mountRadius": self.mount_radius, "shorelineX": self.shoreline_x,
        }


@dataclass(frozen=True)
class Scenario:
    id: str = "nominal"
    name: str = "Nominal"
    description: str = "Calm air, nominal vehicle."
    seed: int = 0
    r0: tuple[float, float, float] = (380.0, 30.0, 5000.0)      # CG start position [m]
    v0: tuple[float, float, float] = (-14.0, -2.0, -300.0)      # [m/s]
    prop0: float = 75e3                                         # landing propellant [kg] (estimate)
    wind_speed: float = 0.0
    wind_heading_deg: float = 0.0
    gust: float = 0.0
    thrust_scale: float = 1.0
    isp_scale: float = 1.0
    cd_scale: float = 1.0
    cn_scale: float = 1.0
    aim_x: float = 30.0               # offshore gate the grid-fin phase steers towards
    divert_x: float = 0.0             # stay x >= this until divert_time before catch (0 = off)
    divert_time: float = 0.0
    t13: float = 6.0                  # seconds on 13 engines before dropping to 3
    vehicle: Vehicle = field(default_factory=Vehicle)
    site: Site = field(default_factory=Site)

    def truth_vehicle(self) -> Vehicle:
        v = self.vehicle
        return replace(v, cd_axial=v.cd_axial * self.cd_scale, cn_alpha=v.cn_alpha * self.cn_scale,
                       cn_cross=v.cn_cross * self.cn_scale)

    def to_meta(self) -> dict:
        return {
            "id": self.id, "name": self.name, "description": self.description, "seed": self.seed,
            "windSpeed": self.wind_speed, "windHeadingDeg": self.wind_heading_deg, "gust": self.gust,
            "thrustScale": self.thrust_scale, "ispScale": self.isp_scale, "cdScale": self.cd_scale,
            "cnScale": self.cn_scale, "prop0": self.prop0,
        }


def nominal() -> Scenario:
    return Scenario()


def windy() -> Scenario:
    return Scenario(id="windy", name="Windy", seed=3,
                    description="12 m/s onshore wind at 100 m with gusts.",
                    wind_speed=12.0, wind_heading_deg=200.0, gust=2.5)


def dispersed(seed: int, name: str | None = None) -> Scenario:
    """Random start state, wind and vehicle errors the guidance does not know about."""
    rng = np.random.default_rng(seed)
    n = Scenario()
    r0 = np.array(n.r0) + rng.normal(0, [60.0, 60.0, 100.0])      # boostback delivery errors
    v0 = np.array(n.v0) + rng.normal(0, [5.0, 5.0, 8.0])
    return Scenario(
        id=f"dispersed-{seed}", name=name or f"Dispersed #{seed}", seed=seed,
        description="Random start state, wind, thrust, Isp and aerodynamic errors.",
        r0=tuple(r0), v0=tuple(v0),
        prop0=float(n.prop0 * rng.uniform(0.95, 1.05)),
        wind_speed=float(rng.uniform(0.0, 14.0)), wind_heading_deg=float(rng.uniform(0, 360)),
        gust=float(rng.uniform(0.0, 3.0)),
        thrust_scale=float(rng.normal(1.0, 0.02)), isp_scale=float(rng.normal(1.0, 0.01)),
        cd_scale=float(rng.normal(1.0, 0.12)), cn_scale=float(rng.normal(1.0, 0.15)),
    )
