import math

import numpy as np
import pytest

from sim.guidance import GuidanceSchedule, PoweredDescentGuidance, TargetSpec
from sim.vehicle import Vehicle

VEH = Vehicle()
E = VEH.engine


def setup():
    g = PoweredDescentGuidance(N=30, isp=E.isp_sl, vehicle=VEH, mass_dry=VEH.dry_mass)
    target = TargetSpec(r=np.array([0.0, 0.0, 66.0]), v=np.array([0.0, 0.0, -3.3]), divert_x=0.0,
                        divert_time=0.0, keepout_x=-1.0, tilt_max=math.radians(20),
                        tilt_final=math.radians(3))
    sched = GuidanceSchedule(t_switch=6.0, n_before=13, n_after=3, thrust_engine=E.thrust_sl,
                             throttle_min=E.throttle_min, throttle_max=0.80, throttle_max_before=0.50,
                             throttle_min_after=0.50)
    return g, target, sched


def wind(_z):
    return np.zeros(3)


@pytest.fixture(scope="module")
def plan():
    g, target, sched = setup()
    p = g.search_tgo(0.0, np.array([200.0, 10.0, 1600.0]), np.array([-15.0, -1.0, -305.0]), 330e3,
                     lambda _t: sched, target, wind, t_lo=8.0, t_hi=30.0, step=1.0)
    assert p is not None
    return p, target, sched


def test_reaches_target(plan):
    p, target, _ = plan
    assert p.terminal_error < 0.05
    assert np.allclose(p.r[:, -1], target.r, atol=0.05)
    assert np.allclose(p.v[:, -1], target.v, atol=0.05)


def test_thrust_bounds_respected(plan):
    """Modelled thrust magnitude (sigma * m) stays inside the engine-count-dependent bounds."""
    p, _, sched = plan
    times = p.t0 + p.dt * np.arange(p.r.shape[1])
    thrust = p.sigma * p.m
    assert np.all(np.linalg.norm(p.u, axis=0) <= p.sigma * (1 + 1e-6))
    n = np.where(times < sched.t_switch, sched.n_before, sched.n_after)
    tmax = np.where(times < sched.t_switch, sched.throttle_max_before, sched.throttle_max)
    upper = n * sched.thrust_engine * tmax
    lower = n * sched.thrust_engine * sched.throttle_min
    assert np.all(thrust <= upper * 1.001)
    assert np.all(thrust >= lower * 0.999)


def test_tilt_mass_and_keepout(plan):
    p, target, _ = plan
    un = np.linalg.norm(p.u, axis=0)
    tilt = np.degrees(np.arccos(np.clip(p.u[2] / un, -1, 1)))
    assert tilt.max() <= math.degrees(target.tilt_max) + 0.05
    assert tilt[-2:].max() <= math.degrees(target.tilt_final) + 0.05
    assert np.all(np.diff(p.m) <= 1e-6)                 # mass only decreases
    assert p.m[-1] >= VEH.dry_mass
    assert np.all(p.r[0] >= target.keepout_x - 1e-3)    # never on the tower side
    assert np.all(p.r[2, 1:] >= target.r[2] - 0.6)      # never below the gate


def test_lossless_convexification_is_tight():
    """The relaxed slack sigma must equal |u| at the optimum (the 'lossless' part).

    Checked on a single engine-count phase, where the theory applies. Just before a forced
    13 -> 3 engine switch the minimum thrust of 13 engines can exceed what the plan wants, and
    the relaxation is then not tight there (see README)."""
    g, target, _ = setup()
    sched = GuidanceSchedule(t_switch=-1.0, n_before=3, n_after=3, thrust_engine=E.thrust_sl,
                             throttle_min=E.throttle_min, throttle_max=0.80)
    p = g.solve(0.0, 12.0, np.array([40.0, 0.0, 400.0]), np.array([-6.0, 0.0, -45.0]), 290e3,
                sched, target, wind)
    assert p is not None
    s = g.vars["s"].value
    un = np.linalg.norm(g.vars["u"].value, axis=0)
    assert np.max(np.abs(s - un) / s) < 1e-3


def test_too_late_ignition_cannot_reach():
    g, target, sched = setup()
    p = g.solve(0.0, 6.0, np.array([0.0, 0.0, 400.0]), np.array([0.0, 0.0, -300.0]), 330e3,
                sched, target, wind)
    assert p is None or p.terminal_error > 1.0
