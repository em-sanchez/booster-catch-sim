import math
from dataclasses import replace

import numpy as np

from sim.dynamics import Actuation, body_axis, make_state, quat_from_axis_angle, rk4_step
from sim.engines import EngineCluster
from sim.environment import GRAVITY, pressure
from sim.vehicle import CENTER_ENGINES, G0, Vehicle


def vacuum_vehicle() -> Vehicle:
    return replace(Vehicle(), cd_axial=0.0, cn_alpha=0.0, cn_cross=0.0, cm_damp=0.0)


def idle(wind=np.zeros(3)) -> Actuation:
    return Actuation(np.zeros(3), np.zeros(3), 0.0, False, wind)


def test_free_fall_matches_analytic_and_conserves_energy():
    veh = vacuum_vehicle()
    x = make_state([10.0, -5.0, 3000.0], [20.0, 3.0, -50.0], m=300e3)
    e0 = 0.5 * x[3:6] @ x[3:6] - GRAVITY[2] * x[2]
    dt, T = 0.01, 10.0
    for _ in range(int(T / dt)):
        x = rk4_step(veh, x, idle(), dt)
    expect_z = 3000.0 - 50.0 * T + 0.5 * GRAVITY[2] * T * T
    assert abs(x[2] - expect_z) < 1e-6
    assert abs(x[0] - (10.0 + 20.0 * T)) < 1e-6
    e1 = 0.5 * x[3:6] @ x[3:6] - GRAVITY[2] * x[2]
    assert abs(e1 - e0) / abs(e0) < 1e-9


def test_torque_free_rotation_conserves_angular_momentum():
    veh = vacuum_vehicle()
    q0 = quat_from_axis_angle(np.array([0.3, 1.0, 0.0]), 0.4)
    x = make_state([0, 0, 1000], [0, 0, 0], q0, [0.02, -0.03, 0.4], m=280e3)
    inertia = veh.inertia(x[13])

    def h_world(x):
        from sim.dynamics import quat_to_mat
        return quat_to_mat(x[6:10]) @ (inertia * x[10:13])

    h0 = h_world(x)
    for _ in range(2000):
        x = rk4_step(veh, x, idle(), 0.01)
    assert np.linalg.norm(h_world(x) - h0) / np.linalg.norm(h0) < 1e-6
    assert abs(np.linalg.norm(x[6:10]) - 1.0) < 1e-12


def test_aero_restores_engines_first_attitude():
    """Falling engines-first with grid fins on top is aerodynamically stable."""
    veh = Vehicle()
    q0 = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), math.radians(8))
    x = make_state([0, 0, 3000], [0, 0, -280], q0, m=300e3)
    for _ in range(300):
        x = rk4_step(veh, x, idle(), 0.01)
    tilt = math.degrees(math.acos(body_axis(x[6:10])[2]))
    assert tilt < 8.0


def test_engine_limits_and_mass_flow():
    veh = Vehicle()
    eng = EngineCluster(veh)
    eng.ignite(CENTER_ENGINES, 0.0)
    spec = veh.engine
    t, dt = 0.0, 0.01
    prev = eng.gimbal.copy()
    for i in range(400):
        cmd = 5.0 if i % 2 else -3.0                     # absurd throttle commands
        g_cmd = np.array([1.0, -1.0]) * (1 if i < 200 else -1)
        eng.step(dt, t, cmd, g_cmd)
        assert spec.throttle_min - 1e-12 <= eng.throttle <= spec.throttle_max + 1e-12
        assert np.linalg.norm(eng.gimbal) <= spec.gimbal_max + 1e-12
        assert np.linalg.norm(eng.gimbal - prev) <= spec.gimbal_rate * dt + 1e-12
        prev = eng.gimbal.copy()
        t += dt
    eng.step(dt, t, 0.7, np.zeros(2))
    for _ in range(300):
        eng.step(dt, t, 0.7, np.zeros(2))
    _, _, mdot, thrust = eng.forces(veh.cg_height(300e3), pressure(0.0))
    assert abs(eng.level[CENTER_ENGINES].mean() - 0.7) < 1e-3
    assert abs(mdot - 3 * 0.7 * spec.thrust_sl / (spec.isp_sl * G0)) / mdot < 1e-2
    assert eng.n_lit == 3
