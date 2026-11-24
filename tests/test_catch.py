"""Closed-loop tests: full 6-DOF flights with guidance, control and dispersions."""
import math
import os
import warnings
from multiprocessing import Pool

import pytest

from sim import scenario as S
from sim.flight import Flight
from sim.vehicle import Vehicle


def run(sc):
    warnings.filterwarnings("ignore")
    return Flight(sc, record=False).run().summary()


@pytest.fixture(scope="module")
def nominal():
    return run(S.nominal())


def test_nominal_catch(nominal):
    assert nominal["caught"]
    assert math.hypot(nominal["catch_error_x"], nominal["catch_error_y"]) < 1.0
    assert -nominal["catch_vz"] < 1.5
    assert nominal["catch_tilt_deg"] < 2.0


def test_nominal_respects_limits(nominal):
    veh = Vehicle()
    assert nominal["max_gimbal_deg"] <= math.degrees(veh.engine.gimbal_max) + 1e-6
    assert nominal["max_throttle"] <= veh.engine.throttle_max + 1e-9
    assert nominal["min_throttle_lit"] >= veh.engine.throttle_min - 1e-9
    assert nominal["min_x_burn"] > -6.0          # stays clear of the tower side
    assert nominal["prop_left"] > 5e3
    assert 1000 < nominal["ignition_altitude"] < 2500


def test_monte_carlo_catch_rate():
    seeds = range(2000, 2012)
    with Pool(min(len(seeds), max(1, (os.cpu_count() or 2) - 1))) as pool:
        results = pool.map(run, [S.dispersed(s) for s in seeds])
    caught = [r for r in results if r["caught"]]
    assert len(caught) >= 11, [(r["scenario_id"], r["catch_error_x"], r["catch_error_y"]) for r in results]
    for r in caught:
        assert math.hypot(r["catch_error_x"], r["catch_error_y"]) < 4.0
