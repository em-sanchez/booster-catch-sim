"""Convex powered-descent guidance (lossless convexification).

Follows Acikmese & Ploen (2007) and Blackmore, Acikmese & Scharf (2010): with u = T/m,
sigma >= |u| and z = ln m, the non-convex thrust bounds rho1 <= |T| <= rho2 become
second-order-cone constraints. The problem is built once with cvxpy Parameters (DPP) so
every closed-loop re-solve only updates numbers and calls the Clarabel interior-point solver.

Extras used here:
- time-varying thrust bounds per node (13 engines, then 3 engines),
- a known external acceleration per node (gravity + drag estimated along the previous plan),
- tilt (pointing) limits, a glide-slope cone, a tower keep-out half-space and an offshore
  "divert" gate that keeps the booster away from the tower until the final seconds,
- soft terminal constraints (heavily weighted slack) so a re-solve never returns nothing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cvxpy as cp
import numpy as np

from .environment import GRAVITY, density


@dataclass
class GuidanceSchedule:
    """Engine schedule, in absolute sim time."""
    t_switch: float          # time the engine count drops
    n_before: int            # engines lit before t_switch
    n_after: int             # engines lit after t_switch
    thrust_engine: float     # rated thrust per engine [N]
    throttle_min: float
    throttle_max: float      # after t_switch; < 1 leaves control authority
    throttle_max_before: float = 0.65   # 13-engine phase cap (limits deceleration to ~4-5 g)
    throttle_min_after: float | None = None  # planned floor on 3 engines (keeps "fall faster" authority)


@dataclass
class TargetSpec:
    r: np.ndarray            # target CG position
    v: np.ndarray            # target velocity
    divert_x: float          # x >= divert_x while time-to-go > divert_time
    divert_time: float
    keepout_x: float         # x >= keepout_x always (tower side)
    tilt_max: float          # rad
    tilt_final: float        # rad, last nodes
    approach_time: float = 3.0                 # s before catch: final-approach limits from here
    tilt_approach: float = math.radians(8.0)
    approach_speed: float = 2.0                # m/s lateral speed cap on final approach
    funnel_radius: float = 0.5                 # m, lateral offset allowed at the gate
    final_time: float = 1.0                    # s before catch: near-vertical thrust
    lateral_jerk: float = 1.0                  # m/s^3, rate the lateral thrust accel may change
    lateral_weight: float = 1e-4               # weight on lateral accel energy (vs fuel)


@dataclass
class Plan:
    t0: float                # absolute time of node 0
    dt: float
    r: np.ndarray            # (3, N+1)
    v: np.ndarray
    u: np.ndarray            # thrust acceleration
    m: np.ndarray
    a_ext: np.ndarray        # (3, N) external accel per interval
    terminal_error: float
    fuel: float
    status: str
    violation: float = 0.0
    sigma: np.ndarray | None = None   # thrust-acceleration magnitude slack (|u| <= sigma)

    @property
    def t_final(self) -> float:
        return self.t0 + self.dt * (self.r.shape[1] - 1)

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Reference (r, v, u) at absolute time t using the exact intra-interval model."""
        N = self.r.shape[1] - 1
        tau_total = min(max(t - self.t0, 0.0), self.dt * N)
        k = min(int(tau_total / self.dt), N - 1)
        tau = tau_total - k * self.dt
        uk, uk1, ae = self.u[:, k], self.u[:, k + 1], self.a_ext[:, k]
        du = (uk1 - uk) / self.dt
        u = uk + du * tau
        v = self.v[:, k] + (uk + ae) * tau + du * tau * tau / 2
        r = self.r[:, k] + self.v[:, k] * tau + (uk + ae) * tau * tau / 2 + du * tau ** 3 / 6
        return r, v, u


class PoweredDescentGuidance:
    def __init__(self, N: int = 30, isp: float = 327.0, glide_slope_deg: float = 60.0,
                 vehicle=None, mass_dry: float = 255e3):
        self.N = N
        self.alpha = 1.0 / (isp * 9.80665)
        self.vehicle = vehicle
        self.mass_dry = mass_dry
        n = N + 1
        r = cp.Variable((3, n))
        v = cp.Variable((3, n))
        u = cp.Variable((3, n))
        s = cp.Variable(n)
        z = cp.Variable(n)
        w = cp.Variable(n)
        er = cp.Variable(3)
        ev = cp.Variable(3)
        du0 = cp.Variable(3)
        sl_gs = cp.Variable(N, nonneg=True)     # soft glide slope
        sl_x = cp.Variable(n, nonneg=True)      # soft divert / keep-out
        sl_v = cp.Variable(n, nonneg=True)      # soft final-approach lateral speed limit
        sl_f = cp.Variable(n, nonneg=True)      # soft final-approach funnel

        p = {
            "r0": cp.Parameter(3), "v0": cp.Parameter(3), "z0": cp.Parameter(),
            "dt": cp.Parameter(nonneg=True), "dt2_3": cp.Parameter(nonneg=True),
            "dt2_6": cp.Parameter(nonneg=True), "adt": cp.Parameter(nonneg=True),
            "dv_ext": cp.Parameter((3, N)), "dr_ext": cp.Parameter((3, N)),
            "zref": cp.Parameter(n), "mu1": cp.Parameter(n, nonneg=True),
            "mu2": cp.Parameter(n, nonneg=True), "cos_tilt": cp.Parameter(n, nonneg=True),
            "x_lb": cp.Parameter(n), "r_t": cp.Parameter(3), "v_t": cp.Parameter(3),
            "u_prev": cp.Parameter(3), "w_prev": cp.Parameter(nonneg=True),
            "z_dry": cp.Parameter(), "vxy_max": cp.Parameter(n, nonneg=True),
            "jdt": cp.Parameter(nonneg=True), "w_lat": cp.Parameter(nonneg=True),
            "funnel": cp.Parameter(n, nonneg=True),
        }
        self.p = p
        ones = np.ones((1, n))
        gs = math.tan(math.radians(glide_slope_deg))
        cons = [
            r[:, 0] == p["r0"], v[:, 0] == p["v0"], z[0] == p["z0"],
            v[:, 1:] == v[:, :-1] + (p["dt"] / 2) * (u[:, :-1] + u[:, 1:]) + p["dv_ext"],
            r[:, 1:] == r[:, :-1] + p["dt"] * v[:, :-1] + p["dt2_3"] * u[:, :-1]
            + p["dt2_6"] * u[:, 1:] + p["dr_ext"],
            z[1:] == z[:-1] - p["adt"] * (s[:-1] + s[1:]),
            w == z - p["zref"],
            cp.norm(u, axis=0) <= s,
            s >= cp.multiply(p["mu1"], 1 - w) + 0.5 * cp.multiply(p["mu1"], cp.square(w)),
            s <= cp.multiply(p["mu2"], 1 - w),
            z >= p["zref"],
            z[N] >= p["z_dry"],
            u[2, :] >= cp.multiply(p["cos_tilt"], s),
            r[0, :] >= p["x_lb"] - sl_x,
            cp.norm(v[:2, :], axis=0) <= p["vxy_max"] + sl_v,
            # funnel: lateral offset allowed shrinks with time-to-go on final approach
            cp.norm(r[:2, :] - cp.reshape(p["r_t"][:2], (2, 1), order="F") @ ones, axis=0)
            <= p["funnel"] + sl_f,
            gs * cp.norm(r[:2, 1:] - cp.reshape(p["r_t"][:2], (2, 1), order="F") @ ones[:, 1:], axis=0)
            <= r[2, 1:] - p["r_t"][2] + 0.5 + sl_gs,
            r[:, N] - p["r_t"] == er,
            v[:, N] - p["v_t"] == ev,
            du0 == u[:, 0] - p["u_prev"],
            # lateral jerk limit: the thrust direction can only turn as fast as the vehicle
            cp.norm(u[:2, 1:] - u[:2, :-1], axis=0) <= p["jdt"],
        ]
        # reaching the target dominates; shaping constraints are soft and much cheaper so that a
        # disturbed state outside the cone/funnel still gets a plan that ends at the arms
        obj = (-z[N] + 200.0 * cp.norm1(er) + 200.0 * cp.norm1(ev)
               + 2.0 * cp.sum(sl_gs) + 20.0 * cp.sum(sl_x) + 1.0 * cp.sum(sl_v) + 1.0 * cp.sum(sl_f)
               + 2e-4 * cp.sum_squares(u[:, 1:] - u[:, :-1])
               # smooth lateral profile: penalise lateral thrust-acceleration energy
               + p["w_lat"] * cp.sum_squares(u[:2, :])
               + p["w_prev"] * cp.sum_squares(du0))
        self.problem = cp.Problem(cp.Minimize(obj), cons)
        assert self.problem.is_dcp(dpp=True)
        self.vars = dict(r=r, v=v, u=u, s=s, z=z, er=er, ev=ev, sl_gs=sl_gs, sl_x=sl_x, sl_v=sl_v, sl_f=sl_f)
        self.solve_count = 0

    # ------------------------------------------------------------------------------------
    def drag_accel(self, r: np.ndarray, v: np.ndarray, m: np.ndarray, wind_fn, thrusting=True) -> np.ndarray:
        """Aerodynamic acceleration along a trajectory (columns), same model as the physics with the
        body axis taken as vertical: axial drag plus crossflow normal force (crosswind matters on a
        71 m tall body)."""
        veh = self.vehicle
        out = np.zeros_like(v)
        if veh is None:
            return out
        cd = veh.cd_axial * (veh.plume_drag_factor if thrusting else 1.0)
        for k in range(v.shape[1]):
            vrel = v[:, k] - wind_fn(r[2, k])
            sp = float(np.linalg.norm(vrel))
            if sp < 1e-3:
                continue
            va = np.array([0.0, 0.0, vrel[2]])
            vn = vrel - va
            cn = veh.cn_alpha + veh.cn_cross * float(np.linalg.norm(vn)) / sp
            out[:, k] = -0.5 * density(r[2, k]) * veh.ref_area * sp * (cd * va + cn * vn) / m[k]
        return out

    def solve(self, t0: float, t_go: float, r0, v0, m0: float, sched: GuidanceSchedule,
              target: TargetSpec, wind_fn, guess: Plan | None = None,
              u_prev: np.ndarray | None = None, w_prev: float = 0.0) -> Plan | None:
        N = self.N
        n = N + 1
        dt = t_go / N
        times = t0 + dt * np.arange(n)
        n_eng = np.where(times < sched.t_switch, sched.n_before, sched.n_after)
        tgo_nodes = t0 + t_go - times
        # planned throttle floor: raised during the burn (keeps authority to descend faster),
        # the true minimum on final approach so the booster can slow to a near-hover
        tmin_after = np.where(tgo_nodes < target.approach_time, sched.throttle_min * 1.03,
                              sched.throttle_min_after or sched.throttle_min)
        rho1 = n_eng * sched.thrust_engine * np.where(times < sched.t_switch, sched.throttle_min * 1.03,
                                                       tmin_after)
        rho2 = n_eng * sched.thrust_engine * np.where(times < sched.t_switch,
                                                       sched.throttle_max_before, sched.throttle_max)
        # max-thrust mass profile -> reference for the Taylor expansions
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (rho2[:-1] + rho2[1:]) * dt)])
        m_ref = np.maximum(m0 - self.alpha * cum, 0.3 * m0)
        zref = np.log(m_ref)

        # external acceleration: gravity + drag along a guessed trajectory
        if guess is not None:
            rg = np.stack([guess.sample(t)[0] for t in times], axis=1)
            vg = np.stack([guess.sample(t)[1] for t in times], axis=1)
            mg = np.interp(times, guess.t0 + guess.dt * np.arange(guess.m.size), guess.m)
        else:
            frac = np.linspace(0.0, 1.0, n)
            rg = np.outer(r0, 1 - frac) + np.outer(target.r, frac)
            vg = np.outer(v0, 1 - frac) + np.outer(target.v, frac)
            mg = m0 - (m0 - m_ref[-1]) * 0.5 * frac
        a_ext = self.drag_accel(rg, vg, mg, wind_fn)[:, :N] + GRAVITY[:, None]

        tgo_nodes = t0 + t_go - times
        x_lb = np.where(tgo_nodes > target.divert_time, target.divert_x, target.keepout_x)
        x_lb[0] = min(x_lb[0], r0[0] - 1.0)   # never infeasible at the current state
        cos_tilt = np.where(tgo_nodes > target.approach_time, math.cos(target.tilt_max),
                            math.cos(target.tilt_approach))
        cos_tilt[tgo_nodes < target.final_time] = math.cos(target.tilt_final)
        vxy_max = np.where(tgo_nodes > target.approach_time, 1e3, target.approach_speed)
        vxy_max[0] = max(vxy_max[0], float(np.linalg.norm(np.asarray(v0)[:2])) + 1.0)

        p = self.p
        p["r0"].value = np.asarray(r0, float)
        p["v0"].value = np.asarray(v0, float)
        p["z0"].value = math.log(m0)
        p["dt"].value = dt
        p["dt2_3"].value = dt * dt / 3
        p["dt2_6"].value = dt * dt / 6
        p["adt"].value = self.alpha * dt / 2
        p["dv_ext"].value = dt * a_ext
        p["dr_ext"].value = dt * dt / 2 * a_ext
        p["zref"].value = zref
        p["mu1"].value = rho1 * np.exp(-zref)
        p["mu2"].value = rho2 * np.exp(-zref)
        p["cos_tilt"].value = cos_tilt
        p["x_lb"].value = x_lb
        p["r_t"].value = target.r
        p["v_t"].value = target.v
        p["u_prev"].value = np.zeros(3) if u_prev is None else u_prev
        p["w_prev"].value = w_prev
        p["z_dry"].value = math.log(self.mass_dry)
        p["vxy_max"].value = vxy_max
        p["jdt"].value = target.lateral_jerk * dt
        p["w_lat"].value = target.lateral_weight * dt
        funnel = np.where(tgo_nodes > target.approach_time, 1e4,
                          target.funnel_radius + target.approach_speed * tgo_nodes)
        funnel[0] = max(funnel[0], float(np.linalg.norm(np.asarray(r0)[:2] - target.r[:2])) + 1.0)
        p["funnel"].value = funnel
        try:
            self.problem.solve(solver=cp.CLARABEL)
        except cp.SolverError:
            return None
        self.solve_count += 1
        if self.problem.status not in ("optimal", "optimal_inaccurate"):
            return None
        V = self.vars
        m = np.exp(V["z"].value)
        err = float(np.linalg.norm(V["er"].value) + np.linalg.norm(V["ev"].value))
        violation = float(max(V["sl_gs"].value.max(), V["sl_x"].value.max(), V["sl_v"].value.max(),
                              V["sl_f"].value.max()))
        return Plan(t0=t0, dt=dt, r=V["r"].value.copy(), v=V["v"].value.copy(),
                    u=V["u"].value.copy(), m=m, a_ext=a_ext, terminal_error=err,
                    fuel=float(m0 - m[-1]), status=self.problem.status, violation=violation,
                    sigma=V["s"].value.copy())

    def search_tgo(self, t0, r0, v0, m0, sched_fn, target, wind_fn, t_lo=8.0, t_hi=32.0,
                   step=1.5, tol=0.3) -> Plan | None:
        """Free-final-time search: coarse grid then golden-section refinement on fuel."""
        best: Plan | None = None
        best_t = None
        results: dict[float, Plan | None] = {}

        def run(tg: float) -> float:
            plan = self.solve(t0, tg, r0, v0, m0, sched_fn(t0), target, wind_fn)
            results[tg] = plan
            if plan is None or plan.terminal_error > tol or plan.violation > 1.0:
                return math.inf
            return plan.fuel

        grid = np.arange(t_lo, t_hi + 1e-9, step)
        costs = [run(float(tg)) for tg in grid]
        i = int(np.argmin(costs))
        if not math.isfinite(costs[i]):
            return None
        a = float(grid[max(i - 1, 0)])
        b = float(grid[min(i + 1, len(grid) - 1)])
        gr = (math.sqrt(5) - 1) / 2
        c, d = b - gr * (b - a), a + gr * (b - a)
        fc, fd = run(c), run(d)
        for _ in range(6):
            if fc < fd:
                b, d, fd = d, c, fc
                c = b - gr * (b - a)
                fc = run(c)
            else:
                a, c, fc = c, d, fd
                d = a + gr * (b - a)
                fd = run(d)
        for tg, plan in results.items():
            if (plan is not None and plan.terminal_error <= tol and plan.violation <= 1.0
                    and (best is None or plan.fuel < best.fuel)):
                best, best_t = plan, tg
        return best
