"""Closed-loop flight: 6-DOF truth model + guidance + control, recorded for the viewer."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from .control import (AttitudeController, GridFins, LateralLQR, clamp_tilt, coast_attitude,
                      gimbal_for_torque, gimbal_torque, roll_rcs, terminal_accel, track_plan)
from .dynamics import Actuation, aero, body_axis, make_state, quat_mul, quat_to_mat, rk4_step
from .engines import EngineCluster
from .environment import GRAVITY, Wind, density, pressure
from .guidance import GuidanceSchedule, Plan, PoweredDescentGuidance, TargetSpec
from .scenario import Scenario
from .vehicle import CENTER_ENGINES, G0, INNER_ENGINES, OUTER_ENGINES

PHASES = ["coast", "burn13", "burn3", "terminal", "caught", "miss"]
PHASE_LABELS = {
    "coast": "Descent - grid-fin steering",
    "burn13": "Landing burn - 13 engines",
    "burn3": "Landing burn - 3 engines",
    "terminal": "Terminal approach",
    "caught": "Caught by tower arms",
    "miss": "Missed catch",
}

# catch tolerances: arms close in y, run along x
TOL_Y = 1.5
TOL_X = 4.0
TOL_VZ = 3.0
TOL_TILT_DEG = 4.0


def quat_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    c = float(a @ b)
    axis = np.cross(a, b)
    s = float(np.linalg.norm(axis))
    if s < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0]) if c > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    ang = math.atan2(s, c)
    axis /= s
    return np.array([math.cos(ang / 2), *(axis * math.sin(ang / 2))])


@dataclass
class FlightResult:
    scenario_id: str
    caught: bool
    catch_error_x: float = math.nan
    catch_error_y: float = math.nan
    catch_vz: float = math.nan
    catch_tilt_deg: float = math.nan
    t_catch: float = math.nan
    t_ignition: float = math.nan
    ignition_altitude: float = math.nan
    fuel_used: float = math.nan
    prop_left: float = math.nan
    max_g: float = 0.0
    max_gimbal_deg: float = 0.0
    max_throttle: float = 0.0
    min_throttle_lit: float = 1.0
    max_tilt_deg: float = 0.0
    min_x_burn: float = math.inf
    guidance_solves: int = 0
    guidance_ms: float = 0.0
    wall_s: float = 0.0
    record: dict = field(default_factory=dict, repr=False)

    def summary(self) -> dict:
        d = {k: v for k, v in self.__dict__.items() if k != "record"}
        return {k: (round(v, 4) if isinstance(v, float) and math.isfinite(v) else
                    (None if isinstance(v, float) else v)) for k, v in d.items()}


class Flight:
    def __init__(self, sc: Scenario, dt: float = 0.01, record_hz: float = 30.0, N: int = 30,
                 record: bool = True):
        self.sc = sc
        self.site = sc.site
        self.nom = sc.vehicle                 # what guidance/control believe
        self.veh = sc.truth_vehicle()         # what the physics uses
        self.dt = dt
        self.record_every = max(1, int(round(1.0 / (record_hz * dt))))
        self.do_record = record
        self.eng = EngineCluster(self.veh, sc.thrust_scale, sc.isp_scale)
        self.fins = GridFins(self.veh)
        self.att = AttitudeController(self.nom)
        self.lqr_burn = LateralLQR()
        self.lqr_term = LateralLQR(pos=1.5, vel=0.8, tilt=0.06, rate=0.05, gimbal=0.12)
        self.i_hp = np.zeros(2)       # terminal integral of hardpoint error [m s]
        self.ki_hp = 0.4
        self.wind = Wind(sc.wind_speed, sc.wind_heading_deg, sc.gust, seed=sc.seed + 100)
        self.gd = PoweredDescentGuidance(N=N, isp=self.nom.engine.isp_sl, vehicle=self.nom,
                                         mass_dry=self.nom.dry_mass)
        m0 = self.nom.dry_mass + sc.prop0
        v0 = np.array(sc.v0, float)
        q0 = quat_between(np.array([0.0, 0.0, 1.0]), -v0)
        self.x = make_state(np.array(sc.r0, float), v0, q0, (0.0, 0.0, 0.0), m0)
        self.m0 = m0
        self.t = 0.0
        self.phase = "coast"
        self.plan: Plan | None = None
        self.sched: GuidanceSchedule | None = None
        self.t_ign: float | None = None
        self.next_ign_plan = 0.0
        self.next_guid = 0.0
        self.tgo_hint = 12.0
        self.a_cmd = np.zeros(3)
        self.throttle_cmd = 0.0
        self.arm = 0.0                # 0 open .. 1 closed
        self.arm_closing = False
        self.catch_state: dict | None = None
        self.events: list[dict] = []
        self.plans: list[dict] = []
        self.rec: dict[str, list] = {k: [] for k in (
            "t", "p", "q", "v", "m", "thrust", "thr", "lit", "lv", "gim", "fins", "arm", "phase",
            "qbar", "g", "wind")}
        self.res = FlightResult(sc.id, caught=False)
        self.solve_time = 0.0
        self.attitude_lead = 1.0    # s of feed-forward lead to cover attitude response time
        self.t_term = 0.6           # hand over to the terminal controller this long before the gate
        self.gate_height = 10.0     # hardpoints this far above the arm rails at the gate [m]
        self.gate_speed = 3.3       # sink rate at the gate [m/s]
        self.t_freeze = 1.0         # stop re-planning this close to the gate; just track
        self.done = False
        self.end_time = None
        self._last_force = np.zeros(3)

    # ---- helpers --------------------------------------------------------------------------
    @property
    def r(self): return self.x[0:3]
    @property
    def v(self): return self.x[3:6]
    @property
    def q(self): return self.x[6:10]
    @property
    def w(self): return self.x[10:13]
    @property
    def m(self): return float(self.x[13])

    def wind_model(self, z: float) -> np.ndarray:
        return self.wind.mean_at(z)          # guidance knows the forecast mean wind, not gusts

    def event(self, kind: str, label: str, **extra) -> None:
        self.events.append({"t": round(self.t, 3), "type": kind, "label": label, **extra})

    def target(self, m_final: float) -> TargetSpec:
        h_cg = self.nom.cg_height(m_final)
        z = self.site.arm_height + self.gate_height - (self.nom.h_hardpoint - h_cg)
        return TargetSpec(r=np.array([0.0, 0.0, z]), v=np.array([0.0, 0.0, -self.gate_speed]),
                          divert_x=self.sc.divert_x, divert_time=self.sc.divert_time,
                          keepout_x=-1.0, tilt_max=math.radians(20.0), tilt_final=math.radians(3.0))

    def schedule(self, t_ign: float, planning: bool) -> GuidanceSchedule:
        """Planning uses tighter throttle caps than the closed loop, leaving authority to absorb
        engine start-up lag, wind and model errors."""
        e = self.nom.engine
        return GuidanceSchedule(t_switch=t_ign + self.sc.t13, n_before=13, n_after=3,
                                thrust_engine=e.thrust_sl, throttle_min=e.throttle_min,
                                throttle_max=0.80 if planning else 0.92,
                                throttle_max_before=0.50 if planning else 0.60,
                                throttle_min_after=0.50 if planning else 0.45)

    def timed_solve(self, *args, **kw):
        t0 = time.perf_counter()
        out = self.gd.solve(*args, **kw)
        self.solve_time += time.perf_counter() - t0
        return out

    def record_plan(self, plan: Plan) -> None:
        h = self.nom.cg_height(float(plan.m[-1]))
        pts = plan.r.T.copy()
        pts[:, 2] -= h
        self.plans.append({"t": round(self.t, 3),
                           "pts": [round(float(c), 2) for c in pts.ravel()]})

    # ---- ignition planning ----------------------------------------------------------------
    def predict_coast(self, dt: float = 0.25, horizon: float = 45.0):
        r, v, t = self.r.copy(), self.v.copy(), self.t
        m = self.m
        z_stop = self.site.arm_height
        out = [(t, r.copy(), v.copy())]
        cd = self.nom.cd_axial
        while r[2] > z_stop + 20 and t - self.t < horizon:
            def acc(rr, vv):
                vrel = vv - self.wind_model(rr[2])
                return GRAVITY - 0.5 * density(rr[2]) * self.nom.ref_area * cd * np.linalg.norm(vrel) * vrel / m
            a1 = acc(r, v)
            vm, rm = v + a1 * dt / 2, r + v * dt / 2
            a2 = acc(rm, vm)
            r = r + vm * dt
            v = v + a2 * dt
            t += dt
            out.append((t, r.copy(), v.copy()))
        return out

    def feasible_from(self, traj, i: int) -> Plan | None:
        """Can a burn that ignites at traj[i] still reach the arms? The plan starts from the
        state half a second later so engine start-up lag is not counted as thrust."""
        t_i = traj[i][0]
        t_s, r, v = traj[min(i + 2, len(traj) - 1)]
        sched = self.schedule(t_i, planning=True)
        target = self.target(self.m - 30e3)
        h = self.tgo_hint
        order = [h, h + 1, h - 1, h + 2, h - 2, h + 3.5, h - 3.5, h + 5, h - 5, h + 7, h + 10]
        for tg in order:
            if tg < 5.0 or tg > 40.0:
                continue
            plan = self.timed_solve(t_s, tg, r, v, self.m, sched, target, self.wind_model)
            if plan is not None and plan.terminal_error < 0.3 and plan.violation < 1.0:
                self.tgo_hint = tg
                return plan
        return None

    def plan_ignition(self) -> None:
        traj = self.predict_coast()
        n = len(traj)
        i = n - 1
        found = None
        while i >= 0:
            if self.feasible_from(traj, i) is not None:
                found = i
                break
            i -= 4
        if found is None:
            return
        lo, hi = found, min(found + 4, n - 1)
        if hi == lo:
            self.t_ign = traj[lo][0]
            return
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.feasible_from(traj, mid) is not None:
                lo = mid
            else:
                hi = mid
        self.t_ign = traj[lo][0]

    # ---- phase logic ----------------------------------------------------------------------
    def ignite(self) -> None:
        self.eng.ignite(CENTER_ENGINES + INNER_ENGINES, self.t)
        self.sched = self.schedule(self.t, planning=False)
        self.sched_plan = self.schedule(self.t, planning=True)
        target = self.target(self.m - 30e3)
        h = self.tgo_hint
        # start the plan where the booster will be once the engines have spooled up
        lag = 0.4
        pred = self.predict_coast(dt=lag / 2, horizon=lag)[-1]
        t0, r0, v0 = pred
        tw = time.perf_counter()
        plan = self.gd.search_tgo(t0, r0, v0, self.m, lambda _t: self.sched_plan,
                                  target, self.wind_model, t_lo=6.0, t_hi=32.0, step=1.0)
        self.solve_time += time.perf_counter() - tw
        if plan is None:
            plan = self.timed_solve(t0, h, r0, v0, self.m, self.sched, target, self.wind_model)
        self.plan = plan
        self.t_final_max = plan.t_final + 1.5
        self.record_plan(plan)
        self.phase = "burn13"
        self.next_guid = self.t + 0.5
        self.res.t_ignition = self.t
        self.res.ignition_altitude = float(self.r[2] - self.nom.cg_height(self.m))
        self.event("ignition", "Landing burn ignition - 13 engines",
                   altitude=round(self.res.ignition_altitude, 1))

    def resolve(self) -> None:
        """Re-plan from the measured state with the same final time. Only if that misses the
        target, try a little more time (bounded, so the burn cannot drift longer and longer)."""
        t_go = self.plan.t_final - self.t
        target = self.target(float(self.plan.m[-1]))
        kw = dict(guess=self.plan, u_prev=self.a_cmd, w_prev=0.2)
        best = None
        # conservative limits first (keeps margins for the tracking loop), then the wider
        # closed-loop limits, then a little more or less time
        for sched, extra in ((self.sched_plan, 0.0), (self.sched, 0.0), (self.sched, 0.5), (self.sched, -0.5)):
            tg = t_go + extra
            if tg < self.t_term or self.t + tg > self.t_final_max:
                continue
            plan = self.timed_solve(self.t, tg, self.r.copy(), self.v.copy(), self.m, sched, target,
                                    self.wind_model, **kw)
            if plan is not None and (best is None or plan.terminal_error < best.terminal_error - 0.1):
                best = plan
            if best is not None and best.terminal_error < 0.5:
                break
        if best is not None:
            self.plan = best
            self.record_plan(best)

    # ---- main step ------------------------------------------------------------------------
    def step(self) -> None:
        dt, t = self.dt, self.t
        R = quat_to_mat(self.q)
        b = R[:, 2]
        m = self.m
        h_cg_true = self.veh.cg_height(m)
        h_cg = self.nom.cg_height(m)
        z_cg = float(self.r[2])
        wind = self.wind.at(z_cg)
        vrel = self.v - wind
        qbar = 0.5 * density(z_cg) * float(vrel @ vrel)
        p_amb = pressure(z_cg)
        rated = self.nom.engine.thrust_sl

        if self.phase == "caught":
            self.step_caught()
            return

        b_des = np.array([0.0, 0.0, 1.0])
        throttle = 0.0
        if self.phase == "coast":
            if z_cg < 3600 and t >= self.next_ign_plan:
                self.plan_ignition()
                self.next_ign_plan = t + 1.5
            if self.t_ign is not None and t >= self.t_ign - 1e-9:
                self.ignite()
        if self.phase == "coast":
            # steer so that, carrying today's lateral velocity through the burn, the booster
            # would arrive at the offshore gate; the burn then only trims the last metres
            z_ign = 1300.0
            t_rem = max((z_cg - z_ign) / max(-self.v[2], 50.0), 0.0) + 5.0
            aim = np.array([self.sc.aim_x, 0.0])
            v_des = (aim - self.r[:2]) / t_rem
            a_lat = np.array([*(0.5 * (v_des - self.v[:2])), 0.0])
            b_des = coast_attitude(self.v, wind, a_lat, qbar, m, self.nom, math.radians(15.0))
        else:
            if self.phase in ("burn13", "burn3"):
                if self.phase == "burn13" and t >= self.sched.t_switch:
                    self.eng.shutdown(INNER_ENGINES)
                    self.phase = "burn3"
                    self.event("engine_cutoff", "Inner 10 engines off - 3 centre engines remain")
                t_go = self.plan.t_final - t
                if t >= self.next_guid and t_go > self.t_freeze:
                    self.resolve()
                    self.next_guid = t + (0.25 if t_go < 5.0 else 0.5)
                if self.plan.t_final - t < self.t_term:
                    hp = self.r + (self.nom.h_hardpoint - h_cg) * b
                    # offshore misalignment is recoverable by hovering; the tower side is not
                    if np.linalg.norm(hp[:2]) > 25.0 or hp[0] < -6.0:
                        self.phase = "miss"
                        self.event("miss", "Abort - too far from the arms at terminal gate",
                                   errX=round(float(hp[0]), 2), errY=round(float(hp[1]), 2))
                        self.done = True
                        return
                    self.phase = "terminal"
                    self.event("terminal", "Terminal approach - settle onto the arms")
                    self.start_arms()
            if self.phase in ("burn13", "burn3"):
                r_ref, v_ref, u_ref = self.plan.sample(t)
                # turn early (attitude lags) but keep today's thrust magnitude
                _, _, u_lead = self.plan.sample(min(t + self.attitude_lead, self.plan.t_final))
                u_ff = u_lead / max(float(np.linalg.norm(u_lead)), 1e-6) * float(np.linalg.norm(u_ref))
                self.a_cmd = track_plan(r_ref, v_ref, u_ff, self.r, self.v)
                self._ref = (r_ref, v_ref, u_ff)
                max_tilt = math.radians(30.0)
            else:
                hp = self.r + (self.nom.h_hardpoint - h_cg) * b
                drag = self.gd.drag_accel(self.r[:, None], self.v[:, None], np.array([m]),
                                          self.wind_model)[:, 0]
                above = float(hp[2] - self.site.arm_height)
                # sink rate shrinks with height above the rails: ~3 m/s at the gate, 0.8 m/s at contact
                sink = min(0.8 + 0.25 * max(above, 0.0), self.gate_speed)
                # not lined up yet? slow the descent (down to a near-hover) and let the hold catch up
                misalign = max(float(np.linalg.norm(hp[:2])) - 0.5, 0.0) + 0.8 * float(np.linalg.norm(self.v[:2]))
                sink *= min(max(1.0 - misalign / 3.0, 0.15), 1.0)
                self.a_cmd = terminal_accel(self.r, self.v, np.zeros(3), sink, drag)
                self._ref = (np.zeros(3), np.zeros(3), np.array([-drag[0], -drag[1], self.a_cmd[2]]))
                max_tilt = math.radians(6.0)
                if hp[2] < self.site.arm_height + 14 and np.linalg.norm(hp[:2]) < 4:
                    self.start_arms()
                if hp[2] <= self.site.arm_height:
                    self.contact(hp, b)
                    if self.phase == "caught":
                        self.step_caught()
                        return
                if self.phase == "miss":
                    self.done = True
                    return
            n_lit = max(self.eng.n_lit, 1)
            # vertical channel: throttle so the vertical thrust matches the command
            throttle = max(m * float(self.a_cmd[2]), 0.0) / max(float(b[2]), 0.7) / (n_lit * rated)

        t_g = self.eng.gimballed_thrust(p_amb)
        if self.phase == "coast" or self.eng.n_lit == 0:
            # grid fins point the vehicle; gimbal idle
            avail = (t_g * math.sin(self.nom.engine.gimbal_max) * h_cg + self.fins.max_torque(qbar, h_cg))
            alpha_max = avail / self.nom.inertia(m)[0]
            tau_cmd = self.att.torque(self.q, self.w, b_des, m, alpha_max)
            tau_fins = self.fins.step(dt, tau_cmd, qbar, h_cg)
            rest = tau_cmd - tau_fins
            g_cmd = gimbal_for_torque(rest, t_g, h_cg)
        else:
            g_cmd, tau_fins, rest = self.lateral_control(R, b, m, h_cg, qbar, t_g, max_tilt)
        self.eng.step(dt, t, throttle, g_cmd)
        self.throttle_cmd = self.eng.throttle if self.eng.n_lit else 0.0
        f_eng, t_eng, mdot, thrust = self.eng.forces(h_cg_true, p_amb)
        tau = t_eng + tau_fins
        tau[2] += roll_rcs(rest[2], self.veh)
        if self.m - mdot * dt < self.veh.dry_mass:
            mdot = 0.0
        act = Actuation(force_body=f_eng, torque_body=tau, mdot=mdot, thrusting=thrust > 1e5, wind=wind)

        f_aero, _ = aero(self.veh, self.r, self.v, R, self.w, wind, h_cg_true, act.thrusting)
        self._last_force = R @ f_eng + f_aero
        self.stats(thrust, b)
        if self.t_ign is None and self.phase == "coast" and z_cg < self.site.arm_height + 200:
            self.phase = "miss"
            self.event("miss", "No feasible landing burn found")
            self.done = True
        self.maybe_record(thrust, qbar, wind)
        self.x = rk4_step(self.veh, self.x, act, dt)
        self.wind.step(dt)
        self.update_arms(dt)
        self.t += dt

    def lateral_control(self, R, b, m, h_cg, qbar, t_g, max_tilt):
        """LQR on both lateral axes; grid fins take part of the torque while airspeed lasts."""
        r_ref, v_ref, u_ff = self._ref
        thrust_now = float(self.eng.thrust_per_engine(pressure(float(self.r[2]))).sum())
        inertia_lat = self.nom.inertia(m)[0]
        a_t = max(thrust_now, 1.0) / m
        b_t = max(t_g, 1.0) * h_cg / inertia_lat
        n_alpha = qbar * self.nom.ref_area * self.nom.cn_alpha
        speed = float(np.linalg.norm(self.v - self.wind_model(float(self.r[2]))))
        self.lqr = self.lqr_term if self.phase == "terminal" else self.lqr_burn
        self.lqr.update(a_t, b_t, n_alpha / m, n_alpha * (self.nom.h_cp - h_cg) / inertia_lat, speed, self.dt)
        u_n = max(float(np.linalg.norm(u_ff)), 1.0)
        tilt_ref = np.clip(u_ff[:2] / u_n, -math.sin(max_tilt), math.sin(max_tilt))
        w_world = R @ self.w
        tilt_rate = np.cross(w_world, b)[:2]
        e_pos = self.r[:2] - r_ref[:2]
        if self.phase == "terminal":
            # hold the hardpoints (not the CG) over the arms: offset the CG for the steady lean
            # into the wind, and integrate the hardpoint error to trim out unknown gusts/drag
            lever = self.nom.h_hardpoint - h_cg
            hp_xy = self.r[:2] + lever * b[:2]
            self.i_hp = np.clip(self.i_hp + hp_xy * self.dt, -25.0, 25.0)
            e_pos = self.r[:2] + lever * tilt_ref + self.ki_hp * self.i_hp
        d_world = self.lqr.command(e_pos, self.v[:2] - v_ref[:2], b[:2], tilt_ref,
                                   tilt_rate, self.nom.engine.gimbal_max)
        d_body = (R.T @ np.array([d_world[0], d_world[1], 0.0]))[:2]
        tau_want = gimbal_torque(d_body, t_g, h_cg)
        tau_fins = self.fins.step(self.dt, tau_want, qbar, h_cg)
        d_rest = d_body - gimbal_for_torque(np.array([tau_fins[0], tau_fins[1], 0.0]), t_g, h_cg)
        inertia = self.nom.inertia(m)
        rest = np.array([0.0, 0.0, -inertia[2] * self.att.kd_roll * self.w[2] - tau_fins[2]])
        return d_rest, tau_fins, rest

    def stats(self, thrust: float, b: np.ndarray) -> None:
        r = self.res
        r.max_g = max(r.max_g, float(np.linalg.norm(self._last_force)) / self.m / G0)
        r.max_gimbal_deg = max(r.max_gimbal_deg, math.degrees(float(np.linalg.norm(self.eng.gimbal))))
        if self.eng.n_lit and self.phase != "coast":
            r.max_throttle = max(r.max_throttle, self.eng.throttle)
            r.min_throttle_lit = min(r.min_throttle_lit, self.eng.throttle)
            r.max_tilt_deg = max(r.max_tilt_deg, math.degrees(math.acos(min(1.0, float(b[2])))))
            r.min_x_burn = min(r.min_x_burn, float(self.r[0]))

    # ---- arms / catch ---------------------------------------------------------------------
    def start_arms(self) -> None:
        if not self.arm_closing:
            self.arm_closing = True
            self.event("arms", "Tower arms closing")

    def update_arms(self, dt: float) -> None:
        if self.arm_closing and self.arm < 1.0:
            self.arm = min(1.0, self.arm + dt / 2.5)

    def contact(self, hp: np.ndarray, b: np.ndarray) -> None:
        tilt = math.degrees(math.acos(min(1.0, float(b[2]))))
        r = self.res
        r.catch_error_x, r.catch_error_y = float(hp[0]), float(hp[1])
        r.catch_vz = float(self.v[2])
        r.catch_tilt_deg = tilt
        r.t_catch = self.t
        ok = (abs(hp[1]) < TOL_Y and abs(hp[0]) < TOL_X and -self.v[2] < TOL_VZ and tilt < TOL_TILT_DEG)
        r.caught = bool(ok)
        r.fuel_used = self.m0 - self.m
        r.prop_left = self.m - self.nom.dry_mass
        if ok:
            self.phase = "caught"
            self.eng.shutdown(range(self.eng.n))
            self.arm = max(self.arm, 0.9)
            self.arm_closing = True
            self.event("catch", "Catch - hardpoints on the arms, engines off",
                       errX=round(r.catch_error_x, 3), errY=round(r.catch_error_y, 3),
                       vz=round(r.catch_vz, 3), tiltDeg=round(tilt, 3))
            yaw_q = self.q.copy()
            self.catch_state = {"t": self.t, "hp": hp.copy(), "vz": float(self.v[2]), "q": yaw_q,
                                "b": b.copy()}
            self.end_time = self.t + 6.0
        else:
            self.phase = "miss"
            self.event("miss", "Missed the arms", errX=round(float(hp[0]), 2), errY=round(float(hp[1]), 2))
            self.end_time = self.t

    def step_caught(self) -> None:
        cs = self.catch_state
        s = self.t - cs["t"]
        dt = self.dt
        w_n, zeta = 5.0, 0.35
        wd = w_n * math.sqrt(1 - zeta * zeta)
        dz = cs["vz"] / wd * math.exp(-zeta * w_n * s) * math.sin(wd * s)
        hp = cs["hp"].copy()
        hp[1] *= math.exp(-s / 0.6)
        hp[2] = self.site.arm_height + dz
        # rotate the residual tilt out
        b0 = cs["b"]
        k = math.exp(-s / 0.5)
        b = b0 * k + np.array([0.0, 0.0, 1.0]) * (1 - k)
        b /= np.linalg.norm(b)
        q = quat_mul(quat_between(body_axis(cs["q"]), b), cs["q"])
        m = self.m
        h_cg = self.veh.cg_height(m)
        r_new = hp - (self.veh.h_hardpoint - h_cg) * b
        v_new = (r_new - self.r) / dt
        self.eng.step(dt, self.t, 0.0, np.zeros(2))
        f_eng, _, mdot, thrust = self.eng.forces(h_cg, pressure(r_new[2]))
        self.x[0:3] = r_new
        self.x[3:6] = v_new
        self.x[6:10] = q
        self.x[10:13] = 0.0
        self.x[13] = max(self.veh.dry_mass, m - mdot * dt)
        wind = self.wind.at(r_new[2])
        self._last_force = np.zeros(3)
        self.maybe_record(thrust, 0.0, wind)
        self.wind.step(dt)
        self.update_arms(dt)
        self.t += dt
        if self.t >= self.end_time:
            self.done = True

    # ---- recording ------------------------------------------------------------------------
    def maybe_record(self, thrust: float, qbar: float, wind: np.ndarray) -> None:
        if not self.do_record:
            self._step_i = getattr(self, "_step_i", 0) + 1
            return
        i = getattr(self, "_step_i", 0)
        self._step_i = i + 1
        if i % self.record_every:
            return
        rec = self.rec
        b = body_axis(self.q)
        h = self.veh.cg_height(self.m)
        base = self.r - h * b
        lv = self.eng.level
        rec["t"].append(round(self.t, 3))
        rec["p"].extend(round(float(c), 3) for c in base)
        rec["q"].extend(round(float(c), 5) for c in self.q)
        rec["v"].extend(round(float(c), 3) for c in self.v)
        rec["m"].append(round(self.m, 1))
        rec["thrust"].append(round(thrust, 0))
        rec["thr"].append(round(self.throttle_cmd, 4))
        rec["lit"].append(self.eng.lit_mask)
        rec["lv"].extend(round(float(lv[idx].mean()), 4) for idx in (CENTER_ENGINES, INNER_ENGINES, OUTER_ENGINES))
        rec["gim"].extend(round(math.degrees(float(g)), 3) for g in self.eng.gimbal)
        rec["fins"].extend(round(math.degrees(float(d)), 2) for d in self.fins.defl)
        rec["arm"].append(round(self.arm, 4))
        rec["phase"].append(PHASES.index(self.phase))
        rec["qbar"].append(round(qbar, 1))
        rec["g"].append(round(float(np.linalg.norm(self._last_force)) / self.m / G0, 3))
        rec["wind"].extend(round(float(c), 2) for c in wind)

    # ---- run ------------------------------------------------------------------------------
    def run(self, t_max: float = 120.0) -> FlightResult:
        wall = time.perf_counter()
        self.event("start", "Descent after boostback - grid fins steering")
        while not self.done and self.t < t_max:
            self.step()
        if self.phase == "caught" or self.res.caught:
            self.event("settled", "Booster held by the arms")
        r = self.res
        r.guidance_solves = self.gd.solve_count
        r.guidance_ms = 1000 * self.solve_time / max(1, self.gd.solve_count)
        r.wall_s = time.perf_counter() - wall
        if math.isnan(r.fuel_used):
            r.fuel_used = self.m0 - self.m
            r.prop_left = self.m - self.nom.dry_mass
        r.record = self.export()
        return r

    def export(self) -> dict:
        return {
            "version": 1,
            "frame": "x east/offshore, y north, z up; metres; quaternion (w,x,y,z) body->world; "
                     "p = engine-plane centre",
            "meta": {
                "scenario": self.sc.to_meta(),
                "vehicle": self.nom.to_meta(),
                "site": self.site.to_meta(),
                "phases": [{"id": p, "label": PHASE_LABELS[p]} for p in PHASES],
                "dt": self.dt * self.record_every,
                "result": self.res.summary(),
            },
            "events": self.events,
            "plans": self.plans,
            "samples": self.rec,
        }
