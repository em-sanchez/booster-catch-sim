import * as THREE from 'three';

/** Sim frame: x east (offshore), y north, z up. Three.js frame: Y up. three = (x, z, -y). */
export function simToThree(x: number, y: number, z: number, out = new THREE.Vector3()): THREE.Vector3 {
  return out.set(x, z, -y);
}

/** Quaternion (w, x, y, z) rotating sim body -> sim world, re-expressed in the three.js frame. */
export function simQuatToThree(w: number, x: number, y: number, z: number, out = new THREE.Quaternion()): THREE.Quaternion {
  return out.set(x, z, -y, w);
}

export interface EngineInfo { x: number; y: number; ring: number }
export interface FlightEvent { t: number; type: string; label: string; [k: string]: unknown }
export interface Site {
  armHeight: number; towerX: number; towerWidth: number; towerHeight: number; armHingeX: number;
  armHingeY: number; armLength: number; armOpenDeg: number; mountHeight: number; mountRadius: number;
  shorelineX: number;
}
export interface VehicleMeta {
  length: number; diameter: number; dryMass: number; hHardpoint: number; hGridfin: number;
  engineThrust: number; throttleMin: number; gimbalMaxDeg: number; engineExitDiameter: number;
  engines: EngineInfo[]; gridfinAzimuthDeg: number[];
}
export interface FlightResultMeta {
  caught: boolean; catch_error_x: number | null; catch_error_y: number | null; catch_vz: number | null;
  catch_tilt_deg: number | null; fuel_used: number | null; prop_left: number | null;
  ignition_altitude: number | null; t_catch: number | null; max_g: number;
}
export interface FlightFile {
  version: number;
  meta: {
    scenario: { id: string; name: string; description: string; windSpeed: number; windHeadingDeg: number };
    vehicle: VehicleMeta; site: Site; phases: { id: string; label: string }[]; dt: number;
    result: FlightResultMeta;
  };
  events: FlightEvent[];
  plans: { t: number; pts: number[] }[];
  samples: {
    t: number[]; p: number[]; q: number[]; v: number[]; m: number[]; thrust: number[]; thr: number[];
    lit: number[]; lv: number[]; gim: number[]; fins: number[]; arm: number[]; phase: number[];
    qbar: number[]; g: number[]; wind: number[];
  };
}

/** Interpolated state at one instant (positions in three.js frame unless named sim*). */
export class Sample {
  t = 0;
  pos = new THREE.Vector3();          // engine-plane centre
  quat = new THREE.Quaternion();
  simPos = new THREE.Vector3();       // sim frame
  simVel = new THREE.Vector3();
  mass = 0;
  thrust = 0;
  throttle = 0;
  lit = 0;
  levels: [number, number, number] = [0, 0, 0];
  gimbal: [number, number] = [0, 0];  // deg, body frame
  fins: [number, number, number, number] = [0, 0, 0, 0];
  arm = 0;
  phase = 0;
  qbar = 0;
  g = 0;
  wind = new THREE.Vector3();
}

export function engineLit(mask: number, i: number): boolean {
  return Math.floor(mask / 2 ** i) % 2 === 1;   // masks exceed 32 bits, so no bitwise ops
}

export class Flight {
  readonly data: FlightFile;
  readonly t: number[];
  readonly duration: number;
  private qa = new THREE.Quaternion();
  private qb = new THREE.Quaternion();

  constructor(data: FlightFile) {
    this.data = data;
    this.t = data.samples.t;
    this.duration = this.t[this.t.length - 1];
  }

  get meta() { return this.data.meta; }
  get events() { return this.data.events; }

  private index(t: number): [number, number] {
    const ts = this.t;
    if (t <= ts[0]) return [0, 0];
    if (t >= ts[ts.length - 1]) return [ts.length - 1, 0];
    let lo = 0, hi = ts.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (ts[mid] <= t) lo = mid; else hi = mid;
    }
    return [lo, (t - ts[lo]) / (ts[lo + 1] - ts[lo])];
  }

  sample(t: number, out: Sample): Sample {
    const s = this.data.samples;
    const [i, f] = this.index(t);
    const j = Math.min(i + 1, this.t.length - 1);
    const lerp = (a: number[], k: number, n: number) => a[i * n + k] + (a[j * n + k] - a[i * n + k]) * f;
    out.t = t;
    out.simPos.set(lerp(s.p, 0, 3), lerp(s.p, 1, 3), lerp(s.p, 2, 3));
    simToThree(out.simPos.x, out.simPos.y, out.simPos.z, out.pos);
    out.simVel.set(lerp(s.v, 0, 3), lerp(s.v, 1, 3), lerp(s.v, 2, 3));
    simQuatToThree(s.q[i * 4], s.q[i * 4 + 1], s.q[i * 4 + 2], s.q[i * 4 + 3], this.qa);
    simQuatToThree(s.q[j * 4], s.q[j * 4 + 1], s.q[j * 4 + 2], s.q[j * 4 + 3], this.qb);
    out.quat.slerpQuaternions(this.qa, this.qb, f);
    out.mass = lerp(s.m, 0, 1);
    out.thrust = lerp(s.thrust, 0, 1);
    out.throttle = lerp(s.thr, 0, 1);
    out.lit = s.lit[f < 0.5 ? i : j];
    out.levels = [lerp(s.lv, 0, 3), lerp(s.lv, 1, 3), lerp(s.lv, 2, 3)];
    out.gimbal = [lerp(s.gim, 0, 2), lerp(s.gim, 1, 2)];
    out.fins = [lerp(s.fins, 0, 4), lerp(s.fins, 1, 4), lerp(s.fins, 2, 4), lerp(s.fins, 3, 4)];
    out.arm = lerp(s.arm, 0, 1);
    out.phase = s.phase[f < 0.5 ? i : j];
    out.qbar = lerp(s.qbar, 0, 1);
    out.g = lerp(s.g, 0, 1);
    out.wind.set(lerp(s.wind, 0, 3), lerp(s.wind, 1, 3), lerp(s.wind, 2, 3));
    return out;
  }

  /** Latest guidance plan issued at or before t (engine-plane path, three.js frame). */
  planAt(t: number): THREE.Vector3[] | null {
    const plans = this.data.plans;
    let best: { t: number; pts: number[] } | null = null;
    for (const p of plans) { if (p.t <= t) best = p; else break; }
    if (!best) return null;
    const out: THREE.Vector3[] = [];
    for (let k = 0; k < best.pts.length; k += 3) out.push(simToThree(best.pts[k], best.pts[k + 1], best.pts[k + 2]));
    return out;
  }

  /** Engine-plane path from start to t, decimated (three.js frame). */
  trail(t: number, step = 3): THREE.Vector3[] {
    const s = this.data.samples;
    const out: THREE.Vector3[] = [];
    for (let i = 0; i < this.t.length && this.t[i] <= t; i += step) {
      out.push(simToThree(s.p[i * 3], s.p[i * 3 + 1], s.p[i * 3 + 2]));
    }
    return out;
  }
}

export interface FlightIndexEntry { id: string; name: string; description: string; file: string; caught: boolean }

export async function loadIndex(): Promise<FlightIndexEntry[]> {
  const r = await fetch('flights/index.json');
  const j = await r.json();
  return j.flights as FlightIndexEntry[];
}

export async function loadFlight(file: string): Promise<Flight> {
  const r = await fetch(file);
  return new Flight(await r.json() as FlightFile);
}
