import * as THREE from 'three';
import { engineLit, type Flight, type Sample } from '../playback/flight';

const fmt = (v: number, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '—');

export class Hud {
  private phaseEl = document.getElementById('phase')!;
  private clockEl = document.getElementById('clock')!;
  private grid = document.getElementById('readouts')!;
  private eng = document.getElementById('engines') as HTMLCanvasElement;
  private cells = new Map<string, HTMLElement>();
  private last = -1;
  private axis = new THREE.Vector3();

  constructor() {
    const rows: [string, string][] = [
      ['alt', 'Altitude'], ['vz', 'Vertical speed'], ['vh', 'Horizontal speed'], ['thr', 'Throttle'],
      ['n', 'Engines lit'], ['thrust', 'Thrust'], ['prop', 'Propellant'], ['g', 'Acceleration'],
      ['q', 'Dyn. pressure'], ['tilt', 'Tilt'], ['gim', 'Gimbal'], ['hp', 'Hardpoints → arms'], ['wind', 'Wind'],
    ];
    for (const [id, label] of rows) {
      const k = document.createElement('div'); k.className = 'k'; k.textContent = label;
      const v = document.createElement('div'); v.className = 'v'; v.textContent = '—';
      this.grid.append(k, v);
      this.cells.set(id, v);
    }
  }

  private set(id: string, text: string): void {
    const el = this.cells.get(id);
    if (el && el.textContent !== text) el.textContent = text;
  }

  update(s: Sample, f: Flight, force = false): void {
    if (!force && Math.abs(s.t - this.last) < 0.05) return;
    this.last = s.t;
    const meta = f.meta;
    this.phaseEl.textContent = meta.phases[s.phase]?.label ?? '';
    this.clockEl.textContent = `T+ ${fmt(s.t, 1).padStart(5, '0')} s`;
    const n = meta.vehicle.engines.reduce((acc, _e, i) => acc + (engineLit(s.lit, i) ? 1 : 0), 0);
    this.set('alt', `${fmt(s.simPos.z, 0)} m`);
    this.set('vz', `${fmt(s.simVel.z, 1)} m/s`);
    this.set('vh', `${fmt(Math.hypot(s.simVel.x, s.simVel.y), 1)} m/s`);
    this.set('thr', n ? `${fmt(s.throttle * 100, 0)} %` : '—');
    this.set('n', `${n} / ${meta.vehicle.engines.length}`);
    this.set('thrust', `${fmt(s.thrust / 1e6, 1)} MN`);
    this.set('prop', `${fmt((s.mass - meta.vehicle.dryMass) / 1000, 1)} t`);
    this.set('g', `${fmt(s.g, 2)} g`);
    this.set('q', `${fmt(s.qbar / 1000, 1)} kPa`);
    // body axis (three frame) -> sim
    this.axis.set(0, 1, 0).applyQuaternion(s.quat);
    const ax = this.axis.x, ay = -this.axis.z, az = this.axis.y;
    this.set('tilt', `${fmt(THREE.MathUtils.radToDeg(Math.acos(Math.min(1, az))), 1)}°`);
    this.set('gim', `${fmt(Math.hypot(s.gimbal[0], s.gimbal[1]), 1)}°`);
    const h = meta.vehicle.hHardpoint;
    const hx = s.simPos.x + h * ax, hy = s.simPos.y + h * ay, hz = s.simPos.z + h * az;
    const d = Math.hypot(hx, hy, hz - meta.site.armHeight);
    this.set('hp', `${fmt(d, d < 20 ? 2 : 0)} m`);
    this.set('wind', `${fmt(Math.hypot(s.wind.x, s.wind.y), 1)} m/s`);
    this.drawEngines(s, f);
  }

  private drawEngines(s: Sample, f: Flight): void {
    const c = this.eng, g = c.getContext('2d')!;
    const W = c.width, R = f.meta.vehicle.diameter / 2;
    const k = (W / 2 - 6) / R;
    g.clearRect(0, 0, W, W);
    g.beginPath(); g.arc(W / 2, W / 2, R * k + 2, 0, Math.PI * 2);
    g.strokeStyle = 'rgba(255,255,255,0.25)'; g.lineWidth = 1.5; g.stroke();
    const rE = (f.meta.vehicle.engineExitDiameter / 2) * k * 0.95;
    f.meta.vehicle.engines.forEach((e, i) => {
      const lit = engineLit(s.lit, i);
      const lvl = lit ? Math.max(0.2, s.levels[e.ring]) : 0;
      const x = W / 2 + e.x * k, y = W / 2 - e.y * k;
      g.beginPath(); g.arc(x, y, rE, 0, Math.PI * 2);
      if (lit) {
        g.fillStyle = `rgba(255, ${Math.round(150 + 80 * lvl)}, ${Math.round(60 + 60 * lvl)}, ${0.45 + 0.55 * lvl})`;
        g.shadowColor = 'rgba(255,160,60,0.9)'; g.shadowBlur = 8 * lvl;
      } else {
        g.fillStyle = 'rgba(255,255,255,0.08)'; g.shadowBlur = 0;
      }
      g.fill();
      g.shadowBlur = 0;
      g.strokeStyle = 'rgba(255,255,255,0.25)'; g.lineWidth = 1; g.stroke();
    });
  }
}
