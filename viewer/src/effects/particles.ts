import * as THREE from 'three';
import { puffTexture, rng } from '../scene/textures';

const vert = /* glsl */ `
attribute float aSize;
attribute float aAlpha;
attribute vec3 aColor;
uniform float uScale;
varying float vAlpha;
varying vec3 vColor;
void main() {
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  gl_PointSize = aSize * uScale / max(-mv.z, 1.0);
  gl_Position = projectionMatrix * mv;
  vAlpha = aAlpha;
  vColor = aColor;
}`;

const frag = /* glsl */ `
uniform sampler2D uMap;
varying float vAlpha;
varying vec3 vColor;
void main() {
  vec4 t = texture2D(uMap, gl_PointCoord);
  float a = t.a * vAlpha;
  if (a < 0.003) discard;
  gl_FragColor = vec4(vColor, a);
}`;

/** CPU-simulated smoke/steam/dust billboards: exhaust haze plus clouds where the plume hits the pad. */
export class Particles {
  readonly points: THREE.Points;
  private N: number;
  private pos: Float32Array;
  private vel: Float32Array;
  private age: Float32Array;
  private life: Float32Array;
  private s0: Float32Array;
  private s1: Float32Array;
  private tone: Float32Array;
  private aSize: Float32Array;
  private aAlpha: Float32Array;
  private aColor: Float32Array;
  private next = 0;
  private r = rng(17);
  private mat: THREE.ShaderMaterial;
  private spawnAcc = { haze: 0, cloud: 0 };

  constructor(capacity = 5000) {
    this.N = capacity;
    this.pos = new Float32Array(capacity * 3);
    this.vel = new Float32Array(capacity * 3);
    this.age = new Float32Array(capacity).fill(1e9);
    this.life = new Float32Array(capacity).fill(1);
    this.s0 = new Float32Array(capacity);
    this.s1 = new Float32Array(capacity);
    this.tone = new Float32Array(capacity);
    this.aSize = new Float32Array(capacity);
    this.aAlpha = new Float32Array(capacity);
    this.aColor = new Float32Array(capacity * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute('aSize', new THREE.BufferAttribute(this.aSize, 1).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute('aAlpha', new THREE.BufferAttribute(this.aAlpha, 1).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute('aColor', new THREE.BufferAttribute(this.aColor, 3).setUsage(THREE.DynamicDrawUsage));
    this.mat = new THREE.ShaderMaterial({
      vertexShader: vert, fragmentShader: frag, transparent: true, depthWrite: false, fog: false,
      uniforms: { uMap: { value: puffTexture() }, uScale: { value: 800 } },
    });
    this.points = new THREE.Points(geo, this.mat);
    this.points.frustumCulled = false;
    this.points.renderOrder = 5;
  }

  setScale(viewportHeight: number, fovDeg: number): void {
    this.mat.uniforms.uScale.value = viewportHeight / (2 * Math.tan((fovDeg * Math.PI) / 360));
  }

  reset(): void {
    this.age.fill(1e9);
    this.aAlpha.fill(0);
  }

  private emit(p: THREE.Vector3, v: THREE.Vector3, life: number, s0: number, s1: number, tone: number): void {
    const i = this.next;
    this.next = (this.next + 1) % this.N;
    this.pos[i * 3] = p.x; this.pos[i * 3 + 1] = p.y; this.pos[i * 3 + 2] = p.z;
    this.vel[i * 3] = v.x; this.vel[i * 3 + 1] = v.y; this.vel[i * 3 + 2] = v.z;
    this.age[i] = 0; this.life[i] = life; this.s0[i] = s0; this.s1[i] = s1; this.tone[i] = tone;
  }

  /**
   * dt in sim seconds. `nozzle`/`dir`: plume cluster origin and exhaust direction; `plumeLen` in m;
   * `power` = sum of engine levels; `vel`: booster velocity (three frame); `mountTop`, `mountR`
   * describe the launch-mount table the plume can hit before the ground.
   */
  update(dt: number, nozzle: THREE.Vector3, dir: THREE.Vector3, plumeLen: number, power: number,
    vel: THREE.Vector3, wind: THREE.Vector3, mountTop: number, mountR: number, light: THREE.PointLight,
    sunColor: THREE.Color): void {
    const r = this.r;
    const tmpP = new THREE.Vector3(), tmpV = new THREE.Vector3();
    if (dt > 0 && power > 0.02 && plumeLen > 0) {
      // exhaust haze trailing the plume
      this.spawnAcc.haze += dt * 60 * Math.min(power, 4);
      while (this.spawnAcc.haze >= 1) {
        this.spawnAcc.haze -= 1;
        tmpP.copy(nozzle).addScaledVector(dir, plumeLen * (0.5 + 0.6 * r()));
        tmpP.x += (r() - 0.5) * 4; tmpP.z += (r() - 0.5) * 4;
        tmpV.copy(dir).multiplyScalar(25 + 25 * r()).addScaledVector(vel, 0.55);
        this.emit(tmpP, tmpV, 2.5 + 2.5 * r(), 4 + 3 * r(), 18 + 10 * r(), 0.82 + 0.1 * r());
      }
      // impingement on the mount table or the ground
      let tHit = Infinity;
      if (dir.y < -0.05) {
        const tm = (mountTop - nozzle.y) / dir.y;
        if (tm > 0) {
          const hx = nozzle.x + dir.x * tm, hz = nozzle.z + dir.z * tm;
          if (Math.hypot(hx, hz) < mountR) tHit = tm;
        }
        const tg = (0.5 - nozzle.y) / dir.y;
        if (tg > 0 && tg < tHit) tHit = tg;
      }
      const reach = plumeLen * 3.2;
      if (tHit < reach) {
        const strength = (1 - tHit / reach) * Math.min(power, 6);
        this.spawnAcc.cloud += dt * 140 * strength;
        const hit = new THREE.Vector3().copy(nozzle).addScaledVector(dir, tHit);
        const onMount = hit.y > 2;
        while (this.spawnAcc.cloud >= 1) {
          this.spawnAcc.cloud -= 1;
          const a = r() * Math.PI * 2, sp = 12 + 30 * r() * (0.4 + strength * 0.2);
          tmpP.copy(hit).add(new THREE.Vector3((r() - 0.5) * 6, r() * 2, (r() - 0.5) * 6));
          tmpV.set(Math.cos(a) * sp, (onMount ? -6 : 2) + 6 * r(), Math.sin(a) * sp);
          const dust = !onMount && r() > 0.55;
          this.emit(tmpP, tmpV, 5 + 5 * r(), 5 + 4 * r(), 30 + 25 * r(), dust ? 0.62 + 0.1 * r() : 0.9 + 0.08 * r());
        }
      }
    }

    // integrate, age, write attributes
    const damp = Math.exp(-dt / 1.6);
    const lp = light.position, li = light.intensity / 4e3;
    for (let i = 0; i < this.N; i++) {
      const age = this.age[i] + dt;
      this.age[i] = age;
      const life = this.life[i];
      if (age >= life) { this.aAlpha[i] = 0; continue; }
      const k = i * 3;
      this.vel[k] *= damp; this.vel[k + 1] = this.vel[k + 1] * damp + 1.2 * dt; this.vel[k + 2] *= damp;
      this.pos[k] += (this.vel[k] + wind.x * 0.8) * dt;
      this.pos[k + 1] += this.vel[k + 1] * dt;
      this.pos[k + 2] += (this.vel[k + 2] + wind.z * 0.8) * dt;
      if (this.pos[k + 1] < 0.8) { this.pos[k + 1] = 0.8; this.vel[k + 1] = Math.abs(this.vel[k + 1]) * 0.3; }
      const f = age / life;
      this.aSize[i] = this.s0[i] + (this.s1[i] - this.s0[i]) * Math.sqrt(f);
      this.aAlpha[i] = 0.42 * Math.min(1, f * 8) * (1 - f) * (1 - f);
      // sunlit grey/tan, warmed by the plume when close to it
      const dx = this.pos[k] - lp.x, dy = this.pos[k + 1] - lp.y, dz = this.pos[k + 2] - lp.z;
      const warm = li * 0.5 * Math.exp(-Math.sqrt(dx * dx + dy * dy + dz * dz) / 45);
      const tone = this.tone[i];
      this.aColor[k] = sunColor.r * tone + warm * 1.6;
      this.aColor[k + 1] = sunColor.g * tone * 0.98 + warm * 0.8;
      this.aColor[k + 2] = sunColor.b * tone * 0.95 + warm * 0.3;
    }
    const g = this.points.geometry;
    (g.attributes.position as THREE.BufferAttribute).needsUpdate = true;
    (g.attributes.aSize as THREE.BufferAttribute).needsUpdate = true;
    (g.attributes.aAlpha as THREE.BufferAttribute).needsUpdate = true;
    (g.attributes.aColor as THREE.BufferAttribute).needsUpdate = true;
  }
}
