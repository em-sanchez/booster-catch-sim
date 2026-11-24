import * as THREE from 'three';
import { engineLit, type Sample } from '../playback/flight';
import type { BoosterModel } from '../scene/booster';

const vert = /* glsl */ `
varying vec3 vN;
varying vec3 vView;
varying float vS;
varying vec2 vUv;
void main() {
  vUv = uv;
  vS = -position.y;                       // 0 at the nozzle exit .. 1 at the tail
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  vView = -mv.xyz;
  vN = normalMatrix * normal;
  gl_Position = projectionMatrix * mv;
}`;

const frag = /* glsl */ `
uniform float uTime;
uniform float uLevel;
uniform float uSeed;
uniform float uIntensity;
varying vec3 vN;
varying vec3 vView;
varying float vS;
varying vec2 vUv;
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  vec2 u = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y);
}
void main() {
  float s = clamp(vS, 0.0, 1.0);
  float facing = abs(dot(normalize(vN), normalize(vView)));
  float body = pow(facing, 1.6);
  float n = noise(vec2(vUv.x * 9.0 + uSeed, s * 7.0 - uTime * 14.0));
  float flicker = 0.8 + 0.35 * n;
  // shock diamonds near the nozzle (sea-level, slightly over-expanded)
  float diamonds = pow(0.5 + 0.5 * cos(s * 6.2832 * 4.0 - 0.6), 8.0) * exp(-s * 4.5);
  float core = exp(-s * 3.0);
  vec3 blue = vec3(0.42, 0.52, 1.0);
  vec3 violet = vec3(0.85, 0.45, 1.0);
  vec3 orange = vec3(1.0, 0.5, 0.18);
  vec3 col = mix(blue, violet, smoothstep(0.05, 0.35, s));
  col = mix(col, orange, smoothstep(0.3, 0.85, s));
  col = col * (0.45 + 0.9 * core) + vec3(1.0, 0.92, 1.0) * diamonds * 2.6 * facing;
  float a = body * (1.0 - smoothstep(0.55, 1.0, s)) * smoothstep(0.0, 0.04, s + 0.02);
  a *= uLevel * flicker;
  gl_FragColor = vec4(col * a * uIntensity, a);
}`;

/** One additive plume cone per engine, parented to the engine's gimbal pivot. */
export class Plumes {
  private meshes: THREE.Mesh[] = [];
  private mats: THREE.ShaderMaterial[] = [];
  readonly light = new THREE.PointLight(0xff9a4a, 0, 900, 1.6);
  readonly glow: THREE.Sprite;
  private tmp = new THREE.Vector3();
  private tmpDir = new THREE.Vector3();
  /** World-space cluster nozzle centre, mean exhaust direction and plume length (for particles). */
  readonly cluster = { pos: new THREE.Vector3(), dir: new THREE.Vector3(0, -1, 0), length: 0, power: 0 };

  constructor(model: BoosterModel, rExit: number) {
    const geo = new THREE.CylinderGeometry(1, 2.4, 1, 28, 24, true);
    geo.translate(0, -0.5, 0);
    model.engines.forEach((e, i) => {
      const mat = new THREE.ShaderMaterial({
        vertexShader: vert, fragmentShader: frag, transparent: true, depthWrite: false,
        blending: THREE.AdditiveBlending, side: THREE.DoubleSide, fog: false, toneMapped: false,
        uniforms: { uTime: { value: 0 }, uLevel: { value: 0 }, uSeed: { value: i * 1.37 }, uIntensity: { value: 5.0 } },
      });
      const m = new THREE.Mesh(geo, mat);
      m.position.copy(e.exitLocal);
      m.scale.set(rExit, 1, rExit);
      m.visible = false;
      m.frustumCulled = false;
      m.renderOrder = 10;
      e.pivot.add(m);
      this.meshes.push(m);
      this.mats.push(m.material as THREE.ShaderMaterial);
    });
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const g = c.getContext('2d')!;
    const gr = g.createRadialGradient(64, 64, 0, 64, 64, 64);
    gr.addColorStop(0, 'rgba(255,240,220,1)');
    gr.addColorStop(0.25, 'rgba(255,170,90,0.55)');
    gr.addColorStop(1, 'rgba(255,120,40,0)');
    g.fillStyle = gr; g.fillRect(0, 0, 128, 128);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    this.glow = new THREE.Sprite(new THREE.SpriteMaterial({
      map: tex, blending: THREE.AdditiveBlending, depthWrite: false, transparent: true, toneMapped: false,
      color: new THREE.Color(1.3, 1.0, 0.75), fog: false, opacity: 0.8,
    }));
    this.glow.renderOrder = 11;
  }

  update(s: Sample, model: BoosterModel, time: number, ambientPressure: number): void {
    let n = 0, power = 0, lenSum = 0;
    const cp = this.cluster.pos.set(0, 0, 0), cd = this.cluster.dir.set(0, 0, 0);
    for (let i = 0; i < this.meshes.length; i++) {
      const e = model.engines[i];
      const lit = engineLit(s.lit, i);
      const lvl = lit ? s.levels[e.ring] : s.levels[e.ring] * 0.5;
      const m = this.meshes[i];
      m.visible = lvl > 0.02;
      if (!m.visible) continue;
      // plume grows with throttle and expands as ambient pressure drops
      const expand = 1 + 0.6 * (1 - ambientPressure);
      const len = (7 + 20 * lvl) * (0.9 + 0.2 * expand);
      m.scale.set(0.6 * expand, len, 0.6 * expand);
      const mat = this.mats[i];
      mat.uniforms.uTime.value = time;
      mat.uniforms.uLevel.value = Math.min(1, lvl * 1.4);
      model.nozzle(i, this.tmp, this.tmpDir);
      cp.add(this.tmp); cd.add(this.tmpDir);
      power += lvl; lenSum += len; n++;
    }
    if (n > 0) {
      cp.multiplyScalar(1 / n);
      cd.normalize();
      this.cluster.length = lenSum / n;
    } else {
      this.cluster.length = 0;
    }
    this.cluster.power = power;
    this.light.intensity = power * 4e3;
    this.light.position.copy(cp).addScaledVector(cd, 4);
    this.glow.visible = n > 0;
    this.glow.position.copy(cp).addScaledVector(cd, 1.5);
    const gs = 6 + Math.sqrt(power) * 5;
    this.glow.scale.set(gs, gs, 1);
  }
}
