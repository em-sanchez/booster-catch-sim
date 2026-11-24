import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export type CamMode = 'ground' | 'tower' | 'chase' | 'pad' | 'orbit';
export const CAM_MODES: { id: CamMode; label: string }[] = [
  { id: 'ground', label: 'Beach' },
  { id: 'tower', label: 'Tower' },
  { id: 'chase', label: 'Chase' },
  { id: 'pad', label: 'Pad' },
  { id: 'orbit', label: 'Free' },
];

/** Camera modes: telephoto spectator, tower-top, chase, pad-level, and free orbit. */
export class CameraRig {
  readonly camera: THREE.PerspectiveCamera;
  readonly controls: OrbitControls;
  mode: CamMode = 'ground';
  private towerMount: THREE.Vector3;
  private smoothTarget = new THREE.Vector3();
  private smoothPos = new THREE.Vector3();
  private initialized = false;
  private shakeT = 0;
  private base = new THREE.Vector3();

  constructor(camera: THREE.PerspectiveCamera, dom: HTMLElement, towerMount: THREE.Vector3) {
    this.camera = camera;
    this.towerMount = towerMount.clone();
    this.controls = new OrbitControls(camera, dom);
    this.controls.enableDamping = true;
    this.controls.enabled = false;
    this.controls.maxDistance = 6000;
  }

  setMode(m: CamMode, focus: THREE.Vector3): void {
    this.mode = m;
    this.controls.enabled = m === 'orbit';
    this.initialized = false;
    if (m === 'orbit') {
      this.camera.position.copy(focus).add(new THREE.Vector3(140, 40, 160));
      this.controls.target.copy(focus);
      this.camera.fov = 45;
      this.camera.updateProjectionMatrix();
    }
  }

  /** focus: booster mid-body (world); shake: 0..1 vibration from nearby engines. */
  update(dt: number, focus: THREE.Vector3, shake: number): void {
    const cam = this.camera;
    const k = this.initialized ? 1 - Math.exp(-dt * 6) : 1;
    this.smoothTarget.lerp(focus, k);
    if (!this.initialized) this.smoothTarget.copy(focus);
    let fov = cam.fov;
    switch (this.mode) {
      case 'ground': {
        this.base.set(260, 3, 1350);
        const dist = this.base.distanceTo(this.smoothTarget);
        const frame = 150 + Math.max(0, this.smoothTarget.y - 400) * 0.08;
        fov = THREE.MathUtils.clamp(2 * THREE.MathUtils.radToDeg(Math.atan(frame / 2 / dist)), 2.5, 50);
        break;
      }
      case 'tower':
        this.base.copy(this.towerMount);
        fov = 62;
        break;
      case 'chase': {
        const want = new THREE.Vector3(70, 14, 105).add(focus);
        want.y = Math.max(want.y, 6);
        this.smoothPos.lerp(want, this.initialized ? 1 - Math.exp(-dt * 2.5) : 1);
        if (!this.initialized) this.smoothPos.copy(want);
        this.base.copy(this.smoothPos);
        fov = 50;
        break;
      }
      case 'pad':
        this.base.set(46, 2.2, 64);
        fov = 64;
        break;
      case 'orbit':
        this.controls.target.lerp(focus, 1 - Math.exp(-dt * 3));
        this.controls.update();
        this.initialized = true;
        return;
    }
    this.shakeT += dt;
    cam.position.copy(this.base);
    if (shake > 0) {
      const a = shake * (this.mode === 'ground' ? 0.15 : 0.6);
      cam.position.x += a * (Math.sin(this.shakeT * 37.1) + 0.5 * Math.sin(this.shakeT * 71.3));
      cam.position.y += a * (Math.sin(this.shakeT * 43.7) + 0.5 * Math.sin(this.shakeT * 89.9));
    }
    cam.lookAt(this.smoothTarget);
    if (Math.abs(cam.fov - fov) > 1e-3) {
      cam.fov = this.initialized ? cam.fov + (fov - cam.fov) * (1 - Math.exp(-dt * 4)) : fov;
      cam.updateProjectionMatrix();
    }
    this.initialized = true;
  }
}
