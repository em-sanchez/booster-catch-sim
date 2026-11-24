import * as THREE from 'three';
import './style.css';
import { Particles } from './effects/particles';
import { Plumes } from './effects/plume';
import { Post } from './effects/post';
import { Flight, loadFlight, loadIndex, Sample, simToThree, type FlightIndexEntry } from './playback/flight';
import { BoosterModel } from './scene/booster';
import { buildEnvironment } from './scene/environment';
import { SiteModel } from './scene/site';
import { TowerModel } from './scene/tower';
import { CAM_MODES, CameraRig, type CamMode } from './ui/cameras';
import { Hud } from './ui/hud';
import { initMonteCarlo } from './ui/montecarlo';
import { Timeline } from './ui/timeline';

const params = new URLSearchParams(location.search);
if (params.get('hud') === '0') document.body.classList.add('clean');

// ---- renderer / scene ------------------------------------------------------------------------
const canvas = document.getElementById('view') as HTMLCanvasElement;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: false, powerPreference: 'high-performance', preserveDrawingBuffer: params.has('shot') });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.55;

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.5, 60000);
const lighting = buildEnvironment(renderer, scene);
const post = new Post(renderer, scene, camera);
const sunColor = new THREE.Color(1.0, 0.95, 0.88);

const hud = new Hud();
const timeline = new Timeline();
const sample = new Sample();

let flight: Flight | null = null;
let booster: BoosterModel | null = null;
let tower: TowerModel | null = null;
let site: SiteModel | null = null;
let plumes: Plumes | null = null;
let rig: CameraRig | null = null;
const particles = new Particles(6000);
scene.add(particles.points);

const planLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x5fd4ff, transparent: true, opacity: 0.85, fog: false }));
const trailLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffc070, transparent: true, opacity: 0.6, fog: false }));
planLine.frustumCulled = trailLine.frustumCulled = false;
scene.add(planLine, trailLine);
const showPlan = document.getElementById('showPlan') as HTMLInputElement;
const showTrail = document.getElementById('showTrail') as HTMLInputElement;

let t = Math.max(0, parseFloat(params.get('t') ?? '0') || 0);
let lastT = t;
let camMode = (params.get('cam') as CamMode) || 'ground';
timeline.toggle(params.get('play') !== '0');
if (params.has('speed')) timeline.setSpeed(parseFloat(params.get('speed')!));
timeline.onSeek = (v) => { t = v; };

// ---- camera buttons --------------------------------------------------------------------------
const camBox = document.getElementById('cams')!;
const camButtons = new Map<CamMode, HTMLButtonElement>();
CAM_MODES.forEach((m, i) => {
  const b = document.createElement('button');
  b.textContent = `${i + 1} ${m.label}`;
  b.addEventListener('click', () => setCam(m.id));
  camBox.appendChild(b);
  camButtons.set(m.id, b);
});
const focus = new THREE.Vector3();
function setCam(m: CamMode): void {
  camMode = m;
  camButtons.forEach((b, id) => b.classList.toggle('active', id === m));
  rig?.setMode(m, focus);
}
window.addEventListener('keydown', (e) => {
  const tag = (e.target as HTMLElement).tagName;
  if (tag === 'INPUT' && (e.target as HTMLInputElement).type !== 'range') return;
  const n = parseInt(e.key, 10);
  if (n >= 1 && n <= CAM_MODES.length) setCam(CAM_MODES[n - 1].id);
  if (e.key === 'ArrowRight') t = Math.min((flight?.duration ?? 0), t + 1);
  if (e.key === 'ArrowLeft') t = Math.max(0, t - 1);
  if (e.key === 'h') document.body.classList.toggle('clean');
});

// ---- flights ---------------------------------------------------------------------------------
const select = document.getElementById('flight') as HTMLSelectElement;
const resultEl = document.getElementById('result')!;
let index: FlightIndexEntry[] = [];

async function setFlight(id: string): Promise<void> {
  const entry = index.find((f) => f.id === id) ?? index[0];
  const f = await loadFlight(entry.file);
  flight = f;
  const meta = f.meta;
  if (!booster) {
    booster = new BoosterModel(meta.vehicle);
    scene.add(booster.group);
    plumes = new Plumes(booster, meta.vehicle.engineExitDiameter / 2);
    scene.add(plumes.light, plumes.glow);
    tower = new TowerModel(meta.site, meta.vehicle.diameter / 2);
    scene.add(tower.group);
    site = new SiteModel(meta.site);
    scene.add(site.group);
    rig = new CameraRig(camera, canvas, tower.cameraMount);
    setCam(camMode);
  }
  timeline.setFlight(f);
  particles.reset();
  const r = meta.result;
  const ok = r.caught;
  const err = r.catch_error_x !== null && r.catch_error_y !== null ? Math.hypot(r.catch_error_x, r.catch_error_y) : null;
  resultEl.innerHTML = [
    `<span class="${ok ? 'ok' : 'fail'}">${ok ? 'CAUGHT' : 'MISSED'}</span>`,
    err !== null ? `miss distance ${err.toFixed(2)} m` : '',
    r.catch_vz !== null ? `contact ${(-r.catch_vz).toFixed(2)} m/s, ${r.catch_tilt_deg?.toFixed(1)}° tilt` : '',
    r.ignition_altitude !== null ? `ignition at ${r.ignition_altitude.toFixed(0)} m` : '',
    r.prop_left !== null ? `${(r.prop_left / 1000).toFixed(1)} t propellant left` : '',
    `<span style="color:var(--muted)">${meta.scenario.description}</span>`,
  ].filter(Boolean).join('<br>');
  if (t > f.duration) t = 0;
}

select.addEventListener('change', () => { void setFlight(select.value); });

// ---- resize ------------------------------------------------------------------------------------
function resize(): void {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h);
  post.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

// ---- main loop ---------------------------------------------------------------------------------
const clock = new THREE.Clock();
let wallTime = 0;
const tmp = new THREE.Vector3();
const windThree = new THREE.Vector3();
const velThree = new THREE.Vector3();
let framesAfterLoad = 0;

function setLine(line: THREE.Line, pts: THREE.Vector3[] | null, visible: boolean): void {
  line.visible = visible && !!pts && pts.length > 1;
  if (!line.visible || !pts) return;
  (line.geometry as THREE.BufferGeometry).setFromPoints(pts);
}

function frame(): void {
  requestAnimationFrame(frame);
  const dtReal = Math.min(clock.getDelta(), 0.1);
  wallTime += dtReal;
  if (!flight || !booster || !tower || !plumes || !rig || !site) return;
  if (timeline.playing) t = Math.min(flight.duration, t + dtReal * timeline.speed);
  const dtSim = t - lastT;
  if (dtSim < -0.01 || dtSim > 1.0) particles.reset();
  lastT = t;

  flight.sample(t, sample);
  booster.update(sample);
  tower.setArms(sample.arm);
  booster.group.updateMatrixWorld(true);
  const pAmb = Math.exp(-Math.max(sample.simPos.z, 0) / 8500);
  plumes.update(sample, booster, wallTime, pAmb);

  simToThree(sample.wind.x, sample.wind.y, sample.wind.z, windThree);
  simToThree(sample.simVel.x, sample.simVel.y, sample.simVel.z, velThree);
  const c = plumes.cluster;
  particles.setScale(renderer.domElement.height, camera.fov);
  particles.update(Math.max(0, Math.min(dtSim, 0.1)), c.pos, c.dir, c.length, c.power, velThree, windThree,
    flight.meta.site.mountHeight, flight.meta.site.mountRadius + 1.5, plumes.light, sunColor);
  site.update(wallTime);

  // camera focus: booster mid-body
  focus.set(0, 32, 0).applyQuaternion(sample.quat).add(sample.pos);
  const dist = camera.position.distanceTo(c.pos);
  const shake = c.power > 0 ? Math.min(1, (c.power * 2.2e4) / (dist * dist + 1e3)) : 0;
  rig.update(dtReal, focus, shake);

  // shadow frustum follows the action near the pad
  tmp.copy(focus);
  tmp.y = 0;
  tmp.clampLength(0, 120);
  lighting.sun.target.position.copy(tmp);
  lighting.sun.position.copy(tmp).addScaledVector(lighting.sunDir, 700);

  setLine(planLine, sample.phase >= 1 && sample.phase <= 2 ? flight.planAt(t) : null, showPlan.checked);
  setLine(trailLine, flight.trail(t), showTrail.checked);

  hud.update(sample, flight);
  timeline.update(t);
  post.composer.render(dtReal);
  if (framesAfterLoad < 1000) framesAfterLoad++;
  if (framesAfterLoad === 8) (window as unknown as { __ready: boolean }).__ready = true;
}

// ---- boot ------------------------------------------------------------------------------------
(async () => {
  index = await loadIndex();
  for (const f of index) {
    const o = document.createElement('option');
    o.value = f.id;
    o.textContent = `${f.name}${f.caught ? '' : ' (miss)'}`;
    select.appendChild(o);
  }
  const want = params.get('flight') ?? index[0].id;
  select.value = want;
  await setFlight(want);
  void initMonteCarlo();
  if (params.get('mc') === '1') {
    const box = document.getElementById('showMc') as HTMLInputElement;
    box.checked = true;
    document.getElementById('mc')!.hidden = false;
  }
  resize();
  frame();
})();
