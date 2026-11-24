import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { engineLit, type Sample, type VehicleMeta } from '../playback/flight';
import { steelTextures } from './textures';

const D2R = Math.PI / 180;

/** Box from p (centre) with size, optionally rotated about Y; returns a geometry ready to merge. */
function box(sx: number, sy: number, sz: number, px: number, py: number, pz: number, rotY = 0): THREE.BufferGeometry {
  const g = new THREE.BoxGeometry(sx, sy, sz);
  if (rotY) g.rotateY(rotY);
  g.translate(px, py, pz);
  return g;
}

/** Grid-fin lattice: a frame plus 45-degree members clipped to the rectangle (X radial, Z tangential). */
function gridFinGeometry(): THREE.BufferGeometry {
  const x0 = 0.35, x1 = 4.9, z0 = -2.9, z1 = 2.9, depth = 0.75, t = 0.07, spacing = 0.62;
  const parts: THREE.BufferGeometry[] = [];
  const frame = 0.16;
  parts.push(box(x1 - x0, depth, frame, (x0 + x1) / 2, 0, z0));
  parts.push(box(x1 - x0, depth, frame, (x0 + x1) / 2, 0, z1));
  parts.push(box(frame, depth, z1 - z0, x0, 0, 0));
  parts.push(box(frame, depth, z1 - z0, x1, 0, 0));
  for (const sgn of [1, -1]) {
    // lines z = sgn * x + c, clipped to the rectangle
    for (let c = -12; c <= 12; c += spacing) {
      const pts: [number, number][] = [];
      const add = (x: number, z: number) => {
        if (x >= x0 - 1e-6 && x <= x1 + 1e-6 && z >= z0 - 1e-6 && z <= z1 + 1e-6) pts.push([x, z]);
      };
      add(x0, sgn * x0 + c); add(x1, sgn * x1 + c);
      add((z0 - c) / sgn, z0); add((z1 - c) / sgn, z1);
      if (pts.length < 2) continue;
      pts.sort((a, b) => a[0] - b[0]);
      const [a, b] = [pts[0], pts[pts.length - 1]];
      const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (len < 0.2) continue;
      parts.push(box(len, depth * 0.96, t, (a[0] + b[0]) / 2, 0, (a[1] + b[1]) / 2, -Math.atan2(b[1] - a[1], b[0] - a[0])));
    }
  }
  return mergeGeometries(parts)!;
}

function bellGeometry(rExit: number, length: number): THREE.LatheGeometry {
  const pts: THREE.Vector2[] = [];
  const rThroat = rExit * 0.34;
  for (let i = 0; i <= 24; i++) {
    const s = i / 24;                           // 0 throat .. 1 exit
    const r = rThroat + (rExit - rThroat) * Math.pow(s, 0.62);
    pts.push(new THREE.Vector2(r, -0.25 - s * length));
  }
  return new THREE.LatheGeometry(pts, 40);
}

export interface EngineNode {
  pivot: THREE.Group;          // gimbal point (rotates)
  glow: THREE.MeshBasicMaterial;
  ring: number;
  exitLocal: THREE.Vector3;    // nozzle exit in pivot frame
}

export class BoosterModel {
  readonly group = new THREE.Group();
  readonly engines: EngineNode[] = [];
  private fins: THREE.Group[] = [];

  readonly bellLength = 1.6;
  private tmpQ = new THREE.Quaternion();
  private tmpV = new THREE.Vector3();
  private down = new THREE.Vector3(0, -1, 0);

  constructor(meta: VehicleMeta) {

    const R = meta.diameter / 2, L = meta.length;
    const { map, rough } = steelTextures(L);
    const steel = new THREE.MeshStandardMaterial({
      color: 0xffffff, map, roughnessMap: rough, metalness: 0.7, roughness: 0.42, envMapIntensity: 1.3,
    });
    const darkSteel = new THREE.MeshStandardMaterial({ color: 0x3b3936, metalness: 0.85, roughness: 0.5 });
    const finMat = new THREE.MeshStandardMaterial({ color: 0x4a4744, metalness: 0.8, roughness: 0.55 });
    const ventMat = new THREE.MeshStandardMaterial({ color: 0x0c0b0a, metalness: 0.4, roughness: 0.8 });

    // main barrel: one cylinder whose UV v spans the whole length so the texture lines up
    const bodyTop = 68.0;
    const body = new THREE.Mesh(new THREE.CylinderGeometry(R, R, bodyTop - 3.2, 128, 1, true), steel);
    body.position.y = 3.2 + (bodyTop - 3.2) / 2;
    const uv = body.geometry.attributes.uv as THREE.BufferAttribute;
    for (let i = 0; i < uv.count; i++) uv.setY(i, (3.2 + uv.getY(i) * (bodyTop - 3.2)) / L);
    body.castShadow = body.receiveShadow = true;
    this.group.add(body);

    // aft skirt with a slight flare
    const skirt = new THREE.Mesh(new THREE.CylinderGeometry(R, R * 1.025, 3.2, 128, 1, true), steel);
    skirt.position.y = 1.6;
    const suv = skirt.geometry.attributes.uv as THREE.BufferAttribute;
    for (let i = 0; i < suv.count; i++) suv.setY(i, (suv.getY(i) * 3.2) / L);
    skirt.castShadow = true;
    this.group.add(skirt);
    const shield = new THREE.Mesh(new THREE.CircleGeometry(R * 1.02, 96), new THREE.MeshStandardMaterial({ color: 0x141312, roughness: 0.9, metalness: 0.2 }));
    shield.rotation.x = Math.PI / 2;
    shield.position.y = 0.35;
    this.group.add(shield);

    // vented hot-staging ring and top bulkhead
    const ring = new THREE.Mesh(new THREE.CylinderGeometry(R, R, L - bodyTop, 128, 1, true), darkSteel);
    ring.position.y = bodyTop + (L - bodyTop) / 2;
    ring.castShadow = true;
    this.group.add(ring);
    const vents: THREE.BufferGeometry[] = [];
    for (let i = 0; i < 36; i++) {
      const a = (i / 36) * Math.PI * 2;
      const g = new THREE.BoxGeometry(0.55, 1.7, 0.2);
      g.translate(0, 0, R + 0.02);
      g.rotateY(a);
      g.translate(0, bodyTop + 1.5, 0);
      vents.push(g);
    }
    this.group.add(new THREE.Mesh(mergeGeometries(vents)!, ventMat));
    const cap = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.995, R, 0.3, 96), darkSteel);
    cap.position.y = L - 0.15;
    this.group.add(cap);
    const capRim = new THREE.Mesh(new THREE.TorusGeometry(R, 0.12, 8, 96), darkSteel);
    capRim.rotation.x = Math.PI / 2;
    capRim.position.y = bodyTop;
    this.group.add(capRim);

    // cable raceway and small chines (azimuth in sim body frame; model = rotate about +Y by azimuth)
    const addAlongBody = (azDeg: number, w: number, d: number, y0: number, y1: number, mat: THREE.Material) => {
      const g = new THREE.BoxGeometry(d, y1 - y0, w);
      g.translate(R + d / 2 - 0.05, (y0 + y1) / 2, 0);
      g.rotateY(azDeg * D2R);
      const m = new THREE.Mesh(g, mat);
      m.castShadow = true;
      this.group.add(m);
    };
    addAlongBody(0, 0.7, 0.35, 3.5, 66, darkSteel);
    addAlongBody(180, 0.5, 0.25, 6, 30, darkSteel);

    // catch hardpoints (rest on the arms), facing body +/-y
    for (const az of [90, -90]) {
      const g = new THREE.BoxGeometry(1.1, 0.9, 1.6);
      g.translate(R + 0.5, meta.hHardpoint + 0.45, 0);
      g.rotateY(az * D2R);
      const m = new THREE.Mesh(g, darkSteel);
      m.castShadow = true;
      this.group.add(m);
    }

    // grid fins
    const finGeo = gridFinGeometry();
    const housingGeo = new THREE.BoxGeometry(1.0, 1.6, 1.8);
    housingGeo.translate(R + 0.3, 0, 0);
    for (const azDeg of meta.gridfinAzimuthDeg) {
      const mount = new THREE.Group();
      mount.rotation.y = azDeg * D2R;
      mount.position.y = meta.hGridfin;
      const housing = new THREE.Mesh(housingGeo, darkSteel);
      housing.castShadow = true;
      mount.add(housing);
      const hinge = new THREE.Group();
      hinge.position.x = R + 0.2;
      const fin = new THREE.Mesh(finGeo, finMat);
      fin.castShadow = true;
      hinge.add(fin);
      mount.add(hinge);
      this.group.add(mount);
      this.fins.push(hinge);
    }

    // engines: bell (outer) + glowing inner surface, on a gimbal pivot at the engine plane
    const rExit = meta.engineExitDiameter / 2;
    const bell = bellGeometry(rExit, this.bellLength);
    const bellMat = new THREE.MeshStandardMaterial({ color: 0x47403a, metalness: 0.9, roughness: 0.35, side: THREE.FrontSide });
    const powerhead = new THREE.CylinderGeometry(0.38, 0.32, 0.9, 20);
    powerhead.translate(0, 0.3, 0);
    const phMat = new THREE.MeshStandardMaterial({ color: 0x2a2826, metalness: 0.7, roughness: 0.6 });
    meta.engines.forEach((e) => {
      const pivot = new THREE.Group();
      pivot.position.set(e.x, 0.0, -e.y);
      const outer = new THREE.Mesh(bell, bellMat);
      outer.castShadow = true;
      const glow = new THREE.MeshBasicMaterial({ color: 0x000000, side: THREE.BackSide, toneMapped: false });
      const inner = new THREE.Mesh(bell, glow);
      inner.scale.setScalar(0.985);
      pivot.add(outer, inner, new THREE.Mesh(powerhead, phMat));
      this.group.add(pivot);
      this.engines.push({ pivot, glow, ring: e.ring, exitLocal: new THREE.Vector3(0, -0.25 - this.bellLength, 0) });
    });
  }

  update(s: Sample): void {
    this.group.position.copy(s.pos);
    this.group.quaternion.copy(s.quat);
    // gimbal: thrust direction in body = normalize(tan dx, tan dy, 1) -> model (x, 1, -y);
    // the nozzle points the opposite way (exhaust direction).
    const tx = Math.tan(s.gimbal[0] * D2R), ty = Math.tan(s.gimbal[1] * D2R);
    this.tmpV.set(-tx, -1, ty).normalize();
    this.tmpQ.setFromUnitVectors(this.down, this.tmpV);
    for (let i = 0; i < this.engines.length; i++) {
      const e = this.engines[i];
      if (e.ring < 2) e.pivot.quaternion.copy(this.tmpQ);
      const lit = engineLit(s.lit, i);
      const lvl = lit ? Math.max(s.levels[e.ring], 0.05) : s.levels[e.ring] * 0.6;
      const k = Math.min(lvl, 1);
      e.glow.color.setRGB(3.2 * k, 1.6 * k, 0.9 * k);
    }
    for (let i = 0; i < this.fins.length; i++) this.fins[i].rotation.x = s.fins[i] * D2R;
  }

  /** Nozzle-exit position and exhaust direction of engine i in world space. */
  nozzle(i: number, pos: THREE.Vector3, dir: THREE.Vector3): void {
    const e = this.engines[i];
    pos.copy(e.exitLocal);
    e.pivot.localToWorld(pos);
    dir.set(0, -1, 0).applyQuaternion(e.pivot.getWorldQuaternion(this.tmpQ));
  }
}
