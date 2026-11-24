import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import type { Site } from '../playback/flight';

const D2R = Math.PI / 180;

/** Collects straight members and builds one InstancedMesh of unit boxes. */
class Beams {
  private mats: THREE.Matrix4[] = [];
  private up = new THREE.Vector3(0, 1, 0);
  add(a: THREE.Vector3, b: THREE.Vector3, t: number, t2 = t): void {
    const d = new THREE.Vector3().subVectors(b, a);
    const len = d.length();
    const q = new THREE.Quaternion().setFromUnitVectors(this.up, d.normalize());
    const m = new THREE.Matrix4().compose(new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5), q,
      new THREE.Vector3(t, len, t2));
    this.mats.push(m);
  }
  build(mat: THREE.Material): THREE.InstancedMesh {
    const mesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), mat, this.mats.length);
    this.mats.forEach((m, i) => mesh.setMatrixAt(i, m));
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }
}

const v = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z);

export class TowerModel {
  readonly group = new THREE.Group();
  private arms: { pivot: THREE.Group; sign: number }[] = [];

  private openRad: number;
  private closedRad: number;
  readonly cameraMount: THREE.Vector3;

  constructor(site: Site, boosterRadius: number) {

    const paint = new THREE.MeshStandardMaterial({ color: 0x8d9296, metalness: 0.25, roughness: 0.7 });
    const darkPaint = new THREE.MeshStandardMaterial({ color: 0x55595d, metalness: 0.3, roughness: 0.65 });
    const steel = new THREE.MeshStandardMaterial({ color: 0x8a8d90, metalness: 0.8, roughness: 0.4 });
    const pad = new THREE.MeshStandardMaterial({ color: 0xd9a441, metalness: 0.3, roughness: 0.6 });

    // ---- lattice tower --------------------------------------------------------------------
    const cx = site.towerX, w = site.towerWidth / 2, H = site.towerHeight;
    const beams = new Beams();
    const corners = [v(cx - w, 0, -w), v(cx + w, 0, -w), v(cx + w, 0, w), v(cx - w, 0, w)];
    const levels = 20;
    const dz = H / levels;
    for (const c of corners) beams.add(c, v(c.x, H, c.z), 1.1);
    for (let k = 0; k <= levels; k++) {
      const y = k * dz;
      for (let i = 0; i < 4; i++) {
        const a = corners[i], b = corners[(i + 1) % 4];
        beams.add(v(a.x, y, a.z), v(b.x, y, b.z), 0.55);
      }
      if (k < levels) {
        for (let i = 0; i < 4; i++) {
          const a = corners[i], b = corners[(i + 1) % 4];
          beams.add(v(a.x, y, a.z), v(b.x, y + dz, b.z), 0.35);
          beams.add(v(b.x, y, b.z), v(a.x, y + dz, a.z), 0.35);
        }
      }
    }
    // internal stair/elevator core
    beams.add(v(cx, 0, 0), v(cx, H, 0), 2.4, 2.4);
    this.group.add(beams.build(paint));

    // crown: platform, mast and a light
    const crown = new THREE.Mesh(new THREE.BoxGeometry(site.towerWidth + 2, 1.2, site.towerWidth + 2), darkPaint);
    crown.position.set(cx, H + 0.6, 0);
    crown.castShadow = true;
    this.group.add(crown);
    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.6, 16, 10), steel);
    mast.position.set(cx, H + 9, 0);
    this.group.add(mast);
    const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.5, 12, 8), new THREE.MeshBasicMaterial({ color: new THREE.Color(4, 0.2, 0.15), toneMapped: false }));
    beacon.position.set(cx, H + 17.2, 0);
    this.group.add(beacon);
    this.cameraMount = v(cx + w + 1, H + 3, -w - 1);

    // ---- carriage at arm height -----------------------------------------------------------
    const zA = site.armHeight;
    const carriage = new Beams();
    const cw = w + 1.6;
    for (const yy of [zA - 6, zA + 1]) {
      const cs = [v(cx - cw, yy, -cw), v(cx + cw, yy, -cw), v(cx + cw, yy, cw), v(cx - cw, yy, cw)];
      for (let i = 0; i < 4; i++) carriage.add(cs[i], cs[(i + 1) % 4], 1.2);
    }
    for (const c of [v(cx - cw, 0, -cw), v(cx + cw, 0, -cw), v(cx + cw, 0, cw), v(cx - cw, 0, cw)]) {
      carriage.add(v(c.x, zA - 6, c.z), v(c.x, zA + 1, c.z), 1.0);
    }
    this.group.add(carriage.build(darkPaint));

    // ---- arms ------------------------------------------------------------------------------
    // Hinge at (hingeX, +/-hingeY) sim -> three z = -/+hingeY. Arm points along local +X.
    this.openRad = site.armOpenDeg * D2R;
    const closedY = boosterRadius + 0.4 + 0.9;  // arm centreline when closed against the booster
    this.closedRad = Math.atan2(closedY - site.armHingeY, -site.armHingeX);
    for (const sign of [1, -1]) {             // sign = +1 for the sim +y arm
      const pivot = new THREE.Group();
      pivot.position.set(site.armHingeX, zA, -sign * site.armHingeY);
      const arm = this.buildArm(site.armLength, paint, darkPaint, pad, sign);
      pivot.add(arm);
      this.group.add(pivot);
      this.arms.push({ pivot, sign });
    }

    // ---- launch mount ----------------------------------------------------------------------
    const mh = site.mountHeight, mr = site.mountRadius + 1.5;
    const shape = new THREE.Shape();
    shape.absarc(0, 0, mr, 0, Math.PI * 2, false);
    const hole = new THREE.Path();
    hole.absarc(0, 0, 5.2, 0, Math.PI * 2, true);
    shape.holes.push(hole);
    const tableGeo = new THREE.ExtrudeGeometry(shape, { depth: 3.0, bevelEnabled: false, curveSegments: 64 });
    tableGeo.rotateX(-Math.PI / 2);
    tableGeo.translate(0, mh - 3.0, 0);
    const table = new THREE.Mesh(tableGeo, darkPaint);
    table.castShadow = table.receiveShadow = true;
    this.group.add(table);
    const legs = new Beams();
    for (let i = 0; i < 6; i++) {
      const a = (i / 6) * Math.PI * 2 + Math.PI / 6;
      const r0 = mr - 1.2;
      legs.add(v(Math.cos(a) * r0, 0, Math.sin(a) * r0), v(Math.cos(a) * r0, mh - 3, Math.sin(a) * r0), 2.2);
      const b = a + Math.PI / 3;
      legs.add(v(Math.cos(a) * r0, 2, Math.sin(a) * r0), v(Math.cos(b) * r0, mh - 4, Math.sin(b) * r0), 0.6);
    }
    this.group.add(legs.build(paint));
    // clamps on the table
    const clamps: THREE.BufferGeometry[] = [];
    for (let i = 0; i < 20; i++) {
      const g = new THREE.BoxGeometry(0.8, 1.2, 1.4);
      g.translate(5.6, mh + 0.6, 0);
      g.rotateY((i / 20) * Math.PI * 2);
      clamps.push(g);
    }
    this.group.add(new THREE.Mesh(mergeGeometries(clamps)!, steel));
    // flame deflector plate
    const plate = new THREE.Mesh(new THREE.CylinderGeometry(10, 11, 0.8, 48), steel);
    plate.position.y = 0.4;
    plate.receiveShadow = true;
    this.group.add(plate);

    // tower-to-mount bridge (piping / walkway)
    const bridge = new Beams();
    bridge.add(v(cx + w, 18, -1.2), v(-mr + 0.5, 18, -1.2), 1.2);
    bridge.add(v(cx + w, 18, 1.2), v(-mr + 0.5, 18, 1.2), 1.2);
    this.group.add(bridge.build(steel));
  }

  private buildArm(L: number, paint: THREE.Material, dark: THREE.Material, pad: THREE.Material, sign: number): THREE.Group {
    const g = new THREE.Group();
    const h = 3.6, wdt = 1.8;
    const b = new Beams();
    const topY = 0, botY = -h;
    for (const z of [-wdt / 2, wdt / 2]) {
      b.add(v(0, topY, z), v(L, topY, z), 0.45);
      b.add(v(0, botY, z), v(L * 0.98, botY + 0.6, z), 0.45);
      const n = 12;
      for (let i = 0; i < n; i++) {
        const x0 = (i / n) * L, x1 = ((i + 1) / n) * L;
        b.add(v(x0, botY + 0.6 * (x0 / L), z), v(x1, topY, z), 0.22);
        b.add(v(x1, botY + 0.6 * (x1 / L), z), v(x1, topY, z), 0.22);
      }
    }
    for (let i = 0; i <= 12; i++) {
      const x = (i / 12) * L;
      b.add(v(x, topY, -wdt / 2), v(x, topY, wdt / 2), 0.2);
    }
    g.add(b.build(paint));
    // hinge drum
    const drum = new THREE.Mesh(new THREE.CylinderGeometry(1.2, 1.2, h + 1, 20), dark);
    drum.position.y = -h / 2;
    drum.castShadow = true;
    g.add(drum);
    // catch rail on the inner top edge (the booster's hardpoints settle here)
    const inner = sign > 0 ? 1 : -1;      // +y arm sits at three z<0, so its inner side is +z
    const rail = new THREE.Mesh(new THREE.BoxGeometry(L * 0.55, 0.35, 0.7), pad);
    rail.position.set(L * 0.62, 0.15, inner * (wdt / 2 - 0.2));
    rail.castShadow = true;
    g.add(rail);
    // closing actuator / shock absorber
    const act = new THREE.Mesh(new THREE.CylinderGeometry(0.35, 0.35, 6, 12), dark);
    act.rotation.z = Math.PI / 2;
    act.position.set(3, -h + 0.4, 0);
    g.add(act);
    return g;
  }

  /** a = 0 open, 1 closed. */
  setArms(a: number): void {
    const ang = this.openRad + (this.closedRad - this.openRad) * a;
    for (const arm of this.arms) arm.pivot.rotation.y = arm.sign * ang;
  }
}
