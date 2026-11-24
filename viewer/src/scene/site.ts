import * as THREE from 'three';
import type { Site } from '../playback/flight';
import { concreteTexture, groundTexture, rng, waterNormalTexture } from './textures';

export class SiteModel {
  readonly group = new THREE.Group();
  private water: THREE.Texture;

  constructor(site: Site) {
    const shore = site.shorelineX;

    // ---- ocean ------------------------------------------------------------------------------
    this.water = waterNormalTexture();
    this.water.repeat.set(900, 900);
    const ocean = new THREE.Mesh(
      new THREE.PlaneGeometry(60000, 60000),
      new THREE.MeshPhysicalMaterial({
        color: 0x1d5268, roughness: 0.12, metalness: 0.0, normalMap: this.water,
        normalScale: new THREE.Vector2(0.55, 0.55), envMapIntensity: 1.0, clearcoat: 0.3, clearcoatRoughness: 0.2,
      }),
    );
    ocean.rotation.x = -Math.PI / 2;
    ocean.position.y = -0.6;
    ocean.receiveShadow = true;
    this.group.add(ocean);

    // ---- land and beach --------------------------------------------------------------------
    const gtex = groundTexture();
    gtex.repeat.set(200, 300);
    const landW = 40000, landD = 60000;
    const land = new THREE.Mesh(new THREE.PlaneGeometry(landW, landD),
      new THREE.MeshStandardMaterial({ map: gtex, roughness: 0.95, metalness: 0 }));
    land.rotation.x = -Math.PI / 2;
    land.position.set(shore - 40 - landW / 2, 0, 0);
    land.receiveShadow = true;
    this.group.add(land);
    // beach slope into the water (wet sand gets darker)
    const beachGeo = new THREE.PlaneGeometry(90, landD, 6, 1);
    const pos = beachGeo.attributes.position as THREE.BufferAttribute;
    const colors: number[] = [];
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i);                 // -45 .. 45 across the beach
      const s = (x + 45) / 90;
      pos.setZ(i, -1.4 * s);                 // drops below the water line
      const c = new THREE.Color().lerpColors(new THREE.Color(0xd8c7a0), new THREE.Color(0x8c7a5c), s);
      colors.push(c.r, c.g, c.b);
    }
    beachGeo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    beachGeo.computeVertexNormals();
    const beach = new THREE.Mesh(beachGeo, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.9 }));
    beach.rotation.x = -Math.PI / 2;
    beach.position.set(shore + 5, 0.01, 0);
    beach.receiveShadow = true;
    this.group.add(beach);
    // a line of surf foam
    const foam = new THREE.Mesh(new THREE.PlaneGeometry(6, landD),
      new THREE.MeshStandardMaterial({ color: 0xffffff, transparent: true, opacity: 0.55, roughness: 1 }));
    foam.rotation.x = -Math.PI / 2;
    foam.position.set(shore + 23, -0.5, 0);
    this.group.add(foam);

    // ---- pad apron, roads ------------------------------------------------------------------
    const ctex = concreteTexture();
    ctex.repeat.set(14, 14);
    const apron = new THREE.Mesh(new THREE.CircleGeometry(95, 96),
      new THREE.MeshStandardMaterial({ map: ctex, roughness: 0.85, metalness: 0 }));
    apron.rotation.x = -Math.PI / 2;
    apron.position.set(-12, 0.03, 0);
    apron.receiveShadow = true;
    this.group.add(apron);
    const scorch = new THREE.Mesh(new THREE.CircleGeometry(26, 64),
      new THREE.MeshStandardMaterial({ color: 0x2a2621, roughness: 1, transparent: true, opacity: 0.55 }));
    scorch.rotation.x = -Math.PI / 2;
    scorch.position.set(0, 0.05, 0);
    this.group.add(scorch);
    const roadMat = new THREE.MeshStandardMaterial({ color: 0x3d3b38, roughness: 0.95 });
    const road = (x: number, z: number, w: number, l: number, rot: number) => {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(w, l), roadMat);
      m.rotation.x = -Math.PI / 2;
      m.rotation.z = rot;
      m.position.set(x, 0.04, z);
      m.receiveShadow = true;
      this.group.add(m);
    };
    road(-60, 300, 14, 520, 0);
    road(-400, 560, 14, 700, Math.PI / 2);
    road(-60, -260, 10, 400, 0);

    // ---- tank farm and buildings -----------------------------------------------------------
    const tankMat = new THREE.MeshStandardMaterial({ color: 0xd5d8da, metalness: 0.2, roughness: 0.7 });
    const r = rng(21);
    for (let i = 0; i < 8; i++) {
      const horiz = i < 5;
      const rad = horiz ? 2.4 : 4.5;
      const len = horiz ? 30 : 22 + r() * 8;
      const t = new THREE.Mesh(new THREE.CylinderGeometry(rad, rad, len, 32), tankMat);
      if (horiz) {
        t.rotation.z = Math.PI / 2;
        t.position.set(-150 - (i % 5) * 0, rad + 0.8, 70 + i * 6.5);
      } else {
        t.position.set(-120 - (i - 5) * 12, len / 2, 120);
      }
      t.castShadow = t.receiveShadow = true;
      this.group.add(t);
    }
    const bldg = new THREE.MeshStandardMaterial({ color: 0xb8b2a8, roughness: 0.8 });
    const darkBldg = new THREE.MeshStandardMaterial({ color: 0x4f565c, roughness: 0.6, metalness: 0.3 });
    const blocks: [number, number, number, number, number, THREE.Material][] = [
      [-210, -40, 40, 14, 26, bldg], [-260, 30, 30, 10, 20, darkBldg], [-190, -120, 22, 8, 18, bldg],
      [-520, 520, 110, 52, 70, darkBldg], [-640, 520, 60, 34, 50, bldg], [-300, -300, 18, 6, 18, bldg],
    ];
    for (const [x, z, w, h, d, m] of blocks) {
      const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m);
      b.position.set(x, h / 2, z);
      b.castShadow = b.receiveShadow = true;
      this.group.add(b);
    }
    // scrub bushes scattered inland
    const bushGeo = new THREE.SphereGeometry(1.4, 7, 5);
    const bushMat = new THREE.MeshStandardMaterial({ color: 0x4f5530, roughness: 1, flatShading: true });
    const bushes = new THREE.InstancedMesh(bushGeo, bushMat, 900);
    const m4 = new THREE.Matrix4();
    let n = 0;
    for (let i = 0; i < 900; i++) {
      const x = -1800 + r() * (shore - 70 + 1800), z = -1500 + r() * 3000;
      if (Math.hypot(x + 12, z) < 140) continue;
      const s = 0.6 + r() * 1.6;
      m4.compose(new THREE.Vector3(x, s * 0.25, z), new THREE.Quaternion(), new THREE.Vector3(s * 1.3, s * 0.45, s * 1.3));
      bushes.setMatrixAt(n++, m4);
    }
    bushes.count = n;
    bushes.receiveShadow = true;
    this.group.add(bushes);
  }

  update(time: number): void {
    this.water.offset.set(time * 0.0016, time * 0.0009);
  }
}
