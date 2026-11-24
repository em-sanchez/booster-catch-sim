import * as THREE from 'three';

/** Small deterministic PRNG so procedural textures look the same every load. */
export function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function canvas(w: number, h: number): [HTMLCanvasElement, CanvasRenderingContext2D] {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  return [c, c.getContext('2d')!];
}

/**
 * Stainless-steel booster skin. Returns a color map (subtle tint, weld rings, soot near the
 * engines) and a roughness map (brushed variation, smoother welds). u wraps around the body,
 * v runs from the engines (v=0) to the top (v=1).
 */
export function steelTextures(lengthM: number): { map: THREE.CanvasTexture; rough: THREE.CanvasTexture } {
  const W = 1024, H = 2048;
  const [c, g] = canvas(W, H);
  const [rc, rg] = canvas(W, H);
  const r = rng(7);
  // base steel tint
  g.fillStyle = '#dfe2e5'; g.fillRect(0, 0, W, H);
  rg.fillStyle = '#5a5a5a'; rg.fillRect(0, 0, W, H);
  // vertical brushed streaks / sheet-to-sheet tone shifts
  const sheets = 8;
  for (let i = 0; i < sheets; i++) {
    const x0 = (i / sheets) * W;
    const tone = 214 + Math.floor(r() * 18);
    g.fillStyle = `rgba(${tone},${tone + 2},${tone + 5},0.55)`;
    g.fillRect(x0, 0, W / sheets, H);
  }
  for (let i = 0; i < 2500; i++) {
    const x = r() * W, y = r() * H, len = 30 + r() * 200;
    const a = 0.03 + r() * 0.05;
    g.fillStyle = r() > 0.5 ? `rgba(255,255,255,${a})` : `rgba(60,60,70,${a})`;
    g.fillRect(x, y, 1, len);
    rg.fillStyle = `rgba(${r() > 0.5 ? 120 : 40},${r() > 0.5 ? 120 : 40},${r() > 0.5 ? 120 : 40},0.08)`;
    rg.fillRect(x, y, 2, len);
  }
  // horizontal weld rings every ~1.8 m (rings of stacked barrel sections)
  const pxPerM = H / lengthM;
  for (let m = 1.8; m < lengthM; m += 1.8) {
    const y = H - m * pxPerM;
    g.fillStyle = 'rgba(90,84,76,0.28)'; g.fillRect(0, y - 1, W, 2);
    g.fillStyle = 'rgba(255,245,230,0.18)'; g.fillRect(0, y + 1, W, 2);
    rg.fillStyle = 'rgba(30,30,30,0.9)'; rg.fillRect(0, y - 2, W, 4);
  }
  // vertical seams
  for (let i = 0; i < sheets; i++) {
    const x = (i / sheets) * W + 3;
    g.fillStyle = 'rgba(90,85,80,0.35)'; g.fillRect(x, 0, 2, H);
  }
  // heat tint + soot: strongest near the engines, streaking upward
  const grad = g.createLinearGradient(0, H, 0, H - 22 * pxPerM);
  grad.addColorStop(0, 'rgba(25,20,16,0.85)');
  grad.addColorStop(0.25, 'rgba(70,52,36,0.45)');
  grad.addColorStop(1, 'rgba(120,90,60,0.0)');
  g.fillStyle = grad; g.fillRect(0, H - 22 * pxPerM, W, 22 * pxPerM);
  for (let i = 0; i < 260; i++) {
    const x = r() * W, len = (4 + r() * 18) * pxPerM;
    g.fillStyle = `rgba(30,24,18,${0.05 + r() * 0.12})`;
    g.fillRect(x, H - len, 3 + r() * 10, len);
  }
  // straw/blue heat tint band at the top (hot-stage area)
  const top = g.createLinearGradient(0, 0, 0, 5 * pxPerM);
  top.addColorStop(0, 'rgba(110,80,60,0.45)'); top.addColorStop(1, 'rgba(110,80,60,0)');
  g.fillStyle = top; g.fillRect(0, 0, W, 5 * pxPerM);

  const map = new THREE.CanvasTexture(c);
  map.colorSpace = THREE.SRGBColorSpace;
  map.anisotropy = 8;
  map.wrapS = THREE.RepeatWrapping;
  const rough = new THREE.CanvasTexture(rc);
  rough.anisotropy = 8;
  rough.wrapS = THREE.RepeatWrapping;
  return { map, rough };
}

/** Soft round puff with a little noise, for smoke/steam particles. */
export function puffTexture(): THREE.CanvasTexture {
  const S = 128;
  const [c, g] = canvas(S, S);
  const r = rng(3);
  const img = g.createImageData(S, S);
  for (let y = 0; y < S; y++) {
    for (let x = 0; x < S; x++) {
      const dx = (x - S / 2) / (S / 2), dy = (y - S / 2) / (S / 2);
      const d = Math.sqrt(dx * dx + dy * dy);
      let a = Math.max(0, 1 - d);
      a = a * a * (3 - 2 * a);
      const n = 0.75 + 0.25 * r();
      const i = (y * S + x) * 4;
      img.data[i] = img.data[i + 1] = img.data[i + 2] = 255;
      img.data[i + 3] = Math.floor(255 * a * n);
    }
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

/** Tileable wave normal map built from a sum of periodic waves. */
export function waterNormalTexture(): THREE.DataTexture {
  const S = 256;
  const data = new Uint8Array(S * S * 4);
  const r = rng(11);
  const waves: { kx: number; ky: number; a: number; p: number }[] = [];
  for (let i = 0; i < 28; i++) {
    const kx = Math.round((r() - 0.5) * 18), ky = Math.round((r() - 0.5) * 18);
    if (kx === 0 && ky === 0) continue;
    waves.push({ kx, ky, a: 1 / Math.hypot(kx, ky), p: r() * Math.PI * 2 });
  }
  for (let y = 0; y < S; y++) {
    for (let x = 0; x < S; x++) {
      let dx = 0, dy = 0;
      for (const w of waves) {
        const ph = (2 * Math.PI * (w.kx * x + w.ky * y)) / S + w.p;
        const c = Math.cos(ph) * w.a;
        dx += c * w.kx; dy += c * w.ky;
      }
      const n = new THREE.Vector3(-dx * 0.08, -dy * 0.08, 1).normalize();
      const i = (y * S + x) * 4;
      data[i] = (n.x * 0.5 + 0.5) * 255;
      data[i + 1] = (n.y * 0.5 + 0.5) * 255;
      data[i + 2] = (n.z * 0.5 + 0.5) * 255;
      data[i + 3] = 255;
    }
  }
  const t = new THREE.DataTexture(data, S, S, THREE.RGBAFormat);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.magFilter = THREE.LinearFilter;
  t.minFilter = THREE.LinearMipmapLinearFilter;
  t.generateMipmaps = true;
  t.needsUpdate = true;
  return t;
}

/** Concrete with stains and expansion joints. */
export function concreteTexture(): THREE.CanvasTexture {
  const S = 512;
  const [c, g] = canvas(S, S);
  const r = rng(5);
  g.fillStyle = '#8c8a84'; g.fillRect(0, 0, S, S);
  for (let i = 0; i < 4000; i++) {
    const v = 110 + Math.floor(r() * 50);
    g.fillStyle = `rgba(${v},${v},${v - 4},0.25)`;
    g.fillRect(r() * S, r() * S, 1 + r() * 3, 1 + r() * 3);
  }
  for (let i = 0; i < 30; i++) {
    const x = r() * S, y = r() * S, rad = 10 + r() * 60;
    const gr = g.createRadialGradient(x, y, 0, x, y, rad);
    gr.addColorStop(0, 'rgba(40,36,30,0.18)'); gr.addColorStop(1, 'rgba(40,36,30,0)');
    g.fillStyle = gr; g.fillRect(x - rad, y - rad, rad * 2, rad * 2);
  }
  g.strokeStyle = 'rgba(50,50,50,0.5)'; g.lineWidth = 2;
  for (let k = 0; k <= 4; k++) {
    g.beginPath(); g.moveTo(0, (k * S) / 4); g.lineTo(S, (k * S) / 4); g.stroke();
    g.beginPath(); g.moveTo((k * S) / 4, 0); g.lineTo((k * S) / 4, S); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}

/** Sand / scrub ground with patches. */
export function groundTexture(): THREE.CanvasTexture {
  const S = 1024;
  const [c, g] = canvas(S, S);
  const r = rng(9);
  g.fillStyle = '#a48e69'; g.fillRect(0, 0, S, S);
  for (let i = 0; i < 260; i++) {
    const x = r() * S, y = r() * S, rad = 20 + r() * 120;
    const green = r() > 0.45;
    const gr = g.createRadialGradient(x, y, 0, x, y, rad);
    gr.addColorStop(0, green ? 'rgba(120,118,76,0.28)' : 'rgba(160,138,100,0.25)');
    gr.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = gr; g.fillRect(x - rad, y - rad, rad * 2, rad * 2);
  }
  for (let i = 0; i < 30000; i++) {
    const v = r();
    g.fillStyle = v > 0.5 ? 'rgba(230,214,180,0.18)' : 'rgba(80,70,50,0.18)';
    g.fillRect(r() * S, r() * S, 1.5, 1.5);
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}
