interface McStat { mean: number; std: number; min: number; max: number; p95: number }
interface McFile {
  runs: number; caught: number; catchRate: number; radialError: McStat; sinkRate: McStat; tiltDeg: McStat;
  propLeft: McStat; ignitionAltitude: McStat; maxG: McStat;
  points: { seed: number; caught: boolean; x: number | null; y: number | null }[];
}

/** Scatter of hardpoint catch errors from the Monte Carlo batch, with the arm tolerance box. */
export async function initMonteCarlo(): Promise<void> {
  const panel = document.getElementById('mc')!;
  const toggle = document.getElementById('showMc') as HTMLInputElement;
  toggle.addEventListener('change', () => { panel.hidden = !toggle.checked; });
  let data: McFile;
  try {
    const r = await fetch('flights/montecarlo.json');
    if (!r.ok) throw new Error('missing');
    data = await r.json();
  } catch {
    toggle.parentElement!.style.display = 'none';
    return;
  }
  const c = document.getElementById('mcplot') as HTMLCanvasElement;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  c.width = 220 * dpr; c.height = 220 * dpr;
  const g = c.getContext('2d')!;
  g.scale(dpr, dpr);
  const S = 220, span = 5;                 // +/- 5 m
  const px = (m: number) => S / 2 + (m / span) * (S / 2 - 10);
  g.fillStyle = 'rgba(255,255,255,0.03)'; g.fillRect(0, 0, S, S);
  g.strokeStyle = 'rgba(255,255,255,0.12)'; g.lineWidth = 1;
  for (let m = -4; m <= 4; m += 2) {
    g.beginPath(); g.moveTo(px(m), 0); g.lineTo(px(m), S); g.stroke();
    g.beginPath(); g.moveTo(0, px(m)); g.lineTo(S, px(m)); g.stroke();
  }
  // tolerance: |x| < 4 m along the arms, |y| < 1.5 m across them
  g.strokeStyle = 'rgba(255,181,71,0.8)'; g.setLineDash([4, 3]);
  g.strokeRect(px(-4), px(-1.5), px(4) - px(-4), px(1.5) - px(-1.5));
  g.setLineDash([]);
  g.fillStyle = 'rgba(255,255,255,0.5)'; g.font = '10px sans-serif';
  g.fillText('x along arms (m)', 6, S - 6);
  g.save(); g.translate(10, 8); g.fillText('y across arms', 0, 6); g.restore();
  for (const p of data.points) {
    if (p.x === null || p.y === null) continue;
    g.beginPath();
    g.arc(px(p.x), px(-p.y), 2.6, 0, Math.PI * 2);
    g.fillStyle = p.caught ? 'rgba(79,209,139,0.85)' : 'rgba(255,107,107,0.9)';
    g.fill();
  }
  const s = document.getElementById('mcstats')!;
  s.innerHTML = [
    `${data.caught}/${data.runs} caught (${(data.catchRate * 100).toFixed(0)} %)`,
    `miss distance  ${data.radialError.mean.toFixed(2)} m mean, ${data.radialError.p95.toFixed(2)} m p95`,
    `sink at contact ${data.sinkRate.mean.toFixed(2)} m/s`,
    `tilt at contact ${data.tiltDeg.mean.toFixed(2)}°`,
    `propellant left ${(data.propLeft.mean / 1000).toFixed(1)} t mean`,
  ].join('<br>');
}
