import type { Flight } from '../playback/flight';

const MARKED = new Set(['ignition', 'engine_cutoff', 'terminal', 'catch', 'miss']);
const SHORT: Record<string, string> = {
  ignition: 'ignition', engine_cutoff: '13→3', terminal: 'terminal', catch: 'catch', miss: 'miss',
};

export class Timeline {
  playing = true;
  speed = 1;
  private play = document.getElementById('play') as HTMLButtonElement;
  private scrub = document.getElementById('scrub') as HTMLInputElement;
  private label = document.getElementById('tlabel')!;
  private marks = document.getElementById('marks')!;
  private speedSel = document.getElementById('speed') as HTMLSelectElement;
  private duration = 1;
  private dragging = false;
  onSeek: (t: number) => void = () => {};

  constructor() {
    this.play.addEventListener('click', () => this.toggle());
    this.speedSel.addEventListener('change', () => { this.speed = parseFloat(this.speedSel.value); });
    this.scrub.addEventListener('input', () => {
      this.dragging = true;
      this.onSeek((parseFloat(this.scrub.value) / 1000) * this.duration);
    });
    this.scrub.addEventListener('change', () => { this.dragging = false; });
    window.addEventListener('keydown', (e) => {
      if ((e.target as HTMLElement).tagName === 'SELECT') return;
      if (e.code === 'Space') { e.preventDefault(); this.toggle(); }
    });
  }

  setSpeed(v: number): void {
    this.speed = v;
    this.speedSel.value = String(v);
  }

  toggle(force?: boolean): void {
    this.playing = force ?? !this.playing;
    this.play.textContent = this.playing ? '❚❚' : '▶';
  }

  setFlight(f: Flight): void {
    this.duration = f.duration;
    this.marks.innerHTML = '';
    for (const e of f.events) {
      if (!MARKED.has(e.type)) continue;
      const m = document.createElement('div');
      m.className = 'm';
      m.style.left = `${(e.t / f.duration) * 100}%`;
      m.title = `${e.label} (T+${e.t.toFixed(1)} s)`;
      const s = document.createElement('span');
      s.textContent = SHORT[e.type] ?? e.type;
      m.appendChild(s);
      this.marks.appendChild(m);
    }
  }

  update(t: number): void {
    if (!this.dragging) this.scrub.value = String(Math.round((t / this.duration) * 1000));
    this.label.textContent = `${t.toFixed(1)} / ${this.duration.toFixed(1)} s`;
  }
}
