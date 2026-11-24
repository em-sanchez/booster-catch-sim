// Renders key moments of a flight to PNGs with a headless browser (uses the installed Edge or Chrome).
//   npm run build && npx vite preview --port 4173   (in another terminal)
//   npm run shots -- [flightId] [outDir]
import { chromium } from 'playwright-core';
import fs from 'node:fs';
import path from 'node:path';

const flightId = process.argv[2] ?? 'nominal';
const outDir = path.resolve(process.argv[3] ?? '../screenshots');
const base = process.env.SHOT_URL ?? 'http://localhost:4173/';
fs.mkdirSync(outDir, { recursive: true });

const data = JSON.parse(fs.readFileSync(path.resolve('public/flights', `${flightId}.json`), 'utf8'));
const ev = Object.fromEntries(data.events.map((e) => [e.type, e.t]));
const end = data.samples.t[data.samples.t.length - 1];
const shots = [
  ['01-descent', 4, 'ground'],
  ['02-ignition', (ev.ignition ?? 10) + 1.2, 'chase'],
  ['03-three-engines', (ev.engine_cutoff ?? 16) + 3, 'ground'],
  ['04-approach', (ev.terminal ?? 28) - 3, 'tower'],
  ['05-terminal', (ev.terminal ?? 28) + 2, 'pad'],
  ['06-catch', (ev.catch ?? end - 6) + 0.3, 'chase'],
  ['07-held', end - 0.5, 'orbit'],
];

const channel = process.env.SHOT_CHANNEL ?? 'msedge';
const browser = await chromium.launch({
  channel, headless: true,
  args: ['--use-angle=d3d11', '--enable-gpu', '--ignore-gpu-blocklist', '--enable-unsafe-swiftshader'],
});
const page = await browser.newPage({ viewport: { width: 1600, height: 900 }, deviceScaleFactor: 1 });
page.on('console', (m) => { if (m.type() === 'error') console.log('[page]', m.text()); });
for (const [name, t, cam] of shots) {
  const url = `${base}?flight=${flightId}&t=${t.toFixed(2)}&cam=${cam}&play=0&shot=1${name === '07-held' ? '&mc=1' : ''}`;
  await page.goto(url);
  await page.waitForFunction(() => window.__ready === true, null, { timeout: 120000 });
  await page.waitForTimeout(600);
  const file = path.join(outDir, `${flightId}-${name}.png`);
  await page.screenshot({ path: file });
  console.log('saved', file);
}
await browser.close();
