"""Run flights and write them for the viewer.

    python -m sim.run                 # showcase flights -> viewer/public/flights/
    python -m sim.run --mc 40         # plus a Monte Carlo batch summary
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import warnings
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from . import scenario as S
from .flight import Flight

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "viewer" / "public" / "flights"
RESULTS = ROOT / "results"


def showcase() -> list[S.Scenario]:
    return [
        S.nominal(),
        S.windy(),
        S.dispersed(7, "Dispersed A"),
        S.dispersed(21, "Dispersed B"),
    ]


def _run(sc: S.Scenario, record: bool = True) -> dict:
    warnings.filterwarnings("ignore")
    res = Flight(sc, record=record).run()
    out = {"summary": res.summary()}
    if record:
        out["record"] = res.record
    return out


def _run_mc(seed: int) -> dict:
    return _run(S.dispersed(seed), record=False)["summary"]


def write_flight(sc: S.Scenario, data: dict) -> dict:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{sc.id}.json"
    with open(path, "w") as f:
        json.dump(data["record"], f, separators=(",", ":"))
    s = data["summary"]
    return {"id": sc.id, "name": sc.name, "description": sc.description, "file": f"flights/{sc.id}.json",
            "caught": s["caught"], "errX": s["catch_error_x"], "errY": s["catch_error_y"]}


def stats(values: list[float]) -> dict:
    a = np.array([v for v in values if v is not None and math.isfinite(v)])
    if a.size == 0:
        return {}
    return {"mean": float(a.mean()), "std": float(a.std()), "min": float(a.min()), "max": float(a.max()),
            "p95": float(np.percentile(a, 95))}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mc", type=int, default=0, help="number of Monte Carlo runs")
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--no-showcase", action="store_true")
    args = ap.parse_args()

    t0 = time.perf_counter()
    with Pool(args.jobs) as pool:
        if not args.no_showcase:
            scs = showcase()
            results = pool.map(_run, scs)
            index = []
            for sc, data in zip(scs, results):
                entry = write_flight(sc, data)
                index.append(entry)
                s = data["summary"]
                print(f"{sc.id:14s} caught={s['caught']!s:5s} err=({s['catch_error_x']}, {s['catch_error_y']}) m "
                      f"vz={s['catch_vz']} tilt={s['catch_tilt_deg']} ign_alt={s['ignition_altitude']} "
                      f"fuel={s['fuel_used']} left={s['prop_left']}")
            with open(OUT / "index.json", "w") as f:
                json.dump({"flights": index}, f, indent=1)

        if args.mc:
            seeds = list(range(args.seed0, args.seed0 + args.mc))
            summaries = pool.map(_run_mc, seeds)
            caught = [s for s in summaries if s["caught"]]
            radial = [math.hypot(s["catch_error_x"], s["catch_error_y"]) for s in caught]
            mc = {
                "runs": len(summaries),
                "caught": len(caught),
                "catchRate": len(caught) / max(1, len(summaries)),
                "radialError": stats(radial),
                "errorX": stats([s["catch_error_x"] for s in caught]),
                "errorY": stats([s["catch_error_y"] for s in caught]),
                "sinkRate": stats([-s["catch_vz"] for s in caught]),
                "tiltDeg": stats([s["catch_tilt_deg"] for s in caught]),
                "fuelUsed": stats([s["fuel_used"] for s in summaries]),
                "propLeft": stats([s["prop_left"] for s in summaries]),
                "ignitionAltitude": stats([s["ignition_altitude"] for s in summaries]),
                "maxG": stats([s["max_g"] for s in summaries]),
                "points": [{"seed": seed, "caught": s["caught"], "x": s["catch_error_x"], "y": s["catch_error_y"]}
                           for seed, s in zip(seeds, summaries)],
                "failures": [{"seed": seed, **s} for seed, s in zip(seeds, summaries) if not s["caught"]],
            }
            RESULTS.mkdir(exist_ok=True)
            with open(RESULTS / "monte_carlo.json", "w") as f:
                json.dump(mc, f, indent=1)
            OUT.mkdir(parents=True, exist_ok=True)
            with open(OUT / "montecarlo.json", "w") as f:
                json.dump(mc, f, separators=(",", ":"))
            print(f"Monte Carlo: {mc['caught']}/{mc['runs']} caught; radial error mean "
                  f"{mc['radialError'].get('mean', float('nan')):.3f} m, p95 {mc['radialError'].get('p95', float('nan')):.3f} m")
            for fl in mc["failures"]:
                print("  fail seed", fl["seed"], {k: fl[k] for k in ("catch_error_x", "catch_error_y", "catch_vz",
                                                                    "catch_tilt_deg", "prop_left")})
    print(f"done in {time.perf_counter() - t0:.1f} s")


if __name__ == "__main__":
    main()
