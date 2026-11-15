"""Bit-exact check: the sums of the RTL against fixed_pass() on scans of the test map.

Each scan: 360 beams 1 degree apart, ray-cast from a random free cell of sim/data/map.pgm
to the nearest wall, with a random heading.
Usage: python sim/check_model.py [number of scans] [threshold on M, map value]   (BUILD as for run.sh, default build/)
"""
import os
import sys
import numpy as np
import model

N_SCANS = int(sys.argv[1]) if len(sys.argv) > 1 else 20
TAU = int(round(float(sys.argv[2]) * 65536)) if len(sys.argv) > 2 else 0
ROOT = os.path.dirname(model.HERE)
BUILD = os.environ.get("BUILD", os.path.join(ROOT, "build"))
grid = model.load_pgm(os.path.join(model.HERE, "data", "map.pgm"))
mh, mw = grid.shape
rng = np.random.default_rng(1)


def raycast(x0, y0, phi):
    for r in np.arange(0.5, 120, 0.05):
        ix, iy = int(np.floor(x0 + r * np.cos(phi))), int(np.floor(y0 + r * np.sin(phi)))
        if not (0 <= ix < mw and 0 <= iy < mh):
            return None
        if grid[iy, ix] >= 200:
            return r
    return None


free = np.argwhere(grid == 0)
n_match, n_points, n_valid, sc_err = 0, 0, 0, 0
for k in range(N_SCANS):
    while True:
        cy, cx = free[rng.integers(len(free))]
        if 10 < cx < mw - 10 and 10 < cy < mh - 10:
            break
    x, y = cx + rng.random(), cy + rng.random()
    heading = int(rng.integers(-8192, 8192))
    pts = []
    for a in range(360):
        r = raycast(x, y, model.beam_rad(heading, a))
        if r is not None and r < 120:
            pts.append((a, int(round(2 * r))))
    px, py = int(round(x * model.Q14)), int(round(y * model.Q14))
    if k > 0:
        os.environ["NOCOMPILE"] = "1"
    sums, sincos, clocks = model.run_rtl(pts, px, py, heading, TAU, grid, BUILD)
    ref, nv = model.fixed_pass(pts, sincos, px, py, heading, TAU, grid)
    for (a, d), (s, c) in zip(pts, sincos):
        phi = model.beam_phase(heading, a) / model.PHASE_PI * np.pi
        sc_err = max(sc_err, abs(s - np.sin(phi) * model.Q14), abs(c - np.cos(phi) * model.Q14))
    same = sums == ref
    n_match += same
    n_points += len(pts)
    n_valid += nv
    print(f"scan {k:2d}: {len(pts):3d} points, {nv:3d} at or above the threshold, {clocks['start_to_irq']:5d} clocks, "
          f"sums {'identical' if same else 'DIFFERENT'}")
    if not same:
        print("  rtl  ", sums)
        print("  model", ref)
print(f"threshold {TAU / 65536:g}: {n_match} of {N_SCANS} scans identical to the model in all nine sums "
      f"({n_points} points, {n_valid} at or above the threshold)")
print(f"largest CORDIC sine or cosine difference from the exact value: {sc_err:.2f} LSB of 2^-14")
