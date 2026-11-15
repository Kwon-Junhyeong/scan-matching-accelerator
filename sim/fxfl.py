"""Fixed point (the RTL in simulation) against double precision (model.double_pass), over
Gauss-Newton passes on the test map.

A scan of 360 beams 1 degree apart is ray-cast from a true pose on sim/data/map.pgm to the top of the first wall ridge of the
interpolated map. Matching starts from a pose that is off by up to 2 cells in x and in y and 3 degrees. Each pass sends the scan through the RTL, reads the nine sums, and
solves the 3 x 3 system in double precision, as the ARM cores do; the new pose goes back into the
registers rounded to their formats. The double path runs the same loop without rounding.
Usage: python sim/fxfl.py [cases] [passes]   (BUILD as for run.sh, default build/)
"""
import os
import sys
import numpy as np
import model

CASES = int(sys.argv[1]) if len(sys.argv) > 1 else 5
PASSES = int(sys.argv[2]) if len(sys.argv) > 2 else 6
ROOT = os.path.dirname(model.HERE)
BUILD = os.environ.get("BUILD", os.path.join(ROOT, "build"))
grid = model.load_pgm(os.path.join(model.HERE, "data", "map.pgm"))
mh, mw = grid.shape
DEG = model.PHASE_PI / 180.0                  # phase units per degree


def map_value(x, y):
    ix, iy = int(np.floor(x)), int(np.floor(y))
    if not (0 <= ix < mw - 1 and 0 <= iy < mh - 1):
        return None
    fx, fy = x - ix, y - iy
    return (grid[iy, ix] * (1 - fx) * (1 - fy) + grid[iy, ix + 1] * fx * (1 - fy)
            + grid[iy + 1, ix] * (1 - fx) * fy + grid[iy + 1, ix + 1] * fx * fy)


def raycast(x0, y0, phi):
    """Distance to the top of the first wall ridge of the interpolated map, as the matcher sees it."""
    prev, rising = None, False
    for r in np.arange(0.5, 85, 0.02):
        m = map_value(x0 + r * np.cos(phi), y0 + r * np.sin(phi))
        if m is None:
            return None
        if m > 100:
            rising = True
        if rising and prev is not None and m < prev:
            return r - 0.02
        prev = m
    return None


# true pose: the free cell near the centre with the most free space around it
best, ty, tx = -1, 0, 0
for y in range(60, 120):
    for x in range(60, 120):
        if grid[y, x] == 0:
            s = int((grid[y - 6:y + 7, x - 6:x + 7] == 0).sum())
            if s > best:
                best, ty, tx = s, y, x
TRUE = np.array([tx + 0.5, ty + 0.5, -7400.0])
pts = []
for a in range(360):
    r = raycast(TRUE[0], TRUE[1], model.beam_rad(TRUE[2], a))
    if r is not None:
        pts.append((a, int(round(2 * r))))
print(f"true pose x {TRUE[0]:.2f} y {TRUE[1]:.2f} cells, heading {TRUE[2] / DEG:.2f} deg; {len(pts)} beams with a return")


def to_regs(p):
    return int(round(p[0] * model.Q14)), int(round(p[1] * model.Q14)), int(round(p[2]))


def step(H, g):
    dx = np.linalg.solve(H, g)
    return np.array([dx[0], dx[1], dx[2] / np.pi * model.PHASE_PI])


def err(p):
    return np.hypot(p[0] - TRUE[0], p[1] - TRUE[1]), abs(p[2] - TRUE[2]) / DEG


rng = np.random.default_rng(7)
offsets = [np.array([1.5, -1.0, 120.0])]
while len(offsets) < CASES:
    offsets.append(np.array([rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-3, 3) * DEG]))

compiled = False
for c, off in enumerate(offsets):
    pd = TRUE + off
    pr = np.array(to_regs(pd), float) / [model.Q14, model.Q14, 1]
    e0 = err(pd)
    print(f"case {c}: start {e0[0]:.3f} cells and {e0[1]:.3f} deg from the true pose")
    for k in range(PASSES):
        if compiled:
            os.environ["NOCOMPILE"] = "1"
        rx, ry, rt = to_regs(pr)
        sums, _, clocks = model.run_rtl(pts, rx, ry, rt, 0, grid, BUILD)
        compiled = True
        Hr, gr = model.sums_to_float(sums)
        if k == 0:
            Hs, gs, _ = model.double_pass(pts, rx / model.Q14, ry / model.Q14, rt, grid)
            dr, ds = step(Hr, gr), step(Hs, gs)
            print(f"  first pass, same pose: largest difference in H {np.max(np.abs(Hr - Hs)) / np.max(np.abs(Hs)):.2e},"
                  f" in g {np.max(np.abs(gr - gs)) / np.max(np.abs(gs)):.2e} (relative to the largest entry);"
                  f" update differs by {np.hypot(*(dr - ds)[:2]):.4f} cells and {abs(dr[2] - ds[2]) / DEG:.4f} deg;"
                  f" {clocks['start_to_irq']} clocks")
        Hd, gd, _ = model.double_pass(pts, pd[0], pd[1], pd[2], grid)
        pd = pd + step(Hd, gd)
        pr = pr + step(Hr, gr)
        pr = np.array(to_regs(pr), float) / [model.Q14, model.Q14, 1]
        ed, er = err(pd), err(pr)
        print(f"  pass {k + 1}: double {ed[0]:.4f} cells {ed[1]:.4f} deg   rtl {er[0]:.4f} cells {er[1]:.4f} deg"
              f"   rtl - double {np.hypot(*(pr - pd)[:2]):.4f} cells {abs(pr[2] - pd[2]) / DEG:.4f} deg")
