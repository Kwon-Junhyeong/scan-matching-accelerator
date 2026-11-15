"""Scan matching on the test map: where the Gauss-Newton passes end, with the RTL and in double precision.

1. Ridge map. sim/data/map.pgm has walls blurred over about one cell; their crests sit below 255 and vary
   along each wall, while the residual f = 255 - M pulls every point toward 255. sim/data/map_ridge.pgm draws the
   same walls (the skeleton of the cells of value 128 or more) as lines with a Gaussian profile of
   sigma 0.6 cells and a crest of 255. Made by experiments/make_ridge_map.py.
2. Both maps, double precision: scans of 360 beams from random poses (seed 3), each ray-cast to the first
   wall crest along the beam, distances rounded to half cells. Gauss-Newton starts at the true pose and from
   3 starts per pose, up to 2 cells off in x and in y and 3 degrees, 8 passes.
3. Ridge map, RTL in the loop for the first poses, 6 passes, the pose rounded into the register formats
   after every pass, next to double precision from the same start.

Usage: python experiments/convergence.py [poses] [rtl poses, default 0]   (Vivado as for sim/run.sh; BUILD default build/)
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
import model

POSES = int(sys.argv[1]) if len(sys.argv) > 1 else 30
RTL_POSES = int(sys.argv[2]) if len(sys.argv) > 2 else 0
BUILD = os.environ.get("BUILD", os.path.join(ROOT, "build"))
DEG = model.PHASE_PI / 180.0
OUT = os.path.join(HERE, "logs")
os.makedirs(OUT, exist_ok=True)


def map_value(G, x, y):
    mh, mw = G.shape
    ix, iy = int(np.floor(x)), int(np.floor(y))
    if not (0 <= ix < mw - 1 and 0 <= iy < mh - 1):
        return None
    fx, fy = x - ix, y - iy
    return (G[iy, ix] * (1 - fx) * (1 - fy) + G[iy, ix + 1] * fx * (1 - fy)
            + G[iy + 1, ix] * (1 - fx) * fy + G[iy + 1, ix + 1] * fx * fy)


def raycast(G, x0, y0, phi):
    """Distance to the first wall crest of the interpolated map along the beam."""
    prev, rising = None, False
    for r in np.arange(0.5, 85, 0.02):
        m = map_value(G, x0 + r * np.cos(phi), y0 + r * np.sin(phi))
        if m is None:
            return None
        if m > 100:
            rising = True
        if rising and prev is not None and m < prev:
            return r - 0.02
        prev = m
    return None


def scan(G, T):
    pts = []
    for a in range(360):
        r = raycast(G, T[0], T[1], model.beam_rad(T[2], a))
        if r is not None:
            pts.append((a, int(round(2 * r))))
    return pts


def step(H, g):
    dx = np.linalg.solve(H, g)
    return np.array([dx[0], dx[1], dx[2] / np.pi * model.PHASE_PI])


def err(p, T):
    return np.hypot(p[0] - T[0], p[1] - T[1]), abs((p[2] - T[2] + 8192) % 16384 - 8192) / DEG


def to_regs(p):
    return int(round(p[0] * model.Q14)), int(round(p[1] * model.Q14)), int(round(p[2]))


def double_run(G, pts, p, n):
    p = np.array(p, float)
    for _ in range(n):
        H, g, _ = model.double_pass(pts, p[0], p[1], p[2], G)
        p = p + step(H, g)
    return p


def poses(test, ridge, n, rng):
    """Random poses in free cells at least 3 cells from any wall line."""
    mh, mw = test.shape
    free = [(y, x) for y in range(3, mh - 3) for x in range(3, mw - 3)
            if test[y, x] == 0 and ridge[y - 3:y + 4, x - 3:x + 4].max() == 0]
    out = []
    while len(out) < n:
        cy, cx = free[rng.integers(len(free))]
        out.append(np.array([cx + rng.random(), cy + rng.random(), rng.uniform(-8192, 8192)]))
    return out


def stats(v):
    v = np.asarray(v)
    return f"median {np.median(v):.3f}, 95th percentile {np.percentile(v, 95):.3f}, largest {v.max():.3f}"


test = model.load_pgm(os.path.join(ROOT, "sim", "data", "map.pgm"))
ridge = model.load_pgm(os.path.join(ROOT, "sim", "data", "map_ridge.pgm"))
rng = np.random.default_rng(3)
P = poses(test, ridge, POSES, rng)
starts = [[np.array([rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-3, 3) * DEG]) for _ in range(3)] for _ in P]
scans = {"test": [scan(test.astype(float), T) for T in P], "ridge": [scan(ridge.astype(float), T) for T in P]}

log = open(os.path.join(OUT, "convergence.txt"), "w")


def say(s):
    print(s, flush=True)
    log.write(s + "\n")
    log.flush()


say(f"{POSES} poses, 360 beams each; returns per scan: test map {np.mean([len(s) for s in scans['test']]):.0f}, "
    f"ridge map {np.mean([len(s) for s in scans['ridge']]):.0f} on average")
for name, G in (("test", test), ("ridge", ridge)):
    tr, fp, fh = [], [], []
    for T, pts, S in zip(P, scans[name], starts):
        tr.append(err(double_run(G, pts, T, 8), T)[0])
        for o in S:
            e = err(double_run(G, pts, T + o, 8), T)
            fp.append(e[0])
            fh.append(e[1])
    say(f"{name} map, double precision, 8 passes:")
    say(f"  started at the true pose, distance after 8 passes (cells): {stats(tr)}")
    say(f"  from {len(fp)} starts, distance from the true pose (cells): {stats(fp)}")
    say(f"  from {len(fp)} starts, heading error (deg): {stats(fh)}")

# RTL in the loop on the ridge map (optional second argument)
if RTL_POSES == 0:
    sys.exit(0)
rows = []
compiled = False
for i in range(min(RTL_POSES, POSES)):
    T, pts = P[i], scans["ridge"][i]
    for j, o in enumerate(starts[i]):
        pd = T + o
        pr = np.array(to_regs(pd), float) / [model.Q14, model.Q14, 1]
        rows.append((i, j, 0, *err(pd, T), *err(pr, T), 0.0))
        for k in range(6):
            if compiled:
                os.environ["NOCOMPILE"] = "1"
            sums, _, _ = model.run_rtl(pts, *to_regs(pr), 0, ridge, BUILD)
            compiled = True
            Hr, gr = model.sums_to_float(sums)
            Hd, gd, _ = model.double_pass(pts, pd[0], pd[1], pd[2], ridge)
            pd = pd + step(Hd, gd)
            pr = pr + step(Hr, gr)
            pr = np.array(to_regs(pr), float) / [model.Q14, model.Q14, 1]
            rows.append((i, j, k + 1, *err(pd, T), *err(pr, T), np.hypot(*(pr - pd)[:2])))
R = np.array(rows)
last = R[R[:, 2] == 6]
say(f"ridge map, RTL in the loop: {RTL_POSES} poses x 3 starts, 6 passes")
say(f"  after pass 6, RTL distance from the true pose (cells): {stats(last[:, 5])}")
say(f"  after pass 6, RTL heading error (deg): {stats(last[:, 6])}")
say(f"  after pass 6, double distance from the true pose (cells): {stats(last[:, 3])}")
say(f"  RTL pose against double pose, all passes (cells): largest {R[:, 7].max():.4f}")
np.savetxt(os.path.join(OUT, "convergence_rtl.dat"), R, fmt="%.5f",
           header="pose start pass dbl_cells dbl_deg rtl_cells rtl_deg rtl_minus_dbl_cells", comments="")
