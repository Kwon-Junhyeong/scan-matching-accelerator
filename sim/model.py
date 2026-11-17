"""Integer model of one pass (fixed_pass), and a runner for the RTL bench (run_rtl)."""
import os
import subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
Q14 = 1 << 14
PHASE_PI = 8192.0
DEG_TO_PHASE = 2982616          # round(8192/180 x 2^16), as in rtl/Stage1_Coord_Calc.v


def wrap(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def load_pgm(path):
    with open(path, "rb") as fh:
        data = fh.read()
    parts = data.split(maxsplit=4)
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(parts[4][: w * h], dtype=np.uint8).reshape(h, w).astype(int)


def beam_phase(heading, angle):
    """Phase the RTL hands to the CORDIC core: heading + angle x 8192/180 (rounded), modulo one turn."""
    beam = ((angle * DEG_TO_PHASE + (1 << 15)) >> 16) & 0x3FFF
    return wrap((heading & 0x3FFF) + beam, 14)


def beam_rad(heading, angle):
    """Exact beam direction in radians: heading in phase units, angle in degrees."""
    return heading / PHASE_PI * np.pi + np.radians(angle)


def fixed_pass(points, sincos, px, py, heading, tau, grid):
    """points: [(angle, dist)], sincos: [(sin, cos)] from the core, px/py in 14 fraction bits."""
    mh, mw = grid.shape
    H = [0] * 6
    K = [0] * 3
    n_valid = 0
    for (a, d), (s, c) in zip(points, sincos):
        # stage 1
        dcos, dsin = d * c, d * s
        gx = wrap((dcos >> 1) + px, 32)
        gy = wrap((dsin >> 1) + py, 32)
        dtx = wrap(-(dsin >> 1), 32)
        dty = wrap(dcos >> 1, 32)
        # stage 2
        xi, yi = gx >> 14, gy >> 14
        fx, fy = gx & 0x3FFF, gy & 0x3FFF
        if 0 <= xi < mw - 1 and 0 <= yi < mh - 1:
            p00, p10 = int(grid[yi, xi]), int(grid[yi, xi + 1])
            p01, p11 = int(grid[yi + 1, xi]), int(grid[yi + 1, xi + 1])
        else:
            p00 = p10 = p01 = p11 = 0
        # stage 3
        ifx, ify = Q14 - fx, Q14 - fy
        m28 = p00 * ifx * ify + p10 * fx * ify + p01 * ifx * fy + p11 * fx * fy
        m16 = wrap(m28 >> 12, 32)
        grx = wrap((p10 - p00) * ify + (p11 - p01) * fy, 32)
        gry = wrap((p01 - p00) * ifx + (p11 - p10) * fx, 32)
        f = wrap((255 << 16) - m16, 32)
        if m16 < tau:
            continue
        # stage 4
        j3 = wrap(((grx * dtx) >> 14) + ((gry * dty) >> 14), 32)
        J = (grx, gry, j3)
        # stage 5
        for k, (i, j) in enumerate(((0, 0), (0, 1), (0, 2), (1, 1), (1, 2))):
            H[k] = wrap(H[k] + J[i] * J[j], 64)
        H[5] = wrap(H[5] + ((J[2] * J[2]) >> 8), 64)
        for i in range(3):
            K[i] = wrap(K[i] + ((J[i] * f) >> 10), 64)
        n_valid += 1
    return H + K, n_valid


def run_rtl(points, px, py, heading, tau, grid, build):
    """Runs one pass of the RTL. Returns (sums, [(sin, cos)], clocks)."""
    mh, mw = grid.shape
    os.makedirs(build, exist_ok=True)
    with open(os.path.join(build, "cfg.txt"), "w") as fh:
        fh.write(f"{len(points)} {px} {py} {heading & 0xFFFFFFFF} {tau} {mw} {mh}\n")
    with open(os.path.join(build, "map.hex"), "w") as fh:
        fh.write("\n".join("%08x" % v for v in grid.reshape(-1)) + "\n")
    with open(os.path.join(build, "points.hex"), "w") as fh:
        fh.write("\n".join("%08x" % (((a & 0xFFFF) << 16) | d) for a, d in points) + "\n")
    env = dict(os.environ, BUILD=build)
    subprocess.run(["sh", os.path.join(HERE, "run.sh"), "tb_pass"], env=env, check=True,
                   stdout=subprocess.DEVNULL)
    sincos, sums, clocks = [], None, None
    with open(os.path.join(build, "pass.txt")) as fh:
        for line in fh:
            t = line.split()
            if t[0] == "P":
                sincos.append((int(t[1]), int(t[2])))
            elif t[0] == "C":
                clocks = dict(zip(("first_accept", "stage1", "fifo", "stage2", "stage3", "stage4", "sums",
                                   "start_to_irq"), map(int, t[1:])))
            elif t[0] == "S":
                sums = [int(v) for v in t[1:]]
    return sums, sincos, clocks
