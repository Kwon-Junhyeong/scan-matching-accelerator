"""Models of one Gauss-Newton pass, and a runner for the RTL bench.

fixed_pass():  the arithmetic of rtl/ step by step, integer for integer. It takes the sine and
               cosine that the CORDIC core produced in simulation, so it checks everything after
               the core.
double_pass(): the same equations in double precision, with exact sine and cosine.
run_rtl():     writes the inputs, runs sim/tb_pass.v through sim/run.sh, reads pass.txt.

Units: x, y in map cells (14 fraction bits in the registers), heading in CORDIC phase units
(8192 = pi), beam angle in degrees (0..359), distance in half cells, map cells 0..255.
"""
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


def sums_to_float(sums):
    h11, h12, h13, h22, h23, h33, k1, k2, k3 = sums
    H = np.array([[h11, h12, h13], [h12, h22, h23], [h13, h23, h33 * 2.0 ** 8]], float) / 2.0 ** 28
    g = np.array([k1, k2, k3], float) / 2.0 ** 20
    return H, g


def double_pass(points, x, y, heading, grid, tau=0.0):
    """x, y in cells, heading in phase units. J3 is per radian."""
    mh, mw = grid.shape
    H = np.zeros((3, 3))
    g = np.zeros(3)
    n = 0
    for a, d in points:
        phi = beam_rad(heading, a)
        c, s = np.cos(phi), np.sin(phi)
        px, py = d * c / 2 + x, d * s / 2 + y
        ix, iy = int(np.floor(px)), int(np.floor(py))
        if not (0 <= ix < mw - 1 and 0 <= iy < mh - 1):
            p00 = p10 = p01 = p11 = 0.0
            fx = fy = 0.0
        else:
            fx, fy = px - ix, py - iy
            p00, p10 = grid[iy, ix], grid[iy, ix + 1]
            p01, p11 = grid[iy + 1, ix], grid[iy + 1, ix + 1]
        M = p00 * (1 - fx) * (1 - fy) + p10 * fx * (1 - fy) + p01 * (1 - fx) * fy + p11 * fx * fy
        if M < tau:
            continue
        gx = (p10 - p00) * (1 - fy) + (p11 - p01) * fy
        gy = (p01 - p00) * (1 - fx) + (p11 - p10) * fx
        J = np.array([gx, gy, gx * (-d * s / 2) + gy * (d * c / 2)])
        H += np.outer(J, J)
        g += J * (255.0 - M)
        n += 1
    return H, g, n


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
