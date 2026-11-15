"""Draws the walls of sim/data/map.pgm as lines with a crest of 255: sim/data/map_ridge.pgm.

The cells of value 128 or more are thinned to a one-cell skeleton, and every cell gets
255 x exp(-d^2 / (2 sigma^2)), d being its distance to the skeleton, sigma 0.6 cells.
Needs SciPy and scikit-image. Usage: python experiments/make_ridge_map.py
"""
import os
import numpy as np
from scipy import ndimage
from skimage.morphology import skeletonize

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIGMA = 0.6
src = os.path.join(ROOT, "sim", "data", "map.pgm")
with open(src, "rb") as fh:
    parts = fh.read().split(maxsplit=4)
w, h = int(parts[1]), int(parts[2])
grid = np.frombuffer(parts[4][: w * h], dtype=np.uint8).reshape(h, w)
d = ndimage.distance_transform_edt(~skeletonize(grid >= 128))
ridge = np.round(255 * np.exp(-d ** 2 / (2 * SIGMA ** 2))).astype(np.uint8)
with open(os.path.join(ROOT, "sim", "data", "map_ridge.pgm"), "wb") as fh:
    fh.write(b"P5\n%d %d\n255\n" % (w, h))
    fh.write(ridge.tobytes())
print(f"map_ridge.pgm: {w} x {h}, {int((ridge == 255).sum())} cells at 255")
