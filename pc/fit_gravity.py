"""Fit the knee's gravity feedforward from a -DJOINT_TEST=3 serial log.

    python fit_gravity.py log.txt

uq = K sin(angle + PHI) = a sin(angle) + b cos(angle), which is linear in a, b, so plain least squares
fits it. Then K = hypot(a, b), PHI = atan2(b, a). Paste the two numbers into config.h.
"""
import re
import sys

import numpy as np

pts = np.array([[float(x) / 1000 for x in m] for m in re.findall(r"grav: (-?\d+),(-?\d+)", open(sys.argv[1]).read())])
if len(pts) < 3:
    sys.exit("need at least 3 'grav:' lines")
ang, uq = pts[:, 0], pts[:, 1]
A = np.column_stack([np.sin(ang), np.cos(ang)])
(a, b), *_ = np.linalg.lstsq(A, uq, rcond=None)
resid = uq - A @ [a, b]
print(f"#define GRAV_K   {np.hypot(a, b):.3f}f")
print(f"#define GRAV_PHI {np.arctan2(b, a):.3f}f")
# what's left is mostly friction: up and down passes disagree by about twice the friction voltage
print(f"residual rms {np.sqrt(np.mean(resid**2)) * 1000:.0f} mV over {len(pts)} points")
