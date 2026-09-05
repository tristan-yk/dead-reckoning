"""Find the held poses in a capture log and derive accelerometer calibration.

Expects a log produced by capture.py while the board is rotated through
orientations, each held still. Segments the log into stable stretches, then uses
the six axis-up/axis-down poses to solve for per-axis bias and scale by the
standard six-position method:

    bias_i  = (a_i_up + a_i_down) / 2
    scale_i = (a_i_up - a_i_down) / (2 * g)
"""
import re
import sys

G = 9.80665

# Stability thresholds, in m/s^2. A pose counts as held when every axis stays
# inside STABLE_RANGE across a window of at least MIN_SAMPLES.
STABLE_RANGE = 0.35
MIN_SAMPLES = 8

ACCEL_RE = re.compile(r"a\[\s*([-+0-9.]+)\s+([-+0-9.]+)\s+([-+0-9.]+)\]")


def read_accel(path):
    samples = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = ACCEL_RE.search(line)
            if m:
                samples.append(tuple(float(g) for g in m.groups()))
    return samples


def segment(samples):
    """Split into maximal runs where no axis moves more than STABLE_RANGE."""
    segments = []
    start = 0
    while start < len(samples):
        lo = list(samples[start])
        hi = list(samples[start])
        end = start
        while end + 1 < len(samples):
            cand = samples[end + 1]
            new_lo = [min(lo[i], cand[i]) for i in range(3)]
            new_hi = [max(hi[i], cand[i]) for i in range(3)]
            if any(new_hi[i] - new_lo[i] > STABLE_RANGE for i in range(3)):
                break
            lo, hi = new_lo, new_hi
            end += 1
        n = end - start + 1
        if n >= MIN_SAMPLES:
            block = samples[start:end + 1]
            mean = tuple(sum(s[i] for s in block) / n for i in range(3))
            segments.append((start, n, mean))
        start = end + 1
    return segments


def main():
    path = sys.argv[1]
    samples = read_accel(path)
    segments = segment(samples)

    print("%d samples, %d held poses\n" % (len(samples), len(segments)))
    print("  #  start   n      ax      ay      az     |a|   dominant")
    for idx, (start, n, mean) in enumerate(segments):
        norm = sum(v * v for v in mean) ** 0.5
        axis = max(range(3), key=lambda i: abs(mean[i]))
        sign = "+" if mean[axis] > 0 else "-"
        print("%3d %6d %3d  %+6.2f  %+6.2f  %+6.2f  %6.2f   %s%s"
              % (idx, start, n, mean[0], mean[1], mean[2], norm,
                 sign, "XYZ"[axis]))

    # For each axis take the most positive and most negative held reading.
    print("\nSix-position calibration")
    print("axis      up      down      bias     scale")
    bias = [0.0] * 3
    scale = [1.0] * 3
    ok = True
    for i in range(3):
        vals = [m[i] for _, _, m in segments]
        up, down = max(vals), min(vals)
        if up < 0.8 * G or down > -0.8 * G:
            print("%s: no clean up/down pair (up=%+.2f down=%+.2f)"
                  % ("XYZ"[i], up, down))
            ok = False
            continue
        bias[i] = (up + down) / 2.0
        scale[i] = (up - down) / (2.0 * G)
        print("  %s  %+7.3f  %+7.3f  %+8.4f  %8.5f"
              % ("XYZ"[i], up, down, bias[i], scale[i]))

    if ok:
        print("\nCorrected = (raw - bias) / scale")
        print("  bias  = {%.4ff, %.4ff, %.4ff}" % tuple(bias))
        print("  scale = {%.5ff, %.5ff, %.5ff}" % tuple(scale))
        print("\nResidual |a| per pose after correction:")
        for idx, (_, _, mean) in enumerate(segments):
            c = [(mean[i] - bias[i]) / scale[i] for i in range(3)]
            print("  pose %2d: %6.3f  (was %6.3f)"
                  % (idx, sum(v * v for v in c) ** 0.5,
                     sum(v * v for v in mean) ** 0.5))
    return 0


if __name__ == "__main__":
    sys.exit(main())
