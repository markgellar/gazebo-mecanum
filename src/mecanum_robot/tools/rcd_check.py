#!/usr/bin/env python3
"""Can walls tell us the robot's heading? Offline test on recorded spins.

Usage:
    python3 rcd_check.py [run_dir ...]      # default: every run in ~/mecanum_ws/slam_records

Regions of Constant Depth (Leonard & Durrant-Whyte): while a wide-cone range sensor sweeps across a
flat wall, its reading stays constant for about one cone width, because the wall's perpendicular
stays inside the cone. The middle of that run points along the wall's normal, far more precisely
than the cone itself.

Indoors, walls are mostly at 90 deg to each other (Manhattan world). So the RCD bearings modulo 90 deg,
measured in the map frame, should cluster at the robot's heading error. This compares that estimate
with the true heading error from pose_error, spin by spin.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam_replay as R  # noqa: E402


def sensor_sequences(rays, map_odom):
    """Each sensor's readings in time order: list of (sensor xy, heading, range, hit) arrays."""
    s = R.odom_to_map(rays[:, 0:2], map_odom)
    e = R.odom_to_map(rays[:, 2:4], map_odom)
    v = e - s
    heading = np.arctan2(v[:, 1], v[:, 0])
    rng = np.hypot(v[:, 0], v[:, 1])
    hit = rays[:, 4] > 0.5

    if rays.shape[1] >= 8:   # recorded sensor id + stamp
        groups = [np.where(rays[:, 6] == sid)[0] for sid in np.unique(rays[:, 6])]
        groups = [g[np.argsort(rays[g, 7])] for g in groups]
    else:
        # Older recordings: rebuild each sensor's track. A sensor moves ~2 mm between its own
        # readings; different sensors are >= 10 cm apart.
        tracks, last = [], []
        for i, p in enumerate(rays[:, 0:2]):
            d = [np.hypot(*(p - q)) for q in last]
            k = int(np.argmin(d)) if d and min(d) < 0.03 else -1
            if k < 0:
                tracks.append([]); last.append(p); k = len(tracks) - 1
            tracks[k].append(i); last[k] = p
        groups = [np.array(t) for t in tracks if len(t) > 20]
    return [(s[g], heading[g], rng[g], hit[g]) for g in groups]


def find_rcds(seq, cone, tol=0.005, width_slack=math.radians(6)):
    """RCDs as (centre bearing, range, width, sensor position): the plateau of readings within `tol` of a run's minimum range,
    about one cone wide. Its middle points along the wall normal (or at a corner)."""
    pos, heading, rng, hit = seq
    h = np.unwrap(heading)
    rcds, i, n = [], 0, len(rng)
    while i < n:
        if not hit[i]:
            i += 1
            continue
        # One continuous stretch of hits on the same surface(s)
        j = i
        while j + 1 < n and hit[j + 1] and abs(h[j + 1] - h[j]) < 0.1 and abs(rng[j + 1] - rng[j]) < 0.05:
            j += 1
        # Grow the plateau out from the closest reading. The slopes either side (the wall still visible
        # at the cone's edge, slightly further away) are excluded by the tight tolerance.
        k = i + int(np.argmin(rng[i:j + 1]))
        rmin = rng[k]
        a = b = k
        while a - 1 >= i and rng[a - 1] <= rmin + tol:
            a -= 1
        while b + 1 <= j and rng[b + 1] <= rmin + tol:
            b += 1
        width = abs(h[b] - h[a])
        if cone - width_slack <= width <= cone + width_slack:
            mid = (a + b) // 2
            rcds.append(((h[a] + h[b]) / 2, float(rmin), width, pos[mid]))
        i = j + 1
    return rcds


def wrap90(a):
    """Angle folded into (-45, 45] deg, in radians."""
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def manhattan_heading(bearings, inlier=math.radians(4)):
    """Robust mod-90 direction: circular mean of 4*angle, then refit on inliers."""
    if len(bearings) == 0:
        return None, 0, float('nan')
    b = np.array(bearings)
    est = np.angle(np.mean(np.exp(4j * b))) / 4
    for _ in range(3):
        resid = wrap90(b - est)
        keep = np.abs(resid) < inlier
        if not keep.any():
            break
        est = wrap90(est + np.angle(np.mean(np.exp(4j * resid[keep]))) / 4)
    resid = wrap90(b - est)
    keep = np.abs(resid) < inlier
    spread = float(np.degrees(np.median(np.abs(resid[keep])))) if keep.any() else float('nan')
    return est, int(keep.sum()), spread


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dirs', nargs='*')
    args = ap.parse_args()
    runs = args.run_dirs or sorted(glob.glob(os.path.join(R.RECORD_ROOT, '*/')))

    errors = []
    print('Heading error at each spin: measured from wall directions vs truth (pose_error).\n')
    print('%-22s %-9s | %5s %7s | %10s %10s %8s' % ('run', 'spin', 'RCDs', 'inliers', 'from walls', 'truth', 'diff'))
    print('-' * 82)
    for run in runs:
        for spin in R.load_run(run):
            rays = spin['rays']
            cone = 2 * (float(np.median(rays[:, 5])) if rays.shape[1] > 5 else 0.22)
            rcds = [r for seq in sensor_sequences(rays, spin['map_odom']) for r in find_rcds(seq, cone)]
            est, inliers, spread = manhattan_heading([r[0] for r in rcds])

            truth = None
            if spin['truth']:
                truth = wrap90(spin['truth']['estimate'][2] - spin['truth']['truth'][2])
            row = '%-22s %-9s | %5d %7d | ' % (os.path.basename(run.rstrip('/')), spin['name'], len(rcds), inliers)
            if est is None:
                print(row + '%10s' % 'no walls')
                continue
            diff = math.degrees(wrap90(est - truth)) if truth is not None else float('nan')
            print(row + '%+9.2f° %+9.2f° %+7.2f°' % (
                math.degrees(est), math.degrees(truth) if truth is not None else float('nan'), diff))
            if truth is not None and inliers >= 3:
                errors.append(abs(diff))

    if errors:
        e = np.array(errors)
        print('-' * 82)
        print('Spins with >= 3 wall inliers: %d   |diff| median %.2f°, 90th pct %.2f°, max %.2f°' % (
            len(e), np.median(e), np.percentile(e, 90), e.max()))
        print('Within 1°: %.0f%%   within 2°: %.0f%%' % (100 * np.mean(e <= 1), 100 * np.mean(e <= 2)))


if __name__ == '__main__':
    main()
