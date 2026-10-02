#!/usr/bin/env python3
"""Offline prototype: heading correction from a map of walls, replayed spin by spin.

Usage:
    python3 wall_heading.py [run_dir ...] [--verbose]    # default: every run in ~/mecanum_ws/slam_records

For each recorded run, as if it were running live:
  1. Each spin's Regions of Constant Depth (rcd_check.find_rcds) give wall sightings:
     a direction (the RCD's centre bearing) and the surface point it saw.
  2. Sightings are associated with walls already in the map (similar direction, close in position).
  3. The heading correction is the median direction difference over associated walls, applied only
     if enough walls agree and it is small (gates below).
  4. Before any wall is known, a Manhattan fallback (walls at 0/90 deg) can be used instead.
  5. The spin's sightings, now corrected, update or add walls.

Works from the raw EKF (odom) readings and ignores corrections applied during the run, so it is
judged on its own: truth heading error = (EKF heading + our correction) - true heading.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam_replay as R  # noqa: E402
import rcd_check as C    # noqa: E402

# Association: a sighting belongs to a stored wall if its direction and position both agree
ASSOC_ANGLE = math.radians(4.0)    # direction difference
ASSOC_LINE = 0.25                  # m, distance from the stored wall's line
ASSOC_REACH = 0.8                  # m, from the nearest point already seen on that wall
# Several sensors in one spin often see the SAME feature (e.g. one box corner): that is one piece of
# evidence, not several. Sightings this close in position and direction are grouped into one feature.
FEATURE_RADIUS = 0.3               # m
FEATURE_ANGLE = math.radians(4.0)
# Heading correction is a 1-D Kalman filter: how sure we are of the heading vs how sure the walls are
MIN_WALLS = 1                      # distinct known walls needed for a wall measurement
MIN_AGREE = 2                      # distinct features needed for a Manhattan measurement
# One distinct feature's direction error: faces are good to ~0.7 deg but corners and oblique views
# mixed in are off by several degrees, so one feature alone is a weak measurement
FEATURE_SIGMA = math.radians(2.0)
REJECT_INFLATE = math.radians(2.0) # after a rejected measurement, admit more uncertainty (no lock-out)
DRIFT_PER_SPIN = math.radians(1.2) # heading uncertainty added each spin (EKF drifts ~1.1 deg/spin)
SIGHTING_SIGMA = math.radians(1.0) # one RCD's bearing noise
WALLS_FLOOR = math.radians(0.75)   # best case for a wall-map measurement
MANHATTAN_FLOOR = math.radians(1.5)  # Manhattan: off-axis walls (rotated boxes) can bias it
GATE_SIGMAS = 3.0                  # reject measurements further than this from the expected heading


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Wall:
    """A wall face: the direction looking at it (sensor -> wall) and the surface points seen on it."""

    def __init__(self, bearing, point):
        self.sum = np.array([math.cos(bearing), math.sin(bearing)])
        self.points = [point]

    @property
    def bearing(self):
        return math.atan2(self.sum[1], self.sum[0])

    def line_distance(self, point):
        n = np.array([math.cos(self.bearing), math.sin(self.bearing)])
        return abs(float(np.dot(point - self.points[0], n)))

    def reach(self, point):
        return min(np.hypot(*(point - p)) for p in self.points)

    def add(self, bearing, point):
        self.sum += [math.cos(bearing), math.sin(bearing)]
        self.points.append(point)


class Corrector:
    def __init__(self):
        self.walls = []
        self.pose = np.zeros(3)   # map <- odom transform (x, y, yaw), heading-only updates
        self.var = math.radians(0.5) ** 2   # heading uncertainty; the start heading is known

    def to_map(self, xy):
        return R.rotate(xy[None, :], np.zeros(2), self.pose[2])[0] + self.pose[:2]

    def sightings(self, rcds):
        """RCDs (odom frame) -> (bearing, surface point) in the map frame, under the current correction."""
        out = []
        for bearing, rng, _, pos in rcds:
            b = bearing + self.pose[2]
            sensor = self.to_map(pos)
            out.append((b, sensor + rng * np.array([math.cos(b), math.sin(b)])))
        return out

    @staticmethod
    def features(seen):
        """Group sightings of the same thing: (mean bearing, mean point, count)."""
        groups = []
        for b, p in seen:
            for g in groups:
                gb = math.atan2(g['s'], g['c'])
                if abs(wrap(gb - b)) < FEATURE_ANGLE and np.hypot(*(g['p'] / g['n'] - p)) < FEATURE_RADIUS:
                    g['c'] += math.cos(b); g['s'] += math.sin(b); g['p'] = g['p'] + p; g['n'] += 1
                    break
            else:
                groups.append({'c': math.cos(b), 's': math.sin(b), 'p': np.array(p, float), 'n': 1})
        return [(math.atan2(g['s'], g['c']), g['p'] / g['n'], g['n']) for g in groups]

    def associate(self, bearing, point):
        best = None
        for w in self.walls:
            d_ang = abs(wrap(w.bearing - bearing))
            if d_ang < ASSOC_ANGLE and w.line_distance(point) < ASSOC_LINE and w.reach(point) < ASSOC_REACH:
                if best is None or d_ang < best[0]:
                    best = (d_ang, w)
        return best[1] if best else None

    def rotate_about(self, pivot, dyaw):
        """Heading-only correction: rotate the map<-odom transform about the robot (keeps its position)."""
        o = self.pose[:2] - pivot
        c, s = math.cos(dyaw), math.sin(dyaw)
        self.pose[:2] = pivot + [c * o[0] - s * o[1], s * o[0] + c * o[1]]
        self.pose[2] = wrap(self.pose[2] + dyaw)

    def step(self, rcds, pivot_odom):
        """One spin. Returns (correction applied, source, n distinct walls/features, reason)."""
        self.var += DRIFT_PER_SPIN ** 2
        feats = self.features(self.sightings(rcds))

        # Wall measurement: one direction difference per distinct known wall
        per_wall = {}
        for b, p, _ in feats:
            w = self.associate(b, p)
            if w is not None:
                per_wall.setdefault(id(w), []).append(wrap(w.bearing - b))
        diffs = [float(np.median(v)) for v in per_wall.values()]

        z = None
        if len(diffs) >= MIN_WALLS:
            source = 'walls'
            d = np.array(diffs)
            z = float(np.median(d))
            spread = max(float(np.median(np.abs(d - z))), FEATURE_SIGMA)
            r = spread ** 2 / len(d) + WALLS_FLOOR ** 2
            support = len(d)
        else:
            source = 'manhattan'
            est, inliers, spread_deg = C.manhattan_heading([b for b, _, _ in feats])
            support = inliers
            if est is not None and inliers >= MIN_AGREE:
                z = -est
                spread = max(math.radians(spread_deg), FEATURE_SIGMA)
                r = spread ** 2 / inliers + MANHATTAN_FLOOR ** 2
        if z is None:
            self.add_walls(rcds)
            return None, source, len(diffs), '%d known walls, %d features: too few' % (len(diffs), len(feats))

        sigma = math.sqrt(self.var + r)
        if abs(z) > GATE_SIGMAS * sigma:
            self.var += REJECT_INFLATE ** 2   # if we are wrong about being right, let evidence back in
            self.add_walls(rcds)
            return None, source, support, '%+.1f° outside 3σ=%.1f°' % (math.degrees(z), math.degrees(3 * sigma))

        gain = self.var / (self.var + r)
        delta = gain * z
        self.var *= (1 - gain)
        self.rotate_about(self.to_map(pivot_odom), delta)
        self.add_walls(rcds)
        return delta, source, support, ''

    def add_walls(self, rcds):
        """Map this spin's (corrected) features: refine known walls, add new ones."""
        for b, p, _ in self.features(self.sightings(rcds)):
            w = self.associate(b, p)
            if w:
                w.add(b, p)
            else:
                self.walls.append(Wall(b, p))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dirs', nargs='*')
    args = ap.parse_args()
    runs = args.run_dirs or sorted(glob.glob(os.path.join(R.RECORD_ROOT, '*/')))

    before_all, after_all, per_run = [], [], []
    for run in runs:
        spins = R.load_run(run)
        if not spins:
            continue
        print('\n%s' % os.path.basename(run.rstrip('/')))
        print('%-9s | %5s %5s | %9s %9s | %-26s | %s' % (
            'spin', 'RCDs', 'assoc', 'EKF err', 'ours err', 'correction', 'walls'))
        print('-' * 92)
        corr = Corrector()
        run_before, run_after = [], []
        for spin in spins:
            rays = spin['rays']
            cone = 2 * (float(np.median(rays[:, 5])) if rays.shape[1] > 5 else 0.22)
            rcds = [r for seq in C.sensor_sequences(rays, [0.0, 0.0, 0.0]) for r in C.find_rcds(seq, cone)]
            pivot = rays[:, 0:2].mean(axis=0)

            delta, source, n_assoc, reason = corr.step(rcds, pivot)

            ekf_err = ours = None
            if spin['truth']:
                # Heading error of the raw EKF (odom) frame, i.e. undoing what the run itself applied
                ekf_err = wrap(spin['truth']['estimate'][2] - spin['truth']['truth'][2] - spin['map_odom'][2])
                ours = wrap(ekf_err + corr.pose[2])
                before_all.append(abs(ekf_err))
                after_all.append(abs(ours))
                run_before.append(abs(ekf_err))
                run_after.append(abs(ours))
            fmt = lambda a: '%+8.2f°' % math.degrees(a) if a is not None else '        ?'
            what = ('%+.2f° (%s)' % (math.degrees(delta), source)) if delta is not None else ('none: ' + reason)
            print('%-9s | %5d %5d | %9s %9s | %-26s | %d' % (
                spin['name'], len(rcds), n_assoc, fmt(ekf_err), fmt(ours), what[:26], len(corr.walls)))
        if run_after:
            per_run.append((os.path.basename(run.rstrip('/')), len(run_after),
                            np.degrees(np.mean(run_before)), np.degrees(np.max(run_before)),
                            np.degrees(np.mean(run_after)), np.degrees(np.max(run_after))))

    if per_run:
        print('\n%-22s %5s | %-21s | %-21s' % ('run', 'spins', 'EKF only: mean / max', 'with walls: mean / max'))
        for name, n, bm, bx, am, ax in per_run:
            print('%-22s %5d | %8.2f° / %7.2f° | %8.2f° / %7.2f°' % (name, n, bm, bx, am, ax))
    if after_all:
        b, a = np.degrees(before_all), np.degrees(after_all)
        print('\nHeading error at spin end over %d spins:' % len(a))
        print('  EKF only:        mean |err| %.2f°, max %.2f°' % (b.mean(), b.max()))
        print('  with walls:      mean |err| %.2f°, max %.2f°' % (a.mean(), a.max()))


if __name__ == '__main__':
    main()
