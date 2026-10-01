#!/usr/bin/env python3
"""Replay recorded spin scans offline and compare the matcher's score with ground truth.

Usage:
    python3 slam_replay.py [run_dir] [--sigma S] [--occ L] [--rot DEG] [--trans M] [--no-plots]

gazebo_start records every run to ~/mecanum_ws/slam_records/<timestamp>/ (latest is the default).
For each spin it prints a row and writes plots/spin_NNN.png:

  truth     the correction that moves the scan onto the TRUE world (from pose_error)
  best      the highest-scoring correction in a wide search (same score as sparse_slam)
  matcher   what sparse_slam decided during the run
  fit@...   the score (mean likelihood-field value per hit, 0-1) at each of those

If fit@truth is clearly below fit@best, the map + sensor model prefer a wrong answer, and no
amount of threshold tuning in the matcher can fix that.

Assumes the robot spawned at the world origin facing +x, so the map frame starts equal to the
world frame (true for gazebo.launch.py).
"""
import argparse
import glob
import json
import math
import os
import warnings
import xml.etree.ElementTree as ET

import numpy as np
warnings.filterwarnings('ignore', message='A NumPy version')   # Ubuntu's scipy 1.8 vs numpy 1.26: works fine
from scipy import ndimage  # noqa: E402

RECORD_ROOT = os.path.expanduser('~/mecanum_ws/slam_records')
DEFAULT_WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds', 'obstacles.world')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate(points, pivot, angle):
    c, s = math.cos(angle), math.sin(angle)
    d = points - pivot
    return pivot + np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)


def odom_to_map(xy, map_odom):
    return rotate(xy, np.zeros(2), map_odom[2]) + np.array(map_odom[:2])


def load_world_shapes(path):
    """Obstacle footprints: ('box', x, y, yaw, sx, sy) or ('circle', x, y, r). Final poses come from <state>."""
    world = ET.parse(path).getroot().find('world')
    state_pose, state_scale = {}, {}
    state = world.find('state')
    if state is not None:
        for m in state.findall('model'):
            if m.find('pose') is not None:
                state_pose[m.get('name')] = [float(v) for v in m.find('pose').text.split()]
            if m.find('scale') is not None:
                state_scale[m.get('name')] = [float(v) for v in m.find('scale').text.split()]
    shapes = []
    for m in world.findall('model'):
        name = m.get('name')
        geom = m.find('./link/collision/geometry')
        if name == 'ground_plane' or geom is None:
            continue
        pose = state_pose.get(name) or [float(v) for v in m.find('pose').text.split()]
        scale = state_scale.get(name, [1.0, 1.0, 1.0])
        x, y, yaw = pose[0], pose[1], pose[5]
        if geom.find('box') is not None:
            sx, sy = [float(v) for v in geom.find('box/size').text.split()][:2]
            shapes.append(('box', x, y, yaw, sx * scale[0], sy * scale[1]))
        elif geom.find('cylinder') is not None:
            shapes.append(('circle', x, y, float(geom.find('cylinder/radius').text) * scale[0]))
        elif geom.find('sphere') is not None:
            shapes.append(('circle', x, y, float(geom.find('sphere/radius').text) * scale[0]))
    return shapes


def load_run(run_dir):
    spins = []
    for meta_path in sorted(glob.glob(os.path.join(run_dir, 'spin_*.json'))):
        prefix = meta_path[:-len('.json')]
        with open(meta_path) as f:
            spin = json.load(f)
        spin['name'] = os.path.basename(prefix)
        spin['rays'] = np.load(prefix + '_rays.npy')
        spin['map'] = np.load(prefix + '_map.npy')
        spins.append(spin)

    truths = []
    truth_path = os.path.join(run_dir, 'truth.jsonl')
    if os.path.exists(truth_path):
        with open(truth_path) as f:
            truths = [json.loads(line) for line in f if line.strip()]
    # Pair each spin with the latest unused truth written no later than it. Both are written at
    # each spin end, in order; older recordings stamped the spin after matching (up to ~20 s late).
    used = set()
    for spin in spins:
        candidates = [i for i, t in enumerate(truths)
                      if i not in used and t['sim_time'] <= spin['sim_time'] + 0.5]
        spin['truth'] = None
        if candidates:
            i = max(candidates, key=lambda i: truths[i]['sim_time'])
            if spin['sim_time'] - truths[i]['sim_time'] < 30.0:
                spin['truth'] = truths[i]
                used.add(i)
    return spins


def likelihood_field(log_odds, res, sigma, occ_threshold):
    """Same field as sparse_slam: 1.0 on wall cells, Gaussian falloff, cut off past 3 sigma."""
    wall = log_odds > occ_threshold
    if not wall.any():
        return np.zeros_like(log_odds, dtype=float)
    dist = ndimage.distance_transform_edt(~wall) * res
    reach = ndimage.distance_transform_cdt(~wall, metric='chessboard')
    field = np.exp(-dist ** 2 / (2 * sigma ** 2))
    field[reach > math.ceil(3 * sigma / res)] = 0.0
    return field


def bilinear(field, fx, fy):
    """Field blended between the 4 nearest cell centres; fx/fy in cells from the centre of cell 0."""
    h, w = field.shape
    x0 = np.floor(fx).astype(int)
    y0 = np.floor(fy).astype(int)
    wx, wy = fx - x0, fy - y0

    def at(x, y):
        ok = (x >= 0) & (x < w) & (y >= 0) & (y < h)
        return np.where(ok, field[np.clip(y, 0, h - 1), np.clip(x, 0, w - 1)], 0.0)

    return ((1 - wx) * (1 - wy) * at(x0, y0) + wx * (1 - wy) * at(x0 + 1, y0) +
            (1 - wx) * wy * at(x0, y0 + 1) + wx * wy * at(x0 + 1, y0 + 1))


class Scorer:
    """Same score as sparse_slam: each reading = best field value on its arc across the cone, minus a
    penalty for walls inside the cone short of the reading (free space). Mean over readings."""

    def __init__(self, sensors, ranges, headings, half_fovs, pivot, field, crop_origin, res,
                 arc_samples=7, free_weight=1.0, free_margin=0.2):
        self.pivot, self.field, self.origin, self.res = pivot, field, np.array(crop_origin), res
        self.free_weight = free_weight
        k = max(1, arc_samples)
        u = np.linspace(-1, 1, k) if k > 1 else np.zeros(1)
        ang = headings[:, None] + half_fovs[:, None] * u[None, :]                      # (N, K)
        self.arcs = sensors[:, None, :] + ranges[:, None, None] * np.stack([np.cos(ang), np.sin(ang)], 2)
        inner_r = np.maximum(ranges - free_margin, 0)[:, None] * np.array([0.25, 0.5, 0.75])[None, :]
        self.inner = (sensors[:, None, None, :] + inner_r[:, :, None, None] *
                      np.stack([np.cos(ang), np.sin(ang)], 2)[:, None, :, :]).reshape(len(ranges), -1, 2)
        self.inner_ok = ranges > free_margin + 0.05
        self.points = sensors + ranges[:, None] * np.stack([np.cos(headings), np.sin(headings)], 1)

    def _values(self, pts, dyaw, dx, dy):
        """Field at pts (N, M, 2) after the correction, shape (N, M)."""
        p = rotate(pts.reshape(-1, 2), self.pivot, dyaw) + [dx, dy]
        f = (p - self.origin) / self.res - 0.5
        return bilinear(self.field, f[:, 0], f[:, 1]).reshape(pts.shape[:2])

    def arc_fit(self, dyaw, dx, dy):
        return float(self._values(self.arcs, dyaw, dx, dy).max(axis=1).mean())

    def violation(self, dyaw, dx, dy):
        return float((self._values(self.inner, dyaw, dx, dy).max(axis=1) * self.inner_ok).mean())

    def fit(self, dyaw, dx, dy):
        return self.arc_fit(dyaw, dx, dy) - self.free_weight * self.violation(dyaw, dx, dy)

    def search(self, max_rot, rot_step, max_trans, band=0.05):
        """Fits on sparse_slam's grid. As there, the free-space penalty is only computed for candidates
        within `band` of the best arc fit; the rest can't win and are left as NaN."""
        n_rot = int(round(max_rot / rot_step))
        n_t = int(round(max_trans / self.res))
        angles = np.arange(-n_rot, n_rot + 1) * rot_step
        steps = np.arange(-n_t, n_t + 1)
        n, k = self.arcs.shape[:2]
        fits = np.zeros((len(angles), len(steps), len(steps)))
        for i, a in enumerate(angles):
            f = (rotate(self.arcs.reshape(-1, 2), self.pivot, a) - self.origin) / self.res - 0.5
            # Arrays of shape (ty, tx, point): every shift applied to every arc point at once
            fx, fy = np.broadcast_arrays(f[None, None, :, 0] + steps[None, :, None],
                                         f[None, None, :, 1] + steps[:, None, None])
            vals = bilinear(self.field, fx, fy).reshape(len(steps), len(steps), n, k)
            fits[i] = vals.max(axis=3).mean(axis=2)
        best_arc = fits.max()
        out = np.full_like(fits, np.nan)
        for i, iy, ix in zip(*np.nonzero(fits >= best_arc - band)):
            a, dx, dy = angles[i], steps[ix] * self.res, steps[iy] * self.res
            out[i, iy, ix] = fits[i, iy, ix] - self.free_weight * self.violation(a, dx, dy)
        return angles, steps * self.res, out


def truth_correction(spin, pivot):
    """Rotation about the pivot + shift that takes the estimated pose onto the true pose."""
    if not spin['truth']:
        return None
    tx, ty, tyaw = spin['truth']['truth']
    ex, ey, eyaw = spin['truth']['estimate']
    dyaw = wrap(tyaw - eyaw)
    moved = rotate(np.array([[ex, ey]]), pivot, dyaw)[0]
    return dyaw, tx - moved[0], ty - moved[1]


def plot_spin(path, spin, scorer, shapes, truth, angles, shifts, fits, best, matcher):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    res = spin['resolution']
    log_odds = spin['map']
    h, w = log_odds.shape
    ox, oy = spin['crop_origin']
    extent = [ox, ox + w * res, oy, oy + h * res]
    prob = 1.0 / (1.0 + np.exp(-log_odds))
    prob[np.abs(log_odds) < 0.01] = 0.5

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    ax = axes[0]
    ax.imshow(prob, origin='lower', extent=extent, cmap='gray_r', vmin=0, vmax=1)
    for shape in shapes:
        if shape[0] == 'box':
            _, x, y, yaw, sx, sy = shape
            corners = np.array([[-sx, -sy], [sx, -sy], [sx, sy], [-sx, sy]]) / 2
            ax.add_patch(Polygon(rotate(corners, np.zeros(2), yaw) + [x, y], fill=False, ec='lime', lw=1.5))
        else:
            ax.add_patch(Circle(shape[1:3], shape[3], fill=False, ec='lime', lw=1.5))
    pts = scorer.points
    ax.scatter(pts[:, 0], pts[:, 1], s=2, c='red', label='scan, uncorrected')
    if truth:
        p = rotate(pts, scorer.pivot, truth[0]) + truth[1:]
        ax.scatter(p[:, 0], p[:, 1], s=2, c='deepskyblue', label='scan at TRUE pose')
    ax.plot(*scorer.pivot, 'k+', ms=12, mew=2)
    ax.set_xlim(extent[:2])
    ax.set_ylim(extent[2:])
    ax.set_aspect('equal')
    ax.set_title('%s: map (gray), real obstacles (green)' % spin['name'])
    ax.legend(loc='upper right', fontsize=8, markerscale=4)

    ax = axes[1]
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # angles with no near-best candidates are all NaN
        ax.plot(np.degrees(angles), np.nanmax(fits, axis=(1, 2)), 'k.-')
    ax.axvline(0, color='gray', ls=':', label='no correction')
    if truth:
        ax.axvline(math.degrees(truth[0]), color='deepskyblue', lw=2, label='truth')
    ax.axvline(math.degrees(best[0]), color='red', ls='--', label='best score')
    if matcher:
        ax.axvline(math.degrees(matcher[0]), color='blue', ls='-.', label='matcher (run)')
    ax.set_xlabel('rotation correction (deg)')
    ax.set_ylabel('best fit over shifts')
    ax.set_title('Score vs rotation (near-best candidates only)')
    ax.legend(fontsize=8)

    ax = axes[2]
    a_idx = int(np.argmin(np.abs(angles - (truth[0] if truth else best[0]))))
    r = shifts[-1] + res / 2
    im = ax.imshow(fits[a_idx], origin='lower', extent=[-r, r, -r, r], cmap='viridis')
    fig.colorbar(im, ax=ax, label='fit')
    ax.plot(0, 0, 'w+', ms=12, mew=2, label='no shift')
    if truth:
        ax.plot(truth[1], truth[2], 'x', color='deepskyblue', ms=12, mew=3, label='truth')
    ax.plot(best[1], best[2], 'rx', ms=10, mew=2, label='best score')
    ax.set_xlabel('dx (m)')
    ax.set_ylabel('dy (m)')
    ax.set_title('Score vs shift at %+.1f deg' % math.degrees(angles[a_idx]))
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir', nargs='?', help='recording folder (default: latest)')
    ap.add_argument('--sigma', type=float, help='likelihood field sigma, m (default: as recorded)')
    ap.add_argument('--occ', type=float, help='log-odds counted as wall (default: as recorded)')
    ap.add_argument('--rot', type=float, default=8.0, help='search +-deg (default 8)')
    ap.add_argument('--trans', type=float, default=0.3, help='search +-m (default 0.3)')
    ap.add_argument('--free-weight', type=float, default=1.0, help='free-space penalty weight (default 1.0)')
    ap.add_argument('--free-margin', type=float, default=0.2, help='free-space look-short distance, m (default 0.2)')
    ap.add_argument('--world', default=DEFAULT_WORLD, help="world file for true obstacles ('none' to skip)")
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()

    run_dir = args.run_dir or max(glob.glob(os.path.join(RECORD_ROOT, '*/')), key=os.path.getmtime, default=None)
    if not run_dir:
        raise SystemExit('No recordings in %s' % RECORD_ROOT)
    spins = load_run(run_dir)
    if not spins:
        raise SystemExit('No spin_*.json in %s' % run_dir)
    shapes = load_world_shapes(args.world) if args.world != 'none' and os.path.exists(args.world) else []
    plot_dir = os.path.join(run_dir, 'plots')
    if not args.no_plots:
        os.makedirs(plot_dir, exist_ok=True)

    print('Run: %s  (%d spins, truth for %d)' % (run_dir, len(spins), sum(1 for s in spins if s['truth'])))
    print('Corrections as (deg, dx m, dy m). rank = share of candidates scoring above truth (0% = truth is the peak)\n')
    print('%-9s | %-22s | %-22s | %-30s | %6s %6s %6s %6s' % (
        'spin', 'truth', 'best score', 'matcher (run)', 'fit@0', '@truth', '@best', 'rank'))
    print('-' * 120)

    fmt = lambda c: '%+5.1f° %+5.2f %+5.2f' % (math.degrees(c[0]), c[1], c[2])
    for spin in spins:
        p = spin['params']
        sigma = args.sigma or p['sigma']
        occ = args.occ if args.occ is not None else p['occ_threshold']
        res = spin['resolution']

        rays = spin['rays']
        hit = rays[:, 4] > 0.5
        if not hit.any():
            print('%-9s | no hits' % spin['name'])
            continue
        all_sensors = odom_to_map(rays[:, 0:2], spin['map_odom'])
        pivot = all_sensors.mean(axis=0)
        s_map, e_map = all_sensors[hit], odom_to_map(rays[hit, 2:4], spin['map_odom'])
        v = e_map - s_map
        half_fov = rays[hit, 5] if rays.shape[1] > 5 else np.full(hit.sum(), 0.22)
        field = likelihood_field(spin['map'], res, sigma, occ)
        scorer = Scorer(s_map, np.hypot(v[:, 0], v[:, 1]), np.arctan2(v[:, 1], v[:, 0]), half_fov, pivot,
                        field, spin['crop_origin'], res, arc_samples=p.get('arc_samples', 7),
                        free_weight=args.free_weight, free_margin=args.free_margin)
        if not field.any():
            print('%-9s | no mapped walls in view' % spin['name'])
            continue

        angles, shifts, fits = scorer.search(math.radians(args.rot), p['rot_step'], args.trans)
        ia, iy, ix = np.unravel_index(np.nanargmax(fits), fits.shape)
        best = (angles[ia], shifts[ix], shifts[iy])
        truth = truth_correction(spin, pivot)

        m = spin['match']
        matcher = (m['dyaw'], m['dx'], m['dy']) if m['attempted'] else None
        verdict = ('applied' if m['applied'] else 'ACCEPT' if m['accepted'] else 'reject') if m['attempted'] \
            else 'skipped'

        fit0 = scorer.fit(0, 0, 0)
        fit_truth = scorer.fit(*truth) if truth else float('nan')
        fit_best = np.nanmax(fits)
        rank = (np.nan_to_num(fits, nan=-1.0) > fit_truth).sum() / fits.size * 100 if truth else float('nan')
        print('%-9s | %-22s | %-22s | %-30s | %6.3f %6.3f %6.3f %5.1f%%' % (
            spin['name'], fmt(truth) if truth else '?', fmt(best),
            (fmt(matcher) + ' ' + verdict) if matcher else verdict,
            fit0, fit_truth, fit_best, rank))

        if not args.no_plots:
            plot_spin(os.path.join(plot_dir, spin['name'] + '.png'), spin, scorer, shapes,
                      truth, angles, shifts, fits, best, matcher)

    if not args.no_plots:
        print('\nPlots: %s' % plot_dir)


if __name__ == '__main__':
    main()
