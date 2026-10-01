#!/usr/bin/env python3
"""Summarize one sim run's scan matching against ground truth, from the ROS logs.

Usage:
    python3 slam_report.py              # latest run in ~/.ros/log
    python3 slam_report.py <log file>   # the run that a given sparse_slam_*.log belongs to

Pairs each spin's 'Scan match' line (sparse_slam) with its 'At spin end' line
(pose_error). Heading/position error there is measured at the END of the spin,
BEFORE that spin's correction is applied, so with working SLAM it should stay
small instead of growing spin after spin.
"""
import glob
import os
import re
import sys

LOG_DIR = os.path.expanduser('~/.ros/log')
RUN_WINDOW_MS = 120_000   # node logs of one launch start within this of each other

STAMP = re.compile(r'\[(\d+\.\d+)\]')
MATCH = re.compile(
    r'Scan match(?P<dry> \(dry run\))?: (?:corrected|would correct) '
    r'dx=(?P<dx>\S+) dy=(?P<dy>\S+) dyaw=(?P<dyaw>\S+) deg \| '
    r'score (?P<score>\S+) \(uncorrected (?P<zero>\S+)\)(?:, margin (?P<margin>\S+))?'
    r'(?:, peak \+-(?P<peak_rot>\S+) deg/(?P<peak_trans>\S+) m)?, '
    r'(?P<frac>\d+)% of (?P<hits>\d+) hits on walls, (?P<ms>\d+) ms'
    r'(?:, window \+-(?P<wrot>\S+) deg/(?P<wtrans>\S+) m)? \| '
    r'(?P<verdict>ACCEPT|REJECT: (?P<reason>.*))')
SKIPPED = re.compile(r'Scan match skipped: (?P<reason>.*)')
SPIN_END = re.compile(r'At spin end: error x=(?P<x>\S+) y=(?P<y>\S+) heading=(?P<h>\S+) deg')
SUMMARY = re.compile(r'Run summary.*')


def run_logs(anchor):
    """All node logs started within RUN_WINDOW_MS of the anchor sparse_slam log."""
    def started_ms(path):
        m = re.search(r'_(\d{13})\.log$', path)
        return int(m.group(1)) if m else None
    t0 = started_ms(anchor)
    return [p for p in glob.glob(os.path.join(LOG_DIR, '*.log'))
            if started_ms(p) is not None and abs(started_ms(p) - t0) < RUN_WINDOW_MS]


def main():
    if len(sys.argv) > 1:
        anchor = sys.argv[1]
    else:
        slam_logs = sorted(glob.glob(os.path.join(LOG_DIR, 'sparse_slam_*.log')), key=os.path.getmtime)
        if not slam_logs:
            sys.exit('No sparse_slam logs in %s' % LOG_DIR)
        anchor = slam_logs[-1]

    events, summary = [], None
    for path in run_logs(anchor):
        with open(path, errors='replace') as f:
            for line in f:
                stamp = STAMP.search(line)
                if not stamp:
                    continue
                t = float(stamp.group(1))
                for kind, pattern in (('match', MATCH), ('skip', SKIPPED), ('truth', SPIN_END)):
                    m = pattern.search(line)
                    if m:
                        events.append((t, kind, m))
                        break
                if SUMMARY.search(line):
                    summary = SUMMARY.search(line).group(0)
    events.sort(key=lambda e: e[0])
    if not events:
        sys.exit('No scan-match or spin-end lines found for run of %s' % os.path.basename(anchor))

    # Each spin end produces one truth line and one match/skip line within a second or so
    spins, pending = [], {}
    for t, kind, m in events:
        key = 'truth' if kind == 'truth' else 'result'
        if key in pending and abs(t - pending['t']) > 2.0:
            spins.append(pending)
            pending = {}
        pending.setdefault('t', t)
        pending[key] = (kind, m)
        if 'truth' in pending and 'result' in pending:
            spins.append(pending)
            pending = {}
    if pending:
        spins.append(pending)

    t_start = spins[0]['t']
    dry = any(s.get('result', ('', None))[0] == 'match' and s['result'][1].group('dry') for s in spins)
    print('Run: %s%s' % (os.path.basename(anchor), '  (DRY RUN: corrections not applied)' if dry else ''))
    print('Error columns = truth at spin end, before that spin\'s correction.\n')
    print('%3s %6s | %8s %7s | %8s %9s %15s %6s %5s | %s' % (
        '#', 't(s)', 'hdg err', 'pos err', 'ideal', 'matched', 'shift (m)', 'peak', 'win', 'result'))
    print('-' * 106)

    accepted = rejected = skipped = 0
    streak = longest_streak = 0
    reasons = {}
    for i, s in enumerate(spins, 1):
        truth = s.get('truth', (None, None))[1]
        hdg = '%+7.2f°' % float(truth.group('h')) if truth else '      ?'
        pos = '%6.3f' % (float(truth.group('x')) ** 2 + float(truth.group('y')) ** 2) ** 0.5 if truth else '     ?'
        ideal = '%+7.2f°' % -float(truth.group('h')) if truth else '      ?'

        kind, m = s.get('result', (None, None))
        if kind == 'match':
            shift = '(%+.2f, %+.2f)' % (float(m.group('dx')), float(m.group('dy')))
            margin = ('±%s°' % m.group('peak_rot')) if m.group('peak_rot') else (m.group('margin') or '-')
            win = '±%.0f°' % float(m.group('wrot')) if m.group('wrot') else ''
            matched = '%+7.2f°' % float(m.group('dyaw'))
            if m.group('verdict') == 'ACCEPT':
                accepted += 1
                streak = 0
                result = 'ACCEPT'
            else:
                rejected += 1
                streak += 1
                reasons[m.group('reason')] = reasons.get(m.group('reason'), 0) + 1
                result = 'reject: ' + m.group('reason')
        elif kind == 'skip':
            skipped += 1
            shift, margin, matched, win, result = '', '', '', '', 'skipped: ' + m.group('reason')
        else:
            shift, margin, matched, win, result = '', '', '', '', '(no match line)'
        longest_streak = max(longest_streak, streak)

        print('%3d %6.0f | %8s %7s | %8s %9s %15s %6s %5s | %s' % (
            i, s['t'] - t_start, hdg, pos, ideal, matched, shift, margin, win, result))

    print('-' * 106)
    attempted = accepted + rejected
    print('Spins: %d   matched: %d   accepted: %d   rejected: %d   skipped: %d' % (
        len(spins), attempted, accepted, rejected, skipped))
    if attempted:
        print('Accept rate: %.0f%%   longest run of rejections: %d' % (100.0 * accepted / attempted, longest_streak))
    for reason, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print('  %2d x %s' % (count, reason))
    if summary:
        print('\n' + summary)


if __name__ == '__main__':
    main()
