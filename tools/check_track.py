#!/usr/bin/env python3
"""
Record /ft/track (and count /ft/detection) for a few seconds and print a
short health report for the single-target tracker.

Usage:
    python3 tools/check_track.py --duration 25
    python3 tools/check_track.py --duration 25 --csv /tmp/track.csv

Run this in a separate terminal while the tracker and a bag are running.
The tracker itself has no lead/prediction marker yet; this checks continuity,
tracking semantics, lost runs, ID switches, NaN and velocity sanity.
"""

import argparse
import csv
import math
import sys
import time

import rospy
from ft_detection.msg import DetectionFrame
from ft_tracker.msg import Track


def percentile(values, q):
    if not values:
        return 0.0
    values = sorted(values)
    k = max(0, min(len(values) - 1, int(round((len(values) - 1) * q))))
    return values[k]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=0.0,
                        help='record seconds; 0 means until Ctrl-C')
    parser.add_argument('--csv', default='', help='optional CSV output path')
    parser.add_argument('--track-topic', default='/ft/track')
    parser.add_argument('--detection-topic', default='/ft/detection')
    args, _ = parser.parse_known_args()

    tracks = []
    det_frames = [0]
    start_wall = time.time()

    def track_cb(msg):
        tracks.append({
            't': msg.header.stamp.to_sec(),
            'id': int(msg.id),
            'tracking': bool(msg.tracking),
            'p': (msg.position.x, msg.position.y, msg.position.z),
            'v': (msg.velocity.x, msg.velocity.y, msg.velocity.z),
        })

    def detection_cb(_msg):
        det_frames[0] += 1

    rospy.init_node('check_track', anonymous=True)
    rospy.Subscriber(args.track_topic, Track, track_cb, queue_size=200)
    rospy.Subscriber(args.detection_topic, DetectionFrame,
                     detection_cb, queue_size=200)

    print('recording... Ctrl-C to finish')
    try:
        while not rospy.is_shutdown():
            if args.duration > 0 and time.time() - start_wall >= args.duration:
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    if not tracks:
        print('no /ft/track messages received')
        return 1

    tracks.sort(key=lambda r: r['t'])
    n = len(tracks)
    n_true = sum(1 for r in tracks if r['tracking'])
    n_false = n - n_true
    ids = sorted(set(r['id'] for r in tracks))

    # Continuous pure-prediction runs.
    lost_runs = []
    start = None
    for r in tracks:
        if not r['tracking'] and start is None:
            start = r
        if r['tracking'] and start is not None:
            lost_runs.append(start['t'])
            lost_runs[-1] = (start['t'], r['t'])
            start = None
    if start is not None:
        lost_runs.append((start['t'], tracks[-1]['t']))

    durations = [b - a for a, b in lost_runs]
    max_lost = max(durations) if durations else 0.0

    # ID changes and the amount of pure prediction immediately before them.
    switches = []
    for i in range(1, n):
        if tracks[i]['id'] != tracks[i - 1]['id']:
            switches.append((tracks[i - 1], tracks[i]))

    # Position/velocity sanity.
    nan_count = 0
    speeds = []
    steps = []
    for i, r in enumerate(tracks):
        vals = list(r['p']) + list(r['v'])
        if any(not math.isfinite(x) for x in vals):
            nan_count += 1
        speeds.append(math.sqrt(sum(x * x for x in r['v'])))
        if i > 0 and tracks[i - 1]['id'] == r['id']:
            dt = r['t'] - tracks[i - 1]['t']
            if dt > 1e-6:
                dp = math.sqrt(sum((a - b) ** 2 for a, b in zip(r['p'], tracks[i - 1]['p'])))
                steps.append(dp / dt)

    print('\n===== /ft/track health =====')
    print('track messages       : %d' % n)
    print('detection frames     : %d' % det_frames[0])
    print('tracking true/false  : %d / %d (%.1f%% true)' %
          (n_true, n_false, 100.0 * n_true / n))
    print('track ids            : %s' % (ids,))
    print('id switches          : %d' % len(switches))
    for a, b in switches[:8]:
        print('  %.3f id %d -> %d' % (b['t'], a['id'], b['id']))
    print('pure-prediction runs : %d' % len(lost_runs))
    print('max lost duration    : %.3f s' % max_lost)
    print('position step speed  : p50=%.2f p95=%.2f max=%.2f m/s' %
          (percentile(steps, 0.50), percentile(steps, 0.95), max(steps or [0.0])))
    print('filter velocity      : p50=%.2f p95=%.2f max=%.2f m/s' %
          (percentile(speeds, 0.50), percentile(speeds, 0.95), max(speeds or [0.0])))
    print('non-finite samples   : %d' % nan_count)

    print('\ncheck points:')
    print('- true ratio should be high (>~90%%) while the bag has detections')
    print('- max lost duration should be <= max_lost_time (0.5 s)')
    print('- id switches should happen only after a long lost run, not every frame')
    print('- non-finite samples must be 0')
    print('- step/filter speed should be physically plausible for your target')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['stamp', 'id', 'tracking', 'px', 'py', 'pz', 'vx', 'vy', 'vz'])
            for r in tracks:
                w.writerow([r['t'], r['id'], int(r['tracking']),
                            r['p'][0], r['p'][1], r['p'][2],
                            r['v'][0], r['v'][1], r['v'][2]])
        print('csv written: %s' % args.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
