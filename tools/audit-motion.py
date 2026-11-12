#!/usr/bin/env python3
# SPDX-License-Identifier: BSL-1.0
"""Bounded, read-only production-v2 tap. Run as the session user, never open sensors.
Records: little-endian receiver CLOCK_MONOTONIC u64 + unchanged 240-byte packet.
This observer's receive age is not driver poseTimeOffset or physical MTP.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import socket
import struct
import time


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return {"count": len(values), "min": values[0], "p50": values[int((len(values)-1)*.5)],
            "p95": values[int((len(values)-1)*.95)], "p99": values[int((len(values)-1)*.99)], "max": values[-1]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--socket', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=30)
    a = p.parse_args()
    if not 1 <= a.seconds <= 120:
        p.error('--seconds must be in 1..120')
    a.output.mkdir(parents=True, exist_ok=True)
    stats, gates = collections.defaultdict(list), collections.Counter()
    previous = None
    started = time.monotonic_ns()
    count = valid = session_changes = sequence_gaps = non_monotonic = 0
    error = None
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as s, (a.output/'packets.bin').open('wb') as raw:
            s.settimeout(1)
            s.connect(a.socket)
            while time.monotonic_ns() - started < a.seconds * 1e9:
                data = s.recv(241)
                now = time.monotonic_ns()
                if len(data) != 240:
                    raise RuntimeError('disconnected or wrong packet size: %s' % len(data))
                h = struct.unpack_from('<4I8Q', data)
                if h[:3] != (0x54523841, 2, 240):
                    raise RuntimeError('wrong production-v2 wire identity')
                raw.write(struct.pack('<Q', now) + data)
                count += 1
                _, _, _, flags, session, seq, gyroboot, accelboot, sample, sent, poseboot, camera = h
                q = struct.unpack_from('<4f', data, 104)
                quality, state, warnings, index, bracket, read = struct.unpack_from('<f5I', data, 132)
                vectors = [struct.unpack_from('<3f', data, o) for o in (80,92,120,168,180,192,204,216,228)]
                checks = {'flags': flags == 15, 'copied': read == 1, 'quality': quality == 1,
                          'tracking_state': state == 4, 'index': index < 80,
                          'pose_time': poseboot == gyroboot,
                          'camera_age': 0 < camera <= gyroboot and gyroboot-camera < 100_000_000,
                          'sample_age': 0 < sample <= sent <= now and now-sample < 100_000_000,
                          'clock_bracket': bracket <= 50_000,
                          'finite': all(math.isfinite(x) for v in vectors+[q] for x in v),
                          'quaternion_norm': abs(sum(x*x for x in q)-1) <= .01}
                for key, ok in checks.items():
                    if not ok: gates[key] += 1
                ok = all(checks.values())
                valid += int(ok)
                stats['observer_sample_age_ms'].append((now-sample)/1e6)
                stats['producer_to_tap_ms'].append((now-sent)/1e6)
                stats['sample_to_send_ms'].append((sent-sample)/1e6)
                stats['clock_bracket_ns'].append(bracket)
                if previous and previous[0] == session:
                    sequence_gaps += max(0, seq-previous[1]-1)
                    non_monotonic += int(seq <= previous[1] or sample <= previous[2])
                    stats['sample_interval_ms'].append((sample-previous[2])/1e6)
                elif previous:
                    session_changes += 1
                previous = session, seq, sample
                if ok:
                    stats['valid_angular_speed_rad_s'].append(math.sqrt(sum(x*x for x in vectors[3])))
                    stats['valid_linear_speed_m_s'].append(math.sqrt(sum(x*x for x in vectors[8]))/1.021)
    except Exception as e:
        error = str(e)
    report = {'schema': 1, 'clock': 'CLOCK_MONOTONIC', 'wire_version': 2, 'record_bytes': 248,
              'seconds': (time.monotonic_ns()-started)/1e9, 'packets': count, 'valid_raw_6dof': valid,
              'rejected_gates': dict(gates), 'session_changes': session_changes,
              'sequence_gaps': sequence_gaps, 'non_monotonic_samples': non_monotonic,
              'statistics': {k: distribution(v) for k,v in stats.items()}, 'error': error,
              'prediction_owner': 'SteamVR; observer performs no prediction',
              'scope': 'raw producer tap, not bridge output; no physical motion or MTP acceptance'}
    (a.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
