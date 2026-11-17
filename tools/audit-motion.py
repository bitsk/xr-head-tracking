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


def pose_fields(data, now):
    """Describe raw field integrity/confidence; never change or supply a pose."""
    h = struct.unpack_from('<4I8Q', data)
    _, _, _, flags, session, seq, gyroboot, accelboot, sample, sent, poseboot, camera = h
    q = struct.unpack_from('<4f', data, 104)
    quality, state, warnings, index, bracket, read = struct.unpack_from('<f5I', data, 132)
    vectors = [struct.unpack_from('<3f', data, o) for o in (80,92,120,168,180,192,204,216,228)]
    norm = sum(x*x for x in q)
    checks = {'source_flags': flags & 7 == 7 and not flags & ~15,
              'session_identity': bool(session and seq and accelboot), 'copied': read == 1,
              'index': index < 80, 'pose_epoch': 0 < poseboot == gyroboot,
              'clock_causality': 0 < sample <= sent <= now,
              'orientation_representation': all(math.isfinite(x) for x in q) and math.isfinite(norm) and norm > 0}
    rotation = all(checks.values())
    tracked = bool(state & 4) and not state & (2 | 8)
    finite3 = lambda v: all(math.isfinite(x) for x in v)
    fields = {'orientation_available': rotation,
              'orientation_tracked': rotation and tracked,
              'position_available': rotation and finite3(vectors[7]),
              'position_tracked': rotation and tracked and finite3(vectors[7]) and math.isfinite(quality) and quality >= .2,
              'angular_velocity_available': rotation and finite3(vectors[3]),
              'linear_velocity_available': rotation and finite3(vectors[8])}
    return h, checks, fields, vectors, state, bracket


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--socket', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=30)
    a = p.parse_args()
    if not 1 <= a.seconds <= 120:
        p.error('--seconds must be in 1..120')
    a.output.mkdir(parents=True, exist_ok=True)
    stats, failures = collections.defaultdict(list), collections.Counter()
    field_counts, states = collections.Counter(), collections.Counter()
    previous = None
    started = time.monotonic_ns()
    count = session_changes = sequence_gaps = non_monotonic = 0
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
                _, checks, fields, vectors, state, bracket = pose_fields(data, now)
                for key, ok in checks.items():
                    if not ok: failures[key] += 1
                field_counts.update({key: int(value) for key, value in fields.items()})
                states[state] += 1
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
                if fields['angular_velocity_available']:
                    stats['available_angular_speed_rad_s'].append(math.sqrt(sum(x*x for x in vectors[3])))
                if fields['linear_velocity_available']:
                    stats['available_linear_speed_m_s'].append(math.sqrt(sum(x*x for x in vectors[8]))/1.021)
    except Exception as e:
        error = str(e)
    report = {'schema': 2, 'clock': 'CLOCK_MONOTONIC', 'wire_version': 2, 'record_bytes': 248,
              'seconds': (time.monotonic_ns()-started)/1e9, 'packets': count,
              'field_packets': dict(field_counts), 'orientation_integrity_failures': dict(failures),
              'oem_tracking_states': dict(states), 'session_changes': session_changes,
              'sequence_gaps': sequence_gaps, 'non_monotonic_samples': non_monotonic,
              'statistics': {k: distribution(v) for k,v in stats.items()}, 'error': error,
              'prediction_owner': 'SteamVR; observer performs no prediction',
              'scope': 'raw producer field integrity/confidence, not runtime acceptance or freshness; no physical motion or MTP acceptance'}
    (a.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
