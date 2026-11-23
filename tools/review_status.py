#!/usr/bin/env python3
"""Finite, read-only recording status; does not start/stop any service."""
import csv
import json
from pathlib import Path
import subprocess
import time

root = Path.home() / 'flight_reviews'
subprocess.run(['systemctl', 'is-active', 'balloon-vision.service'], check=False)
sessions = sorted(p for p in root.glob('*') if p.is_dir())
if not sessions:
    raise SystemExit('No recording sessions found. Check balloon-vision.service logs.')
session = sessions[-1]
print('SESSION:', session)
manifest = session / 'manifest.json'
if manifest.exists():
    m = json.loads(manifest.read_text())
    for key in ('status', 'frames_written', 'segments', 'frame_queue_drops', 'event_queue_drops', 'error'):
        print(f'{key}: {m.get(key)}')
    print('manifest age:', round(time.time() - manifest.stat().st_mtime, 1), 's')
for path in sorted(session.iterdir()):
    if path.is_file():
        print(f'{path.name}: {path.stat().st_size / 1024:.1f} KiB')

topics = {}
last = {}
log = session / 'telemetry.jsonl'
if log.exists():
    with log.open() as f:
        for line in f:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue  # The recorder may still be writing the last line.
            topic = event.get('topic', '?')
            topics[topic] = topics.get(topic, 0) + 1
            last[topic] = event
print('RECORDED TOPICS:')
for topic, count in sorted(topics.items()):
    print(f'  {topic}: {count}')
status = last.get('/fmu/out/vehicle_status_v1', {}).get('data')
if status:
    print('LAST PX4:', {k: status.get(k) for k in ('arming_state', 'nav_state', 'failsafe')})
else:
    print('WARNING: no PX4 status recorded yet; check DDS connection/domain.')
print('AVI playback should be checked after a clean detector stop closes the segment.')
