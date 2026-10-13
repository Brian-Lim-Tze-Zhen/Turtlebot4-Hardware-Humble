#!/usr/bin/env python3
"""headon_evidence_12oct.py - lane decision per head-on run for the thesis (12 Oct 2026).
From /rosout of each bag: the first 'lane rule now follows track N (d m ahead), robot keeps SIDE' and the first
'pass side fixed: robot keeps SIDE (walker's mean offset X m ...)', relative to t+0 (headon_metrics.txt).
usage (robot image, offline): python3 headon_evidence_12oct.py <label>=<bag>,<bag>,..."""
import sys, re, sqlite3, statistics as st
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log
for arg in sys.argv[1:]:
    label, bags = arg.split('=', 1); dist = []; sides = {}
    print(f'== {label}')
    for b in bags.split(','):
        t0 = float(re.search(r't\+0 = ([0-9.]+)', open(b + '/headon_metrics.txt').read()).group(1))
        c = sqlite3.connect(b + '/bag/bag_0.db3'); tid = c.execute("select id from topics where name='/rosout'").fetchone()[0]
        lane = fix = None
        for ts, data in c.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid,)):
            m = deserialize_message(data, Log).msg
            g = re.search(r'lane rule now follows track \d+ \(([0-9.]+) m ahead\), robot keeps (\w+)', m)
            if g and lane is None: lane = (ts / 1e9 - t0, float(g.group(1)), g.group(2))
            g = re.search(r'pass side fixed: robot keeps (\w+) \(walker.s mean offset ([-+0-9.]+) m', m)
            if g and fix is None: fix = (ts / 1e9 - t0, g.group(1), float(g.group(2)))
        if lane: dist.append(lane[1]); sides[lane[2]] = sides.get(lane[2], 0) + 1
        print(f'{b.split("/")[-1]:46s} lane ' + (f't{lane[0]:+.1f} s at {lane[1]:.2f} m keeps {lane[2]:5s}' if lane else 'none') +
              ' | fixed ' + (f't{fix[0]:+.1f} s keeps {fix[1]:5s} offset {fix[2]:+.2f} m' if fix else 'none'))
    print(f'SET {label}: lane distance {st.mean(dist):.2f} +/- {st.stdev(dist):.2f} m ({min(dist):.2f}-{max(dist):.2f}), sides {sides}\n')
