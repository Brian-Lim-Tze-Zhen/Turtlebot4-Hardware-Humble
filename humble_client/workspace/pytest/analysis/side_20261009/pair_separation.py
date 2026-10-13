#!/usr/bin/env python3
"""pair_separation.py - member separation of the standing pair in the robot pair runs (10 Oct 2026).
Median distance between the two members published on /social_groups (field 10, 'x;y|x;y') over the whole bag,
and the largest surface clearance a robot of radius 0.189 m could keep from both members of a person radius
of 0.25 m when passing through the middle: separation / 2 - 0.439 m.
usage: python3 pair_separation.py <bag folder> ..."""
import sys, math, statistics as st
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
seps_all = []
for b in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=b + '/bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/social_groups']))
    seps = []
    while rd.has_next():
        t, d, ts = rd.read_next()
        for g in deserialize_message(d, String).data.strip().split('\n'):
            f = g.split(',')
            if len(f) < 10: continue
            ms = [tuple(float(v) for v in p.split(';')) for p in f[9].split('|')]
            if len(ms) == 2: seps.append(math.dist(ms[0], ms[1]))
    if seps:
        s = st.median(seps); seps_all.append(s)
        print(f'{b.split("/")[-1]:48s} msgs {len(seps):4d}  separation {s:.2f} m  max surface between {s/2-0.439:+.2f} m')
    else:
        print(f'{b.split("/")[-1]:48s} no pair message')
if seps_all:
    print(f'ALL n {len(seps_all)}  separation {st.mean(seps_all):.2f} +/- {st.stdev(seps_all):.2f} m (range {min(seps_all):.2f} - {max(seps_all):.2f})')
