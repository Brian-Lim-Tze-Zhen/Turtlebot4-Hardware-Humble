#!/usr/bin/env python3
"""pair_evidence_12oct.py - one table per standing-pair set for the thesis (12 Oct 2026).
For every bag: member separation (pair_separation.py method: median distance between the two members on /social_groups),
where the pair stood and the side passed (pair_run.py, unchanged, hallway frame of the set), nearest return not on the map
near the pair (pair_run.py), and every lane the critic started (/rosout 'lane rule now follows track N'). In these runs the
only people in the hallway were the two members, so a run with a lane line is a run with a lane for a pair member (or for a
duplicate track of one); the track ids are listed so each can be checked.
usage (robot image, offline): python3 pair_evidence_12oct.py <label>=<bag>,<bag>,... [<label>=...]
  narrow sets need FRAME=-1.18,8.82,-6.80,13.50 in the environment (as pair_run.py)."""
import sys, os, re, math, sqlite3, subprocess, statistics as st
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from rcl_interfaces.msg import Log

HERE = os.path.dirname(os.path.abspath(__file__))


def t0_of(b):
    return float(re.search(r't\+0 = ([0-9.]+)', open(b + '/headon_metrics.txt').read()).group(1))


def separation(b):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=b + '/bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/social_groups']))
    seps, types = [], {}
    while rd.has_next():
        _, d, _ = rd.read_next()
        for g in deserialize_message(d, String).data.strip().split('\n'):
            f = g.split(',')
            if len(f) < 10:
                continue
            types[f[1]] = types.get(f[1], 0) + 1
            ms = [tuple(float(v) for v in p.split(';')) for p in f[9].split('|')]
            if len(ms) == 2:
                seps.append(math.dist(ms[0], ms[1]))
    return (st.median(seps) if seps else float('nan')), types


def lanes(b, t0):
    c = sqlite3.connect(b + '/bag/bag_0.db3')
    tid = c.execute("select id from topics where name='/rosout'").fetchone()
    out = []
    if not tid:
        return None
    for ts, data in c.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid[0],)):
        m = deserialize_message(data, Log)
        g = re.search(r'lane rule now follows track (\d+) \(([0-9.]+) m ahead\), robot keeps (\w+)', m.msg)
        if g:
            out.append(f't{ts / 1e9 - t0:+.1f} track {g.group(1)} {g.group(2)} m keeps {g.group(3)}')
    return out


def pair_run(b, t0):
    r = subprocess.run([sys.executable, os.path.join(HERE, 'pair_run.py'), b, str(t0)], capture_output=True, text=True)
    txt = r.stdout
    pr = re.search(r'pair \(t 0-10 s\): s ([0-9.]+) m from the start, members between l ([-+0-9.]+) and ([-+0-9.]+)', txt)
    ra = re.search(r'robot at the pair \(t\+([0-9.]+) s\): l ([-+0-9.]+) -> passed (\w+)', txt)
    nr = re.search(r'nearest return not on the map NEAR THE PAIR [^:]*: ([0-9.]+) m', txt)
    return (float(pr.group(1)), float(pr.group(2)), float(pr.group(3))) if pr else None, \
           (float(ra.group(2)), ra.group(3)) if ra else None, float(nr.group(1)) if nr else float('nan')


for arg in sys.argv[1:]:
    label, bags = arg.split('=', 1)
    rows, seps, near, nlane = [], [], [], 0
    print(f'== {label}')
    print(f'{"bag":46s} {"apart":>6s} {"pair s":>6s} {"members l":>15s} {"robot l":>8s} {"passed":8s} {"nearest":>7s}  lanes')
    for b in bags.split(','):
        t0 = t0_of(b)
        sep, types = separation(b)
        pr, ra, nr = pair_run(b, t0)
        ln = lanes(b, t0)
        seps.append(sep); near.append(nr); nlane += 1 if ln else 0
        pos = f'{pr[1]:+.2f} / {pr[2]:+.2f}' if pr else '-'
        print(f'{b.split("/")[-1]:46s} {sep:6.2f} {pr[0] if pr else float("nan"):6.2f} {pos:>15s} '
              f'{ra[0] if ra else float("nan"):+8.2f} {ra[1] if ra else "-":8s} {nr:7.3f}  '
              f'{len(ln) if ln is not None else "no rosout"}{": " + "; ".join(ln) if ln else ""}')
    print(f'SET {label}: n {len(seps)}  separation {st.mean(seps):.2f} +/- {st.stdev(seps):.2f} m ({min(seps):.2f}-{max(seps):.2f})  '
          f'nearest {st.mean(near):.3f} +/- {st.stdev(near):.3f} m  runs with a lane {nlane}/{len(seps)}\n')
