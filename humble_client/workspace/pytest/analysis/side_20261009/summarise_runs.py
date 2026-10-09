#!/usr/bin/env python3
"""summarise_runs.py - metrics per rec20 noghost run from rec20_runs_data.json (extract_runs.py). 9 Oct 2026.
Writes rec20_summary.json (page data) and prints a table. Windows relative to t+0 (goal sent):
side/decision from the critic log; robot side = sign of the extreme sideways position t+2..9 s;
closest unmapped LiDAR return = nearest scan point not on the map, ahead of the start (s > -0.2 m), t+2..12 s
(the robot's own table/start area behind it excluded); slowest speed t+3..10 s; time to goal = first to last odom vx > 0.02."""
import json, re
D = json.load(open('rec20_runs_data.json'))
S = []
for r in D['runs']:
    E = r['events']; run = re.search(r'(run\d\d)', r['name']).group(1)
    fixed = [e for e in E if 'pass side fixed' in e[1] and f"track {r['track']} " in e[1]][0]
    side = 'R' if 'RIGHT' in fixed[1] else 'L'
    off = float(re.search(r'offset ([-+0-9.]+) m', fixed[1]).group(1))
    lane = [e for e in E if 'lane rule now follows' in e[1] and f"track {r['track']} " in e[1] and e[0] <= fixed[0]]
    lane = lane[-1] if lane else [e for e in E if 'lane rule now follows' in e[1] and e[0] <= fixed[0]][-1]
    dist = float(re.search(r'\(([0-9.]+) m ahead\)', lane[1]).group(1))
    P = r['path']; mv = [p for p in P if p[4] > 0.02]
    win = [p for p in P if 2 <= p[0] <= 9]
    ext = max(win, key=lambda p: abs(p[2]))
    head = max(win, key=lambda p: abs(p[3]))
    near = [n for n in r['near'] if 2 <= n[0] <= 12 and n[2] > -0.2]
    cl = min(near, key=lambda n: n[1])
    spd = min((p for p in P if 3 <= p[0] <= 10), key=lambda p: p[4])
    S.append({'run': run, 'name': r['name'], 'loc_err_deg': r['loc_err_deg'], 'side': side, 'offset': off,
              'decision_t': fixed[0], 'decision_dist': dist, 'robot_side': 'R' if ext[2] < 0 else 'L',
              'max_lateral': round(abs(ext[2]), 2), 'max_heading': round(abs(head[3]), 0),
              'clearance': round(cl[1], 2), 'clearance_t': cl[0], 'clearance_sl': [cl[2], cl[3]],
              'min_speed': round(spd[4], 2), 'time_to_goal': round(mv[-1][0] - mv[0][0], 1),
              'occlusion': any('occlusion slow-down ON' in e[1] for e in E if 0 < e[0] < 12),
              'ghost_drop': next((e[0] for e in E if 'dropped as a ghost' in e[1]), None)})
json.dump(S, open('rec20_summary.json', 'w'), indent=1)
print('run   side off   dec_t dist  robot maxlat maxhead clear@t   minv  ttg   occl ghost loc')
for s in S:
    print(f"{s['run']} {s['side']}   {s['offset']:+.2f} {s['decision_t']:4.1f} {s['decision_dist']:4.2f}  {s['robot_side']}     {s['max_lateral']:.2f}   {s['max_heading']:3.0f}    {s['clearance']:.2f}@{s['clearance_t']:4.1f} {s['min_speed']:.2f} {s['time_to_goal']:4.1f} {str(s['occlusion'])[0]}    {s['ghost_drop']}  {s['loc_err_deg']}")
import statistics as st
for k in ('clearance', 'max_lateral', 'max_heading', 'time_to_goal', 'decision_dist', 'min_speed'):
    v = [s[k] for s in S]; print(f'{k}: mean {st.mean(v):.2f} sd {st.stdev(v):.2f} min {min(v):.2f} max {max(v):.2f}')
