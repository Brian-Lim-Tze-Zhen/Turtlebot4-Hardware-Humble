#!/usr/bin/env python3
"""compare_days_precise.py - same measure as compare_days.py, printed with 4 decimals (11 Oct 2026).
Imports compare_days.py unchanged (with no arguments, so its own loop does nothing) and calls its run() per bag.
usage (robot image, offline): python3 compare_days_precise.py <label>=<bag folder>[,<bag folder>...] ...
Also prints the surface clearance (nearest return - 0.189 m) and the counts at 0.36, 0.40 and 0.45 m."""
import sys, statistics as st
args, sys.argv = sys.argv[1:], [sys.argv[0]]
import compare_days as C
for arg in args:
    label, bags = arg.split('=', 1)
    R = []
    for b in bags.split(','):
        r = C.run(b); R.append(r)
        print(f'  {label:28s} {b.split("/")[-1][:44]:44s} clear {r["clear"]:.4f} m  gap {r["clear"]-0.189:+.4f} m  time {r["time"]:.2f} s')
    c = [r['clear'] for r in R]; g = [x - 0.189 for x in c]; t = [r['time'] for r in R]
    sd = lambda v: st.stdev(v) if len(v) > 1 else 0.0
    print(f'SET {label:24s} n {len(R):2d} | clear {st.mean(c):.4f} +/- {sd(c):.4f} ({min(c):.4f}-{max(c):.4f}) | gap {st.mean(g):.4f} '
          f'| time {st.mean(t):.3f} +/- {sd(t):.3f} | gap>=0.36: {sum(x >= 0.36 for x in g)}  >=0.40: {sum(x >= 0.40 for x in g)}  >=0.45: {sum(x >= 0.45 for x in g)}  >0.45: {sum(x > 0.45 for x in g)}')
