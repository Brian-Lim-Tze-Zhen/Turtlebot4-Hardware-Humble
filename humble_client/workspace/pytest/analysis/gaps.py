#!/usr/bin/env python3
"""Attribute camera-ray output gaps to their upstream cause.

For every gap in /camera_ray_clusters, checks whether /person_positions_map
(YOLO) also had a gap at the same moment. If it did, the camera-ray node
had no input and the loss is inherited from the camera stage; if it did
not, the ray/selection/guard logic dropped it and the loss belongs to
this node.

Usage: python3 gaps.py <bag.db3> [min_gap_s]
"""
import sqlite3, struct, sys

CAMRAY="/camera_ray_clusters"; YOLO="/person_positions_map"

def pstr(d):
    (ln,)=struct.unpack_from('<I',d,4); return d[8:8+ln-1].decode('utf8','replace')

def load(c,T,t):
    if t not in T: return []
    return [ts/1e9 for ts,_ in c.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',(T[t],))]

def gaps(ts,thr):
    return [(ts[i-1],ts[i]-ts[i-1]) for i in range(1,len(ts)) if ts[i]-ts[i-1]>thr]

def main(path,thr=0.5):
    c=sqlite3.connect(path); T={r[1]:r[0] for r in c.execute('select id,name from topics')}
    cam=load(c,T,CAMRAY); yol=load(c,T,YOLO)
    if not cam: print("no camera_ray data"); return
    t0=cam[0]
    cg=gaps(cam,thr); yg=gaps(yol,thr)
    print(f"{path.split('/')[-3] if '/' in path else path}")
    print(f"  camera_ray {len(cam)} msgs, {len(cg)} gaps >{thr}s")
    print(f"  YOLO       {len(yol)} msgs, {len(yg)} gaps >{thr}s\n")
    inherited=own=0
    print(f"  {'at':>8} {'gap':>7}  cause")
    for at,g in cg:
        # does a YOLO gap overlap this window?
        hit=[y for y in yg if not (y[0]+y[1] < at or y[0] > at+g)]
        if hit:
            inherited+=g; cause=f"INHERITED (YOLO gap {hit[0][1]:.2f}s)"
        else:
            own+=g; cause="OWN (ray/selection/guard)"
        print(f"  {at-t0:7.2f}s {g:6.2f}s  {cause}")
    tot=inherited+own
    if tot:
        print(f"\n  total gap {tot:.1f}s = inherited {inherited:.1f}s "
              f"({inherited/tot*100:.0f}%) + own {own:.1f}s ({own/tot*100:.0f}%)")

if __name__=="__main__":
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv)>2 else 0.5)
