#!/usr/bin/env python3
"""Sweep occlusion-guard designs for camera_ray_person_node.

The shipped guard compares each new range against the LAST ACCEPTED
range for that camera id. When the ray is bistable that reference can
latch onto the wrong branch and reject every subsequent reading until
it times out - observed live as cam:11 rejecting 3.94/4.42/3.65/3.62
for a full second, then rejecting the way back down again.

Designs compared:
  none      no guard (baseline)
  last-acc  current: compare vs last ACCEPTED range (latching)
  last-raw  compare vs last RAW measurement, accepted or not
  median3   compare vs median of last 3 raw measurements
  hysteres  last-raw, but force-accept after N consecutive rejects

Metric is what the guard exists to control:
  rejected%   how much data it throws away
  >0.3m %     residual instability among ACCEPTED readings
  maxrun      longest consecutive reject streak (the latching failure)
"""
import sqlite3, struct, math, sys

SCAN="/turtlebot4/scan"; CAM="/person_positions_map"
CX=124.95; YAW=math.radians(2.25); FX=201.308
HALF_W=math.radians(6.0)          # Result 1 value
RMIN,RMAX=0.20,8.0
JUMP_M=0.8; JUMP_SPEED=2.0; HIST_T=1.5
FORCE_AFTER=3

def align(o,a): return (o+a-1)//a*a
def parse_scan(d):
    o=4; o+=8
    (ln,)=struct.unpack_from('<I',d,o); o+=4; o+=ln; o=align(o,4)
    amin,amax,ainc,ti,st,rmin,rmax=struct.unpack_from('<7f',d,o); o+=28
    (n,)=struct.unpack_from('<I',d,o); o+=4
    return amin,ainc,struct.unpack_from('<%df'%n,d,o)
def pstr(d):
    (ln,)=struct.unpack_from('<I',d,4); return d[8:8+ln-1].decode('utf8','replace')
def wrap(x):
    while x>math.pi: x-=2*math.pi
    while x<-math.pi: x+=2*math.pi
    return x

def measure(path):
    """Replay -> list of (t, cam_id, range) raw ray measurements."""
    c=sqlite3.connect(path); T={r[1]:r[0] for r in c.execute('select id,name from topics')}
    scans=[(ts,parse_scan(d)) for ts,d in c.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',(T[SCAN],))]
    out=[]; si=0
    for ts,d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(T[CAM],)):
        while si+1<len(scans) and scans[si+1][0]<=ts: si+=1
        amin,ainc,ranges=scans[si][1]
        p=pstr(d).split(',')
        if len(p)<7 or not p[6].strip(): continue
        k=[]
        for pr in p[6].split(';'):
            if ':' in pr:
                try:
                    px,py=pr.split(':'); k.append((float(px),float(py)))
                except ValueError: pass
        if not k: continue
        k.sort(key=lambda q:-q[1]); u=sum(q[0] for q in k[:2])/len(k[:2])
        b=-math.atan((u-CX)/FX)+YAW
        best=None
        for i,r in enumerate(ranges):
            a=amin+i*ainc
            if abs(wrap(a-b))<=HALF_W and RMIN<=r<=RMAX and not math.isinf(r) and not math.isnan(r):
                if best is None or r<best: best=r
        if best is not None:
            out.append((ts/1e9,int(float(p[0])),best))
    return out

def run(meas,mode):
    acc={}; raw={}; hist={}; streak={}
    accepted=[]; nrej=0; maxrun=0
    for t,cid,r in meas:
        ref=None
        if mode=="none": ref=None
        elif mode=="last-acc":
            v=acc.get(cid); ref=v if v and t-v[1]<=HIST_T else None
        elif mode in ("last-raw","hysteres"):
            v=raw.get(cid); ref=v if v and t-v[1]<=HIST_T else None
        elif mode=="median3":
            h=[x for x in hist.get(cid,[]) if t-x[1]<=HIST_T]
            if h:
                s=sorted(x[0] for x in h); ref=(s[len(s)//2],h[-1][1])
        ok=True
        if ref is not None:
            allowed=JUMP_M+JUMP_SPEED*(t-ref[1])
            if abs(r-ref[0])>allowed: ok=False
        if mode=="hysteres" and not ok and streak.get(cid,0)>=FORCE_AFTER:
            ok=True
        raw[cid]=(r,t)
        hist.setdefault(cid,[]).append((r,t)); hist[cid]=hist[cid][-3:]
        if ok:
            acc[cid]=(r,t); accepted.append((t,cid,r)); streak[cid]=0
        else:
            nrej+=1; streak[cid]=streak.get(cid,0)+1
            maxrun=max(maxrun,streak[cid])
    jumps=sum(1 for i in range(1,len(accepted))
              if accepted[i][1]==accepted[i-1][1]
              and abs(accepted[i][2]-accepted[i-1][2])>0.3)
    n=max(1,len(accepted)-1)
    return len(meas),nrej,nrej/max(1,len(meas))*100,jumps/n*100,maxrun

for path in sys.argv[1:]:
    m=measure(path)
    print(f"\n{path}")
    print(f"  {len(m)} raw ray measurements")
    print(f"  {'guard':>10} {'rejected':>9} {'>0.3m':>8} {'maxrun':>7}")
    for mode in ("none","last-acc","last-raw","median3","hysteres"):
        tot,nrej,pr,pj,mr=run(m,mode)
        print(f"  {mode:>10} {pr:8.1f}% {pj:7.2f}% {mr:6d}")
