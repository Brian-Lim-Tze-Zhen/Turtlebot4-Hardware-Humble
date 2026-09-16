import sqlite3, struct, math, sys
from collections import defaultdict
SCAN="/turtlebot4/scan"; CAM="/person_positions_map"
CX=124.95; YAW=math.radians(2.25); FX=201.308
RMIN,RMAX=0.20,8.0
def align(o,a): return (o+a-1)//a*a
def parse_scan(d):
    o=4; sec,nsec=struct.unpack_from('<iI',d,o); o+=8
    (ln,)=struct.unpack_from('<I',d,o); o+=4; o+=ln; o=align(o,4)
    amin,amax,ainc,tinc,st,rmin,rmax=struct.unpack_from('<7f',d,o); o+=28
    (n,)=struct.unpack_from('<I',d,o); o+=4
    return amin,ainc,struct.unpack_from('<%df'%n,d,o)
def pstr(d):
    (ln,)=struct.unpack_from('<I',d,4); return d[8:8+ln-1].decode('utf8','replace')
def wrap(x):
    while x>math.pi: x-=2*math.pi
    while x<-math.pi: x+=2*math.pi
    return x
def window(amin,ainc,ranges,b,hw):
    out=[]
    for i,r in enumerate(ranges):
        a=amin+i*ainc
        if abs(wrap(a-b))<=hw and RMIN<=r<=RMAX and not math.isinf(r) and not math.isnan(r):
            out.append((a,r))
    return out

def rule_min(w,b): return min(w,key=lambda t:t[1]) if w else None
def rule_median(w,b):
    if not w: return None
    s=sorted(w,key=lambda t:t[1]); return s[len(s)//2]
def rule_centre(w,b):
    return min(w,key=lambda t:abs(wrap(t[0]-b))) if w else None
def rule_cluster(w,b):
    """Group returns into range-contiguous clusters, take the cluster
    whose centroid bearing is nearest the ray, then its median range."""
    if not w: return None
    s=sorted(w,key=lambda t:t[0])
    cl=[[s[0]]]
    for a,r in s[1:]:
        if abs(r-cl[-1][-1][1])<=0.25: cl[-1].append((a,r))
        else: cl.append([(a,r)])
    best=min(cl,key=lambda c:abs(wrap(sum(a for a,_ in c)/len(c)-b)))
    rs=sorted(x[1] for x in best)
    return (sum(a for a,_ in best)/len(best), rs[len(rs)//2])

RULES=[("min",rule_min),("median",rule_median),("nearest-centre",rule_centre),("cluster",rule_cluster)]

def run(path,hw_deg):
    hw=math.radians(hw_deg)
    c=sqlite3.connect(path); T={r[1]:r[0] for r in c.execute('select id,name from topics')}
    scans=[(ts,parse_scan(d)) for ts,d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(T[SCAN],))]
    cams=list(c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(T[CAM],)))
    out={n:[] for n,_ in RULES}
    si=0
    for ts,d in cams:
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
        w=window(amin,ainc,ranges,b,hw)
        for n,f in RULES:
            res=f(w,b)
            if res: out[n].append((ts/1e9,res[1],math.degrees(wrap(res[0]-b))))
    return out

def stats(v):
    """Jitter = median |consecutive range change|. Low = stable lock."""
    if len(v)<3: return None
    d=[abs(v[i][1]-v[i-1][1]) for i in range(1,len(v))]
    d.sort(); jit=d[len(d)//2]
    big=sum(1 for x in d if x>0.3)/len(d)*100
    rs=[x[1] for x in v]; m=sum(rs)/len(rs)
    sd=(sum((r-m)**2 for r in rs)/len(rs))**0.5
    return len(v),jit,big,sd

path=sys.argv[1]
for hw in (3,6,10):
    print(f"=== window +/-{hw} deg ===")
    print(f"{'rule':>15} | {'n':>5} {'jitter':>8} {'>0.3m %':>8} {'range sd':>9}")
    r=run(path,hw)
    for n,_ in RULES:
        s=stats(r[n])
        if s: print(f"{n:>15} | {s[0]:>5} {s[1]:>7.3f}m {s[2]:>7.1f}% {s[3]:>8.3f}m")
    print()
