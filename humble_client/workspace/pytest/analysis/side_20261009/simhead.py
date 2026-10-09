import csv,math,glob,os
def heading_stats(R):
    best=(0,0); wzmax=0; t1=0
    for i,r in enumerate(R):
        wzmax=max(wzmax,abs(r['wz']))
        if i and abs(r['wz'])>1.0: t1+=r['t']-R[i-1]['t']
        j=i
        while j<len(R)-1 and math.hypot(R[j]['x']-r['x'],R[j]['y']-r['y'])<0.10: j+=1
        if j<len(R)-1 and R[j]['t']-r['t']<1.5:
            h=math.degrees(math.atan2(R[j]['y']-r['y'],R[j]['x']-r['x']))
            if abs(h)>abs(best[0]): best=(h,r['y'])
    return best,wzmax,t1
if __name__=='__main__':
    print('trial                  max|heading|  max|wz|  lateral at max  time |wz|>1')
    for d in sorted(glob.glob('headon_rec15_trial*')):
        if not os.path.exists(d+'/trajectory.csv'): continue
        R=[{'t':float(r['t']),'x':float(r['x_ref']),'y':float(r['y_ref']),'wz':float(r['wz'])} for r in csv.DictReader(open(d+'/trajectory.csv'))]
        (h,y),w,t1=heading_stats(R)
        print(f'{d:22s} {h:+6.0f} deg   {w:4.2f}     {y:+5.2f} m       {t1:4.1f} s')
