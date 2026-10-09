import sys, math
def flips(seq, thr=0.15):
    n=0; last=0
    for w in seq:
        s=1 if w>thr else (-1 if w<-thr else 0)
        if s and last and s!=last: n+=1
        if s: last=s
    return n
if __name__=='__main__' and sys.argv[1]=='sim':
    import csv,glob,os
    for d in sorted(glob.glob('/home/brian/thesis_social_navigation/ros2_ws/bags/headon_rec15_trial*')):
        if not os.path.exists(d+'/trajectory.csv'): continue
        R=[r for r in csv.DictReader(open(d+'/trajectory.csv')) if abs(float(r['vx']))>0.02]
        dist=sum(abs(float(R[i]['vx']))*(float(R[i]['t'])-float(R[i-1]['t'])) for i in range(1,len(R)))
        print(f'{os.path.basename(d):26s} wz flips {flips([float(r["wz"]) for r in R]):3d}  over {dist:5.2f} m  -> {flips([float(r["wz"]) for r in R])/dist:4.1f}/m')
if __name__=='__main__' and sys.argv[1]=='robot':
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    for b in sys.argv[2:]:
        r=rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=b+'/bag',storage_id=''),rosbag2_py.ConverterOptions('',''))
        r.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/odom']))
        W=[]; dist=0; lt=None
        while r.has_next():
            t,d,ts=r.read_next(); m=deserialize_message(d,Odometry); vx=m.twist.twist.linear.x
            if abs(vx)>0.02:
                W.append(m.twist.twist.angular.z)
                if lt: dist+=abs(vx)*min(0.2,ts*1e-9-lt)
            lt=ts*1e-9
        f=flips(W); print(f'{b.split("/")[-1][:38]:38s} wz flips {f:3d}  over {dist:5.2f} m  -> {f/dist:4.1f}/m')
if __name__=='__main__' and sys.argv[1]=='cmd':
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist
    for b in sys.argv[2:]:
        out=[]
        for top in ['/turtlebot4/cmd_vel_nav','/turtlebot4/cmd_vel']:
            r=rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=b+'/bag',storage_id=''),rosbag2_py.ConverterOptions('',''))
            r.set_filter(rosbag2_py.StorageFilter(topics=[top])); W=[]
            while r.has_next():
                t,d,ts=r.read_next(); m=deserialize_message(d,Twist)
                if abs(m.linear.x)>0.02: W.append(m.angular.z)
            out.append(f'{top.split("/")[-1]} {flips(W):3d}')
        print(f'{b.split("/")[-1][:36]:36s} flips: '+'  '.join(out))
