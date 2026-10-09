import sys, rosbag2_py, math, re, statistics as st
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from std_msgs.msg import String
H=math.radians(141.8); SX,SY=-6.80,13.50; GX,GY=-12.20,17.75
def sl(x,y): dx,dy=x-SX,y-SY; return dx*math.cos(H)+dy*math.sin(H), -dx*math.sin(H)+dy*math.cos(H)
for arg in sys.argv[1:]:
    b,T0=arg.split(':'); T0=float(T0)
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b+'/bag',storage_id=''),rosbag2_py.ConverterOptions('',''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/tf','/turtlebot4/odom','/rosout','/person_positions_fused']))
    m2o=None; robot=None; rays=[]; walk=[]; start=None
    while r.has_next():
        t,d,ts=r.read_next(); tt=ts*1e-9-T0
        if t.endswith('/tf'):
            for tr in deserialize_message(d,TFMessage).transforms:
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    q=tr.transform.rotation; m2o=(tr.transform.translation.x,tr.transform.translation.y,2*math.atan2(q.z,q.w))
        elif t.endswith('odom') and m2o:
            m=deserialize_message(d,Odometry); p=m.pose.pose.position; q=m.pose.pose.orientation; ox,oy,oth=m2o
            x=ox+p.x*math.cos(oth)-p.y*math.sin(oth); y=oy+p.x*math.sin(oth)+p.y*math.cos(oth); yaw=oth+2*math.atan2(q.z,q.w)
            robot=(x,y,yaw)
            if start is None and tt>-0.5: start=robot
        elif t=='/rosout' and 0.5<tt<3.5:
            m=deserialize_message(d,Log)
            g=re.search(r'hit ([-+0-9.]+) deg @ ([0-9.]+) m',m.msg)
            if 'camera_ray' in m.name and g and robot:
                rays.append((float(g.group(1)),float(g.group(2)),robot))
        elif t=='/person_positions_fused' and 0.5<tt<3.5:
            for e in deserialize_message(d,String).data.split(';'):
                f=e.split(',')
                if len(f)>3 and robot:
                    walk.append((float(f[2]),float(f[3]),robot))
    s0,l0=sl(*start[:2]); yawdev=math.degrees(start[2]-H); yawdev=(yawdev+180)%360-180
    # walker offset vs robot-goal line, and vs hallway axis
    off_goal=[]; l_hall=[]
    for wx,wy,(rx,ry,ryaw) in walk:
        gx,gy=GX-rx,GY-ry; n=math.hypot(gx,gy); ux,uy=gx/n,gy/n
        off_goal.append(-uy*(wx-rx)+ux*(wy-ry)); l_hall.append(sl(wx,wy)[1])
    # same from camera rays in robot frame, independent of the person pipeline
    ray_goal=[]
    for hb,rg,(rx,ry,ryaw) in rays:
        gb=math.atan2(GY-ry,GX-rx)-ryaw
        ray_goal.append(rg*math.sin(math.radians(hb)-gb))
    name=b.split('/')[-1][:30]
    print(f'{name:30s} start l {l0:+.2f} yaw {yawdev:+4.0f} | walker l(hall) {st.median(l_hall):+.2f} | offset vs goal line: fused {st.median(off_goal):+.2f} (n{len(off_goal)}), rays {st.median(ray_goal) if ray_goal else float("nan"):+.2f} (n{len(ray_goal)})')
