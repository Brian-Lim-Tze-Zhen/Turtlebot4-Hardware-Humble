import sys, rosbag2_py, math, re
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import Log
H=math.radians(141.8)
for arg in sys.argv[1:]:
    b,T0=arg.split(':'); T0=float(T0)
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b+'/bag',storage_id=''),rosbag2_py.ConverterOptions('',''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/tf','/turtlebot4/odom','/person_positions_fused','/person_positions_map','/rosout']))
    m2o=None; robot=None; lastcam=None; yolo=[]; firstlid=None; walker=None; notes=[]
    while r.has_next():
        t,d,ts=r.read_next(); tt=ts*1e-9-T0
        if tt>9: break
        if t.endswith('/tf'):
            for tr in deserialize_message(d,TFMessage).transforms:
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    q=tr.transform.rotation; m2o=(tr.transform.translation.x,tr.transform.translation.y,2*math.atan2(q.z,q.w))
        elif t.endswith('odom') and m2o:
            m=deserialize_message(d,Odometry); p=m.pose.pose.position; q=m.pose.pose.orientation; ox,oy,oth=m2o
            robot=(ox+p.x*math.cos(oth)-p.y*math.sin(oth), oy+p.x*math.sin(oth)+p.y*math.cos(oth), oth+2*math.atan2(q.z,q.w))
        elif t=='/person_positions_map' and tt>0:
            f=deserialize_message(d,String).data.split(',')
            if len(f)>5: yolo.append((tt,int(f[2]),int(f[4])))
        elif t=='/person_positions_fused' and tt>0 and robot:
            for e in deserialize_message(d,String).data.split(';'):
                f=e.split(',')
                if len(f)>3:
                    wx,wy=float(f[2]),float(f[3]); src=f[-2]
                    brg=math.degrees(math.atan2(wy-robot[1],wx-robot[0])-robot[2]); brg=(brg+180)%360-180
                    yaw=(math.degrees(robot[2]-H)+180)%360-180
                    if src=='camera_confirmed': lastcam=(tt,brg,yaw,math.hypot(wx-robot[0],wy-robot[1]))
                    elif lastcam and firstlid is None and tt>lastcam[0]: firstlid=(tt,brg,yaw)
        elif t=='/rosout' and lastcam and tt<lastcam[0]+1.5 and tt>lastcam[0]-0.3:
            m=deserialize_message(d,Log)
            if 'camera_ray' in m.name and any(k in m.msg for k in ('refused','no scan','release','lost','edge')): notes.append(f'{tt:+.2f} {m.msg[:70]}')
            if 'yolo' in m.name and 'REJECT' in m.msg: notes.append(f'{tt:+.2f} YOLO {m.msg[:60]}')
    name=b.split('/')[-1][:28]
    if not lastcam: print(name,'no camera_confirmed after t+0'); continue
    yb=[y for y in yolo if lastcam[0]-0.3<y[0]<lastcam[0]+1.0]
    print(f'{name}: last camera fix t+{lastcam[0]:.2f}, walker {lastcam[3]:.1f} m at bearing {lastcam[1]:+.0f} deg, robot yaw {lastcam[2]:+.0f} deg; '
          + (f'next lidar_only t+{firstlid[0]:.2f} bearing {firstlid[1]:+.0f}, yaw {firstlid[2]:+.0f}' if firstlid else ''))
    print('   YOLO boxes x1-x2 (image 0-250) around then:', ' '.join(f'{t:+.1f}:{a}-{c}' for t,a,c in yb[-8:]))
    for n in notes[:6]: print('   ',n)
