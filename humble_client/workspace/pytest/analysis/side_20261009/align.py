import sys, rosbag2_py, math, numpy as np
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt
from PIL import Image
import yaml
M=yaml.safe_load(open('/root/thesis_social_navigation_ws/map/hall_map_name.yaml'))
img=np.array(Image.open('/root/thesis_social_navigation_ws/map/'+M['image']))
occ=img<100; res=M['resolution']; ox0,oy0=M['origin'][:2]; Hh=img.shape[0]
dist=distance_transform_edt(~occ)*res
def dmap(x,y):
    c=((x-ox0)/res).astype(int); r=(Hh-1-((y-oy0)/res)).astype(int)
    ok=(c>=0)&(c<img.shape[1])&(r>=0)&(r<Hh); out=np.full(x.shape,2.0); out[ok]=dist[r[ok],c[ok]]; return out
for arg in sys.argv[1:]:
    b,T0=arg.split(':'); T0=float(T0)
    rd=rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=b+'/bag',storage_id=''),rosbag2_py.ConverterOptions('',''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/tf','/turtlebot4/tf_static','/turtlebot4/odom','/turtlebot4/scan']))
    m2o=None; robot=None; lyaw=None; res_=[]
    while rd.has_next():
        t,d,ts=rd.read_next(); tt=ts*1e-9-T0
        if t.endswith('tf') or t.endswith('tf_static'):
            for tr in deserialize_message(d,TFMessage).transforms:
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    q=tr.transform.rotation; m2o=(tr.transform.translation.x,tr.transform.translation.y,2*math.atan2(q.z,q.w))
                if tr.child_frame_id.endswith('rplidar_link'):
                    q=tr.transform.rotation; lyaw=2*math.atan2(q.z,q.w); lxy=(tr.transform.translation.x,tr.transform.translation.y)
        elif t.endswith('odom') and m2o:
            m=deserialize_message(d,Odometry); p=m.pose.pose.position; q=m.pose.pose.orientation; ox,oy,oth=m2o
            robot=(ox+p.x*math.cos(oth)-p.y*math.sin(oth), oy+p.x*math.sin(oth)+p.y*math.cos(oth), oth+2*math.atan2(q.z,q.w))
        elif t.endswith('scan') and robot and lyaw is not None and -1.5<tt<0:
            m=deserialize_message(d,LaserScan); rg=np.array(m.ranges); a=m.angle_min+np.arange(len(rg))*m.angle_increment
            ok=np.isfinite(rg)&(rg>0.2)&(rg<8); rg,a=rg[ok],a[ok]
            px=lxy[0]+rg*np.cos(a+lyaw); py=lxy[1]+rg*np.sin(a+lyaw)
            best=[]
            for dyaw in np.arange(-12,12.01,0.5):
                th=robot[2]+math.radians(dyaw)
                X=robot[0]+px*math.cos(th)-py*math.sin(th); Y=robot[1]+px*math.sin(th)+py*math.cos(th)
                best.append((np.mean(np.minimum(dmap(X,Y),0.5)),dyaw))
            res_.append((min(best)[1], dict((d_,v) for v,d_ in best)[0.0], min(best)[0]))
    if res_:
        print(f'{b.split("/")[-1][:30]:30s} best yaw correction {np.median([r[0] for r in res_]):+5.1f} deg | median scan-to-map dist at AMCL yaw {np.median([r[1] for r in res_]):.3f} m, at best {np.median([r[2] for r in res_]):.3f} m  (n {len(res_)})')
