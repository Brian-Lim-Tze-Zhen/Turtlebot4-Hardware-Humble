#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, TwistStamped
from tf2_ros import Buffer, TransformListener
FRAMES=['base_link','turtlebot4/base_link','base_footprint','turtlebot4/base_footprint']
class D(Node):
    def __init__(s):
        super().__init__('stop_diag')
        s.tf=Buffer(); TransformListener(s.tf,s); s.frame=None
        s.v=float('nan'); s.dmin=float('nan'); s.run_min=float('inf'); s.bend=float('nan'); s.npts=0
        s.create_subscription(PointCloud2,'/predicted_person_cloud',s.cloud,qos_profile_sensor_data)
        s.create_subscription(Path,'/turtlebot4/plan',s.plan,10)
        types=dict(s.get_topic_names_and_types())
        t=types.get('/turtlebot4/cmd_vel',['geometry_msgs/msg/Twist'])[0]
        if 'Stamped' in t: s.create_subscription(TwistStamped,'/turtlebot4/cmd_vel',lambda m:setattr(s,'v',m.twist.linear.x),10)
        else: s.create_subscription(Twist,'/turtlebot4/cmd_vel',lambda m:setattr(s,'v',m.linear.x),10)
        s.create_timer(1.0,s.tick)
    def robot(s):
        for f in ([s.frame] if s.frame else FRAMES):
            try:
                t=s.tf.lookup_transform('map',f,rclpy.time.Time()).transform.translation
                if s.frame is None: s.frame=f; s.get_logger().info(f'base frame: {f}')
                return t.x,t.y
            except Exception: pass
        return None
    def cloud(s,m):
        r=s.robot(); pts=list(pc2.read_points(m,field_names=('x','y'),skip_nans=True)); s.npts=len(pts)
        if r and pts:
            s.dmin=min(math.hypot(p[0]-r[0],p[1]-r[1]) for p in pts); s.run_min=min(s.run_min,s.dmin)
        elif not pts: s.dmin=float('nan')
    def plan(s,m):
        p=[(q.pose.position.x,q.pose.position.y) for q in m.poses]
        if len(p)<3: s.bend=0.0; return
        (x0,y0),(x1,y1)=p[0],p[-1]; L=math.hypot(x1-x0,y1-y0)
        if L<1e-3: s.bend=0.0; return
        s.bend=max(abs((x1-x0)*(y0-y)-(x0-x)*(y1-y0))/L for x,y in p)
    def tick(s):
        print(f'v={s.v:5.2f} m/s  dist_cloud={s.dmin:5.2f} m (run min {s.run_min:5.2f})  cloud_pts={s.npts:4d}  plan_bend={s.bend:5.2f} m',flush=True)
rclpy.init(); rclpy.spin(D())
