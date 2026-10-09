import sys, rosbag2_py, math
sys.path.insert(0,'/tmp')
from simhead import heading_stats
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
H=math.radians(141); SX,SY=-6.80,13.50
for b in sys.argv[1:]:
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b+'/bag',storage_id=''),rosbag2_py.ConverterOptions('',''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/tf','/turtlebot4/odom']))
    m2o=None; R=[]
    while r.has_next():
        t,d,ts=r.read_next()
        if t=='/turtlebot4/tf':
            for tr in deserialize_message(d,TFMessage).transforms:
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    q=tr.transform.rotation; m2o=(tr.transform.translation.x,tr.transform.translation.y,2*math.atan2(q.z,q.w))
        elif m2o:
            m=deserialize_message(d,Odometry); p=m.pose.pose.position
            ox,oy,oth=m2o; x=ox+p.x*math.cos(oth)-p.y*math.sin(oth); y=oy+p.x*math.sin(oth)+p.y*math.cos(oth)
            dx,dy=x-SX,y-SY; s=dx*math.cos(H)+dy*math.sin(H); l=-dx*math.sin(H)+dy*math.cos(H)
            if s<5.6:  # robot's own start-to-goal leg only (goal at ~6.9 m)
                R.append({'t':ts*1e-9,'x':s,'y':l,'wz':m.twist.twist.angular.z})
    (h,y),w,t1=heading_stats(R)
    print(f'{b.split("/")[-1][:38]:38s} {h:+6.0f} deg   {w:4.2f}     {y:+5.2f} m       {t1:4.1f} s')
