#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
T=['/person_positions_map','/lidar_person_clusters','/person_positions_fused','/predicted_person_positions']
class G(Node):
    def __init__(s):
        super().__init__('gap_diag'); s.last={}; s.t0=time.monotonic()
        for t in T: s.create_subscription(String,t,lambda m,t=t:s.cb(t),10)
        s.create_timer(0.5,s.tick)
    def cb(s,t):
        n=time.monotonic()
        if t in s.last and n-s.last[t]>0.4: print(f'{n-s.t0:7.1f}s  GAP {n-s.last[t]:5.2f}s on {t}',flush=True)
        s.last[t]=n
    def tick(s):
        n=time.monotonic()
        print(f'{n-s.t0:7.1f}s  '+'  '.join(f'{t.split("/")[-1][:9]}={n-s.last[t]:4.1f}' if t in s.last else f'{t.split("/")[-1][:9]}= --' for t in T),flush=True)
rclpy.init(); rclpy.spin(G())
