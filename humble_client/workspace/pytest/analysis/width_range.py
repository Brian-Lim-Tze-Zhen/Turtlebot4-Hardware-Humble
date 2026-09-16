import re, sys, bisect, statistics
yolo = {}   # cam -> [(t, width)]
rays = []   # (t, cam, range)
for line in open(sys.argv[1]):
    m = re.search(r"\[(\d+\.\d+)\] \[yolo_bytetrack_position_node\]: id:(\d+) conf:[0-9.]+ bbox=\((\d+),\d+,(\d+),\d+\)", line)
    if m:
        t, c, x1, x2 = float(m[1]), int(m[2]), int(m[3]), int(m[4])
        if x1 > 2 and x2 < 248:
            yolo.setdefault(c, []).append((t, x2 - x1))
        continue
    m = re.search(r"\[(\d+\.\d+)\] \[camera_ray_person_node\]: ray cam:(\d+) .*@ ([0-9.]+) m", line)
    if m:
        rays.append((float(m[1]), int(m[2]), float(m[3])))
k = []
for t, c, r in rays:
    ys = yolo.get(c, [])
    ts = [y[0] for y in ys]
    i = bisect.bisect_right(ts, t) - 1
    if i >= 0 and t - ts[i] < 0.2 and r < 6:
        k.append(r * ys[i][1])
if not k:
    print("no pairs"); sys.exit()
k.sort()
print(f"pairs {len(k)}  range*width_px: median {statistics.median(k):.0f}  "
      f"p10 {k[len(k)//10]:.0f}  p90 {k[9*len(k)//10]:.0f}")
