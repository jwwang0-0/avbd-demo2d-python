import avbd2d as av
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.patches import Polygon
import time

cfg = av.WorldConfig(); cfg.gy = -9.81; cfg.iterations = 20
w = av.World(cfg)
w.add_box(0,-0.5,50,1, density=0.0, fixed=True)  # ground
for i in range(5):
    w.add_box(0, 1.5+i*1.05, 1,1, density=1.0, fixed=False)

# Run & capture frames
frames = []
start_time = time.time()
for _ in range(300):
    w.step(1/60)
    frames.append(w.get_world_vertices())
used_time = time.time() - start_time

print(f"Used Time is {used_time}")
# Animate
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-6, 6); ax.set_ylim(-2, 8)
patches = []
for _ in frames[0]:
    p = Polygon([(0,0)], closed=True, fill=False)
    ax.add_patch(p); patches.append(p)

def update(k):
    polys = frames[k]
    # grow patch list if new bodies appear
    while len(patches) < len(polys):
        p = Polygon([(0,0)], closed=True, fill=False); ax.add_patch(p); patches.append(p)
    for p, poly in zip(patches, polys):
        p.set_xy([(pt.x, pt.y) for pt in poly])
    return patches

ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=16, blit=True)
plt.show()
