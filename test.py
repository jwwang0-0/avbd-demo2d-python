import avbd2d as av

cfg = av.WorldConfig()
cfg.gy = -9.81
cfg.iterations = 20

w = av.World(cfg)
w.add_box(0, -0.5, 50, 1, density=1.0, fixed=True, friction=0.6)  # ground
w.add_box(0,  0.5,  1, 1, density=1.0, fixed=False, friction=0.6)

for _ in range(50):
    w.step(1/60)
    print([(b.x, b.y, b.theta, b.vx, b.vy, b.omega) for b in w.get_states()[:2]])

# print(w.get_states())
# print(len(w.get_states()), "bodies")
# print([(b.x, b.y, b.theta) for b in w.get_states()[:2]])
