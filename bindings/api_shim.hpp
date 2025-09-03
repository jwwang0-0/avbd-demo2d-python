#pragma once
#include <vector>
#include <memory>

struct AvbdWorldConfig {
    double gx{0.0};      // not used by this solver (gravity is along Y)
    double gy{-9.81};    // mapped to Solver::gravity (acts on +Y; negative = downward)
    int    iterations{20};
    // (You can expose alpha/beta/gamma/postStabilize later if desired)
};

struct AvbdBodyState {
    double x, y, theta;  // theta stored in Rigid::position.z (radians)
    double vx, vy, omega;  // velocity and angular velocity
};

class AvbdWorld {
public:
    explicit AvbdWorld(const AvbdWorldConfig& cfg);
    ~AvbdWorld();

    // NOTE: Keep this signature in sync with api_shim.cpp and avbd_py.cpp
    int add_box(double cx, double cy, double w, double h,
                double density = 1.0, bool fixed = false,
                double friction = 0.6);

    void step(double dt);

    std::vector<AvbdBodyState> get_states() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};
