#include <cmath>
#include <stdexcept>
#include "api_shim.hpp"

#include "solver.h"   // Solver, Rigid, etc.
#include "maths.h"    // float2, float3, etc.

#include <vector>

static inline void rot2(double c, double s, double x, double y, double& ox, double& oy) {
    ox = c*x - s*y;
    oy = s*x + c*y;
}

struct AvbdWorld::Impl {
    Solver world;                   // from solver.h
    std::vector<Rigid*> order;      // keep creation order for stable indexing
};

AvbdWorld::AvbdWorld(const AvbdWorldConfig& cfg) : p_(new Impl()) {
    // Set reasonable defaults first
    p_->world.defaultParams();

    // Map config -> solver fields
    p_->world.gravity    = static_cast<float>(cfg.gy);   // acts along +Y; negative means downward
    p_->world.iterations = cfg.iterations;

    // If you later expose alpha/beta/gamma/postStabilize:
    // p_->world.alpha = ...; p_->world.beta = ...; p_->world.gamma = ...; p_->world.postStabilize = true/false;
}

AvbdWorld::~AvbdWorld() = default;

int AvbdWorld::add_box(double cx, double cy, double w, double h,
                       double density, bool fixed, double friction) {
    // In this codebase, a Rigid with mass <= 0 is treated as static/kinematic in the step loop
    const float dens = fixed ? 0.0f : static_cast<float>(density);

    float2 size = float2{ static_cast<float>(w), static_cast<float>(h) };
    float3 pos  = float3{ static_cast<float>(cx), static_cast<float>(cy), 0.0f };
    float3 vel  = float3{ 0.0f, 0.0f, 0.0f };

    // IMPORTANT: match constructor order exactly as in solver.h
    Rigid* body = new Rigid(&p_->world, size, dens, static_cast<float>(friction), pos, vel);

    p_->order.push_back(body);
    return static_cast<int>(p_->order.size() - 1);
}

int AvbdWorld::add_trapezoid(double cx, double cy,
                             double top_w, double bottom_w, double h,
                             double density, bool fixed, double friction) {
    // We'll wire this after polygon support lands in the core:
    throw std::runtime_error("add_trapezoid not implemented yet in core.");
}

void AvbdWorld::step(double dt) {
    // Solver::step() uses its internal dt
    p_->world.dt = static_cast<float>(dt);
    p_->world.step();
}

std::vector<AvbdBodyState> AvbdWorld::get_states() const {
    std::vector<AvbdBodyState> out;
    out.reserve(p_->order.size());
    for (const Rigid* b : p_->order) {
        // position.x/y = translation, position.z = rotation (radians)
        out.push_back( AvbdBodyState{
            static_cast<double>(b->position.x),
            static_cast<double>(b->position.y),
            static_cast<double>(b->position.z),
            static_cast<double>(b->velocity.x),
            static_cast<double>(b->velocity.y),
            static_cast<double>(b->velocity.z)     // omega
        });
    }
    return out;
}

std::vector<std::vector<AvbdPt>> AvbdWorld::get_world_vertices() const {
    std::vector<std::vector<AvbdPt>> polys;
    polys.reserve(p_->order.size());
    for (const Rigid* b : p_->order) {
        // Current demo uses rectangles of size (w,h) in local coords
        double w = b->size.x;
        double h = b->size.y;
        double hx = 0.5 * w, hy = 0.5 * h;

        // local rect CCW
        double lx[4] = {-hx, +hx, +hx, -hx};
        double ly[4] = {-hy, -hy, +hy, +hy};

        double c = std::cos(b->position.z);
        double s = std::sin(b->position.z);

        std::vector<AvbdPt> poly;
        poly.reserve(4);
        for (int i=0;i<4;++i) {
            double wx, wy;
            rot2(c, s, lx[i], ly[i], wx, wy);
            poly.push_back({ b->position.x + wx, b->position.y + wy });
        }
        polys.push_back(std::move(poly));
    }
    return polys;
}
