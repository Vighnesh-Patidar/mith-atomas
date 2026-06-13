#pragma once

// FlockingSystem — see ARCHITECTURE.md §5.3
//
// Reynolds flocking rules computed against the NeighbourTable:
//
//   Separation  Steer away from neighbours within separation_radius_m.
//               Inverse-distance weighting — closer neighbours push harder.
//   Alignment   Steer toward the average velocity of all neighbours.
//   Cohesion    Steer toward the average position of all neighbours.
//
// Per tick: read self Position + Velocity, iterate NeighbourTable, compute
// the three weighted steering vectors, integrate into Velocity (v += a·dt),
// clamp magnitude to max_speed_mps.
//
// Does NOT integrate position — that's a separate KinematicsSystem (lands
// alongside the flocking demo).

#include "mith/core/system.h"

namespace mith {

class World;
class NeighbourTable;

class FlockingSystem : public System {
public:
    struct Params {
        float separation_radius_m  = 5.0f;
        float separation_weight    = 1.5f;
        float alignment_weight     = 1.0f;
        float cohesion_weight      = 1.0f;
        float max_speed_mps        = 5.0f;
    };

    explicit FlockingSystem(World& world) noexcept;
    FlockingSystem(World& world, Params params) noexcept;

    SystemDescriptor describe() const override;
    void tick(EntityRegistry& registry,
              const SwarmContext& ctx,
              float delta_time) override;

    const Params& params() const noexcept { return params_; }

private:
    NeighbourTable* nt_;
    Params          params_;
};

} // namespace mith
