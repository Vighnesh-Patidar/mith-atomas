#pragma once

// KinematicsSystem — integrates position from velocity per tick.
//
// pos += vel * dt
//
// Runs after FlockingSystem in Sequential mode by lexicographic order
// (F < K), so the new velocity computed this tick is the one that moves
// the robot. Default-constructible — no World reference needed.

#include "mith/core/system.h"

namespace mith {

class KinematicsSystem : public System {
public:
    KinematicsSystem() noexcept = default;

    SystemDescriptor describe() const override;
    void tick(EntityRegistry& registry,
              const SwarmContext& ctx,
              float delta_time) override;
};

} // namespace mith
