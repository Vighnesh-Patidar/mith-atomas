#pragma once

// SwarmContext — see ARCHITECTURE.md §6.2
//
// Per-tick read-only context passed to every System::tick() and every
// ActionProvider::evaluate(). Carries the swarm membership, current
// virtual time, and the tick counter. Constructed by the runtime from
// World state immediately before dispatching the tick.

#include "mith/identity/hierarchical_id.h"   // SwarmID

#include <cstddef>

namespace mith {

struct SwarmContext {
    SwarmID      swarm_id        = 0;
    float        elapsed_time_s  = 0.0f;
    float        delta_time_s    = 0.0f;
    std::size_t  tick_count      = 0;
};

} // namespace mith
