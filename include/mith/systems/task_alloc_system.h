#pragma once

// TaskAllocSystem — see ARCHITECTURE.md §5.3, §16 v0.2
//
// Distributed role allocation via a response-threshold model
// (Bonabeau-style, deterministic variant). Each tick:
//
//   1. Count how many neighbours currently hold each role (the NeighbourTable
//      carries each peer's RoleComponent from their last beacon).
//   2. For each role r, compute deficit_r = desired_ratio[r] * known_swarm_size
//      minus the observed count.
//   3. The role's stimulus is s_r = max(0, deficit_r) * stimulus_gain.
//   4. The robot's threshold theta_r for role r is set by Params.
//   5. If any role's stimulus exceeds theta_r AND the stability window has
//      elapsed since the last role switch, switch the self-entity's role
//      to the role with the largest (s_r - theta_r) margin.
//
// Pre-partition-merge per the §16 v0.2 entry: no reconciliation of stale
// role state after a network partition heals — that's deferred to v0.3+
// once clock sync (§16 v0.3 line 1395) lets us define an "epoch" for
// version-vector merge.
//
// Deterministic by design: no randomness, no `std::random_device` call.
// Sequential-mode SimBus + this system gives bit-identical traces.

#include "mith/core/system.h"
#include "mith/core/builtin_components.h"

#include <array>

namespace mith {

class World;
class NeighbourTable;

class TaskAllocSystem : public System {
public:
    // Maximum number of distinct roles the system tracks. Anything beyond
    // is invisible to the threshold logic — the mission can still use
    // those role IDs, but TaskAllocSystem won't rebalance toward them.
    static constexpr std::size_t MAX_TRACKED_ROLES = 8;

    struct Params {
        // For each role 0..MAX_TRACKED_ROLES-1, the target fraction of the
        // observed swarm (self + neighbours) that should hold this role.
        // Sum need not equal 1.0 — un-rebalanced "free" roles can exist
        // by leaving their ratio at 0.
        std::array<float, MAX_TRACKED_ROLES> desired_ratios{};

        // Per-role threshold theta. Smaller theta → more eager to take
        // this role. Default 1.0 (moderate eagerness).
        std::array<float, MAX_TRACKED_ROLES> thresholds{};

        // Minimum time between role switches. Prevents thrashing under
        // noisy neighbour counts.
        float stability_window_s = 1.0f;

        // Linear gain applied to the deficit before threshold comparison.
        // Larger → more aggressive rebalancing.
        float stimulus_gain = 1.0f;

        Params() noexcept {
            thresholds.fill(1.0f);
        }
    };

    explicit TaskAllocSystem(World& world) noexcept;
    TaskAllocSystem(World& world, Params params) noexcept;

    SystemDescriptor describe() const override;
    void tick(EntityRegistry& registry,
              const SwarmContext& ctx,
              float delta_time) override;

    // Tick-side observability for tests + traces.
    std::uint64_t total_role_switches() const noexcept { return total_switches_; }
    float         time_since_last_switch_s() const noexcept { return time_since_last_switch_s_; }

private:
    NeighbourTable* neighbour_table_;
    Params          params_;
    float           time_since_last_switch_s_ = 0.0f;
    std::uint64_t   total_switches_           = 0;
};

} // namespace mith
