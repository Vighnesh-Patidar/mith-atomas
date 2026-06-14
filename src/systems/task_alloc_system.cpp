#include "mith/systems/task_alloc_system.h"

#include "mith/comms/neighbour_table.h"
#include "mith/core/registry.h"
#include "mith/core/system.h"
#include "mith/core/world.h"

#include <algorithm>

namespace mith {

TaskAllocSystem::TaskAllocSystem(World& world) noexcept
    : neighbour_table_(&world.neighbour_table())
    , params_(Params{}) {}

TaskAllocSystem::TaskAllocSystem(World& world, Params params) noexcept
    : neighbour_table_(&world.neighbour_table())
    , params_(params) {}

SystemDescriptor TaskAllocSystem::describe() const {
    return SystemDescriptor{
        /*name=*/             "TaskAllocSystem",
        /*reads_components=*/ {
            component_id<RoleComponent>(),
            component_id<BehaviourStateComponent>(),
        },
        /*writes_components=*/{
            component_id<RoleComponent>(),
        },
        /*reads_resources=*/  {ResourceID::NeighbourTable},
        /*writes_resources=*/ {},
    };
}

void TaskAllocSystem::tick(EntityRegistry& registry,
                            const SwarmContext& /*ctx*/,
                            float delta_time) {
    time_since_last_switch_s_ += delta_time;

    if (!neighbour_table_) return;

    auto& self_role = registry.get<RoleComponent>(registry.self_id());

    // 1. Count role distribution across self + neighbours.
    std::array<std::uint32_t, TaskAllocSystem::MAX_TRACKED_ROLES> counts{};
    std::uint32_t total = 1;   // self counts
    if (self_role.role < counts.size()) counts[self_role.role]++;

    for (const auto& entry : *neighbour_table_) {
        const RoleID r = entry.role.role;
        if (r < counts.size()) counts[r]++;
        ++total;
    }

    const float total_f = static_cast<float>(total);

    // 2 + 3. Compute stimulus per role.
    // 4. Compute margin = stimulus - threshold; track the largest positive
    //    margin across roles.
    RoleID best_role         = self_role.role;
    float  best_margin       = 0.0f;
    bool   candidate_exists  = false;

    for (std::size_t r = 0; r < params_.desired_ratios.size(); ++r) {
        const float ratio   = params_.desired_ratios[r];
        if (ratio <= 0.0f) continue;                       // role not managed

        const float desired_count = ratio * total_f;
        const float deficit       = desired_count - static_cast<float>(counts[r]);
        if (deficit <= 0.0f) continue;                     // role already saturated

        const float stimulus = deficit * params_.stimulus_gain;
        const float theta    = params_.thresholds[r];
        const float margin   = stimulus - theta;
        if (margin <= 0.0f) continue;                      // below threshold

        if (!candidate_exists || margin > best_margin) {
            candidate_exists = true;
            best_margin      = margin;
            best_role        = static_cast<RoleID>(r);
        }
    }

    // 5. Switch role if a strictly better candidate exists, stability
    //    window has elapsed, and the candidate is not already our role.
    if (candidate_exists
        && best_role != self_role.role
        && time_since_last_switch_s_ >= params_.stability_window_s) {
        self_role.role = best_role;
        time_since_last_switch_s_ = 0.0f;
        ++total_switches_;
    }
}

} // namespace mith
