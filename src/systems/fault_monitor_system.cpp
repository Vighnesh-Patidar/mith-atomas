#include "mith/systems/fault_monitor_system.h"

#include "mith/behaviour/action_type.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"

namespace mith {

FaultMonitorSystem::FaultMonitorSystem(World& world) noexcept
    : FaultMonitorSystem(world, Params{}) {}

FaultMonitorSystem::FaultMonitorSystem(World& world, Params params) noexcept
    : params_(params)
    , world_(&world) {}

SystemDescriptor FaultMonitorSystem::describe() const {
    return SystemDescriptor{
        /*name=*/             "FaultMonitorSystem",
        /*reads_components=*/ {component_id<HealthComponent>()},
        /*writes_components=*/{component_id<HealthComponent>(),
                                component_id<PermissionMaskComponent>(),
                                component_id<BehaviourStateComponent>()},
        /*reads_resources=*/  {},
        /*writes_resources=*/ {},
    };
}

void FaultMonitorSystem::tick(EntityRegistry& registry,
                              const SwarmContext& /*ctx*/,
                              float /*delta_time*/) {
    const EntityID self = registry.self_id();
    auto& health = registry.get<HealthComponent>(self);
    auto& mask   = registry.get<PermissionMaskComponent>(self);
    auto& state  = registry.get<BehaviourStateComponent>(self);

    // Process new faults reported since the last tick.
    const std::uint64_t now_count   = world_->fault_count();
    const std::uint64_t new_faults  = now_count - last_fault_count_;
    last_fault_count_ = now_count;

    if (new_faults > 0) {
        const std::uint64_t total_drop =
            new_faults * static_cast<std::uint64_t>(params_.decrement_per_fault);
        if (total_drop >= health.value) {
            health.value = 0;
        } else {
            health.value = static_cast<std::uint8_t>(health.value - total_drop);
        }
    }

    // Degraded transitions.
    if (!in_degraded_state_ && health.value < params_.degraded_threshold) {
        saved_allowed_builtins_   = mask.allowed_builtins;
        saved_allow_user_actions_ = mask.allow_user_actions;
        mask.allowed_builtins =
            (1u << actions::IDLE) | (1u << actions::REGROUP);
        mask.allow_user_actions = false;
        state.state = DEGRADED_STATE;
        in_degraded_state_ = true;
    } else if (in_degraded_state_ &&
               health.value >= static_cast<std::uint32_t>(params_.degraded_threshold)
                              + static_cast<std::uint32_t>(params_.degraded_hysteresis)) {
        mask.allowed_builtins   = saved_allowed_builtins_;
        mask.allow_user_actions = saved_allow_user_actions_;
        state.state = NORMAL_STATE;
        in_degraded_state_ = false;
    }
}

} // namespace mith
