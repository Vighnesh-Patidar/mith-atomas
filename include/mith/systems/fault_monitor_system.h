#pragma once

// FaultMonitorSystem — see ARCHITECTURE.md §13.1, §13.2
//
// Activates the dormant §13 mechanisms documented in v0.1 but unwired
// until now. Per tick:
//   1. Reads World::fault_count() — delta since last tick is the new
//      faults reported by transports / user code / other systems.
//   2. Decrements HealthComponent::value by decrement_per_fault * new_faults
//      (saturating at 0).
//   3. State transitions per §13.2:
//        - Entering degraded: health < degraded_threshold → snapshot the
//          current PermissionMaskComponent, write the restricted mask
//          (IDLE | REGROUP only, allow_user_actions = false), set
//          BehaviourStateComponent::state to DEGRADED_STATE.
//        - Recovering: health >= degraded_threshold + degraded_hysteresis
//          → restore the snapshotted mask, clear DEGRADED_STATE.
//      Hysteresis prevents thrashing near the threshold.
//
// Not yet wired:
//   - Missed-beacon detection (requires an "expected peers" concept that
//     v0.2's discovery / bootstrap protocol will introduce).
//   - Per-queue FaultTrigger reads (architectural hook exists but no
//     built-in queue is FaultTrigger; users can plumb their own via
//     report_fault() until the hook lands).

#include "mith/core/system.h"

#include <cstdint>

namespace mith {

class World;

class FaultMonitorSystem : public System {
public:
    static constexpr std::uint32_t NORMAL_STATE   = 0;
    static constexpr std::uint32_t DEGRADED_STATE = 1;

    struct Params {
        std::uint8_t degraded_threshold  = 40;   // health below this → degrade
        std::uint8_t degraded_hysteresis = 10;   // health must rise this far above to recover
        std::uint8_t decrement_per_fault = 5;    // health drop per reported fault
    };

    explicit FaultMonitorSystem(World& world) noexcept;
    FaultMonitorSystem(World& world, Params params) noexcept;

    SystemDescriptor describe() const override;
    void tick(EntityRegistry& registry,
              const SwarmContext& ctx,
              float delta_time) override;

    const Params& params()            const noexcept { return params_; }
    bool          in_degraded_state() const noexcept { return in_degraded_state_; }

private:
    Params          params_;
    World*          world_;
    std::uint64_t   last_fault_count_         = 0;
    std::uint32_t   saved_allowed_builtins_   = 0xFFFFFFFFu;
    bool            saved_allow_user_actions_ = true;
    bool            in_degraded_state_        = false;
};

} // namespace mith
