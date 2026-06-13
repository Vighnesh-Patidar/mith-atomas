#pragma once

// SystemScheduler — see ARCHITECTURE.md §5.2
//
// Two execution modes, selected per World via WorldConfig::scheduler_mode (§8):
//
//   Parallel (default in production)
//     Thread pool dispatches independent systems concurrently per the §5.1
//     two-axis hazard graph. Determinism NOT guaranteed (thread completion
//     order, FP-reduction order across cores).
//     [v0.1 first slice: NOT YET IMPLEMENTED. Calling tick() aborts.]
//
//   Sequential (default in sim — SimBus sets it)
//     Single-threaded execution in topological order with stable tie-breaking
//     on SystemDescriptor::name. Fully deterministic given fixed inputs.
//     [v0.1 first slice: implemented. Lexicographic order on names is the
//     current rule — hazards are documentation only in Sequential mode
//     because no concurrency exists to race against.]
//
// build_graph() must be called after all systems are registered and before
// the first tick(). Registering a system after build_graph() invalidates the
// built state; the caller must rebuild before ticking again.

#include "mith/core/system.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace mith {

class EntityRegistry;   // fwd — defined in registry.h

enum class SchedulerMode : std::uint8_t {
    Parallel    = 0,   // v0.1 first slice: not yet implemented (tick aborts)
    Sequential  = 1,   // default for SystemScheduler; SimBus also defaults to this
};

enum class SchedulerStatus : std::uint8_t {
    Ok            = 0,
    DuplicateName = 1,   // register_system: another system already has this name
    AlreadyBuilt  = 2,   // build_graph: already built; rebuild not needed
};

namespace detail {

[[noreturn]] void scheduler_assert_fail(const char* msg) noexcept;

} // namespace detail

class SystemScheduler {
public:
    explicit SystemScheduler(SchedulerMode mode = SchedulerMode::Sequential) noexcept;

    // Register a system. Ownership transferred. The system's name (from its
    // SystemDescriptor) must be unique within this scheduler.
    //
    // Calling after build_graph() succeeds but invalidates the built state —
    // the caller must call build_graph() again before tick().
    SchedulerStatus register_system(std::unique_ptr<System> system);

    // Resolve the execution order from registered systems.
    //   Sequential mode: lexicographic sort on SystemDescriptor::name.
    //   Parallel mode (deferred): would build the §5.1 hazard DAG here.
    // Idempotent: a second call without intervening register_system() returns
    // AlreadyBuilt.
    SchedulerStatus build_graph();

    // Execute one tick: dispatch all registered systems exactly once.
    // Aborts if build_graph() has not been called since the last
    // register_system(). Aborts in Parallel mode (not yet implemented).
    void tick(EntityRegistry& registry, const SwarmContext& ctx, float delta_time);

    SchedulerMode mode()         const noexcept;
    std::size_t   system_count() const noexcept;
    bool          is_built()     const noexcept;

private:
    SchedulerMode                         mode_;
    std::vector<std::unique_ptr<System>>  systems_;
    std::vector<std::size_t>              order_;     // indices into systems_
    bool                                  built_ = false;
};

} // namespace mith
