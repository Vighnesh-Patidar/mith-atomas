#pragma once

// World — see ARCHITECTURE.md §8
//
// "Robot-as-runtime-process": registry + scheduler + swarm context +
// lifecycle, all wired together. One World per robot process (v0.1 N=1
// per §3.2). The single entity inside is the robot itself; its built-in
// hot components (§4.4) are registered + emplaced at init().
//
// Lifecycle:
//   1. Construct: World w(WorldConfig{...});
//   2. Optionally: register user systems / components / trace sink.
//   3. Call w.init() — registers the §4.4 built-ins, emplaces defaults
//      on the self entity, locks the registry, builds the scheduler
//      graph. Idempotent calls abort.
//   4. Drive ticks: w.tick() each frame, or w.run(stop_flag) for a
//      blocking loop at WorldConfig::tick_rate_hz.
//
// What's NOT yet in this v0.1 first slice (depends on later work):
//   - TransportLayer constructor argument (lands with SimTransport)
//   - set_action_provider() (lands with ActionProvider, which needs
//     NeighbourView from §7.4)
//   - neighbour_table() accessor (lands with NeighbourTable)
//   - create_entity() (degenerate at N=1; will be a stub returning
//     self_id() when added)

#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/scheduler.h"
#include "mith/core/swarm_context.h"
#include "mith/identity/hierarchical_id.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace mith {

class TraceSink;   // fwd

// Mirrors §8 WorldConfig. SwarmID 0 is reserved (broadcast / unset);
// production deployments use 1..0xFFFE.
struct WorldConfig {
    SwarmID  swarm_id            = 1;
    float    tick_rate_hz        = 20.0f;
    float    beacon_rate_hz      = 10.0f;
    float    neighbour_timeout_s = 0.5f;
    std::size_t thread_pool_size = 0;    // 0 = hardware_concurrency - 1 (Parallel mode)

    // Built-in-system toggles. Effective when those systems land; today
    // they're recorded for forward compat.
    bool enable_flocking      = true;
    bool enable_task_alloc    = true;
    bool enable_fault_monitor = true;

    ComponentRegistrationPolicy registration_policy =
        ComponentRegistrationPolicy::LockAfterInit;
    SchedulerMode scheduler_mode = SchedulerMode::Sequential;
};

class World {
public:
    explicit World(WorldConfig config) noexcept;

    // ---- Lifecycle ----

    // Register all §4.4 built-in components, emplace defaults on the self
    // entity (IdentityComponent gets a fresh HID for config.swarm_id),
    // lock the registry, build the scheduler graph. Aborts on a second
    // call.
    void init();

    // Single tick. Advances SwarmContext (tick_count, elapsed_time_s),
    // dispatches the scheduler. Aborts if init() has not been called.
    void tick();

    // Blocking loop at WorldConfig::tick_rate_hz. Uses
    // std::this_thread::sleep_until to maintain rate. Exits cleanly when
    // stop_flag becomes true. Uses std::atomic<bool> (not bare bool&) to
    // avoid the data race in the original §8 spec.
    void run(std::atomic<bool>& stop_flag);

    bool is_initialized() const noexcept;

    // ---- Accessors ----

    EntityID                self_id()  const noexcept;
    const HierarchicalID&   identity() const noexcept;
    const SwarmContext&     context()  const noexcept;
    const WorldConfig&      config()   const noexcept;

    EntityRegistry&         registry() noexcept;
    const EntityRegistry&   registry() const noexcept;

    SystemScheduler&        scheduler() noexcept;
    const SystemScheduler&  scheduler() const noexcept;

    // ---- Convenience forwards ----

    // Forwards to scheduler.register_system(). For consistent results,
    // call before init(); registering after init invalidates the built
    // scheduler graph and the next tick will abort.
    SchedulerStatus register_system(std::unique_ptr<System> system);

    // Forwards to registry.register_component<T>() — User origin.
    template<typename T>
    RegistrationStatus register_component() {
        return registry_.register_component<T>();
    }

    // Forwards to both registry and scheduler. The same sink receives
    // component_registered (from registry) and tick_completed (from
    // scheduler) events. Pass nullptr to clear.
    void set_trace_sink(TraceSink* sink) noexcept;

private:
    WorldConfig      config_;
    EntityRegistry   registry_;
    SystemScheduler  scheduler_;
    SwarmContext     context_{};
    bool             initialized_ = false;
};

} // namespace mith
