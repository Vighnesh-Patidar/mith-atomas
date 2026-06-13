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

#include "mith/comms/neighbour_table.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/scheduler.h"
#include "mith/core/swarm_context.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_rotation.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace mith {

class TraceSink;       // fwd
class TransportLayer;  // fwd — full type in mith/comms/transport.h

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

    // Identity rotation policy per §3.4. v0.1 honours PERMANENT only;
    // the other values' enforcement lands in v0.2 alongside signed mode.
    IdentityRotationPolicy identity_rotation_policy =
        IdentityRotationPolicy::PERMANENT;
};

class World {
public:
    explicit World(WorldConfig config) noexcept;

    // Transport-taking ctor. The transport is held but not actively
    // exercised until BeaconSystem (§5.3, v0.2) lands and starts driving
    // it; in v0.1 it serves as the wiring point for sim and (eventually)
    // hardware transports.
    World(WorldConfig config, std::unique_ptr<TransportLayer> transport) noexcept;

    // Destructor declared here, defined in world.cpp where TransportLayer's
    // full type is visible. Required because std::unique_ptr<TransportLayer>'s
    // deleter needs sizeof(TransportLayer) at instantiation time.
    ~World();

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

    // The HID of the self entity. ONLY valid after init() — pre-init this
    // aborts via the registry (IdentityComponent not yet emplaced).
    const HierarchicalID&   identity() const noexcept;

    const SwarmContext&     context()  const noexcept;
    const WorldConfig&      config()   const noexcept;

    EntityRegistry&         registry() noexcept;
    const EntityRegistry&   registry() const noexcept;

    SystemScheduler&        scheduler() noexcept;
    const SystemScheduler&  scheduler() const noexcept;

    // Nullable — the transport-less ctor leaves this null. BeaconSystem
    // consumes this via send_beacon / send_message / poll.
    TransportLayer*         transport() noexcept;
    const TransportLayer*   transport() const noexcept;

    // NeighbourTable owned by this World. BeaconSystem populates it from
    // received StateVector beacons; FlockingSystem and TaskAllocSystem
    // read it.
    NeighbourTable&         neighbour_table() noexcept;
    const NeighbourTable&   neighbour_table() const noexcept;

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

    // Rotate the robot's identity (§3.4). v0.1 ships this as a no-op
    // stub — actual rotation under PER_MISSION / PERIODIC / EVENT_DRIVEN
    // policies lands in v0.2 alongside the Ed25519 / IdentityCertificate
    // work. Under PERMANENT (v0.1 default) this is a meaningful no-op
    // by design; under the other policies it will perform the rotation
    // dance once v0.2 wires it up.
    void rotate_identity() noexcept;

    // Serialise the local runtime state to JSON (§14.1). Includes self-
    // entity component summary, NeighbourTable contents, scheduler tick
    // timings, swarm context. Allocates a std::string — for debugging,
    // tests, and snapshot streaming, not the hot tick path.
    std::string dump_state() const;

    // Report a fault event. Transports (link-down, CRC failures), user
    // code (sensor timeouts, hardware faults), and built-in systems
    // call this; FaultMonitorSystem (§13.1) reads the cumulative count
    // delta each tick and decrements HealthComponent::value accordingly.
    // Thread-safe — atomic increment, suitable for the Parallel scheduler.
    void          report_fault() noexcept;

    // Cumulative fault count since World construction. Monotonic.
    std::uint64_t fault_count() const noexcept;

private:
    WorldConfig                       config_;
    EntityRegistry                    registry_;
    SystemScheduler                   scheduler_;
    SwarmContext                      context_{};
    std::unique_ptr<TransportLayer>   transport_;
    NeighbourTable                    neighbour_table_;
    std::atomic<std::uint64_t>        fault_count_{0};   // §13.1 fault counter
    bool                              initialized_ = false;
};

} // namespace mith
