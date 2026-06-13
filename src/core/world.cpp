#include "mith/core/world.h"

#include "mith/core/builtin_components.h"
#include "mith/core/trace_sink.h"
#include "mith/identity/hierarchical_id.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace mith {

namespace detail {

[[noreturn]] static void world_assert_fail(const char* msg) noexcept {
    std::fprintf(stderr, "mith::World assertion failed: %s\n", msg);
    std::abort();
}

} // namespace detail

World::World(WorldConfig config) noexcept
    : config_(config)
    , registry_(config.registration_policy)
    , scheduler_(config.scheduler_mode) {
    context_.swarm_id = config_.swarm_id;
}

void World::init() {
    if (initialized_) {
        detail::world_assert_fail("init(): called twice");
    }

    // Register the §4.4 built-in components through the privileged path.
    // The status return is intentionally ignored — these are first-time
    // registrations and cannot fail at this point.
    (void) registry_.register_builtin_component<IdentityComponent>();
    (void) registry_.register_builtin_component<PositionComponent>();
    (void) registry_.register_builtin_component<VelocityComponent>();
    (void) registry_.register_builtin_component<OrientationComponent>();
    (void) registry_.register_builtin_component<HealthComponent>();
    (void) registry_.register_builtin_component<RoleComponent>();
    (void) registry_.register_builtin_component<BehaviourStateComponent>();
    (void) registry_.register_builtin_component<PermissionMaskComponent>();
    (void) registry_.register_builtin_component<ActionQueueComponent>();
    (void) registry_.register_builtin_component<CommBufferComponent>();

    // Emplace defaults on the self entity. IdentityComponent gets a freshly
    // generated UUID v4 for the configured swarm; all other built-ins use
    // their default values (origin position, identity quaternion, full
    // health, no role, no state, permissive mask, empty queues).
    const EntityID self = registry_.self_id();
    registry_.emplace<IdentityComponent>(self,
        IdentityComponent{HierarchicalID::generate(config_.swarm_id)});
    registry_.emplace<PositionComponent>      (self, PositionComponent{});
    registry_.emplace<VelocityComponent>      (self, VelocityComponent{});
    registry_.emplace<OrientationComponent>   (self, OrientationComponent{});
    registry_.emplace<HealthComponent>        (self, HealthComponent{});
    registry_.emplace<RoleComponent>          (self, RoleComponent{});
    registry_.emplace<BehaviourStateComponent>(self, BehaviourStateComponent{});
    registry_.emplace<PermissionMaskComponent>(self, PermissionMaskComponent{});
    registry_.emplace<ActionQueueComponent>   (self, ActionQueueComponent{});
    registry_.emplace<CommBufferComponent>    (self, CommBufferComponent{});

    // No more component registrations from here on.
    registry_.lock();

    // Build the scheduler graph. In Sequential mode this is a lexicographic
    // sort of system names; in Parallel mode (deferred) it would also build
    // the hazard DAG.
    (void) scheduler_.build_graph();

    initialized_ = true;
}

void World::tick() {
    if (!initialized_) {
        detail::world_assert_fail("tick(): init() not called");
    }

    // Update the swarm context for this tick.
    context_.delta_time_s   = 1.0f / config_.tick_rate_hz;
    context_.elapsed_time_s += context_.delta_time_s;
    ++context_.tick_count;

    scheduler_.tick(registry_, context_, context_.delta_time_s);
}

void World::run(std::atomic<bool>& stop_flag) {
    if (!initialized_) {
        detail::world_assert_fail("run(): init() not called");
    }

    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / config_.tick_rate_hz));

    auto next_tick = clock::now();
    while (!stop_flag.load(std::memory_order_relaxed)) {
        tick();
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }
}

bool World::is_initialized() const noexcept { return initialized_; }

EntityID              World::self_id()  const noexcept { return registry_.self_id(); }
const SwarmContext&   World::context()  const noexcept { return context_; }
const WorldConfig&    World::config()   const noexcept { return config_; }

const HierarchicalID& World::identity() const noexcept {
    // Tied to IdentityComponent on the self entity — populated by init().
    // Caller must have called init() before reading.
    return registry_.get<IdentityComponent>(registry_.self_id()).id;
}

EntityRegistry&        World::registry()       noexcept { return registry_; }
const EntityRegistry&  World::registry() const noexcept { return registry_; }

SystemScheduler&       World::scheduler()       noexcept { return scheduler_; }
const SystemScheduler& World::scheduler() const noexcept { return scheduler_; }

SchedulerStatus World::register_system(std::unique_ptr<System> system) {
    return scheduler_.register_system(std::move(system));
}

void World::set_trace_sink(TraceSink* sink) noexcept {
    registry_.set_trace_sink(sink);
    scheduler_.set_trace_sink(sink);
}

} // namespace mith
