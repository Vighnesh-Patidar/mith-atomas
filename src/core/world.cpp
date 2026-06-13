#include "mith/core/world.h"

#include "mith/comms/transport.h"
#include "mith/core/builtin_components.h"
#include "mith/core/json_writer.h"
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
    , scheduler_(config.scheduler_mode, config.thread_pool_size) {
    context_.swarm_id = config_.swarm_id;
}

World::World(WorldConfig config, std::unique_ptr<TransportLayer> transport) noexcept
    : config_(config)
    , registry_(config.registration_policy)
    , scheduler_(config.scheduler_mode, config.thread_pool_size)
    , transport_(std::move(transport)) {
    context_.swarm_id = config_.swarm_id;
}

World::~World() = default;

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

TransportLayer*        World::transport()       noexcept { return transport_.get(); }
const TransportLayer*  World::transport() const noexcept { return transport_.get(); }

NeighbourTable&        World::neighbour_table()       noexcept { return neighbour_table_; }
const NeighbourTable&  World::neighbour_table() const noexcept { return neighbour_table_; }

void World::report_fault() noexcept {
    fault_count_.fetch_add(1, std::memory_order_relaxed);
}

std::uint64_t World::fault_count() const noexcept {
    return fault_count_.load(std::memory_order_relaxed);
}

SchedulerStatus World::register_system(std::unique_ptr<System> system) {
    return scheduler_.register_system(std::move(system));
}

void World::set_trace_sink(TraceSink* sink) noexcept {
    registry_.set_trace_sink(sink);
    scheduler_.set_trace_sink(sink);
}

void World::rotate_identity() noexcept {
    // v0.1 stub. Effective policies (PER_MISSION, PERIODIC, EVENT_DRIVEN)
    // land in v0.2 alongside cryptographic identity (§3.3, §3.4). Under
    // PERMANENT — the only policy honoured in v0.1 — this is a no-op
    // by design.
    //
    // The hook exists now so API consumers can write the EVENT_DRIVEN
    // call shape today; it will become functional in v0.2 without
    // breaking the call site.
}

std::string World::dump_state() const {
    JsonWriter w;
    w.begin_object();

    // Top-level swarm / clock state.
    w.key("swarm_id");       w.write_u64(static_cast<std::uint64_t>(config_.swarm_id));
    w.key("tick");           w.write_u64(static_cast<std::uint64_t>(context_.tick_count));
    w.key("elapsed_time_s"); w.write_f64(static_cast<double>(context_.elapsed_time_s));
    w.key("initialized");    w.write_bool(initialized_);

    // Self entity — only populated post-init.
    if (initialized_) {
        w.key("self");
        w.begin_object();

        w.key("id"); w.write_string(identity().to_string());

        const auto& pos = registry_.get<PositionComponent>(self_id());
        w.key("position");
        w.begin_object();
        w.key("x"); w.write_f64(static_cast<double>(pos.x));
        w.key("y"); w.write_f64(static_cast<double>(pos.y));
        w.key("z"); w.write_f64(static_cast<double>(pos.z));
        w.end_object();

        const auto& vel = registry_.get<VelocityComponent>(self_id());
        w.key("velocity");
        w.begin_object();
        w.key("vx"); w.write_f64(static_cast<double>(vel.vx));
        w.key("vy"); w.write_f64(static_cast<double>(vel.vy));
        w.key("vz"); w.write_f64(static_cast<double>(vel.vz));
        w.end_object();

        w.key("health"); w.write_u64(registry_.get<HealthComponent>(self_id()).value);
        w.key("role");   w.write_u64(registry_.get<RoleComponent>(self_id()).role);
        w.key("state");  w.write_u64(registry_.get<BehaviourStateComponent>(self_id()).state);

        const auto& aq = registry_.get<ActionQueueComponent>(self_id());
        w.key("action_queue_size");        w.write_u64(static_cast<std::uint64_t>(aq.queue.size()));
        w.key("action_queue_dropped");     w.write_u64(aq.queue.dropped_count());
        w.key("permission_rejections");    w.write_u64(aq.permission_rejections_total);

        const auto& cb = registry_.get<CommBufferComponent>(self_id());
        w.key("comm_buffer_size");    w.write_u64(static_cast<std::uint64_t>(cb.queue.size()));
        w.key("comm_buffer_dropped"); w.write_u64(cb.queue.dropped_count());

        w.end_object();
    }

    // NeighbourTable.
    w.key("neighbour_count"); w.write_u64(static_cast<std::uint64_t>(neighbour_table_.count()));
    w.key("neighbour_observations"); w.write_u64(neighbour_table_.total_observations());
    w.key("neighbour_evictions");    w.write_u64(neighbour_table_.total_evictions());
    w.key("neighbours");
    w.begin_array();
    for (const auto& n : neighbour_table_) {
        w.begin_object();
        w.key("id");          w.write_string(n.hid.to_string());
        w.key("last_seen_s"); w.write_f64(static_cast<double>(n.last_seen_s));
        w.key("position");
        w.begin_object();
        w.key("x"); w.write_f64(static_cast<double>(n.position.x));
        w.key("y"); w.write_f64(static_cast<double>(n.position.y));
        w.key("z"); w.write_f64(static_cast<double>(n.position.z));
        w.end_object();
        w.end_object();
    }
    w.end_array();

    // Scheduler tick timings.
    w.key("scheduler");
    w.begin_object();
    w.key("system_count"); w.write_u64(static_cast<std::uint64_t>(scheduler_.system_count()));
    w.key("last_tick_timings");
    w.begin_array();
    for (const auto& t : scheduler_.last_tick_timings()) {
        w.begin_object();
        w.key("name");        w.write_string(t.name);
        w.key("start_us");    w.write_f64(t.start_us);
        w.key("duration_us"); w.write_f64(t.duration_us);
        w.key("thread_id");   w.write_u64(t.thread_id);
        w.end_object();
    }
    w.end_array();
    w.end_object();

    w.end_object();
    return w.take();
}

} // namespace mith
