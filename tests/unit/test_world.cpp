#include "doctest.h"
#include "test_helpers.h"

#include "mith/behaviour/action.h"
#include "mith/behaviour/action_type.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/scheduler.h"
#include "mith/core/system.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using mith::ComponentOrigin;
using mith::EntityRegistry;
using mith::RegistrationStatus;
using mith::SchedulerMode;
using mith::SchedulerStatus;
using mith::SwarmID;
using mith::SwarmContext;
using mith::System;
using mith::SystemDescriptor;
using mith::World;
using mith::WorldConfig;
using mith_test::JsonCapturingSink;
using mith_test::contains;

namespace {

class TaggingSystem : public System {
public:
    TaggingSystem(std::string name, std::vector<std::string>& log)
        : name_(std::move(name)), log_(log) {}
    SystemDescriptor describe() const override {
        return SystemDescriptor{name_, {}, {}, {}, {}};
    }
    void tick(EntityRegistry&, const SwarmContext&, float) override {
        log_.push_back(name_);
    }
private:
    std::string                name_;
    std::vector<std::string>&  log_;
};

} // namespace

TEST_CASE("default-constructed World is not initialized") {
    World w(WorldConfig{});
    CHECK_FALSE(w.is_initialized());
    CHECK(w.self_id() == mith::SELF_ENTITY);
    CHECK(w.context().swarm_id == 1u);   // WorldConfig default
    CHECK(w.context().tick_count == 0u);
    CHECK(w.context().elapsed_time_s == 0.0f);
    CHECK(w.registry().registered_count() == 0u);
    CHECK_FALSE(w.scheduler().is_built());
}

TEST_CASE("World forwards config swarm_id into SwarmContext") {
    WorldConfig cfg;
    cfg.swarm_id = 0xABCD;
    World w(cfg);
    CHECK(w.context().swarm_id == 0xABCDu);
    CHECK(w.config().swarm_id == 0xABCDu);
}

TEST_CASE("init() registers all 10 §4.4 built-in components") {
    World w(WorldConfig{});
    w.init();

    CHECK(w.is_initialized());
    CHECK(w.registry().registered_count() == 10u);
    CHECK(w.registry().is_locked());
    CHECK(w.scheduler().is_built());

    CHECK(w.registry().origin_of<mith::IdentityComponent>()        == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::PositionComponent>()        == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::VelocityComponent>()        == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::OrientationComponent>()     == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::HealthComponent>()          == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::RoleComponent>()            == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::BehaviourStateComponent>()  == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::PermissionMaskComponent>()  == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::ActionQueueComponent>()     == ComponentOrigin::Built_In);
    CHECK(w.registry().origin_of<mith::CommBufferComponent>()      == ComponentOrigin::Built_In);
}

TEST_CASE("init() emplaces all 10 built-ins with valid defaults on the self entity") {
    WorldConfig cfg;
    cfg.swarm_id = 42;
    World w(cfg);
    w.init();

    const auto self = w.self_id();
    REQUIRE(w.registry().has<mith::IdentityComponent>(self));
    REQUIRE(w.registry().has<mith::PositionComponent>(self));
    REQUIRE(w.registry().has<mith::VelocityComponent>(self));
    REQUIRE(w.registry().has<mith::OrientationComponent>(self));
    REQUIRE(w.registry().has<mith::HealthComponent>(self));
    REQUIRE(w.registry().has<mith::RoleComponent>(self));
    REQUIRE(w.registry().has<mith::BehaviourStateComponent>(self));
    REQUIRE(w.registry().has<mith::PermissionMaskComponent>(self));
    REQUIRE(w.registry().has<mith::ActionQueueComponent>(self));
    REQUIRE(w.registry().has<mith::CommBufferComponent>(self));

    // Identity carries the configured swarm + a freshly generated v4 UUID.
    const auto& id = w.registry().get<mith::IdentityComponent>(self).id;
    CHECK(id.swarm_id == 42u);
    CHECK(id.unit_id.version() == 4u);
    CHECK_FALSE(id.unit_id.is_nil());

    // Spot-check a few defaults.
    CHECK(w.registry().get<mith::HealthComponent>(self).value == 100u);
    CHECK(w.registry().get<mith::OrientationComponent>(self).qw == 1.0f);
    CHECK(w.registry().get<mith::PermissionMaskComponent>(self).allowed_builtins == 0xFFFFFFFFu);
    CHECK(w.registry().get<mith::ActionQueueComponent>(self).queue.empty());
    CHECK(w.registry().get<mith::CommBufferComponent>(self).queue.empty());
}

TEST_CASE("identity() returns the HID emplaced by init()") {
    WorldConfig cfg;
    cfg.swarm_id = 0xFF;
    World w(cfg);
    w.init();
    const auto& id = w.identity();
    CHECK(id.swarm_id == 0xFFu);
    CHECK(id.unit_id.version() == 4u);
}

TEST_CASE("tick() advances SwarmContext consistently") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 20.0f;
    World w(cfg);
    w.init();

    CHECK(w.context().tick_count == 0u);
    CHECK(w.context().elapsed_time_s == 0.0f);

    w.tick();
    CHECK(w.context().tick_count == 1u);
    CHECK(w.context().delta_time_s == doctest::Approx(0.05f));
    CHECK(w.context().elapsed_time_s == doctest::Approx(0.05f));

    for (int i = 0; i < 4; ++i) w.tick();
    CHECK(w.context().tick_count == 5u);
    CHECK(w.context().elapsed_time_s == doctest::Approx(0.25f));
}

TEST_CASE("register_system / tick dispatches user systems in lexicographic order") {
    std::vector<std::string> log;
    World w(WorldConfig{});
    REQUIRE(w.register_system(std::make_unique<TaggingSystem>("Charlie", log)) == SchedulerStatus::Ok);
    REQUIRE(w.register_system(std::make_unique<TaggingSystem>("Alpha", log))   == SchedulerStatus::Ok);
    REQUIRE(w.register_system(std::make_unique<TaggingSystem>("Bravo", log))   == SchedulerStatus::Ok);

    w.init();
    w.tick();

    REQUIRE(log.size() == 3u);
    CHECK(log[0] == "Alpha");
    CHECK(log[1] == "Bravo");
    CHECK(log[2] == "Charlie");
}

TEST_CASE("register_component<T>() forwards to registry with User origin") {
    struct MyKalmanState : mith::HotComponent<MyKalmanState> {
        float covariance[9]{};
    };

    World w(WorldConfig{});
    REQUIRE(w.register_component<MyKalmanState>() == RegistrationStatus::Ok);
    CHECK(w.registry().is_registered<MyKalmanState>());
    CHECK(w.registry().origin_of<MyKalmanState>() == ComponentOrigin::User);
}

TEST_CASE("set_trace_sink forwards to registry and scheduler") {
    JsonCapturingSink sink;
    World w(WorldConfig{});
    w.set_trace_sink(&sink);

    // init() registers 10 built-ins → 10 component_registered events.
    w.init();
    CHECK(sink.lines.size() == 10u);
    CHECK(contains(sink.lines[0], "\"event\":\"component_registered\""));

    // tick() emits one tick_completed event.
    w.tick();
    CHECK(sink.lines.size() == 11u);
    CHECK(contains(sink.lines.back(), "\"event\":\"tick_completed\""));
    CHECK(contains(sink.lines.back(), "\"tick\":1"));
}

TEST_CASE("set_trace_sink(nullptr) clears both sinks") {
    JsonCapturingSink sink;
    World w(WorldConfig{});
    w.set_trace_sink(&sink);
    w.init();
    REQUIRE(sink.lines.size() == 10u);

    w.set_trace_sink(nullptr);
    w.tick();
    CHECK(sink.lines.size() == 10u);   // tick_completed not captured after clear
}

TEST_CASE("ActionQueue and CommBuffer are usable through the registry after init") {
    World w(WorldConfig{});
    w.init();

    auto& aq = w.registry().get<mith::ActionQueueComponent>(w.self_id());
    REQUIRE(aq.queue.empty());

    mith::Action a;
    a.type = mith::actions::MOVE;
    REQUIRE(aq.queue.push(std::move(a)));
    CHECK(w.registry().get<mith::ActionQueueComponent>(w.self_id()).queue.size() == 1u);

    auto& cb = w.registry().get<mith::CommBufferComponent>(w.self_id());
    CHECK(cb.queue.empty());
    CHECK(cb.queue.capacity() == mith::CommBufferComponent::CAPACITY);
}

TEST_CASE("identity is stable across ticks") {
    World w(WorldConfig{});
    w.init();
    const auto initial = w.identity();
    for (int i = 0; i < 10; ++i) w.tick();
    const auto& after = w.identity();
    CHECK(after == initial);
}

TEST_CASE("run() exits cleanly when the stop_flag transitions to true") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 200.0f;   // 5ms ticks — keep the test fast
    World w(cfg);
    w.init();

    std::atomic<bool> stop{false};
    std::thread setter([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        stop.store(true, std::memory_order_relaxed);
    });

    w.run(stop);
    setter.join();

    CHECK(w.context().tick_count > 0u);
    CHECK(stop.load());
}
