#include "doctest.h"

#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"
#include "mith/systems/kinematics_system.h"

#include <memory>

using mith::KinematicsSystem;
using mith::PositionComponent;
using mith::VelocityComponent;
using mith::World;
using mith::WorldConfig;

TEST_CASE("KinematicsSystem integrates position from velocity") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 10.0f;     // dt = 0.1
    World w(cfg);
    REQUIRE(w.register_system(std::make_unique<KinematicsSystem>())
            == mith::SchedulerStatus::Ok);
    w.init();

    w.registry().get<PositionComponent>(w.self_id()) = PositionComponent{0.0f, 0.0f, 0.0f};
    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{2.0f, -1.0f, 0.5f};

    w.tick();   // dt = 0.1

    const auto& p = w.registry().get<PositionComponent>(w.self_id());
    CHECK(p.x == doctest::Approx(0.2f));
    CHECK(p.y == doctest::Approx(-0.1f));
    CHECK(p.z == doctest::Approx(0.05f));
}

TEST_CASE("KinematicsSystem accumulates across ticks") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 10.0f;
    World w(cfg);
    REQUIRE(w.register_system(std::make_unique<KinematicsSystem>())
            == mith::SchedulerStatus::Ok);
    w.init();

    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{1.0f, 0.0f, 0.0f};
    for (int i = 0; i < 10; ++i) w.tick();   // 1.0 s

    const auto& p = w.registry().get<PositionComponent>(w.self_id());
    CHECK(p.x == doctest::Approx(1.0f));
    CHECK(p.y == doctest::Approx(0.0f));
}

TEST_CASE("KinematicsSystem does not modify velocity") {
    World w(WorldConfig{});
    REQUIRE(w.register_system(std::make_unique<KinematicsSystem>())
            == mith::SchedulerStatus::Ok);
    w.init();

    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{3.0f, 4.0f, 0.0f};
    w.tick();
    w.tick();

    const auto& v = w.registry().get<VelocityComponent>(w.self_id());
    CHECK(v.vx == 3.0f);
    CHECK(v.vy == 4.0f);
}

TEST_CASE("KinematicsSystem SystemDescriptor declares pos write / vel read") {
    KinematicsSystem ks;
    const auto d = ks.describe();
    CHECK(d.name == "KinematicsSystem");
    CHECK(d.reads_components.size() == 1u);
    CHECK(d.writes_components.size() == 1u);
}
