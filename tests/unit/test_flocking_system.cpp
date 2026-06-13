#include "doctest.h"

#include <cmath>

#include "mith/comms/beacon_system.h"
#include "mith/comms/neighbour_table.h"
#include "mith/comms/state_vector.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/sim/sim_bus.h"
#include "mith/systems/flocking_system.h"

#include <memory>

using mith::BeaconSystem;
using mith::FlockingSystem;
using mith::HierarchicalID;
using mith::PositionComponent;
using mith::StateVector;
using mith::SwarmID;
using mith::VelocityComponent;
using mith::World;
using mith::WorldConfig;
using mith::sim::SimBus;
using mith::sim::SimBusConfig;

TEST_CASE("FlockingSystem is a no-op when NeighbourTable is empty") {
    World w(WorldConfig{});
    REQUIRE(w.register_system(std::make_unique<FlockingSystem>(w))
            == mith::SchedulerStatus::Ok);
    w.init();

    // Seed velocity to a known value.
    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{1.0f, 0.0f, 0.0f};

    w.tick();

    const auto& v = w.registry().get<VelocityComponent>(w.self_id());
    CHECK(v.vx == 1.0f);
    CHECK(v.vy == 0.0f);
    CHECK(v.vz == 0.0f);
}

TEST_CASE("Cohesion pulls velocity toward a single distant neighbour") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 10.0f;
    World w(cfg);
    FlockingSystem::Params p;
    p.separation_radius_m = 1.0f;   // tiny — keeps separation inactive at 10 m
    p.cohesion_weight     = 1.0f;
    p.alignment_weight    = 0.0f;
    p.separation_weight   = 0.0f;
    p.max_speed_mps       = 100.0f; // don't clamp during this test
    REQUIRE(w.register_system(std::make_unique<FlockingSystem>(w, p))
            == mith::SchedulerStatus::Ok);
    w.init();

    // Place a neighbour 10 m to the right (+x). Cohesion should pull +x.
    StateVector sv;
    sv.id = HierarchicalID::generate(SwarmID{1});
    sv.position.x = 10.0f;
    w.neighbour_table().upsert(sv, /*now=*/1.0f);

    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{0.0f, 0.0f, 0.0f};
    w.tick();

    const auto& v = w.registry().get<VelocityComponent>(w.self_id());
    CHECK(v.vx > 0.0f);
    CHECK(v.vy == doctest::Approx(0.0f));
    CHECK(v.vz == doctest::Approx(0.0f));
}

TEST_CASE("Separation pushes velocity away from a too-close neighbour") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 10.0f;
    World w(cfg);
    FlockingSystem::Params p;
    p.separation_radius_m = 5.0f;
    p.separation_weight   = 1.0f;
    p.alignment_weight    = 0.0f;
    p.cohesion_weight     = 0.0f;
    p.max_speed_mps       = 100.0f;
    REQUIRE(w.register_system(std::make_unique<FlockingSystem>(w, p))
            == mith::SchedulerStatus::Ok);
    w.init();

    // Neighbour 2 m to the right, well within separation radius.
    StateVector sv;
    sv.id = HierarchicalID::generate(SwarmID{1});
    sv.position.x = 2.0f;
    w.neighbour_table().upsert(sv, 1.0f);

    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{0.0f, 0.0f, 0.0f};
    w.tick();

    const auto& v = w.registry().get<VelocityComponent>(w.self_id());
    CHECK(v.vx < 0.0f);   // pushed away (left)
}

TEST_CASE("Alignment nudges velocity toward neighbour's velocity") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 10.0f;
    World w(cfg);
    FlockingSystem::Params p;
    p.separation_radius_m = 0.0f;   // never separates
    p.separation_weight   = 0.0f;
    p.cohesion_weight     = 0.0f;
    p.alignment_weight    = 1.0f;
    p.max_speed_mps       = 100.0f;
    REQUIRE(w.register_system(std::make_unique<FlockingSystem>(w, p))
            == mith::SchedulerStatus::Ok);
    w.init();

    // Neighbour moving +y at 3 m/s; we're at rest. Alignment should
    // nudge us +y.
    StateVector sv;
    sv.id = HierarchicalID::generate(SwarmID{1});
    sv.position.x = 10.0f;
    sv.velocity.vy = 3.0f;
    w.neighbour_table().upsert(sv, 1.0f);

    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{};
    w.tick();

    const auto& v = w.registry().get<VelocityComponent>(w.self_id());
    CHECK(v.vy > 0.0f);
    CHECK(v.vx == doctest::Approx(0.0f));
}

TEST_CASE("Separation does NOT activate outside separation_radius") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 10.0f;
    World w(cfg);
    FlockingSystem::Params p;
    p.separation_radius_m = 5.0f;
    p.separation_weight   = 100.0f;   // very strong, would dominate if active
    p.alignment_weight    = 0.0f;
    p.cohesion_weight     = 0.0f;
    p.max_speed_mps       = 100.0f;
    REQUIRE(w.register_system(std::make_unique<FlockingSystem>(w, p))
            == mith::SchedulerStatus::Ok);
    w.init();

    // Neighbour 50 m away — well outside 5 m separation radius.
    StateVector sv;
    sv.id = HierarchicalID::generate(SwarmID{1});
    sv.position.x = 50.0f;
    w.neighbour_table().upsert(sv, 1.0f);

    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{};
    w.tick();

    const auto& v = w.registry().get<VelocityComponent>(w.self_id());
    CHECK(v.vx == doctest::Approx(0.0f));
    CHECK(v.vy == doctest::Approx(0.0f));
    CHECK(v.vz == doctest::Approx(0.0f));
}

TEST_CASE("Max-speed clamp limits velocity magnitude") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 1.0f;       // huge dt to blow past max_speed in one tick
    World w(cfg);
    FlockingSystem::Params p;
    p.separation_radius_m = 0.0f;
    p.separation_weight   = 0.0f;
    p.alignment_weight    = 0.0f;
    p.cohesion_weight     = 1000.0f;  // strong pull
    p.max_speed_mps       = 5.0f;
    REQUIRE(w.register_system(std::make_unique<FlockingSystem>(w, p))
            == mith::SchedulerStatus::Ok);
    w.init();

    StateVector sv;
    sv.id = HierarchicalID::generate(SwarmID{1});
    sv.position.x = 100.0f;       // far — strong cohesion pull
    w.neighbour_table().upsert(sv, 1.0f);

    w.registry().get<VelocityComponent>(w.self_id()) = VelocityComponent{};
    w.tick();

    const auto& v = w.registry().get<VelocityComponent>(w.self_id());
    const float speed = std::sqrt(v.vx*v.vx + v.vy*v.vy + v.vz*v.vz);
    CHECK(speed <= 5.0f + 1e-3f);
}

TEST_CASE("SystemDescriptor declares the §5.3 reads/writes for FlockingSystem") {
    World w(WorldConfig{});
    FlockingSystem fs(w);
    const auto d = fs.describe();

    CHECK(d.name == "FlockingSystem");
    CHECK(d.reads_components.size() == 2u);   // Position, Velocity
    CHECK(d.writes_components.size() == 1u);  // Velocity
    REQUIRE(d.reads_resources.size() == 1u);
    CHECK(d.reads_resources[0] == mith::ResourceID::NeighbourTable);
    CHECK(d.writes_resources.empty());
}

TEST_CASE("Multi-tick: two robots converge on shared velocity (alignment-only)") {
    SimBus bus(SimBusConfig{ /*tick=*/20.0f, /*range=*/100.0f });
    auto a = bus.create_world(SwarmID{1});
    auto b = bus.create_world(SwarmID{1});

    REQUIRE(a->register_system(std::make_unique<BeaconSystem>(*a))
            == mith::SchedulerStatus::Ok);
    REQUIRE(b->register_system(std::make_unique<BeaconSystem>(*b))
            == mith::SchedulerStatus::Ok);

    FlockingSystem::Params align_only;
    align_only.separation_radius_m = 0.0f;
    align_only.separation_weight   = 0.0f;
    align_only.cohesion_weight     = 0.0f;
    align_only.alignment_weight    = 4.0f;
    align_only.max_speed_mps       = 10.0f;

    REQUIRE(a->register_system(std::make_unique<FlockingSystem>(*a, align_only))
            == mith::SchedulerStatus::Ok);
    REQUIRE(b->register_system(std::make_unique<FlockingSystem>(*b, align_only))
            == mith::SchedulerStatus::Ok);

    a->init();
    b->init();

    // A moves +x at 2 m/s; B at rest. Alignment should bring B toward +x.
    a->registry().get<VelocityComponent>(a->self_id()) = VelocityComponent{2.0f, 0.0f, 0.0f};
    b->registry().get<VelocityComponent>(b->self_id()) = VelocityComponent{};

    bus.advance(40);   // 2 s

    const auto& vb = b->registry().get<VelocityComponent>(b->self_id());
    CHECK(vb.vx > 0.0f);   // B has been pulled toward A's heading
}
