#include "doctest.h"

// End-to-end test for the §16 v0.4 partition-merge path. Two subswarms
// are positioned out of each other's comm range (simulating a network
// partition), each settles into its own state, then everyone is moved
// into range (the heal) and PartitionMergeSystem opens a merge window
// that lets TaskAllocSystem rebalance fast despite a long stability
// window.
//
// Also injects faults during the partition phase to confirm the v0.4
// roadmap's "fault-injection integration tests exercising the merge
// path" — fault counters survive the partition and the heal both.

#include "mith/comms/beacon_system.h"
#include "mith/comms/neighbour_table.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/scheduler.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/sim/sim_bus.h"
#include "mith/systems/fault_monitor_system.h"
#include "mith/systems/partition_merge_system.h"
#include "mith/systems/task_alloc_system.h"

#include <array>
#include <cstdint>
#include <memory>

using mith::BeaconSystem;
using mith::FaultMonitorSystem;
using mith::HealthComponent;
using mith::PartitionMergeSystem;
using mith::PositionComponent;
using mith::RoleComponent;
using mith::SchedulerStatus;
using mith::SwarmID;
using mith::TaskAllocSystem;
using mith::World;
using mith::WorldConfig;
using mith::sim::SimBus;
using mith::sim::SimBusConfig;

namespace {

// One World wired up the same way every robot in this test is.
struct Robot {
    std::unique_ptr<World>     world;
    PartitionMergeSystem*      partition_merge;
    TaskAllocSystem*           task_alloc;
    FaultMonitorSystem*        fault_monitor;
};

TaskAllocSystem::Params alloc_params() {
    TaskAllocSystem::Params p;
    p.desired_ratios[1]  = 0.5f;     // role 1 — 50%
    p.desired_ratios[2]  = 0.5f;     // role 2 — 50%
    p.thresholds[1]      = 1.0f;
    p.thresholds[2]      = 1.0f;
    p.stability_window_s = 5.0f;     // long enough to suppress passive churn
    p.stimulus_gain      = 1.0f;
    return p;
}

PartitionMergeSystem::Params merge_params() {
    PartitionMergeSystem::Params p;
    // Threshold 3: skips the initial NeighbourTable warmup
    // (0 → 2 peers as the subswarm beacons settle) but catches the
    // real heal in phase 2 (2 → 5 peers when the swarm rejoins).
    p.partition_heal_threshold = 3;
    p.merge_window_s           = 1.0f;
    return p;
}

FaultMonitorSystem::Params fault_params() {
    FaultMonitorSystem::Params p;
    p.decrement_per_fault = 5;
    p.degraded_threshold  = 40;
    p.degraded_hysteresis = 10;
    return p;
}

Robot make_robot(SimBus& bus, float x, RoleComponent initial_role) {
    Robot r;
    r.world = bus.create_world(SwarmID{1});

    auto pm = std::make_unique<PartitionMergeSystem>(*r.world, merge_params());
    auto ta = std::make_unique<TaskAllocSystem>(*r.world, alloc_params());
    auto fm = std::make_unique<FaultMonitorSystem>(*r.world, fault_params());
    r.partition_merge = pm.get();
    r.task_alloc      = ta.get();
    r.fault_monitor   = fm.get();

    REQUIRE(r.world->register_system(std::make_unique<BeaconSystem>(*r.world))
            == SchedulerStatus::Ok);
    REQUIRE(r.world->register_system(std::move(pm)) == SchedulerStatus::Ok);
    REQUIRE(r.world->register_system(std::move(ta)) == SchedulerStatus::Ok);
    REQUIRE(r.world->register_system(std::move(fm)) == SchedulerStatus::Ok);
    r.world->init();

    r.world->registry().get<PositionComponent>(r.world->self_id()).x = x;
    r.world->registry().get<RoleComponent>(r.world->self_id()) = initial_role;
    return r;
}

void set_x(Robot& r, float x) {
    r.world->registry().get<PositionComponent>(r.world->self_id()).x = x;
}

std::size_t neighbour_count(const Robot& r) {
    std::size_t n = 0;
    for (auto it = r.world->neighbour_table().begin();
         it != r.world->neighbour_table().end(); ++it) ++n;
    return n;
}

} // namespace

TEST_CASE("Integration: partition → heal opens the merge window across the swarm") {
    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f, /*comm_range_m=*/15.0f });

    // Two subswarms, three robots each, at x ≈ -50 and x ≈ +50 — 100 m
    // apart, far outside the 15 m comm range.
    std::vector<Robot> robots;
    robots.push_back(make_robot(bus, -50.0f, RoleComponent{1}));
    robots.push_back(make_robot(bus, -48.0f, RoleComponent{1}));
    robots.push_back(make_robot(bus, -52.0f, RoleComponent{1}));
    robots.push_back(make_robot(bus,  50.0f, RoleComponent{2}));
    robots.push_back(make_robot(bus,  48.0f, RoleComponent{2}));
    robots.push_back(make_robot(bus,  52.0f, RoleComponent{2}));

    // --- Phase 1: partitioned. Each subswarm only sees its two peers ---
    bus.advance(20);

    // Each robot should now see exactly 2 peers (the other members of
    // its subswarm). The opposite subswarm is invisible.
    for (const auto& r : robots) {
        const auto n = neighbour_count(r);
        CHECK(n == 2u);
    }

    // No heal has happened yet — PartitionMergeSystem hasn't fired.
    for (const auto& r : robots) {
        CHECK(r.partition_merge->heal_events() == 0u);
        CHECK_FALSE(r.world->is_merging());
    }

    // Inject faults on robot 0 during the partition — verify the
    // fault path still works in isolation.
    for (int i = 0; i < 4; ++i) robots[0].world->report_fault();
    bus.advance(1);
    CHECK(robots[0].world->registry().get<HealthComponent>(robots[0].world->self_id()).value
          == 80u);   // 100 - 4*5

    // --- Phase 2: heal — collapse positions to within comm range ---
    set_x(robots[0],  0.0f);
    set_x(robots[1],  2.0f);
    set_x(robots[2], -2.0f);
    set_x(robots[3],  4.0f);
    set_x(robots[4],  6.0f);
    set_x(robots[5], -4.0f);

    // Track whether is_merging() is observed at any point post-heal.
    bool any_merging_observed = false;
    std::size_t max_n_seen     = 0;
    for (int t = 0; t < 30; ++t) {
        bus.advance(1);
        for (const auto& r : robots) {
            if (r.world->is_merging()) any_merging_observed = true;
            const auto n = neighbour_count(r);
            if (n > max_n_seen) max_n_seen = n;
        }
    }

    // Every robot should now see 5 peers (the rest of the merged swarm).
    for (const auto& r : robots) {
        CHECK(neighbour_count(r) == 5u);
    }
    CHECK(max_n_seen == 5u);

    // PartitionMergeSystem on at least one robot detected the jump.
    std::uint64_t total_heals = 0;
    for (const auto& r : robots) total_heals += r.partition_merge->heal_events();
    CHECK(total_heals >= 1u);

    // World::is_merging() was true at least once during phase 2 — the
    // merge window opened.
    CHECK(any_merging_observed);

    // Fault counter on robot 0 survived the heal — same value as
    // pre-heal, no spurious decrement.
    CHECK(robots[0].world->registry().get<HealthComponent>(robots[0].world->self_id()).value
          == 80u);
    // Other robots remain at full health — partition heal doesn't
    // propagate faults.
    for (std::size_t i = 1; i < robots.size(); ++i) {
        CHECK(robots[i].world->registry().get<HealthComponent>(robots[i].world->self_id()).value
              == 100u);
    }
}

TEST_CASE("Integration: merge window relaxes long stability window for TaskAlloc rebalance") {
    // Smaller scenario: one robot in role 0 surrounded by peers in
    // role 1. Long stability window normally blocks any switch. A
    // PartitionMergeSystem-driven merge window must let it switch
    // toward role 2 (the under-supplied role).

    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f, /*comm_range_m=*/100.0f });

    // Build a robot with a very long stability window — would normally
    // suppress all role changes for the duration of the test.
    auto w = bus.create_world(SwarmID{1});

    TaskAllocSystem::Params p = alloc_params();
    p.stability_window_s = 9999.0f;
    auto ta = std::make_unique<TaskAllocSystem>(*w, p);
    auto pm = std::make_unique<PartitionMergeSystem>(*w, merge_params());
    TaskAllocSystem* ta_raw = ta.get();
    REQUIRE(w->register_system(std::make_unique<BeaconSystem>(*w)) == SchedulerStatus::Ok);
    REQUIRE(w->register_system(std::move(pm)) == SchedulerStatus::Ok);
    REQUIRE(w->register_system(std::move(ta)) == SchedulerStatus::Ok);
    w->init();
    w->registry().get<RoleComponent>(w->self_id()) = RoleComponent{1};

    // No peers initially — nothing for the partition merger to detect.
    bus.advance(5);
    CHECK_FALSE(w->is_merging());
    CHECK(ta_raw->total_role_switches() == 0u);

    // Add 5 peers within range, all in role 1. From this World's
    // perspective the NeighbourTable jumps from 0 → 5 — heal detected.
    std::vector<std::unique_ptr<World>> peers;
    for (int i = 0; i < 5; ++i) {
        auto p_world = bus.create_world(SwarmID{1});
        REQUIRE(p_world->register_system(std::make_unique<BeaconSystem>(*p_world))
                == SchedulerStatus::Ok);
        p_world->init();
        p_world->registry().get<RoleComponent>(p_world->self_id()) = RoleComponent{1};
        peers.push_back(std::move(p_world));
    }

    bus.advance(20);

    // PartitionMergeSystem should have opened a merge window at some
    // point AND TaskAllocSystem should have switched at least once.
    CHECK(ta_raw->total_role_switches() >= 1u);
    // Self should have moved to role 2 — the under-supplied role in
    // a swarm of all-role-1s.
    CHECK(w->registry().get<RoleComponent>(w->self_id()).role == 2u);
}
