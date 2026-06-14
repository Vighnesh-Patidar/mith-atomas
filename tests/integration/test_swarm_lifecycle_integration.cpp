#include "doctest.h"

// Full v0.2 stack integration test. Three Worlds participate via SimBus,
// each running BeaconSystem (channel-aware split transport),
// TaskAllocSystem (threshold-based role rebalancing), and
// FaultMonitorSystem (degraded mode). Verifies:
//
//   - Beacons propagate end-to-end across SimBus into each World's
//     NeighbourTable.
//   - TaskAllocSystem reads the populated NeighbourTable and rebalances
//     roles such that the swarm converges to the desired ratio.
//   - Fault injection on ONE participant transitions only that
//     participant to degraded mode; the others are unaffected.
//
// This is the v0.2 acceptance gate — if it passes, the channel-aware
// transport + rotation + alloc + fault-handling pieces compose without
// stepping on each other's hazards.

#include "mith/comms/beacon_system.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/scheduler.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/sim/sim_bus.h"
#include "mith/systems/fault_monitor_system.h"
#include "mith/systems/task_alloc_system.h"

#include <array>
#include <memory>

using mith::BeaconSystem;
using mith::FaultMonitorSystem;
using mith::HealthComponent;
using mith::PermissionMaskComponent;
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

// Each World gets identical TaskAlloc params: roles 1 and 2 at 50/50.
TaskAllocSystem::Params alloc_params() {
    TaskAllocSystem::Params p;
    p.desired_ratios[1] = 0.5f;
    p.desired_ratios[2] = 0.5f;
    p.thresholds[1]     = 1.0f;
    p.thresholds[2]     = 1.0f;
    p.stability_window_s = 0.0f;
    p.stimulus_gain      = 1.0f;
    return p;
}

FaultMonitorSystem::Params fault_params() {
    FaultMonitorSystem::Params p;
    p.decrement_per_fault = 5;
    p.degraded_threshold  = 40;
    p.degraded_hysteresis = 10;
    return p;
}

void wire_world(World& w, RoleComponent initial_role) {
    REQUIRE(w.register_system(std::make_unique<BeaconSystem>(w))
            == SchedulerStatus::Ok);
    REQUIRE(w.register_system(std::make_unique<TaskAllocSystem>(w, alloc_params()))
            == SchedulerStatus::Ok);
    REQUIRE(w.register_system(std::make_unique<FaultMonitorSystem>(w, fault_params()))
            == SchedulerStatus::Ok);
    w.init();
    w.registry().get<RoleComponent>(w.self_id()) = initial_role;
}

} // namespace

TEST_CASE("Integration: beacons propagate and TaskAllocSystem converges the swarm to target ratios") {
    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f, /*range=*/1000.0f });

    auto wa = bus.create_world(SwarmID{1});
    auto wb = bus.create_world(SwarmID{1});
    auto wc = bus.create_world(SwarmID{1});

    // All three start in role 1 — under TaskAlloc rebalance, one should
    // drift to role 2 to converge to the 50/50 target.
    wire_world(*wa, RoleComponent{1});
    wire_world(*wb, RoleComponent{1});
    wire_world(*wc, RoleComponent{1});

    // Beacon rate default is non-zero. Advance long enough for beacons
    // to propagate (each World needs to see the others) AND for
    // TaskAllocSystem to act on the data.
    bus.advance(50);   // 2.5 s at 20 Hz

    // After convergence, role 1 + role 2 counts should be close to 50/50
    // (i.e. NOT all-1 anymore). At minimum, the swarm should not still
    // be unanimously role 1.
    int role1_count = 0;
    int role2_count = 0;
    for (auto* w : {wa.get(), wb.get(), wc.get()}) {
        const auto r = w->registry().get<RoleComponent>(w->self_id()).role;
        if (r == 1) ++role1_count;
        if (r == 2) ++role2_count;
    }
    CHECK(role1_count + role2_count == 3);
    CHECK(role2_count >= 1);    // at least one robot took role 2

    // Each World's NeighbourTable should have observed at least one peer
    // (sanity check that BeaconSystem actually delivered).
    for (auto* w : {wa.get(), wb.get(), wc.get()}) {
        std::size_t neighbour_count = 0;
        for (auto it = w->neighbour_table().begin();
             it != w->neighbour_table().end(); ++it) {
            ++neighbour_count;
        }
        CHECK(neighbour_count >= 1);   // saw at least one peer
    }
}

TEST_CASE("Integration: degraded mode is local — one robot down doesn't take the swarm with it") {
    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f, /*range=*/1000.0f });

    auto wa = bus.create_world(SwarmID{1});
    auto wb = bus.create_world(SwarmID{1});

    wire_world(*wa, RoleComponent{1});
    wire_world(*wb, RoleComponent{2});

    bus.advance(5);   // warmup — beacons propagate, baseline state settled

    // Capture each World's FaultMonitorSystem via a search of registered
    // systems would be ergonomic; instead, just observe via the public
    // health + permission state. Inject 13 faults on A only.
    for (int i = 0; i < 13; ++i) wa->report_fault();
    bus.advance(1);

    const auto a_health = wa->registry().get<HealthComponent>(wa->self_id()).value;
    const auto b_health = wb->registry().get<HealthComponent>(wb->self_id()).value;
    CHECK(a_health == 35u);     // damaged
    CHECK(b_health == 100u);    // untouched

    // B's permission mask is unchanged by A's faults — locality holds.
    const auto a_mask = wa->registry().get<PermissionMaskComponent>(wa->self_id()).allowed_builtins;
    const auto b_mask = wb->registry().get<PermissionMaskComponent>(wb->self_id()).allowed_builtins;
    CHECK(a_mask != b_mask);    // A is restricted, B is not
}

TEST_CASE("Integration: long-running tick loop does not leak state across ticks") {
    // Regression check: the v0.2 wiring (channel-aware split transports,
    // identity rotation accumulators, fault counter delta, TaskAlloc
    // counters) all reset cleanly per tick. A 5-second sim run with no
    // injected events should leave the swarm in steady state.

    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f, /*range=*/1000.0f });
    auto w = bus.create_world(SwarmID{1});
    wire_world(*w, RoleComponent{1});

    bus.advance(100);   // 5 s — no fault injection

    CHECK(w->registry().get<HealthComponent>(w->self_id()).value == 100u);
    CHECK(w->fault_count() == 0u);
    // PermissionMask still in its post-init state (full permissions).
    CHECK(w->registry().get<PermissionMaskComponent>(w->self_id()).allowed_builtins != 0u);
}
