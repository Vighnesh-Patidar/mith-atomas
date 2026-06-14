#include "doctest.h"

// End-to-end integration test for the §13.1 / §13.2 fault-handling path,
// running under the actual SimBus scheduler instead of hand-driven ticks.
// Exercises:
//
//   - FaultMonitorSystem registered into a SimBus-owned World, ticked
//     via SimBus::advance() (not manually).
//   - World::report_fault() events propagated through the §4.5 fault
//     counter into HealthComponent decrements at FaultMonitorSystem's
//     normal hazard-graph slot.
//   - PermissionMaskComponent transitions to the restricted mask on
//     degraded entry (§13.2) and back on recovery, observed from
//     outside the system.
//   - Hysteresis: no thrashing across many ticks while health hovers
//     near the threshold.
//
// Distinct from the FaultMonitorSystem unit test in that it goes through
// SimBus → SimClock → World::tick → SystemScheduler → FaultMonitorSystem,
// catching wiring regressions that unit tests can't.

#include "mith/comms/state_vector.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/scheduler.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/sim/sim_bus.h"
#include "mith/systems/fault_monitor_system.h"

#include <memory>

using mith::FaultMonitorSystem;
using mith::HealthComponent;
using mith::PermissionMaskComponent;
using mith::SchedulerStatus;
using mith::SwarmID;
using mith::World;
using mith::WorldConfig;
using mith::sim::SimBus;
using mith::sim::SimBusConfig;

namespace {

FaultMonitorSystem::Params test_params() {
    FaultMonitorSystem::Params p;
    p.decrement_per_fault = 5;
    p.degraded_threshold  = 40;
    p.degraded_hysteresis = 10;
    return p;
}

} // namespace

TEST_CASE("Integration: SimBus-driven fault injection drives health to degraded mode and back") {
    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f });
    auto w  = bus.create_world(SwarmID{1});

    auto fm        = std::make_unique<FaultMonitorSystem>(*w, test_params());
    FaultMonitorSystem* fm_raw = fm.get();
    REQUIRE(w->register_system(std::move(fm)) == SchedulerStatus::Ok);

    w->init();

    // --- Baseline: no faults, no degraded mode ---
    bus.advance(5);
    CHECK(w->registry().get<HealthComponent>(w->self_id()).value == 100u);
    CHECK_FALSE(fm_raw->in_degraded_state());
    const auto baseline_mask =
        w->registry().get<PermissionMaskComponent>(w->self_id()).allowed_builtins;
    CHECK(baseline_mask != 0u);

    // --- Inject faults: 13 reports → 65 health damage → health = 35 ---
    for (int i = 0; i < 13; ++i) w->report_fault();
    bus.advance(1);

    CHECK(w->registry().get<HealthComponent>(w->self_id()).value == 35u);
    CHECK(fm_raw->in_degraded_state());

    const auto degraded_mask =
        w->registry().get<PermissionMaskComponent>(w->self_id()).allowed_builtins;
    CHECK(degraded_mask != baseline_mask);
    CHECK(degraded_mask != 0u);     // not "no actions allowed" — restricted, not gagged

    // --- Continue ticking — hysteresis must prevent thrashing ---
    bus.advance(50);
    CHECK(fm_raw->in_degraded_state());          // still degraded
    CHECK(w->registry().get<HealthComponent>(w->self_id()).value == 35u);

    // --- Heal back above threshold + hysteresis (>=50) ---
    w->registry().get<HealthComponent>(w->self_id()).value = 60;
    bus.advance(1);

    CHECK_FALSE(fm_raw->in_degraded_state());
    CHECK(w->registry().get<PermissionMaskComponent>(w->self_id()).allowed_builtins ==
          baseline_mask);
}

TEST_CASE("Integration: fault counter delta is consumed exactly once per tick") {
    // Verifies the §4.5 contract: FaultMonitorSystem reads the World
    // fault_count delta and applies decrements exactly once. Faults
    // reported during a tick are visible only in the next tick's
    // decrement, not double-counted across subsequent ticks.

    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f });
    auto w  = bus.create_world(SwarmID{1});
    REQUIRE(w->register_system(std::make_unique<FaultMonitorSystem>(*w, test_params()))
            == SchedulerStatus::Ok);
    w->init();

    w->report_fault();
    bus.advance(1);
    const auto after_one_fault = w->registry().get<HealthComponent>(w->self_id()).value;
    CHECK(after_one_fault == 95u);     // -5 from one fault

    // No further faults — health must stay constant across many idle ticks.
    bus.advance(20);
    CHECK(w->registry().get<HealthComponent>(w->self_id()).value == 95u);
    CHECK_FALSE(w->registry().get<HealthComponent>(w->self_id()).value == 90u);
}

TEST_CASE("Integration: degraded mask is reproducibly snapshot/restored across the cycle") {
    // The §13.2 contract requires that the PermissionMask BEFORE degraded
    // entry is bit-exactly restored on recovery — not a rebuilt default.
    // Verifies snapshot/restore round-trip through a full down-and-back
    // cycle.

    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f });
    auto w  = bus.create_world(SwarmID{1});
    REQUIRE(w->register_system(std::make_unique<FaultMonitorSystem>(*w, test_params()))
            == SchedulerStatus::Ok);
    w->init();

    // Mutate the baseline permission mask to something non-default — the
    // snapshot/restore path must round-trip THIS exact value, not the
    // default mask the registry emplaces.
    w->registry().get<PermissionMaskComponent>(w->self_id()).allowed_builtins = 0xABCD1234u;
    const auto custom_baseline = 0xABCD1234u;

    bus.advance(1);   // settle

    // Knock health below threshold.
    for (int i = 0; i < 13; ++i) w->report_fault();
    bus.advance(1);
    CHECK(w->registry().get<PermissionMaskComponent>(w->self_id()).allowed_builtins !=
          custom_baseline);

    // Heal back above the hysteresis bar.
    w->registry().get<HealthComponent>(w->self_id()).value = 60;
    bus.advance(1);
    CHECK(w->registry().get<PermissionMaskComponent>(w->self_id()).allowed_builtins ==
          custom_baseline);
}
