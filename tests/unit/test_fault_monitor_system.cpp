#include "doctest.h"

#include "mith/behaviour/action_type.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"
#include "mith/systems/fault_monitor_system.h"

#include <memory>

using mith::BehaviourStateComponent;
using mith::FaultMonitorSystem;
using mith::HealthComponent;
using mith::PermissionMaskComponent;
using mith::SchedulerStatus;
using mith::World;
using mith::WorldConfig;

namespace {

FaultMonitorSystem::Params lenient() {
    FaultMonitorSystem::Params p;
    p.degraded_threshold  = 40;
    p.degraded_hysteresis = 10;
    p.decrement_per_fault = 5;
    return p;
}

} // namespace

TEST_CASE("FaultMonitorSystem: idle ticks do not change health") {
    World w(WorldConfig{});
    REQUIRE(w.register_system(std::make_unique<FaultMonitorSystem>(w, lenient()))
            == SchedulerStatus::Ok);
    w.init();

    REQUIRE(w.registry().get<HealthComponent>(w.self_id()).value == 100u);
    for (int i = 0; i < 10; ++i) w.tick();
    CHECK(w.registry().get<HealthComponent>(w.self_id()).value == 100u);
}

TEST_CASE("FaultMonitorSystem: a reported fault decrements health by decrement_per_fault") {
    World w(WorldConfig{});
    REQUIRE(w.register_system(std::make_unique<FaultMonitorSystem>(w, lenient()))
            == SchedulerStatus::Ok);
    w.init();

    w.report_fault();
    w.tick();
    CHECK(w.registry().get<HealthComponent>(w.self_id()).value == 95u);

    w.report_fault();
    w.report_fault();
    w.tick();
    CHECK(w.registry().get<HealthComponent>(w.self_id()).value == 85u);
}

TEST_CASE("FaultMonitorSystem: health saturates at 0 under heavy fault flood") {
    World w(WorldConfig{});
    REQUIRE(w.register_system(std::make_unique<FaultMonitorSystem>(w, lenient()))
            == SchedulerStatus::Ok);
    w.init();

    for (int i = 0; i < 50; ++i) w.report_fault();   // 50 * 5 = 250 > 100
    w.tick();

    CHECK(w.registry().get<HealthComponent>(w.self_id()).value == 0u);
}

TEST_CASE("FaultMonitorSystem: crossing threshold installs the degraded mask") {
    World w(WorldConfig{});
    auto fm = std::make_unique<FaultMonitorSystem>(w, lenient());
    FaultMonitorSystem* fm_raw = fm.get();
    REQUIRE(w.register_system(std::move(fm)) == SchedulerStatus::Ok);
    w.init();

    CHECK_FALSE(fm_raw->in_degraded_state());

    // Drop health below 40. 13 faults × 5 = 65 → 100 - 65 = 35.
    for (int i = 0; i < 13; ++i) w.report_fault();
    w.tick();

    CHECK(w.registry().get<HealthComponent>(w.self_id()).value == 35u);
    CHECK(fm_raw->in_degraded_state());

    const auto& mask = w.registry().get<PermissionMaskComponent>(w.self_id());
    const std::uint32_t expected =
        (1u << mith::actions::IDLE) | (1u << mith::actions::REGROUP);
    CHECK(mask.allowed_builtins == expected);
    CHECK(mask.allow_user_actions == false);

    const auto& state = w.registry().get<BehaviourStateComponent>(w.self_id());
    CHECK(state.state == FaultMonitorSystem::DEGRADED_STATE);
}

TEST_CASE("FaultMonitorSystem: mask blocks non-IDLE/REGROUP actions in degraded state") {
    World w(WorldConfig{});
    REQUIRE(w.register_system(std::make_unique<FaultMonitorSystem>(w, lenient()))
            == SchedulerStatus::Ok);
    w.init();

    for (int i = 0; i < 13; ++i) w.report_fault();
    w.tick();

    const auto& mask = w.registry().get<PermissionMaskComponent>(w.self_id());
    CHECK(mask.allows(mith::actions::IDLE));
    CHECK(mask.allows(mith::actions::REGROUP));
    CHECK_FALSE(mask.allows(mith::actions::MOVE));
    CHECK_FALSE(mask.allows(mith::actions::TRANSMIT));
    CHECK_FALSE(mask.allows(mith::actions::CUSTOM));
}

TEST_CASE("FaultMonitorSystem: recovery above threshold + hysteresis restores the mask") {
    World w(WorldConfig{});
    auto fm = std::make_unique<FaultMonitorSystem>(w, lenient());
    FaultMonitorSystem* fm_raw = fm.get();
    REQUIRE(w.register_system(std::move(fm)) == SchedulerStatus::Ok);
    w.init();

    // Snapshot the initial mask before degrading.
    const auto initial_mask = w.registry().get<PermissionMaskComponent>(w.self_id());

    // Drop into degraded.
    for (int i = 0; i < 13; ++i) w.report_fault();
    w.tick();
    REQUIRE(fm_raw->in_degraded_state());

    // Simulate recovery: heal health back. Threshold 40 + hysteresis 10 = 50.
    w.registry().get<HealthComponent>(w.self_id()).value = 55u;
    w.tick();

    CHECK_FALSE(fm_raw->in_degraded_state());
    const auto& mask = w.registry().get<PermissionMaskComponent>(w.self_id());
    CHECK(mask.allowed_builtins == initial_mask.allowed_builtins);
    CHECK(mask.allow_user_actions == initial_mask.allow_user_actions);
    CHECK(w.registry().get<BehaviourStateComponent>(w.self_id()).state
          == FaultMonitorSystem::NORMAL_STATE);
}

TEST_CASE("FaultMonitorSystem: hysteresis prevents thrashing near the threshold") {
    World w(WorldConfig{});
    auto fm = std::make_unique<FaultMonitorSystem>(w, lenient());
    FaultMonitorSystem* fm_raw = fm.get();
    REQUIRE(w.register_system(std::move(fm)) == SchedulerStatus::Ok);
    w.init();

    for (int i = 0; i < 13; ++i) w.report_fault();
    w.tick();
    REQUIRE(fm_raw->in_degraded_state());

    // Heal to exactly threshold (40) — should NOT recover (need ≥50).
    w.registry().get<HealthComponent>(w.self_id()).value = 40u;
    w.tick();
    CHECK(fm_raw->in_degraded_state());

    // Heal to threshold + hysteresis - 1 = 49 — still degraded.
    w.registry().get<HealthComponent>(w.self_id()).value = 49u;
    w.tick();
    CHECK(fm_raw->in_degraded_state());

    // Heal to 50 — now recovers.
    w.registry().get<HealthComponent>(w.self_id()).value = 50u;
    w.tick();
    CHECK_FALSE(fm_raw->in_degraded_state());
}

TEST_CASE("FaultMonitorSystem: SystemDescriptor declares the §13 reads/writes") {
    World w(WorldConfig{});
    FaultMonitorSystem fm(w);
    const auto d = fm.describe();
    CHECK(d.name == "FaultMonitorSystem");
    CHECK(d.reads_components.size() == 1u);   // Health
    CHECK(d.writes_components.size() == 3u);  // Health, Mask, BehaviourState
    CHECK(d.reads_resources.empty());
    CHECK(d.writes_resources.empty());
}

TEST_CASE("World::report_fault: thread-safe counter increment") {
    World w(WorldConfig{});
    CHECK(w.fault_count() == 0u);
    w.report_fault();
    CHECK(w.fault_count() == 1u);
    for (int i = 0; i < 100; ++i) w.report_fault();
    CHECK(w.fault_count() == 101u);
}
