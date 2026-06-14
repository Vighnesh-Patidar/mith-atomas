#include "doctest.h"

#include "mith/comms/neighbour_table.h"
#include "mith/comms/state_vector.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/systems/task_alloc_system.h"

using mith::HierarchicalID;
using mith::NeighbourTable;
using mith::RoleComponent;
using mith::RoleID;
using mith::StateVector;
using mith::SwarmID;
using mith::TaskAllocSystem;
using mith::World;
using mith::WorldConfig;

namespace {

// Seed the NeighbourTable with N neighbours all holding `role`.
void populate_neighbours(NeighbourTable& nt, std::size_t count, RoleID role, SwarmID swarm = SwarmID{1}) {
    for (std::size_t i = 0; i < count; ++i) {
        StateVector sv;
        sv.id        = HierarchicalID::generate(swarm);
        sv.role.role = role;
        nt.upsert(sv, /*now_s=*/0.0f);
    }
}

TaskAllocSystem::Params two_role_params() {
    TaskAllocSystem::Params p;
    p.desired_ratios[1] = 0.5f;     // role 1: 50% of swarm
    p.desired_ratios[2] = 0.5f;     // role 2: 50% of swarm
    p.thresholds[1]     = 1.0f;
    p.thresholds[2]     = 1.0f;
    p.stability_window_s = 0.0f;    // no hysteresis for these tests
    p.stimulus_gain      = 1.0f;
    return p;
}

} // namespace

TEST_CASE("TaskAllocSystem: lone robot switches toward an under-supplied role") {
    World w(WorldConfig{});
    w.init();

    // Self starts in role 0 (default).
    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 0u);

    // Build a swarm view: 6 neighbours all in role 1. Desired ratio is
    // 50/50 between roles 1 and 2 → role 2 has a big deficit (7 total *
    // 0.5 = 3.5 desired, 0 observed → deficit 3.5, far above theta=1.0).
    populate_neighbours(w.neighbour_table(), 6, /*role=*/1);

    TaskAllocSystem sys(w, two_role_params());
    sys.tick(w.registry(), w.context(), /*delta_time=*/0.1f);

    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 2u);
    CHECK(sys.total_role_switches() == 1u);
}

TEST_CASE("TaskAllocSystem: saturated role triggers no switch") {
    World w(WorldConfig{});
    w.init();

    // Self in role 1; 5 neighbours also in role 1, 5 in role 2. Total
    // 11, half should be each → no role has deficit.
    w.registry().get<RoleComponent>(w.self_id()).role = 1u;
    populate_neighbours(w.neighbour_table(), 5, 1);
    populate_neighbours(w.neighbour_table(), 5, 2);

    TaskAllocSystem sys(w, two_role_params());
    sys.tick(w.registry(), w.context(), 0.1f);

    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 1u);
    CHECK(sys.total_role_switches() == 0u);
}

TEST_CASE("TaskAllocSystem: high threshold suppresses small-deficit switches") {
    World w(WorldConfig{});
    w.init();
    populate_neighbours(w.neighbour_table(), 1, 1);   // total swarm 2; deficit for role 2 = 0.5

    TaskAllocSystem::Params p = two_role_params();
    p.thresholds[2] = 5.0f;    // very high threshold — stimulus 0.5 won't clear it
    TaskAllocSystem sys(w, p);
    sys.tick(w.registry(), w.context(), 0.1f);

    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 0u);
    CHECK(sys.total_role_switches() == 0u);
}

TEST_CASE("TaskAllocSystem: stability window suppresses rapid re-switching") {
    World w(WorldConfig{});
    w.init();
    populate_neighbours(w.neighbour_table(), 6, 1);

    TaskAllocSystem::Params p = two_role_params();
    p.stability_window_s = 5.0f;
    TaskAllocSystem sys(w, p);

    sys.tick(w.registry(), w.context(), 0.1f);   // first tick: t_since_switch starts at 0.1
    // First tick can switch because the counter has just incremented to 0.1
    // which is < 5.0. Verify the gate works correctly: nothing should switch.
    CHECK(sys.total_role_switches() == 0u);
    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 0u);

    // Now advance time past the window without switching, then tick again.
    for (int i = 0; i < 60; ++i) sys.tick(w.registry(), w.context(), 0.1f);
    CHECK(sys.total_role_switches() == 1u);
    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 2u);
}

TEST_CASE("TaskAllocSystem: largest-margin candidate wins (deterministic tie-break)") {
    World w(WorldConfig{});
    w.init();

    // Set up a swarm where BOTH role 1 and role 2 are under-supplied,
    // but role 2's deficit is larger. Total swarm = 11 (self + 10
    // neighbours). Desired 50/50 = 5.5 each. Observed: role 1 = 4, role 2 = 0.
    // Deficits: role 1 = 1.5, role 2 = 5.5. Self should pick role 2.
    populate_neighbours(w.neighbour_table(), 4, /*role=*/1);
    populate_neighbours(w.neighbour_table(), 6, /*role=*/3);  // unmanaged role

    TaskAllocSystem sys(w, two_role_params());
    sys.tick(w.registry(), w.context(), 0.1f);

    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 2u);
}

TEST_CASE("TaskAllocSystem: ratio==0 means 'do not rebalance this role'") {
    World w(WorldConfig{});
    w.init();
    populate_neighbours(w.neighbour_table(), 6, 1);

    TaskAllocSystem::Params p;
    // All ratios default to 0 → nothing to rebalance.
    p.stability_window_s = 0.0f;
    TaskAllocSystem sys(w, p);

    sys.tick(w.registry(), w.context(), 0.1f);

    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 0u);
    CHECK(sys.total_role_switches() == 0u);
}

TEST_CASE("TaskAllocSystem: empty neighbour table — self alone, just count self") {
    World w(WorldConfig{});
    w.init();

    TaskAllocSystem::Params p = two_role_params();
    TaskAllocSystem sys(w, p);
    sys.tick(w.registry(), w.context(), 0.1f);

    // Total = 1 (self in role 0). desired role 1 = 0.5, desired role 2 = 0.5.
    // Stimulus for both = 0.5, threshold = 1.0 → no switch.
    CHECK(w.registry().get<RoleComponent>(w.self_id()).role == 0u);
    CHECK(sys.total_role_switches() == 0u);
}

TEST_CASE("TaskAllocSystem: SystemDescriptor advertises the right reads/writes") {
    World w(WorldConfig{});
    w.init();
    TaskAllocSystem sys(w);
    const auto d = sys.describe();

    CHECK(d.name == "TaskAllocSystem");

    auto has_comp = [](const std::vector<mith::ComponentTypeID>& v, mith::ComponentTypeID id) {
        for (auto x : v) if (x == id) return true;
        return false;
    };
    CHECK(has_comp(d.reads_components,  mith::component_id<RoleComponent>()));
    CHECK(has_comp(d.reads_components,  mith::component_id<mith::BehaviourStateComponent>()));
    CHECK(has_comp(d.writes_components, mith::component_id<RoleComponent>()));

    auto has_res = [](const std::vector<mith::ResourceID>& v, mith::ResourceID r) {
        for (auto x : v) if (x == r) return true;
        return false;
    };
    CHECK(has_res(d.reads_resources, mith::ResourceID::NeighbourTable));
}

TEST_CASE("TaskAllocSystem: deterministic across two identical runs") {
    auto run_once = []() -> RoleID {
        World w(WorldConfig{});
        w.init();
        populate_neighbours(w.neighbour_table(), 4, 1);
        TaskAllocSystem sys(w, two_role_params());
        for (int i = 0; i < 10; ++i) sys.tick(w.registry(), w.context(), 0.1f);
        return w.registry().get<RoleComponent>(w.self_id()).role;
    };
    CHECK(run_once() == run_once());
}
