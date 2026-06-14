#include "doctest.h"

#include "mith/comms/state_vector.h"
#include "mith/identity/hierarchical_id.h"

#include <cstdint>
#include <type_traits>

using mith::HierarchicalID;
using mith::StateVector;
using mith::SwarmID;

TEST_CASE("default-constructed StateVector: nil id, default components, tick=0") {
    StateVector sv;
    CHECK(sv.id.swarm_id == 0u);
    CHECK(sv.id.unit_id.is_nil());
    CHECK(sv.position.x == 0.0f);
    CHECK(sv.velocity.vx == 0.0f);
    CHECK(sv.health.value == 100u);    // built-in default
    CHECK(sv.role.role == 0u);
    CHECK(sv.state.state == 0u);
    CHECK(sv.tick == 0u);
}

TEST_CASE("StateVector fields are individually settable") {
    StateVector sv;
    sv.id        = HierarchicalID::generate(SwarmID{0x42});
    sv.position  = mith::PositionComponent{1.0f, 2.0f, 3.0f};
    sv.velocity  = mith::VelocityComponent{0.1f, 0.2f, 0.3f};
    sv.health    = mith::HealthComponent{75u};
    sv.role      = mith::RoleComponent{2u};
    sv.state     = mith::BehaviourStateComponent{5u};
    sv.tick      = 1234;

    CHECK(sv.id.swarm_id == 0x42u);
    CHECK(sv.position.z == 3.0f);
    CHECK(sv.velocity.vy == doctest::Approx(0.2f));
    CHECK(sv.health.value == 75u);
    CHECK(sv.role.role == 2u);
    CHECK(sv.state.state == 5u);
    CHECK(sv.tick == 1234u);
}

TEST_CASE("StateVector is trivially copyable / movable") {
    static_assert(std::is_default_constructible_v<StateVector>);
    static_assert(std::is_nothrow_move_constructible_v<StateVector>);
    static_assert(std::is_nothrow_move_assignable_v<StateVector>);
    static_assert(std::is_copy_constructible_v<StateVector>);
}

TEST_CASE("StateVector size is reasonable for a beacon payload") {
    // §7.2 docs target ~150 bytes once the v0.3 sender_pubkey (32) +
    // signature (64) are accounted for. With padding this stays well
    // under 200 bytes — fits in a single UDP datagram with headers.
    static_assert(sizeof(StateVector) <= 200u);
}
