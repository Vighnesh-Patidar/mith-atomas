#include "doctest.h"

#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"

#include <cstdint>

using mith::EntityRegistry;
using mith::RegistrationStatus;

TEST_CASE("every built-in component is a well-formed HotComponent") {
    static_assert(mith::is_hot_component_v<mith::IdentityComponent>);
    static_assert(mith::is_hot_component_v<mith::PositionComponent>);
    static_assert(mith::is_hot_component_v<mith::VelocityComponent>);
    static_assert(mith::is_hot_component_v<mith::OrientationComponent>);
    static_assert(mith::is_hot_component_v<mith::HealthComponent>);
    static_assert(mith::is_hot_component_v<mith::RoleComponent>);
    static_assert(mith::is_hot_component_v<mith::BehaviourStateComponent>);
    static_assert(mith::is_hot_component_v<mith::PermissionMaskComponent>);

    static_assert(mith::is_well_formed_component_v<mith::IdentityComponent>);
    static_assert(mith::is_well_formed_component_v<mith::PermissionMaskComponent>);
}

TEST_CASE("default values match §4.4 documented defaults") {
    mith::IdentityComponent id;
    CHECK(id.id.swarm_id == 0u);
    CHECK(id.id.unit_id.is_nil());

    mith::PositionComponent pos;
    CHECK(pos.x == 0.0f);
    CHECK(pos.y == 0.0f);
    CHECK(pos.z == 0.0f);

    mith::VelocityComponent vel;
    CHECK(vel.vx == 0.0f);
    CHECK(vel.vy == 0.0f);
    CHECK(vel.vz == 0.0f);

    mith::OrientationComponent ori;
    CHECK(ori.qw == 1.0f);    // identity quaternion
    CHECK(ori.qx == 0.0f);
    CHECK(ori.qy == 0.0f);
    CHECK(ori.qz == 0.0f);

    mith::HealthComponent h;
    CHECK(h.value == 100u);

    mith::RoleComponent r;
    CHECK(r.role == 0u);

    mith::BehaviourStateComponent bs;
    CHECK(bs.state == 0u);

    mith::PermissionMaskComponent pm;
    CHECK(pm.allowed_builtins == 0xFFFFFFFFu);
    CHECK(pm.allow_user_actions == true);
}

TEST_CASE("convenience constructors populate fields") {
    mith::PositionComponent pos{1.0f, 2.0f, 3.0f};
    CHECK(pos.x == 1.0f);
    CHECK(pos.y == 2.0f);
    CHECK(pos.z == 3.0f);

    mith::VelocityComponent vel{4.0f, 5.0f, 6.0f};
    CHECK(vel.vz == 6.0f);

    mith::OrientationComponent ori{0.7071f, 0.0f, 0.7071f, 0.0f};
    CHECK(ori.qw == doctest::Approx(0.7071f));
    CHECK(ori.qy == doctest::Approx(0.7071f));

    mith::HealthComponent h{50u};
    CHECK(h.value == 50u);

    mith::RoleComponent r{42u};
    CHECK(r.role == 42u);

    mith::BehaviourStateComponent bs{7u};
    CHECK(bs.state == 7u);
}

TEST_CASE("IdentityComponent carries a HierarchicalID") {
    const auto hid = mith::HierarchicalID::generate(mith::SwarmID{0x1234});
    mith::IdentityComponent comp{hid};
    CHECK(comp.id.swarm_id == 0x1234u);
    CHECK(comp.id.unit_id.version() == 4u);
    CHECK(comp.id == hid);
}

TEST_CASE("PermissionMaskComponent default: every built-in is allowed") {
    mith::PermissionMaskComponent pm;
    CHECK(pm.allows(mith::actions::IDLE));
    CHECK(pm.allows(mith::actions::MOVE));
    CHECK(pm.allows(mith::actions::HOVER));
    CHECK(pm.allows(mith::actions::TRANSMIT));
    CHECK(pm.allows(mith::actions::SCAN));
    CHECK(pm.allows(mith::actions::REGROUP));
    CHECK(pm.allows(mith::actions::FOLLOW));
}

TEST_CASE("PermissionMaskComponent default: user actions allowed") {
    mith::PermissionMaskComponent pm;
    CHECK(pm.allows(mith::actions::CUSTOM));
    CHECK(pm.allows(mith::actions::CUSTOM + 42u));
    CHECK(pm.allows(0xFFFFFFFFu));
}

TEST_CASE("PermissionMaskComponent: degraded-mode mask matches §13.2 spec") {
    // §13.2 — when degraded, allowed_builtins = IDLE|REGROUP, allow_user = false
    constexpr std::uint32_t degraded =
        (1u << mith::actions::IDLE) | (1u << mith::actions::REGROUP);
    mith::PermissionMaskComponent pm{degraded, /*allow_user=*/false};

    CHECK(pm.allows(mith::actions::IDLE));
    CHECK(pm.allows(mith::actions::REGROUP));

    CHECK_FALSE(pm.allows(mith::actions::MOVE));
    CHECK_FALSE(pm.allows(mith::actions::HOVER));
    CHECK_FALSE(pm.allows(mith::actions::TRANSMIT));
    CHECK_FALSE(pm.allows(mith::actions::SCAN));
    CHECK_FALSE(pm.allows(mith::actions::FOLLOW));

    CHECK_FALSE(pm.allows(mith::actions::CUSTOM));
    CHECK_FALSE(pm.allows(mith::actions::CUSTOM + 1u));
}

TEST_CASE("PermissionMaskComponent: reserved range (32..CUSTOM-1) always forbidden") {
    mith::PermissionMaskComponent pm;   // fully permissive default
    CHECK_FALSE(pm.allows(32u));
    CHECK_FALSE(pm.allows(100u));
    CHECK_FALSE(pm.allows(mith::actions::CUSTOM - 1u));
    // CUSTOM and above are user range — gated by allow_user_actions.
    CHECK(pm.allows(mith::actions::CUSTOM));
}

TEST_CASE("PermissionMaskComponent: allow_user_actions toggles user range only") {
    mith::PermissionMaskComponent pm{0xFFFFFFFFu, /*allow_user=*/false};
    CHECK(pm.allows(mith::actions::IDLE));      // built-ins unaffected
    CHECK(pm.allows(mith::actions::FOLLOW));
    CHECK_FALSE(pm.allows(mith::actions::CUSTOM));
    CHECK_FALSE(pm.allows(mith::actions::CUSTOM + 1u));
}

TEST_CASE("all eight built-in components register via the privileged path") {
    EntityRegistry reg;

    REQUIRE(reg.register_builtin_component<mith::IdentityComponent>()        == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::PositionComponent>()        == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::VelocityComponent>()        == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::OrientationComponent>()     == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::HealthComponent>()          == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::RoleComponent>()            == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::BehaviourStateComponent>()  == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::PermissionMaskComponent>()  == RegistrationStatus::Ok);

    CHECK(reg.registered_count() == 8u);

    CHECK(reg.origin_of<mith::IdentityComponent>()       == mith::ComponentOrigin::Built_In);
    CHECK(reg.origin_of<mith::PositionComponent>()       == mith::ComponentOrigin::Built_In);
    CHECK(reg.origin_of<mith::PermissionMaskComponent>() == mith::ComponentOrigin::Built_In);
}

TEST_CASE("built-in components round-trip through emplace / get") {
    EntityRegistry reg;
    REQUIRE(reg.register_builtin_component<mith::IdentityComponent>()        == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::PositionComponent>()        == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::VelocityComponent>()        == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::OrientationComponent>()     == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::HealthComponent>()          == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::RoleComponent>()            == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::BehaviourStateComponent>()  == RegistrationStatus::Ok);
    REQUIRE(reg.register_builtin_component<mith::PermissionMaskComponent>()  == RegistrationStatus::Ok);

    const auto self = reg.self_id();
    const auto hid  = mith::HierarchicalID::generate(mith::SwarmID{0x42});

    reg.emplace<mith::IdentityComponent>      (self, mith::IdentityComponent{hid});
    reg.emplace<mith::PositionComponent>      (self, mith::PositionComponent{1.0f, 2.0f, 3.0f});
    reg.emplace<mith::VelocityComponent>      (self, mith::VelocityComponent{0.1f, 0.2f, 0.3f});
    reg.emplace<mith::OrientationComponent>   (self, mith::OrientationComponent{1.0f, 0.0f, 0.0f, 0.0f});
    reg.emplace<mith::HealthComponent>        (self, mith::HealthComponent{75u});
    reg.emplace<mith::RoleComponent>          (self, mith::RoleComponent{2u});
    reg.emplace<mith::BehaviourStateComponent>(self, mith::BehaviourStateComponent{5u});
    reg.emplace<mith::PermissionMaskComponent>(self, mith::PermissionMaskComponent{});

    CHECK(reg.get<mith::IdentityComponent>(self).id          == hid);
    CHECK(reg.get<mith::PositionComponent>(self).x           == 1.0f);
    CHECK(reg.get<mith::PositionComponent>(self).z           == 3.0f);
    CHECK(reg.get<mith::VelocityComponent>(self).vz          == doctest::Approx(0.3f));
    CHECK(reg.get<mith::OrientationComponent>(self).qw       == 1.0f);
    CHECK(reg.get<mith::HealthComponent>(self).value         == 75u);
    CHECK(reg.get<mith::RoleComponent>(self).role            == 2u);
    CHECK(reg.get<mith::BehaviourStateComponent>(self).state == 5u);
    CHECK(reg.get<mith::PermissionMaskComponent>(self).allow_user_actions);
}

TEST_CASE("mutating a component in place via get is visible on the next read") {
    EntityRegistry reg;
    REQUIRE(reg.register_builtin_component<mith::HealthComponent>() == RegistrationStatus::Ok);
    reg.emplace<mith::HealthComponent>(reg.self_id(), mith::HealthComponent{100u});

    reg.get<mith::HealthComponent>(reg.self_id()).value = 40u;
    CHECK(reg.get<mith::HealthComponent>(reg.self_id()).value == 40u);
}

TEST_CASE("ActionTypeID built-in constants are stable and distinct") {
    static_assert(mith::actions::IDLE     == 0u);
    static_assert(mith::actions::MOVE     == 1u);
    static_assert(mith::actions::HOVER    == 2u);
    static_assert(mith::actions::TRANSMIT == 3u);
    static_assert(mith::actions::SCAN     == 4u);
    static_assert(mith::actions::REGROUP  == 5u);
    static_assert(mith::actions::FOLLOW   == 6u);
    static_assert(mith::actions::CUSTOM   == 0x1000u);
    static_assert(mith::actions::IDLE     < mith::actions::CUSTOM);
    static_assert(mith::actions::BUILTIN_MAX == 31u);
}
