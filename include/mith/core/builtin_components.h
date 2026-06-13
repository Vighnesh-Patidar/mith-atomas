#pragma once

// Built-in hot components — see ARCHITECTURE.md §4.4
//
// These types are registered by the runtime in World::init() via the
// privileged register_builtin_component<T>() path (§4.1, §13.5). User
// code accesses them through the same registry API as user-defined
// components (registry.get<T>(), etc.); the only distinction is the
// ComponentOrigin tag.
//
// All components in this header are POD-style: trivially copyable, noexcept-
// default-constructible, no allocations. They are intended as transparent
// state on the self entity — systems read and write them directly.
//
// Components that depend on §6 (Action) or §7 (Message) — ActionQueueComponent
// and CommBufferComponent — land alongside those sections' implementation.

#include "mith/behaviour/action_type.h"
#include "mith/core/component.h"
#include "mith/identity/hierarchical_id.h"

#include <cstdint>

namespace mith {

// §4.4 — set once at init. The robot's swarm-scoped identity.
struct IdentityComponent : HotComponent<IdentityComponent> {
    HierarchicalID id;

    constexpr IdentityComponent() noexcept = default;
    constexpr explicit IdentityComponent(HierarchicalID hid) noexcept
        : id(hid) {}
};

// §4.4 — world-frame position. Units up to deployment; convention: metres.
struct PositionComponent : HotComponent<PositionComponent> {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr PositionComponent() noexcept = default;
    constexpr PositionComponent(float x_, float y_, float z_) noexcept
        : x(x_), y(y_), z(z_) {}
};

// §4.4 — current velocity. Same frame as PositionComponent. m/s.
struct VelocityComponent : HotComponent<VelocityComponent> {
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

    constexpr VelocityComponent() noexcept = default;
    constexpr VelocityComponent(float vx_, float vy_, float vz_) noexcept
        : vx(vx_), vy(vy_), vz(vz_) {}
};

// §4.4 — orientation as a unit quaternion. Default = identity (no rotation).
struct OrientationComponent : HotComponent<OrientationComponent> {
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;

    constexpr OrientationComponent() noexcept = default;
    constexpr OrientationComponent(float w, float x, float y, float z) noexcept
        : qw(w), qx(x), qy(y), qz(z) {}
};

// §4.4 — managed by FaultMonitorSystem (§13.1, v0.2). 0..100.
struct HealthComponent : HotComponent<HealthComponent> {
    std::uint8_t value = 100;

    constexpr HealthComponent() noexcept = default;
    constexpr explicit HealthComponent(std::uint8_t v) noexcept : value(v) {}
};

// Role and state IDs are opaque to the runtime — mission code defines the
// enumerations. Both default to 0, conventionally "no role" / "initial state".
using RoleID  = std::uint32_t;
using StateID = std::uint32_t;

// §4.4 — current swarm role (worker, scout, relay, ...).
struct RoleComponent : HotComponent<RoleComponent> {
    RoleID role = 0;

    constexpr RoleComponent() noexcept = default;
    constexpr explicit RoleComponent(RoleID r) noexcept : role(r) {}
};

// §4.4 — current state for the user's behaviour state machine. Read by
// ActionProvider (§6.2) via EntitySnapshot.
struct BehaviourStateComponent : HotComponent<BehaviourStateComponent> {
    StateID state = 0;

    constexpr BehaviourStateComponent() noexcept = default;
    constexpr explicit BehaviourStateComponent(StateID s) noexcept
        : state(s) {}
};

// §4.4, §13.2 — gates which ActionTypeIDs ActionValidatorSystem (§6.4) will
// accept. Written only by FaultMonitorSystem (§13.2, v0.2); read by the
// validator each tick. Default is fully permissive.
//
//   allowed_builtins   Bit N maps to ActionTypeID N (0..31). Bits 0..6 cover
//                      the named built-ins (IDLE..FOLLOW); 7..31 are reserved
//                      for future built-ins. Default 0xFFFFFFFF.
//
//   allow_user_actions Coarse-grained gate for ActionTypeID >= actions::CUSTOM.
//                      Default true. Finer-grained user-action permissioning is
//                      the application's responsibility — gate inside the
//                      handler.
struct PermissionMaskComponent : HotComponent<PermissionMaskComponent> {
    std::uint32_t allowed_builtins   = 0xFFFFFFFFu;
    bool          allow_user_actions = true;

    constexpr PermissionMaskComponent() noexcept = default;

    constexpr PermissionMaskComponent(std::uint32_t builtins,
                                       bool          user_actions) noexcept
        : allowed_builtins(builtins), allow_user_actions(user_actions) {}

    // Is t allowed under this mask?
    //   t in [0..31]                — checked against allowed_builtins
    //   t in [32..actions::CUSTOM)  — reserved range; forbidden by default
    //                                 (no built-in has been allocated here)
    //   t >= actions::CUSTOM        — gated by allow_user_actions
    constexpr bool allows(ActionTypeID t) const noexcept {
        if (t >= actions::CUSTOM)     return allow_user_actions;
        if (t > actions::BUILTIN_MAX) return false;
        return (allowed_builtins & (1u << t)) != 0u;
    }
};

} // namespace mith
