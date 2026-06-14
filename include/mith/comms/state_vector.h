#pragma once

// StateVector — see ARCHITECTURE.md §7.2
//
// Beacon payload. Fixed schema, compact (~64 bytes), serialisable to a
// single UDP datagram or radio frame. Periodically broadcast by
// BeaconSystem (§5.3); deserialised on receipt and merged into the
// recipient's NeighbourTable (§7.4) as a fresh observation.
//
// Field set is deliberately narrow — only what neighbours need to model
// each other. ActionQueue, CommBuffer, PermissionMask, IdentityKey are
// NOT shipped over beacons; they're not part of "what a neighbour knows
// about you."

#include "mith/core/builtin_components.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_auth.h"

#include <array>
#include <cstdint>

namespace mith {

// Beacon payload (§7.2). Always carries the sender's public key and a
// signature slot regardless of build mode — uniform layout keeps SimBus
// + UDP transport interchangeable. In unsigned-mode builds and when
// running the unsigned beacon path, both fields are zero-filled and
// receivers skip verification. In signed mode (MITH_AUTH_ENABLED + a
// keypair on the sender) the fields are populated by BeaconSystem
// before send, and verified on receive against a TOFU per-HID cache.
struct StateVector {
    HierarchicalID          id;
    PositionComponent       position;
    VelocityComponent       velocity;
    HealthComponent         health;
    RoleComponent           role;
    BehaviourStateComponent state;
    std::uint32_t           tick = 0;
    IdentityKey             sender_pubkey{};
    std::array<std::uint8_t, IdentityKey::SIGNATURE_LEN> signature{};
};

} // namespace mith
