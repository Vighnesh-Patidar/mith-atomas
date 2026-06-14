#pragma once

// PeerKeyRegistry — see ARCHITECTURE.md §3.3, §16 v0.3
//
// Swarm-wide trust-on-first-use cache mapping HierarchicalID → IdentityKey.
// Multiple systems contribute to and consume from this single registry:
//
//   - DiscoverySystem PINS keys it learns from DISCOVERY_HELLO and
//     DISCOVERY_WELCOME messages (sender embeds pubkey in payload[0..31]
//     in signed mode).
//   - BeaconSystem PINS keys observed on first signed beacon from a new
//     HID, and CONSULTS the registry to reject subsequent beacons whose
//     pubkey doesn't match the pinned record.
//
// The registry is per-World (no cross-World sharing in v0.3) and lives
// for World's lifetime. Always present in the struct so callers don't
// need conditional access; only meaningful when MITH_AUTH_ENABLED runs
// the signed paths.

#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_auth.h"

#include <cstddef>
#include <map>
#include <optional>

namespace mith {

class PeerKeyRegistry {
public:
    // Pin (hid, key) if hid is not already in the registry, or if it's
    // already pinned with the same key. Returns:
    //   true  — registry now holds (hid, key). Either the pin was new
    //           or the existing pin matched.
    //   false — hid was already pinned to a DIFFERENT key. TOFU violation
    //           — the existing pin is preserved.
    bool try_pin(const HierarchicalID& hid, const IdentityKey& key);

    // Lookup. Returns nullptr if no pin for `hid`.
    const IdentityKey* find(const HierarchicalID& hid) const noexcept;

    // Total pins currently held.
    std::size_t size() const noexcept;

    // Drop all pins. Intended for tests; production code should not
    // need to clear the registry (rotated identities are NEW HIDs).
    void clear() noexcept;

private:
    std::map<HierarchicalID, IdentityKey> keys_;
};

} // namespace mith
