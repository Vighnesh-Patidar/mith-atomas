#pragma once

// Identity rotation policy — see ARCHITECTURE.md §3.4
//
// Selects how (and when) a robot's UnitID changes during a deployment.
// v0.1 ships only PERMANENT as functional; the other policies' enum
// values exist for API stability and are honoured starting in v0.2
// when cryptographic identity (§3.3) lands.
//
//   PERMANENT     UnitID generated once at first boot; never changes.
//                 Default. Suitable for research, sim, fleet management,
//                 industrial deployments where stable IDs matter for
//                 logging, audit, debugging.
//
//   PER_MISSION   Rotate on SwarmID change or mission boundary. Useful
//                 for mission compartmentalisation. Signed mode only
//                 (rotation in unsigned mode just leaks metadata
//                 without buying security).
//
//   PERIODIC      Rotate every WorldConfig::identity_rotation_period_s
//                 seconds. For privacy / anti-tracking in long-running
//                 deployments. Signed mode only.
//
//   EVENT_DRIVEN  Application calls World::rotate_identity() to trigger.
//                 For compromise response. Signed mode only.
//
// In signed mode (v0.2), rotations issue an IdentityCertificate signed by
// the previous key (continuity proof) so neighbours can correlate
// old→new identity without losing reputation / task history. Pseudonymous
// rotation (no continuity) lands as a separate option.

#include <cstdint>

namespace mith {

enum class IdentityRotationPolicy : std::uint8_t {
    PERMANENT     = 0,
    PER_MISSION   = 1,
    PERIODIC      = 2,
    EVENT_DRIVEN  = 3,
};

} // namespace mith

#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_auth.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace mith {

// Continuity proof binding a rotated identity to its predecessor (§3.4).
// In signed mode, World::rotate_identity() generates a fresh keypair,
// signs this cert with the OLD private key, and publishes it via the
// next beacon so neighbours can correlate old→new without losing
// reputation / task history.
//
// In unsigned mode, no cert is produced (signature_by_prev is unused —
// World::last_identity_certificate() returns nullopt).
struct IdentityCertificate {
    HierarchicalID                              prev_id;
    HierarchicalID                              new_id;
    IdentityKey                                 new_key;            // pubkey of the rotated identity
    std::array<std::uint8_t,
               IdentityKey::SIGNATURE_LEN>      signature_by_prev{}; // signed by PREVIOUS private key
    float                                       issued_at_s = 0.0f;
};

// Byte layout of the cert's signed payload — what sign_payload() sees.
// Stable across implementations so verify_signature() reconstructs it
// the same way.
//   2 bytes  prev_id.swarm_id (little-endian)
//  16 bytes  prev_id.unit_id  (raw)
//   2 bytes  new_id.swarm_id  (little-endian)
//  16 bytes  new_id.unit_id   (raw)
//  32 bytes  new_key.public_key
//  = 68 bytes total
inline constexpr std::size_t IDENTITY_CERT_PAYLOAD_LEN = 2 + 16 + 2 + 16 + 32;

// Serialise the cert's signed payload into a fixed-size buffer.
// Exposed so tests / verifiers can reconstruct the bytes that
// signature_by_prev covers.
void serialise_cert_payload(const HierarchicalID& prev_id,
                            const HierarchicalID& new_id,
                            const IdentityKey&    new_key,
                            std::uint8_t          out[IDENTITY_CERT_PAYLOAD_LEN]) noexcept;

} // namespace mith
