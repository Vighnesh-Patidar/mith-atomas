#include "doctest.h"

#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_auth.h"
#include "mith/identity/identity_rotation.h"

#include <cstdint>
#include <type_traits>

using mith::HierarchicalID;
using mith::IdentityKey;
using mith::IdentityRotationPolicy;
using mith::IdentityVerifier;
using mith::NoopIdentityVerifier;
using mith::SwarmID;
using mith::World;
using mith::WorldConfig;

// ------------------------------------------------------------------------
// IdentityKey — §3.3
// ------------------------------------------------------------------------

TEST_CASE("IdentityKey constants match Ed25519 sizes") {
    static_assert(IdentityKey::PUBLIC_KEY_LEN == 32u);
    static_assert(IdentityKey::SIGNATURE_LEN  == 64u);
}

TEST_CASE("IdentityKey default construction zero-initialises the public key") {
    IdentityKey k;
    CHECK(k.public_key.size() == IdentityKey::PUBLIC_KEY_LEN);
    for (auto b : k.public_key) CHECK(b == 0u);
}

TEST_CASE("IdentityKey fields can be set") {
    IdentityKey k;
    for (std::size_t i = 0; i < IdentityKey::PUBLIC_KEY_LEN; ++i) {
        k.public_key[i] = static_cast<std::uint8_t>(i);
    }
    CHECK(k.public_key[0] == 0u);
    CHECK(k.public_key[15] == 15u);
    CHECK(k.public_key[31] == 31u);
}

TEST_CASE("IdentityKey is trivially copyable for forward use") {
    static_assert(std::is_default_constructible_v<IdentityKey>);
    static_assert(std::is_nothrow_move_constructible_v<IdentityKey>);
    static_assert(std::is_nothrow_move_assignable_v<IdentityKey>);
    static_assert(std::is_copy_constructible_v<IdentityKey>);
}

// ------------------------------------------------------------------------
// IdentityVerifier interface + NoopIdentityVerifier — §3.3
// ------------------------------------------------------------------------

TEST_CASE("NoopIdentityVerifier accepts any payload + signature combination") {
    NoopIdentityVerifier v;
    const auto hid = HierarchicalID::generate(SwarmID{1});

    const std::uint8_t payload[]   = {1, 2, 3, 4};
    const std::uint8_t signature[] = {0xDE, 0xAD, 0xBE, 0xEF};

    CHECK(v.verify(hid, payload, sizeof(payload), signature, sizeof(signature)));

    // Empty payload + signature also accepted.
    CHECK(v.verify(hid, nullptr, 0, nullptr, 0));

    // Wrong "claimed" identity — still accepted (no auth in unsigned mode).
    const auto other = HierarchicalID::generate(SwarmID{99});
    CHECK(v.verify(other, payload, sizeof(payload), signature, sizeof(signature)));
}

TEST_CASE("IdentityVerifier interface is usable as an abstract base") {
    // Custom deny-all verifier for tests that want to simulate signed-mode
    // rejection without the Ed25519 impl (v0.2).
    struct DenyAllVerifier : IdentityVerifier {
        bool verify(const HierarchicalID&,
                    const std::uint8_t*, std::size_t,
                    const std::uint8_t*, std::size_t) const noexcept override {
            return false;
        }
    };

    DenyAllVerifier deny;
    NoopIdentityVerifier accept;

    const auto hid = HierarchicalID::generate(SwarmID{1});
    const std::uint8_t buf[] = {1};

    CHECK_FALSE(deny.verify(hid, buf, sizeof(buf), buf, sizeof(buf)));
    CHECK(accept.verify(hid, buf, sizeof(buf), buf, sizeof(buf)));

    // Both satisfy the interface — IdentityVerifier* can hold either.
    IdentityVerifier* poly = &deny;
    CHECK_FALSE(poly->verify(hid, buf, sizeof(buf), buf, sizeof(buf)));
    poly = &accept;
    CHECK(poly->verify(hid, buf, sizeof(buf), buf, sizeof(buf)));
}

// ------------------------------------------------------------------------
// IdentityRotationPolicy — §3.4
// ------------------------------------------------------------------------

TEST_CASE("IdentityRotationPolicy values are stable") {
    static_assert(static_cast<std::uint8_t>(IdentityRotationPolicy::PERMANENT)    == 0u);
    static_assert(static_cast<std::uint8_t>(IdentityRotationPolicy::PER_MISSION)  == 1u);
    static_assert(static_cast<std::uint8_t>(IdentityRotationPolicy::PERIODIC)     == 2u);
    static_assert(static_cast<std::uint8_t>(IdentityRotationPolicy::EVENT_DRIVEN) == 3u);
}

TEST_CASE("WorldConfig::identity_rotation_policy defaults to PERMANENT") {
    WorldConfig cfg;
    CHECK(cfg.identity_rotation_policy == IdentityRotationPolicy::PERMANENT);
}

TEST_CASE("WorldConfig::identity_rotation_policy is user-settable") {
    WorldConfig cfg;
    cfg.identity_rotation_policy = IdentityRotationPolicy::PERIODIC;

    World w(cfg);
    CHECK(w.config().identity_rotation_policy == IdentityRotationPolicy::PERIODIC);
}

// Behavioural tests for rotate_identity() moved to test_identity_rotation.cpp
// in the v0.2 rotation slice — the v0.1 "no-op stub" contract is gone.
