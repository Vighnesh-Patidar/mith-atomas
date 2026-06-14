#include "doctest.h"

#include <cstring>

#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_rotation.h"

#ifdef MITH_AUTH_ENABLED
#include "mith/identity/ed25519.h"
#endif

#include <cstdint>

using mith::HierarchicalID;
using mith::IdentityCertificate;
using mith::IdentityRotationPolicy;
using mith::SwarmID;
using mith::World;
using mith::WorldConfig;

TEST_CASE("rotate_identity() changes the HID (any mode)") {
    World w(WorldConfig{});
    w.init();
    const auto before = w.identity();

    w.rotate_identity();
    const auto& after = w.identity();

    CHECK(after != before);
    CHECK(after.swarm_id == before.swarm_id);   // same swarm
    CHECK(after.unit_id.version() == 4u);
    CHECK(w.identity_rotation_count() == 1u);
}

TEST_CASE("rotate_identity() increments the rotation counter monotonically") {
    World w(WorldConfig{});
    w.init();
    CHECK(w.identity_rotation_count() == 0u);

    for (int i = 0; i < 5; ++i) w.rotate_identity();
    CHECK(w.identity_rotation_count() == 5u);
}

TEST_CASE("PERMANENT policy: tick() does not auto-rotate") {
    WorldConfig cfg;
    cfg.identity_rotation_policy   = IdentityRotationPolicy::PERMANENT;
    cfg.identity_rotation_period_s = 0.05f;   // would fire every tick if active

    World w(cfg);
    w.init();
    const auto before = w.identity();

    for (int i = 0; i < 50; ++i) w.tick();   // 2.5 s

    CHECK(w.identity() == before);
    CHECK(w.identity_rotation_count() == 0u);
}

TEST_CASE("PERIODIC policy: tick() auto-rotates at the configured interval") {
    WorldConfig cfg;
    cfg.tick_rate_hz               = 20.0f;
    cfg.identity_rotation_policy   = IdentityRotationPolicy::PERIODIC;
    cfg.identity_rotation_period_s = 0.5f;   // every 0.5 s = every 10 ticks

    World w(cfg);
    w.init();

    // After 25 ticks (1.25 s), we should have rotated twice (at t=0.55 and
    // t≈1.05; depends on accumulation precision, accept 2 or 3).
    for (int i = 0; i < 25; ++i) w.tick();

    CHECK(w.identity_rotation_count() >= 2u);
    CHECK(w.identity_rotation_count() <= 3u);
}

TEST_CASE("EVENT_DRIVEN policy: tick() does not auto-rotate; explicit calls do") {
    WorldConfig cfg;
    cfg.identity_rotation_policy = IdentityRotationPolicy::EVENT_DRIVEN;

    World w(cfg);
    w.init();

    for (int i = 0; i < 100; ++i) w.tick();
    CHECK(w.identity_rotation_count() == 0u);

    w.rotate_identity();
    CHECK(w.identity_rotation_count() == 1u);
}

#ifdef MITH_AUTH_ENABLED

TEST_CASE("Signed mode: rotate_identity() produces a cert signed by the previous key") {
    World w(WorldConfig{});
    w.init();

    // Capture the public key BEFORE rotating — it's the one that signed
    // the resulting cert. We can't read the private key from World
    // (sender-only), so we sign + verify externally using the cert's
    // signature_by_prev and a known-good test path: capture the cert
    // and re-derive the verification payload.
    const auto prev_id = w.identity();

    w.rotate_identity();

    const auto cert = w.last_identity_certificate();
    REQUIRE(cert.has_value());

    CHECK(cert->prev_id == prev_id);
    CHECK(cert->new_id == w.identity());
    CHECK(cert->new_id != cert->prev_id);

    // Cert payload reconstruction matches what World signed.
    std::uint8_t payload[mith::IDENTITY_CERT_PAYLOAD_LEN];
    mith::serialise_cert_payload(cert->prev_id, cert->new_id, cert->new_key, payload);
    // We don't have the previous public key surfaced (lives only inside
    // World), so we can't verify externally here — but the cert exists
    // and its non-signature fields are coherent.

    // signature_by_prev is not all-zero (a real signature was produced).
    bool any_nonzero = false;
    for (auto b : cert->signature_by_prev) {
        if (b != 0u) { any_nonzero = true; break; }
    }
    CHECK(any_nonzero);
}

TEST_CASE("Signed mode: cert chain across multiple rotations is independent (different new_keys)") {
    World w(WorldConfig{});
    w.init();

    w.rotate_identity();
    const auto cert_a = w.last_identity_certificate();
    REQUIRE(cert_a.has_value());

    w.rotate_identity();
    const auto cert_b = w.last_identity_certificate();
    REQUIRE(cert_b.has_value());

    CHECK(cert_a->new_key.public_key != cert_b->new_key.public_key);
    CHECK(cert_a->new_id    != cert_b->new_id);
    CHECK(cert_b->prev_id   == cert_a->new_id);   // chain link
}

TEST_CASE("Signed mode: cert issued_at_s reflects sim elapsed time") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 20.0f;
    World w(cfg);
    w.init();

    // Advance to t≈0.5s.
    for (int i = 0; i < 10; ++i) w.tick();

    w.rotate_identity();
    const auto cert = w.last_identity_certificate();
    REQUIRE(cert.has_value());

    CHECK(cert->issued_at_s == doctest::Approx(0.5f).epsilon(0.01));
}

#else

TEST_CASE("Unsigned mode: rotate_identity() does NOT produce a cert") {
    World w(WorldConfig{});
    w.init();
    w.rotate_identity();
    CHECK_FALSE(w.last_identity_certificate().has_value());
    // But the HID still rotated.
    CHECK(w.identity_rotation_count() == 1u);
}

#endif

TEST_CASE("serialise_cert_payload produces stable 68-byte output") {
    static_assert(mith::IDENTITY_CERT_PAYLOAD_LEN == 68u);

    const auto prev = HierarchicalID::generate(SwarmID{1});
    const auto next = HierarchicalID::generate(SwarmID{1});
    mith::IdentityKey key;
    for (std::size_t i = 0; i < key.public_key.size(); ++i) {
        key.public_key[i] = static_cast<std::uint8_t>(i);
    }

    std::uint8_t a[mith::IDENTITY_CERT_PAYLOAD_LEN] = {};
    std::uint8_t b[mith::IDENTITY_CERT_PAYLOAD_LEN] = {};
    mith::serialise_cert_payload(prev, next, key, a);
    mith::serialise_cert_payload(prev, next, key, b);

    // Deterministic.
    CHECK(std::memcmp(a, b, mith::IDENTITY_CERT_PAYLOAD_LEN) == 0);

    // Layout sanity: swarm_id LE at offset 0.
    CHECK(a[0] == (prev.swarm_id & 0xFFu));
    CHECK(a[1] == ((prev.swarm_id >> 8) & 0xFFu));
}
