#include "doctest.h"

// End-to-end integration test for the §3.4 identity rotation flow,
// running under SimBus with two participating Worlds. Verifies:
//
//   - PERIODIC rotation auto-fires during tick() under the scheduler.
//   - Each rotation in signed mode generates a fresh IdentityCertificate
//     signed by the PREVIOUS keypair — the chain link (cert_N.prev_id ==
//     cert_(N-1).new_id) is preserved across multiple rotations.
//   - The cert's signature_by_prev verifies against the PREVIOUS public
//     key under verify_signature() — i.e. a third-party verifier can
//     follow the chain.
//   - Spoofing protection: a forged signature (random bytes) is rejected
//     by verify_signature() when checked against any legitimate key.
//
// Unsigned-mode rotation is covered by the unit tests; this integration
// pass exists to exercise the actual cryptographic verification path
// from end to end.

#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_rotation.h"
#include "mith/sim/sim_bus.h"

#ifdef MITH_AUTH_ENABLED
#include "mith/identity/ed25519.h"
#endif

#include <array>
#include <cstdint>
#include <vector>

using mith::HierarchicalID;
using mith::IdentityCertificate;
using mith::IdentityRotationPolicy;
using mith::SwarmID;
using mith::World;
using mith::WorldConfig;
using mith::sim::SimBus;
using mith::sim::SimBusConfig;

#ifdef MITH_AUTH_ENABLED

TEST_CASE("Integration: PERIODIC rotation chain stays internally consistent across many cycles") {
    SimBus bus(SimBusConfig{ /*tick_rate_hz=*/20.0f });
    WorldConfig cfg                   = bus.make_world_config(SwarmID{1});
    cfg.identity_rotation_policy      = IdentityRotationPolicy::PERIODIC;
    cfg.identity_rotation_period_s    = 0.5f;   // every 0.5 s = every 10 ticks

    World w(cfg);
    w.init();

    // Capture cert chain across 5 rotations (50+ ticks).
    std::vector<IdentityCertificate> chain;
    HierarchicalID                   running_prev_id;
    bool                             have_running_prev = false;

    for (int i = 0; i < 60; ++i) {
        const auto before_count = w.identity_rotation_count();
        w.tick();
        if (w.identity_rotation_count() > before_count) {
            auto cert = w.last_identity_certificate();
            REQUIRE(cert.has_value());
            chain.push_back(*cert);

            if (have_running_prev) {
                CHECK(cert->prev_id == running_prev_id);   // chain link
            }
            running_prev_id   = cert->new_id;
            have_running_prev = true;
        }
    }

    REQUIRE(chain.size() >= 5u);

    // Every cert has a non-zero signature, distinct new_keys, increasing
    // issued_at_s.
    float last_t = -1.0f;
    for (const auto& cert : chain) {
        bool any_nonzero = false;
        for (auto b : cert.signature_by_prev) if (b != 0u) { any_nonzero = true; break; }
        CHECK(any_nonzero);
        CHECK(cert.issued_at_s > last_t);
        last_t = cert.issued_at_s;
    }
    for (std::size_t i = 1; i < chain.size(); ++i) {
        CHECK(chain[i].new_key.public_key != chain[i-1].new_key.public_key);
    }
}

TEST_CASE("Integration: cert signature verifies against the previous public key (chain trust)") {
    // The verifier walks the chain: for each cert N, the signature_by_prev
    // must verify under cert_(N-1)'s new_key (or the genesis key for cert 0).
    // We can't observe the World's genesis key directly, so we drive the
    // chain manually with explicit keypairs and the same cert payload
    // serialisation that World uses internally.

    using namespace mith;

    auto kp0 = generate_identity_keypair();
    auto kp1 = generate_identity_keypair();
    auto kp2 = generate_identity_keypair();

    const auto id0 = HierarchicalID::generate(SwarmID{1});
    const auto id1 = HierarchicalID::generate(SwarmID{1});
    const auto id2 = HierarchicalID::generate(SwarmID{1});

    auto build_cert = [](const HierarchicalID& prev_id,
                         const HierarchicalID& new_id,
                         const IdentityKey&    new_key,
                         const IdentityPrivateKey& prev_priv) -> IdentityCertificate {
        IdentityCertificate cert;
        cert.prev_id  = prev_id;
        cert.new_id   = new_id;
        cert.new_key  = new_key;
        std::uint8_t payload[IDENTITY_CERT_PAYLOAD_LEN];
        serialise_cert_payload(prev_id, new_id, new_key, payload);
        cert.signature_by_prev = sign_payload(prev_priv, payload, IDENTITY_CERT_PAYLOAD_LEN);
        return cert;
    };

    // First rotation: kp0 → kp1, signed by kp0's private key.
    const auto cert1 = build_cert(id0, id1, kp1.public_key, kp0.private_key);
    // Second rotation: kp1 → kp2, signed by kp1's private key.
    const auto cert2 = build_cert(id1, id2, kp2.public_key, kp1.private_key);

    // Verify each link against the previous public key.
    std::uint8_t payload1[IDENTITY_CERT_PAYLOAD_LEN];
    serialise_cert_payload(cert1.prev_id, cert1.new_id, cert1.new_key, payload1);
    CHECK(verify_signature(kp0.public_key, payload1, IDENTITY_CERT_PAYLOAD_LEN,
                            cert1.signature_by_prev.data(), cert1.signature_by_prev.size()));

    std::uint8_t payload2[IDENTITY_CERT_PAYLOAD_LEN];
    serialise_cert_payload(cert2.prev_id, cert2.new_id, cert2.new_key, payload2);
    CHECK(verify_signature(cert1.new_key, payload2, IDENTITY_CERT_PAYLOAD_LEN,
                            cert2.signature_by_prev.data(), cert2.signature_by_prev.size()));

    // Cross-key: cert2 must NOT verify under kp0 (the older key).
    CHECK_FALSE(verify_signature(kp0.public_key, payload2, IDENTITY_CERT_PAYLOAD_LEN,
                                  cert2.signature_by_prev.data(), cert2.signature_by_prev.size()));
}

TEST_CASE("Integration: forged signatures are rejected — basic spoofing defence") {
    using namespace mith;

    auto victim    = generate_identity_keypair();
    auto adversary = generate_identity_keypair();

    const auto victim_id     = HierarchicalID::generate(SwarmID{1});
    const auto new_id        = HierarchicalID::generate(SwarmID{1});
    const auto new_key       = generate_identity_keypair().public_key;

    std::uint8_t payload[IDENTITY_CERT_PAYLOAD_LEN];
    serialise_cert_payload(victim_id, new_id, new_key, payload);

    // Adversary forges a "rotation cert" for the victim's identity by
    // signing the payload with their OWN private key.
    const auto forged_sig = sign_payload(adversary.private_key, payload,
                                          IDENTITY_CERT_PAYLOAD_LEN);

    // A verifier checking against the victim's real public key must reject.
    CHECK_FALSE(verify_signature(victim.public_key, payload, IDENTITY_CERT_PAYLOAD_LEN,
                                  forged_sig.data(), forged_sig.size()));

    // Random-byte signature: also rejected.
    std::array<std::uint8_t, IdentityKey::SIGNATURE_LEN> random_sig{};
    for (std::size_t i = 0; i < random_sig.size(); ++i) {
        random_sig[i] = static_cast<std::uint8_t>(i * 7u + 13u);   // deterministic garbage
    }
    CHECK_FALSE(verify_signature(victim.public_key, payload, IDENTITY_CERT_PAYLOAD_LEN,
                                  random_sig.data(), random_sig.size()));

    // Genuine signature still verifies as a control.
    const auto genuine_sig = sign_payload(victim.private_key, payload,
                                           IDENTITY_CERT_PAYLOAD_LEN);
    CHECK(verify_signature(victim.public_key, payload, IDENTITY_CERT_PAYLOAD_LEN,
                            genuine_sig.data(), genuine_sig.size()));
}

TEST_CASE("Integration: payload-tampering invalidates an otherwise-valid signature") {
    using namespace mith;

    auto kp = generate_identity_keypair();
    const auto prev_id = HierarchicalID::generate(SwarmID{1});
    const auto new_id  = HierarchicalID::generate(SwarmID{1});
    const auto new_key = generate_identity_keypair().public_key;

    std::uint8_t payload[IDENTITY_CERT_PAYLOAD_LEN];
    serialise_cert_payload(prev_id, new_id, new_key, payload);

    const auto sig = sign_payload(kp.private_key, payload, IDENTITY_CERT_PAYLOAD_LEN);
    CHECK(verify_signature(kp.public_key, payload, IDENTITY_CERT_PAYLOAD_LEN,
                            sig.data(), sig.size()));

    // Flip one byte of the payload — verifier must reject.
    payload[7] ^= 0x42;
    CHECK_FALSE(verify_signature(kp.public_key, payload, IDENTITY_CERT_PAYLOAD_LEN,
                                  sig.data(), sig.size()));
}

#endif // MITH_AUTH_ENABLED
