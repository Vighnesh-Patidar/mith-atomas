#include "doctest.h"

#include "mith/comms/peer_key_registry.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/identity/identity_auth.h"

#include <cstdint>

using mith::HierarchicalID;
using mith::IdentityKey;
using mith::PeerKeyRegistry;
using mith::SwarmID;

namespace {

IdentityKey make_key(std::uint8_t fill) {
    IdentityKey k;
    k.public_key.fill(fill);
    return k;
}

} // namespace

TEST_CASE("PeerKeyRegistry: empty by default") {
    PeerKeyRegistry r;
    CHECK(r.size() == 0u);
    CHECK(r.find(HierarchicalID::generate(SwarmID{1})) == nullptr);
}

TEST_CASE("PeerKeyRegistry: first pin succeeds and is retrievable") {
    PeerKeyRegistry r;
    const auto hid = HierarchicalID::generate(SwarmID{1});
    const auto key = make_key(0x42);

    CHECK(r.try_pin(hid, key));
    CHECK(r.size() == 1u);

    const auto* found = r.find(hid);
    REQUIRE(found != nullptr);
    CHECK(found->public_key == key.public_key);
}

TEST_CASE("PeerKeyRegistry: re-pin with same key is idempotent") {
    PeerKeyRegistry r;
    const auto hid = HierarchicalID::generate(SwarmID{1});
    const auto key = make_key(0x42);

    CHECK(r.try_pin(hid, key));
    CHECK(r.try_pin(hid, key));   // same key, second call
    CHECK(r.try_pin(hid, key));   // and a third
    CHECK(r.size() == 1u);        // still one pin
}

TEST_CASE("PeerKeyRegistry: re-pin with a different key is rejected (TOFU)") {
    PeerKeyRegistry r;
    const auto hid     = HierarchicalID::generate(SwarmID{1});
    const auto pinned  = make_key(0x11);
    const auto attempt = make_key(0xFF);

    REQUIRE(r.try_pin(hid, pinned));
    CHECK_FALSE(r.try_pin(hid, attempt));

    // Original pin preserved.
    const auto* found = r.find(hid);
    REQUIRE(found != nullptr);
    CHECK(found->public_key == pinned.public_key);
}

TEST_CASE("PeerKeyRegistry: independent pins for different HIDs") {
    PeerKeyRegistry r;
    const auto a = HierarchicalID::generate(SwarmID{1});
    const auto b = HierarchicalID::generate(SwarmID{1});
    const auto ka = make_key(0xAA);
    const auto kb = make_key(0xBB);

    CHECK(r.try_pin(a, ka));
    CHECK(r.try_pin(b, kb));
    CHECK(r.size() == 2u);
    CHECK(r.find(a)->public_key == ka.public_key);
    CHECK(r.find(b)->public_key == kb.public_key);
}

TEST_CASE("PeerKeyRegistry: clear() drops all pins") {
    PeerKeyRegistry r;
    r.try_pin(HierarchicalID::generate(SwarmID{1}), make_key(0x01));
    r.try_pin(HierarchicalID::generate(SwarmID{1}), make_key(0x02));
    REQUIRE(r.size() == 2u);

    r.clear();
    CHECK(r.size() == 0u);
}
