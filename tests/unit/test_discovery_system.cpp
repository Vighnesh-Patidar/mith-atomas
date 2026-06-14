#include "doctest.h"

#include "mith/comms/message.h"
#include "mith/comms/neighbour_table.h"
#include "mith/comms/state_vector.h"
#include "mith/comms/transport.h"
#include "mith/core/world.h"
#include "mith/identity/hierarchical_id.h"
#include "mith/systems/discovery_system.h"

#include <memory>
#include <vector>

using mith::DiscoveryState;
using mith::DiscoverySystem;
using mith::HierarchicalID;
using mith::StateVector;
using mith::SwarmID;
using mith::World;
using mith::WorldConfig;

namespace {

void seed_neighbours(mith::NeighbourTable& nt, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        StateVector sv;
        sv.id = HierarchicalID::generate(SwarmID{1});
        nt.upsert(sv, 0.0f);
    }
}

} // namespace

TEST_CASE("DiscoverySystem: starts in Bootstrapping with zero peers seen") {
    World w(WorldConfig{});
    w.init();
    DiscoverySystem ds(w);

    CHECK(ds.is_bootstrapping());
    CHECK_FALSE(ds.is_active());
    CHECK(ds.current_state() == DiscoveryState::Bootstrapping);
    CHECK(ds.peers_seen() == 0u);
    CHECK(ds.time_in_bootstrap_s() == 0.0f);
}

TEST_CASE("DiscoverySystem: promotes to Active when quorum reached") {
    World w(WorldConfig{});
    w.init();

    DiscoverySystem::Params p;
    p.bootstrap_quorum     = 3;
    p.bootstrap_timeout_s  = 999.0f;     // disable timeout path
    DiscoverySystem ds(w, p);

    // Two peers — below quorum, stays in bootstrap.
    seed_neighbours(w.neighbour_table(), 2);
    ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(ds.is_bootstrapping());
    CHECK(ds.peers_seen() == 2u);

    // Add a third peer → quorum met on next tick.
    seed_neighbours(w.neighbour_table(), 1);
    ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(ds.is_active());
    CHECK(ds.peers_seen() == 3u);
}

TEST_CASE("DiscoverySystem: promotes to Active on timeout even with zero peers") {
    World w(WorldConfig{});
    w.init();

    DiscoverySystem::Params p;
    p.bootstrap_quorum     = 5;          // unreachable in this test
    p.bootstrap_timeout_s  = 1.0f;
    DiscoverySystem ds(w, p);

    // Tick for 0.9 s — still bootstrapping.
    for (int i = 0; i < 9; ++i) ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(ds.is_bootstrapping());

    // One more tick → 1.0 s elapsed, timeout triggers.
    ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(ds.is_active());
}

TEST_CASE("DiscoverySystem: stays Active once promoted, even if NeighbourTable empties") {
    World w(WorldConfig{});
    w.init();

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 1;
    p.bootstrap_timeout_s = 999.0f;
    DiscoverySystem ds(w, p);

    seed_neighbours(w.neighbour_table(), 1);
    ds.tick(w.registry(), w.context(), 0.1f);
    REQUIRE(ds.is_active());

    // Age out the table (no-op equivalent for the test — just clear it).
    w.neighbour_table().clear();
    ds.tick(w.registry(), w.context(), 0.1f);
    ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(ds.is_active());   // discovery state is sticky once promoted
}

TEST_CASE("DiscoverySystem: time_in_bootstrap stops accumulating after promotion") {
    World w(WorldConfig{});
    w.init();

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 1;
    p.bootstrap_timeout_s = 999.0f;
    DiscoverySystem ds(w, p);

    seed_neighbours(w.neighbour_table(), 1);
    ds.tick(w.registry(), w.context(), 0.5f);
    const float t_at_promotion = ds.time_in_bootstrap_s();
    REQUIRE(ds.is_active());

    for (int i = 0; i < 10; ++i) ds.tick(w.registry(), w.context(), 0.5f);
    CHECK(ds.time_in_bootstrap_s() == t_at_promotion);
}

TEST_CASE("DiscoverySystem: bootstrap_quorum=0 promotes on the first tick") {
    World w(WorldConfig{});
    w.init();

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 0;
    p.bootstrap_timeout_s = 999.0f;
    DiscoverySystem ds(w, p);

    ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(ds.is_active());
    CHECK(ds.peers_seen() == 0u);
}

TEST_CASE("DiscoverySystem: SystemDescriptor advertises NeighbourTable as a read-resource") {
    World w(WorldConfig{});
    w.init();
    DiscoverySystem ds(w);
    const auto d = ds.describe();

    CHECK(d.name == "DiscoverySystem");
    CHECK(d.reads_components.empty());
    CHECK(d.writes_components.empty());

    bool has_neighbour_table = false;
    for (auto r : d.reads_resources) {
        if (r == mith::ResourceID::NeighbourTable) { has_neighbour_table = true; break; }
    }
    CHECK(has_neighbour_table);
}

// ============================================================================
// Active HELLO / WELCOME — v0.3 second slice
// ============================================================================

namespace {

// Tiny TransportLayer stand-in that records every send and stores
// the bytes locally — lets the tests verify HELLO emission and
// WELCOME responses without going through SimBus.
class RecordingTransport : public mith::TransportLayer {
public:
    std::vector<mith::Message>     sent_messages;
    std::vector<mith::StateVector> sent_beacons;

    bool send_beacon(const mith::StateVector& sv) override {
        sent_beacons.push_back(sv);
        return true;
    }
    bool send_message(const mith::Message& m) override {
        sent_messages.push_back(m);
        return true;
    }
    void poll_beacons(std::vector<mith::StateVector>& out) override { out.clear(); }
    void poll_messages(std::vector<mith::Message>& out) override    { out.clear(); }
    bool is_healthy() const override { return true; }
};

} // namespace

TEST_CASE("DiscoverySystem: emits HELLO at hello_period_s during bootstrap") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 999;          // never quorum-promote in this test
    p.bootstrap_timeout_s = 999.0f;
    p.hello_period_s      = 0.5f;
    DiscoverySystem ds(w, p);

    // Tick 1.0 s — should fire 2 HELLOs (at t=0.5 and t=1.0).
    for (int i = 0; i < 10; ++i) ds.tick(w.registry(), w.context(), 0.1f);

    CHECK(ds.hellos_sent() == 2u);
    REQUIRE(rec->sent_messages.size() == 2u);
    CHECK(rec->sent_messages[0].type   == mith::messages::DISCOVERY_HELLO);
    CHECK(rec->sent_messages[0].sender == w.identity());
    CHECK(mith::is_broadcast(rec->sent_messages[0].recipient));
}

TEST_CASE("DiscoverySystem: stops emitting HELLOs once promoted to Active") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 1;
    p.bootstrap_timeout_s = 999.0f;
    p.hello_period_s      = 0.1f;
    DiscoverySystem ds(w, p);

    // First tick emits a HELLO + sees 0 peers → still bootstrapping.
    ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(rec->sent_messages.size() == 1u);

    // Add a peer → next tick promotes to Active.
    seed_neighbours(w.neighbour_table(), 1);
    ds.tick(w.registry(), w.context(), 0.1f);
    REQUIRE(ds.is_active());
    const auto after_promote = rec->sent_messages.size();

    // Many more ticks — no new HELLOs even though period would fire.
    for (int i = 0; i < 50; ++i) ds.tick(w.registry(), w.context(), 0.1f);
    CHECK(rec->sent_messages.size() == after_promote);
}

TEST_CASE("DiscoverySystem: hello_period_s=0 disables active discovery") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 999;
    p.bootstrap_timeout_s = 999.0f;
    p.hello_period_s      = 0.0f;
    DiscoverySystem ds(w, p);

    for (int i = 0; i < 100; ++i) ds.tick(w.registry(), w.context(), 0.1f);

    CHECK(ds.hellos_sent() == 0u);
    CHECK(rec->sent_messages.empty());
}

TEST_CASE("DiscoverySystem: inbound HELLO triggers a directed WELCOME reply") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.hello_period_s = 0.0f;   // we drive the inbound side; suppress outbound
    DiscoverySystem ds(w, p);
    ds.tick(w.registry(), w.context(), 0.1f);   // capture self_id_

    // Synthesise an inbound HELLO from a peer.
    mith::Message hello;
    hello.sender    = mith::HierarchicalID::generate(mith::SwarmID{1});
    hello.recipient = mith::BROADCAST_ID;
    hello.type      = mith::messages::DISCOVERY_HELLO;

    // Route through the World's handler list (BeaconSystem would normally
    // do this — we exercise the handler directly).
    bool claimed = false;
    for (const auto& h : w.message_handlers()) {
        if (h && h(hello)) { claimed = true; break; }
    }
    CHECK(claimed);
    CHECK(ds.hellos_received() == 1u);
    CHECK(ds.welcomes_sent() == 1u);

    REQUIRE(rec->sent_messages.size() == 1u);
    CHECK(rec->sent_messages[0].type      == mith::messages::DISCOVERY_WELCOME);
    CHECK(rec->sent_messages[0].sender    == w.identity());
    CHECK(rec->sent_messages[0].recipient == hello.sender);
}

TEST_CASE("DiscoverySystem: inbound WELCOME increments welcomes_received but doesn't reply") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.hello_period_s = 0.0f;
    DiscoverySystem ds(w, p);
    ds.tick(w.registry(), w.context(), 0.1f);

    mith::Message welcome;
    welcome.sender    = mith::HierarchicalID::generate(mith::SwarmID{1});
    welcome.recipient = w.identity();
    welcome.type      = mith::messages::DISCOVERY_WELCOME;

    for (const auto& h : w.message_handlers()) {
        if (h) h(welcome);
    }
    CHECK(ds.welcomes_received() == 1u);
    CHECK(ds.welcomes_sent()     == 0u);
    CHECK(rec->sent_messages.empty());
}

TEST_CASE("DiscoverySystem: message handler claims discovery types, lets others through") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    DiscoverySystem ds(w);

    auto run_handler = [&](const mith::Message& m) {
        for (const auto& h : w.message_handlers()) {
            if (h && h(m)) return true;
        }
        return false;
    };

    mith::Message hello;
    hello.type = mith::messages::DISCOVERY_HELLO;
    mith::Message welcome;
    welcome.type = mith::messages::DISCOVERY_WELCOME;
    mith::Message task_bid;
    task_bid.type = mith::messages::TASK_BID;
    mith::Message custom;
    custom.type = mith::messages::CUSTOM;

    CHECK(run_handler(hello));
    CHECK(run_handler(welcome));
    CHECK_FALSE(run_handler(task_bid));     // not a discovery type
    CHECK_FALSE(run_handler(custom));
}

TEST_CASE("DiscoverySystem: WELCOME responder works even after the responder is Active") {
    // A robot already past bootstrap should still help newcomers — verifies
    // handle_message_() is not gated on the responder's discovery state.
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 1;
    p.bootstrap_timeout_s = 999.0f;
    p.hello_period_s      = 0.0f;
    DiscoverySystem ds(w, p);

    seed_neighbours(w.neighbour_table(), 1);
    ds.tick(w.registry(), w.context(), 0.1f);
    REQUIRE(ds.is_active());

    mith::Message hello;
    hello.sender = mith::HierarchicalID::generate(mith::SwarmID{1});
    hello.type   = mith::messages::DISCOVERY_HELLO;
    for (const auto& h : w.message_handlers()) {
        if (h && h(hello)) break;
    }
    CHECK(ds.welcomes_sent() == 1u);
    REQUIRE(rec->sent_messages.size() == 1u);
    CHECK(rec->sent_messages[0].type == mith::messages::DISCOVERY_WELCOME);
}

// ============================================================================
// Signed-mode pubkey carriage — v0.3 (WELCOME pubkey slice)
// ============================================================================

#ifdef MITH_AUTH_ENABLED

#include "mith/comms/peer_key_registry.h"
#include "mith/identity/ed25519.h"

TEST_CASE("DiscoverySystem: signed-mode HELLO embeds the sender's pubkey in payload[0..31]") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.bootstrap_quorum    = 999;
    p.bootstrap_timeout_s = 999.0f;
    p.hello_period_s      = 0.1f;
    DiscoverySystem ds(w, p);

    ds.tick(w.registry(), w.context(), 0.1f);
    REQUIRE(rec->sent_messages.size() == 1u);
    const auto& hello = rec->sent_messages[0];
    CHECK(hello.type == mith::messages::DISCOVERY_HELLO);

    const auto kp = w.identity_keypair();
    REQUIRE(kp.has_value());
    for (std::size_t i = 0; i < mith::IdentityKey::PUBLIC_KEY_LEN; ++i) {
        CHECK(hello.payload[i] == kp->public_key.public_key[i]);
    }
}

TEST_CASE("DiscoverySystem: signed-mode WELCOME embeds the sender's pubkey") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    auto* rec = static_cast<RecordingTransport*>(w.transport());

    DiscoverySystem::Params p;
    p.hello_period_s = 0.0f;   // suppress outbound HELLOs
    DiscoverySystem ds(w, p);
    ds.tick(w.registry(), w.context(), 0.1f);

    // Inbound HELLO from a peer triggers a WELCOME response.
    mith::Message hello;
    hello.sender = mith::HierarchicalID::generate(mith::SwarmID{1});
    hello.type   = mith::messages::DISCOVERY_HELLO;
    for (const auto& h : w.message_handlers()) {
        if (h && h(hello)) break;
    }
    REQUIRE(rec->sent_messages.size() == 1u);
    const auto& welcome = rec->sent_messages[0];
    CHECK(welcome.type == mith::messages::DISCOVERY_WELCOME);

    const auto kp = w.identity_keypair();
    REQUIRE(kp.has_value());
    for (std::size_t i = 0; i < mith::IdentityKey::PUBLIC_KEY_LEN; ++i) {
        CHECK(welcome.payload[i] == kp->public_key.public_key[i]);
    }
}

TEST_CASE("DiscoverySystem: signed-mode HELLO receipt pins sender's pubkey in PeerKeyRegistry") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();

    DiscoverySystem::Params p;
    p.hello_period_s = 0.0f;
    DiscoverySystem ds(w, p);
    ds.tick(w.registry(), w.context(), 0.1f);

    REQUIRE(w.peer_keys().size() == 0u);

    // Synthesise a HELLO from a peer carrying their pubkey.
    auto peer_kp = mith::generate_identity_keypair();
    const auto peer_hid = mith::HierarchicalID::generate(mith::SwarmID{1});

    mith::Message hello;
    hello.sender = peer_hid;
    hello.type   = mith::messages::DISCOVERY_HELLO;
    for (std::size_t i = 0; i < mith::IdentityKey::PUBLIC_KEY_LEN; ++i) {
        hello.payload[i] = peer_kp.public_key.public_key[i];
    }

    for (const auto& h : w.message_handlers()) {
        if (h && h(hello)) break;
    }

    REQUIRE(w.peer_keys().size() == 1u);
    const auto* pinned = w.peer_keys().find(peer_hid);
    REQUIRE(pinned != nullptr);
    CHECK(pinned->public_key == peer_kp.public_key.public_key);
}

TEST_CASE("DiscoverySystem: signed-mode WELCOME receipt pins sender's pubkey") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();

    DiscoverySystem ds(w);
    ds.tick(w.registry(), w.context(), 0.1f);

    auto peer_kp = mith::generate_identity_keypair();
    const auto peer_hid = mith::HierarchicalID::generate(mith::SwarmID{1});

    mith::Message welcome;
    welcome.sender = peer_hid;
    welcome.type   = mith::messages::DISCOVERY_WELCOME;
    for (std::size_t i = 0; i < mith::IdentityKey::PUBLIC_KEY_LEN; ++i) {
        welcome.payload[i] = peer_kp.public_key.public_key[i];
    }

    for (const auto& h : w.message_handlers()) {
        if (h && h(welcome)) break;
    }

    const auto* pinned = w.peer_keys().find(peer_hid);
    REQUIRE(pinned != nullptr);
    CHECK(pinned->public_key == peer_kp.public_key.public_key);
}

TEST_CASE("DiscoverySystem: HELLO with all-zero payload (unsigned peer) does NOT create a pin") {
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    DiscoverySystem ds(w);
    ds.tick(w.registry(), w.context(), 0.1f);

    mith::Message hello;
    hello.sender = mith::HierarchicalID::generate(mith::SwarmID{1});
    hello.type   = mith::messages::DISCOVERY_HELLO;
    // payload left all-zero — represents an unsigned-mode peer.

    for (const auto& h : w.message_handlers()) {
        if (h && h(hello)) break;
    }
    CHECK(w.peer_keys().size() == 0u);   // no pin for unsigned peers
}

TEST_CASE("DiscoverySystem: HELLO with adversary key for an already-pinned HID is silently ignored") {
    // Once a pin exists (e.g. from an earlier WELCOME), a later HELLO
    // from an adversary claiming the same HID with a different pubkey
    // does NOT overwrite the pin. DiscoverySystem doesn't bump any
    // rejection counter — that's BeaconSystem's job when the adversary
    // tries to actually send a beacon — but the pin survives intact.
    World w(WorldConfig{}, std::make_unique<RecordingTransport>());
    w.init();
    DiscoverySystem ds(w);
    ds.tick(w.registry(), w.context(), 0.1f);

    const auto peer_hid    = mith::HierarchicalID::generate(mith::SwarmID{1});
    const auto real_kp     = mith::generate_identity_keypair();
    const auto attacker_kp = mith::generate_identity_keypair();

    // First pin: real key.
    mith::Message welcome;
    welcome.sender = peer_hid;
    welcome.type   = mith::messages::DISCOVERY_WELCOME;
    for (std::size_t i = 0; i < mith::IdentityKey::PUBLIC_KEY_LEN; ++i) {
        welcome.payload[i] = real_kp.public_key.public_key[i];
    }
    for (const auto& h : w.message_handlers()) {
        if (h && h(welcome)) break;
    }
    REQUIRE(w.peer_keys().find(peer_hid)->public_key == real_kp.public_key.public_key);

    // Attacker HELLO claiming the same HID with a different key.
    mith::Message hello;
    hello.sender = peer_hid;
    hello.type   = mith::messages::DISCOVERY_HELLO;
    for (std::size_t i = 0; i < mith::IdentityKey::PUBLIC_KEY_LEN; ++i) {
        hello.payload[i] = attacker_kp.public_key.public_key[i];
    }
    for (const auto& h : w.message_handlers()) {
        if (h && h(hello)) break;
    }

    // Pin still holds the real key.
    CHECK(w.peer_keys().find(peer_hid)->public_key == real_kp.public_key.public_key);
}

#endif // MITH_AUTH_ENABLED

TEST_CASE("DiscoverySystem: deterministic — identical peer schedule → identical transition tick") {
    auto run = []() {
        World w(WorldConfig{});
        w.init();
        DiscoverySystem::Params p;
        p.bootstrap_quorum = 3;
        DiscoverySystem ds(w, p);

        int promo_tick = -1;
        for (int t = 0; t < 100; ++t) {
            if (t == 5)  seed_neighbours(w.neighbour_table(), 1);
            if (t == 12) seed_neighbours(w.neighbour_table(), 1);
            if (t == 25) seed_neighbours(w.neighbour_table(), 1);
            ds.tick(w.registry(), w.context(), 0.1f);
            if (ds.is_active() && promo_tick == -1) { promo_tick = t; break; }
        }
        return promo_tick;
    };
    CHECK(run() == run());
}
