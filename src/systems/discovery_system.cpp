#include "mith/systems/discovery_system.h"

#include "mith/comms/message.h"
#include "mith/comms/neighbour_table.h"
#include "mith/comms/peer_key_registry.h"
#include "mith/comms/transport.h"
#include "mith/core/system.h"
#include "mith/core/world.h"

#include <cstring>

namespace mith {

namespace {

// Layout of the pubkey advertisement carried in DISCOVERY_HELLO /
// DISCOVERY_WELCOME payloads (v0.3 §16, signed-mode discovery): the
// sender's 32-byte Ed25519 public key occupies payload[0..31]. Bytes
// 32..127 are reserved (zero). In unsigned-mode builds and on
// unsigned-mode peers, the whole payload is zero — receivers treat
// an all-zero pubkey as "no advertisement, do not pin".
constexpr std::size_t PUBKEY_PAYLOAD_OFFSET = 0;

void embed_pubkey_into_payload(Message& m, const IdentityKey& key) noexcept {
    m.payload.fill(0);
    std::memcpy(m.payload.data() + PUBKEY_PAYLOAD_OFFSET,
                key.public_key.data(),
                IdentityKey::PUBLIC_KEY_LEN);
}

bool extract_pubkey_from_payload(const Message& m, IdentityKey& out) noexcept {
    bool any_nonzero = false;
    for (std::size_t i = 0; i < IdentityKey::PUBLIC_KEY_LEN; ++i) {
        const std::uint8_t b = m.payload[PUBKEY_PAYLOAD_OFFSET + i];
        if (b != 0u) any_nonzero = true;
    }
    if (!any_nonzero) return false;
    std::memcpy(out.public_key.data(),
                m.payload.data() + PUBKEY_PAYLOAD_OFFSET,
                IdentityKey::PUBLIC_KEY_LEN);
    return true;
}

} // namespace

DiscoverySystem::DiscoverySystem(World& world) noexcept
    : DiscoverySystem(world, Params{}) {}

DiscoverySystem::DiscoverySystem(World& world, Params params) noexcept
    : world_(&world)
    , neighbour_table_(&world.neighbour_table())
    , message_transport_(world.message_transport())
    , params_(params) {
    // self_id_ is captured at first tick (post-init) — World::identity()
    // aborts pre-init.

    // Register the inbound message handler. BeaconSystem consults this
    // list when draining the message channel; DISCOVERY_HELLO and
    // DISCOVERY_WELCOME are claimed here and never reach mission code's
    // CommBufferComponent.
    world.register_message_handler(
        [this](const Message& m) { return handle_message_(m); });
}

SystemDescriptor DiscoverySystem::describe() const {
    return SystemDescriptor{
        /*name=*/             "DiscoverySystem",
        /*reads_components=*/ {},
        /*writes_components=*/{},
        /*reads_resources=*/  {ResourceID::NeighbourTable},
        /*writes_resources=*/ {ResourceID::TransportTx},   // sends HELLOs / WELCOMEs
    };
}

void DiscoverySystem::tick(EntityRegistry& /*registry*/,
                            const SwarmContext& /*ctx*/,
                            float delta_time) {
    if (!world_) return;

    // Capture self_id_ once World is initialised — identity() aborts
    // pre-init.
    if (self_id_.unit_id.is_nil() && world_->is_initialized()) {
        self_id_ = world_->identity();
    }

    if (state_ == DiscoveryState::Active) return;

    time_in_bootstrap_s_ += delta_time;
    time_since_last_hello_s_ += delta_time;

    // Active HELLO emission during bootstrap (skip if disabled or no
    // transport).
    if (params_.hello_period_s > 0.0f
        && time_since_last_hello_s_ >= params_.hello_period_s
        && message_transport_
        && message_transport_->supports_messages()) {
        Message hello;
        hello.sender     = self_id_;
        hello.recipient  = BROADCAST_ID;
        hello.type       = messages::DISCOVERY_HELLO;
#ifdef MITH_AUTH_ENABLED
        // Signed mode: advertise our pubkey so receivers can pin us
        // before our first beacon arrives.
        if (auto kp = world_->identity_keypair()) {
            embed_pubkey_into_payload(hello, kp->public_key);
        }
#endif
        message_transport_->send_message(hello);
        ++hellos_sent_;
        time_since_last_hello_s_ = 0.0f;
    }

    // Count distinct peers from the NeighbourTable.
    std::uint32_t count = 0;
    if (neighbour_table_) {
        for (auto it = neighbour_table_->begin();
             it != neighbour_table_->end(); ++it) {
            ++count;
        }
    }
    peers_seen_ = count;

    if (peers_seen_ >= params_.bootstrap_quorum) {
        promote_to_active_();
        return;
    }
    if (time_in_bootstrap_s_ >= params_.bootstrap_timeout_s) {
        promote_to_active_();
    }
}

bool DiscoverySystem::handle_message_(const Message& m) noexcept {
    if (m.type == messages::DISCOVERY_HELLO) {
        ++hellos_received_;
#ifdef MITH_AUTH_ENABLED
        // Pin the sender's advertised pubkey BEFORE we ever see their
        // beacon — eliminates the TOFU window for actively-discovered
        // peers. Conflict (sender claims an HID we already pinned to a
        // different key) is silently ignored here; BeaconSystem will
        // reject the first signed beacon and bump rejected_beacons.
        IdentityKey advertised;
        if (world_ && extract_pubkey_from_payload(m, advertised)) {
            world_->peer_keys().try_pin(m.sender, advertised);
        }
#endif
        // Respond with a directed WELCOME to the HELLO's sender. Always
        // respond — peers in Active state still help newcomers bootstrap.
        if (message_transport_ && message_transport_->supports_messages()) {
            Message welcome;
            welcome.sender    = self_id_;
            welcome.recipient = m.sender;
            welcome.type      = messages::DISCOVERY_WELCOME;
#ifdef MITH_AUTH_ENABLED
            if (auto kp = world_->identity_keypair()) {
                embed_pubkey_into_payload(welcome, kp->public_key);
            }
#endif
            message_transport_->send_message(welcome);
            ++welcomes_sent_;
        }
        return true;
    }
    if (m.type == messages::DISCOVERY_WELCOME) {
        ++welcomes_received_;
#ifdef MITH_AUTH_ENABLED
        IdentityKey advertised;
        if (world_ && extract_pubkey_from_payload(m, advertised)) {
            world_->peer_keys().try_pin(m.sender, advertised);
        }
#endif
        // The accompanying beacon channel (which runs in parallel) will
        // populate the NeighbourTable on its own cycle.
        return true;
    }
    return false;   // not a discovery message — fall through to mission queue
}

void DiscoverySystem::promote_to_active_() noexcept {
    state_ = DiscoveryState::Active;
}

} // namespace mith
