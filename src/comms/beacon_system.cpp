#include "mith/comms/beacon_system.h"

#include "mith/comms/neighbour_table.h"
#include "mith/comms/state_vector.h"
#include "mith/comms/transport.h"
#include "mith/comms/udp_wire.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"

#ifdef MITH_AUTH_ENABLED
#include "mith/identity/ed25519.h"
#endif

#include <cstring>
#include <utility>
#include <vector>

namespace mith {

BeaconSystem::BeaconSystem(World& world) noexcept
    : world_(&world)
    , neighbour_table_(&world.neighbour_table())
    , beacon_transport_(world.beacon_transport())
    , message_transport_(world.message_transport())
    , beacon_period_s_(world.config().beacon_rate_hz > 0.0f
                        ? 1.0f / world.config().beacon_rate_hz
                        : 0.0f)
    , neighbour_timeout_s_(world.config().neighbour_timeout_s) {}

SystemDescriptor BeaconSystem::describe() const {
    return SystemDescriptor{
        /*name=*/             "BeaconSystem",
        /*reads_components=*/ {
            component_id<IdentityComponent>(),
            component_id<PositionComponent>(),
            component_id<VelocityComponent>(),
            component_id<HealthComponent>(),
            component_id<RoleComponent>(),
            component_id<BehaviourStateComponent>(),
        },
        /*writes_components=*/{
            component_id<CommBufferComponent>(),
        },
        /*reads_resources=*/  {ResourceID::TransportRx},
        /*writes_resources=*/ {ResourceID::NeighbourTable, ResourceID::TransportTx},
    };
}

void BeaconSystem::tick(EntityRegistry& registry,
                        const SwarmContext& ctx,
                        float delta_time) {
    const EntityID self = registry.self_id();

    // Always age out — independent of whether we have a transport.
    if (neighbour_table_) {
        neighbour_table_->age_out(ctx.elapsed_time_s, neighbour_timeout_s_);
    }

    if (!beacon_transport_ && !message_transport_) {
        return;   // no transports at all — nothing more to do this tick
    }

    // 1. Build StateVector from the self entity. All built-in hot
    //    components are guaranteed emplaced after World::init().
    StateVector sv;
    sv.id        = registry.get<IdentityComponent>(self).id;
    sv.position  = registry.get<PositionComponent>(self);
    sv.velocity  = registry.get<VelocityComponent>(self);
    sv.health    = registry.get<HealthComponent>(self);
    sv.role      = registry.get<RoleComponent>(self);
    sv.state     = registry.get<BehaviourStateComponent>(self);
    sv.tick      = static_cast<std::uint32_t>(ctx.tick_count);

    // 2. Send beacon if the beacon period has elapsed and the beacon
    //    channel exists + supports it. In signed mode, sign over the
    //    87-byte canonical prefix (everything except the signature
    //    field itself) using the current keypair from World.
    time_since_last_beacon_s_ += delta_time;
    if (beacon_period_s_ > 0.0f
        && time_since_last_beacon_s_ >= beacon_period_s_
        && beacon_transport_
        && beacon_transport_->supports_beacons()) {
#ifdef MITH_AUTH_ENABLED
        if (auto kp = world_->identity_keypair()) {
            sv.sender_pubkey = kp->public_key;
            std::uint8_t signed_bytes[udp_wire::BEACON_SIGNED_PREFIX_BYTES];
            udp_wire::serialise_beacon_signed_payload(
                sv, signed_bytes, sizeof signed_bytes);
            sv.signature = sign_payload(
                kp->private_key, signed_bytes, sizeof signed_bytes);
        }
#endif
        beacon_transport_->send_beacon(sv);
        time_since_last_beacon_s_ = 0.0f;
    }

    // 3. Drain inbound beacons (beacon channel). Signed mode applies
    //    Ed25519 verification + TOFU per HID: the first pubkey we see
    //    for a given HID is pinned; later beacons must match or they
    //    bump rejected_beacons_ and skip the NeighbourTable upsert.
    //    Unsigned mode (or beacons with an all-zero pubkey, e.g. from a
    //    legacy peer) bypasses verification.
    if (beacon_transport_ && beacon_transport_->supports_beacons()) {
        std::vector<StateVector> beacons;
        beacon_transport_->poll_beacons(beacons);
        for (const auto& b : beacons) {
            if (b.id == sv.id) continue;          // skip our own echo

#ifdef MITH_AUTH_ENABLED
            // A non-zero pubkey indicates the sender intends signed
            // mode. Apply both signature verification + TOFU pin.
            bool pubkey_present = false;
            for (auto byte : b.sender_pubkey.public_key) {
                if (byte != 0u) { pubkey_present = true; break; }
            }
            if (pubkey_present) {
                std::uint8_t signed_bytes[udp_wire::BEACON_SIGNED_PREFIX_BYTES];
                udp_wire::serialise_beacon_signed_payload(
                    b, signed_bytes, sizeof signed_bytes);
                if (!verify_signature(b.sender_pubkey,
                                       signed_bytes, sizeof signed_bytes,
                                       b.signature.data(), b.signature.size())) {
                    ++rejected_beacons_;
                    continue;
                }
                // TOFU pin: first sighting wins; mismatch on later
                // sightings is a rejection.
                auto it = tofu_keys_.find(b.id);
                if (it == tofu_keys_.end()) {
                    tofu_keys_.emplace(b.id, b.sender_pubkey);
                } else if (std::memcmp(it->second.public_key.data(),
                                        b.sender_pubkey.public_key.data(),
                                        IdentityKey::PUBLIC_KEY_LEN) != 0) {
                    ++rejected_beacons_;
                    continue;
                }
            }
#endif
            neighbour_table_->upsert(b, ctx.elapsed_time_s);
        }
    }

    // 4. Drain inbound messages (message channel — may be a different
    //    transport entirely). Consult World::message_handlers() first —
    //    if any handler claims the message, it's NOT pushed to the
    //    mission-facing CommBufferComponent. DiscoverySystem uses this
    //    to intercept DISCOVERY_HELLO / DISCOVERY_WELCOME.
    if (message_transport_ && message_transport_->supports_messages()) {
        std::vector<Message> messages;
        message_transport_->poll_messages(messages);
        auto& cb       = registry.get<CommBufferComponent>(self);
        const auto& hs = world_->message_handlers();
        for (auto& m : messages) {
            bool claimed = false;
            for (const auto& h : hs) {
                if (h && h(m)) { claimed = true; break; }
            }
            if (!claimed) cb.queue.push(std::move(m));
        }
    }
}

} // namespace mith
