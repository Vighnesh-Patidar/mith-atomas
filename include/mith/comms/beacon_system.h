#pragma once

// BeaconSystem — see ARCHITECTURE.md §5.3, §7.2, §7.4
//
// Periodic broadcaster + receiver. Each tick:
//   1. Build a StateVector from the self entity's components.
//   2. If beacon period elapsed, send it via the transport (broadcast).
//   3. Poll inbound beacons + messages from the transport.
//   4. Upsert each received beacon into the NeighbourTable.
//   5. Push received messages into the local CommBufferComponent queue.
//   6. Age out stale entries from the NeighbourTable.
//
// Pulls all its dependencies from a World reference at construction.
// If the World has no transport, BeaconSystem still ages out neighbours
// each tick but skips send/poll (no-op transport path).

#include "mith/core/system.h"

#include <cstdint>

namespace mith {

class World;
class NeighbourTable;
class BeaconTransport;
class MessageTransport;

class BeaconSystem : public System {
public:
    explicit BeaconSystem(World& world) noexcept;

    SystemDescriptor describe() const override;
    void tick(EntityRegistry& registry,
              const SwarmContext& ctx,
              float delta_time) override;

    // Observability — incremented when an inbound signed beacon fails
    // signature verification or violates TOFU (different pubkey for an
    // HID we've seen before). Always 0 in unsigned-mode builds.
    std::uint64_t rejected_beacons() const noexcept { return rejected_beacons_; }

private:
    World*            world_;             // for World::message_handlers()
    NeighbourTable*   neighbour_table_;
    BeaconTransport*  beacon_transport_;
    MessageTransport* message_transport_;
    float             beacon_period_s_;
    float             neighbour_timeout_s_;
    float             time_since_last_beacon_s_ = 0.0f;

    // Per-system observability counter — the TOFU pin/lookup itself
    // lives on World::peer_keys() so DiscoverySystem can pre-populate
    // it from DISCOVERY_HELLO / DISCOVERY_WELCOME.
    std::uint64_t rejected_beacons_ = 0;
};

} // namespace mith
