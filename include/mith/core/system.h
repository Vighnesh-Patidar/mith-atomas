#pragma once

// System interface + SystemDescriptor + ResourceID — see ARCHITECTURE.md §5.1
//
// A System is a unit of logic that reads / writes components and / or
// non-component shared resources (NeighbourTable, transport queues).
// Systems are the only code allowed to mutate component state or shared
// resources.
//
// Each System declares a SystemDescriptor at registration time: name +
// two-axis read / write hazards (components and resources). The
// SystemScheduler (§5.2) consumes these to determine execution order
// (lexicographic on name in Sequential mode) and, in the future Parallel
// mode, to build a hazard DAG for concurrent dispatch.

#include "mith/core/component.h"
#include "mith/core/swarm_context.h"

#include <cstdint>
#include <string>
#include <vector>

namespace mith {

class EntityRegistry;   // forward — defined in registry.h

// Typed handles for non-component shared resources. Built-in IDs are
// stable across versions; user resources register at init via
// SystemScheduler::register_resource() (lands when v0.2+ extension
// resources arrive: spatial index, identity verifier, etc.).
enum class ResourceID : std::uint16_t {
    NeighbourTable = 0,
    TransportTx    = 1,
    TransportRx    = 2,
    // Reserved range for user / future-runtime resources.
    First_User     = 0x1000,
};

// Two-axis hazard declaration. Empty vectors are valid (a system that
// touches no shared state — e.g. a heartbeat — declares no hazards).
struct SystemDescriptor {
    std::string                  name;
    std::vector<ComponentTypeID> reads_components;
    std::vector<ComponentTypeID> writes_components;
    std::vector<ResourceID>      reads_resources;
    std::vector<ResourceID>      writes_resources;
};

class System {
public:
    virtual ~System() = default;

    // Declare R/W hazards on both axes. The scheduler may call this multiple
    // times (it caches at build_graph()); implementations should return the
    // same descriptor each call.
    virtual SystemDescriptor describe() const = 0;

    // Called every tick.
    //   registry    — access point for components (get / has / emplace / remove)
    //   ctx         — read-only swarm context (§6.2)
    //   delta_time  — tick duration in seconds (also available as ctx.delta_time_s;
    //                 the explicit parameter matches §5.1 for ergonomics)
    virtual void tick(EntityRegistry& registry,
                      const SwarmContext& ctx,
                      float delta_time) = 0;
};

} // namespace mith
