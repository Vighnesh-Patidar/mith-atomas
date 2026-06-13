# MithAtomas — Swarm Robotics Orchestration Runtime
## Architecture & Design Document v0.1

> **Repo:** `mith-atomas`
> **Namespace:** `mith`
> **Language:** C++17 (Python bindings planned, post-v1.0)
> **License:** Apache 2.0
> **Status:** Pre-implementation design document

---

## 0. Name & Philosophy

**MithAtomas** is derived from two roots:

- **Mith** — from *mithril*, the legendary metal: lightweight, extraordinarily strong, rare. Reflects the runtime's goal: minimal overhead, maximum resilience.
- **Atomas** — from Greek *atomos* (indivisible). Each robot in a swarm is an indivisible unit. The swarm is the emergent whole.

The core philosophy:

> **Decentralised by default. Composable by design. Hardware-agnostic by necessity.**

MithAtomas does not assume a central coordinator. Every node in a swarm runs the same runtime. Coordination emerges from shared protocols, not from a master process. The runtime makes no assumptions about the robot's physical platform, communication medium, or decision-making strategy.

---

## 1. Problem Statement

Swarm robotics lacks a practical, open-source orchestration layer that:

1. Runs **on the robot** (not just in simulation)
2. Provides a **data-oriented abstraction** for robot state rather than prescribing behaviour
3. Is **transport-agnostic** (UDP, serial, ROS 2, simulation bus — all interchangeable)
4. Supports **arbitrary decision-making backends** — FSMs, rule engines, RL policies, neural nets
5. Scales from **5 robots to 5000** without architectural changes
6. Is built for **fault tolerance from day one**, not bolted on later

Existing tools (ROS 2, ARGoS, Buzz, OLSAC) address subsets of this. None provide a unified runtime with a clean entity abstraction that a firmware engineer and an ML researcher can both target without compromise.

MithAtomas fills that gap.

---

## 2. System Overview

```
┌─────────────────────────────────────────────────────────┐
│                    MithAtomas Runtime                   │
│                                                         │
│  ┌──────────────┐   ┌──────────────┐  ┌─────────────┐  │
│  │  Entity &    │   │   System     │  │  Transport  │  │
│  │  Component   │◄──│  Scheduler   │  │   Layer     │  │
│  │  Registry    │   │  (async DAG) │  │  (pluggable)│  │
│  └──────┬───────┘   └──────┬───────┘  └──────┬──────┘  │
│         │                  │                  │         │
│  ┌──────▼───────┐   ┌──────▼───────┐  ┌──────▼──────┐  │
│  │  Component   │   │  Built-in    │  │  Neighbour  │  │
│  │  Stores      │   │  Systems     │  │  Table      │  │
│  │  (hybrid     │   │  (Flock,     │  │  + Beacon   │  │
│  │   archetype) │   │   TaskAlloc, │  │  System     │  │
│  └──────────────┘   │   Fault)     │  └─────────────┘  │
│                     └──────────────┘                    │
│  ┌──────────────────────────────────────────────────┐   │
│  │              Action Interface Layer              │   │
│  │   ActionProvider (user-implemented) → ActionQueue│   │
│  │   ActionExecutorSystem → Component writes        │   │
│  └──────────────────────────────────────────────────┘   │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │               Identity Layer                     │   │
│  │   HierarchicalID: SwarmID (16-bit) : UUID (128b) │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

Every physical or simulated robot runs one instance of this runtime. The runtime manages:

- **What the robot knows about itself** (Entity + Components)
- **What the robot knows about its neighbours** (NeighbourTable)
- **What the robot does this tick** (ActionProvider → ActionQueue → ActionExecutorSystem)
- **How information moves between robots** (TransportLayer)
- **How systems are scheduled and parallelised** (SystemScheduler)

---

## 3. Identity Model

### 3.1 HierarchicalID

Every entity in MithAtomas has a `HierarchicalID`:

```
┌────────────┬──────────────────────────────────────────┐
│  SwarmID   │                  UnitID                  │
│  (16-bit)  │               (UUID v4, 128-bit)         │
└────────────┴──────────────────────────────────────────┘
```

- **SwarmID** — assigned at deployment time. Config file, environment variable, or broadcast from a mission controller at swarm init. Identifies which swarm a unit belongs to. Enables multi-swarm deployments where robots from different swarms must coexist in a shared space without ID collision.
- **UnitID** — UUID v4, generated on first boot and persisted to non-volatile storage. Self-sovereign: no coordination required to generate. Hardware-independent.

```cpp
namespace mith {

struct HierarchicalID {
    uint16_t swarm_id;
    UUID     unit_id;     // 128-bit, RFC 4122 v4

    bool operator==(const HierarchicalID&) const noexcept;
    std::string to_string() const;           // "0001:550e8400-e29b-41d4-a716-..."
    static HierarchicalID generate(uint16_t swarm_id);
};

} // namespace mith
```

### 3.2 EntityID

Within a single runtime instance (one robot's process), entities are identified by a compact `EntityID` (32-bit integer). The `HierarchicalID` is stored as a component (`IdentityComponent`) on the entity. This keeps the inner loop fast — integer comparisons, not UUID comparisons.

```cpp
using EntityID = uint32_t;
constexpr EntityID INVALID_ENTITY = 0;
```

---

## 4. Entity & Component System (ECS)

### 4.1 Design: Hybrid Archetype Model

MithAtomas uses a **hybrid archetype ECS**:

- **Hot components** (accessed every tick by multiple systems) are stored in **dense archetype-style arrays**. Components for the same type across all entities with that component are contiguous in memory. Cache-friendly. These are fixed at compile time via a registered set.
- **Cold components** (sparse, infrequent access, user-defined) are stored in a **per-entity component map** (`unordered_map<ComponentTypeID, ComponentPtr>`). Flexible. No cache pressure because they're not in the hot path.

The distinction between hot and cold is declared by the component author:

```cpp
// Hot — registered at compile time, gets a dense store
struct PositionComponent : mith::HotComponent<PositionComponent> {
    float x, y, z;
};

// Cold — stored in the flexible map
struct MissionTagComponent : mith::ColdComponent<MissionTagComponent> {
    std::string mission_id;
    uint8_t     priority;
};
```

### 4.2 Archetype

An **Archetype** is a unique combination of hot component types. All entities sharing the same set of hot components are grouped in the same archetype. Within an archetype, each component type is stored in a packed array indexed by a dense entity index.

```
Archetype { Position, Velocity, Health }:
    position_store[0..N-1]   ← packed
    velocity_store[0..N-1]   ← packed
    health_store[0..N-1]     ← packed
```

When a hot component is added or removed from an entity, the entity **migrates** to the appropriate archetype. This migration is O(1) amortised for our use case (swarm entities rarely change their hot component set after init).

### 4.3 EntityRegistry

The central registry owns all archetypes and the cold component maps.

```cpp
namespace mith {

class EntityRegistry {
public:
    EntityID    create_entity();
    void        destroy_entity(EntityID id);

    // Hot component access — O(1), cache-friendly
    template<HotComponentType T>
    T&          get(EntityID id);

    template<HotComponentType T>
    bool        has(EntityID id) const;

    template<HotComponentType T>
    T&          emplace(EntityID id, T&& component);

    template<HotComponentType T>
    void        remove(EntityID id);

    // Cold component access
    template<ColdComponentType T>
    T&          get_cold(EntityID id);

    template<ColdComponentType T>
    T&          emplace_cold(EntityID id, T&& component);

    // Query — returns a view over all entities with the given hot components
    template<HotComponentType... Ts>
    ArchetypeView<Ts...> view();

    // Snapshot for read-only cross-system access (used by ActionProvider)
    EntitySnapshot snapshot(EntityID id) const;
};

} // namespace mith
```

### 4.4 Core Hot Components (Built-in)

These are registered by the runtime and always available:

| Component | Fields | Notes |
|---|---|---|
| `IdentityComponent` | `HierarchicalID id` | Set once at init |
| `PositionComponent` | `float x, y, z` | World-frame position |
| `VelocityComponent` | `float vx, vy, vz` | Current velocity |
| `OrientationComponent` | `float qw, qx, qy, qz` | Quaternion |
| `HealthComponent` | `uint8_t value` | 0–100, managed by FaultMonitorSystem |
| `RoleComponent` | `RoleID role` | Current swarm role (worker, scout, relay...) |
| `BehaviourStateComponent` | `StateID state` | Current state, used by ActionProvider |
| `ActionQueueComponent` | `RingBuffer<Action, 8>` | Pending actions |
| `CommBufferComponent` | `RingBuffer<Message, 16>` | Inbound messages |

---

## 5. System Model

### 5.1 System Interface

A System is a unit of logic that reads and/or writes components. Systems are the only entities allowed to mutate component data.

```cpp
namespace mith {

class System {
public:
    virtual ~System() = default;

    // Declare read/write access — used by scheduler to build dependency graph
    virtual SystemDescriptor describe() const = 0;

    // Called every tick. registry is passed as the access point.
    virtual void tick(EntityRegistry& registry,
                      const SwarmContext& ctx,
                      float delta_time) = 0;
};

struct SystemDescriptor {
    std::string              name;
    std::vector<ComponentTypeID> reads;
    std::vector<ComponentTypeID> writes;
    // Systems with no shared writes can run in parallel
};

} // namespace mith
```

### 5.2 SystemScheduler — Async DAG Execution

The scheduler builds a **dependency graph** from system descriptors at startup. Two systems have a dependency if one writes a component that the other reads or writes (write-after-read, read-after-write, write-after-write hazards).

At each tick:
1. Systems with no unresolved dependencies are dispatched to a **thread pool** concurrently.
2. When a system completes, its dependents are checked — if all their dependencies are resolved, they are dispatched.
3. The tick completes when all systems have run.

This is a standard parallel task DAG, equivalent to a topological sort with concurrent execution at each level.

```cpp
namespace mith {

class SystemScheduler {
public:
    void register_system(std::unique_ptr<System> system);
    void build_graph();     // Call after all systems registered, before first tick
    void tick(EntityRegistry& registry, const SwarmContext& ctx, float delta_time);

private:
    std::vector<std::unique_ptr<System>>    systems_;
    DAG<SystemNode>                         dep_graph_;
    ThreadPool                              thread_pool_;
};

} // namespace mith
```

**Thread pool sizing:** Defaults to `std::thread::hardware_concurrency() - 1`. On a single-core embedded target, degrades gracefully to sequential execution.

### 5.3 Built-in Systems

The following systems ship with MithAtomas. All are optional — disable any in the `WorldConfig`.

| System | Reads | Writes | Notes |
|---|---|---|---|
| `BeaconSystem` | Identity, Position, Velocity, Health, Role | CommBuffer (outbound) | Broadcasts StateVector; processes incoming beacons into NeighbourTable |
| `FlockingSystem` | Position, Velocity, NeighbourTable | Velocity | Reynolds rules: separation, alignment, cohesion |
| `TaskAllocSystem` | Role, NeighbourTable, BehaviourState | Role | Distributed task allocation via auction or threshold |
| `FaultMonitorSystem` | Health, CommBuffer | Health, BehaviourState | Detects comm loss, hardware faults; triggers degraded-mode |
| `ActionExecutorSystem` | ActionQueue | (all writable) | Drains ActionQueue, validates and applies actions |

---

## 6. Action Interface

This is the primary extension point for user-defined robot behaviour. It is deliberately backend-agnostic.

### 6.1 Action

An `Action` represents a single intended operation by a robot for the current tick or a short horizon.

```cpp
namespace mith {

using ActionTypeID = uint32_t;

// Well-known built-in action types
namespace actions {
    constexpr ActionTypeID IDLE       = 0;
    constexpr ActionTypeID MOVE       = 1;
    constexpr ActionTypeID HOVER      = 2;
    constexpr ActionTypeID TRANSMIT   = 3;
    constexpr ActionTypeID SCAN       = 4;
    constexpr ActionTypeID REGROUP    = 5;
    constexpr ActionTypeID FOLLOW     = 6;
    constexpr ActionTypeID CUSTOM     = 0x1000; // User-defined range starts here
}

struct Action {
    ActionTypeID                  type;
    float                         priority;       // Higher = executed first on conflict
    ComponentMask                 modifies;       // Declared write mask for validation
    std::array<uint8_t, 64>       params;         // Serialisable payload, user-defined layout
    HierarchicalID                target;         // Optional: directed action target
};

} // namespace mith
```

### 6.2 ActionProvider Interface

Users subclass `ActionProvider` to implement any decision-making backend.

```cpp
namespace mith {

// Read-only snapshot of a single entity's hot components
struct EntitySnapshot {
    EntityID             id;
    HierarchicalID       hid;
    PositionComponent    position;
    VelocityComponent    velocity;
    OrientationComponent orientation;
    HealthComponent      health;
    RoleComponent        role;
    BehaviourStateComponent state;
};

// Read-only view of current neighbours from the NeighbourTable
struct NeighbourView {
    struct NeighbourEntry {
        HierarchicalID   hid;
        PositionComponent position;
        VelocityComponent velocity;
        HealthComponent   health;
        RoleComponent     role;
        float             last_seen_ms;   // Age of this entry
        float             rssi;           // Signal strength if available, NaN if not
    };

    std::span<const NeighbourEntry> neighbours() const;
    size_t count() const;
};

// Global swarm context — read-only
struct SwarmContext {
    uint16_t            swarm_id;
    float               elapsed_time_s;
    float               delta_time_s;
    size_t              tick_count;
};

// THE USER-FACING INTERFACE
class ActionProvider {
public:
    virtual ~ActionProvider() = default;

    // Called once at runtime start
    virtual void on_init(const EntitySnapshot& self) {}

    // Called every tick. Return an Action (or actions::IDLE).
    // Must be non-blocking. No I/O. No heap allocation recommended.
    virtual Action evaluate(const EntitySnapshot&  self,
                            const NeighbourView&   neighbours,
                            const SwarmContext&    ctx) = 0;

    // Called when a directed Message arrives for this entity
    virtual void on_message(const Message& msg,
                            const EntitySnapshot& self) {}
};

} // namespace mith
```

### 6.3 Backend Examples

**FSM-backed provider:**
```cpp
class PatrolFSM : public mith::ActionProvider {
    enum State { PATROL, REGROUP, IDLE };
    State current_ = PATROL;
public:
    mith::Action evaluate(const mith::EntitySnapshot& self,
                          const mith::NeighbourView& neighbours,
                          const mith::SwarmContext& ctx) override {
        if (neighbours.count() < 2) {
            current_ = REGROUP;
        }
        switch (current_) {
            case REGROUP: return {mith::actions::REGROUP, 1.0f};
            case PATROL:  return {mith::actions::MOVE, 0.5f};
            default:      return {mith::actions::IDLE, 0.0f};
        }
    }
};
```

**RL policy provider:**
```cpp
class RLPolicyProvider : public mith::ActionProvider {
    NeuralNet policy_;   // User's inference engine
public:
    mith::Action evaluate(const mith::EntitySnapshot& self,
                          const mith::NeighbourView& neighbours,
                          const mith::SwarmContext& ctx) override {
        // Build observation vector from EntitySnapshot + NeighbourView
        auto obs = build_obs(self, neighbours);
        int  action_idx = policy_.forward(obs).argmax();
        return action_from_index(action_idx);   // User-defined mapping
    }
};
```

**Rule-based / threshold provider (stigmergy-style):**
```cpp
class ThresholdProvider : public mith::ActionProvider {
    mith::Action evaluate(const mith::EntitySnapshot& self,
                          const mith::NeighbourView& neighbours,
                          const mith::SwarmContext& ctx) override {
        float density = static_cast<float>(neighbours.count()) / MAX_NEIGHBOURS;
        if (density > 0.8f) return {mith::actions::MOVE, 1.0f};   // disperse
        if (density < 0.2f) return {mith::actions::REGROUP, 1.0f};
        return {mith::actions::IDLE, 0.0f};
    }
};
```

### 6.4 ActionExecutorSystem

`ActionExecutorSystem` runs last in every tick (it has a write dependency on almost everything). It:

1. Drains `ActionQueueComponent` for each entity (priority-sorted).
2. Validates `Action.modifies` against a permission mask (configurable per entity, useful for degraded mode).
3. Dispatches to registered `ActionHandler` functions keyed by `ActionTypeID`.
4. Built-in handlers: MOVE updates `VelocityComponent`, TRANSMIT enqueues a `Message` to `CommBufferComponent`, etc.
5. User registers custom handlers for `ActionTypeID >= actions::CUSTOM`.

---

## 7. Communication Model

### 7.1 Two-Channel Architecture

Inter-robot communication is split into two logical channels:

```
┌─────────────────────────────────────┐
│          TransportLayer             │
│                                     │
│  ┌────────────────┐  ┌───────────┐  │
│  │  Beacon Channel│  │  Message  │  │
│  │  (periodic,    │  │  Channel  │  │
│  │   low-prio,    │  │  (event,  │  │
│  │   broadcast)   │  │   higher  │  │
│  └────────────────┘  │   prio)   │  │
│                      └───────────┘  │
└─────────────────────────────────────┘
```

**Beacon Channel** — broadcasts a `StateVector` from each robot at a fixed interval (default 100ms, configurable). Received beacons are processed by `BeaconSystem` into the `NeighbourTable`. This is how every robot maintains a world model of its neighbourhood without any directed communication.

**Message Channel** — typed, directed or broadcast `Message` packets. Used for explicit coordination: task bids, formation commands, fault alerts. Higher transport priority than beacons.

### 7.2 StateVector

The beacon payload. Fixed schema, compact:

```cpp
namespace mith {

struct StateVector {
    HierarchicalID       id;
    PositionComponent    position;
    VelocityComponent    velocity;
    HealthComponent      health;
    RoleComponent        role;
    BehaviourStateComponent state;
    uint32_t             tick;
    // Total: ~80 bytes. Fits in a single UDP packet alongside headers.
};

} // namespace mith
```

### 7.3 Message

```cpp
namespace mith {

using MessageTypeID = uint32_t;

namespace messages {
    constexpr MessageTypeID TASK_BID      = 1;
    constexpr MessageTypeID TASK_ASSIGN   = 2;
    constexpr MessageTypeID FAULT_ALERT   = 3;
    constexpr MessageTypeID FORMATION_CMD = 4;
    constexpr MessageTypeID CUSTOM        = 0x1000;
}

struct Message {
    HierarchicalID  sender;
    HierarchicalID  recipient;     // Broadcast if recipient == BROADCAST_ID
    MessageTypeID   type;
    uint32_t        seq;
    float           timestamp_s;
    std::array<uint8_t, 128> payload;
};

} // namespace mith
```

### 7.4 NeighbourTable

Maintained by `BeaconSystem`. Entries are aged out after a configurable timeout (default 500ms — 5 missed beacons). Thread-safe for concurrent reads from multiple systems.

```cpp
namespace mith {

class NeighbourTable {
public:
    void upsert(const StateVector& sv, float rssi = NaN);
    void age_out(float current_time_s, float timeout_s);

    NeighbourView view_for(EntityID local_entity) const;
    size_t        count() const;
};

} // namespace mith
```

### 7.5 TransportLayer Interface

All comms go through a pluggable transport. Implementations provided: `UDPMulticastTransport`, `SerialTransport`, `SimTransport`.

```cpp
namespace mith {

class TransportLayer {
public:
    virtual ~TransportLayer() = default;

    virtual bool send_beacon(const StateVector& sv)           = 0;
    virtual bool send_message(const Message& msg)             = 0;

    // Called by runtime to drain received data into queues
    virtual void poll(std::vector<StateVector>& beacons_out,
                      std::vector<Message>&     messages_out) = 0;

    virtual bool is_healthy() const = 0;
};

} // namespace mith
```

---

## 8. World — Top-Level Runtime

`World` is the root object. One per robot process.

```cpp
namespace mith {

struct WorldConfig {
    uint16_t            swarm_id;
    float               tick_rate_hz     = 20.0f;
    float               beacon_rate_hz   = 10.0f;
    float               neighbour_timeout_s = 0.5f;
    size_t              thread_pool_size = 0;    // 0 = hardware_concurrency - 1
    bool                enable_flocking      = true;
    bool                enable_task_alloc    = true;
    bool                enable_fault_monitor = true;
};

class World {
public:
    explicit World(WorldConfig config, std::unique_ptr<TransportLayer> transport);

    // Entity management
    EntityID    create_entity();
    void        set_action_provider(EntityID id, std::unique_ptr<ActionProvider> provider);

    // System registration (user systems, in addition to built-ins)
    void        register_system(std::unique_ptr<System> system);

    // Runtime control
    void        init();
    void        tick();             // Single tick — call from your control loop
    void        run(bool& stop_flag); // Blocking loop at configured tick_rate_hz

    EntityRegistry&  registry();
    NeighbourTable&  neighbour_table();
    SwarmContext     context() const;
};

} // namespace mith
```

---

## 9. Simulation Harness

The sim harness allows full swarm scenarios to run in-process with no hardware. Used for demos, testing, CI.

### 9.1 SimTransport

`SimTransport` is a loopback transport backed by an in-process message bus. Multiple `World` instances (each representing one robot) share a `SimBus`. The bus applies configurable latency, packet loss, and range-limiting (robots beyond `comm_range_m` don't receive each other's packets).

```cpp
namespace mith::sim {

struct SimBusConfig {
    float comm_range_m    = 50.0f;
    float packet_loss_pct = 0.0f;
    float latency_ms      = 0.0f;
};

class SimBus {
public:
    explicit SimBus(SimBusConfig config);
    std::unique_ptr<TransportLayer> make_transport(HierarchicalID id,
                                                   PositionProvider pos_fn);
};

} // namespace mith::sim
```

### 9.2 SimClock

A deterministic virtual clock. All `World` instances in a sim share one clock. Tick ordering is round-robin by default, configurable.

### 9.3 Visualiser Hook

The sim harness exposes a `SwarmSnapshot` at each tick — all entity states, neighbour tables, pending actions — serialised to JSON or a binary wire format. A bundled Python script reads this and renders a 2D matplotlib animation. This is not part of the core runtime and has no compile-time dependency.

---

## 10. Repository Structure

```
mith-atomas/
├── CMakeLists.txt
├── README.md
├── ARCHITECTURE.md                  ← this document
├── LICENSE                          ← Apache 2.0
│
├── include/
│   └── mith/
│       ├── mith.h                   ← single include header
│       ├── core/
│       │   ├── entity.h
│       │   ├── registry.h
│       │   ├── component.h
│       │   ├── system.h
│       │   └── world.h
│       ├── identity/
│       │   ├── hierarchical_id.h
│       │   └── uuid.h
│       ├── comms/
│       │   ├── transport.h
│       │   ├── state_vector.h
│       │   ├── message.h
│       │   └── neighbour_table.h
│       ├── behaviour/
│       │   ├── action.h
│       │   ├── action_provider.h
│       │   └── action_executor_system.h
│       └── sim/
│           ├── sim_bus.h
│           └── sim_clock.h
│
├── src/
│   ├── core/
│   │   ├── registry.cpp
│   │   ├── system_scheduler.cpp
│   │   └── world.cpp
│   ├── identity/
│   │   └── uuid.cpp
│   ├── comms/
│   │   ├── udp_multicast_transport.cpp
│   │   ├── serial_transport.cpp
│   │   ├── neighbour_table.cpp
│   │   └── beacon_system.cpp
│   ├── behaviour/
│   │   └── action_executor_system.cpp
│   ├── systems/
│   │   ├── flocking_system.cpp
│   │   ├── task_alloc_system.cpp
│   │   └── fault_monitor_system.cpp
│   └── sim/
│       ├── sim_bus.cpp
│       └── sim_clock.cpp
│
├── transport/
│   ├── udp/
│   └── serial/
│
├── examples/
│   ├── flocking_demo/
│   │   ├── main.cpp
│   │   └── CMakeLists.txt
│   └── rl_policy_demo/
│       ├── main.cpp
│       └── CMakeLists.txt
│
├── tools/
│   └── visualiser/
│       └── visualise.py             ← matplotlib swarm visualiser
│
└── tests/
    ├── unit/
    │   ├── test_registry.cpp
    │   ├── test_scheduler.cpp
    │   ├── test_neighbour_table.cpp
    │   └── test_action_executor.cpp
    └── integration/
        ├── test_flocking_sim.cpp
        └── test_fault_recovery.cpp
```

---

## 11. Build System

CMake 3.21+, C++17 standard.

```cmake
cmake_minimum_required(VERSION 3.21)
project(mith-atomas VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(MITH_BUILD_EXAMPLES  "Build example programs"   ON)
option(MITH_BUILD_TESTS     "Build test suite"          ON)
option(MITH_BUILD_SIM       "Build simulation harness"  ON)
option(MITH_ENABLE_UDP      "Build UDP transport"       ON)
option(MITH_ENABLE_SERIAL   "Build serial transport"    OFF)
```

**Dependencies (all vendored or header-only, no external package manager required):**

| Dependency | Version | Use | Vendored |
|---|---|---|---|
| `nlohmann/json` | 3.11 | Sim snapshot serialisation | Yes (single header) |
| `doctest` | 2.4 | Unit testing | Yes (single header) |
| `ThreadPool` (custom) | — | System scheduler | Internal |
| `etl` (Embedded Template Library) | optional | Heap-free containers on embedded targets | Optional |

No ROS, no Boost, no external comms library required for the core.

---

## 12. CMake Usage Pattern

```cmake
# In a user project:
find_package(mith-atomas REQUIRED)
target_link_libraries(my_robot mith::mith)
```

Or via `add_subdirectory`:
```cmake
add_subdirectory(extern/mith-atomas)
target_link_libraries(my_robot mith::mith)
```

---

## 13. Fault Tolerance Design

Fault tolerance is first-class, not an afterthought.

### 13.1 HealthComponent

`HealthComponent::value` (0–100) is managed by `FaultMonitorSystem`. It decrements on:
- Missed beacon threshold exceeded (comm degradation)
- Hardware sensor timeout (user-reported via `FaultMonitorSystem::report_fault()`)
- ActionExecutorSystem permission violations (entity attempting writes beyond its mask)

### 13.2 Degraded Mode

When `health < DEGRADED_THRESHOLD` (default: 40), `FaultMonitorSystem` sets a `DEGRADED` state on `BehaviourStateComponent`. `ActionExecutorSystem` restricts the permission mask — for example, a degraded robot may only execute `IDLE` and `REGROUP` actions.

### 13.3 Neighbour Fault Detection

`NeighbourTable` age-out is the primary mechanism. When a neighbour disappears from the table, interested systems (TaskAllocSystem, FlockingSystem) are notified via a lightweight observer pattern — no polling.

### 13.4 No Single Point of Failure

The runtime has no global coordinator. Each node operates independently. If a subset of the swarm loses comms entirely, both partitions continue operating in their local context. When comms is restored, the `NeighbourTable` converges automatically within one beacon interval.

---

## 14. Observability

Without runtime introspection, the scheduler, neighbour table, and action validation are opaque — tests can't assert what happened, and bugs surface as silence. MithAtomas exposes structured introspection from the core. No external tooling required to debug.

### 14.1 State snapshot

`World::dump_state()` serialises the full local runtime state to JSON: registry contents, neighbour table entries, pending action queues, last-tick scheduler timings. This is the single canonical introspection path — the matplotlib visualiser (§9.3), unit tests, and any future GUI all consume the same snapshot.

```cpp
namespace mith {

class World {
public:
    // ... existing API ...
    std::string dump_state() const;   // JSON
};

} // namespace mith
```

### 14.2 Scheduler timings

Each tick records per-system wall time and dispatch order. Lets tests verify that systems declared parallel by the DAG actually ran concurrently, and surfaces tick overruns before they cascade.

```cpp
namespace mith {

struct SystemTiming {
    std::string name;
    float       start_us;
    float       duration_us;
    uint32_t    thread_id;
};

class SystemScheduler {
public:
    // ... existing API ...
    std::span<const SystemTiming> last_tick_timings() const noexcept;
};

} // namespace mith
```

### 14.3 Counters on bounded resources

Every bounded queue (`ActionQueueComponent`, `CommBufferComponent`, transport TX queues) maintains drop / overflow counters. Counters are exposed as queryable components — readable via the registry, included in `dump_state()`, and assert-able in tests. This makes the queue overflow policy (whichever is chosen in §15 Pre-v0.1) *verifiable* rather than implicit.

### 14.4 Trace mode

`WorldConfig::trace_level` controls structured logging:

| Level | Emits |
|---|---|
| `OFF` (default) | Nothing |
| `WARN` | Fault transitions, dropped messages, validation rejections |
| `INFO` | System tick boundaries, action validation outcomes |
| `DEBUG` | Per-component reads / writes |

Trace output is one JSON object per line on stderr (or a configurable sink). Production deployments stay at `OFF`; CI integration tests use `INFO`; manual debugging uses `DEBUG`. No external logging dependency.

### 14.5 What this is not

- **Not a metrics service.** No Prometheus endpoint in the core. If you need that, write a system that reads counters and pushes to your stack.
- **Not a tracing protocol.** No OpenTelemetry, no spans. Scheduler timings cover what's actually needed.
- **Not a visualiser.** The matplotlib script and any future GUI live downstream of `dump_state()`. The runtime ships the data; consumers ship the view.

Keeping these out of the core preserves the dependency footprint promised in §11 and the embedded-target focus in §13.

---

## 15. Design Constraints & Non-Goals

**Constraints:**
- No dynamic memory allocation in hot path systems after init. All component stores pre-allocate.
- No exceptions in the runtime core. Error returns via `std::optional` or result types.
- No RTTI in the component system. Type IDs are compile-time hashed strings.
- `ActionProvider::evaluate()` must be non-blocking. The scheduler will not protect against a blocking provider.

**Non-goals for v0.1:**
- Python bindings (planned post-v1.0)
- ROS 2 transport (planned, community contribution welcome)
- Hardware abstraction layer (HAL) — MithAtomas is above the HAL. Use your platform's HAL to feed sensor data into components.
- Centralised mission planner — MithAtomas is a swarm runtime, not a ground control station.
- Security / encrypted comms — transport encryption is the responsibility of the transport implementation.

---

## 16. Roadmap

The schedule below is dependency-ordered: each tier resolves blockers for the next. Pre-v0.1 is a doc-only phase — open architectural questions must be answered in §3–§9 before v0.1 code can be coherently written.

### Pre-v0.1 — Design resolution (doc-only, blocks implementation)

- [ ] **ECS identity model** — commit to one entity per `World`, or describe what additional entities represent. Touches every system signature.
- [ ] **Scheduler hazard model** — promote `NeighbourTable`, comm buffers, and transport state to first-class hazard nodes in the DAG alongside `ComponentTypeID`. Required before any parallelism claim is sound (§5.2).
- [ ] **`ActionExecutorSystem` write set** — split into typed handlers with bounded declared writes, or document the tick-wide barrier behaviour explicitly (§6.4).
- [ ] **Hot-component registration** — compile-time set (own the recompile-to-extend cost) or runtime-registerable (own the perf delta). Pick one and update §4.1.
- [ ] **Determinism scope** — pin sim scheduler to single-threaded, or define schedule capture/replay. Both claims as written conflict (§5.2 vs §9.2).
- [ ] **Bounded-queue overflow contract** — per queue (`ActionQueueComponent`, `CommBufferComponent`): drop-oldest / drop-newest / fault-trigger.
- [ ] **Action permission mask** — where it lives, who writes it, how degraded mode mutates it (§6.4, §13.2).
- [ ] **v0.1 scope coherence** — pull `BeaconSystem` + `NeighbourTable` into v0.1 (the flocking demo depends on them), or push the flocking demo to v0.2. Decide here.
- [ ] **Visualiser dependency** — gate `nlohmann/json` behind `MITH_BUILD_SIM`; drop the "no compile-time dependency" claim in §9.3 or scope it to the core library only.

### v0.1 — Core Runtime (current target)
- [ ] EntityRegistry with hybrid archetype ECS
- [ ] SystemScheduler with async DAG execution (per pre-v0.1 hazard model)
- [ ] HierarchicalID + UUID generation
- [ ] ActionProvider interface + ActionExecutorSystem (per pre-v0.1 write-set decision)
- [ ] Built-in hot components (Position, Velocity, Health, Role, BehaviourState, ActionQueue, CommBuffer)
- [ ] SimTransport + SimBus + SimClock
- [ ] FlockingSystem (Reynolds rules)
- [ ] `BeaconSystem` + `NeighbourTable` *(if pre-v0.1 resolved scope this way)*
- [ ] Flocking demo (50 simulated robots, matplotlib visualiser)
- [ ] Observability primitives — `World::dump_state()`, `SystemScheduler::last_tick_timings()`, queue counters, `trace_level` (see §14)
- [ ] Unit tests for registry, scheduler, neighbour table (leveraging observability primitives for assertions)
- [ ] CMake install target

### v0.2 — Distributed Bootstrap & Channel Separation

Must precede any honest multi-robot claim — the current init path in §3.1 has a hidden coordinator.

- [ ] **Discovery / bootstrap protocol** — how robots acquire `SwarmID` and find peers without a mission controller. Resolves the §0 ↔ §3.1 contradiction.
- [ ] **Channel-aware transport** — split `TransportLayer` into beacon + message transports, or define a multiplexing contract. Lets beacons ride lossy broadcast media while messages use reliable links.
- [ ] UDPMulticastTransport
- [ ] FaultMonitorSystem + degraded mode
- [ ] TaskAllocSystem (threshold-based, pre-partition-merge)
- [ ] Integration test: fault injection in sim

### v0.3 — Time & Space

Both required before scale and fault claims hold. Clock sync must land before v0.4's partition merge.

- [ ] **Clock-sync layer** — pairwise offset estimation piggy-backed on beacons. Makes message timestamps comparable across robots.
- [ ] **Spatial index on `NeighbourTable`** — hash-grid keyed by position. Required for the v1.0 1000-entity benchmark; without it FlockingSystem is O(N²).
- [ ] SerialTransport
- [ ] RL policy demo (dummy policy, shows interface)
- [ ] Tested on Raspberry Pi 4 (reference embedded target)
- [ ] SerialTransport validated on physical robot pair

### v0.4 — Fault Semantics

Partition behaviour is part of the public contract — must land before API freeze.

- [ ] **TaskAllocSystem partition merge** — epoch-leader or version-vector reconciliation for role/assignment state after partition heal. Depends on v0.3 clock sync.
- [ ] **Fault-injection integration tests** exercising the merge path, not just neighbour-table age-out.

### v1.0 — Stable API
- [ ] API stability guarantee (covers a runtime whose architectural questions are all answered)
- [ ] Full documentation (Doxygen)
- [ ] CI (GitHub Actions): build + test on Linux/macOS/ARM
- [ ] Performance benchmarks (1000-entity sim, leveraging v0.3 spatial index)

### Post v1.0
- [ ] Python bindings (pybind11)
- [ ] ROS 2 transport plugin
- [ ] WebSocket transport (for browser-based visualiser)
- [ ] ETL integration for heap-free embedded targets

---

## 17. Contributing & Conventions

- **Branch model:** `main` is always stable. Feature branches off `dev`.
- **Commit style:** conventional commits (`feat:`, `fix:`, `docs:`, `refactor:`)
- **Naming:** `snake_case` for files, functions, variables. `PascalCase` for types. `UPPER_CASE` for constants and macros.
- **Headers:** all public API in `include/mith/`. Implementation details in `src/`. No implementation in headers except templates.
- **Tests:** every public function gets a unit test. No PR merged without tests.
- **No STL heap in hot path:** prefer `std::array`, `std::span`. `std::vector` is allowed in init paths only.

---

*Document version: 0.1.0 — initial architecture*
*Authors: Vighnesh Patidar*
*Repository: github.com/Vighnesh-Patidar/mith-atomas*
