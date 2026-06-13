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

**v0.1 entity count is exactly one per `World`** — the robot itself. The single entity holds all built-in self-state components from §4.4. Neighbours are not entities; they live in `NeighbourTable` (§7.4) as stale-by-default observations with their own lifecycle metadata. `World::self_id()` (§8) returns the one `EntityID`.

The ECS API is shaped for N>1 (queries, archetypes, `create_entity`) but operates as a degenerate N=1 registry in v0.1. The shape stays for forward compatibility — multi-entity uses (sub-agents, task-as-entity, in-process multi-robot sim) are a **v0.5 extension on demand** behind a `WorldConfig::multi_entity` flag, not an API break. See §16 v0.5.

Parallelism in MithAtomas comes from running **many systems concurrently on one self entity**, not from entity-parallel iteration. See §4.1 and §5.2.

### 3.3 Authentication & Spoofing

Identity is uniquely *generated* by §3.1. It is not, by default, uniquely *provable* — a UUID v4 alone says nothing about whether the sender actually owns it. This section adds optional cryptographic binding so that claiming a `HierarchicalID` requires possessing the matching keypair.

#### Threat model

| Threat | Defended by | Layer |
|---|---|---|
| Spoofing — claim a `HierarchicalID` you don't own | Cryptographic identity binding (this section) | Identity |
| Message tampering | Per-packet signature, verified on receipt | Identity → Transport |
| Replay | Signature over `{tick, seq}` | Identity → Transport |
| Sybil — mint many identities | Keypair generation + application-level allow-list | Identity + Application |
| Trust policy — *who* is allowed in the swarm | Allow-list / attestation | Application |

#### Two-mode identity

**Unsigned mode** (default; research, sim, trusted single-host): `HierarchicalID` is exactly §3.1. No crypto. Cheap.

**Signed mode** (production, hostile RF, mesh relay): `UnitID = BLAKE3(pub_key)[0..16]`, bound by construction to an Ed25519 keypair. Beacons and messages carry an Ed25519 signature (~64 bytes). Spoofing requires the private key.

Both modes use the **same `HierarchicalID` wire layout** — what changes is whether an `IdentityKey` is associated with the sender and whether an `IdentityVerifier` is plugged into the transport on the receiver.

```cpp
namespace mith {

// Companion to HierarchicalID, present only in signed mode.
struct IdentityKey {
    static constexpr size_t PUB_LEN = 32;   // Ed25519 public key
    std::array<uint8_t, PUB_LEN> public_key;
};

// Plugged into TransportLayer. Default = no-op verifier (unsigned mode).
class IdentityVerifier {
public:
    virtual ~IdentityVerifier() = default;
    virtual bool verify(const HierarchicalID& claimed,
                        std::span<const uint8_t> payload,
                        std::span<const uint8_t> signature) const noexcept = 0;
};

} // namespace mith
```

Private keys never appear in any serialised form. They live in a sender-only `IdentityPrivateKey` struct that's never copied, moved across processes, or written to disk in cleartext (storage at rest is the deployer's responsibility — keyring, TPM, secure element).

#### What this section is NOT

- **Not a trust policy.** "This pubkey is allowed in our swarm" is an application-layer decision (mission config, attestation, MIS). MithAtomas verifies *that the sender owns the claimed identity*, not *that we want to listen to them*.
- **Not encryption.** Channel confidentiality stays with the transport.
- **Not a CA model.** Self-sovereign, no central authority. Sybil resistance is delegated to the application.

Implementation lands in v0.2 alongside channel-aware transport (§16). v0.1 ships unsigned-mode types and the `IdentityVerifier` interface only; the Ed25519 implementation arrives in v0.2 as a vendored, opt-in CMake feature (`MITH_ENABLE_AUTH`).

#### Entropy source

UUID generation and (future) keypair generation depend on a cryptographically secure RNG. v0.1 reads directly from `std::random_device`, which is CSPRNG-backed on Linux (≥ 5.3), macOS, Windows, and recent MinGW. PRNG expansion via `mt19937` is **not** used — Mersenne Twister state can be recovered from ~625 outputs, making it unsafe once rotation modes (§3.4) generate multiple identities per robot.

v0.2 (`MITH_ENABLE_AUTH`) replaces `random_device` with a vendored ChaCha20 CSPRNG, removing the platform-quality dependency entirely. If `std::random_device` is degraded or unavailable on a target, the runtime cannot safely produce identity material and terminates rather than emit predictable identifiers.

### 3.4 Identity Rotation

Identity is **permanent by default**: `UnitID` is generated once on first boot and persisted (§3.1). Stable identity is load-bearing for `NeighbourTable` (§7.4), `TaskAllocSystem` (§5.3), and fault tracking (§13.1).

Rotation is **optional** and configurable. Trade-off: rotation breaks long-running reputation, formation memory, and task-allocation history. Use only when the deployment requires it.

| Policy | Behaviour | Use case |
|---|---|---|
| `PERMANENT` (default) | `UnitID` stable for the robot's lifetime | Research, sim, industrial |
| `PER_MISSION` | Rotates on `SwarmID` change or mission boundary | Mission compartmentalization |
| `PERIODIC` | Rotates every `WorldConfig::rotation_interval_s` | Privacy / anti-tracking |
| `EVENT_DRIVEN` | Application calls `World::rotate_identity()` | Compromise response |

```cpp
namespace mith {

enum class IdentityRotationPolicy : uint8_t {
    PERMANENT     = 0,
    PER_MISSION   = 1,
    PERIODIC      = 2,
    EVENT_DRIVEN  = 3,
};

} // namespace mith
```

**Unsigned mode rotation**: generate new UUID, persist, announce via the next beacon. NeighbourTable entries for the prior ID age out naturally. No correlation — neighbours treat the rotated robot as a new one.

**Signed mode rotation**: generate new keypair, derive new UUID, issue an `IdentityCertificate` signed by the *previous* private key (continuity proof), then destroy the previous private key (forward secrecy). Announce the cert via the next beacon. Receivers correlate old → new through the cert chain without losing reputation / task history.

```cpp
namespace mith {

// Continuity proof binding a new identity to its predecessor.
// Only used in signed mode.
struct IdentityCertificate {
    HierarchicalID                              new_id;
    IdentityKey                                 new_key;
    HierarchicalID                              prev_id;
    std::array<uint8_t, 64>                     signature_by_prev;  // Ed25519
    float                                       issued_at_s;
};

} // namespace mith
```

Implementation lands in v0.2 alongside §3.3. v0.1 ships only `IdentityRotationPolicy` and the `World::rotate_identity()` API stub.

---

## 4. Entity & Component System (ECS)

### 4.1 Design: Hybrid Archetype Model

**Parallelism axis.** MithAtomas's ECS is shaped for many-entity iteration but v0.1 runs at N=1 per `World` (§3.2). Concurrency comes from the `SystemScheduler` (§5.2) running multiple systems on the same self entity in parallel, gated by component-level read/write hazards — *not* from entity-parallel iteration within a single system. Data-parallel hot paths (e.g. iterating `NeighbourTable` in `FlockingSystem` or `BeaconSystem`) live inside the system's own loop and are independent of the entity model.

The archetype machinery below is implementation flexibility for the eventual N>1 case; at N=1 it degenerates to per-component slots with O(1) access. The cost is one layer of indirection; the benefit is uniform state access and a non-breaking path to multi-entity.

MithAtomas uses a **hybrid archetype ECS**:

- **Hot components** (accessed every tick by multiple systems) are stored in **dense archetype-style arrays**. Components for the same type across all entities with that component are contiguous in memory. Cache-friendly.
- **Cold components** (sparse, infrequent access, user-defined) are stored in a **per-entity component map** (`unordered_map<ComponentTypeID, ComponentPtr>`). Flexible. No cache pressure because they're not in the hot path.

The hot/cold distinction is a **storage strategy hint** declared by the component author. It is *not* a registration mechanism.

```cpp
// Hot — gets a dense slot
struct PositionComponent : mith::HotComponent<PositionComponent> {
    float x, y, z;
};

// Cold — stored in the flexible map
struct MissionTagComponent : mith::ColdComponent<MissionTagComponent> {
    std::string mission_id;
    uint8_t     priority;
};
```

**Registration is unified**: built-in and user components both flow through `EntityRegistry::register_component<T>()`, called by the runtime in `World::init()` for the built-in list (§4.4) and by user code before init for extensions. Type IDs remain compile-time hashed strings (§15); registration allocates storage for that ID at runtime.

Every registration carries a **`ComponentOrigin`** tag that the runtime enforces by API surface:

```cpp
namespace mith {

enum class ComponentOrigin : std::uint8_t {
    Built_In = 0,   // Registered by the runtime in World::init(). Privileged path.
    User     = 1,   // Registered by mission/plugin code via register_component<T>().
};

} // namespace mith
```

`Built_In` is set by an internal `register_builtin_component<T>()` path that user code cannot reach — there is no public overload of `register_component<T>(Origin)`. This makes the origin tag tamper-resistant within the trusted compute base.

Registration is gated by **`WorldConfig::registration_policy`** (§8). Three levels:

- **`Open`** — registration allowed any time. Test / sim convenience.
- **`LockAfterInit`** (default) — register before `World::init()`; locked after.
- **`BuiltInOnly`** — only `Built_In` origin accepted; user `register_component<T>()` calls are hard-rejected. This is the **EW deployment posture** — see §13.5.

Every registration emits a structured `component_registered` event to the observability stream (§14) carrying origin, type name, type ID, and tick — the audit substrate for post-mission review.

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

The central registry owns all archetypes and the cold component maps. The API below is shaped for multi-entity use; in v0.1 (§3.2) the registry holds exactly one entity, so `create_entity()`, `destroy_entity()`, and `view<>` are forward-compatibility hooks that degenerate to N=1 operations. Component access (`get`, `has`, `emplace`, `remove`) and `snapshot()` are the load-bearing parts at v0.1.

```cpp
namespace mith {

class EntityRegistry {
public:
    EntityID    create_entity();
    void        destroy_entity(EntityID id);

    // Component registration — public path, always tagged ComponentOrigin::User.
    // Must be called before World::init() under the default LockAfterInit policy.
    // Rejected under BuiltInOnly policy (§13.5). Type ID collisions error here,
    // not later — see §15.
    template<ComponentType T>
    void        register_component();

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

These are registered by the runtime in `World::init()` through the privileged `register_builtin_component<T>()` path (§4.1), giving them `ComponentOrigin::Built_In`. They are *not* exempted from the registration audit trail — registrations of built-ins emit the same `component_registered` event to the observability stream (§14), with `origin: built_in`.

| Component | Fields | Notes |
|---|---|---|
| `IdentityComponent` | `HierarchicalID id` | Set once at init |
| `PositionComponent` | `float x, y, z` | World-frame position |
| `VelocityComponent` | `float vx, vy, vz` | Current velocity |
| `OrientationComponent` | `float qw, qx, qy, qz` | Quaternion |
| `HealthComponent` | `uint8_t value` | 0–100, managed by FaultMonitorSystem |
| `RoleComponent` | `RoleID role` | Current swarm role (worker, scout, relay...) |
| `BehaviourStateComponent` | `StateID state` | Current state, used by ActionProvider |
| `ActionQueueComponent` | `BoundedQueue<Action, 8, DropNewest>` + rejection counters | Pending actions. Overflow keeps in-flight intent and drops new (§4.5). Permission rejections counted per §6.4. |
| `CommBufferComponent` | `BoundedQueue<Message, 16, DropOldest>` | Inbound messages. Overflow evicts stale for fresh (§4.5). |
| `PermissionMaskComponent` | `uint32_t allowed_builtins` + `bool allow_user_actions` | Gates which `ActionTypeID`s `ActionValidatorSystem` accepts. Written by `FaultMonitorSystem` (§13.2); read by `ActionValidatorSystem`. Default = fully permissive. |

### 4.5 Bounded Queues

Every bounded queue in the runtime — built-in or user — declares its overflow behaviour as part of its type. There is no global policy and no "what should I do?" branch at the call site. The policy is a template parameter; `push()` is branchless on the hot path.

```cpp
namespace mith {

enum class OverflowPolicy : std::uint8_t {
    DropOldest    = 0,   // FIFO eviction. Newest survives; oldest evicted to make room.
                         // Use for transient/observational streams (e.g. inbound beacons,
                         // outbound state vectors) where staleness > loss.
    DropNewest    = 1,   // FIFO reject. A full queue refuses new items.
                         // Use when in-flight items must complete and overflow signals
                         // overload (e.g. queued actions whose intent was already validated).
    FaultTrigger  = 2,   // DropNewest + raise an overflow fault to FaultMonitorSystem (§13.1).
                         // Use for critical user queues where any drop is a fault condition.
};

template<typename T, std::size_t N, OverflowPolicy Policy>
class BoundedQueue {
public:
    // Returns true if accepted, false on drop. Never blocks, never allocates,
    // never throws (per §15 constraints).
    bool                push(T&& item) noexcept;

    std::optional<T>    pop() noexcept;

    std::size_t         size() const noexcept;
    constexpr std::size_t capacity() const noexcept { return N; }

    // §14.3 observability — exposed via dump_state() and assert-able in tests.
    std::uint32_t       dropped_count() const noexcept;
    std::uint32_t       overflow_events() const noexcept;
};

} // namespace mith
```

`dropped_count` counts individual items lost; `overflow_events` counts the number of distinct overflow occurrences (one event may drop multiple items in `DropOldest` if a burst arrives at a full queue). Both are exposed by `dump_state()` (§14.1) and rendered into the §14.4 trace stream.

**Built-in queue policies** (referenced from §4.4):

| Queue | Capacity | Policy | Rationale |
|---|---|---|---|
| `ActionQueueComponent::queue` | 8 | `DropNewest` | Already-queued actions were validated by `ActionValidatorSystem` (§5.3, §6.4). Losing new pushes and surfacing the count is preferable to evicting in-flight intent. |
| `CommBufferComponent::queue` | 16 | `DropOldest` | Network bursts are expected: 16 slots × 50 neighbours × 100 Hz beacon = ~3 ms to fill. Old messages are stale; transport-level retry handles critical traffic. |
| Transport TX (per impl, §7.5) | impl-defined | `DropOldest` recommended | State vectors are observational; latest supersedes. Custom transports declare their own. |

**`FaultTrigger` mechanics.** When a `BoundedQueue<..., FaultTrigger>` overflows:

1. `push()` returns `false`, increments `dropped_count` and `overflow_events`.
2. `FaultMonitorSystem` (§13.1) reads the counter deltas on its next tick — the §14.3 counters *are* the signal; there is no global side-channel.
3. `FaultMonitorSystem` decrements `HealthComponent::value` and emits a `WARN`-level trace (§14.4): `{"event": "queue_overflow", "queue": "<component_name>", "dropped": N}`.
4. If health crosses `DEGRADED_THRESHOLD`, the normal §13.2 degraded-mode response applies.

This makes overflow a first-class fault condition without inventing a new signaling mechanism. Under `Sequential` scheduler mode (§5.2) the overflow-to-detection ordering is deterministic; under `Parallel` it is bounded by one `FaultMonitorSystem` tick.

User queues that need a policy not covered by the three above (e.g. blocking push, custom eviction) should wrap their own ring buffer rather than extend the enum — the three policies cover all cases the runtime promises to integrate with the fault path.

---

## 5. System Model

### 5.1 System Interface

A System is a unit of logic that reads and/or writes components and/or non-component shared resources (`NeighbourTable`, transport buffers). Systems are the only code allowed to mutate component data or shared resources.

The hazard graph is **two-axis**: components *and* resources. The original (component-only) model in earlier drafts silently allowed `BeaconSystem` to write `NeighbourTable` while `FlockingSystem` read from it. Making resources first-class closes that gap.

```cpp
namespace mith {

// Non-component shared state, tracked alongside components in the hazard graph.
// Built-in identifiers are stable; user code can register more at init via
// SystemScheduler::register_resource().
enum class ResourceID : std::uint16_t {
    NeighbourTable = 0,
    TransportTx    = 1,
    TransportRx    = 2,
    // Spatial index (v0.3), identity verifier (v0.2), etc., land in this range.
    First_User     = 0x1000,
};

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

    // Declare read/write access on both axes — drives the SystemScheduler DAG.
    virtual SystemDescriptor describe() const = 0;

    // Called every tick. registry is passed as the access point for components;
    // resources are accessed via SwarmContext (transport, NeighbourTable, etc.).
    virtual void tick(EntityRegistry& registry,
                      const SwarmContext& ctx,
                      float delta_time) = 0;
};

} // namespace mith
```

Two systems conflict iff they share any (component or resource) where at least one writes. Two systems run in parallel iff they have no overlapping hazards on either axis. This is the standard ECS framework shape (Bevy, Specs, Flecs).

### 5.2 SystemScheduler — Async DAG Execution

The scheduler builds a **dependency graph** from system descriptors at startup over both hazard axes (§5.1): components *and* resources. Two systems have a dependency if one writes a component or resource that the other reads or writes (W-W, R-W, W-R hazards on either axis).

At each tick:
1. Systems with no unresolved dependencies are dispatched to a **thread pool** concurrently.
2. When a system completes, its dependents are checked — if all their dependencies are resolved, they are dispatched.
3. The tick completes when all systems have run.

This is a standard parallel task DAG, equivalent to a topological sort with concurrent execution at each level.

**Two execution modes** are supported, selectable per `World` via `WorldConfig::scheduler_mode` (§8):

- **`Parallel`** (default in production) — thread pool dispatches independent systems concurrently per the description above. Maximum throughput. **Determinism is not guaranteed** between runs: even with a perfect hazard graph, thread-completion ordering, FP-reduction order on different cores, and CPU contention all leak non-determinism into shared state.
- **`Sequential`** (default in sim, see §9.1) — single-threaded execution in a topologically-sorted order with stable tie-breaking (lexicographic on `SystemDescriptor::name`). Slower per tick, but **fully deterministic given fixed inputs** (RNG seeds, transport feed, beacon arrivals). Re-running with the same inputs produces bit-identical traces.

The hazard graph is the same in both modes. The difference is whether the scheduler exploits the parallel opportunities the graph permits.

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
    ThreadPool                              thread_pool_;   // Unused in Sequential mode
};

} // namespace mith
```

**Thread pool sizing:** Defaults to `std::thread::hardware_concurrency() - 1` in `Parallel` mode. On a single-core embedded target, degrades gracefully to sequential execution. `Sequential` mode ignores `thread_pool_size`.

### 5.3 Built-in Systems

The following systems ship with MithAtomas. All are optional — disable any in the `WorldConfig`.

Reads / writes split into **components** (C:) and **resources** (R:) per the two-axis hazard model in §5.1. `ActionExecutorSystem` is no longer a single system — it is the **Action Handler Registry** described in §6.4.

| System | Reads | Writes | Notes |
|---|---|---|---|
| `BeaconSystem` | C: Identity, Position, Velocity, Health, Role · R: `TransportRx` | C: CommBuffer (outbound) · R: `NeighbourTable`, `TransportTx` | Broadcasts StateVector; processes incoming beacons into NeighbourTable |
| `FlockingSystem` | C: Position, Velocity · R: `NeighbourTable` | C: Velocity | Reynolds rules: separation, alignment, cohesion |
| `TaskAllocSystem` | C: Role, BehaviourState · R: `NeighbourTable` | C: Role | Distributed task allocation via auction or threshold |
| `FaultMonitorSystem` | C: Health, CommBuffer, ActionQueue (counters) | C: Health, BehaviourState, PermissionMask | Detects comm loss, hardware faults, queue overflows (§4.5); on degradation, restricts PermissionMask per §13.2 |
| `ActionValidatorSystem` | C: ActionQueue, PermissionMask | C: ActionQueue (marks rejected, increments counter) | Single barrier before action handlers — rejects actions disallowed by PermissionMaskComponent (§6.4, §13.2) |
| `MoveActionHandler` | C: ActionQueue | C: Velocity | Drains MOVE actions, updates velocity |
| `TransmitActionHandler` | C: ActionQueue | R: `TransportTx` | Drains TRANSMIT actions, enqueues `Message` for outbound transport |
| `<user handler>` | C: ActionQueue + user-declared | user-declared bounded set | Registered via `World::register_action_handler<H>(action_type)` |

Action handlers writing disjoint components/resources run in parallel. The shared read on `ActionQueueComponent` is R-R, no conflict.

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

### 6.4 Action Handler Registry

Earlier drafts had a single `ActionExecutorSystem` that declared "writes (all writable)" — which collapsed the scheduler DAG into a single barrier every tick, killing parallelism. The fix is to **split execution into handler systems**, each with a bounded, statically declared write set that the §5.1 hazard graph can reason about.

Action execution becomes a small pipeline of regular Systems (visible in §5.3's table):

1. **`ActionValidatorSystem`** runs once. Reads `ActionQueueComponent` and `PermissionMaskComponent` (§4.4, §13.2). For each pending action, checks `mask.allows(action.type)`. Rejected actions are marked in-place (the handlers skip them) and increment a `permission_rejections_total` counter on `ActionQueueComponent` (assert-able in tests via `dump_state()`, §14.1). A `WARN`-level trace event is emitted (§14.4):

    ```json
    {"event": "action_rejected", "action_type": "MOVE",
     "reason": "permission_mask", "current_state": "DEGRADED", "tick": 1234}
    ```

    Rejections do **not** decrement `HealthComponent` — that would create a positive-feedback loop with degraded mode. Applications wanting harsher behaviour can read the counter and escalate themselves.
2. **Action handler systems** run concurrently. Each handler is a `System` registered for one `ActionTypeID`. It reads `ActionQueueComponent`, drains the actions matching its type that survived validation, and applies its declared writes. Handlers writing disjoint components/resources run in parallel.

Built-in handlers ship with the runtime:

- `MoveActionHandler` — writes `VelocityComponent`.
- `TransmitActionHandler` — writes the `TransportTx` resource (enqueues a `Message` for outbound transport).
- `HoverActionHandler` — IDLE-like, no writes declared.
- Additional handlers land alongside their action types.

User code registers custom handlers:

```cpp
class MyScanHandler : public mith::System {
public:
    mith::SystemDescriptor describe() const override {
        return {
            "MyScanHandler",
            /* reads_components  */ {component_id<ActionQueueComponent>()},
            /* writes_components */ {component_id<MyScanResultComponent>()},
            /* reads_resources   */ {},
            /* writes_resources  */ {},
        };
    }
    void tick(mith::EntityRegistry&, const mith::SwarmContext&, float) override { /* ... */ }
};

// At World init:
world.register_action_handler<MyScanHandler>(actions::CUSTOM + 1);
```

Each registration becomes a `System` in the scheduler graph with the handler's declared writes — same shape as any built-in system, fully visible to the DAG. There is no special "action executor" code path; action dispatch is just N parallel systems sharing a read on `ActionQueueComponent`.

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

Each transport owns its own outbound TX queues and must declare an `OverflowPolicy` (§4.5) per channel. **`DropOldest` is the recommended default** for state-vector traffic (latest supersedes; loss is acceptable). Reliable-channel transports may pair `DropNewest` or `FaultTrigger` with an application-level ack/retry. The chosen policy is documented in the transport's header and surfaces in `dump_state()` (§14.1).

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

`World` is the root object. One instance per robot process; holds exactly one entity (the robot itself, §3.2). `World` is *not* a wrapper over that entity — it is "robot-as-runtime-process": registry + scheduler + neighbour table + transport + swarm context + lifecycle. The entity inside is "robot-as-embodied-state" — composable self-state via §4.4 components.

```cpp
namespace mith {

enum class ComponentRegistrationPolicy : std::uint8_t {
    Open           = 0,   // Registration allowed any time. Test/sim only.
    LockAfterInit  = 1,   // Default — register before init(); locked after.
    BuiltInOnly    = 2,   // EW posture — user register_component<T>() hard-rejected. See §13.5.
};

enum class SchedulerMode : std::uint8_t {
    Parallel    = 0,   // Default in production. Thread pool. Determinism NOT guaranteed (§5.2).
    Sequential  = 1,   // Default in sim (§9.1). Single-threaded topo order with stable
                       // tie-breaking. Fully deterministic given fixed inputs.
};

struct WorldConfig {
    uint16_t            swarm_id;
    float               tick_rate_hz     = 20.0f;
    float               beacon_rate_hz   = 10.0f;
    float               neighbour_timeout_s = 0.5f;
    size_t              thread_pool_size = 0;    // 0 = hardware_concurrency - 1 (Parallel only)
    bool                enable_flocking      = true;
    bool                enable_task_alloc    = true;
    bool                enable_fault_monitor = true;

    ComponentRegistrationPolicy registration_policy = ComponentRegistrationPolicy::LockAfterInit;
    SchedulerMode               scheduler_mode      = SchedulerMode::Parallel;
};

class World {
public:
    explicit World(WorldConfig config, std::unique_ptr<TransportLayer> transport);

    // The one self entity (§3.2). At v0.1 N=1; reserved as a forward-compat handle.
    EntityID                 self_id() const noexcept;

    // Shortcut for `registry.get<IdentityComponent>(self_id()).id`. Stable across the
    // lifetime of this World unless identity rotation (§3.4) is enabled.
    const HierarchicalID&    identity() const noexcept;

    // Entity management — degenerate at v0.1 (registry already holds the one self).
    // Reserved for v1.0+ multi-entity uses (sub-agents, task-as-entity).
    EntityID    create_entity();
    void        set_action_provider(EntityID id, std::unique_ptr<ActionProvider> provider);

    // System registration (user systems, in addition to built-ins)
    void        register_system(std::unique_ptr<System> system);

    // Runtime control
    void        init();
    void        tick();             // Single tick — call from your control loop
    void        run(std::atomic<bool>& stop_flag); // Blocking loop at configured tick_rate_hz

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

`SimBus::make_world_config()` produces a `WorldConfig` with `scheduler_mode = SchedulerMode::Sequential` set by default (§8). This is what makes the sim deterministic — without it, the thread pool reshuffles system-completion order across runs and `SimClock` (§9.2) loses its determinism guarantee. Users running perf benchmarks in sim can override to `Parallel` and accept non-determinism.

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

A virtual clock advancing in fixed `delta_time_s` increments. All `World` instances in a sim share one clock. Tick ordering across Worlds is round-robin by default, configurable.

**Determinism contract:** `SimClock` produces bit-identical traces across runs **only when every participating `World` uses `SchedulerMode::Sequential`** (§5.2). This is the `SimBus` default (§9.1). Mixing modes — one World in `Sequential` and another in `Parallel` — gives partial determinism (per-World traces reproducible only in isolation) and is supported but not recommended.

Determinism also requires the caller to control all other input sources: RNG seeds (`UUID::generate()` in unsigned mode, future Ed25519 keygen in v0.2), transport feed (sim-replayable), and any user-supplied `ActionProvider` that has its own randomness. The runtime commits to *scheduler* determinism; *input* determinism is the caller's responsibility.

A richer "capture the parallel schedule, replay it deterministically" mode is on the post-v1.0 roadmap (§16) for advanced debugging of production-mode behaviour; v0.1 does not include it.

### 9.3 Visualiser Hook

The sim harness exposes a `SwarmSnapshot` at each tick — all entity states, neighbour tables, pending actions — serialised to JSON via the core's hand-rolled writer (§14.1). A bundled Python script reads this stream and renders a 2D matplotlib animation. The **Python visualiser imposes no build-time dependency on the C++ core** — Python is downstream of the snapshot output.

The sim harness itself vendors `nlohmann/json` (§11) under `MITH_BUILD_SIM` for snapshot *parsing* (replay, run-diff, scenario replay across saved traces). The core runtime does not depend on it.

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

| Dependency | Version | Scope | Use | Vendored |
|---|---|---|---|---|
| *(none)* | — | **Core runtime** | Zero external deps. JSON emission for `dump_state()` and trace output is hand-rolled (§14.1). | — |
| `nlohmann/json` | 3.11 | Sim + tests (gated by `MITH_BUILD_SIM`) | Snapshot parsing, replay, run-diff | Yes (single header) |
| `doctest` | 2.4 | Tests only | Unit testing | Yes (single header) |
| `ThreadPool` (custom) | — | Core runtime | System scheduler under `SchedulerMode::Parallel` (§5.2) | Internal |
| `etl` (Embedded Template Library) | optional | Future microcontroller targets | Heap-free containers if/when we expand the tier (post-v1.0) | Optional |

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
- Bounded-queue overflow under `FaultTrigger` policy — `FaultMonitorSystem` reads `BoundedQueue::overflow_events` deltas (§4.5) and decrements health per event.

Action rejections by `ActionValidatorSystem` (§6.4) do **not** decrement health by design — see §6.4 for the rationale.

### 13.2 Degraded Mode

When `health < DEGRADED_THRESHOLD` (default: 40), `FaultMonitorSystem`:

1. Sets `DEGRADED` on `BehaviourStateComponent`.
2. Snapshots the current `PermissionMaskComponent` (held internally on `FaultMonitorSystem`, not on the component itself — keeps the component POD).
3. Writes a restricted `PermissionMaskComponent`:
   - `allowed_builtins = (1u << actions::IDLE) | (1u << actions::REGROUP)`
   - `allow_user_actions = false`

When health recovers above `DEGRADED_THRESHOLD + HYSTERESIS` (default hysteresis: 10), `FaultMonitorSystem` restores the snapshotted mask and clears `DEGRADED`. The hysteresis band prevents thrashing when health oscillates near the threshold.

Under `Sequential` scheduler mode (§5.2) the Fault → Validator ordering is deterministic each tick; under `Parallel`, the §5.1 hazard graph still forces Fault-then-Validator within a tick (Fault writes `PermissionMaskComponent`, Validator reads it). The only non-determinism is *which* tick the transition lands on — exactly the case hysteresis covers.

`ActionValidatorSystem` honours the mask on its next tick (§6.4). Action handlers themselves are not aware of the mask; rejection happens upstream of dispatch.

### 13.3 Neighbour Fault Detection

`NeighbourTable` age-out is the primary mechanism. When a neighbour disappears from the table, interested systems (TaskAllocSystem, FlockingSystem) are notified via a lightweight observer pattern — no polling.

### 13.4 No Single Point of Failure

The runtime has no global coordinator. Each node operates independently. If a subset of the swarm loses comms entirely, both partitions continue operating in their local context. When comms is restored, the `NeighbourTable` converges automatically within one beacon interval.

### 13.5 Adversarial Posture (EW)

For electronic-warfare and other adversarial deployments, the runtime supports a **hardened registration posture** that prevents arbitrary component types from being instantiated at runtime. Combined with §3.3 signed identity, this raises the bar for a hostile actor trying to join or subvert a fleet through anything short of full binary compromise.

| Threat | Defended by |
|---|---|
| Hostile node spoofing identity over the air | §3.3 signed mode + application trust policy |
| Hostile-but-trusted-build plugin slipped into the deployment (supply chain, accidental dependency) | **This section** — `BuiltInOnly` registration policy |
| Captured node broadcasts our positions using existing components | §3.3 signed mode + verification on receive |
| Captured binary tampered with at the flash level | **Out of scope for the runtime** — handled by the deployment's secure-boot / flash-integrity layer (planned alongside the firmware work) |

**Hardened deployment recipe:**

1. Custom-build MithAtomas with all mission-specific components compiled in as `Built_In` (via the `MITH_BUILTIN_COMPONENT(...)` CMake hook). These become part of the trusted compute base; the build supply chain is what guarantees their integrity.
2. Set `WorldConfig::registration_policy = BuiltInOnly`. Any `register_component<T>()` call from user code is rejected before init even runs.
3. Set `MITH_ENABLE_AUTH=ON` and signed mode (§3.3) — bind identity to keypairs so an attacker on the same medium cannot impersonate a fleet member.
4. Trace level `INFO` or above so the `component_registered` audit events (§14) are captured for post-mission review.

What this **does not** defend against:

- **Binary tampering on a captured node.** A flash-level modification of the runtime can do anything; runtime policy is moot. Defence-in-depth here is secure-boot, signed firmware, and TPM-attested boot measurements — out of scope for v0.1 and handled at the firmware layer.
- **Side-channel exfiltration via legitimate components.** If a hostile but trusted-build component is in the `Built_In` set, it can do whatever it wants. The defence is build-pipeline auditing, not runtime enforcement.
- **Application trust policy.** The runtime authenticates *that the claimed identity is legitimate*; the application decides *whether to act on it*. Allow-lists, attestation, and mission-issued certs live above the runtime.

This section will expand alongside the firmware/flash integrity work in a later release.

---

## 14. Observability

Without runtime introspection, the scheduler, neighbour table, and action validation are opaque — tests can't assert what happened, and bugs surface as silence. MithAtomas exposes structured introspection from the core. No external tooling required to debug.

### 14.1 State snapshot

`World::dump_state()` serialises the full local runtime state to JSON: registry contents (including `PermissionMaskComponent` and queue rejection counters, §4.4), neighbour table entries, pending action queues, last-tick scheduler timings. This is the single canonical introspection path — the matplotlib visualiser (§9.3), unit tests, and any future GUI all consume the same snapshot.

**JSON emission is hand-rolled in the core** — a small (~50 LOC) write-only writer covering strings, numbers, booleans, arrays, objects. Emit-only is trivial; the core takes no external JSON dependency (`nlohmann/json` is gated by `MITH_BUILD_SIM`, §11). `dump_state()` allocates (`std::string`) and is intended for explicit debugging calls and per-tick sim snapshots, not the hot tick path — see §15 for the allocation-policy boundary and the platform-tier note.

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

Every `BoundedQueue<T, N, Policy>` instance (§4.5) exposes `dropped_count` and `overflow_events`. Built-in queues — `ActionQueueComponent`, `CommBufferComponent`, transport TX — surface these counters via `dump_state()` (§14.1); custom user queues inherit the same observability by using the same template.

These counters are *the* signaling substrate for the `FaultTrigger` policy: `FaultMonitorSystem` reads `overflow_events` deltas across all bounded queues and drives §13.1 health decrements from them. No separate fault-event channel — the counters are the contract. This makes overflow behaviour verifiable in tests (`CHECK(queue.dropped_count() == 0)`) and replayable in deterministic-mode traces (§5.2 `Sequential`).

### 14.4 Trace mode

`WorldConfig::trace_level` controls structured logging:

| Level | Emits |
|---|---|
| `OFF` (default) | Nothing |
| `WARN` | Fault transitions, dropped messages, validation rejections |
| `INFO` | System tick boundaries, action validation outcomes, **`component_registered` audit events** (§4.1 / §13.5) |
| `DEBUG` | Per-component reads / writes |

Production deployments stay at `OFF`; CI integration tests use `INFO`; manual debugging uses `DEBUG`. EW deployments (§13.5) run at `INFO` so the component-registration audit trail is captured.

**Trace output is pluggable via `TraceSink`.** The runtime assembles each event as `(level, event_name, [key-value fields])` and hands it to a sink. The default sink is `JsonTraceSink` writing one JSON object per line to stderr. Embedded users plug in their own sink (binary frame, logfmt, MAVLink-style telemetry, syslog) without touching runtime code:

```cpp
namespace mith {

struct TraceField {
    std::string_view key;
    enum class Kind : std::uint8_t { Str, I64, U64, F64, Bool } kind;
    // tagged union — POD, no allocation
};

class TraceSink {
public:
    virtual ~TraceSink() = default;
    virtual void emit(TraceLevel level,
                      std::string_view event,
                      std::span<const TraceField> fields) noexcept = 0;
};

class JsonTraceSink : public TraceSink { /* hand-rolled JSON line writer, default */ };

// In WorldConfig:
//   std::unique_ptr<TraceSink> trace_sink;   // default-constructed = JsonTraceSink to stderr

} // namespace mith
```

The runtime's emit path is allocation-free up to the sink call; the sink itself decides whether to allocate. `JsonTraceSink` does (`std::string` per line); a binary sink can pre-allocate ring buffers and never alloc per event.

Sample `JsonTraceSink` output for the `component_registered` event:

```json
{"event": "component_registered",
 "origin": "user", "type_name": "MyKalmanState", "type_id": "0x9d2f...",
 "tick": 0, "wall_time_s": 1.234}
```

A binary sink for tight platforms is listed in §16 post-v1.0 — interface lands now so the API doesn't break later. No external logging dependency.

### 14.5 What this is not

- **Not a metrics service.** No Prometheus endpoint in the core. If you need that, write a system that reads counters and pushes to your stack.
- **Not a tracing protocol.** No OpenTelemetry, no spans. Scheduler timings cover what's actually needed.
- **Not a visualiser.** The matplotlib script and any future GUI live downstream of `dump_state()`. The runtime ships the data; consumers ship the view.

Keeping these out of the core preserves the dependency footprint promised in §11 and the embedded-target focus in §13.

---

## 15. Design Constraints & Non-Goals

**Platform tier (v0.1–v1.0):** MithAtomas targets **SoC-class embedded platforms** running Linux — Raspberry Pi 3/4/5, Jetson Nano/Orin, BeagleBone, and similar boards with MB-to-GB-class heap budgets and `glibc`/`musl` standard libraries. This is the realistic deployment tier for swarm robotics: each robot is typically a multi-SoC system anyway (compute + sensor controllers + radio), and the swarm runtime lives on the application-class SoC. **Microcontroller-class targets (STM32, ESP32, sub-512-KB heap) are explicitly out of scope through v1.0.** ETL integration (§16 post-v1.0) is the starting point if/when they become a requirement; the `TraceSink` interface (§14.4) is already shaped to allow a binary sink without API changes.

**Constraints:**
- No dynamic memory allocation in hot path systems after init. All component stores pre-allocate. Observability paths (`dump_state()`, `JsonTraceSink::emit`) are *not* hot path and may allocate.
- No exceptions in the runtime core. Error returns via `std::optional` or result types.
- No RTTI in the component system. Type IDs are compile-time hashed strings.
- `ActionProvider::evaluate()` must be non-blocking. The scheduler will not protect against a blocking provider.

**Non-goals for v0.1:**
- Python bindings (planned post-v1.0)
- ROS 2 transport (planned, community contribution welcome)
- Hardware abstraction layer (HAL) — MithAtomas is above the HAL. Use your platform's HAL to feed sensor data into components.
- Centralised mission planner — MithAtomas is a swarm runtime, not a ground control station.
- Channel encryption — confidentiality on the wire is the transport implementation's responsibility. Identity authentication is *not* deferred and is addressed in §3.3.

---

## 16. Roadmap

The schedule below is dependency-ordered: each tier resolves blockers for the next. Pre-v0.1 is a doc-only phase — open architectural questions must be answered in §3–§9 before v0.1 code can be coherently written.

### Pre-v0.1 — Design resolution (doc-only, blocks implementation)

- [x] **ECS identity model** — **resolved**: one entity per `World`, representing the robot itself. Neighbours stay in `NeighbourTable` (§7.4), not as entities. Parallelism is system-level on the one self entity. See §3.2 / §4.1 / §8. Multi-entity is a v0.5 extension on demand behind a `WorldConfig::multi_entity` flag, not an API break.
- [x] **Scheduler hazard model** — **resolved**: two-axis hazard graph (components + typed resources). `NeighbourTable`, `TransportTx`, `TransportRx` are first-class `ResourceID`s; user resources register at init. See §5.1 / §5.2 / §5.3.
- [x] **`ActionExecutorSystem` write set** — **resolved**: split into the **Action Handler Registry** — `ActionValidatorSystem` plus N typed handler systems, each declaring bounded writes visible to the DAG. No tick-wide barrier. See §6.4 / §5.3.
- [x] **Hot-component registration** — **resolved**: unified runtime registration. CRTP base (`HotComponent<T>` / `ColdComponent<T>`) is a storage hint; built-in and user components share one API. Origin tracked via `ComponentOrigin`; built-ins use a privileged internal path user code cannot reach. `WorldConfig::registration_policy` (Open / LockAfterInit / BuiltInOnly) gates registration — `BuiltInOnly` is the EW lockdown posture (§13.5). Every registration emits a `component_registered` audit event (§14). See §4.1 / §4.3 / §4.4 / §8 / §13.5 / §14.4.
- [x] **Determinism scope** — **resolved**: two scheduler modes (`Parallel` / `Sequential`) selectable per `World` via `WorldConfig::scheduler_mode`. `Parallel` is the production default and makes no determinism promise; `Sequential` is the sim default (`SimBus` sets it) and is fully deterministic given fixed inputs. Schedule capture/replay for parallel-mode determinism is on the post-v1.0 roadmap. See §5.2 / §8 / §9.1 / §9.2.
- [x] **Bounded-queue overflow contract** — **resolved**: `BoundedQueue<T, N, OverflowPolicy>` template with three compile-time policies (`DropOldest`, `DropNewest`, `FaultTrigger`). Built-ins: `ActionQueueComponent` → `DropNewest` (protect validated intent), `CommBufferComponent` → `DropOldest` (network bursts, stale evicted). `FaultTrigger` integrates with `FaultMonitorSystem` via §14.3 counter deltas — no separate signaling channel. See §4.5 / §4.4 / §7.5 / §13.1 / §14.3.
- [x] **Action permission mask** — **resolved**: `PermissionMaskComponent` (uint32 builtin bitmask + bool user-action flag) on the self entity. Written only by `FaultMonitorSystem`; read by `ActionValidatorSystem`. Hazard graph orders Fault → Validator naturally. Degraded mode (§13.2) restricts to `IDLE|REGROUP`, no user actions, with hysteresis on recovery. Rejections increment a counter on `ActionQueueComponent` and emit `WARN` traces; rejections do NOT decrement health (avoids feedback loop). See §4.4 / §5.3 / §6.4 / §13.1 / §13.2 / §14.1.
- [x] **v0.1 scope coherence** — **resolved**: `BeaconSystem` + `NeighbourTable` stay in v0.1 (without them §1's "swarm" claim is unfulfilled and `SimTransport` has nothing to do). Flocking demo right-sized from 50 → 10 sim robots. Hardware transport, identity authentication, and active fault response defer to v0.2; the *architecture* of those mechanisms ships in v0.1 with dormant hooks. See v0.1 / Dormant in v0.1 / v0.2 below.
- [x] **Visualiser dependency** — **resolved**: core hand-rolls JSON emission (~50 LOC writer) for `dump_state()` and the default `JsonTraceSink` — zero external runtime deps. `nlohmann/json` gated behind `MITH_BUILD_SIM` for sim + tests only (snapshot parsing, run-diff). Trace output is pluggable via a `TraceSink` interface so embedded users can drop in binary / logfmt / MAVLink sinks without API changes. §15 adds the explicit platform tier — SoC-class (Pi/Jetson/BeagleBone), not microcontrollers, through v1.0. See §9.3 / §11 / §14.1 / §14.4 / §15.

### v0.1 — Core Runtime (current target)

Architecture per Pre-v0.1 #1–#7. Inter-robot interaction lands through `BeaconSystem` + `SimTransport`; the *active* fault response (degraded-mode mask transitions, network fault detection, identity rotation) defers to v0.2 with dormant hooks already in place — see "Dormant in v0.1" below.

- [x] `mith::UUID` + `mith::HierarchicalID` (unsigned identity, §3.1)
- [ ] `IdentityKey` data type + `IdentityVerifier` interface with no-op default (§3.3) — Ed25519 impl deferred to v0.2
- [ ] `IdentityRotationPolicy` enum + `World::rotate_identity()` API stub (§3.4) — policy enforcement deferred to v0.2
- [ ] `BoundedQueue<T, N, Policy>` template + drop / overflow counters (§4.5)
- [ ] `EntityRegistry` + unified `register_component<T>()` machinery; `ComponentOrigin` + `ComponentRegistrationPolicy` enforcement (§4.1, §4.3, §13.5)
- [ ] Built-in hot components per §4.4 — Identity, Position, Velocity, Orientation, Health, Role, BehaviourState, ActionQueue, CommBuffer, **PermissionMask**
- [ ] `SystemDescriptor` (two-axis: components + resources, §5.1) + `SystemScheduler` — both `Sequential` and `Parallel` modes (§5.2)
- [ ] `ActionProvider` + `ActionValidatorSystem` + built-in action handlers (Move, Transmit, Idle / Hover) per §6.4
- [ ] `BeaconSystem` + `NeighbourTable` + `StateVector` (§7.2, §7.4)
- [ ] `SimTransport` + `SimBus` + `SimClock` (§9); `SimBus::make_world_config()` defaults to `SchedulerMode::Sequential`
- [ ] `FlockingSystem` (Reynolds rules: separation, alignment, cohesion)
- [ ] **Flocking demo — 10 simulated robots, matplotlib visualiser** (§9.3)
- [ ] Observability primitives — `World::dump_state()`, `SystemScheduler::last_tick_timings()`, queue counters, `trace_level` with `component_registered` / `action_rejected` / `queue_overflow` events (§14)
- [ ] Unit + integration tests (registry, scheduler, neighbour table, bounded queues, permission mask, flocking convergence — leveraging observability primitives for assertions)
- [ ] CMake install target

### Dormant in v0.1 (activated in v0.2)

Several Pre-v0.1 resolutions ship their architecture in v0.1 but their *active code path* in v0.2. The hooks are in place; nothing flips them yet. This is intentional — v0.1 ships the full architectural surface so the fault response in v0.2 plugs in without API changes.

| Mechanism | v0.1 state | v0.2 activation |
|---|---|---|
| `HealthComponent` decrement on queue overflow / fault | Counters observable in `dump_state()` (§14.1, §14.3) | `FaultMonitorSystem` reads deltas, decrements health per event |
| Degraded-mode mask transition (§13.2) | `PermissionMaskComponent` honoured; defaults fully permissive | `FaultMonitorSystem` snapshots + writes restricted mask on health drop, with hysteresis |
| Identity rotation (§3.4) | Enum + `World::rotate_identity()` stub; `Permanent` is the only working policy | `PER_MISSION` / `PERIODIC` / `EVENT_DRIVEN` impls + `IdentityCertificate` chain in signed mode |
| Cryptographic identity (§3.3) | `IdentityKey` / `IdentityVerifier` types exist; default verifier is a no-op | Vendored Ed25519 + ChaCha20 CSPRNG behind `MITH_ENABLE_AUTH` |
| Action-rejection trace events (§6.4) | `WARN` event emitted whenever the mask rejects | Same path, now exercised by degraded-mode transitions |

### v0.2 — Distributed Bootstrap, Channel Separation & Cryptographic Identity

Must precede any honest multi-robot claim — the current init path in §3.1 has a hidden coordinator, and unsigned identity is spoofable on any shared channel. **This tier also activates the v0.1 dormant mechanisms** ("Dormant in v0.1" above) — `FaultMonitorSystem` lands here and turns the architecturally-defined fault response into real behaviour.

- [ ] **Discovery / bootstrap protocol** — how robots acquire `SwarmID` and find peers without a mission controller. Resolves the §0 ↔ §3.1 contradiction.
- [ ] **Channel-aware transport** — split `TransportLayer` into beacon + message transports, or define a multiplexing contract. Lets beacons ride lossy broadcast media while messages use reliable links.
- [ ] **Cryptographic identity (signed mode)** (§3.3) — vendored Ed25519 (libsodium or hand-rolled), vendored ChaCha20 CSPRNG (replaces `std::random_device` dependency for entropy), `IdentityVerifier` Ed25519 impl, signing hook on `BeaconSystem` outbound, verification on inbound. Opt-in via `MITH_ENABLE_AUTH`.
- [ ] **Identity rotation** (§3.4) — implement `PER_MISSION`, `PERIODIC`, `EVENT_DRIVEN` policies. `IdentityCertificate` chain in signed mode; `NeighbourTable` correlates old→new across rotation.
- [ ] **`FaultMonitorSystem` + degraded mode** — activates §13.1 health-decrement-on-overflow, §13.2 mask transitions with hysteresis, missed-beacon detection
- [ ] UDPMulticastTransport
- [ ] TaskAllocSystem (threshold-based, pre-partition-merge)
- [ ] Integration test: fault injection in sim (exercises the now-active dormant pathways)
- [ ] Integration test: spoofing attempt rejected in signed mode

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

### v0.5 — Multi-Entity Opt-In (on demand)

Single placeholder tier — expanded only if a concrete research use case materialises during v0.2–v0.4. Pre-v0.1 #1 committed to N=1 with the API shaped for N>1 (§3.2, §4.3); this tier is the lift.

- [ ] **Multi-entity opt-in** behind `WorldConfig::multi_entity = true` — only if a concrete research use case (sub-agents sharing transport, hierarchical-policy entities, runtime-managed predictive selves) materialises during v0.2–v0.4. Otherwise the N=1 constraint stays through v1.0. Lift includes: per-entity scheduler hazard granularity, multi-entity `ActionProvider` call shape, per-entity `IdentityComponent` semantics, and an integration test.

### v1.0 — Stable API
- [ ] API stability guarantee (covers a runtime whose architectural questions are all answered)
- [ ] Full documentation (Doxygen)
- [ ] CI (GitHub Actions): build + test on Linux/macOS/ARM
- [ ] Performance benchmarks (1000-entity sim, leveraging v0.3 spatial index)

### Post v1.0
- [ ] Python bindings (pybind11)
- [ ] ROS 2 transport plugin
- [ ] WebSocket transport (for browser-based visualiser)
- [ ] ETL integration for heap-free embedded targets — paired with the microcontroller-tier expansion (§15)
- [ ] **Binary `TraceSink` implementation** — for microcontroller-tier targets where `JsonTraceSink`'s `std::string` allocations are too expensive. Pre-allocated ring buffer, fixed-size frames, no per-event alloc. Interface already shipped in v0.1 (§14.4); only the impl lands here.
- [ ] **Schedule capture/replay** — capture the parallel-mode schedule (system order + thread assignments) into an artifact; replay scheduler honours it for deterministic re-runs of production traces. Resolves the Pre-v0.1 #5 deferral if a real need surfaces (§9.2).

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
