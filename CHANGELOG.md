# MithAtomas Changelog

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Stability tiers (`stable` / `experimental` / `internal`) on every public surface are governed by [`docs/SEMVER.md`](docs/SEMVER.md).

## [1.0.0-rc1] — 2026-06-14

First release candidate. **41/43 ARCHITECTURE.md §16 roadmap items shipped.** Two outstanding items are physical-hardware validation gated on hardware access.

### Added — v1.0 tier

- `MITH_STABLE_API` / `MITH_EXPERIMENTAL_API` / `MITH_INTERNAL` markers at every public class/struct declaration. Documented in `docs/SEMVER.md`. Macros expand to nothing at compile time; consumed by Doxygen and future static-analysis tooling.
- `Doxyfile` + `scripts/build_docs.sh`. Generates HTML API reference under `docs/api/html/`. CI builds the docs on every push and uploads as an `api-docs` artifact.
- GitHub Actions CI: build matrix on Linux × {all-off, udp-auth, all-on}, asan+ubsan, `-Werror`, ubuntu-24.04-arm, and the Doxygen build.
- `examples/swarm_benchmark/` + `scripts/run_swarm_benchmark.sh` — the v1.0 1000-entity perf gate. CSV stream + per-system breakdown + RSS snapshot per run.
- `examples/spatial_index_benchmark/` — NeighbourTable lookup speedup demo (linear scan vs hash-grid query).
- `scripts/run_full_qa.sh` — comprehensive build matrix + demo + profiling harness.

### Added — v0.3 tier

- `ClockSyncSystem` — bounded-drift GTSP variant; sender stamps `sync_time_s` on every beacon, receivers nudge `clock_offset_s` toward the swarm mean each tick.
- `PartitionMergeSystem` — detects neighbour-count jumps as heal events, opens `World::merge_window_remaining_s`, `TaskAllocSystem` collapses its `stability_window_s` to 0 during the window for fast reconvergence.
- `DiscoverySystem` — passive bootstrap state machine + active `DISCOVERY_HELLO`/`DISCOVERY_WELCOME` over the message channel + signed-mode pubkey carriage in HELLO/WELCOME payloads.
- `PeerKeyRegistry` — swarm-wide TOFU pubkey pin per HID. `DiscoverySystem` pre-populates from HELLO/WELCOME; `BeaconSystem` consults + populates from signed-beacon traffic.
- Signed beacons end-to-end: `StateVector` carries `sender_pubkey` + `signature` (always present, zero in unsigned mode); `BeaconSystem` Ed25519-signs every outbound beacon, verifies every inbound, rejects TOFU mismatches.
- `UDPMulticastTransport` (POSIX sockets, IPv4 multicast) and `SerialTransport` (POSIX termios + sync-byte/length framing via `serial_framing`).
- `BinaryTraceSink` — allocation-free fixed-size ring buffer; microcontroller-tier observability path.
- Spatial index on `NeighbourTable` — hash-grid keyed by position; `for_each_within(centre, radius, cb)` brings FlockingSystem from O(N²) to O(N·k_avg). 3.65× speedup at N=1024 in the benchmark.
- SimBus delivery uses the same hash-grid pattern — broadcast/beacon delivery is O(k) per emit instead of O(N).
- Action Handler Registry: `ActionValidatorSystem` + `MoveActionHandler` + `TransmitActionHandler` + `HoverActionHandler`. Closes the v0.1 ledger gap.
- 20-robot 3D flocking demo + `tools/visualiser/visualise3d.py`.

### Added — v0.2 tier

- Ed25519 sign/verify via vendored TweetNaCl + ChaCha20 CSPRNG (RFC 8439, per-thread).
- Identity rotation with `PERIODIC` + `EVENT_DRIVEN` policies + signed `IdentityCertificate` continuity chain.
- Channel-aware transport split (`BeaconTransport` + `MessageTransport` interfaces; `TransportLayer` is the combined convenience type).
- `FaultMonitorSystem` (§13.1, §13.2) — fault counter delta → health decrement → degraded mask snapshot/restore with hysteresis.
- `TaskAllocSystem` (§5.3) — threshold-based deterministic role allocation.

### Added — v0.1 tier

- `UUID` (RFC 4122 v4), `HierarchicalID`, `IdentityKey` + `IdentityVerifier` interface with `NoopIdentityVerifier`, `IdentityRotationPolicy`.
- ECS: `EntityRegistry` (registration policies + view + snapshot + `ComponentOrigin` + sink-wired audit), `BoundedQueue<T,N,Policy>`, all 10 §4.4 built-in components.
- `SystemDescriptor` (two-axis hazards: components + resources) + `SystemScheduler` (both `Sequential` and `Parallel` modes).
- `StateVector`, `Message`, `NeighbourTable`, `BeaconSystem`, `BROADCAST_ID` semantics.
- `FlockingSystem` (Reynolds rules) + `KinematicsSystem`.
- `SimBus` + `SimClock` + `SimTransport` (range-limited delivery).
- Hand-rolled JSON writer + `TraceSink` interface + `JsonTraceSink` + `NullTraceSink`.
- `World` (config, init, tick, run, identity, transports, sink wiring).
- 10-robot 2D flocking demo + `tools/visualiser/visualise.py`.
- CMake STATIC library, install target with `find_package(mith-atomas)` config, doctest-vendored test suite.

### Test surface

- **all-off**: 373 cases, 13,552 assertions
- **udp-auth**: 395 cases, 13,631 assertions
- **all-on**: 419 cases, 13,801 assertions

All green under Debug + asan + ubsan + `-Werror`.

### Outstanding for GA

| Item | Tier |
|---|---|
| Raspberry Pi 4 build verification | hardware-gated |
| `SerialTransport` on a physical robot pair | hardware-gated |
| RL policy demo | nice-to-have |
