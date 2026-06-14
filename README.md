# MithAtomas

[![CI](https://github.com/Vighnesh-Patidar/mith-atomas/actions/workflows/ci.yml/badge.svg)](https://github.com/Vighnesh-Patidar/mith-atomas/actions/workflows/ci.yml)

Swarm robotics orchestration runtime — C++17.

> **Decentralised by default. Composable by design. Hardware-agnostic by necessity.**

See [ARCHITECTURE.md](ARCHITECTURE.md) for the authoritative design.

<table>
<tr>
<td align="center" width="50%">

<img src="docs/assets/flocking_demo.gif" alt="2D flocking — 10 robots, 200 ticks @ 20 Hz" width="100%" />

**2D — 10 robots, 5×2 grid → tight cluster.**
The mean radius around the centroid contracts ~70% (6.87 m → 2.10 m) across 10 s of sim time. Reynolds rules pulling the flock together via cohesion + alignment; separation keeps them from overlapping.

</td>
<td align="center" width="50%">

<img src="docs/assets/flocking_demo_3d.gif" alt="3D flocking — 20 robots, 400 ticks @ 20 Hz" width="100%" />

**3D — 20 robots, scattered cube → spherical cluster.**
Same `BeaconSystem` + `FlockingSystem` + `KinematicsSystem`, full 3-axis motion. 20 robots seeded inside a 16 m cube collapse into a coherent sphere of motion in 20 s. Reproduce with `flocking_demo_3d | visualise3d.py`.

</td>
</tr>
</table>

Frames captured live; rendered by `tools/visualiser/visualise.py` (2D) and `tools/visualiser/visualise3d.py` (3D). Reproduce with [the demo commands](#run-the-flocking-demo). Full 2D snapshot grid at `docs/assets/flocking_demo.png`.

---

## Status — v1.0.0-rc1

First release candidate. Every software item on the [§16 ARCHITECTURE roadmap](ARCHITECTURE.md#16-roadmap) up to v1.0 is in. The two remaining roadmap entries (Raspberry Pi 4 build verification + `SerialTransport` validated on a physical robot pair) are field-hardware items that need actual hardware access — the ARM build target in CI already proves the code compiles + tests on aarch64 Linux. RC1 freezes the API surface under the [SemVer policy](docs/SEMVER.md); the hardware items roll into the GA tag once validated.

### What ships

| Area | What ships |
|---|---|
| Identity | `UUID` (RFC 4122 v4), `HierarchicalID`, Ed25519 sign/verify via vendored TweetNaCl, ChaCha20 CSPRNG (RFC 8439, per-thread), identity rotation (`PERIODIC` + `EVENT_DRIVEN` + `IdentityCertificate` continuity chain signed by previous key) |
| ECS | `EntityRegistry` (registration policies, view, snapshot, `ComponentOrigin` tag, sink-wired audit), all 10 §4.4 built-in components, `BoundedQueue<T,N,Policy>` |
| Scheduling | `SystemDescriptor` two-axis hazards, `SystemScheduler` with both `Sequential` and `Parallel` modes (thread pool + hazard DAG), `last_tick_timings()` |
| Comms | `StateVector` (now carrying `sync_time_s` + `sender_pubkey` + `signature`), `Message`, `NeighbourTable`, channel-aware transport (`BeaconTransport` + `MessageTransport` interfaces, unified `TransportLayer` for combined impls), `BROADCAST_ID` semantics |
| Transports | `SimTransport` (in-process), `UDPMulticastTransport` (POSIX sockets), **`SerialTransport`** (POSIX termios + sync-byte/length framing via `serial_framing`) |
| Motion | `FlockingSystem` (Reynolds), `KinematicsSystem`, 20-robot 3D demo |
| Fault handling | `FaultMonitorSystem` (§13.1, §13.2) — fault counter delta → health decrement → degraded mask install with snapshot/restore + hysteresis |
| Task allocation | `TaskAllocSystem` (§5.3) — threshold-based, deterministic, with merge-window override during partition heals |
| Discovery | **`DiscoverySystem`** — passive (NeighbourTable quorum / timeout) + active (HELLO/WELCOME over message channel) + signed-mode pubkey carriage in HELLO/WELCOME payload |
| Security | **Signed beacons end-to-end** — BeaconSystem signs each emit with the World's keypair; receivers verify; TOFU pin per HID via `PeerKeyRegistry` (writable by DiscoverySystem ahead of first beacon) |
| Clock sync | **`ClockSyncSystem`** — bounded-drift GTSP variant, sender stamps `sync_time_s` on every beacon, receivers nudge `clock_offset_s` toward the swarm mean each tick |
| Partition handling | **`PartitionMergeSystem`** — detects neighbour-count jumps (heal events); opens a merge window during which `TaskAllocSystem` collapses stability hysteresis for fast reconvergence |
| Sim | `SimClock`, `SimBus` (range-limited delivery), `SimTransport` |
| Observability | Hand-rolled JSON writer, `TraceSink` interface + `JsonTraceSink` + `NullTraceSink`, **`BinaryTraceSink`** (allocation-free fixed-size ring buffer, microcontroller-tier), `World::dump_state()`, `component_registered` / `tick_completed` audit events |
| Runtime | `World` (config, init, tick, run, identity, transports — unified or per-channel, neighbour table, scheduler / registry forwarders, sink wiring, peer-key registry, clock offset, merge window) |
| Demos | 10-robot 2D flocking demo + 20-robot 3D demo + matplotlib visualisers |
| Build | CMake STATIC library, install target with `find_package(mith-atomas)` config, doctest-vendored test suite, build matrix gated by `MITH_ENABLE_UDP` / `MITH_ENABLE_AUTH` / `MITH_ENABLE_SERIAL` |

Test suite — verified across the build matrix:

| Config | Cases | Assertions |
|---|---:|---:|
| all options off | 352 | 13,410 |
| `UDP=ON AUTH=ON SERIAL=ON` | 398 | 13,659 |

All green, including integration tests for SimBus-driven fault injection, the signed-mode rotation chain, the full-stack lifecycle, UDP multicast loopback, signed beacons end-to-end (with adversary spoofing rejection), serial transport via socketpair, the partition → heal merge path under fault injection, and the v0.3 1000-entity perf gate (run at N=500 for the rc1 baseline — 6.5 s median per tick on a 4-core i5-11260H, FlockingSystem inner loop at 0.003 ms/world thanks to the spatial index).

CI runs the build matrix on Linux × {all-off, udp-auth, all-on}, asan+ubsan under Debug, `-Werror` under Release, macOS-latest with Apple Clang, and ubuntu-24.04-arm for the aarch64 platform tier. Doxygen build verified per push; HTML artifact uploaded for 14 days.

Pre-v0.1 design phase: **9/9 questions resolved**. v1.0 software ledger: **41/43 items shipped** (the two open items are physical-hardware validation — see [`qa-report/`](qa-report/) for the full progress mapping).

### Outstanding for GA (post-rc1)

| Item | Notes |
|---|---|
| Raspberry Pi 4 build verification | CI's aarch64 job covers the compile; runtime numbers on physical Pi 4 needed |
| `SerialTransport` on a physical robot pair | Loopback via `socketpair` is in CI; physical UART pair (USB-serial adapters) still TODO |
| RL policy demo *(nice-to-have)* | Dummy policy consuming `EntitySnapshot` and emitting `Action`s |

---

## Build

```sh
cmake -B build -S .
cmake --build build -j$(nproc)
```

## Test

```sh
cd build && ctest --output-on-failure
# Or the binary directly for the doctest summary:
./build/tests/mith_unit_tests
```

Single test case:

```sh
./build/tests/mith_unit_tests -tc='Parallel mode: hazardous systems run in lex order'
```

Stress (re-run the whole suite N times — catches flaky entropy / timing):

```sh
./build/tests/mith_unit_tests --count=100
```

List every case:

```sh
./build/tests/mith_unit_tests --list-test-cases
```

## Run the flocking demo

**2D — 10 robots:**

```sh
./build/examples/flocking_demo/flocking_demo | python3 tools/visualiser/visualise.py
```

200 ticks at 20 Hz (10 s sim time). Robots start in a 5×2 grid; emit one JSON object per tick describing each robot's `(x, y)`; the Python visualiser animates the scatter live.

**3D — 20 robots:**

```sh
./build/examples/flocking_demo_3d/flocking_demo_3d | python3 tools/visualiser/visualise3d.py
```

400 ticks at 20 Hz (20 s). Scattered cube → coherent sphere. Same JSON-line protocol with `z` added to each entry. Pass `--save out.gif` to export an animated GIF instead of opening the matplotlib window.

Both visualisers require `matplotlib` (and `pillow` for `--save`); nothing else. For headless capture:

```sh
./build/examples/flocking_demo/flocking_demo    > /tmp/demo.jsonl
./build/examples/flocking_demo_3d/flocking_demo_3d > /tmp/demo3d.jsonl
```

## Run the swarm benchmark

The v1.0 §16 performance gate — 1000 worlds running BeaconSystem + FlockingSystem + KinematicsSystem under SimBus with the v0.3 spatial index (NeighbourTable + SimBus delivery both indexed):

```sh
scripts/run_swarm_benchmark.sh                     # N=1000, T=200, defaults
scripts/run_swarm_benchmark.sh 500 100             # smaller / faster run
```

Drops a timestamped folder under `qa-report/swarm-benchmark/`:

- `per-tick.csv` — `tick, wall_ms, mean_neighbours` (one row per tick).
- `summary.log` — aggregate timings, per-system breakdown for world 0.
- `summary.md`  — human-readable headline numbers.

There's also a focused `spatial_index_benchmark` for the NeighbourTable lookup speedup alone:

```sh
./build/examples/spatial_index_benchmark/spatial_index_benchmark
```

CSV to stdout: linear-scan µs vs indexed µs per query, sweeping N from 64 to 1024.

## API docs

API reference is generated by Doxygen. To build locally:

```sh
sudo apt-get install doxygen graphviz   # or brew install on macOS
scripts/build_docs.sh
# open docs/api/html/index.html
```

CI also builds the docs on every push and uploads them as a workflow artifact (`api-docs`). API stability tiers — **stable**, **experimental**, **internal** — are defined in [`docs/SEMVER.md`](docs/SEMVER.md) and marked at each declaration via `MITH_STABLE_API` / `MITH_EXPERIMENTAL_API` / `MITH_INTERNAL`.

## Install

```sh
cmake --install build --prefix /your/install/prefix
```

Installs `libmith.a`, headers under `include/mith/`, and CMake package config under `lib/cmake/mith-atomas/`. Downstream consumers:

```cmake
find_package(mith-atomas REQUIRED)
target_link_libraries(my_robot PRIVATE mith::mith)
```

## CMake options

| Option | Default | Purpose |
|---|---|---|
| `MITH_BUILD_EXAMPLES` | `ON`  | Build the flocking demo |
| `MITH_BUILD_TESTS`    | `ON`  | Build the test suite |
| `MITH_BUILD_SIM`      | `ON`  | Build the simulation harness (reserved — currently always on) |
| `MITH_ENABLE_UDP`     | `ON`  | Build `UDPMulticastTransport` (POSIX sockets, IPv4 multicast) |
| `MITH_ENABLE_SERIAL`  | `OFF` | Build serial transport (lands v0.3) |
| `MITH_ENABLE_AUTH`    | `OFF` | Build cryptographic identity — Ed25519 sign/verify, ChaCha20 CSPRNG, signed `IdentityCertificate` chain |

---

## Architecture pointer

[`ARCHITECTURE.md`](ARCHITECTURE.md) is the spec. Quick map:

- **§3** Identity — `HierarchicalID`, signed/unsigned modes, rotation policies
- **§4** ECS — registry, components, type system, bounded queues
- **§5** System model — two-axis hazards, scheduler modes (Sequential / Parallel)
- **§6** Action handler registry
- **§7** Comms — `StateVector`, `Message`, `NeighbourTable`, transport, broadcast semantics
- **§8** `World` — top-level runtime, lifecycle, config
- **§9** Simulation harness
- **§13** Fault tolerance + EW posture (§13.5)
- **§14** Observability — `TraceSink`, JSON writer, `dump_state`, `last_tick_timings`
- **§15** Constraints + platform tier (SoC-class — Pi, Jetson, BeagleBone)
- **§16** Roadmap

---

## Contributing

PRs welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md) for setup, conventions, commit style, and what we look for in review. The [Code of Conduct](CODE_OF_CONDUCT.md) governs all community spaces — issues, PRs, discussions.

Templates:

- `.github/PULL_REQUEST_TEMPLATE.md` — auto-applied on new PRs
- `.github/ISSUE_TEMPLATE/bug_report.md`
- `.github/ISSUE_TEMPLATE/feature_request.md`
- `.github/ISSUE_TEMPLATE/question.md`

For security-relevant issues (anything touching §3.3 identity, §13.5 EW posture, or transport auth), do not file a public issue — email the maintainer directly. See [CONTRIBUTING.md](CONTRIBUTING.md#reporting-security-issues).

## License

Apache 2.0 — see [LICENSE](LICENSE).
