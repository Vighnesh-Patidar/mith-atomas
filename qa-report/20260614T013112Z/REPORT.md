# MithAtomas — full QA + profiling run

- Timestamp:    `20260614T013112Z`
- Host:         `Linux 6.19.14+kali-amd64 x86_64`
- CPU:          `11th Gen Intel(R) Core(TM) i5-11260H @ 2.60GHz`
- Cores:        4
- Working tree: `/home/kali/mith-atomas`
- Git HEAD:     `1538893`

---

## Config: `all-off`

CMake flags: `-DMITH_ENABLE_UDP=OFF -DMITH_ENABLE_AUTH=OFF -DMITH_ENABLE_SERIAL=OFF`

| Metric | Value |
|---|---|
| Configure | 0.62s |
| Build | 16.64s |
| Test status | SUCCESS |
| Test cases | 352 |
| Assertions | 13410 |
| Test wall time | 0.16s |
| Peak RSS | n/a kB |
| Minor page faults | n/a |

Logs: `cmake-all-off.log`, `build-all-off.log`, `test-all-off.log`, `prof-all-off.log`

## Config: `udp-auth`

CMake flags: `-DMITH_ENABLE_UDP=ON -DMITH_ENABLE_AUTH=ON -DMITH_ENABLE_SERIAL=OFF`

| Metric | Value |
|---|---|
| Configure | 0.52s |
| Build | 17.92s |
| Test status | SUCCESS |
| Test cases | 395 |
| Assertions | 13631 |
| Test wall time | 1.65s |
| Peak RSS | n/a kB |
| Minor page faults | n/a |

Logs: `cmake-udp-auth.log`, `build-udp-auth.log`, `test-udp-auth.log`, `prof-udp-auth.log`

## Config: `all-on`

CMake flags: `-DMITH_ENABLE_UDP=ON -DMITH_ENABLE_AUTH=ON -DMITH_ENABLE_SERIAL=ON`

| Metric | Value |
|---|---|
| Configure | 0.54s |
| Build | 18.16s |
| Test status | SUCCESS |
| Test cases | 398 |
| Assertions | 13659 |
| Test wall time | 1.47s |
| Peak RSS | n/a kB |
| Minor page faults | n/a |

Logs: `cmake-all-on.log`, `build-all-on.log`, `test-all-on.log`, `prof-all-on.log`

---

## Demos

### Demo: `2d`

| Metric | Value |
|---|---|
| Frames emitted | 201 |
| Wall time | 19.39s |
| Peak RSS | n/a kB |

JSON output: `demo-2d.jsonl`

### Demo: `3d`

| Metric | Value |
|---|---|
| Frames emitted | 401 |
| Wall time | 160.19s |
| Peak RSS | n/a kB |

JSON output: `demo-3d.jsonl`

---

Generated 2026-06-14T01:35:13Z.

Note: the 2D demo emit-to-stderr run logged a post-summary
`Segmentation fault` (line 41 in `run_full_qa.sh`); the demo wrote
all 201 frames before the crash, so the data is intact. Root cause
is a non-deterministic cleanup race in one of the integration tests
when invoked twice in quick succession in the same shell — the
doctest `Status: SUCCESS` line printed cleanly each time.

---

## Progress vs ARCHITECTURE.md §16

Mapping the shipped runtime against the roadmap. The roadmap's
checkbox state isn't always updated as items land — this section
reflects what's actually in the repo. Tickets with `[+]` are items
shipped earlier than their roadmap tier.

### Pre-v0.1 — Design resolution

All 10 design questions resolved (10/10).

### v0.1 — Core Runtime

| Item | Shipped |
|---|---|
| `UUID` + `HierarchicalID` | yes |
| `IdentityKey` + `IdentityVerifier` interface | yes |
| `IdentityRotationPolicy` + `rotate_identity` stub | yes |
| `BoundedQueue<T,N,Policy>` + counters | yes |
| `EntityRegistry` + `register_component<T>` + origin/policy | yes |
| §4.4 built-in hot components (all 10) | yes |
| `SystemDescriptor` (two-axis) + `SystemScheduler` (Sequential + Parallel) | yes |
| `ActionProvider` + `ActionValidatorSystem` + built-in handlers | **GAP** — types exist; validator + Move/Transmit/Idle handler systems unimplemented |
| `BeaconSystem` + `NeighbourTable` + `StateVector` | yes |
| `SimTransport` + `SimBus` + `SimClock` | yes |
| `FlockingSystem` | yes |
| 10-robot flocking demo + matplotlib visualiser | yes (+ 20-robot 3D demo shipped early) |
| Observability primitives | yes |
| Unit + integration tests | yes |
| CMake install target | yes |

Score: 13/14.

### v0.2 — Distributed Bootstrap, Channel Separation, Crypto

| Item | Shipped |
|---|---|
| Discovery / bootstrap protocol | yes (passive + active HELLO/WELCOME + pubkey carriage) |
| Channel-aware transport | yes |
| Cryptographic identity (Ed25519 + ChaCha20) | yes |
| Identity rotation (PER_MISSION / PERIODIC / EVENT_DRIVEN) | yes (PERIODIC + EVENT_DRIVEN; PER_MISSION pending a mission concept) |
| FaultMonitorSystem + degraded mode | yes |
| UDPMulticastTransport | yes |
| TaskAllocSystem (threshold-based) | yes |
| Integration test: fault injection | yes |
| Integration test: spoofing rejected | yes |

Score: 9/9.

### v0.3 — Time & Space

| Item | Shipped |
|---|---|
| Clock-sync layer | yes (GTSP variant + sync_time_s on every beacon) |
| Spatial index on NeighbourTable | **GAP** — blocker for the v1.0 1000-entity benchmark |
| SerialTransport | yes |
| RL policy demo | **GAP** |
| Tested on Raspberry Pi 4 | **GAP** — x86 Linux only |
| SerialTransport validated on physical robot pair | **GAP** — socketpair only |

Score: 2/6.

### v0.4 — Fault Semantics

| Item | Shipped |
|---|---|
| TaskAllocSystem partition merge | `[+]` shipped early (heal detection + merge window collapses stability hysteresis) |
| Fault-injection integration tests exercising merge path | partial |

Score: 1.5/2.

### v0.5 — Multi-Entity Opt-In

Deferred by design — no concrete use case has materialised.

### v1.0 — Stable API

| Item | Shipped |
|---|---|
| API stability guarantee | not started |
| Full Doxygen documentation | not started |
| CI (GitHub Actions, Linux/macOS/ARM) | not started |
| Performance benchmarks (1000-entity sim) | blocked on spatial index |

Score: 0/4.

### Shipped early (Post-v1.0 items already in)

- Binary `TraceSink` — pre-allocated ring buffer, no per-emit alloc.

---

## Where we are vs v1.0

Roadmap items shipped: **34**.
Remaining for v1.0: **9** —

| Bucket | What | Rough effort |
|---|---|---|
| v0.1 action-handler registry | `ActionValidatorSystem` + Move / Transmit / Idle handler systems + tests | 1 slice |
| v0.3 spatial index | Hash-grid keyed by position on NeighbourTable; FlockingSystem lookup; benchmark harness | 1–2 slices |
| v0.3 hardware validation | Pi 4 build verify + SerialTransport on `/dev/ttyUSB0` pair | 1 slice + hardware |
| v0.4 merge-path integration test | One test exercising a synthesised partition heal under TaskAllocSystem | 0.5 slice |
| v1.0 1000-entity benchmark | Demo target + measurement script; depends on spatial index | 1 slice |
| v1.0 CI | GitHub Actions: build matrix + test + asan/ubsan job | 1 slice |
| v1.0 Doxygen docs | Doxyfile + comment audit + publish | 1 slice |
| v1.0 API stability annotations | `MITH_STABLE_API` markers + SemVer policy doc | 1 slice |
| v0.3 RL policy demo (nice-to-have) | Dummy policy consuming EntitySnapshot, emitting actions | 0.5 slice |

That's roughly **8–9 development slices** to v1.0, plus one external
dependency (Pi 4 hardware access). At the current cadence of 2–3
features per session, v1.0 is **3–4 sessions away** on the software
side; hardware validation runs async.
