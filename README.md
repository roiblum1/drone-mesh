# Pack mesh for the Airborne Mission Computer Model 8270

[![CI](https://github.com/roiblum1/drone-mesh/actions/workflows/ci.yml/badge.svg)](https://github.com/roiblum1/drone-mesh/actions/workflows/ci.yml)

A peer-to-peer airborne mesh that lets a pack of 150 kg combustion UAVs share sensor readings, build one picture of the world, and decide on their own which drones should verify a detection, while the GMCS keeps full control of the one drone it is flying.

The GMCS can route the whole pack but can actively control only one drone in mission mode. The other drones fly the route with their sensors and processors idle. This repo makes them useful.

| Deliverable | Where |
|---|---|
| 1. Marketing storyboard and text-to-image prompts | [`docs/01_marketing_storyboard.md`](docs/01_marketing_storyboard.md) |
| 2. Baseline versus mesh-enabled operations | [`docs/02_pack_capability_overview.md`](docs/02_pack_capability_overview.md) |
| 3. Hardware block diagram (Mermaid, ASCII, interface table) | [`docs/03_hardware_block_diagram.md`](docs/03_hardware_block_diagram.md) |
| 4. Embedded C++17 for the 8270 mesh | [`include/amc8270/mesh/`](include/amc8270/mesh), [`src/`](src), [`tests/`](tests), [`sim/`](sim) |
| One-page overview for sharing | [`docs/pack-mesh-overview.html`](docs/pack-mesh-overview.html), generated from the files above by `python3 -I tools/build_overview_page.py docs/pack-mesh-overview.html` |

## Build and run

Needs a C++17 compiler and `make`. No other dependencies. CI builds with GCC and Clang on Ubuntu and Apple Clang on macOS.

```
make test     # 29 unit tests
make sim      # 6-drone scenario with a printed timeline and pass/fail checks
make check    # tests and sim under AddressSanitizer + UndefinedBehaviorSanitizer
make audit    # grep the flight library for banned constructs (new, throw, std::vector, ...)
```

CMake works too: `cmake -S . -B build-cmake && cmake --build build-cmake && ctest --test-dir build-cmake`.

## Continuous integration

`.github/workflows/ci.yml` runs on every push and pull request:

| Job | What it checks |
|---|---|
| build + test | `make test` and `make sim` with GCC and Clang (Ubuntu) and Apple Clang (macOS), plus a check that two simulation runs are byte-identical, plus `make audit` |
| ASan + UBSan | Tests and simulation under AddressSanitizer and UndefinedBehaviorSanitizer |
| CMake + ctest | The CMake build and its test registration |
| clang-tidy + cppcheck | Static analysis of the flight library; any finding fails the build (rules in `.clang-tidy`) |

## What is in the code

| File | Role |
|---|---|
| `types.hpp`, `config.hpp` | Ids, enums, capacities (`kMaxPackSize = 16`, `kMaxEvents = 64`, ...) and the tunable `MeshConfig` |
| `drone_state.hpp` | `DroneState`: position, remaining endurance (of 7 h), active homing sensor, mode `PACK_ROUTING` / `ACTIVE_MISSION` (plus `VERIFYING`, `RTB`) |
| `detected_event.hpp` | `SensorObservation`, `DetectedEvent`, `VerificationTask` |
| `mesh_packet.hpp/.cpp` | `MeshPacket` and its wire format: little-endian, CRC-32, strict validation on decode |
| `geo.hpp/.cpp` | Distance and bearing, local east/north frame, bearing-only triangulation |
| `situational_awareness.hpp/.cpp` | `SituationalAwarenessTable`: peers, fused events, verifier slots, learned-benign signatures |
| `coordination.hpp/.cpp` | Urgency, suitability, verifier selection, alert levels and alert-owner ranking (pure functions) |
| `pack_mesh_manager.hpp/.cpp` | `PackMeshManager`: discovery, state sync, `evaluateAndCoordinateAlerts()` |
| `hal.hpp` | Interfaces to the radio, clock, C2 link and FCS, so the logic runs on any RTOS, a host, or the simulator |

### How a pack decides, with no leader

1. Each drone broadcasts a heartbeat (1 Hz) with its position, fuel, active sensor and mode, and each sensor reading as it happens.
2. Every drone folds the same readings into the same table. Readings are keyed by (observer, sensor) and kept in sorted order, so tables converge. Own readings are re-sent every 2 s, so a lost packet only delays convergence.
3. `evaluateAndCoordinateAlerts()` runs every 0.5 s on every drone: fuse bearings into positions, merge duplicate events, rank by urgency, and compute the verifier plan. The plan is a deterministic function of the table, so drones with the same table agree without negotiating.
4. A drone that finds itself in the plan sends a request to the FCS (standoff point, approach bearing, sensor to cue) and claims its slot on the mesh. Claims are arbitrated by score, ties by lowest id.
5. Alerts to the GMCS: ADVISORY once an event is localised well enough, PRIORITY once two sensor types agree. One drone is the owner, with ranked backups; the GMCS also de-duplicates by event id.

### Rules the flight library follows

- No exceptions, no RTTI (`-fno-exceptions -fno-rtti`).
- No heap allocation after construction: fixed-capacity containers only. A test replaces global `operator new` and proves `tick()` allocates nothing.
- Every loop is bounded by a compile-time capacity.
- Explicit little-endian serialisation; no struct copies onto the wire.
- `make audit` fails the build if the library uses `new`, `malloc`, `throw`, `dynamic_cast`, `std::vector`, `std::string` and similar.

One `PackMeshManager` is about 60 KB of fixed storage.

## What was verified

- `make test`: 29 tests, 409 checks pass in the release build and under ASan/UBSan.
- clang-tidy (bugprone, CERT, clang-analyzer, performance and selected Core Guidelines checks) and cppcheck report nothing on the flight library.
- `make sim`: all 11 scenario checks pass; output is byte-identical across runs and across the release and sanitizer builds.
- A 20,000-trial Monte Carlo of the triangulation shows the reported CEP is calibrated for well-conditioned geometry (about 50 % of fixes inside it) with heavy tails for poor geometry. Details in deliverable 2.

## What is not done

These are the open items you should know about before relying on any of this:

- **No authentication or encryption.** Packets carry a CRC, which catches accidents, not an attacker. A real mesh needs authenticated encryption, key management and replay protection.
- **No flight validation.** All performance numbers come from a software simulation with idealised sensors and radio.
- **The Model 8270's real interfaces are assumed.** Everything touches hardware through `hal.hpp`; the bus choices in deliverable 3 are recommendations, not an ICD.
- **The FCS interface has no reply.** A real one needs accept / refuse.
- **Time assumes a shared GNSS-disciplined clock.** GNSS-denied operation is not handled.
- **The corridor is a circle.** A real approved corridor is a polygon.
- **Mesh partitions** are handled by design (each side keeps working, the GMCS de-duplicates) but not tested.

## Scope

The system detects, localises, verifies and alerts. It contains no weapons or engagement logic and does not command flight controls: it sends requests to the FCS, which enforces the approved corridor and can refuse or revert at any time.
