# Pack mesh for the Airborne Mission Computer Model 8270

[![CI](https://github.com/roiblum1/drone-mesh/actions/workflows/ci.yml/badge.svg)](https://github.com/roiblum1/drone-mesh/actions/workflows/ci.yml)

A peer-to-peer airborne mesh that lets a pack of 150 kg combustion UAVs share sensor readings, build one picture of the world, and decide on their own which drones should verify a detection, while the GMCS keeps full control of the one drone it is flying.

The GMCS can route the whole pack but can actively control only one drone in mission mode. The other drones fly the route with their sensors and processors idle. That is the single-drone GMCS bottleneck. This repo removes it by exploiting drones while they're under pack routing: they scan, triangulate, share a common learning of the situation and decide among themselves which of them should verify a detection.

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
make test     # 40 unit tests
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
| `drone_state.hpp` | `DroneState`: position, remaining endurance (of the 7-hour combustion EMU's nominal), active device on the Variable Homing Devices bus, mode `PACK_ROUTING` / `ACTIVE_MISSION` (plus `VERIFYING`, `RTB`) |
| `detected_event.hpp` | `SensorObservation`, `DetectedEvent`, `VerificationTask` |
| `mesh_packet.hpp/.cpp` | `MeshPacket`, the plaintext body codec (little-endian, strict validation on decode) and the v2 frame header |
| `mesh_security.hpp/.cpp` | `MeshSecurity`: seals and opens frames; key epochs, per-drone keys, replay windows, freshness, revocation |
| `crypto.hpp/.cpp` | ChaCha20-Poly1305 (RFC 8439), constant-time tag check, key zeroization |
| `geo.hpp/.cpp` | Distance and bearing, local east/north frame, bearing-only triangulation |
| `situational_awareness.hpp/.cpp` | `SituationalAwarenessTable`: peers, fused events, verifier slots, and the common learning of the situation (benign signatures) |
| `coordination.hpp/.cpp` | Mission task prioritisation: urgency, suitability, verifier selection, alert levels and alert-owner ranking (pure functions) |
| `pack_mesh_manager.hpp/.cpp` | `PackMeshManager`: discovery, state sync, `evaluateAndCoordinateAlerts()` |
| `hal.hpp` | Interfaces to the Mesh Network Transceiver, clock, C2 link and FCS, so the logic runs on any RTOS, a host, or the simulator |

### Pack independence: how a pack decides with no leader

1. Each drone broadcasts a heartbeat (1 Hz) with its position, fuel, active sensor and mode, and each sensor reading as it happens.
2. Every drone folds the same readings into the same table. Readings are keyed by (observer, sensor) and kept in sorted order, so tables converge. Own readings are re-sent every 2 s, so a lost packet only delays convergence.
3. `evaluateAndCoordinateAlerts()` runs every 0.5 s on every drone: fuse bearings into positions, merge duplicate events, rank by urgency (mission task prioritisation), and compute the verifier plan. The plan is a deterministic function of the table, so drones with the same table agree without negotiating.
4. A drone that finds itself in the plan sends a request to the FCS (standoff point, approach bearing, sensor to cue) and claims its slot on the mesh. Claims are arbitrated by score (quantised, so every drone ends with the same holder whatever the arrival order), ties by lowest id. A drone that loses arbitration is not ordered off its task; it gives way at the next plan, because the winner now holds the incumbent bonus.
5. Alerts to the GMCS: ADVISORY once an event is localised well enough, PRIORITY once two sensor types agree. One drone is the owner, with ranked backups; the GMCS also de-duplicates by event id.

### Rules the flight library follows

- No exceptions, no RTTI (`-fno-exceptions -fno-rtti`).
- No heap allocation after construction: fixed-capacity containers only. A test replaces global `operator new` and proves `tick()` allocates nothing.
- Every loop is bounded by a compile-time capacity.
- Explicit little-endian serialisation; no struct copies onto the wire.
- `make audit` fails the build if the library uses `new`, `malloc`, `throw`, `dynamic_cast`, `std::vector`, `std::string` and similar.

One `PackMeshManager` is about 63 KB of fixed storage.

### Mesh security

Every frame is sealed with ChaCha20-Poly1305. Nothing is sent or accepted until a key epoch is installed.

```
 0      2    3     4     6       8         12      16     18             18+len   +16
 |magic |ver |flags|src  |epoch  |session  |seq    |len   |ciphertext    |tag     |
 |0x8270|2   |0    |u16  |u16    |u32      |u32    |u16   |type,time,body|16 B    |
  \_____________ clear, authenticated as AAD ____________/
```

- **Key epochs.** The GMCS issues one 256-bit key per epoch, in the mission data load or as a re-key over a C2 link (`installEpochKey`). An epoch has a send window and is still accepted 10 s either side of it, so frames in flight across a re-key survive. Up to four epochs are held at once; `zeroizeKeys()` erases them all.
- **Per-drone keys.** A drone seals with its own key, derived from the epoch key and its id. The id and epoch are in the authenticated header, so a frame verifies only under the key of the drone it names; relabelling a frame as another drone's breaks it.
- **Nonces.** The nonce is (boot session, sequence number, sender, epoch). The boot session is a non-volatile counter that must grow on every boot, so a reboot never reuses a nonce.
- **Replay.** Per sender, a 64-frame sliding window over (session, seq) accepts each frame once, even out of order, and frames from an older boot session are refused. An authentic frame sent more than 5 s ago, or stamped more than 2 s in the future, is refused as stale, which bounds replay to a drone that has no window state yet.
- **Revocation.** `revokeDrone(id)` refuses the drone's frames at once, drops it from the pack and forgets its readings. Issuing the next epoch key to every drone but that one completes the exclusion.
- **Authentication first.** The tag is checked before anything is decrypted or parsed. Authentic frames still pass every content check (ranges, NaN, sender is the named observer or claimant), because a key holder can seal anything.

Each reason a frame is refused is counted separately in `MeshStats` (`rx_auth_failed`, `rx_no_key`, `rx_replayed`, `rx_stale`, `rx_revoked`, `rx_rejected`).

## What was verified

- `make test`: 40 tests, 635 checks pass in the release build and under ASan/UBSan, including regression tests for every confirmed finding of an independent code review (see below).
- clang-tidy (bugprone, CERT, clang-analyzer, performance and selected Core Guidelines checks) and cppcheck report nothing on the flight library.
- `make sim`: all 11 scenario checks pass; output is byte-identical across runs and across the release and sanitizer builds.
- A 20,000-trial Monte Carlo of the triangulation shows the reported CEP is calibrated for well-conditioned geometry (about 50 % of fixes inside it) with heavy tails for poor geometry. Details in deliverable 2.

## What is not done

These are the open items you should know about before relying on any of this:

- **The cipher is not a validated module.** ChaCha20-Poly1305 is implemented from RFC 8439 and checked against the RFC's vectors and against OpenSSL on 858 random cases, but it has not been through a crypto-module validation or a side-channel review on the target. For anything that flies, put a validated module behind `crypto.hpp`; nothing else depends on how it is implemented.
- **Insiders can forge within an epoch.** Keys are symmetric, so every pack member holds the epoch key. A drone captured with its key store intact can forge frames in any member's name until the next epoch. Revocation drops its own identity at once; re-keying without it ends the rest. Only per-sender signatures would stop an insider sooner.
- **The boot session must persist.** It has to come from a non-volatile counter that increases on every boot. If it repeats, nonces repeat and the mesh loses secrecy and integrity. Key fill, key storage and the C2 re-key message are outside this repo.
- **No flight validation.** All performance numbers come from a software simulation with idealised sensors and radio.
- **The Model 8270's real interfaces are assumed.** Everything touches hardware through `hal.hpp`; the bus choices in deliverable 3 are recommendations, not an ICD.
- **The FCS interface has no reply.** A real one needs accept / refuse.
- **Time assumes a shared GNSS-disciplined clock.** GNSS-denied operation is not handled.
- **The corridor is a circle.** A real approved corridor is a polygon.
- **Mesh partitions** are handled by design (each side keeps working, the GMCS de-duplicates) but not tested.
- **A pack member can still lie in its own name.** Authentication proves which drone sent a frame, not that its content is true. A drone that holds a valid key can report false readings or send a `SIGNATURE_UPDATE` that dismisses a frequency pack-wide. Content checks limit the damage (ranges, a drone speaks only for itself); revocation removes the drone.
- **Association depends on arrival order in one corner case.** Three sources closer together than the association gate (500 m plus 1.5 x CEP) can be grouped differently depending on which reading arrives first, and the grouping is not revisited. A deterministic re-clustering in `refresh()` would fix it.
- **Bearing-only readings with no signature are never triangulated.** Readings are linked by frequency until a position exists. Every RF direction-finder reports a frequency, so this only matters for a sensor that does not.
- **Plans assume nearly synchronous state.** Each drone plans from its own fresh state, its peers' last heartbeats (up to 1 s old) and its own clock. Claims and the incumbent bonus absorb the small differences; large clock disagreement is not handled.

## Code review

An independent review of the C++ found these defects, all now fixed and covered by tests:

| Defect | Fix |
|---|---|
| Two sources on one frequency (e.g. two 406 MHz beacons) 80 km apart fused into one ghost event between them | A shared signature is necessary but no longer sufficient: localised events must also pass a distance or bearing gate |
| A rebooted drone could reuse an event id, folding an unrelated detection into a stale event | Ids already in use are skipped, the counter is seeded from boot time, and an id hint must pass the same gate as any other reading |
| Fused positions broke across the 180 degree meridian | Longitudes are unwrapped before averaging and in the local frame |
| Claim arbitration could leave different holders depending on arrival order | Scores are compared quantised, giving a total order |
| A clock step backwards stopped all transmission, so peers dropped the drone | A backwards step counts as "period elapsed" |
| Anti-entropy re-sent the same 16 readings every cycle and starved the rest | The start point rotates |
| A packet could name another drone as observer, claimant or issuer; future timestamps never aged out; claim scores were unbounded | Rejected on receipt |
| NaN or out-of-range local inputs were accepted (a NaN speed made every peer reject the drone) | Validated; counted in `local_rejected` |
| The pack snapshot was one drone short of the peer table | Peer table sized to the pack minus self, with a `static_assert` |

Also tightened: `[[nodiscard]]` on results that must not be ignored, `static_assert`s on capacities, bounds asserts in `StaticVector`, non-copyable manager and table, non-sliceable interfaces, and `isValid(MeshConfig)`.

## Scope

The system detects, localises, verifies and alerts. It contains no weapons or engagement logic and does not command flight controls: it sends requests to the FCS, which enforces the approved corridor and can refuse or revert at any time.
