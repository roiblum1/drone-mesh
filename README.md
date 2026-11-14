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
| Animated replay of the simulated pack, for sharing | [`docs/pack-mesh-replay.html`](docs/pack-mesh-replay.html), built by `make replay` |
| One-page overview for sharing | [`docs/pack-mesh-overview.html`](docs/pack-mesh-overview.html), generated from the files above by `python3 -I tools/build_overview_page.py docs/pack-mesh-overview.html` |

## Build and run

Needs a C++17 compiler and `make`. No other dependencies. CI builds with GCC and Clang on Ubuntu and Apple Clang on macOS.

```
make test       # 72 unit tests
make sim        # 6-drone storyline under realistic conditions: timeline, summary, pass/fail checks
make sim16      # the 16-drone scenario
make scenarios  # every scenario file, one line each
make mc         # Monte Carlo over the storyline and the 16-drone scenario (TRIALS=200)
make check    # tests and sim under AddressSanitizer + UndefinedBehaviorSanitizer
make audit    # grep the flight library for banned constructs (new, throw, std::vector, ...)
```

CMake works too: `cmake -S . -B build-cmake && cmake --build build-cmake && ctest --test-dir build-cmake`.

### Fuzzing

```
make fuzz              # libFuzzer + ASan + UBSan, 60 s (FUZZ_SECONDS=...), needs clang
make fuzz-regress      # replay fuzz/corpus through the same harness, any compiler
make corpus            # regenerate the seed corpus
```

`fuzz/fuzz_packet.cpp` offers every input to the frame decoder as a raw datagram, which must never open, since nobody else holds its key. It then reads the input as a list of records and seals each one as a pack member would. Records are under the fuzzer's control: the sender, replays of earlier frames, reboots and clock steps. Each sealed frame goes through the security layer into a live `PackMeshManager`. The harness aborts if a decodable body does not re-encode to the same bytes, if a fresh frame is refused or a replayed one accepted, or if the manager's table is ever inconsistent: duplicate ids, unsorted readings, one drone in both slots, a fused value out of range. Random bytes almost never get past the tag, so sealing inside the harness is what lets the fuzzer reach fusion, planning and alerting.

The seed corpus (`fuzz/corpus`, written by `fuzz/make_corpus.cpp`) holds one record of each type plus scenarios: triangulate, confirm, dismiss, two beacons on one frequency, and replay, reboot and leave. Inputs the fuzzer found are kept there as regression seeds (`regress-*`).

## Simulation

`sim/` runs the real flight library on simulated drones. Scenario files in `sim/scenarios/` describe the pack, the sources, the terrain, the radio, the sensors, scripted faults and the pass/fail checks:

| File | What it is |
|---|---|
| `ideal6.scn` | The original 6-drone storyline with the original idealised models (no latency, independent 5 % loss, flat ground, no multipath) |
| `storyline6.scn` | The same storyline under realistic conditions (the default for `make sim`) |
| `pack16.scn` | All 16 drones in two rows; two distress beacons on one frequency, a fire with no RF emission, a ridge, IR clutter, a lost verifier, a mesh partition and heal, an operator dismissal |

What the models add over the original sim:

- **Radio:** fixed latency plus exponential jitter, and bursty loss from a Gilbert-Elliott chain on every link. Drones' clocks are a few ms apart, and each drone ticks at its own phase.
- **Terrain:** Gaussian hills and ridges, with line of sight and earth curvature (4/3 earth radius for radio, 1 for optics). Terrain can mask DF, imaging and mesh links.
- **DF multipath:** a drone that enters a reflection sees the source 10 to 30° off for 5 to 30 s, and reflections are four times as likely at low elevation. The DF does not know it is reflected.
- **Clutter:** false imaging detections as a Poisson process per drone.
- **Ground:** C2 latency per link type at the GMCS.

`pack_sim <file> --mc N` runs N trials, reseeding every random process and moving each source by up to the scenario's jitter. It reports time to first alert, time to PRIORITY and PRIORITY error, each as p5, p50, p95 and max. It also reports false alerts, split by cause (clutter, or a real source mislocalised by multipath, from the sim's ground-truth label on every reading), duplicates, FCS requests, mesh load and the pass rate of every check. `--mc-json` writes the distributions. It names the seeds of any trial with a false PRIORITY; `pack_sim <file> --seed N --jitter` replays that trial with its timeline. A run is a pure function of (scenario, seed); the Monte Carlo output does not depend on the thread count.

### Replay viewer

`make replay` runs both realistic scenarios with `--trace` and builds [`docs/pack-mesh-replay.html`](docs/pack-mesh-replay.html), a self-contained page that animates the pack. It shows mesh links, every DF bearing as it is reported (reflections are drawn dashed), the pack's own uncertainty discs coloured by status, verifier paths to their standoff points, the true sources, and each alert as it reaches the GMCS. A scrubber is marked with alerts, faults and operator actions, alongside the timeline log. It switches between the 16-drone pack and the 6-drone storyline, works in light and dark, and needs no server (`tools/build_replay.py` embeds the traces in `tools/replay_viewer_template.html`).

### What the realistic simulation found

Running the flight code under these conditions exposed defects that the synchronised, idealised sim had hidden. All of them are fixed in the flight library and covered by unit tests. Numbers in brackets were measured when each fix went in; the current results are in deliverable 2, section 3.

| Defect | Fix |
|---|---|
| Two verifiers on one event traded primary and secondary roles every few seconds, re-pointing both aircraft | Incumbency is per slot (median time to PRIORITY in the storyline: 319 s to 162 s) |
| Drones with slightly different tables both diverted; the loser of the claim turned back half a second later (up to 115 FCS requests per drone in the 16-drone run) | Claim before flying: the FCS is asked to divert once the slot has held for 1.5 s (median 85 to 19 requests) |
| The FCS was re-pointed whenever the estimate moved 3 CEP, which noise alone does about 4 % of the time | The move must exceed 3x the combined error of both estimates and last 8 s; a much better fix re-points once |
| A second reading from the same DF that a noisy gate called "different" was kept forever, splitting the source, and the split sustained itself | A newer reading supersedes an older one that agrees with it (gate widened by sqrt 2 for two noisy readings) or that the sensor stopped reporting |
| Two reflected bearings always cross: ghost ADVISORY alerts (1.3 per drone-hour) | A bearing-only ADVISORY needs 3 bearings and must hold for 10 s; consensus over bearing pairs rejects reflected outliers (0.04 per drone-hour) |
| A false IR detection plus DF bearings passing near it made a false PRIORITY (0.1 per drone-hour in the 16-drone run) | Bearings count as a second sensor type only if at least three, none explained better by another source on the frequency, triangulate there on their own (0 in every scenario since) |
| A regrouped source lost its "alert sent" flag and was alerted again; a confirmed source was re-verified after its fixes aged out | A ledger of alerts by id and by place |
| A missed `ALERT_ISSUED` notice in a radio fade let a backup drone alert again | Each drone repeats its notices for 20 s |
| Lone bearings and reflections on a frequency already localised competed for verifiers with the real source | Their urgency is halved, and a lone bearing on a localised frequency is not tasked |
| A one-frame false IR or EO detection, and three or four DF bearings on a beacon's frequency (mostly reflections) that happened to cross near it, made a false PRIORITY (6 in a 200-trial run of the 16-drone scenario) | Each drone confirms its own imaging detections over two looks before it shares them, as an imaging tracker does (`fix_confirm_count`) |
| A reading re-sent under the id the observer's table had since moved it to was counted twice. The copy stood alone as an event with no frequency, collected another frequency's bearings that passed near it, and turned a verifier on and off every few seconds | A reading is the same reading whatever id it was filed under |
| Two reflected bearings on a confirmed beacon's frequency crossed 8 km from it and pulled a verifier off for the ghost | On a frequency localised better elsewhere, a crossing of fewer than three bearings is not tasked (the same bar as a bearing-only ADVISORY) |

The last three were found in the final 200-trial runs, after the 40-trial runs had shown no false PRIORITY. One was in the simulator itself: its ground-truth labels for two sources on one frequency, sampled in the same DF scan, overwrote each other. That only affected the false-alert accounting; it is fixed too.

## Continuous integration

`.github/workflows/ci.yml` runs on every push and pull request:

| Job | What it checks |
|---|---|
| build + test | `make test`, every scenario and `make audit` with GCC and Clang (Ubuntu) and Apple Clang (macOS); two runs of each realistic scenario must be byte-identical. Scenario checks gate on Linux; macOS runs them for information (libm differs in the last bit, which changes the random trajectory) |
| Monte Carlo | 20 trials of each realistic scenario; the distributions are kept as a build artifact |
| ASan + UBSan | Tests and simulation under AddressSanitizer and UndefinedBehaviorSanitizer |
| fuzz | libFuzzer with ASan and UBSan on the decoder and receive path for 120 s; any crashing input is kept as an artifact. Every matrix build also replays the committed corpus |
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
| `situational_awareness.hpp/.cpp` | `SituationalAwarenessTable`: peers, fused events, verifier slots, the common learning of the situation (benign signatures), and the deterministic re-clustering |
| `coordination.hpp/.cpp` | Mission task prioritisation: urgency, suitability, verifier selection, alert levels and alert-owner ranking (pure functions) |
| `pack_mesh_manager.hpp/.cpp` | `PackMeshManager`: discovery, state sync, `evaluateAndCoordinateAlerts()` |
| `hal.hpp` | Interfaces to the Mesh Network Transceiver, clock, C2 link and FCS, so the logic runs on any RTOS, a host, or the simulator |
| `gmcs/alert_consumer.hpp`, `src/gmcs_alert_consumer.cpp` | GMCS side: the alert consumer (de-duplication, acknowledgement, dismissal workflow, latency accounting) |

### Pack independence: how a pack decides with no leader

1. Each drone broadcasts a heartbeat (1 Hz) with its position, fuel, active sensor and mode, and each sensor reading as it happens; an imager's detection once a second look has confirmed it.
2. Every drone folds the same readings into the same table. Own readings are re-sent every 2 s, so a lost packet only delays convergence. On every refresh the table re-clusters all its readings from scratch in a canonical order (fixes first, then bearings one frequency at a time), so which readings form an event, and the event's id, depend only on the set of readings: not on the order they arrived in, and not on which drone holds them. An event's id is the lowest id its readings carry, so ids converge across the pack; every id that moves keeps an alias.
3. `evaluateAndCoordinateAlerts()` runs every 0.5 s on every drone: fuse bearings into positions, merge duplicate events, rank by urgency (mission task prioritisation), and compute the verifier plan. The plan is a deterministic function of the table, so drones with the same table agree without negotiating.
4. A drone that finds itself in the plan sends a request to the FCS (standoff point, approach bearing, sensor to cue) and claims its slot on the mesh. Claims are arbitrated by score (quantised, so every drone ends with the same holder whatever the arrival order), ties by lowest id. A drone that loses arbitration is not ordered off its task; it gives way at the next plan, because the winner now holds the incumbent bonus.
5. Alerts to the GMCS: ADVISORY once an event is localised well enough, PRIORITY once two sensor types agree. One drone is the owner, with ranked backups; the GMCS also de-duplicates by event id.

### At the GMCS: the alert consumer

`amc8270::gmcs::AlertConsumer` turns the pack's alerts into records an operator works through.

- **De-duplication.** An alert folds into an existing record by event id, or else by place: a compatible frequency, within max(2 km, 3 x the combined CEP), seen within 30 minutes. That catches the same source reported under two ids, as a mesh partition or a split under multipath can do. A PRIORITY on an ADVISORY record upgrades it and re-opens it for acknowledgement. A better estimate updates the position. Anything else is counted as a duplicate.
- **Acknowledgement.** Records start NEW. An unacknowledged PRIORITY past 30 s, or an ADVISORY past 2 minutes, is flagged as escalated.
- **Dismissal.** The operator declares a record benign. If it carries a frequency, the consumer sends it to the pack through `IUplink`, retrying until the C2 link accepts it, and every drone then learns it. If the pack keeps alerting on a dismissed frequency, the uplink is sent again. A record with no frequency (a fire, say) can only be suppressed at the GMCS, by place. A frequency the pack never alerted on can be dismissed directly.
- **Latency.** Transport time (drone issue to GMCS receipt) per link, first detection to receipt per alert level, and operator receipt to acknowledgement, in fixed-bin histograms.

The simulator runs every alert through it. In the 16-drone Monte Carlo, raw duplicate alerts (0.29 per trial) become 0.01 duplicate records.

### Rules the flight library follows

- No exceptions, no RTTI (`-fno-exceptions -fno-rtti`).
- No heap allocation after construction: fixed-capacity containers only. A test replaces global `operator new` and proves `tick()` allocates nothing.
- Every loop is bounded by a compile-time capacity.
- Explicit little-endian serialisation; no struct copies onto the wire.
- `make audit` fails the build if the library uses `new`, `malloc`, `throw`, `dynamic_cast`, `std::vector`, `std::string` and similar.

One `PackMeshManager` is about 128 KB of fixed storage. Half of that is a second event list the re-clustering builds into. A refresh takes 0.1 to 1.5 ms for realistic tables on a desktop core, and 8.5 ms in the worst case measured: a full table of 512 bearings on one frequency.

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

- `make test`: 72 tests, 1181 checks pass in the release build and under ASan/UBSan. They include regression tests for every confirmed finding of the independent code review (see below) and for every defect the realistic simulation found. Each of the last three fixes was checked by reverting it: a test fails.
- clang-tidy (bugprone, CERT, clang-analyzer, performance and selected Core Guidelines checks) and cppcheck report nothing on the flight library or the GMCS consumer.
- `make scenarios`: every check in the three scenario files passes. Each run is byte-identical from one run to the next and between the release and sanitizer builds.
- Monte Carlo, 200 trials of each scenario (deliverable 2, section 3.1):
  - no false PRIORITY in 600 trials;
  - PRIORITY on the storyline's beacon in 98 % of trials, median 224 s after switch-on;
  - 0.09 false ADVISORY per drone-hour in the storyline and 0.24 in the 16-drone pack, every one a real source mislocalised by multipath.
- Fuzzing: libFuzzer with ASan and UBSan ran 365,000 inputs on the final code (7 minutes), and about 1.9 million more on earlier versions during this work, with no crash and no property failure. CI fuzzes for 120 s on every push.
- ChaCha20-Poly1305 matches the RFC 8439 test vectors and OpenSSL on 858 random cases.
- A 20,000-trial Monte Carlo of the triangulation shows the reported CEP is calibrated for well-conditioned geometry (about 50 % of fixes inside it) with heavy tails for poor geometry. Details in deliverable 2.

## What is not done

These are the open items you should know about before relying on any of this:

- **The cipher is not a validated module.** ChaCha20-Poly1305 is implemented from RFC 8439 and checked against the RFC's vectors and against OpenSSL on 858 random cases, but it has not been through a crypto-module validation or a side-channel review on the target. For anything that flies, put a validated module behind `crypto.hpp`; nothing else depends on how it is implemented.
- **Insiders can forge within an epoch.** Keys are symmetric, so every pack member holds the epoch key. A drone captured with its key store intact can forge frames in any member's name until the next epoch. Revocation drops its own identity at once; re-keying without it ends the rest. Only per-sender signatures would stop an insider sooner.
- **The boot session must persist.** It has to come from a non-volatile counter that increases on every boot. If it repeats, nonces repeat and the mesh loses secrecy and integrity. Key fill, key storage and the C2 re-key message are outside this repo.
- **No flight validation.** All performance numbers come from a software simulation. Its radio, terrain, multipath and clutter models are generic, with parameters chosen to be plausible, not measured on the real radio, DF or airframe.
- **The mesh does not relay.** Every frame is one hop. A pack wider than the radio range splits into groups that cannot hear each other, and each group alerts on its own. `pack16.scn` keeps the pack inside 60 km for that reason.
- **One drone's thermal detection can raise an ADVISORY.** An IR detection of a thermal hazard, once a second look confirms it, is a fix with confidence 0.8, which meets the advisory gate. The simulator's clutter lasts one frame, so confirmation removes it (none of the false alerts in 600 trials). Persistent clutter, such as a sun-heated rock, is not modelled and would get through. Requiring two drones for a fix-only ADVISORY would stop it, at the cost of a later warning for a fire only one drone has seen; that is a programme decision.
- **Persistent clutter could still corroborate itself.** A PRIORITY on a fix plus bearings counts only bearings that fit the fix and no other source on the frequency. Those bearings are chosen because they pass near the fix, so on a busy frequency a confirmed false detection could still collect three that cross near it. It took a one-frame false detection and reflections to do that in the simulation, and confirmation removed it, but the test is not independent of the fix.
- **The Model 8270's real interfaces are assumed.** Everything touches hardware through `hal.hpp`; the bus choices in deliverable 3 are recommendations, not an ICD.
- **The FCS interface has no reply.** A real one needs accept / refuse.
- **Time assumes a shared GNSS-disciplined clock.** GNSS-denied operation is not handled.
- **The corridor is a circle.** A real approved corridor is a polygon.
- **A pack member can still lie in its own name.** Authentication proves which drone sent a frame, not that its content is true. A drone that holds a valid key can report false readings or send a `SIGNATURE_UPDATE` that dismisses a frequency pack-wide. Content checks limit the damage (ranges, a drone speaks only for itself); revocation removes the drone.
- **A partition doubles the alerts.** Each side of a split mesh localises the source under its own id and alerts the GMCS. After the heal the pack converges on one event and does not alert again (tested). The GMCS alert consumer folds the second alert into the first record by place, so the operator sees one record, but two alerts did cross the C2 links.
- **Re-clustering is greedy, not optimal.** It is deterministic and does not fragment a source under bearing noise, but it is not a global assignment. Two sources on one frequency, seen only by bearings, at an awkward geometry can still be grouped wrongly until a fix or more geometry arrives, and every drone then makes the same mistake.
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
