# Deliverable 2 — Pack capability and operational improvement

## Summary

Today the GMCS can steer the whole pack along a route but can actively fly only **one** drone in mission mode. The other drones carry sensors and a mission computer and use neither. With the airborne mesh, every Model 8270 shares what its sensors see, builds the same picture as its neighbours, and decides **without a leader** which drones should leave the route to verify a detection. The operator receives a short alert instead of a raw stream, and keeps full control of the one drone they are flying.

Every number below is labelled **[simulated]** (output of this repo's 6-drone scenario, `make sim`), **[computed]** (derived from the code's constants) or **[estimate]** (engineering judgement, to be replaced by flight data). None is a flight-test result.

---

## 1. Baseline versus mesh-enabled operations

| Aspect | Baseline: single-drone bottleneck | Mesh-enabled pack |
|---|---|---|
| Sensor suites contributing | 1 of 6 (the controlled drone) | All 6 can share readings; 5 of 6 can also be tasked (the controlled drone never is) [by design] |
| Where detections come from | Only where the controlled drone happens to be | Anywhere along a pack front tens of km wide |
| Localising an RF source | Needs the operator to fly one drone along a baseline | Several bearings arrive at once; triangulated with no drone leaving formation [simulated: 5 bearings, first fix in the same 0.1 s cycle as the first reading] |
| Verifying a detection | One drone, one angle, after the operator re-tasks it | Best-placed drones, different angles, different sensor types, nominated automatically |
| Operator workload per event | Detect, decide, re-task, interpret, report | Receive one advisory and one verified alert; optionally dismiss a false source [simulated: 0 operator actions to reach a PRIORITY alert] |
| What reaches the GMCS | Raw sensor data from one drone | Fused, de-duplicated alerts: one ADVISORY, then one PRIORITY, per event |
| C2 load | Video / IQ streams over SATCOM | Alerts only (tens to ~100 bytes each; format is out of this repo's scope) [estimate] |
| Mesh load | none | 1.7 kbit/s mean, 3.7 kbit/s peak transmitted per drone [simulated, includes headers and CRC] |
| False-alarm control | Operator judgement, per event | Multi-sensor confirmation required for PRIORITY; learned-benign signatures shared pack-wide |
| Loss of a drone | Operator notices, re-plans | Peer times out after 5 s of silence and its task is re-assigned [simulated: 4.6 s after the loss] |
| Dependence on the C2 link | Total | Detection, fusion and nomination run on board; only alerts need a link, and any drone with a link can send them |

---

## 2. What the passive drones do

### 2.1 Continuous wide-area scanning

Each drone keeps its RF direction-finder and imaging sensors running on the route. A drone that gets a reading calls `reportLocalDetection()`, which files it under an event and broadcasts it to the pack. Drones in `ACTIVE_MISSION` also share their readings, but are never tasked by the mesh.

Result in the scenario [simulated]: when the beacon came on at t = 20 s, all five drones within 35 km of it contributed a bearing at their next sensor cycle. The pack covers a front of 50 km with six aircraft; the controlled drone alone would have seen nothing, because the source was outside its range.

### 2.2 Cooperative signal triangulation

Bearing-only readings are intersected by weighted least squares in a local east/north plane (`triangulate()` in `src/geo.cpp`):

- Each bearing is weighted by its angular error and by range, because cross-range error grows with distance.
- The fix is rejected if the lines are near-parallel (smallest eigenvalue of the line-normal matrix below 0.03, about 14° crossing for two lines) or if the intersection falls behind an observer.
- A lone bearing gives only a provisional point 15 km down the line and is never trusted as a position.

Result [simulated]: 5 bearings, reported CEP 708 m, actual error 795 m.

**Is the reported CEP honest?** A 20,000-trial Monte Carlo (2.5° bearing noise, scenario geometries) gives:

| Geometry | Reported CEP (mean) | P(error ≤ CEP) | P(error > 3 × CEP) |
|---|---|---|---|
| Beacon, 6 observers spread over 50 km | 647 m | 49.9 % | 0.5 % |
| ISM emitter, 4 observers, one side, 30 km | 1,394 m | 51.3 % | 2.0 % |
| ISM emitter, 3 observers | 1,866 m | 51.5 % | 2.8 % |
| Weather radio, 3 observers at the edge of range | 3,417 m | 50.1 % | 5.8 % |

So the CEP is calibrated for the typical case (about half of fixes fall inside it, as a CEP should) but poor geometry has heavy tails: a few percent of fixes are more than 3 × CEP off. That is acceptable for an **early-warning advisory**, which only cues a closer look, and is why PRIORITY requires a second sensor type.

### 2.3 Autonomous event prioritisation

Every event gets an urgency, computed identically on every drone:

```
urgency = severity(class) × novelty × (0.3 + 0.7 × confidence) × (0.5 + 0.5 × freshness)
```

| Class | Severity |
|---|---|
| Distress beacon | 1.0 |
| Thermal hazard | 0.9 |
| Anomaly | 0.7 |
| Unclassified RF emitter | 0.5 |
| Unknown | 0.4 |

- **Novelty** is 1.0, or 0.2 if the signature is known benign (pre-briefed in mission data, or taught by the operator).
- **Confidence** is a noisy-OR of the individual readings, capped at 0.85 when only one sensor type contributes, 0.99 with two or more, and 0.5 if the event is not localised. A single sensor type can therefore never confirm an event.
- **Freshness** falls from 1 to 0 over 60 s without a new reading.

Events are ranked by urgency (ties by event id). Below 0.25 an event is tracked but never causes a diversion.

Result [simulated]: the beacon ranks 0.90, the novel ISM emitter 0.45, the pre-briefed weather radio 0.09, so only the beacon pulls aircraft off the route first.

### 2.4 Shared environmental learning

- **Pre-briefed benign signatures** are loaded on every drone from mission data.
- **Operator dismissals** are learned and shared: when the operator marks an emitter benign, the drone holding the GMCS link broadcasts a `SIGNATURE_UPDATE`. Every peer marks the signature benign, drops the event, and releases any drone verifying it. Learned signatures are re-broadcast every 2 s so a drone that missed the message, or joined late, catches up.
- **Convergence** under loss: observations are re-sent every 2 s by the drone that made them. In a 30 %-loss test all five tables held the same event with the same four observations after 25 s.

Result [simulated]: at t = 420 s the dismissal of the ISM emitter reached all live drones; the drone verifying it was released the next cycle.

### 2.5 Rapid multi-sensor verification and alerting

**Who goes** is decided by `planAssignments()` from the shared table, so drones with the same table reach the same answer with no negotiation. For each event, in urgency order, the best-fitting free drone is chosen:

```
suitability = 0.40 × time-to-standoff  +  0.25 × new sensor modality
            + 0.15 × fuel margin       +  0.20 × viewing-angle diversity
            (+ 0.15 if already the verifier, to prevent flip-flopping)
```

Hard limits:

- Only drones in `PACK_ROUTING` (or already verifying) can be chosen. `ACTIVE_MISSION` and `RTB` never are.
- Fuel: `endurance remaining − bingo reserve − (2 × time to standoff + 300 s dwell) ≥ 0`.
- Standoff point must be inside the GMCS-approved corridor (circular geofence in this implementation) and within 60 km of the drone.
- At most `floor(0.4 × free drones)` are diverted at once, minimum one, so most of the pack keeps scanning.
- Two verifiers if urgency ≥ 0.6, otherwise one. The second verifier approaches from the angle that is furthest from everything already covering the source.

**What they do:** the chosen drone sends a verification request (standoff point, approach bearing, sensor to cue) to the FCS. The FCS keeps authority and may refuse. Every ~0.5 s the nominee re-checks; the FCS is re-tasked only if the target estimate moved by more than `max(500 m, 3 × target CEP)`, so noise in the fused position does not make the aircraft wander [simulated: busiest drone issued 4 requests in 600 s].

**Alerts** go to the GMCS once per level per event:

| Level | Condition | Result in the scenario [simulated] |
|---|---|---|
| ADVISORY (early warning) | severity ≥ 0.7, confidence ≥ 0.6, CEP ≤ 2 km | t = 20.1 s, CEP 708 m, via LEO SATCOM (the owner had no LOS) |
| PRIORITY (verified) | confidence ≥ 0.9, CEP ≤ 250 m, ≥ 2 observers, ≥ 2 sensor types | t = 328.1 s, CEP 70 m, actual error 28 m, via LOS |

One drone is the alert owner: the primary verifier first, then the secondary, then observers, then any free drone with a link. A lower-ranked drone waits 3 s per rank and cancels if it hears `ALERT_ISSUED`, so the pack sends one alert per event; the GMCS also de-duplicates by event id.

---

## 3. Scenario timeline [simulated]

Source: `make sim`, 600 s, deterministic. Pack: D1 operator-controlled, D2–D6 routing, 10 km apart, 40 m/s. D5 is near bingo fuel. D4 has no LOS link.

| t (s) | What happens |
|---|---|
| 10 | Pre-briefed weather radio detected by 3 drones; urgency 0.09, ignored |
| 20.0 | Distress beacon (with a thermal source) switches on 16–28 km from the five drones that can hear it; the controlled drone D1 is 36 km away, out of DF range |
| 20.1 | 5 drones share bearings; triangulated, CEP 708 m; **ADVISORY** sent by D4 via LEO SATCOM; D4 (east approach) and D6 (north approach) nominated; D5, the second-nearest drone, is excluded for fuel |
| 90 | Fault injection: D4, the primary verifier, is lost |
| 94.6 | D6 re-tasked as primary after peers time D4 out; budget now allows one verifier |
| 100 | Novel ISM emitter appears; localised (CEP 1.5 km), urgency 0.45, waits behind the beacon |
| 304–310 | D6 closes to 10 km; IR fixes arrive; the FCS is re-pointed twice as the estimate improves |
| 328.1 | Two sensor types agree: CEP 70 m, error 28 m, confidence 0.99; **PRIORITY** sent; D6 released |
| 328.6 | D2 nominated to look at the ISM emitter |
| 420 | Operator dismisses the ISM emitter via D1; all drones learn it; D2 released |

The five-minute gap between ADVISORY and PRIORITY is flight time: the verifier flew at 45 m/s until its imaging sensor was within 10 km of the source. The mesh does not shorten the transit. It makes sure the right drones start it at once, from a position estimate already good to under a kilometre, and tells the operator in the meantime.

---

## 4. Mesh traffic [simulated and measured]

Encoded sizes (header 20 B + payload + 4 B CRC):

| Message | Size | When |
|---|---|---|
| HEARTBEAT (position, fuel, sensor, mode, task) | 70 B | 1 Hz, and immediately on a mode/task change |
| OBSERVATION | 85 B | On each reading, then every 2 s while it is current |
| TASK_CLAIM | 35 B | On a task change, then every 4 s |
| ALERT_ISSUED | 31 B | Once per alert |
| SIGNATURE_UPDATE | 28 B | On dismissal, then every 2 s |
| LEAVE | 24 B | On shutdown |

In the scenario: **1.69 kbit/s mean and 3.69 kbit/s peak transmitted per drone**, including 5 % simulated loss. Every transmission is a broadcast, so a shared channel carries the sum: 6 × 1.69 ≈ 10 kbit/s mean and 6 × 3.69 ≈ 22 kbit/s peak [computed], about 1 % and 2 % of a modest 1 Mbit/s mesh radio. Load grows roughly linearly with pack size. By comparison a single compressed EO/IR video stream is of the order of megabits per second [estimate], which is why the system shares detections and not video.

Memory: one `PackMeshManager` is about 60 KB of fixed storage, allocated once. No heap allocation occurs after start-up (verified by a test that replaces global `operator new`).

---

## 5. Failure modes and safeguards

| Situation | What the design does | Verified? |
|---|---|---|
| Corrupt or malformed packet | CRC-32, framing, enum and range checks; NaN or out-of-range values rejected and counted | Yes: corrupting any one byte of a test packet, any truncation, NaN and out-of-range fields are all rejected |
| Packet loss | Sequence gaps estimate link quality; own readings re-sent every 2 s | Yes: 30 % loss converges in 25 s |
| Peer lost | Dropped after 5 s; its verification slot freed and re-assigned | Yes, unit test and scenario |
| Drone leaves the mesh deliberately | `LEAVE` removes it at once | Yes |
| GMCS takes a drone into `ACTIVE_MISSION` | Mesh task cancelled at once; peers re-plan | Yes |
| Low fuel | Excluded by the fuel rule; it still contributes readings | Yes |
| Alert owner has no link, or the send fails | Retried each cycle; the next-ranked drone takes over after 3 s | Yes |
| Mesh partition | Each side keeps working from its own table; both sides may alert for the same event; the GMCS de-duplicates by event id | By design; not tested |
| Same source filed under two ids | Merged into the lowest id; old id kept as an alias | Yes |
| Table full | Lowest-urgency event without a verifier is evicted | Yes |
| FCS refuses a task | Not modelled: the request interface returns nothing; a real interface needs an accept/reject reply | Open |
| **Message authentication and encryption** | **Not implemented.** A CRC detects accidents, not an attacker. The packet format has no key or MAC field yet | **Open: required before any real use** |
| **GNSS-denied time** | Timestamps assume a PPS-disciplined clock shared by the pack; if it drifts, observation ageing and ordering degrade | **Open** |

## 6. What this does not do

- No weapons or engagement logic. The mesh detects, localises, verifies and alerts.
- No steering of the flight controls. It sends requests to the FCS, which enforces the approved corridor.
- No claim of flight-validated performance. The scenario uses idealised sensors (Gaussian bearing error, no multipath, no terrain masking), zero radio latency, and a mesh range limited only by distance.

## 7. Decisions needed from the programme

1. Mesh radio type, capacity and range, to replace the sim's 60 km / 5 % loss assumption.
2. Authentication, encryption and key management for the mesh, and replay protection.
3. The GMCS alert message format and the operator's dismissal workflow.
4. The FCS's accept/reject reply and the shape of the approved corridor (polygon rather than circle).
5. Whether `ACTIVE_MISSION` drones should share raw readings (assumed yes) or only alerts.
6. Flight-test measurements to set the urgency weights, standoff distance and DF error model.
