# Deliverable 2 — Pack capability and operational improvement

## Summary

Today the GMCS can steer the whole pack along a route but can actively fly only **one** drone in mission mode. The other drones carry sensors and a mission computer and use neither. This is the **single-drone GMCS bottleneck**, and the mesh removes it by **exploiting drones while they're under pack routing**. With the airborne mesh, every Model 8270 shares what its sensors see, builds the same picture as its neighbours, and decides **without a leader** which drones should leave the route to verify a detection. The operator receives a short alert instead of a raw stream, and keeps full control of the one drone they are flying.

Every number below is labelled **[simulated]**, **[computed]** (derived from the code's constants) or **[estimate]** (engineering judgement, to be replaced by flight data). Simulated numbers come from this repo's simulator running the real flight code under modelled radio latency and fades, terrain, DF multipath and sensor clutter: either the 6-drone storyline (`make sim`, seed 1) or a 200-trial Monte Carlo (`make mc`), as marked. None is a flight-test result.

---

## 1. Baseline versus mesh-enabled operations

| Aspect | Baseline: single-drone GMCS bottleneck | Mesh-enabled pack |
|---|---|---|
| Sensor suites contributing | 1 of 6 (the controlled drone) | All 6 can share readings; 5 of 6 can also be tasked (the controlled drone never is) [by design] |
| Where detections come from | Only where the controlled drone happens to be | Anywhere along a pack front tens of km wide |
| Localising an RF source | Needs the operator to fly one drone along a baseline | Several bearings arrive at once; triangulated with no drone leaving formation [simulated: first fix from 3 bearings 1 s after the first reading] |
| Verifying a detection | One drone, one angle, after the operator re-tasks it | Best-placed drones, different angles, different sensor types, nominated automatically |
| Operator workload per event | Detect, decide, re-task, interpret, report | Receive one advisory and one verified alert; optionally dismiss a false source [simulated: 0 operator actions to reach a PRIORITY alert] |
| What reaches the GMCS | Raw sensor data from one drone | Fused, de-duplicated alerts: one ADVISORY, then one PRIORITY, per event, folded at the GMCS into one record per source [simulated, Monte Carlo: 1.05 records per trial for the one real source] |
| C2 load | Video / IQ streams over SATCOM | Alerts only (tens to ~100 bytes each; format is out of this repo's scope) [estimate] |
| Mesh load | none | 2.2 kbit/s mean per drone; 5.0 kbit/s in the busiest second (p95 over trials) [simulated, Monte Carlo, sealed frames including header and tag] |
| False-alarm control | Operator judgement, per event | Multi-sensor confirmation required for PRIORITY; common learning of the situation spreads each benign signature pack-wide |
| Loss of a drone | Operator notices, re-plans | Peer times out after 5 s of silence and its task is re-assigned [simulated: 5.1 s after the loss] |
| Pack independence | None: every decision waits for the operator and the C2 link | Detection, fusion, mission task prioritisation and nomination run on board with no leader; only alerts need a link, and any drone with a link can send them |

---

## 2. Exploiting drones while they're under pack routing

A drone in `PACK_ROUTING` flies the route the GMCS set for the pack. The mesh does not change that route unless the pack decides the drone should verify something, and even then it only asks the FCS. Everything below happens on drones the operator is not flying.

### 2.1 Continuous wide-area scanning

Each drone keeps its RF direction-finder and imaging sensors running on the route. A drone that gets a reading calls `reportLocalDetection()`, which files it under an event and broadcasts it to the pack. Drones in `ACTIVE_MISSION` also share their readings, but are never tasked by the mesh.

An imager's detection is shared only once a second look from the same sensor confirms it, as an imaging tracker would (`fix_confirm_count`, default 2 looks within 5 s). A one-frame false detection, such as a glint, then never reaches the pack. A confirmed source's later detections go out at once.

Result in the storyline [simulated]: the beacon came on at t = 20 s. Three drones had a bearing on it within 2 s, and all five within 35 km of it by the time of the advisory. The pack covers a front of 50 km with six aircraft; the controlled drone alone would have seen nothing, because the source was 37 km away, outside its DF range.

### 2.2 Cooperative signal triangulation

Bearing-only readings are intersected by weighted least squares in a local east/north plane (`triangulate()` in `src/geo.cpp`):

- Each bearing is weighted by its angular error and by range, because cross-range error grows with distance.
- The fix is rejected if the lines are near-parallel (smallest eigenvalue of the line-normal matrix below 0.03, about 14° crossing for two lines) or if the intersection falls behind an observer.
- A lone bearing gives only a provisional point 15 km down the line and is never trusted as a position.

Result [simulated]: the first fix, from 3 bearings, had a reported CEP of 913 m and an actual error of 475 m. Ten seconds later, from 5 bearings, the CEP was 604 m but the error 1.8 km: D2's bearing had entered a multipath reflection and read 12° off. That is the tail the next table describes, and why the ADVISORY is only a cue.

**Is the reported CEP honest?** A 20,000-trial Monte Carlo (2.5° bearing noise, scenario geometries) gives:

| Geometry | Reported CEP (mean) | P(error ≤ CEP) | P(error > 3 × CEP) |
|---|---|---|---|
| Beacon, 6 observers spread over 50 km | 647 m | 49.9 % | 0.5 % |
| ISM emitter, 4 observers, one side, 30 km | 1,394 m | 51.3 % | 2.0 % |
| ISM emitter, 3 observers | 1,866 m | 51.5 % | 2.8 % |
| Weather radio, 3 observers at the edge of range | 3,417 m | 50.1 % | 5.8 % |

So the CEP is calibrated for the typical case (about half of fixes fall inside it, as a CEP should) but poor geometry has heavy tails: a few percent of fixes are more than 3 × CEP off. That table assumes Gaussian bearing error. Multipath makes the tails heavier: a reflected bearing is wrong by 10 to 30° for seconds at a time, and two reflected bearings always cross somewhere. Two safeguards follow from that:

- **Outlier rejection.** Bearings on one frequency are grouped by consensus over pairs, so a bearing that disagrees with the rest is set aside rather than averaged in.
- **Holds.** A bearing-only ADVISORY needs at least 3 bearings, and the position must hold for 10 s. A PRIORITY needs a second sensor type that the bearings confirm on their own (section 2.5).

That is acceptable for an **early-warning advisory**, which only cues a closer look.

### 2.3 Mission task prioritisation

Mission task prioritisation decides which detections earn a drone's time. Every event gets an urgency, computed identically on every drone:

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
- **Secondary sources.** A second event on a frequency the pack has already localised elsewhere is more often a reflection than a second source. Its urgency is halved so it waits behind the first. A lone bearing there is never tasked, and neither is a crossing of fewer than 3 bearings, since two reflected bearings always cross somewhere.

Events are ranked by urgency (ties by event id). Below 0.25 an event is tracked but never causes a diversion.

Result [simulated]: the beacon ranks 0.82, the novel ISM emitter 0.41, the pre-briefed weather radio 0.08, so only the beacon pulls aircraft off the route first.

### 2.4 Common learning of the situation

The pack keeps one shared table of what it has learned about the environment: which signatures are benign and which events have already been reported. Every drone holds the same copy, so a lesson learned once applies to all of them.

- **Pre-briefed benign signatures** are loaded on every drone from mission data.
- **Operator dismissals** are learned and shared: when the operator marks an emitter benign, the drone holding the GMCS link broadcasts a `SIGNATURE_UPDATE`. Every peer marks the signature benign, drops the event, and releases any drone verifying it. Learned signatures are re-broadcast every 2 s so a drone that missed the message, or joined late, catches up.
- **Convergence** under loss: observations are re-sent every 2 s by the drone that made them. In a 30 %-loss test all five tables held the same event with the same four observations after 25 s.
- **Same grouping everywhere.** Every refresh re-clusters all readings in a canonical order, so drones holding the same readings hold the same events under the same ids, whatever order the readings arrived in. A test feeds the same six readings in all 720 orders and gets one result; the previous greedy association produced three different groupings of the same readings.

Result [simulated]: at t = 420 s the operator dismissed the ISM emitter through D1, and every live drone learned it. D4, which was verifying it, was released 0.6 s later. Over 200 trials the dismissal was learned by every drone in every trial.

### 2.5 Rapid multi-sensor verification and alerting

**Who goes** is decided by `planAssignments()` from the shared table, so drones with the same table reach the same answer with no negotiation. For each event, in urgency order, the best-fitting free drone is chosen:

```
suitability = 0.40 × time-to-standoff  +  0.25 × new sensor modality
            + 0.15 × fuel margin       +  0.20 × viewing-angle diversity
            (+ 0.30 if already holding this slot, so a newcomer must be clearly better)
```

Hard limits:

- Only drones in `PACK_ROUTING` (or already verifying) can be chosen. `ACTIVE_MISSION` and `RTB` never are.
- Fuel: `endurance remaining − bingo reserve − (2 × time to standoff + 300 s dwell) ≥ 0`.
- Standoff point must be inside the GMCS-approved corridor (circular geofence in this implementation) and within 60 km of the drone.
- At most `floor(0.4 × free drones)` are diverted at once, minimum one, so most of the pack keeps scanning.
- Two verifiers if urgency ≥ 0.6, otherwise one. The second verifier approaches from the angle that is furthest from everything already covering the source.

**What they do:** the chosen drone claims its slot on the mesh and, once the claim has held for 1.5 s, sends a verification request (standoff point, approach bearing, sensor to cue) to the FCS. The wait means two drones whose tables differ for a moment do not both turn. The FCS keeps authority and may refuse. Every ~0.5 s the nominee re-checks. The FCS is re-pointed only if the estimate has moved by more than 3 × the combined error of the old and new estimates (at least 500 m), and the move has lasted 8 s. Neither noise nor a reflection lasting a few seconds makes the aircraft wander. A much better fix, such as an imager's, re-points it once [simulated: the busiest drone sent 9 requests in the storyline, most of them following the ISM emitter's wandering estimate; over 200 trials, 7 at the median and 12 at p95].

**Alerts** go to the GMCS once per level per event:

| Level | Condition | Result in the scenario [simulated] |
|---|---|---|
| ADVISORY (early warning) | severity ≥ 0.7, confidence ≥ 0.6, CEP ≤ 2 km. From bearings alone: at least 3 bearings, and the position must hold for 10 s | t = 31.7 s (11.7 s after switch-on), 5 bearings, CEP 604 m, actual error 1.8 km, via LOS |
| PRIORITY (verified) | confidence ≥ 0.9, CEP ≤ 250 m, ≥ 2 observers, ≥ 2 sensor types. If bearings are one of the types, at least 3 bearings that fit no other source on the frequency better must triangulate to the fused position on their own | t = 151.8 s (131.8 s after switch-on), CEP 69 m, actual error 24 m, via LEO SATCOM (the owner, D4, has no LOS) |

The PRIORITY rule on bearings matters because a false imaging detection is a point that bearings on a busy frequency may pass near. The rule alone was not enough. In a 200-trial run of the 16-drone scenario, 6 trials still produced a false PRIORITY: a one-frame false detection with three or four bearings, mostly reflections, crossing near it. Since imaging detections need a second look (section 2.1), no trial has (section 3.1).

One drone is the alert owner: the primary verifier first, then the secondary, then observers, then any free drone with a link. A lower-ranked drone waits 3 s per rank and cancels if it hears `ALERT_ISSUED`. Each drone repeats its notices for 20 s, so one lost in a radio fade is heard on a repeat. The pack therefore sends one alert per level per event, and the GMCS alert consumer folds whatever duplicates still arrive (section 2.7).

### 2.6 Pack independence

Pack independence means the pack keeps doing all of the above with no leader and with no help from the GMCS:

- **No leader.** Every drone runs the same `evaluateAndCoordinateAlerts()` on the same table and reaches the same plan. No drone is special, so no single loss stops the pack.
- **No C2 link needed to decide.** Detection, fusion, mission task prioritisation and verifier nomination all run on board. Only the alert itself needs a link, and any drone with one can send it.
- **Losses are absorbed.** A silent drone times out after 5 s and its task moves to another drone.
- **The operator keeps the one drone they fly.** The mesh reads that drone's sensors but never tasks it.

### 2.7 At the GMCS: the alert consumer

In the common case the pack sends one ADVISORY and one PRIORITY per source. Some failures can still report one source under two event ids: a mesh partition, a source split by multipath, or an `ALERT_ISSUED` notice lost in a fade. `amc8270::gmcs::AlertConsumer` (`include/amc8270/gmcs/alert_consumer.hpp`) turns whatever arrives into records an operator works through:

- **De-duplication.** An alert folds into an existing record by event id, or else by place: a compatible frequency, within max(2 km, 3 × the combined CEP), seen in the last 30 minutes. A PRIORITY on an ADVISORY record upgrades it and puts it back in front of the operator. A better estimate updates the position.
- **Acknowledgement.** Every record starts NEW. A PRIORITY not acknowledged within 30 s, or an ADVISORY within 2 minutes, is flagged as escalated.
- **Dismissal workflow.** The operator declares a record benign.
  - With a frequency, the dismissal goes up to the pack and is retried until the C2 link accepts it. Every drone learns it, and later alerts on it are suppressed. If the pack keeps alerting on it, the dismissal is sent again.
  - Without a frequency (a fire), the record can only be suppressed at the GMCS, by place.
  - A frequency the pack never alerted on can be dismissed directly, as the storyline does with the ISM emitter.
- **Alert-latency accounting** in fixed-bin histograms:
  - transport (drone to GMCS) per link;
  - first detection to receipt, per level;
  - receipt to acknowledgement.

It is ground software, but it follows the flight-library rules (fixed capacity, no heap, no exceptions), so it can run on an embedded ground terminal. Every simulated alert goes through it:

| [simulated, Monte Carlo] | Storyline (6 drones) | 16-drone pack |
|---|---|---|
| Raw duplicate alerts per trial | 0.02 | 0.29 |
| Records beyond the first for one source, per trial | 0.01 | 0.01 |
| Transport, LOS: p50 / p95 | 78 / 141 ms | 82 / 146 ms |
| Transport, LEO SATCOM: p50 / p95 | 201 / 273 ms | 200 / 277 ms |
| First detection to ADVISORY at the GMCS: p50 / p95 | 11.2 / 12.8 s | 11.1 / 46.2 s |
| First detection to PRIORITY at the GMCS: p50 / p95 | 200 / 436 s | 15 / 230 s |

In the 16-drone pack the median detection-to-PRIORITY time is seconds, because most PRIORITY alerts there are for the fire. An imager passing within 3 km finds it, and a second sensor type follows within seconds.

---

## 3. Scenario timeline [simulated]

Source: `make sim` (`sim/scenarios/storyline6.scn`, seed 1), 600 s, deterministic. Pack: D1 operator-controlled, D2–D6 routing, 10 km apart, 40 m/s. D5 is near bingo fuel. D4 has no LOS link. Conditions:

- **Radio:** latency 8 ms plus jitter (mean 20 ms); 2 % loss, and fades on each link that lose 60 % of about ten datagrams in a row.
- **Terrain:** a hill in front of the ISM emitter and a ridge to the north.
- **DF multipath:** 2 % of readings start a reflection that lasts 5–30 s.

| t (s) | What happens |
|---|---|
| 10 | Pre-briefed weather radio comes on; three drones localise it by 12 s; urgency 0.08, ignored |
| 20.0 | Distress beacon (with a thermal source) switches on, 16–28 km from the five routing drones. D1, the controlled drone, is 37 km away, out of DF range |
| 22.0 | Three bearings: localised, CEP 913 m, actual error 475 m, urgency 0.82. D2 (EO, from the east) and D4 (IR, from the south-south-east) are nominated. D5, second-nearest, is excluded for fuel |
| 31.7 | Five bearings, held for the 10 s a bearing-only warning needs: **ADVISORY** sent by D2 over LOS. CEP 604 m; actual error 1.8 km, because D2's own bearing was in a reflection |
| 90 | Fault injection: D2, the primary verifier, is lost |
| 95.1 | Peers time D2 out and D4 becomes primary; with one drone fewer, the budget allows one verifier |
| 100–103 | Novel ISM emitter comes on behind the hill; two drones hear it, CEP 5.8 km, urgency 0.41; it waits behind the beacon |
| 151.8 | D4's IR has seen the source on two looks, and its fix and the pack's bearings agree: CEP 69 m, actual error 24 m, confidence 0.99. **PRIORITY** sent by D4 over LEO SATCOM |
| 153.6–412 | D4, free again, is sent to the ISM emitter. Behind the hill few drones hear it at once, so its estimate moves by kilometres as bearings come and go. D4 is re-pointed six times as it moves (9 FCS requests in all, the most of any drone) |
| 420 | Operator dismisses the ISM emitter by frequency (no alert was raised for it: an unclassified emitter is below the alert severity). Every drone learns it; D4 is released 0.6 s later |

The two minutes between ADVISORY and PRIORITY are flight time: D4 had to close from 16 km to within its IR's 10 km cued range. The mesh does not shorten the transit. It makes sure the right drones start it at once, from an estimate with a CEP under a kilometre, and tells the operator in the meantime.

The same storyline with the earlier idealised models (`ideal6.scn`: no latency, independent 5 % loss, flat ground, no multipath) gives an ADVISORY at the same 11.7 s and a PRIORITY at 131.8 s with an error of 81 m.

### 3.1 Monte Carlo [simulated]

One run is one draw. `make mc` repeats each scenario 200 times. Each trial reseeds every random process (sensor noise, multipath, fades, clutter) and moves each source and its switch-on time by up to the scenario's jitter: 3 km and 10 s in the storyline, 4 km and 20 s in the 16-drone pack. Times are from the source switching on. A false alert is one with no real, alertable source within max(2 km, 3 × CEP) of it.

| | Storyline, idealised models (`ideal6`) | Storyline, realistic (`storyline6`) | 16-drone pack (`pack16`) |
|---|---|---|---|
| Drones, length of run | 6, 600 s | 6, 600 s | 16, 900 s |
| First alert on the beacon, p50 / p95 | 11.5 / 12.2 s | 11.5 / 12.3 s | beacon 1: 11.0 / 578 s |
| PRIORITY on the beacon: trials, p50 | 100 %, 136 s | 98 %, 224 s | beacon 1: 75 %, 582 s |
| PRIORITY position error, p50 / p95 | 54 / 105 m | 50 / 104 m | 56 / 114 m |
| False PRIORITY | 0 | 0 | 0 |
| False ADVISORY per drone-hour | 0.01 | 0.09 | 0.24 |
| Duplicate alerts per trial (same source, same level) | 0.00 | 0.02 | 0.29 |
| GMCS records beyond the first for one source, per trial | 0.00 | 0.01 | 0.01 |
| Alert transport, p50: LOS / LEO SATCOM | 82 / 200 ms | 78 / 201 ms | 82 / 200 ms |
| FCS requests by the busiest drone, p50 / p95 | 2 / 3 | 7 / 12 | 18 / 25 |
| Mesh load per drone: mean / busiest second, p95 | 2.20 / 4.10 kbit/s | 2.22 / 4.93 kbit/s | 2.43 / 4.92 kbit/s |

No trial of any scenario raised a false PRIORITY. Every false ADVISORY is a real source mislocalised by multipath.

The 16-drone scenario is deliberately hard:

- **Faults.** Beacon 1's verifier is lost at 240 s, and the mesh is split in two from 300 s to 360 s.
- **Two beacons on one frequency.** They are 33 km apart, and a ridge hides the second from part of the pack.
- **A fire with no RF emission.** Only an imager passing within 3 km can find it.
- **Clutter.** IR and EO sensors make 0.2 false detections per drone-hour.

Beacon 1 is warned of within 11 s in most trials. Its PRIORITY has to wait for a verifier that survives the loss and the partition, and arrives within the 900 s in 75 % of trials. Beacon 2 is the hard case: alerted in 68.5 % of trials and confirmed in 12.5 %, because the pack must first tell two sources on one frequency apart from bearings. The fire is alerted in every trial and confirmed in 99.5 % (median 207 s).

Two numbers changed in the last round of fixes, measured on the same 200 trials of the 16-drone scenario:

- **False PRIORITY: 6 → 0.** Each false PRIORITY was a one-frame false detection with three or four bearings on a beacon's frequency, mostly reflections, crossing near it. A drone now shares an imaging detection only once a second look confirms it.
- **False ADVISORY: 0.41 → 0.24 per drone-hour.** Clutter no longer causes any.

### 3.2 The advisory hold: early warning against false warnings [simulated]

A bearing-only ADVISORY waits until the position has held for `advisory_bearing_hold_ms`. A reflection lasts seconds and a source stays put, so the hold filters reflections, but it delays every warning by the same amount. Storyline, 200 trials per row:

| Hold | First alert, p50 / p95 | False ADVISORY per drone-hour | Trials with a false ADVISORY | False records at the GMCS per drone-hour | ADVISORY before PRIORITY |
|---|---|---|---|---|---|
| 0 s | 1.5 / 21.5 s | 1.72 | 78 % | 1.54 | 94 % |
| 5 s | 6.5 / 7.2 s | 0.32 | 22.5 % | 0.30 | 95 % |
| **10 s (default)** | **11.5 / 12.3 s** | **0.09** | **7.5 %** | **0.07** | **94.5 %** |
| 20 s | 21.6 / 291 s | 0.17 | 14.5 % | 0.10 | 87 % |

Ten seconds removes 95 % of the false warnings for 10 s of delay. Longer holds stop paying. At 20 s the hold is often broken before it completes, as bearings come and go, so in 13 % of trials the ADVISORY does not come before the PRIORITY (5.5 % at 10 s), and false warnings do not fall further. The hold is a programme choice (section 7).

---

## 4. Mesh traffic [simulated and measured]

Encoded sizes (18 B clear header + 9 B encrypted type and time + payload + 16 B authentication tag):

| Message | Size | When |
|---|---|---|
| HEARTBEAT (position, fuel, sensor, mode, task) | 89 B | 1 Hz, and immediately on a mode/task change |
| OBSERVATION | 104 B | On each reading, then every 2 s while it is current |
| TASK_CLAIM | 54 B | On a task change, then every 4 s |
| ALERT_ISSUED | 50 B | Once per alert |
| SIGNATURE_UPDATE | 47 B | On dismissal, then every 2 s |
| LEAVE | 43 B | On shutdown |

In the storyline: **2.28 kbit/s mean and 3.13 kbit/s in the busiest second, offered per drone** [simulated]. Over 200 trials the mean is 2.22 kbit/s and the busiest second reaches 4.93 kbit/s at p95. Every transmission is a broadcast, so a shared channel carries the sum: 6 × 2.2 ≈ 13 kbit/s mean, and at most 6 × 4.9 ≈ 30 kbit/s if every drone hit its busiest second at once [computed]. That is about 1.3 % and 3 % of a modest 1 Mbit/s mesh radio. In the 16-drone pack the mean is 2.43 kbit/s per drone, so 16 drones put about 39 kbit/s on a shared channel [computed]: load per drone grows only slowly with pack size, and the channel load roughly linearly. By comparison a single compressed EO/IR video stream is of the order of megabits per second [estimate], which is why the system shares detections and not video.

Memory: one `PackMeshManager` is about 128 KB of fixed storage, allocated once; half of it is a second event list that the re-clustering builds into. No heap allocation occurs after start-up (verified by a test that replaces global `operator new`).

---

## 5. Failure modes and safeguards

| Situation | What the design does | Verified? |
|---|---|---|
| Corrupt or malformed frame | Poly1305 tag checked before anything is parsed; then framing, enum and range checks; NaN or out-of-range values rejected and counted | Yes: corrupting any one byte of a frame, any truncation, NaN and out-of-range fields are all rejected |
| Packet loss | Sequence gaps estimate link quality; own readings re-sent every 2 s | Yes: 30 % loss converges in 25 s |
| Peer lost | Dropped after 5 s; its verification slot freed and re-assigned | Yes, unit test and scenario |
| Drone leaves the mesh deliberately | `LEAVE` removes it at once | Yes |
| GMCS takes a drone into `ACTIVE_MISSION` | Mesh task cancelled at once; peers re-plan | Yes |
| Low fuel | Excluded by the fuel rule; it still contributes readings | Yes |
| Alert owner has no link, or the send fails | Retried each cycle; the next-ranked drone takes over after 3 s | Yes |
| `ALERT_ISSUED` notice lost in a radio fade | Each drone repeats its notices for 20 s, so backups hear it on a repeat and stand down | Yes: unit test; Monte Carlo duplicate alerts 0.02 per trial in the storyline |
| Drones' tables differ for a moment, so two think they are the verifier | Claim before flying: the FCS is asked only once the claim has held for 1.5 s, and the loser gives way before it turns | Yes: unit test; when the fix went in, FCS requests by the busiest drone fell from 85 to 19 (median) in the 16-drone run |
| DF bearing in a multipath reflection | Consensus over bearing pairs sets outliers aside; a bearing-only ADVISORY needs 3 bearings held for 10 s; the FCS is re-pointed only for a move that lasts 8 s | Yes: Monte Carlo, see 3.1 and 3.2 |
| False imaging detection (clutter) | A drone shares an imaging detection only once a second look from the same sensor confirms it, so one-frame clutter never reaches the pack. PRIORITY also needs bearings that confirm the position on their own | Yes: unit test; 0 false PRIORITY and 0 clutter alerts in 600 Monte Carlo trials. Persistent clutter is not modelled: **open** (section 7) |
| Mesh partition | Each side keeps working from its own table and may alert under its own id. On heal the tables converge on the lowest id, the plans merge to at most two verifiers, losing verifiers are released, and nobody alerts again. A dismissal made on one side reaches the other on heal. The GMCS alert consumer folds the second side's alert into the first record by place | Yes: split, alert, heal, confirm; split during verification; dismissal across a split; the 16-drone scenario splits and heals its pack |
| Two sources on the same frequency | Fused only if their positions or bearings agree; a shared frequency alone is not enough | Yes |
| Packet naming another drone as its source, future timestamp, absurd values | Rejected on receipt and counted, even when sealed by a key holder | Yes |
| Forged frame from outside the pack | Fails authentication (ChaCha20-Poly1305, per-drone keys); counted in `rx_auth_failed` | Yes: an outsider's frames, dismissals and alert notices change nothing |
| Recorded frames replayed later | Refused by the per-sender replay window, or as stale after 5 s | Yes |
| Drone captured or compromised | The GMCS revokes it (its frames are refused and its readings forgotten), then re-keys without it | Revocation yes; re-key distribution is outside this repo |
| Drone reboots | New boot session: its frames are accepted at once, and frames from its old session are refused | Yes |
| Clock stepped backwards (GNSS correction) | Periodic messages keep flowing | Yes |
| Same source filed under two ids | Re-clustering names it by the lowest id its readings carry; every old id kept as an alias | Yes |
| One reading re-sent under a new id (the observer's table renamed its event) | Counted once: a reading's identity ignores the id it was filed under. Counted twice, the copy stood alone as an event with no frequency and captured other frequencies' bearings | Yes: unit test; found in the realistic storyline |
| Readings arrive in a different order on different drones | Re-clustered from scratch each refresh in a canonical order: the grouping and ids depend only on the readings | Yes: all 720 orders of six readings give one grouping |
| One DF hears two sources on one frequency | A second reading from the same sensor that disagrees with the first starts a new event instead of overwriting it | Yes |
| Table full | Lowest-urgency event without a verifier is evicted | Yes |
| FCS refuses a task | Not modelled: the request interface returns nothing; a real interface needs an accept/reject reply | Open |
| **Insider forgery within an epoch** | Symmetric keys: any member can forge another's frames until the next epoch. Needs per-sender signatures to close | **Open** |
| **Validated crypto module** | ChaCha20-Poly1305 is implemented from RFC 8439 and vector-tested, not validated | **Open: required before any real use** |
| **GNSS-denied time** | Timestamps assume a PPS-disciplined clock shared by the pack; if it drifts, observation ageing and ordering degrade | **Open** |

## 6. What this does not do

- No weapons or engagement logic. The mesh detects, localises, verifies and alerts.
- No steering of the flight controls. It sends requests to the FCS, which enforces the approved corridor.
- No claim of flight-validated performance. The simulator models radio latency and fades, terrain masking, DF multipath and imaging clutter, but with generic parameters chosen to be plausible, not measured on the real radio, DF or airframe.
- No relaying. Every mesh frame is one hop, so a pack wider than the radio range splits into groups that each alert on their own.

## 7. Decisions needed from the programme

1. Mesh radio type, capacity and range, to replace the sim's 60 km range and generic latency and fade model; and whether the mesh must relay for packs wider than that range.
2. Key management for the mesh: who generates epoch keys, how they are filled and stored on the aircraft, how the GMCS re-keys over C2, and whether per-sender signatures are needed against a captured drone.
3. The GMCS alert message format, and whether the consumer's workflow suits the operators: acknowledgement deadlines of 30 s for PRIORITY and 2 min for ADVISORY, and merging by place within 2 km or 3 CEP.
4. The advisory hold: 10 s by default (section 3.2). Shorter gives earlier warning and more false ones.
5. Whether one drone's imaging detection of a thermal hazard, confirmed over two looks, may raise an ADVISORY on its own. The simulator's clutter lasts one frame, so the confirmation removes it. Persistent clutter, such as a sun-heated rock or a vehicle exhaust, is not modelled.
6. The FCS's accept/reject reply and the shape of the approved corridor (polygon rather than circle).
7. Whether `ACTIVE_MISSION` drones should share raw readings (assumed yes) or only alerts.
8. Flight-test measurements to set the urgency weights, standoff distance and the DF error and multipath model.
