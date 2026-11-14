// Pack simulation: scenario description, world model and results. Not flight
// code: it uses the standard library freely and exists to exercise the Model
// 8270 mesh logic end to end and to produce measured numbers for the docs.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "amc8270/mesh/pack_mesh_manager.hpp"

namespace sim {

namespace mesh = amc8270::mesh;

// ------------------------------------------------------------ scenario

struct Hill {
  double e = 0, n = 0, height_m = 0, radius_m = 1;  // Gaussian bump
};
struct Ridge {
  double e1 = 0, n1 = 0, e2 = 0, n2 = 0, height_m = 0, width_m = 1;  // Gaussian ridge along a segment
};

// Mesh radio. Loss is a Gilbert-Elliott two-state chain per directed link,
// stepped once per datagram on that link, so losses come in bursts.
struct RadioModel {
  double range_m = 60000;
  double latency_ms = 0;  // fixed part
  double jitter_ms = 0;   // mean of an exponential extra delay
  double loss_good = 0.05;
  double loss_bad = 0.05;
  double p_good_to_bad = 0.0;
  double p_bad_to_good = 1.0;
  bool terrain_masking = false;
  double clock_skew_ms = 0;  // each drone's clock is offset by up to this, either way
};

// Direction finding. Multipath: a drone that enters a reflection keeps seeing
// the source at a fixed false offset for a while (a persistent ghost).
struct DfModel {
  double range_m = 35000;
  double sigma_deg = 2.5;
  double period_s = 2.0;
  double mp_prob = 0;           // chance per reading of entering a reflection
  double mp_bias_deg = 0;       // reflection offset, uniform in +-this
  double mp_sigma_deg = 0;      // extra noise while reflected
  double mp_min_s = 5, mp_max_s = 30;  // how long a reflection lasts
  double mp_low_elev_deg = 0;   // below this elevation angle ...
  double mp_low_elev_factor = 1;  // ... the chance is multiplied by this
};

struct ImagingModel {
  double passive_m = 3000;  // wide-area look while routing
  double cued_m = 10000;    // when the mesh has cued this sensor on a target
  double sigma_m = 45;
  float cep_m = 70;
};

// False imaging detections: a Poisson process per drone with an imager.
struct ClutterModel {
  double per_drone_hour = 0;
  mesh::EventClass cls = mesh::EventClass::THERMAL_HAZARD;
  float conf = 0.8f;
};

// C2 links to the GMCS: one-way latency per link type, plus exponential jitter.
struct C2Model {
  std::array<double, 3> latency_ms{60, 180, 650};  // LOS, LEO, GEO
  double jitter_ms = 30;
  double operator_ack_s = 8;  // operator reaction time to a new GMCS record
};

struct DroneSpec {
  int id = 0;
  mesh::DroneMode mode = mesh::DroneMode::PACK_ROUTING;
  mesh::SensorMask sensors = 0;
  mesh::LinkMask links = 0;
  double endurance_s = 0;
  double bingo_s = 1800;
  double e = 0, n = 0;  // start position = formation slot
};

struct EmitterSpec {
  std::string label;
  std::uint32_t sig = 0;
  double e = 0, n = 0;
  double on_s = 0, off_s = 1e18;
  mesh::EventClass cls = mesh::EventClass::UNKNOWN;
  bool thermal = false;
  double mast_m = 2;
  bool alertable = false;  // derived: severe enough to alert and not pre-briefed benign
};

enum class ScriptKind { LOSE_VERIFIER, LOSE_DRONE, DISMISS, PARTITION, HEAL };
struct ScriptEvent {
  double t = 0;
  ScriptKind kind = ScriptKind::LOSE_DRONE;
  std::string label;            // emitter (LOSE_VERIFIER, DISMISS)
  int drone = 0;                // LOSE_DRONE
  std::vector<int> group_b;     // PARTITION: these drones form the second side
};

// A pass/fail check on a single run. kind: advisory_once, priority_once,
// priority_after_advisory, never_tasked, reassigned_after_loss, quiet,
// dismissal_learned, all_released, max_fcs_requests, advisory_within,
// priority_within, max_false_alerts, converged.
struct Expect {
  std::string kind;
  std::string label;
  int drone = 0;
  double value = 0;
  std::string text;  // the line as written, for the report
};

struct Scenario {
  std::string name;
  std::string path;
  double duration_s = 600;
  std::uint32_t seed = 1;
  mesh::GeoPoint origin{36.0, -12.0, 0.0f};
  double altitude_m = 3000;
  double route_speed = 40, dash_speed = 45, rejoin_speed = 55;
  RadioModel radio;
  DfModel df;
  ImagingModel img;
  ClutterModel clutter;
  C2Model c2;
  std::vector<Hill> hills;
  std::vector<Ridge> ridges;
  std::vector<DroneSpec> drones;
  std::vector<EmitterSpec> emitters;
  std::vector<std::uint32_t> benign;
  std::vector<ScriptEvent> script;
  std::vector<Expect> expects;
  mesh::MeshConfig cfg{};
  // Monte Carlo: each trial reseeds every random process and, if set, moves
  // each emitter by up to mc_jitter_m and its start by up to mc_jitter_on_s.
  int mc_trials = 100;
  double mc_jitter_m = 0;
  double mc_jitter_on_s = 0;
};

// False (with a message naming the file and line) on any error.
bool loadScenario(const std::string& path, Scenario& out, std::string& error);

// ------------------------------------------------------------ results

struct AlertRecord {
  double t_issue = 0;  // drone sent it
  double t_rx = 0;     // GMCS received it
  mesh::GmcsAlert alert{};
  mesh::LinkType link = mesh::LinkType::LOS;
  int emitter = -1;    // the real source within max(2 km, 3 CEP) of it, -1 if none (a false alert)
  double err_m = -1;   // position error against that source
  // Why the event behind it existed, from the true origin of its readings
  // (the sim labels every reading it generates):
  int majority_source = -1;  // the emitter most of its readings came from, -1 if clutter
  int reflected = 0;         // how many of its readings were multipath reflections
  int readings = 0;
};

struct TaskRecord {
  double t = 0;
  int drone = 0;
  mesh::EventId event = mesh::kNoEvent;
  std::uint8_t role = 0;
  mesh::SensorType cue = mesh::SensorType::NONE;
  int emitter = -1;
  double se = 0, sn = 0;  // standoff point
};

struct CheckResult {
  std::string text;
  bool ok = false;
};

struct EmitterOutcome {
  double t_advisory = -1;  // s after the source switched on, -1 if never
  double t_priority = -1;
  double err_priority_m = -1;
};

struct DroneSummary {
  int id = 0;
  bool active = false, tasked = false, alive = true;
  mesh::MeshStats stats{};
  double tx_kbps = 0;
};

struct TrialResult {
  std::uint32_t seed = 0;
  std::vector<DroneSummary> drones;
  std::vector<AlertRecord> alerts;
  std::vector<TaskRecord> tasks;
  std::vector<EmitterOutcome> emitters;
  std::vector<CheckResult> checks;
  int false_advisories = 0;
  int false_priorities = 0;
  int false_from_clutter = 0;    // false alerts whose readings were mostly clutter
  int false_from_multipath = 0;  // ... mostly a real source's readings, some reflected: a mislocalised source
  int duplicate_alerts = 0;  // more than one alert of a level for one source
  std::uint64_t datagrams = 0, datagrams_lost = 0, bytes = 0;
  double mean_kbps_per_drone = 0, peak_kbps_per_drone = 0;
  double hours_flown = 0;
  int max_fcs_requests = 0;
  bool passed() const {
    for (const auto& c : checks) {
      if (!c.ok) return false;
    }
    return true;
  }
};

struct RunOptions {
  std::uint32_t seed = 1;
  bool mc_jitter = false;      // apply the scenario's Monte Carlo jitter
  bool verbose = false;        // print the timeline
  std::string trace_path;      // write a replay trace here if set
  double trace_period_s = 1.0;
};

TrialResult runScenario(const Scenario& sc, const RunOptions& opt);

}  // namespace sim
