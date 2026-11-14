#include "amc8270/mesh/coordination.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "amc8270/mesh/geo.hpp"

namespace amc8270::mesh {

namespace {

constexpr std::size_t kApproachSteps = 12;  // candidate approach bearings every 30 deg
constexpr double kApproachStepDeg = 30.0;
constexpr double kApproachTieDeg = 15.0;  // prefer the candidate nearest the drone within this of the best
constexpr double kDiversityRefDeg = 90.0;

float clamp01(double v) noexcept { return static_cast<float>(std::min(1.0, std::max(0.0, v))); }

SensorType chooseSensor(const DroneState& d, const DetectedEvent& ev) noexcept {
  // Prefer a modality the event does not have yet, imaging first.
  static constexpr SensorType kPreferred[] = {SensorType::IR, SensorType::EO, SensorType::ANOMALY, SensorType::RF_DF};
  for (SensorType s : kPreferred) {
    if ((d.installed_sensors & maskOf(s)) != 0 && (ev.modality_mask & maskOf(s)) == 0) return s;
  }
  for (SensorType s : kPreferred) {
    if ((d.installed_sensors & maskOf(s)) != 0) return s;
  }
  return SensorType::NONE;
}

}  // namespace

float computeUrgency(const DetectedEvent& ev, bool benign, TimeMs now, const MeshConfig& cfg) noexcept {
  const double sev = static_cast<double>(eventSeverity(ev.cls));
  const double novelty = benign ? cfg.benign_novelty : 1.0;
  const double conf = static_cast<double>(clamp01(ev.confidence));
  const double age = static_cast<double>(elapsedMs(now, ev.last_update_ms));
  const double fresh = static_cast<double>(clamp01(1.0 - age / cfg.urgency_decay_ms));
  return static_cast<float>(sev * novelty * (0.3 + 0.7 * conf) * (0.5 + 0.5 * fresh));
}

void updateUrgencies(SituationalAwarenessTable& table, TimeMs now) noexcept {
  const MeshConfig& cfg = table.config();
  for (auto& ev : table.mutableEvents()) {
    ev.urgency = computeUrgency(ev, table.isBenign(ev.signature), now, cfg);
  }
}

bool isTaskable(const DroneState& d) noexcept {
  return (d.mode == DroneMode::PACK_ROUTING || d.mode == DroneMode::VERIFYING) && d.installed_sensors != 0;
}

bool eligibleForTasking(const DetectedEvent& ev, TimeMs now, const MeshConfig& cfg) noexcept {
  if (ev.status == EventStatus::DISMISSED || ev.status == EventStatus::CONFIRMED || ev.priority_sent) return false;
  if (ev.urgency < cfg.min_task_urgency) return false;
  if (ev.verify_started_ms != 0 && elapsedMs(now, ev.verify_started_ms) > cfg.verify_timeout_ms) return false;
  return true;
}

std::size_t rankEvents(const SituationalAwarenessTable& table, std::uint8_t* order, std::size_t cap) noexcept {
  const auto& events = table.events();
  std::size_t n = 0;
  for (std::size_t i = 0; i < events.size() && n < cap; ++i) {
    if (events[i].status == EventStatus::DISMISSED) continue;
    // insertion sort: urgency desc, id asc
    std::size_t j = n++;
    while (j > 0) {
      const DetectedEvent& prev = events[order[j - 1]];
      const DetectedEvent& cur = events[i];
      const bool before = cur.urgency > prev.urgency || (cur.urgency == prev.urgency && cur.id < prev.id);
      if (!before) break;
      order[j] = order[j - 1];
      --j;
    }
    order[j] = static_cast<std::uint8_t>(i);
  }
  return n;
}

bool computeSuitability(const DroneState& d, const DetectedEvent& ev, const double* covered, std::size_t n_covered,
                        bool incumbent, const MeshConfig& cfg, Suitability& out) noexcept {
  const double az_now = bearingDeg(ev.position, d.position);

  // Approach bearing: maximise separation from angles already covered; among
  // near-best candidates take the one closest to where the drone already is.
  std::array<double, kApproachSteps> sep{};
  double best_sep = -1.0;
  for (std::size_t k = 0; k < kApproachSteps; ++k) {
    const double c = kApproachStepDeg * static_cast<double>(k);
    double s = 180.0;
    for (std::size_t i = 0; i < n_covered; ++i) s = std::min(s, angleDiffDeg(c, covered[i]));
    sep[k] = s;
    best_sep = std::max(best_sep, s);
  }
  std::size_t pick = 0;
  double pick_dist = 1.0e9;
  for (std::size_t k = 0; k < kApproachSteps; ++k) {
    if (sep[k] < best_sep - kApproachTieDeg) continue;
    const double dist = angleDiffDeg(kApproachStepDeg * static_cast<double>(k), az_now);
    if (dist < pick_dist) {
      pick_dist = dist;
      pick = k;
    }
  }
  const double approach = kApproachStepDeg * static_cast<double>(pick);

  GeoPoint standoff = destinationPoint(ev.position, approach, cfg.standoff_m);
  standoff.alt_m = d.position.alt_m;
  if (cfg.geofence_radius_m > 0.0 && distanceM(standoff, cfg.geofence_center) > cfg.geofence_radius_m) return false;

  const double dist_so = distanceM(d.position, standoff);
  if (dist_so > cfg.max_detour_m) return false;

  const double speed = std::max(static_cast<double>(d.speed_mps), cfg.min_speed_mps);
  const double eta_s = dist_so / speed;
  const double detour_s = 2.0 * eta_s + cfg.verify_dwell_s;  // there, dwell, and back to route
  const double margin_s = static_cast<double>(d.endurance_remaining_s) - static_cast<double>(d.bingo_endurance_s) - detour_s;
  if (margin_s < 0.0) return false;

  const SensorType cue = chooseSensor(d, ev);
  const bool new_modality = !onlyBits(d.installed_sensors, ev.modality_mask);
  double diversity = 1.0;
  if (n_covered > 0) {
    double m = 180.0;
    for (std::size_t i = 0; i < n_covered; ++i) m = std::min(m, angleDiffDeg(az_now, covered[i]));
    diversity = static_cast<double>(clamp01(m / kDiversityRefDeg));
  }

  const double eta_score = 1.0 - static_cast<double>(clamp01(eta_s / cfg.eta_ref_s));
  const double sensor_score = new_modality ? 1.0 : 0.3;
  const double fuel_score = static_cast<double>(clamp01(margin_s / cfg.fuel_ref_s));
  double score = static_cast<double>(cfg.w_eta) * eta_score + static_cast<double>(cfg.w_sensor) * sensor_score +
                 static_cast<double>(cfg.w_fuel) * fuel_score + static_cast<double>(cfg.w_diversity) * diversity;
  if (incumbent) score += static_cast<double>(cfg.hysteresis_bonus);

  out.score = static_cast<float>(score);
  out.standoff = standoff;
  out.approach_bearing_deg = static_cast<float>(approach);
  out.fuel_margin_s = static_cast<float>(margin_s);
  out.sensor_to_cue = cue;
  return true;
}

void planAssignments(const SituationalAwarenessTable& table, const DroneList& drones, TimeMs now,
                     const MeshConfig& cfg, AssignmentList& out) noexcept {
  out.clear();

  std::size_t pool = 0;
  for (const auto& d : drones) pool += isTaskable(d) ? 1 : 0;
  if (pool == 0) return;
  // Keep most of the pack scanning: divert at most a fraction of it.
  const std::size_t budget =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(static_cast<double>(pool) * cfg.max_diversion_fraction)));

  std::array<std::uint8_t, kMaxEvents> order{};
  const std::size_t n_events = rankEvents(table, order.data(), order.size());
  const auto& events = table.events();

  std::array<bool, kMaxPackSize> taken{};
  std::size_t used = 0;

  for (std::size_t k = 0; k < n_events && used < budget; ++k) {
    const DetectedEvent& ev = events[order[k]];
    if (!eligibleForTasking(ev, now, cfg)) continue;

    const std::size_t want = std::min<std::size_t>(ev.urgency >= cfg.dual_verifier_urgency ? 2 : 1, budget - used);
    std::array<double, 2> chosen{};
    std::size_t n_chosen = 0;

    for (std::size_t slot = 0; slot < want; ++slot) {
      int best = -1;
      Suitability best_s;
      for (std::size_t di = 0; di < drones.size(); ++di) {
        const DroneState& d = drones[di];
        if (taken[di] || !isTaskable(d)) continue;

        std::array<double, kMaxObsPerEvent + 2> cov{};
        std::size_t nc = 0;
        for (const auto& o : ev.observations) {
          if (o.observer != d.id) cov[nc++] = bearingDeg(ev.position, o.observer_position);
        }
        for (std::size_t c = 0; c < n_chosen; ++c) cov[nc++] = chosen[c];

        const bool incumbent = ev.verifier[0].id == d.id || ev.verifier[1].id == d.id;
        Suitability s;
        if (!computeSuitability(d, ev, cov.data(), nc, incumbent, cfg, s)) continue;
        if (best < 0 || s.score > best_s.score + 1.0e-6f) {  // ties keep the lower drone id
          best = static_cast<int>(di);
          best_s = s;
        }
      }
      if (best < 0) break;

      taken[static_cast<std::size_t>(best)] = true;
      ++used;
      chosen[n_chosen++] = static_cast<double>(best_s.approach_bearing_deg);

      Assignment a;
      a.event_id = ev.id;
      a.drone = drones[static_cast<std::size_t>(best)].id;
      a.role = static_cast<std::uint8_t>(slot);
      a.score = best_s.score;
      a.target = ev.position;
      a.target_cep_m = ev.cep_m;
      a.standoff = best_s.standoff;
      a.approach_bearing_deg = best_s.approach_bearing_deg;
      a.sensor_to_cue = best_s.sensor_to_cue;
      a.urgency = ev.urgency;
      (void)out.push_back(a);  // cannot fail: one assignment per drone, out holds kMaxPackSize
    }
  }
}

void applyAssignments(SituationalAwarenessTable& table, const AssignmentList& plan, TimeMs now) noexcept {
  for (auto& ev : table.mutableEvents()) {
    std::array<const Assignment*, 2> mine{nullptr, nullptr};
    for (const auto& a : plan) {
      if (a.event_id == ev.id && a.role < 2) mine[a.role] = &a;
    }
    bool any = false;
    for (std::size_t s = 0; s < 2; ++s) {
      if (mine[s] == nullptr) {
        ev.verifier[s] = VerifierSlot{};
        continue;
      }
      any = true;
      if (ev.verifier[s].id == mine[s]->drone) {
        ev.verifier[s].score = mine[s]->score;
      } else {
        ev.verifier[s] = VerifierSlot{mine[s]->drone, mine[s]->score, now};
      }
    }
    // The verification clock runs per tenure: it restarts when every current
    // verifier is new (e.g. after the previous one was lost), and stops
    // counting nothing once no drone holds a slot.
    if (any) {
      TimeMs earliest = now;
      for (const auto& slot : ev.verifier) {
        if (slot.id != kNoDrone && slot.since_ms < earliest) earliest = slot.since_ms;
      }
      ev.verify_started_ms = earliest;
    }
  }
}

AlertLevel evaluateAlertLevel(const DetectedEvent& ev, bool benign, const MeshConfig& cfg) noexcept {
  if (benign || ev.status == EventStatus::DISMISSED || !ev.localized) return AlertLevel::NONE;
  const double sev = static_cast<double>(eventSeverity(ev.cls));
  if (sev < cfg.advisory_min_severity) return AlertLevel::NONE;
  const double conf = static_cast<double>(ev.confidence);
  const double cep = static_cast<double>(ev.cep_m);

  if (conf >= cfg.priority_min_conf && cep <= cfg.priority_max_cep_m &&
      ev.observer_count >= cfg.priority_min_observers && popCount(ev.modality_mask) >= cfg.priority_min_modalities) {
    return AlertLevel::PRIORITY;
  }
  if (conf >= cfg.advisory_min_conf && cep <= cfg.advisory_max_cep_m) return AlertLevel::ADVISORY;
  return AlertLevel::NONE;
}

int alertOwnerRank(const SituationalAwarenessTable& table, const DetectedEvent& ev, DroneId self) noexcept {
  StaticVector<DroneId, 2 + kMaxObsPerEvent + kMaxPackSize> order;
  const auto add = [&order](DroneId id) {
    if (id == kNoDrone) return;
    for (const DroneId e : order) {
      if (e == id) return;
    }
    (void)order.push_back(id);  // cannot fail: capacity is 2 slots + all observers + all drones
  };
  add(ev.verifier[0].id);
  add(ev.verifier[1].id);
  for (const auto& o : ev.observations) add(o.observer);
  DroneList drones;
  table.snapshotDrones(drones);
  for (const auto& d : drones) add(d.id);

  int rank = 0;
  for (const DroneId id : order) {
    const DroneState* d = table.findDrone(id);
    if (d == nullptr || d->available_links == 0) continue;
    if (d->mode != DroneMode::PACK_ROUTING && d->mode != DroneMode::VERIFYING) continue;
    if (id == self) return rank;
    ++rank;
  }
  return -1;
}

}  // namespace amc8270::mesh
