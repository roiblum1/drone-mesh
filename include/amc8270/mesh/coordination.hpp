// Distributed prioritisation and verifier selection. Everything here is a
// deterministic function of (table, pack snapshot, time), so every drone that
// holds the same table computes the same plan without a leader.
#pragma once

#include <cstddef>
#include <cstdint>

#include "amc8270/mesh/config.hpp"
#include "amc8270/mesh/situational_awareness.hpp"

namespace amc8270::mesh {

struct Assignment {
  EventId event_id = kNoEvent;
  DroneId drone = kNoDrone;
  std::uint8_t role = 0;  // 0 primary, 1 secondary
  float score = 0.0f;
  GeoPoint target{};
  float target_cep_m = 0.0f;
  GeoPoint standoff{};
  float approach_bearing_deg = 0.0f;
  SensorType sensor_to_cue = SensorType::NONE;
  float urgency = 0.0f;
};
using AssignmentList = StaticVector<Assignment, kMaxPackSize>;

// severity(class) * novelty * (0.3 + 0.7 conf) * (0.5 + 0.5 freshness)
float computeUrgency(const DetectedEvent& ev, bool benign, TimeMs now, const MeshConfig& cfg) noexcept;
void updateUrgencies(SituationalAwarenessTable& table, TimeMs now) noexcept;

// Only drones in PACK_ROUTING (or already VERIFYING) may be tasked.
// ACTIVE_MISSION and RTB drones never are.
bool isTaskable(const DroneState& d) noexcept;

bool eligibleForTasking(const DetectedEvent& ev, TimeMs now, const MeshConfig& cfg) noexcept;

// Event indices sorted by (urgency desc, id asc), skipping DISMISSED events.
std::size_t rankEvents(const SituationalAwarenessTable& table, std::uint8_t* order, std::size_t cap) noexcept;

// Weighted fit of a drone to an event; false if infeasible (fuel, range,
// geofence). Components: ETA, new-sensor-modality, fuel margin, angular diversity.
struct Suitability {
  float score = 0.0f;
  GeoPoint standoff{};
  float approach_bearing_deg = 0.0f;
  float fuel_margin_s = 0.0f;
  SensorType sensor_to_cue = SensorType::NONE;
};
bool computeSuitability(const DroneState& d, const DetectedEvent& ev, const double* covered_azimuths_deg,
                        std::size_t n_covered, bool incumbent, const MeshConfig& cfg,
                        Suitability& out) noexcept;

void planAssignments(const SituationalAwarenessTable& table, const DroneList& drones, TimeMs now,
                     const MeshConfig& cfg, AssignmentList& out) noexcept;

// Write the plan into the events' verifier slots (clears slots not in the plan).
void applyAssignments(SituationalAwarenessTable& table, const AssignmentList& plan, TimeMs now) noexcept;

AlertLevel evaluateAlertLevel(const DetectedEvent& ev, bool benign, const MeshConfig& cfg) noexcept;

// Position of `self` in the ordered list of drones allowed to speak to the
// GMCS for this event (primary, secondary, observers, then any free drone),
// or -1 if `self` may not. Rank r waits r * alert_backup_delay_ms.
int alertOwnerRank(const SituationalAwarenessTable& table, const DetectedEvent& ev, DroneId self) noexcept;

}  // namespace amc8270::mesh
