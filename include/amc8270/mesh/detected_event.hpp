#pragma once

#include <array>

#include "amc8270/mesh/config.hpp"
#include "amc8270/mesh/static_vector.hpp"
#include "amc8270/mesh/types.hpp"

namespace amc8270::mesh {

// One drone's reading of one source. Keyed by (observer, sensor); a newer
// reading replaces the older one.
struct SensorObservation {
  DroneId observer = kNoDrone;
  SensorType sensor = SensorType::NONE;
  EventClass class_hint = EventClass::UNKNOWN;
  bool has_fix = false;           // true: geolocated fix; false: bearing only
  EventId event_hint = kNoEvent;  // event the observer filed this under
  GeoPoint observer_position{};
  float bearing_deg = 0.0f;  // true bearing observer -> source (bearing-only)
  float bearing_sigma_deg = 3.0f;
  GeoPoint fix{};  // source position (has_fix)
  float fix_cep_m = 0.0f;
  float confidence = 0.0f;      // sensor-level, 0..1
  std::uint32_t signature = 0;  // e.g. RF frequency in kHz; 0 = none
  TimeMs timestamp_ms = 0;
};

// A drone holding (or granted) one of an event's two verification slots.
struct VerifierSlot {
  DroneId id = kNoDrone;
  float score = 0.0f;
  TimeMs since_ms = 0;
};

// Fused picture of one source, identical on every drone once tables converge.
struct DetectedEvent {
  EventId id = kNoEvent;
  EventClass cls = EventClass::UNKNOWN;
  EventStatus status = EventStatus::CANDIDATE;
  std::uint32_t signature = 0;

  GeoPoint position{};
  bool localized = false;
  float cep_m = 0.0f;  // 50% circular error probable
  float confidence = 0.0f;
  float urgency = 0.0f;
  SensorMask modality_mask = 0;
  std::uint8_t observer_count = 0;

  StaticVector<SensorObservation, kMaxObsPerEvent> observations{};
  std::array<VerifierSlot, 2> verifier{};  // [0] primary, [1] secondary

  bool advisory_sent = false;
  bool priority_sent = false;
  TimeMs advisory_ready_ms = 0;
  TimeMs priority_ready_ms = 0;

  TimeMs first_seen_ms = 0;
  TimeMs last_update_ms = 0;
  TimeMs verify_started_ms = 0;
};

// What the mesh asks the FCS to fly. The FCS keeps authority and may refuse.
struct VerificationTask {
  EventId event_id = kNoEvent;
  std::uint8_t role = 0;  // 0 primary, 1 secondary
  GeoPoint target{};
  float target_cep_m = 0.0f;
  GeoPoint standoff{};           // where to be
  float approach_bearing_deg = 0.0f;  // bearing from target to standoff
  SensorType sensor_to_cue = SensorType::NONE;
  float urgency = 0.0f;
};

}  // namespace amc8270::mesh
