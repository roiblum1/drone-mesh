// Compile-time capacities (no heap after init) and runtime tunables.
#pragma once

#include <cstddef>
#include <cstdint>

#include "amc8270/mesh/types.hpp"

namespace amc8270::mesh {

inline constexpr std::size_t kMaxPackSize = 16;
inline constexpr std::size_t kMaxEvents = 64;
inline constexpr std::size_t kMaxObsPerEvent = 8;
inline constexpr std::size_t kMaxKnownSignatures = 32;
inline constexpr std::size_t kMaxAliases = 32;
inline constexpr std::size_t kMaxPacketBytes = 160;
inline constexpr std::size_t kMaxRxPerTick = 32;

inline constexpr float kNominalEnduranceS = 7.0f * 3600.0f;  // 7-hour combustion EMU nominal endurance

// Capacity relationships the code relies on.
static_assert(kMaxEvents <= 256, "event order arrays index events with std::uint8_t");
static_assert(kMaxObsPerEvent <= 255, "DetectedEvent::observer_count is std::uint8_t");
static_assert(kMaxPackSize >= 2 && kMaxPackSize <= 0xFFFF, "a pack needs >= 2 drones; DroneId is 16-bit");
static_assert(kMaxPacketBytes <= 0xFFFF, "the payload length field is 16-bit");

// Tunables. Defaults are illustrative; flight values come from the mission
// data load and must be validated in flight test.
struct MeshConfig {
  // ---- timing
  std::uint32_t heartbeat_period_ms = 1000;
  std::uint32_t peer_timeout_ms = 5000;
  std::uint32_t anti_entropy_period_ms = 2000;
  std::uint32_t coordination_period_ms = 500;
  std::uint32_t observation_max_age_ms = 90000;
  std::uint32_t verify_timeout_ms = 600000;
  std::uint32_t slot_adopt_grace_ms = 2500;
  // Claim before flying: a drone the plan picks claims its slot at once but asks
  // the FCS to divert only once it has held the slot this long. Drones with
  // slightly different tables otherwise both divert, and the loser of the claim
  // arbitration turns back half a second later. Must be below slot_adopt_grace_ms.
  std::uint32_t claim_confirm_ms = 1500;
  std::uint32_t alert_backup_delay_ms = 3000;
  // A drone repeats ALERT_ISSUED for its own alerts in each anti-entropy round
  // for this long, so a backup drone that missed the first notice in a radio
  // fade still stands down instead of alerting again.
  std::uint32_t alert_notice_repeat_ms = 20000;
  std::uint32_t max_antientropy_per_cycle = 16;
  // Readings or states stamped further than this in the future are rejected:
  // a far-future stamp would otherwise never age out and could never be replaced.
  std::uint32_t max_clock_skew_ms = 2000;

  // ---- mesh security (mesh_security.hpp)
  // An authentic frame sent longer ago than this is refused. It bounds how long
  // a recorded frame could be replayed to a drone that holds no replay-window
  // state for its sender yet, e.g. just after it boots.
  std::uint32_t frame_max_age_ms = 5000;
  // A key epoch is accepted this long before it starts and after it ends, so
  // frames in flight across a re-key are not lost.
  std::uint32_t key_epoch_grace_ms = 10000;

  // ---- association and fusion
  // Fusion works in a flat local frame, which holds at pack scale. A reading
  // from further than this from an event (observer for a bearing, fix for a
  // fix) is never associated with it, and no fused position further than
  // this from its readings is reported.
  double max_association_range_m = 100000.0;
  // A reading from a sensor that disagrees with a newer one from the same
  // sensor (and frequency) is kept as a second source only while the sensor
  // keeps reporting it; once it is this much older than the newest, it is a
  // stale reflection and is dropped. A DF that tracks two sources on one
  // frequency reports both every scan, so both stay fresh.
  std::uint32_t same_sensor_supersede_ms = 10000;
  double association_gate_m = 500.0;
  double association_cep_factor = 1.5;
  double bearing_gate_deg = 2.0;       // added to 3*sigma when gating a bearing
  double min_geometry_eigen = 0.03;    // ~14 deg crossing angle for two lines
  double min_bearing_sigma_deg = 0.5;
  double provisional_range_m = 15000.0;  // assumed range for a lone bearing
  double provisional_cep_m = 8000.0;
  double single_modality_conf_cap = 0.85;
  double multi_modality_conf_cap = 0.99;
  double unlocalized_conf_cap = 0.5;

  // ---- urgency and tasking
  double benign_novelty = 0.2;
  // A second event on a frequency that a better-supported event already
  // localises is more often multipath than a second source: its novelty is
  // scaled by this, so it waits behind the first for a verifier.
  double secondary_source_novelty = 0.5;
  double urgency_decay_ms = 60000.0;
  float min_task_urgency = 0.25f;
  float dual_verifier_urgency = 0.6f;
  double max_diversion_fraction = 0.4;  // of drones free to be tasked
  // Added to the current verifier's score. Switching a verifier mid-transit throws
  // away minutes of flight, so a newcomer must be clearly better (0.15 let a single
  // dropped bearing hand the task over in the realistic simulation).
  float hysteresis_bonus = 0.30f;
  // Re-issue a task to the FCS only when the target estimate has really moved:
  //  - by more than max(retask_move_m, retask_cep_factor * hypot(CEP then, CEP
  //    now)). Two noisy estimates of one point differ by their combined error,
  //    so this ignores noise; or
  //  - once the estimate has at least halved its CEP, by more than
  //    max(retask_move_m, retask_cep_factor * CEP now): a much better fix (an
  //    imager's) that puts the old aim point outside its own error re-points
  //    the FCS. The task then carries the better CEP, so this happens a few
  //    times at most.
  double retask_move_m = 500.0;
  double retask_cep_factor = 3.0;
  // ... and only once the move has lasted this long. A DF reflection can hold the
  // estimate off by kilometres for 5-30 s, and the FCS must not chase that (Monte
  // Carlo, realistic storyline: 3 s -> 16 requests for the busiest verifier, 8 s
  // -> 13, 15 s -> 11, with the same confirmation rate).
  std::uint32_t retask_confirm_ms = 8000;

  // suitability weights (sum to 1)
  float w_eta = 0.40f;
  float w_sensor = 0.25f;
  float w_fuel = 0.15f;
  float w_diversity = 0.20f;
  double eta_ref_s = 1200.0;
  double fuel_ref_s = 3600.0;
  double verify_dwell_s = 300.0;
  double min_speed_mps = 20.0;
  double standoff_m = 6000.0;
  double max_detour_m = 60000.0;

  // GMCS-approved corridor. Circular geofence; 0 radius disables the check.
  GeoPoint geofence_center{};
  double geofence_radius_m = 0.0;

  // ---- alerting
  double advisory_min_severity = 0.7;
  double advisory_min_conf = 0.6;
  double advisory_max_cep_m = 2000.0;
  // A position from bearings alone needs this many before an ADVISORY. Two lines
  // always cross somewhere, so two reflected bearings make a convincing ghost;
  // a third gives the fusion something to disagree with. A fix lifts the rule.
  unsigned advisory_min_bearings = 3;
  // ... and must have held for this long. A reflection lasts seconds; a source
  // stays put. 0 disables the hold (see the Monte Carlo trade-off in deliverable 2).
  std::uint32_t advisory_bearing_hold_ms = 10000;
  double priority_min_conf = 0.9;
  double priority_max_cep_m = 250.0;
  unsigned priority_min_observers = 2;
  unsigned priority_min_modalities = 2;
  // When bearings are one of those sensor types, this many must triangulate on
  // their own to the fused position (see corroboratingModalities()).
  unsigned priority_min_bearings = 3;
};

}  // namespace amc8270::mesh
