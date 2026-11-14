#include "amc8270/mesh/validation.hpp"

#include <cmath>

namespace amc8270::mesh {

namespace {

bool inRange(float v, float lo, float hi) noexcept { return std::isfinite(v) && v >= lo && v <= hi; }

bool isValidMode(DroneMode m) noexcept { return static_cast<std::uint8_t>(m) <= kDroneModeMax; }

}  // namespace

bool isPlausible(const GeoPoint& p) noexcept {
  return std::isfinite(p.lat_deg) && std::isfinite(p.lon_deg) && p.lat_deg >= -90.0 && p.lat_deg <= 90.0 &&
         p.lon_deg >= -180.0 && p.lon_deg <= 180.0 && inRange(p.alt_m, -kMaxPlausibleAltM, kMaxPlausibleAltM);
}

bool isPlausible(const DroneState& s) noexcept {
  return s.id != kNoDrone && isValidMode(s.mode) && isValidSensor(static_cast<std::uint8_t>(s.active_sensor)) &&
         onlyBits(s.installed_sensors, kSensorMaskAll) && onlyBits(s.available_links, kLinkMaskAll) &&
         isPlausible(s.position) && inRange(s.speed_mps, 0.0f, kMaxPlausibleSpeedMps) &&
         inRange(s.heading_deg, 0.0f, 360.0f) && inRange(s.endurance_remaining_s, 0.0f, kMaxPlausibleEnduranceS) &&
         inRange(s.bingo_endurance_s, 0.0f, kMaxPlausibleEnduranceS);
}

bool isPlausible(const SensorObservation& o) noexcept {
  const auto sensor = static_cast<std::uint8_t>(o.sensor);
  if (o.observer == kNoDrone || sensor == 0 || !isValidSensor(sensor)) return false;
  if (static_cast<std::uint8_t>(o.class_hint) > kEventClassMax) return false;
  if (!isPlausible(o.observer_position) || !inRange(o.confidence, 0.0f, 1.0f)) return false;
  if (o.has_fix) return isPlausible(o.fix) && inRange(o.fix_cep_m, 0.0f, kMaxPlausibleCepM);
  // Bearing-only: a zero sigma would give the line infinite weight in the fusion.
  return inRange(o.bearing_deg, 0.0f, 360.0f) && std::isfinite(o.bearing_sigma_deg) && o.bearing_sigma_deg > 0.0f &&
         o.bearing_sigma_deg <= kMaxPlausibleSigmaDeg;
}

bool isPlausibleClaimScore(float score) noexcept { return inRange(score, 0.0f, kMaxClaimScore); }

bool isValid(const MeshConfig& c) noexcept {
  const bool periods = c.heartbeat_period_ms > 0 && c.peer_timeout_ms > c.heartbeat_period_ms &&
                       c.anti_entropy_period_ms > 0 && c.coordination_period_ms > 0 &&
                       c.observation_max_age_ms > 0 && c.max_antientropy_per_cycle > 0 && c.frame_max_age_ms > 0;
  const bool divisors = c.urgency_decay_ms > 0.0 && c.eta_ref_s > 0.0 && c.fuel_ref_s > 0.0 && c.min_speed_mps > 0.0;
  const bool fractions = c.max_diversion_fraction >= 0.0 && c.max_diversion_fraction <= 1.0 &&
                         c.benign_novelty >= 0.0 && c.benign_novelty <= 1.0 &&
                         c.single_modality_conf_cap > 0.0 && c.single_modality_conf_cap <= 1.0 &&
                         c.multi_modality_conf_cap > 0.0 && c.multi_modality_conf_cap <= 1.0 &&
                         c.unlocalized_conf_cap >= 0.0 && c.unlocalized_conf_cap <= 1.0;
  const bool weights = c.w_eta >= 0.0f && c.w_sensor >= 0.0f && c.w_fuel >= 0.0f && c.w_diversity >= 0.0f &&
                       c.hysteresis_bonus >= 0.0f &&
                       static_cast<double>(c.w_eta + c.w_sensor + c.w_fuel + c.w_diversity + c.hysteresis_bonus) <=
                           static_cast<double>(kMaxClaimScore);
  const bool geometry = c.association_gate_m > 0.0 && c.max_association_range_m > c.provisional_range_m && c.min_geometry_eigen > 0.0 && c.min_bearing_sigma_deg > 0.0 &&
                        c.standoff_m > 0.0 && c.max_detour_m > 0.0 && c.geofence_radius_m >= 0.0 &&
                        c.retask_move_m >= 0.0;
  return periods && divisors && fractions && weights && geometry;
}

}  // namespace amc8270::mesh
