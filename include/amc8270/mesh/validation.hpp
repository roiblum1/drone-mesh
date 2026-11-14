// Plausibility rules for everything that enters the mesh logic, whether it
// arrives over the radio or from this drone's own avionics. One definition,
// used by the packet decoder and by the manager's local inputs.
#pragma once

#include "amc8270/mesh/config.hpp"
#include "amc8270/mesh/detected_event.hpp"
#include "amc8270/mesh/drone_state.hpp"

namespace amc8270::mesh {

inline constexpr float kMaxPlausibleSpeedMps = 300.0f;
inline constexpr float kMaxPlausibleEnduranceS = 2.0f * kNominalEnduranceS;
inline constexpr float kMaxPlausibleSigmaDeg = 90.0f;
inline constexpr float kMaxPlausibleCepM = 1.0e6f;
inline constexpr float kMaxPlausibleAltM = 30000.0f;
inline constexpr float kMaxClaimScore = 2.0f;  // suitability (<= 1) + hysteresis bonus, with margin

[[nodiscard]] bool isPlausible(const GeoPoint& p) noexcept;
[[nodiscard]] bool isPlausible(const DroneState& s) noexcept;
[[nodiscard]] bool isPlausible(const SensorObservation& o) noexcept;
[[nodiscard]] bool isPlausibleClaimScore(float score) noexcept;

// True if every tunable is in a range where the algorithms are well defined
// (no zero periods or divisors, fractions within [0, 1], weights >= 0).
[[nodiscard]] bool isValid(const MeshConfig& cfg) noexcept;

}  // namespace amc8270::mesh
