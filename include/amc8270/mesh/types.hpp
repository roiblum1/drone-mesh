// Basic vocabulary types shared by every mesh module on the Model 8270.
#pragma once

#include <cstdint>

namespace amc8270::mesh {

using DroneId = std::uint16_t;  // 1..N within a pack, 0 = none
using EventId = std::uint32_t;  // (origin DroneId << 16) | counter, 0 = none
using TimeMs = std::uint64_t;   // UTC milliseconds, PPS/GNSS disciplined

inline constexpr DroneId kNoDrone = 0;
inline constexpr EventId kNoEvent = 0;

// PACK_ROUTING and ACTIVE_MISSION are the two GMCS-commanded modes of the
// operational concept. VERIFYING is the mesh's own sub-state of PACK_ROUTING
// (drone has accepted a verification task); RTB is the recovery mode.
enum class DroneMode : std::uint8_t {
  PACK_ROUTING = 0,
  ACTIVE_MISSION = 1,
  VERIFYING = 2,
  RTB = 3,
};
inline constexpr std::uint8_t kDroneModeMax = 3;

enum class SensorType : std::uint8_t {
  NONE = 0,
  RF_DF = 1,    // RF signal / beacon direction finding (bearing-only)
  EO = 2,       // electro-optical imaging (geolocated fix)
  IR = 4,       // thermal imaging (geolocated fix)
  ANOMALY = 8,  // specialised anomaly detector (geolocated fix)
};
using SensorMask = std::uint8_t;
inline constexpr SensorMask kSensorMaskAll = 0x0FU;

constexpr SensorMask maskOf(SensorType s) noexcept { return static_cast<SensorMask>(s); }

constexpr unsigned popCount(std::uint8_t m) noexcept {
  unsigned bits = m;
  unsigned c = 0;
  while (bits != 0U) {
    c += bits & 1U;
    bits >>= 1U;
  }
  return c;
}

// True if v has no bits outside `allowed`.
constexpr bool onlyBits(std::uint8_t v, std::uint8_t allowed) noexcept {
  return (static_cast<unsigned>(v) & ~static_cast<unsigned>(allowed)) == 0U;
}

constexpr bool isValidSensor(std::uint8_t v) noexcept {
  return v == 0 || v == 1 || v == 2 || v == 4 || v == 8;
}

// C2 data links to the GMCS, in order of preference (lowest latency first).
enum class LinkType : std::uint8_t { LOS = 1, LEO = 2, GEO = 4 };
using LinkMask = std::uint8_t;
inline constexpr LinkMask kLinkMaskAll = 0x07U;

enum class EventClass : std::uint8_t {
  UNKNOWN = 0,
  RF_EMITTER = 1,
  ANOMALY = 2,
  THERMAL_HAZARD = 3,
  DISTRESS_BEACON = 4,
};
inline constexpr std::uint8_t kEventClassMax = 4;

enum class EventStatus : std::uint8_t {
  CANDIDATE = 0,  // seen, not yet localised
  LOCALIZED = 1,  // position fused
  VERIFYING = 2,  // at least one drone tasked to verify
  CONFIRMED = 3,  // PRIORITY alert issued
  DISMISSED = 4,  // GMCS declared benign
};

enum class AlertLevel : std::uint8_t { NONE = 0, ADVISORY = 1, PRIORITY = 2 };
inline constexpr std::uint8_t kAlertLevelMax = 2;

// Base urgency weight of an event class (0..1).
constexpr float eventSeverity(EventClass c) noexcept {
  switch (c) {
    case EventClass::DISTRESS_BEACON: return 1.0f;
    case EventClass::THERMAL_HAZARD: return 0.9f;
    case EventClass::ANOMALY: return 0.7f;
    case EventClass::RF_EMITTER: return 0.5f;
    case EventClass::UNKNOWN: return 0.4f;
  }
  return 0.4f;
}

struct GeoPoint {
  double lat_deg = 0.0;  // WGS-84
  double lon_deg = 0.0;
  float alt_m = 0.0f;  // above ellipsoid
};

}  // namespace amc8270::mesh
