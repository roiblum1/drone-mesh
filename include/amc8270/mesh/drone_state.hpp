#pragma once

#include "amc8270/mesh/types.hpp"

namespace amc8270::mesh {

// What every peer needs to know about a drone. Carried by HEARTBEAT.
struct DroneState {
  DroneId id = kNoDrone;
  DroneMode mode = DroneMode::PACK_ROUTING;
  SensorType active_sensor = SensorType::NONE;  // Variable Homing Devices bus device currently cued
  SensorMask installed_sensors = 0;
  LinkMask available_links = 0;  // GMCS links currently up (LOS/LEO/GEO)
  GeoPoint position{};
  float speed_mps = 0.0f;
  float heading_deg = 0.0f;
  float endurance_remaining_s = 0.0f;  // from the 7-hour combustion EMU's fuel data, out of kNominalEnduranceS
  float bingo_endurance_s = 0.0f;      // reserve needed to reach recovery
  EventId tasked_event = kNoEvent;     // event being verified, if any
  TimeMs timestamp_ms = 0;
};

}  // namespace amc8270::mesh
