// Hardware abstraction for the Model 8270. The mesh logic never touches a
// device directly, so it runs unchanged on the target RTOS, on a POSIX
// host and in the simulator.
#pragma once

#include <cstddef>
#include <cstdint>

#include "amc8270/mesh/detected_event.hpp"
#include "amc8270/mesh/types.hpp"

namespace amc8270::mesh {

class IClock {
 public:
  virtual ~IClock() = default;
  virtual TimeMs nowMs() const = 0;

 protected:
  IClock() = default;
  IClock(const IClock&) = default;
  IClock& operator=(const IClock&) = default;
  IClock(IClock&&) = default;
  IClock& operator=(IClock&&) = default;
};

// Inter-drone mesh transceiver (UDP multicast over the GbE mesh radio).
class IMeshTransceiver {
 public:
  virtual ~IMeshTransceiver() = default;
  virtual bool send(const std::uint8_t* data, std::size_t len) = 0;
  // Copies one pending datagram into buf; returns its length, 0 if none.
  virtual std::size_t receive(std::uint8_t* buf, std::size_t cap) = 0;

 protected:
  IMeshTransceiver() = default;
  IMeshTransceiver(const IMeshTransceiver&) = default;
  IMeshTransceiver& operator=(const IMeshTransceiver&) = default;
  IMeshTransceiver(IMeshTransceiver&&) = default;
  IMeshTransceiver& operator=(IMeshTransceiver&&) = default;
};

// Fused, verified alert as the GMCS sees it. Deduplicated by event_id.
struct GmcsAlert {
  EventId event_id = kNoEvent;
  AlertLevel level = AlertLevel::NONE;
  EventClass cls = EventClass::UNKNOWN;
  GeoPoint position{};
  float cep_m = 0.0f;
  float confidence = 0.0f;
  SensorMask modality_mask = 0;
  std::uint8_t observer_count = 0;
  DroneId issuer = kNoDrone;
  DroneId primary_verifier = kNoDrone;
  TimeMs timestamp_ms = 0;
};

// C2 link manager (LOS / LEO SATCOM / GEO SATCOM). Returns false if the
// message could not be queued on the requested link.
class IGmcsLink {
 public:
  virtual ~IGmcsLink() = default;
  virtual bool sendAlert(const GmcsAlert& alert, LinkType link) = 0;

 protected:
  IGmcsLink() = default;
  IGmcsLink(const IGmcsLink&) = default;
  IGmcsLink& operator=(const IGmcsLink&) = default;
  IGmcsLink(IGmcsLink&&) = default;
  IGmcsLink& operator=(IGmcsLink&&) = default;
};

// Guidance requests to the Flight Control System. Requests only: the FCS
// keeps authority and enforces the approved corridor.
class IGuidanceSink {
 public:
  virtual ~IGuidanceSink() = default;
  virtual void requestVerification(const VerificationTask& task) = 0;
  virtual void cancelVerification() = 0;  // revert to planned route

 protected:
  IGuidanceSink() = default;
  IGuidanceSink(const IGuidanceSink&) = default;
  IGuidanceSink& operator=(const IGuidanceSink&) = default;
  IGuidanceSink(IGuidanceSink&&) = default;
  IGuidanceSink& operator=(IGuidanceSink&&) = default;
};

}  // namespace amc8270::mesh
