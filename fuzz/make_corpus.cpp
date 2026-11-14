// Writes the seed corpus for fuzz_packet (record format: see fuzz_packet.cpp).
// Seeds are valid scenarios, so the fuzzer starts from inputs that reach deep
// code (triangulation, fusion, planning, alerting, replay and reboot handling)
// and mutates from there. Usage: make_corpus <output dir>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "amc8270/mesh/geo.hpp"
#include "amc8270/mesh/mesh_packet.hpp"
#include "amc8270/mesh/mesh_security.hpp"

using namespace amc8270::mesh;

namespace {

constexpr TimeMs kT0 = 1'700'000'000'000ULL;
const LocalFrame kFrame(GeoPoint{36.0, -12.0, 0.0f});

GeoPoint enu(double e, double n, float alt = 3000.0f) { return kFrame.fromEnu(Vec2{e, n}, alt); }

struct Seed {
  std::vector<std::uint8_t> bytes;

  // ctl: sender index 0..7 (drone 2..9), replay, reboot, clock ticks before delivery.
  void record(unsigned sender, MeshPacket pkt, unsigned ticks = 0, bool replay = false, bool reboot = false) {
    std::uint8_t body[kMaxBodyBytes];
    pkt.header.time_ms = kT0;
    const std::size_t n = encodeBody(pkt, body, sizeof body);
    if (n == 0) std::abort();
    const auto ctl = static_cast<std::uint8_t>((sender & 7U) | (replay ? 0x08U : 0U) | (reboot ? 0x10U : 0U) |
                                               ((ticks & 7U) << 5U));
    bytes.push_back(ctl);
    bytes.push_back(static_cast<std::uint8_t>(n));
    bytes.insert(bytes.end(), body, body + n);
  }
  void replay(unsigned sender, unsigned ticks = 0) {
    bytes.push_back(static_cast<std::uint8_t>((sender & 7U) | 0x08U | ((ticks & 7U) << 5U)));
    bytes.push_back(0);
  }
  void write(const std::string& dir, const std::string& name) const {
    std::ofstream out(dir + "/" + name, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  }
};

DroneId idOf(unsigned sender) { return static_cast<DroneId>(2 + sender); }

MeshPacket heartbeat(unsigned sender, double e, double n, DroneMode mode = DroneMode::PACK_ROUTING) {
  DroneState s;
  s.id = idOf(sender);
  s.mode = mode;
  s.active_sensor = SensorType::RF_DF;
  s.installed_sensors = maskOf(SensorType::RF_DF) | maskOf(SensorType::IR) | maskOf(SensorType::EO);
  s.available_links = kLinkMaskAll;
  s.position = enu(e, n);
  s.speed_mps = 40.0f;
  s.heading_deg = 90.0f;
  s.endurance_remaining_s = 4.0f * 3600.0f;
  s.bingo_endurance_s = 1800.0f;
  s.timestamp_ms = kT0;
  MeshPacket p;
  p.payload = HeartbeatPayload{s};
  return p;
}

MeshPacket bearing(unsigned sender, double e, double n, double te, double tn, std::uint32_t sig = 406025,
                   EventId hint = kNoEvent) {
  SensorObservation o;
  o.observer = idOf(sender);
  o.sensor = SensorType::RF_DF;
  o.class_hint = EventClass::DISTRESS_BEACON;
  o.observer_position = enu(e, n);
  o.bearing_deg = static_cast<float>(bearingDeg(o.observer_position, enu(te, tn, 0.0f)));
  o.bearing_sigma_deg = 2.5f;
  o.confidence = 0.5f;
  o.signature = sig;
  o.event_hint = hint != kNoEvent ? hint : (static_cast<EventId>(idOf(sender)) << 16U) | 1U;
  o.timestamp_ms = kT0;
  MeshPacket p;
  p.payload = ObservationPayload{o};
  return p;
}

MeshPacket fix(unsigned sender, SensorType s, double e, double n, double te, double tn, EventId hint) {
  SensorObservation o;
  o.observer = idOf(sender);
  o.sensor = s;
  o.class_hint = EventClass::THERMAL_HAZARD;
  o.has_fix = true;
  o.observer_position = enu(e, n);
  o.fix = enu(te, tn, 0.0f);
  o.fix_cep_m = 60.0f;
  o.confidence = 0.8f;
  o.event_hint = hint;
  o.timestamp_ms = kT0;
  MeshPacket p;
  p.payload = ObservationPayload{o};
  return p;
}

template <typename Payload>
MeshPacket of(const Payload& payload) {
  MeshPacket p;
  p.payload = payload;
  return p;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <output dir>\n", argv[0]);
    return 2;
  }
  const std::string dir = argv[1];
  const EventId ev = (static_cast<EventId>(2) << 16U) | 1U;
  int written = 0;

  // One record of each message type.
  {
    Seed s;
    s.record(0, heartbeat(0, 0, 0));
    s.write(dir, "type-heartbeat");
    ++written;
  }
  {
    Seed s;
    s.record(0, bearing(0, 0, 0, 10000, 2000));
    s.write(dir, "type-observation-bearing");
    ++written;
  }
  {
    Seed s;
    s.record(0, fix(0, SensorType::IR, 0, 0, 9000, 0, ev));
    s.write(dir, "type-observation-fix");
    ++written;
  }
  {
    Seed s;
    s.record(0, of(TaskClaimPayload{ev, 2, 0, 0.8f}));
    s.write(dir, "type-claim");
    ++written;
  }
  {
    Seed s;
    s.record(0, of(AlertIssuedPayload{ev, AlertLevel::ADVISORY, 2}));
    s.write(dir, "type-alert-issued");
    ++written;
  }
  {
    Seed s;
    s.record(0, of(SignatureUpdatePayload{433920}));
    s.write(dir, "type-signature-update");
    ++written;
  }
  {
    Seed s;
    s.record(0, of(LeavePayload{}));
    s.write(dir, "type-leave");
    ++written;
  }

  // Four drones in line abreast triangulate a beacon, then two imagers confirm it:
  // drives fusion, the advisory, planning, claims and the priority alert.
  Seed tri;
  for (unsigned k = 0; k < 4; ++k) tri.record(k, heartbeat(k, 0, -9000.0 + 6000.0 * k));
  for (unsigned k = 0; k < 4; ++k) tri.record(k, bearing(k, 0, -9000.0 + 6000.0 * k, 10000, 2000, 406025, ev), 1);
  tri.record(0, heartbeat(0, 0, -9000), 7);
  tri.write(dir, "scenario-triangulate");
  ++written;
  tri.record(1, fix(1, SensorType::IR, 4000, -3000, 10050, 1980, ev), 3);
  tri.record(3, fix(3, SensorType::EO, 4000, 9000, 9970, 2030, ev), 3);
  tri.record(1, of(TaskClaimPayload{ev, 3, 0, 0.9f}), 5);
  tri.record(3, of(TaskClaimPayload{ev, 5, 1, 0.7f}));
  tri.record(2, of(AlertIssuedPayload{ev, AlertLevel::PRIORITY, 4}), 7);
  tri.write(dir, "scenario-priority");
  ++written;

  // The operator dismisses the triangulated source: verifiers are released.
  {
    Seed s = tri;
    s.record(2, of(SignatureUpdatePayload{406025}), 7);
    s.record(2, heartbeat(2, 0, 3000), 7);
    s.write(dir, "scenario-dismissal");
    ++written;
  }

  // Two beacons on one frequency, far apart: must stay two events.
  {
    Seed s;
    const double oe[3] = {0.0, 25000.0, 0.0};
    const double on[3] = {0.0, 0.0, 25000.0};
    for (unsigned k = 0; k < 3; ++k) s.record(k, heartbeat(k, oe[k], on[k]));
    for (unsigned k = 0; k < 3; ++k) {
      s.record(k, bearing(k, oe[k], on[k], 30000, 40000, 406025, (static_cast<EventId>(idOf(k)) << 16U) | 7U), 1);
    }
    for (unsigned k = 0; k < 3; ++k) {
      s.record(k, bearing(k, oe[k], on[k], 30000, -40000, 406025, (static_cast<EventId>(idOf(k)) << 16U) | 8U), 1);
    }
    s.record(0, heartbeat(0, 0, 0), 7);
    s.write(dir, "scenario-same-frequency");
    ++written;
  }

  // Replays, reboots, an operator dismissal, a drone leaving.
  {
    Seed s;
    s.record(0, heartbeat(0, 0, 0));
    s.replay(0);
    s.record(0, bearing(0, 0, 0, 10000, 2000), 1);
    s.replay(0, 2);
    s.record(0, heartbeat(0, 0, 0), 1, false, true);  // rebooted: new session
    s.record(1, heartbeat(1, 0, 6000, DroneMode::ACTIVE_MISSION));
    s.record(1, of(SignatureUpdatePayload{406025}), 5);
    s.record(1, of(LeavePayload{}), 7);
    s.write(dir, "scenario-replay-reboot-leave");
    ++written;
  }

  // A genuinely sealed frame as raw input: parses, then fails the raw receiver's key.
  {
    MeshConfig cfg;
    MeshSecurity tx(2, 1U, cfg);
    crypto::Key k{};
    for (std::size_t i = 0; i < k.size(); ++i) k[i] = static_cast<std::uint8_t>(0x3C ^ (i * 13U));
    if (!tx.installEpochKey(1, k, 0, kNoExpiry)) return 1;
    MeshPacket p = heartbeat(0, 0, 0);
    std::uint8_t frame[kMaxPacketBytes];
    const std::size_t n = tx.seal(p, kT0, frame, sizeof frame);
    std::ofstream out(dir + "/raw-sealed-frame", std::ios::binary);
    out.write(reinterpret_cast<const char*>(frame), static_cast<std::streamsize>(n));
    ++written;
  }
  std::printf("wrote %d seeds to %s\n", written, dir.c_str());
  return 0;
}
