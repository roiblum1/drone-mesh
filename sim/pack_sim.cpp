// Six-drone pack scenario on an in-memory mesh. Not flight code: it uses the
// standard library freely and exists to exercise the Model 8270 mesh logic
// end to end and to produce measured numbers for the documentation.
//
// Pack: D1 is under operator control (ACTIVE_MISSION); D2..D6 fly the route
// passively (PACK_ROUTING), 10 km apart, line abreast. D5 is almost at
// bingo fuel. Three RF sources appear:
//   406.025 MHz distress beacon + thermal source  -> real event
//   433.920 MHz ISM emitter                        -> novel, later dismissed by the operator
//   162.550 MHz weather radio                      -> pre-briefed benign
// A verifier is lost mid-transit to exercise re-assignment.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <vector>

#include "amc8270/mesh/geo.hpp"
#include "amc8270/mesh/pack_mesh_manager.hpp"

using namespace amc8270::mesh;

namespace {

constexpr TimeMs kT0 = 1'700'000'000'000ULL;
constexpr double kDt = 0.1;  // s per tick (mesh runs at 10 Hz)
constexpr double kEndS = 600.0;
constexpr double kRouteSpeed = 40.0;  // m/s cruise (~144 km/h)
constexpr double kDashSpeed = 45.0;   // m/s while verifying
constexpr double kDfRangeM = 35000.0;
constexpr double kDfSigmaDeg = 2.5;
constexpr double kFixSigmaM = 45.0;
constexpr double kMeshRangeM = 60000.0;
constexpr double kMeshLoss = 0.05;

constexpr std::uint32_t kSigBeacon = 406025;
constexpr std::uint32_t kSigIsm = 433920;
constexpr std::uint32_t kSigKnown = 162550;

const GeoPoint kOrigin{36.0, -12.0, 0.0f};
const LocalFrame kFrame(kOrigin);

double g_t = 0.0;  // simulation time, s (for log lines)

struct Rng {
  std::uint32_t s = 0x9E3779B9u;
  std::uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  double uniform() { return (static_cast<double>(next()) + 1.0) / 4294967297.0; }
  double gauss() { return std::sqrt(-2.0 * std::log(uniform())) * std::cos(6.283185307179586 * uniform()); }
};

struct Emitter {
  std::uint32_t sig;
  Vec2 pos;
  double on_s;
  EventClass cls;
  bool thermal;
  const char* label;
};

const std::array<Emitter, 3> kEmitters = {{
    {kSigBeacon, {16000.0, 8000.0}, 20.0, EventClass::DISTRESS_BEACON, true, "distress beacon"},
    {kSigIsm, {30000.0, -12000.0}, 100.0, EventClass::RF_EMITTER, false, "ISM emitter"},
    {kSigKnown, {22000.0, 30000.0}, 10.0, EventClass::RF_EMITTER, false, "weather radio"},
}};

const char* labelOf(std::uint32_t sig) {
  for (const auto& e : kEmitters) {
    if (e.sig == sig) return e.label;
  }
  return "unknown";
}

const char* sensorName(SensorType s) {
  switch (s) {
    case SensorType::RF_DF: return "RF-DF";
    case SensorType::EO: return "EO";
    case SensorType::IR: return "IR";
    case SensorType::ANOMALY: return "ANOM";
    default: return "-";
  }
}

const char* linkName(LinkType l) {
  switch (l) {
    case LinkType::LOS: return "LOS";
    case LinkType::LEO: return "LEO-SATCOM";
    case LinkType::GEO: return "GEO-SATCOM";
  }
  return "?";
}

void say(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void say(const char* fmt, ...) {
  std::printf("[t=%6.1fs] ", g_t);
  va_list ap;
  va_start(ap, fmt);
  std::vprintf(fmt, ap);
  va_end(ap);
  std::printf("\n");
}

struct SimDrone;

struct Net {
  std::vector<SimDrone*> drones;
  Rng rng;
  std::uint64_t bytes_total = 0;
  std::uint64_t datagrams_sent = 0;
  std::uint64_t datagrams_lost = 0;
  std::array<std::uint64_t, 6> bytes_by_src{};
  std::vector<std::uint64_t> bytes_per_second = std::vector<std::uint64_t>(static_cast<std::size_t>(kEndS) + 2, 0);
  void broadcast(int src, const std::uint8_t* d, std::size_t n);
};

struct Node : IMeshTransceiver {
  Net* net = nullptr;
  int idx = 0;
  std::deque<std::vector<std::uint8_t>> inbox;
  bool send(const std::uint8_t* d, std::size_t n) override {
    net->broadcast(idx, d, n);
    return true;
  }
  std::size_t receive(std::uint8_t* buf, std::size_t cap) override {
    if (inbox.empty()) return 0;
    const std::vector<std::uint8_t>& f = inbox.front();
    const std::size_t n = std::min(cap, f.size());
    std::memcpy(buf, f.data(), n);
    inbox.pop_front();
    return n;
  }
};

struct TaskLog {
  double t;
  int drone;
  std::uint32_t sig;
  std::uint8_t role;
  SensorType cue;
};
struct AlertLog {
  double t;
  GmcsAlert a;
  LinkType link;
  double err_m;  // fused position vs truth (beacon only)
};

struct Ground {
  std::vector<AlertLog> alerts;
  std::vector<TaskLog> tasks;
};
Ground g_ground;

struct SimClock : IClock {
  TimeMs now = kT0;
  TimeMs nowMs() const override { return now; }
};
SimClock g_clock;

struct SimGmcs : IGmcsLink {
  int drone = 0;
  bool sendAlert(const GmcsAlert& a, LinkType link) override {
    double err = -1.0;
    if (a.cls == EventClass::DISTRESS_BEACON) err = distanceM(a.position, kFrame.fromEnu(kEmitters[0].pos, 0.0f));
    g_ground.alerts.push_back({g_t, a, link, err});
    say("GMCS <- D%d  %-8s %s evt=%06X  pos err=%.0f m  CEP=%.0f m  conf=%.2f  sensor types=%u observers=%u  via %s",
        drone, a.level == AlertLevel::PRIORITY ? "PRIORITY" : "ADVISORY",
        a.cls == EventClass::DISTRESS_BEACON ? "distress-beacon" : "event", a.event_id, err,
        static_cast<double>(a.cep_m), static_cast<double>(a.confidence), popCount(a.modality_mask),
        static_cast<unsigned>(a.observer_count), linkName(link));
    return true;
  }
};

struct SimGuidance : IGuidanceSink {
  SimDrone* owner = nullptr;
  void requestVerification(const VerificationTask& t) override;
  void cancelVerification() override;
};

struct SimDrone {
  int id = 0;  // 1-based, equals DroneId
  DroneMode commanded = DroneMode::PACK_ROUTING;
  SensorMask sensors = 0;
  LinkMask links = kLinkMaskAll;
  Vec2 pos{};
  Vec2 slot{};  // formation slot on the planned route
  double speed = kRouteSpeed;
  double heading_deg = 90.0;
  double endurance_s = 0.0;
  double bingo_s = 1800.0;
  bool alive = true;

  bool ever_tasked = false;
  bool tasked = false;
  bool orbiting = false;
  double orbit_phase = 0.0;
  Vec2 target{};
  Vec2 standoff{};
  SensorType cue = SensorType::NONE;
  EventId task_event = kNoEvent;

  Node node;
  SimGmcs gmcs;
  SimGuidance guidance;
  std::unique_ptr<PackMeshManager> mgr;

  void advance();
  DroneState state() const;
};

void Net::broadcast(int src, const std::uint8_t* d, std::size_t n) {
  const SimDrone& s = *drones[static_cast<std::size_t>(src)];
  if (!s.alive) return;
  ++datagrams_sent;
  bytes_total += n;
  bytes_by_src[static_cast<std::size_t>(src)] += n;
  bytes_per_second[static_cast<std::size_t>(g_t)] += n;
  for (std::size_t i = 0; i < drones.size(); ++i) {
    SimDrone& r = *drones[i];
    if (static_cast<int>(i) == src || !r.alive) continue;
    if (std::hypot(s.pos.e - r.pos.e, s.pos.n - r.pos.n) > kMeshRangeM) continue;
    if (rng.uniform() < kMeshLoss) {
      ++datagrams_lost;
      continue;
    }
    r.node.inbox.emplace_back(d, d + n);
  }
}

void SimGuidance::requestVerification(const VerificationTask& t) {
  SimDrone& d = *owner;
  const DetectedEvent* ev = d.mgr->table().findEvent(d.mgr->table().resolve(t.event_id));
  const std::uint32_t sig = ev != nullptr ? ev->signature : 0;
  const Vec2 so = kFrame.toEnu(t.standoff);
  d.target = kFrame.toEnu(t.target);
  d.standoff = so;
  d.cue = t.sensor_to_cue;
  d.task_event = t.event_id;
  d.tasked = true;
  d.orbiting = false;
  d.ever_tasked = true;
  g_ground.tasks.push_back({g_t, d.id, sig, t.role, t.sensor_to_cue});
  say("D%d  TASK  %-9s verifier for %s (evt=%06X)  cue=%s  standoff=(%.1f,%.1f) km  approach=%.0f deg  %.1f km away",
      d.id, t.role == 0 ? "primary" : "secondary", labelOf(sig), t.event_id, sensorName(t.sensor_to_cue), so.e / 1000.0,
      so.n / 1000.0, static_cast<double>(t.approach_bearing_deg), std::hypot(so.e - d.pos.e, so.n - d.pos.n) / 1000.0);
}

void SimGuidance::cancelVerification() {
  SimDrone& d = *owner;
  if (!d.tasked) return;
  d.tasked = false;
  d.orbiting = false;
  say("D%d  RELEASED from verification, rejoining route slot", d.id);
}

DroneState SimDrone::state() const {
  DroneState s;
  s.id = static_cast<DroneId>(id);
  s.mode = commanded;
  s.installed_sensors = sensors;
  s.available_links = links;
  s.active_sensor = tasked ? cue : SensorType::RF_DF;
  s.position = kFrame.fromEnu(pos, 3000.0f);
  s.speed_mps = static_cast<float>(speed);
  s.heading_deg = static_cast<float>(heading_deg);
  s.endurance_remaining_s = static_cast<float>(endurance_s);
  s.bingo_endurance_s = static_cast<float>(bingo_s);
  return s;
}

// Kinematic stand-in for the FCS: formation slot moves east; a tasked drone
// dashes to its standoff point, then orbits the target.
void SimDrone::advance() {
  slot.e += kRouteSpeed * kDt;
  endurance_s -= kDt;
  const Vec2 before = pos;
  if (tasked && commanded == DroneMode::PACK_ROUTING) {
    speed = kDashSpeed;
    if (!orbiting) {
      const double de = standoff.e - pos.e;
      const double dn = standoff.n - pos.n;
      const double dist = std::hypot(de, dn);
      if (dist < 200.0) {
        orbiting = true;
        orbit_phase = std::atan2(pos.e - target.e, pos.n - target.n);
      } else {
        pos.e += de / dist * kDashSpeed * kDt;
        pos.n += dn / dist * kDashSpeed * kDt;
      }
    }
    if (orbiting) {
      const double r = std::max(1000.0, std::hypot(standoff.e - target.e, standoff.n - target.n));
      orbit_phase += kDashSpeed / r * kDt;
      pos.e = target.e + r * std::sin(orbit_phase);
      pos.n = target.n + r * std::cos(orbit_phase);
    }
  } else {
    speed = kRouteSpeed;
    const double de = slot.e - pos.e;
    const double dn = slot.n - pos.n;
    const double dist = std::hypot(de, dn);
    if (dist > 300.0) {
      speed = 55.0;
      pos.e += de / dist * 55.0 * kDt;
      pos.n += dn / dist * 55.0 * kDt;
    } else {
      pos = slot;
    }
  }
  const double me = pos.e - before.e;
  const double mn = pos.n - before.n;
  if (std::hypot(me, mn) > 1e-6) heading_deg = wrap360(std::atan2(me, mn) * 180.0 / 3.14159265358979);
}

// ------------------------------------------------------------ sensor models

void sampleSensors(SimDrone& d, Rng& rng) {
  const GeoPoint here = kFrame.fromEnu(d.pos, 3000.0f);
  for (const Emitter& e : kEmitters) {
    if (g_t < e.on_s) continue;
    const double range = std::hypot(e.pos.e - d.pos.e, e.pos.n - d.pos.n);

    if ((d.sensors & maskOf(SensorType::RF_DF)) != 0 && range <= kDfRangeM) {
      SensorObservation o;
      o.sensor = SensorType::RF_DF;
      o.class_hint = e.cls;
      o.has_fix = false;
      const double truth = std::atan2(e.pos.e - d.pos.e, e.pos.n - d.pos.n) * 180.0 / 3.14159265358979;
      o.bearing_deg = static_cast<float>(wrap360(truth + rng.gauss() * kDfSigmaDeg));
      o.bearing_sigma_deg = static_cast<float>(kDfSigmaDeg);
      o.confidence = 0.5f;
      o.signature = e.sig;
      d.mgr->reportLocalDetection(o);
    }

    if (!e.thermal) continue;
    for (const SensorType s : {SensorType::IR, SensorType::EO}) {
      if ((d.sensors & maskOf(s)) == 0) continue;
      const double reach = (d.tasked && d.cue == s) ? 10000.0 : 3000.0;  // cued vs passive wide-area look
      if (range > reach) continue;
      SensorObservation o;
      o.sensor = s;
      o.class_hint = EventClass::UNKNOWN;
      o.has_fix = true;
      o.fix = kFrame.fromEnu(Vec2{e.pos.e + rng.gauss() * kFixSigmaM, e.pos.n + rng.gauss() * kFixSigmaM}, 0.0f);
      o.fix_cep_m = 70.0f;
      o.confidence = s == SensorType::IR ? 0.8f : 0.7f;
      o.observer_position = here;
      d.mgr->reportLocalDetection(o);
    }
  }
}

// --------------------------------------------------------------- scenario

struct SrcTrack {
  EventStatus status = EventStatus::CANDIDATE;
  bool seen = false;
  bool localized = false;
  bool cep_small = false;
};

}  // namespace

int main() {
  std::printf("Model 8270 pack mesh - 6-drone scenario (deterministic, %.0f s)\n\n", kEndS);

  MeshConfig cfg;
  Net net;
  std::vector<std::unique_ptr<SimDrone>> drones;

  struct Spec {
    SensorMask sensors;
    LinkMask links;
    double endurance_h;
    DroneMode mode;
  };
  const SensorMask RF = maskOf(SensorType::RF_DF), EO = maskOf(SensorType::EO), IR = maskOf(SensorType::IR),
                   AN = maskOf(SensorType::ANOMALY);
  const std::array<Spec, 6> specs = {{
      {static_cast<SensorMask>(RF | EO | IR), kLinkMaskAll, 3.5, DroneMode::ACTIVE_MISSION},
      {static_cast<SensorMask>(RF | EO), kLinkMaskAll, 4.0, DroneMode::PACK_ROUTING},
      {static_cast<SensorMask>(RF | IR), kLinkMaskAll, 4.5, DroneMode::PACK_ROUTING},
      {static_cast<SensorMask>(RF | IR | AN), static_cast<LinkMask>(2 | 4), 4.1, DroneMode::PACK_ROUTING},  // LOS shadowed
      {static_cast<SensorMask>(RF | EO), kLinkMaskAll, 2000.0 / 3600.0, DroneMode::PACK_ROUTING},         // near bingo
      {static_cast<SensorMask>(RF | IR), kLinkMaskAll, 4.2, DroneMode::PACK_ROUTING},
  }};

  for (std::size_t i = 0; i < specs.size(); ++i) {
    auto d = std::make_unique<SimDrone>();
    d->id = static_cast<int>(i) + 1;
    d->commanded = specs[i].mode;
    d->sensors = specs[i].sensors;
    d->links = specs[i].links;
    d->slot = Vec2{0.0, (static_cast<double>(i) - 2.5) * 10000.0};
    d->pos = d->slot;
    d->endurance_s = specs[i].endurance_h * 3600.0;
    d->node.net = &net;
    d->node.idx = static_cast<int>(i);
    d->gmcs.drone = d->id;
    d->guidance.owner = d.get();
    d->mgr = std::make_unique<PackMeshManager>(static_cast<DroneId>(d->id), cfg, g_clock, d->node, d->gmcs, d->guidance);
    d->mgr->addKnownBenignSignature(kSigKnown);  // pre-briefed in the mission data load
    net.drones.push_back(d.get());
    drones.push_back(std::move(d));
  }

  std::printf("pack: D1 ACTIVE_MISSION (operator)  D2..D6 PACK_ROUTING, 10 km line abreast, route speed %.0f m/s\n", kRouteSpeed);
  std::printf("      D5 endurance %.0f s vs bingo %.0f s (cannot be diverted)   D4 has no LOS link\n\n",
              drones[4]->endurance_s, drones[4]->bingo_s);

  Rng sensor_rng;
  sensor_rng.s = 0xC0FFEEu;
  net.rng.s = 0x1234ABCDu;

  std::map<std::uint32_t, SrcTrack> tracks;
  bool killed = false;
  int killed_id = 0;
  double killed_t = 0.0;
  bool dismissed = false;
  std::vector<bool> emitter_announced(kEmitters.size(), false);
  double first_beacon_alert_t = -1.0;

  const std::size_t steps = static_cast<std::size_t>(kEndS / kDt);
  for (std::size_t step = 0; step < steps; ++step) {
    g_clock.now += static_cast<TimeMs>(kDt * 1000.0);
    g_t = static_cast<double>(step + 1) * kDt;

    // ---- scripted events
    for (std::size_t k = 0; k < kEmitters.size(); ++k) {
      if (!emitter_announced[k] && g_t >= kEmitters[k].on_s) {
        emitter_announced[k] = true;
        say("WORLD  %s starts transmitting at (%.1f,%.1f) km", kEmitters[k].label, kEmitters[k].pos.e / 1000.0,
            kEmitters[k].pos.n / 1000.0);
      }
    }
    if (!killed && g_t >= 90.0) {
      for (const auto& d : drones) {
        const DetectedEvent* ev = nullptr;
        for (const auto& e : d->mgr->table().events()) {
          if (e.signature == kSigBeacon) ev = &e;
        }
        if (!d->alive || ev == nullptr) continue;
        int victim = ev->verifier[0].id != kNoDrone ? ev->verifier[0].id : ev->verifier[1].id;
        if (victim == kNoDrone) continue;
        drones[static_cast<std::size_t>(victim - 1)]->alive = false;
        killed = true;
        killed_id = victim;
        killed_t = g_t;
        say("FAULT  D%d (primary verifier) lost - no more traffic from it", victim);
        break;
      }
    }
    if (!dismissed && g_t >= 420.0) {
      dismissed = true;
      say("GMCS   operator dismisses the %s as benign (uplink via D1)", labelOf(kSigIsm));
      drones[0]->mgr->applyGmcsDismissal(kSigIsm);
    }

    // ---- flight, sensors, mesh
    for (auto& d : drones) {
      if (!d->alive) continue;
      d->advance();
      d->mgr->updateOwnState(d->state());
    }
    if (step % 20 == 0) {
      for (auto& d : drones) {
        if (d->alive) sampleSensors(*d, sensor_rng);
      }
    }
    for (auto& d : drones) {
      if (d->alive) d->mgr->tick();
    }

    // ---- observe the pack through the lowest-numbered live drone's table
    if (step % 10 == 0) {
      const SimDrone* ref = nullptr;
      for (const auto& d : drones) {
        if (d->alive) {
          ref = d.get();
          break;
        }
      }
      for (const auto& ev : ref->mgr->table().events()) {
        if (ev.signature == 0) continue;
        SrcTrack& tr = tracks[ev.signature];
        if (!tr.seen) {
          tr.seen = true;
          say("MESH   %s first detected; %u observers share bearings", labelOf(ev.signature),
              static_cast<unsigned>(ev.observer_count));
        }
        const Emitter* em = nullptr;
        for (const auto& e : kEmitters) {
          if (e.sig == ev.signature) em = &e;
        }
        if (ev.localized && !tr.localized) {
          tr.localized = true;
          const double err = em != nullptr ? distanceM(ev.position, kFrame.fromEnu(em->pos, 0.0f)) : -1.0;
          say("MESH   %s localised by triangulation: %u bearings, CEP %.0f m, actual error %.0f m, conf %.2f, urgency %.2f",
              labelOf(ev.signature), static_cast<unsigned>(ev.observer_count), static_cast<double>(ev.cep_m), err,
              static_cast<double>(ev.confidence), static_cast<double>(ev.urgency));
        }
        if (ev.localized && ev.cep_m <= 250.0f && !tr.cep_small && popCount(ev.modality_mask) >= 2) {
          tr.cep_small = true;
          const double err = em != nullptr ? distanceM(ev.position, kFrame.fromEnu(em->pos, 0.0f)) : -1.0;
          say("MESH   %s multi-sensor fix: CEP %.0f m, actual error %.0f m, %u sensor types, conf %.2f", labelOf(ev.signature),
              static_cast<double>(ev.cep_m), err, popCount(ev.modality_mask), static_cast<double>(ev.confidence));
        }
        const bool noteworthy = ev.status == EventStatus::VERIFYING || ev.status == EventStatus::CONFIRMED ||
                                ev.status == EventStatus::DISMISSED;
        if (ev.status != tr.status && noteworthy) {  // CANDIDATE/LOCALIZED flapping is not logged
          tr.status = ev.status;
          const char* name = ev.status == EventStatus::VERIFYING   ? "VERIFYING"
                             : ev.status == EventStatus::CONFIRMED ? "CONFIRMED"
                             : ev.status == EventStatus::DISMISSED ? "DISMISSED"
                             : ev.status == EventStatus::LOCALIZED ? "LOCALIZED"
                                                                   : "CANDIDATE";
          say("MESH   %s -> %s (urgency %.2f)", labelOf(ev.signature), name, static_cast<double>(ev.urgency));
        }
      }
    }
    if (first_beacon_alert_t < 0.0 && !g_ground.alerts.empty()) first_beacon_alert_t = g_ground.alerts.front().t;
  }

  // ------------------------------------------------------------- summary
  std::printf("\n================ summary ================\n");
  std::printf("%-5s %-15s %-7s %7s %7s %5s %7s %7s %7s %9s\n", "drone", "role", "tasked", "tx", "rx", "bad", "tasks",
              "claims", "lost", "tx kbit/s");
  std::uint32_t max_tasks = 0;
  for (std::size_t i = 0; i < drones.size(); ++i) {
    const SimDrone& d = *drones[i];
    const MeshStats& s = d.mgr->stats();
    max_tasks = std::max(max_tasks, s.tasks_issued);
    std::printf("D%-4d %-15s %-7s %7u %7u %5u %7u %7u %7u %9.2f\n", d.id,
                d.commanded == DroneMode::ACTIVE_MISSION ? "ACTIVE_MISSION" : "PACK_ROUTING", d.ever_tasked ? "yes" : "no",
                s.tx_packets, s.rx_packets, s.rx_rejected, s.tasks_issued, s.claims_sent, s.peers_lost,
                static_cast<double>(net.bytes_by_src[i]) * 8.0 / 1000.0 / kEndS);
  }
  const double kbps = static_cast<double>(net.bytes_total) * 8.0 / 1000.0 / kEndS / static_cast<double>(drones.size());
  std::uint64_t peak_bytes = 0;
  for (const auto b : net.bytes_per_second) peak_bytes = std::max(peak_bytes, b);
  std::printf("\nmesh traffic: %llu datagrams, %llu bytes total, %llu lost to simulated link loss (%.0f%% configured)\n",
              static_cast<unsigned long long>(net.datagrams_sent), static_cast<unsigned long long>(net.bytes_total),
              static_cast<unsigned long long>(net.datagrams_lost), kMeshLoss * 100.0);
  std::printf("offered load (all message types, incl. headers + CRC): mean %.2f kbit/s per drone, "
              "peak %.2f kbit/s per drone in any 1 s window\n",
              kbps, static_cast<double>(peak_bytes) * 8.0 / 1000.0 / static_cast<double>(drones.size()));

  // ---------------------------------------------------------------- checks
  struct Check {
    const char* what;
    bool ok;
  };
  std::vector<Check> checks;

  std::size_t adv = 0, pri = 0, other_alerts = 0;
  double pri_err = -1.0, pri_t = -1.0, adv_t = -1.0;
  for (const auto& a : g_ground.alerts) {
    if (a.a.cls != EventClass::DISTRESS_BEACON) {
      ++other_alerts;
      continue;
    }
    if (a.a.level == AlertLevel::PRIORITY) {
      ++pri;
      pri_err = a.err_m;
      pri_t = a.t;
    } else {
      ++adv;
      adv_t = a.t;
    }
  }
  bool d1_tasked = drones[0]->ever_tasked;
  bool d5_tasked = drones[4]->ever_tasked;
  bool known_tasked = false, ism_tasked = false, reassigned = false;
  for (const auto& t : g_ground.tasks) {
    if (t.sig == kSigKnown) known_tasked = true;
    if (t.sig == kSigIsm) ism_tasked = true;
    if (t.sig == kSigBeacon && killed && t.t > killed_t && t.drone != killed_id) reassigned = true;
  }
  bool ism_learned = true;
  for (const auto& d : drones) {
    if (d->alive && !d->mgr->table().isBenign(kSigIsm)) ism_learned = false;
  }
  bool none_verifying = true;
  for (const auto& d : drones) {
    if (d->alive && d->mgr->isVerifying()) none_verifying = false;
  }

  checks.push_back({"early-warning ADVISORY for the beacon issued exactly once", adv == 1});
  checks.push_back({"PRIORITY alert for the beacon issued exactly once", pri == 1});
  checks.push_back({"PRIORITY came after the ADVISORY", adv_t >= 0.0 && pri_t > adv_t});
  checks.push_back({"PRIORITY position within 150 m of the true source", pri == 1 && pri_err >= 0.0 && pri_err < 150.0});
  checks.push_back({"ACTIVE_MISSION drone D1 was never tasked by the mesh", !d1_tasked});
  checks.push_back({"near-bingo drone D5 was never diverted", !d5_tasked});
  checks.push_back({"lost verifier replaced by another drone", killed && reassigned});
  checks.push_back({"pre-briefed benign emitter never tasked and never alerted", !known_tasked && other_alerts == 0});
  checks.push_back({"operator dismissal learned by every live drone", ism_learned});
  checks.push_back({"no drone still diverted at end of run (all released)", none_verifying});
  checks.push_back({"no task churn: every drone issued <= 6 FCS requests", max_tasks <= 6});
  (void)ism_tasked;

  std::printf("\n");
  int failed = 0;
  for (const auto& c : checks) {
    std::printf("  [%s] %s\n", c.ok ? "PASS" : "FAIL", c.what);
    failed += c.ok ? 0 : 1;
  }
  std::printf("\n%s\n", failed == 0 ? "scenario result: PASS" : "scenario result: FAIL");
  return failed == 0 ? 0 : 1;
}
