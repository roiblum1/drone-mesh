// The simulated world: formation flight, terrain, a radio with latency and
// bursty loss, direction finders with multipath, imagers, clutter, C2 links
// and the GMCS. Every random process draws from a per-run generator, so a run
// is a pure function of (scenario, seed): bit-identical on one platform.
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <queue>
#include <set>
#include <tuple>

#include "amc8270/mesh/crypto.hpp"
#include "amc8270/mesh/geo.hpp"
#include "sim.hpp"

namespace sim {

using mesh::DetectedEvent;
using mesh::DroneMode;
using mesh::DroneState;
using mesh::EventClass;
using mesh::EventId;
using mesh::GeoPoint;
using mesh::LinkType;
using mesh::SensorObservation;
using mesh::SensorType;
using mesh::TimeMs;
using mesh::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371008.8;
constexpr double kRadioK = 4.0 / 3.0;  // effective earth radius factor for radio paths
constexpr double kOpticalK = 1.0;
constexpr TimeMs kT0 = 1'700'000'000'000ULL;
constexpr int kStepMs = 10;   // world resolution
constexpr int kTickMs = 100;  // each manager ticks at 10 Hz, at its own phase

// xorshift64* with Box-Muller: the same sequence on every platform and library
// (std:: distributions are not), so documented numbers reproduce anywhere.
struct Rng {
  std::uint64_t s;
  explicit Rng(std::uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ULL + 0x632BE59BD9B4E019ULL) {
    if (s == 0) s = 1;
  }
  std::uint64_t next() {
    s ^= s >> 12U;
    s ^= s << 25U;
    s ^= s >> 27U;
    return s * 0x2545F4914F6CDD1DULL;
  }
  double uniform() { return (static_cast<double>(next() >> 11U) + 0.5) / 9007199254740992.0; }  // (0, 1)
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }
  double gauss() { return std::sqrt(-2.0 * std::log(uniform())) * std::cos(2.0 * kPi * uniform()); }
  double exponential(double mean) { return mean > 0.0 ? -mean * std::log(uniform()) : 0.0; }
};

double wrap360(double d) { return mesh::wrap360(d); }

class Terrain {
 public:
  Terrain(const std::vector<Hill>& hills, const std::vector<Ridge>& ridges) : hills_(hills), ridges_(ridges) {}

  double height(double e, double n) const {
    double h = 0.0;
    for (const auto& hl : hills_) {
      const double r2 = (e - hl.e) * (e - hl.e) + (n - hl.n) * (n - hl.n);
      h += hl.height_m * std::exp(-r2 / (2.0 * hl.radius_m * hl.radius_m));
    }
    for (const auto& r : ridges_) {
      const double se = r.e2 - r.e1, sn = r.n2 - r.n1;
      const double len2 = se * se + sn * sn;
      double t = len2 > 0.0 ? ((e - r.e1) * se + (n - r.n1) * sn) / len2 : 0.0;
      t = std::min(1.0, std::max(0.0, t));
      const double de = e - (r.e1 + t * se), dn = n - (r.n1 + t * sn);
      h += r.height_m * std::exp(-(de * de + dn * dn) / (2.0 * r.width_m * r.width_m));
    }
    return h;
  }

  // Straight path from a (height za) to b (zb), with the earth's bulge for an
  // effective radius k * R. Sampled every 250 m.
  bool clear(Vec2 a, double za, Vec2 b, double zb, double k) const {
    if (hills_.empty() && ridges_.empty()) return true;
    const double d = std::hypot(b.e - a.e, b.n - a.n);
    const int steps = std::max(2, static_cast<int>(d / 250.0));
    for (int i = 1; i < steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      const double bulge = (t * d) * ((1.0 - t) * d) / (2.0 * k * kEarthRadiusM);
      const double z = za + t * (zb - za) - bulge;
      if (height(a.e + t * (b.e - a.e), a.n + t * (b.n - a.n)) > z) return false;
    }
    return true;
  }

  bool empty() const { return hills_.empty() && ridges_.empty(); }

 private:
  std::vector<Hill> hills_;
  std::vector<Ridge> ridges_;
};

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

const char* statusName(mesh::EventStatus s) {
  switch (s) {
    case mesh::EventStatus::CANDIDATE: return "CANDIDATE";
    case mesh::EventStatus::LOCALIZED: return "LOCALIZED";
    case mesh::EventStatus::VERIFYING: return "VERIFYING";
    case mesh::EventStatus::CONFIRMED: return "CONFIRMED";
    case mesh::EventStatus::DISMISSED: return "DISMISSED";
  }
  return "?";
}

int linkIndex(LinkType l) { return l == LinkType::LOS ? 0 : (l == LinkType::LEO ? 1 : 2); }

LinkType firstLink(mesh::LinkMask m) {
  if ((m & 1U) != 0) return LinkType::LOS;
  if ((m & 2U) != 0) return LinkType::LEO;
  return LinkType::GEO;
}

// ------------------------------------------------------------------- world

class World;

struct Node : mesh::IMeshTransceiver {
  World* w = nullptr;
  int idx = 0;
  std::deque<std::vector<std::uint8_t>> inbox;
  bool send(const std::uint8_t* d, std::size_t n) override;
  std::size_t receive(std::uint8_t* buf, std::size_t cap) override {
    if (inbox.empty()) return 0;
    const std::size_t n = std::min(cap, inbox.front().size());
    std::memcpy(buf, inbox.front().data(), n);
    inbox.pop_front();
    return n;
  }
};

struct DroneClock : mesh::IClock {
  const TimeMs* world_ms = nullptr;
  std::int64_t offset_ms = 0;
  TimeMs nowMs() const override { return static_cast<TimeMs>(static_cast<std::int64_t>(*world_ms) + offset_ms); }
};

struct Downlink : mesh::IGmcsLink {
  World* w = nullptr;
  int idx = 0;
  bool sendAlert(const mesh::GmcsAlert& a, LinkType link) override;
};

struct Guidance : mesh::IGuidanceSink {
  World* w = nullptr;
  int idx = 0;
  void requestVerification(const mesh::VerificationTask& t) override;
  void cancelVerification() override;
};

struct Reflection {
  double until_s = -1;
  double bias_deg = 0;
};

struct SimDrone {
  DroneSpec spec;
  DroneMode commanded = DroneMode::PACK_ROUTING;
  Vec2 pos{}, slot{};
  double speed = 0, heading_deg = 90, endurance_s = 0;
  bool alive = true;
  int group = 0;
  int phase_ms = 0;       // tick phase within each 100 ms
  int sensor_offset = 0;  // which of the 20 ticks in a 2 s sensor cycle it samples on
  std::uint64_t ticks = 0;

  bool tasked = false, orbiting = false, ever_tasked = false;
  double orbit_phase = 0;
  Vec2 target{}, standoff{};
  SensorType cue = SensorType::NONE;

  std::vector<Reflection> reflections;  // per emitter

  Node node;
  DroneClock clock;
  Downlink downlink;
  Guidance guidance;
  std::unique_ptr<mesh::PackMeshManager> mgr;
};

struct Pending {
  TimeMs at = 0;
  std::uint64_t order = 0;
  int dst = 0;
  std::vector<std::uint8_t> bytes;
  bool operator>(const Pending& o) const { return at != o.at ? at > o.at : order > o.order; }
};

struct PendingAlert {
  TimeMs at = 0;
  std::uint64_t order = 0;
  AlertRecord rec;
  bool operator>(const PendingAlert& o) const { return at != o.at ? at > o.at : order > o.order; }
};

struct PendingUplink {
  TimeMs at = 0;
  int drone = 0;
  std::uint32_t signature = 0;
};

// Replay trace, one frame per trace period.
struct TraceFrame {
  double t = 0;
  std::vector<std::array<double, 6>> drones;  // id, e, n, heading, state, task event
  std::vector<std::array<double, 11>> events; // id, e, n, cep, status, localized, sig, v0, v1, observers, class
  std::vector<std::array<double, 5>> bearings;// observer, e, n, bearing, reflected
  std::vector<std::array<int, 2>> links;      // drone index pairs that can hear each other
};

class World {
 public:
  World(const Scenario& sc, const RunOptions& opt);
  TrialResult run();

  // HAL callbacks
  void broadcast(int src, const std::uint8_t* d, std::size_t n);
  bool toGround(int idx, const mesh::GmcsAlert& a, LinkType link);
  void onTask(int idx, const mesh::VerificationTask& t);
  void onRelease(int idx);

 private:
  void say(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  double t() const { return static_cast<double>(now_ms_ - kT0) / 1000.0; }  // since the scenario started
  bool radioPath(const SimDrone& a, const SimDrone& b) const;
  void stepDrone(SimDrone& d);
  void advanceFlight(SimDrone& d, double dt);
  DroneState stateOf(const SimDrone& d) const;
  void sampleSensors(SimDrone& d);
  void runScript();
  void deliver();
  void observe();
  void traceFrame();
  int matchEmitter(const GeoPoint& p, double cep_m) const;
  const SimDrone* reference() const;
  const DetectedEvent* eventFor(const SimDrone& ref, int emitter) const;
  void evaluate(TrialResult& r);
  void writeTrace(const TrialResult& r) const;

  const Scenario& sc_;
  RunOptions opt_;
  mesh::LocalFrame frame_;
  Terrain terrain_;
  Rng rng_;  // world events (radio, faults); sensors draw from sensor_rng_
  Rng sensor_rng_;
  std::vector<EmitterSpec> emitters_;  // with Monte Carlo jitter applied
  std::vector<std::unique_ptr<SimDrone>> drones_;
  TimeMs now_ms_ = kT0;
  std::uint64_t order_ = 0;

  std::priority_queue<Pending, std::vector<Pending>, std::greater<>> air_;
  std::priority_queue<PendingAlert, std::vector<PendingAlert>, std::greater<>> c2_down_;
  std::vector<PendingUplink> c2_up_;
  std::vector<std::vector<bool>> link_bad_;  // Gilbert-Elliott state per directed link

  std::vector<AlertRecord> alerts_;
  std::vector<TaskRecord> tasks_;
  std::vector<std::uint64_t> bytes_per_s_;
  std::vector<std::uint64_t> bytes_by_src_;
  std::uint64_t datagrams_ = 0, lost_ = 0, bytes_ = 0;
  std::vector<std::size_t> next_script_;
  std::vector<bool> script_done_;
  std::vector<double> loss_t_;      // per emitter: when its verifier was lost (-1 never)
  std::vector<int> loss_victim_;

  // timeline bookkeeping (signature -> what has been announced)
  struct Seen {
    bool seen = false, localized = false, fixed = false;
    mesh::EventStatus status = mesh::EventStatus::CANDIDATE;
  };
  std::vector<Seen> seen_;
  std::vector<bool> announced_;

  std::vector<TraceFrame> trace_;
  std::vector<std::pair<double, std::string>> log_;
  std::vector<std::array<double, 5>> recent_bearings_;
  // True origin of every reading the sim generated -> (emitter or -1, reflected). One sensor can
  // report several sources in the same millisecond, so the key includes the frequency and, for a
  // fix, where it was.
  using TruthKey = std::tuple<int, int, TimeMs, std::uint32_t, long long>;
  std::map<TruthKey, std::pair<int, bool>> truth_;
  static TruthKey keyOf(int observer, const SensorObservation& o) {
    const long long where = o.has_fix ? std::llround(o.fix.lat_deg * 1e5) * 100000000LL + std::llround(o.fix.lon_deg * 1e5) : 0;
    return {observer, static_cast<int>(o.sensor), o.timestamp_ms, o.signature, where};
  }
  void label(const SimDrone& d, const SensorObservation& o, int emitter, bool reflected) {
    truth_[keyOf(d.spec.id, o)] = {emitter, reflected};
  }
};

bool Node::send(const std::uint8_t* d, std::size_t n) {
  w->broadcast(idx, d, n);
  return true;
}
bool Downlink::sendAlert(const mesh::GmcsAlert& a, LinkType link) { return w->toGround(idx, a, link); }
void Guidance::requestVerification(const mesh::VerificationTask& t) { w->onTask(idx, t); }
void Guidance::cancelVerification() { w->onRelease(idx); }

World::World(const Scenario& sc, const RunOptions& opt)
    : sc_(sc),
      opt_(opt),
      frame_(sc.origin),
      terrain_(sc.hills, sc.ridges),
      rng_(opt.seed * 2654435761ULL + 17U),
      sensor_rng_(opt.seed * 40503ULL + 0xC0FFEEULL),
      emitters_(sc.emitters) {
  Rng setup(opt.seed + 99991U);
  if (opt.mc_jitter) {
    for (auto& e : emitters_) {
      const double r = sc.mc_jitter_m * std::sqrt(setup.uniform());
      const double a = setup.uniform(0, 2 * kPi);
      e.e += r * std::cos(a);
      e.n += r * std::sin(a);
      e.on_s = std::max(0.0, e.on_s + setup.uniform(-sc.mc_jitter_on_s, sc.mc_jitter_on_s));
    }
  }
  mesh::crypto::Key key{};  // simulation key, not a real one
  for (std::size_t k = 0; k < key.size(); ++k) key[k] = static_cast<std::uint8_t>(0xA5U ^ (k * 29U));
  const std::size_t n = sc.drones.size();
  link_bad_.assign(n, std::vector<bool>(n, false));
  for (std::size_t i = 0; i < n; ++i) {
    auto d = std::make_unique<SimDrone>();
    d->spec = sc.drones[i];
    d->commanded = d->spec.mode;
    d->pos = d->slot = Vec2{d->spec.e, d->spec.n};
    d->speed = sc.route_speed;
    d->endurance_s = d->spec.endurance_s;
    d->phase_ms = static_cast<int>(setup.next() % (kTickMs / kStepMs)) * kStepMs;
    d->sensor_offset = static_cast<int>(setup.next() % 20U);
    d->reflections.assign(emitters_.size(), Reflection{});
    d->node.w = this;
    d->node.idx = static_cast<int>(i);
    d->clock.world_ms = &now_ms_;
    d->clock.offset_ms = static_cast<std::int64_t>(std::llround(setup.uniform(-sc.radio.clock_skew_ms, sc.radio.clock_skew_ms)));
    d->downlink.w = this;
    d->downlink.idx = static_cast<int>(i);
    d->guidance.w = this;
    d->guidance.idx = static_cast<int>(i);
    d->mgr = std::make_unique<mesh::PackMeshManager>(static_cast<mesh::DroneId>(d->spec.id), sc.cfg, 1U, d->clock,
                                                     d->node, d->downlink, d->guidance);
    (void)d->mgr->installEpochKey(1, key, 0, mesh::kNoExpiry);
    for (const auto s : sc.benign) d->mgr->addKnownBenignSignature(s);
    drones_.push_back(std::move(d));
  }
  bytes_per_s_.assign(static_cast<std::size_t>(sc.duration_s) + 2, 0);
  bytes_by_src_.assign(n, 0);
  script_done_.assign(sc.script.size(), false);
  loss_t_.assign(emitters_.size(), -1);
  loss_victim_.assign(emitters_.size(), 0);
  seen_.assign(emitters_.size(), Seen{});
  announced_.assign(emitters_.size(), false);
}

void World::say(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (opt_.verbose) std::printf("[t=%6.1fs] %s\n", t(), buf);
  if (!opt_.trace_path.empty()) log_.emplace_back(t(), buf);
}

// ------------------------------------------------------------------ radio

bool World::radioPath(const SimDrone& a, const SimDrone& b) const {
  if (!a.alive || !b.alive || a.group != b.group) return false;
  if (std::hypot(a.pos.e - b.pos.e, a.pos.n - b.pos.n) > sc_.radio.range_m) return false;
  return !sc_.radio.terrain_masking || terrain_.clear(a.pos, sc_.altitude_m, b.pos, sc_.altitude_m, kRadioK);
}

void World::broadcast(int src, const std::uint8_t* d, std::size_t n) {
  const SimDrone& s = *drones_[static_cast<std::size_t>(src)];
  if (!s.alive) return;
  ++datagrams_;
  bytes_ += n;
  bytes_by_src_[static_cast<std::size_t>(src)] += n;
  bytes_per_s_[std::min(bytes_per_s_.size() - 1, static_cast<std::size_t>(t()))] += n;
  const RadioModel& r = sc_.radio;
  for (std::size_t i = 0; i < drones_.size(); ++i) {
    if (static_cast<int>(i) == src) continue;
    const SimDrone& dst = *drones_[i];
    if (!radioPath(s, dst)) continue;
    // Gilbert-Elliott: step the link's state, then lose with that state's probability.
    std::vector<bool>::reference bad = link_bad_[static_cast<std::size_t>(src)][i];
    if (bad) {
      if (rng_.uniform() < r.p_bad_to_good) bad = false;
    } else if (rng_.uniform() < r.p_good_to_bad) {
      bad = true;
    }
    if (rng_.uniform() < (bad ? r.loss_bad : r.loss_good)) {
      ++lost_;
      continue;
    }
    const double delay = r.latency_ms + rng_.exponential(r.jitter_ms);
    air_.push(Pending{now_ms_ + static_cast<TimeMs>(std::llround(delay)), order_++, static_cast<int>(i),
                      std::vector<std::uint8_t>(d, d + n)});
  }
}

// --------------------------------------------------------------- C2 / GMCS

bool World::toGround(int idx, const mesh::GmcsAlert& a, LinkType link) {
  const SimDrone& d = *drones_[static_cast<std::size_t>(idx)];
  if (!d.alive) return false;
  AlertRecord rec;
  rec.t_issue = t();
  rec.alert = a;
  rec.link = link;
  // What the event behind the alert was made of, by the true origin of its readings.
  if (const DetectedEvent* ev = d.mgr->table().findEvent(d.mgr->table().resolve(a.event_id))) {
    std::map<int, int> votes;
    for (const auto& o : ev->observations) {
      const auto it = truth_.find(keyOf(o.observer, o));
      if (it == truth_.end()) continue;
      ++rec.readings;
      ++votes[it->second.first];
      rec.reflected += it->second.second ? 1 : 0;
    }
    int best = 0;
    for (const auto& [src, n] : votes) {
      if (n > best) {
        best = n;
        rec.majority_source = src;
      }
    }
  }
  const double delay = sc_.c2.latency_ms[static_cast<std::size_t>(linkIndex(link))] + rng_.exponential(sc_.c2.jitter_ms);
  rec.t_rx = t() + delay / 1000.0;
  c2_down_.push(PendingAlert{now_ms_ + static_cast<TimeMs>(std::llround(delay)), order_++, rec});
  return true;
}

int World::matchEmitter(const GeoPoint& p, double cep_m) const {
  int best = -1;
  double best_d = 0;
  const Vec2 v = frame_.toEnu(p);
  for (std::size_t k = 0; k < emitters_.size(); ++k) {
    const double dist = std::hypot(v.e - emitters_[k].e, v.n - emitters_[k].n);
    if (dist > std::max(2000.0, 3.0 * cep_m)) continue;
    if (best < 0 || dist < best_d) {
      best = static_cast<int>(k);
      best_d = dist;
    }
  }
  return best;
}

void World::onTask(int idx, const mesh::VerificationTask& task) {
  SimDrone& d = *drones_[static_cast<std::size_t>(idx)];
  const Vec2 so = frame_.toEnu(task.standoff);
  d.target = frame_.toEnu(task.target);
  d.standoff = so;
  d.cue = task.sensor_to_cue;
  d.tasked = d.ever_tasked = true;
  d.orbiting = false;
  TaskRecord rec;
  rec.t = t();
  rec.drone = d.spec.id;
  rec.event = task.event_id;
  rec.role = task.role;
  rec.cue = task.sensor_to_cue;
  rec.emitter = matchEmitter(task.target, 3000.0);
  rec.se = so.e;
  rec.sn = so.n;
  tasks_.push_back(rec);
  say("D%d  TASK  %-9s verifier for %s (evt=%06X)  cue=%s  standoff=(%.1f,%.1f) km  approach=%.0f deg  %.1f km away",
      d.spec.id, task.role == 0 ? "primary" : "secondary",
      rec.emitter >= 0 ? emitters_[static_cast<std::size_t>(rec.emitter)].label.c_str() : "unknown", task.event_id,
      sensorName(task.sensor_to_cue), so.e / 1000.0, so.n / 1000.0, static_cast<double>(task.approach_bearing_deg),
      std::hypot(so.e - d.pos.e, so.n - d.pos.n) / 1000.0);
}

void World::onRelease(int idx) {
  SimDrone& d = *drones_[static_cast<std::size_t>(idx)];
  if (!d.tasked) return;
  d.tasked = d.orbiting = false;
  say("D%d  RELEASED from verification, rejoining route slot", d.spec.id);
}

// ------------------------------------------------------------------ flight

DroneState World::stateOf(const SimDrone& d) const {
  DroneState s;
  s.id = static_cast<mesh::DroneId>(d.spec.id);
  s.mode = d.commanded;
  s.installed_sensors = d.spec.sensors;
  s.available_links = d.spec.links;
  s.active_sensor = d.tasked ? d.cue : SensorType::RF_DF;
  s.position = frame_.fromEnu(d.pos, static_cast<float>(sc_.altitude_m));
  s.speed_mps = static_cast<float>(d.speed);
  s.heading_deg = static_cast<float>(d.heading_deg);
  s.endurance_remaining_s = static_cast<float>(std::max(0.0, d.endurance_s));
  s.bingo_endurance_s = static_cast<float>(d.spec.bingo_s);
  return s;
}

// Kinematic stand-in for the FCS: the formation slot moves east; a tasked
// drone dashes to its standoff point, then orbits the target.
void World::advanceFlight(SimDrone& d, double dt) {
  d.slot.e += sc_.route_speed * dt;
  d.endurance_s -= dt;
  const Vec2 before = d.pos;
  if (d.tasked && d.commanded == DroneMode::PACK_ROUTING) {
    d.speed = sc_.dash_speed;
    if (!d.orbiting) {
      const double de = d.standoff.e - d.pos.e, dn = d.standoff.n - d.pos.n;
      const double dist = std::hypot(de, dn);
      if (dist < 200.0) {
        d.orbiting = true;
        d.orbit_phase = std::atan2(d.pos.e - d.target.e, d.pos.n - d.target.n);
      } else {
        d.pos.e += de / dist * sc_.dash_speed * dt;
        d.pos.n += dn / dist * sc_.dash_speed * dt;
      }
    }
    if (d.orbiting) {
      const double r = std::max(1000.0, std::hypot(d.standoff.e - d.target.e, d.standoff.n - d.target.n));
      d.orbit_phase += sc_.dash_speed / r * dt;
      d.pos.e = d.target.e + r * std::sin(d.orbit_phase);
      d.pos.n = d.target.n + r * std::cos(d.orbit_phase);
    }
  } else {
    d.speed = sc_.route_speed;
    const double de = d.slot.e - d.pos.e, dn = d.slot.n - d.pos.n;
    const double dist = std::hypot(de, dn);
    if (dist > 300.0) {
      d.speed = sc_.rejoin_speed;
      d.pos.e += de / dist * sc_.rejoin_speed * dt;
      d.pos.n += dn / dist * sc_.rejoin_speed * dt;
    } else {
      d.pos = d.slot;
    }
  }
  const double me = d.pos.e - before.e, mn = d.pos.n - before.n;
  if (std::hypot(me, mn) > 1e-6) d.heading_deg = wrap360(std::atan2(me, mn) * 180.0 / kPi);
}

// ----------------------------------------------------------------- sensors

void World::sampleSensors(SimDrone& d) {
  const GeoPoint here = frame_.fromEnu(d.pos, static_cast<float>(sc_.altitude_m));
  const DfModel& df = sc_.df;
  for (std::size_t k = 0; k < emitters_.size(); ++k) {
    const EmitterSpec& e = emitters_[k];
    if (t() < e.on_s || t() >= e.off_s) continue;
    const Vec2 ep{e.e, e.n};
    const double ground = terrain_.height(e.e, e.n) + e.mast_m;
    const double range = std::hypot(e.e - d.pos.e, e.n - d.pos.n);

    if (e.sig != 0 && (d.spec.sensors & mesh::maskOf(SensorType::RF_DF)) != 0 && range <= df.range_m &&  // sig 0: no RF
        terrain_.clear(d.pos, sc_.altitude_m, ep, ground, kRadioK)) {
      const double truth = std::atan2(e.e - d.pos.e, e.n - d.pos.n) * 180.0 / kPi;
      Reflection& refl = d.reflections[k];
      if (t() >= refl.until_s && df.mp_prob > 0.0) {
        const double elev = std::atan2(sc_.altitude_m - ground, std::max(range, 1.0)) * 180.0 / kPi;
        const double p = df.mp_prob * (elev < df.mp_low_elev_deg ? df.mp_low_elev_factor : 1.0);
        if (sensor_rng_.uniform() < p) {
          refl.until_s = t() + sensor_rng_.uniform(df.mp_min_s, df.mp_max_s);
          const double mag = sensor_rng_.uniform(df.mp_bias_deg / 3.0, df.mp_bias_deg);
          refl.bias_deg = sensor_rng_.uniform() < 0.5 ? -mag : mag;
        }
      }
      const bool reflected = t() < refl.until_s;
      const double noise = sensor_rng_.gauss() * df.sigma_deg +
                           (reflected ? refl.bias_deg + sensor_rng_.gauss() * df.mp_sigma_deg : 0.0);
      SensorObservation o;
      o.sensor = SensorType::RF_DF;
      o.class_hint = e.cls;
      o.has_fix = false;
      o.bearing_deg = static_cast<float>(wrap360(truth + noise));
      o.bearing_sigma_deg = static_cast<float>(df.sigma_deg);  // the DF does not know it is reflected
      o.confidence = 0.5f;
      o.signature = e.sig;
      o.timestamp_ms = d.clock.nowMs();
      label(d, o, static_cast<int>(k), reflected);
      (void)d.mgr->reportLocalDetection(o);  // false = implausible reading, counted in stats
      if (!opt_.trace_path.empty()) {
        recent_bearings_.push_back({static_cast<double>(d.spec.id), d.pos.e, d.pos.n,
                                    static_cast<double>(o.bearing_deg), reflected ? 1.0 : 0.0});
      }
    }

    if (!e.thermal) continue;
    for (const SensorType s : {SensorType::IR, SensorType::EO}) {
      if ((d.spec.sensors & mesh::maskOf(s)) == 0) continue;
      const double reach = (d.tasked && d.cue == s) ? sc_.img.cued_m : sc_.img.passive_m;
      if (range > reach || !terrain_.clear(d.pos, sc_.altitude_m, ep, ground, kOpticalK)) continue;
      SensorObservation o;
      o.sensor = s;
      // The imager classifies a pure thermal source itself; next to an RF source the DF names the class.
      o.class_hint = e.sig == 0 ? e.cls : EventClass::UNKNOWN;
      o.has_fix = true;
      o.fix = frame_.fromEnu(Vec2{e.e + sensor_rng_.gauss() * sc_.img.sigma_m, e.n + sensor_rng_.gauss() * sc_.img.sigma_m},
                             static_cast<float>(ground));
      o.fix_cep_m = sc_.img.cep_m;
      o.confidence = s == SensorType::IR ? 0.8f : 0.7f;
      o.observer_position = here;
      o.timestamp_ms = d.clock.nowMs();
      label(d, o, static_cast<int>(k), false);
      (void)d.mgr->reportLocalDetection(o);
    }
  }

  // Clutter: a false imaging detection somewhere in the wide-area look.
  const bool imager = (d.spec.sensors & (mesh::maskOf(SensorType::IR) | mesh::maskOf(SensorType::EO))) != 0;
  if (imager && sc_.clutter.per_drone_hour > 0.0 &&
      sensor_rng_.uniform() < sc_.clutter.per_drone_hour * df.period_s / 3600.0) {
    const double r = sc_.img.passive_m * std::sqrt(sensor_rng_.uniform());
    const double a = sensor_rng_.uniform(0, 2 * kPi);
    const Vec2 at{d.pos.e + r * std::cos(a), d.pos.n + r * std::sin(a)};
    SensorObservation o;
    o.sensor = (d.spec.sensors & mesh::maskOf(SensorType::IR)) != 0 ? SensorType::IR : SensorType::EO;
    o.class_hint = sc_.clutter.cls;
    o.has_fix = true;
    o.fix = frame_.fromEnu(at, static_cast<float>(terrain_.height(at.e, at.n)));
    o.fix_cep_m = sc_.img.cep_m;
    o.confidence = sc_.clutter.conf;
    o.timestamp_ms = d.clock.nowMs();
    label(d, o, -1, false);
    (void)d.mgr->reportLocalDetection(o);
    say("WORLD  clutter: D%d %s reports a false detection at (%.1f,%.1f) km", d.spec.id, sensorName(o.sensor),
        at.e / 1000.0, at.n / 1000.0);
  }
}

// ------------------------------------------------------------------ script

const SimDrone* World::reference() const {
  for (const auto& d : drones_) {
    if (d->alive) return d.get();
  }
  return nullptr;
}

const DetectedEvent* World::eventFor(const SimDrone& ref, int emitter) const {
  const EmitterSpec& em = emitters_[static_cast<std::size_t>(emitter)];
  const DetectedEvent* best = nullptr;
  double best_d = 0;
  for (const auto& ev : ref.mgr->table().events()) {
    const Vec2 p = frame_.toEnu(ev.position);
    const double dist = std::hypot(p.e - em.e, p.n - em.n);
    const bool sig_match = em.sig != 0 && ev.signature == em.sig;
    if (!sig_match && (!ev.localized || dist > 3000.0)) continue;
    const double score = sig_match ? dist : dist + 1e7;  // prefer the right frequency
    if (best == nullptr || score < best_d) {
      best = &ev;
      best_d = score;
    }
  }
  return best;
}

void World::runScript() {
  for (std::size_t i = 0; i < sc_.script.size(); ++i) {
    const ScriptEvent& s = sc_.script[i];
    if (script_done_[i] || t() < s.t) continue;
    int em = -1;
    for (std::size_t k = 0; k < emitters_.size(); ++k) {
      if (emitters_[k].label == s.label) em = static_cast<int>(k);
    }
    switch (s.kind) {
      case ScriptKind::LOSE_VERIFIER: {
        const SimDrone* ref = reference();
        const DetectedEvent* ev = ref != nullptr && em >= 0 ? eventFor(*ref, em) : nullptr;
        const mesh::DroneId victim =
            ev == nullptr ? mesh::kNoDrone : (ev->verifier[0].id != mesh::kNoDrone ? ev->verifier[0].id : ev->verifier[1].id);
        if (victim == mesh::kNoDrone) continue;  // retry until someone is verifying it
        for (auto& d : drones_) {
          if (d->spec.id == victim) d->alive = false;
        }
        loss_t_[static_cast<std::size_t>(em)] = t();
        loss_victim_[static_cast<std::size_t>(em)] = victim;
        say("FAULT  D%u (verifier of the %s) lost - no more traffic from it", victim, s.label.c_str());
        break;
      }
      case ScriptKind::LOSE_DRONE:
        for (auto& d : drones_) {
          if (d->spec.id == s.drone) d->alive = false;
        }
        say("FAULT  D%d lost", s.drone);
        break;
      case ScriptKind::DISMISS: {
        // The operator marks the source benign; the GMCS uplinks it to the
        // drone it is flying (or any drone with a C2 link) over the fastest link.
        int to = -1;
        for (std::size_t k = 0; k < drones_.size() && to < 0; ++k) {
          if (drones_[k]->alive && drones_[k]->commanded == DroneMode::ACTIVE_MISSION) to = static_cast<int>(k);
        }
        for (std::size_t k = 0; k < drones_.size() && to < 0; ++k) {
          if (drones_[k]->alive && drones_[k]->spec.links != 0) to = static_cast<int>(k);
        }
        if (to < 0 || em < 0) break;
        const LinkType link = firstLink(drones_[static_cast<std::size_t>(to)]->spec.links);
        const double delay = sc_.c2.latency_ms[static_cast<std::size_t>(linkIndex(link))] + rng_.exponential(sc_.c2.jitter_ms);
        c2_up_.push_back(PendingUplink{now_ms_ + static_cast<TimeMs>(std::llround(delay)), to, emitters_[static_cast<std::size_t>(em)].sig});
        say("GMCS   operator dismisses the %s as benign (uplink to D%d via %s)", s.label.c_str(),
            drones_[static_cast<std::size_t>(to)]->spec.id, linkName(link));
        break;
      }
      case ScriptKind::PARTITION:
        for (auto& d : drones_) {
          d->group = std::find(s.group_b.begin(), s.group_b.end(), d->spec.id) != s.group_b.end() ? 1 : 0;
        }
        say("FAULT  mesh partitioned: %zu drones cannot hear the rest", s.group_b.size());
        break;
      case ScriptKind::HEAL:
        for (auto& d : drones_) d->group = 0;
        say("WORLD  mesh partition healed");
        break;
    }
    script_done_[i] = true;
  }
}

// ---------------------------------------------------------------- the loop

void World::deliver() {
  while (!air_.empty() && air_.top().at <= now_ms_) {
    const Pending& p = air_.top();
    SimDrone& d = *drones_[static_cast<std::size_t>(p.dst)];
    if (d.alive) d.node.inbox.push_back(p.bytes);
    air_.pop();
  }
  while (!c2_down_.empty() && c2_down_.top().at <= now_ms_) {
    AlertRecord rec = c2_down_.top().rec;
    c2_down_.pop();
    rec.emitter = matchEmitter(rec.alert.position, static_cast<double>(rec.alert.cep_m));
    if (rec.emitter >= 0) {
      const EmitterSpec& e = emitters_[static_cast<std::size_t>(rec.emitter)];
      rec.err_m = std::hypot(frame_.toEnu(rec.alert.position).e - e.e, frame_.toEnu(rec.alert.position).n - e.n);
    }
    alerts_.push_back(rec);
    const bool real = rec.emitter >= 0 && emitters_[static_cast<std::size_t>(rec.emitter)].alertable;
    say("GMCS <- D%u  %-8s %s evt=%06X  pos err=%.0f m  CEP=%.0f m  conf=%.2f  sensor types=%u observers=%u  via %s  (+%.0f ms)",
        rec.alert.issuer, rec.alert.level == mesh::AlertLevel::PRIORITY ? "PRIORITY" : "ADVISORY",
        real ? emitters_[static_cast<std::size_t>(rec.emitter)].label.c_str() : "FALSE", rec.alert.event_id, rec.err_m,
        static_cast<double>(rec.alert.cep_m), static_cast<double>(rec.alert.confidence),
        mesh::popCount(rec.alert.modality_mask), static_cast<unsigned>(rec.alert.observer_count), linkName(rec.link),
        (rec.t_rx - rec.t_issue) * 1000.0);
  }
  for (std::size_t i = c2_up_.size(); i-- > 0;) {
    if (c2_up_[i].at > now_ms_) continue;
    SimDrone& d = *drones_[static_cast<std::size_t>(c2_up_[i].drone)];
    if (d.alive) d.mgr->applyGmcsDismissal(c2_up_[i].signature);
    c2_up_.erase(c2_up_.begin() + static_cast<std::ptrdiff_t>(i));
  }
}

void World::stepDrone(SimDrone& d) {
  advanceFlight(d, kTickMs / 1000.0);
  d.mgr->updateOwnState(stateOf(d));
  if (d.ticks % 20U == static_cast<std::uint64_t>(d.sensor_offset)) sampleSensors(d);
  d.mgr->tick();
  ++d.ticks;
}

void World::observe() {
  const SimDrone* ref = reference();
  if (ref == nullptr) return;
  for (std::size_t k = 0; k < emitters_.size(); ++k) {
    const EmitterSpec& em = emitters_[k];
    if (!announced_[k] && t() >= em.on_s) {
      announced_[k] = true;
      say("WORLD  %s starts transmitting at (%.1f,%.1f) km", em.label.c_str(), em.e / 1000.0, em.n / 1000.0);
    }
    const DetectedEvent* ev = eventFor(*ref, static_cast<int>(k));
    if (ev == nullptr) continue;
    Seen& s = seen_[k];
    const double err = std::hypot(frame_.toEnu(ev->position).e - em.e, frame_.toEnu(ev->position).n - em.n);
    if (!s.seen) {
      s.seen = true;
      say("MESH   %s first detected; %u observers share readings", em.label.c_str(), static_cast<unsigned>(ev->observer_count));
    }
    if (ev->localized && !s.localized) {
      s.localized = true;
      say("MESH   %s localised: %u observers, CEP %.0f m, actual error %.0f m, conf %.2f, urgency %.2f", em.label.c_str(),
          static_cast<unsigned>(ev->observer_count), static_cast<double>(ev->cep_m), err,
          static_cast<double>(ev->confidence), static_cast<double>(ev->urgency));
    }
    if (ev->localized && ev->cep_m <= 250.0f && !s.fixed && mesh::popCount(ev->modality_mask) >= 2) {
      s.fixed = true;
      say("MESH   %s multi-sensor fix: CEP %.0f m, actual error %.0f m, %u sensor types, conf %.2f", em.label.c_str(),
          static_cast<double>(ev->cep_m), err, mesh::popCount(ev->modality_mask), static_cast<double>(ev->confidence));
    }
    const bool noteworthy = ev->status == mesh::EventStatus::VERIFYING || ev->status == mesh::EventStatus::CONFIRMED ||
                            ev->status == mesh::EventStatus::DISMISSED;
    if (ev->status != s.status && noteworthy) {
      s.status = ev->status;
      say("MESH   %s -> %s (urgency %.2f)", em.label.c_str(), statusName(ev->status), static_cast<double>(ev->urgency));
    }
  }
}

void World::traceFrame() {
  TraceFrame f;
  f.t = t();
  for (const auto& d : drones_) {
    double state = d->commanded == DroneMode::ACTIVE_MISSION ? 1 : 0;
    if (d->mgr->isVerifying()) state = 2;
    if (!d->alive) state = 3;
    f.drones.push_back({static_cast<double>(d->spec.id), d->pos.e, d->pos.n, d->heading_deg, state,
                        static_cast<double>(d->mgr->isVerifying() ? d->mgr->currentTask().event_id : 0)});
  }
  if (const SimDrone* ref = reference()) {
    for (const auto& ev : ref->mgr->table().events()) {
      const Vec2 p = frame_.toEnu(ev.position);
      f.events.push_back({static_cast<double>(ev.id), p.e, p.n, static_cast<double>(ev.cep_m),
                          static_cast<double>(ev.status), ev.localized ? 1.0 : 0.0, static_cast<double>(ev.signature),
                          static_cast<double>(ev.verifier[0].id), static_cast<double>(ev.verifier[1].id),
                          static_cast<double>(ev.observer_count), static_cast<double>(ev.cls)});
    }
  }
  f.bearings = recent_bearings_;
  recent_bearings_.clear();
  for (std::size_t i = 0; i < drones_.size(); ++i) {
    for (std::size_t j = i + 1; j < drones_.size(); ++j) {
      if (radioPath(*drones_[i], *drones_[j])) f.links.push_back({static_cast<int>(i), static_cast<int>(j)});
    }
  }
  trace_.push_back(std::move(f));
}

TrialResult World::run() {
  const auto end_ms = static_cast<TimeMs>(sc_.duration_s * 1000.0);
  const auto trace_ms = static_cast<TimeMs>(std::max(0.1, opt_.trace_period_s) * 1000.0);
  for (TimeMs elapsed = kStepMs; elapsed <= end_ms; elapsed += kStepMs) {
    now_ms_ = kT0 + elapsed;
    runScript();
    deliver();
    for (auto& d : drones_) {
      if (d->alive && (elapsed + static_cast<TimeMs>(d->phase_ms)) % kTickMs == 0) stepDrone(*d);
    }
    if (elapsed % 1000 == 0) observe();
    if (!opt_.trace_path.empty() && elapsed % trace_ms == 0) traceFrame();
  }
  TrialResult r;
  r.seed = opt_.seed;
  evaluate(r);
  if (!opt_.trace_path.empty()) writeTrace(r);
  return r;
}

// ----------------------------------------------------------------- results

void World::evaluate(TrialResult& r) {
  r.alerts = alerts_;
  r.tasks = tasks_;
  r.emitters.assign(emitters_.size(), EmitterOutcome{});
  std::map<std::pair<int, int>, int> per_level;
  for (const auto& a : alerts_) {
    const bool pri = a.alert.level == mesh::AlertLevel::PRIORITY;
    const bool real = a.emitter >= 0 && emitters_[static_cast<std::size_t>(a.emitter)].alertable;
    if (!real) {
      (pri ? r.false_priorities : r.false_advisories) += 1;
      (a.majority_source < 0 ? r.false_from_clutter : r.false_from_multipath) += 1;
      continue;
    }
    if (++per_level[{a.emitter, pri ? 1 : 0}] > 1) ++r.duplicate_alerts;
    EmitterOutcome& o = r.emitters[static_cast<std::size_t>(a.emitter)];
    const double dt = a.t_rx - emitters_[static_cast<std::size_t>(a.emitter)].on_s;
    if (pri) {
      if (o.t_priority < 0) {
        o.t_priority = dt;
        o.err_priority_m = a.err_m;
      }
    } else if (o.t_advisory < 0) {
      o.t_advisory = dt;
    }
  }
  for (auto& o : r.emitters) {  // a PRIORITY also warns: it counts as the first alert if it came first
    if (o.t_priority >= 0 && (o.t_advisory < 0 || o.t_priority < o.t_advisory)) o.t_advisory = o.t_priority;
  }
  r.datagrams = datagrams_;
  r.datagrams_lost = lost_;
  r.bytes = bytes_;
  const auto n = static_cast<double>(drones_.size());
  r.mean_kbps_per_drone = static_cast<double>(bytes_) * 8.0 / 1000.0 / sc_.duration_s / n;
  std::uint64_t peak = 0;
  for (const auto b : bytes_per_s_) peak = std::max(peak, b);
  r.peak_kbps_per_drone = static_cast<double>(peak) * 8.0 / 1000.0 / n;
  for (std::size_t i = 0; i < drones_.size(); ++i) {
    const SimDrone& d = *drones_[i];
    DroneSummary s;
    s.id = d.spec.id;
    s.active = d.commanded == DroneMode::ACTIVE_MISSION;
    s.tasked = d.ever_tasked;
    s.alive = d.alive;
    s.stats = d.mgr->stats();
    s.tx_kbps = static_cast<double>(bytes_by_src_[i]) * 8.0 / 1000.0 / sc_.duration_s;
    r.drones.push_back(s);
  }
  for (const auto& d : drones_) {
    r.max_fcs_requests = std::max(r.max_fcs_requests, static_cast<int>(d->mgr->stats().tasks_issued));
    r.hours_flown += (d->alive ? sc_.duration_s : 0.0) / 3600.0;
  }

  const auto emitterIndex = [&](const std::string& label) {
    for (std::size_t k = 0; k < emitters_.size(); ++k) {
      if (emitters_[k].label == label) return static_cast<int>(k);
    }
    return -1;
  };
  const auto count = [&](int em, mesh::AlertLevel lv) {
    int c = 0;
    for (const auto& a : alerts_) c += (a.emitter == em && a.alert.level == lv) ? 1 : 0;
    return c;
  };
  const auto firstAt = [&](int em, mesh::AlertLevel lv) {
    for (const auto& a : alerts_) {
      if (a.emitter == em && a.alert.level == lv) return a.t_rx;
    }
    return -1.0;
  };
  for (const Expect& x : sc_.expects) {
    CheckResult c;
    c.text = x.text;
    const int em = x.label.empty() ? -1 : emitterIndex(x.label);
    if (x.kind == "advisory_once") {
      c.ok = count(em, mesh::AlertLevel::ADVISORY) == 1;
    } else if (x.kind == "priority_once") {
      c.ok = count(em, mesh::AlertLevel::PRIORITY) == 1;
      if (c.ok && x.value > 0) {
        for (const auto& a : alerts_) {
          if (a.emitter == em && a.alert.level == mesh::AlertLevel::PRIORITY) c.ok = a.err_m >= 0 && a.err_m < x.value;
        }
      }
    } else if (x.kind == "priority_after_advisory") {
      const double ta = firstAt(em, mesh::AlertLevel::ADVISORY), tp = firstAt(em, mesh::AlertLevel::PRIORITY);
      c.ok = ta >= 0 && tp > ta;
    } else if (x.kind == "never_tasked") {
      c.ok = std::none_of(tasks_.begin(), tasks_.end(), [&](const TaskRecord& tr) { return tr.drone == x.drone; });
    } else if (x.kind == "reassigned_after_loss") {
      const double tl = loss_t_[static_cast<std::size_t>(em)];
      c.ok = tl >= 0 && std::any_of(tasks_.begin(), tasks_.end(), [&](const TaskRecord& tr) {
               return tr.emitter == em && tr.t > tl && tr.drone != loss_victim_[static_cast<std::size_t>(em)];
             });
    } else if (x.kind == "quiet") {
      c.ok = std::none_of(tasks_.begin(), tasks_.end(), [&](const TaskRecord& tr) { return tr.emitter == em; }) &&
             std::none_of(alerts_.begin(), alerts_.end(), [&](const AlertRecord& a) { return a.emitter == em; });
    } else if (x.kind == "dismissal_learned") {
      c.ok = true;
      for (const auto& d : drones_) {
        if (d->alive && !d->mgr->table().isBenign(emitters_[static_cast<std::size_t>(em)].sig)) c.ok = false;
      }
    } else if (x.kind == "all_released") {
      c.ok = std::none_of(drones_.begin(), drones_.end(), [](const auto& d) { return d->alive && d->mgr->isVerifying(); });
    } else if (x.kind == "max_fcs_requests") {
      c.ok = r.max_fcs_requests <= static_cast<int>(x.value);
    } else if (x.kind == "advisory_within") {
      c.ok = r.emitters[static_cast<std::size_t>(em)].t_advisory >= 0 &&
             r.emitters[static_cast<std::size_t>(em)].t_advisory <= x.value;
    } else if (x.kind == "priority_within") {
      c.ok = r.emitters[static_cast<std::size_t>(em)].t_priority >= 0 &&
             r.emitters[static_cast<std::size_t>(em)].t_priority <= x.value;
    } else if (x.kind == "max_false_alerts") {
      c.ok = r.false_advisories + r.false_priorities <= static_cast<int>(x.value);
    } else if (x.kind == "converged") {
      // Every live drone holds the same events under the same ids.
      std::set<EventId> ref_ids;
      bool first = true;
      c.ok = true;
      for (const auto& d : drones_) {
        if (!d->alive) continue;
        std::set<EventId> ids;
        for (const auto& ev : d->mgr->table().events()) ids.insert(ev.id);
        if (first) {
          ref_ids = ids;
          first = false;
        } else if (ids != ref_ids) {
          c.ok = false;
        }
      }
    }
    r.checks.push_back(c);
  }
}

// ------------------------------------------------------------------- trace

void World::writeTrace(const TrialResult& r) const {
  FILE* f = std::fopen(opt_.trace_path.c_str(), "w");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot write %s\n", opt_.trace_path.c_str());
    return;
  }
  const auto str = [f](const std::string& s) {
    std::fputc('"', f);
    for (const char ch : s) {
      if (ch == '"' || ch == '\\') {
        std::fputc('\\', f);
        std::fputc(ch, f);
      } else if (static_cast<unsigned char>(ch) < 0x20) {
        std::fprintf(f, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(ch)));
      } else {
        std::fputc(ch, f);
      }
    }
    std::fputc('"', f);
  };
  std::fprintf(f, "{\"format\":1,\"scenario\":");
  str(sc_.name);
  std::fprintf(f, ",\"seed\":%u,\"duration\":%.1f,\"altitude\":%.0f,\"df_range\":%.0f,\"radio_range\":%.0f,",
               opt_.seed, sc_.duration_s, sc_.altitude_m, sc_.df.range_m, sc_.radio.range_m);
  std::fprintf(f, "\"terrain\":{\"hills\":[");
  for (std::size_t i = 0; i < sc_.hills.size(); ++i) {
    const Hill& h = sc_.hills[i];
    std::fprintf(f, "%s[%.0f,%.0f,%.0f,%.0f]", i ? "," : "", h.e, h.n, h.height_m, h.radius_m);
  }
  std::fprintf(f, "],\"ridges\":[");
  for (std::size_t i = 0; i < sc_.ridges.size(); ++i) {
    const Ridge& g = sc_.ridges[i];
    std::fprintf(f, "%s[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]", i ? "," : "", g.e1, g.n1, g.e2, g.n2, g.height_m, g.width_m);
  }
  std::fprintf(f, "]},\"emitters\":[");
  for (std::size_t i = 0; i < emitters_.size(); ++i) {
    const EmitterSpec& e = emitters_[i];
    std::fprintf(f, "%s{\"label\":", i ? "," : "");
    str(e.label);
    std::fprintf(f, ",\"sig\":%u,\"e\":%.0f,\"n\":%.0f,\"on\":%.1f,\"off\":%.1f,\"cls\":%d,\"thermal\":%s,\"alertable\":%s}",
                 e.sig, e.e, e.n, e.on_s, std::min(e.off_s, sc_.duration_s + 1), static_cast<int>(e.cls),
                 e.thermal ? "true" : "false", e.alertable ? "true" : "false");
  }
  std::fprintf(f, "],\"drones\":[");
  for (std::size_t i = 0; i < drones_.size(); ++i) {
    const DroneSpec& s = drones_[i]->spec;
    std::fprintf(f, "%s{\"id\":%d,\"active\":%s,\"sensors\":%u,\"links\":%u,\"endurance\":%.0f,\"bingo\":%.0f}", i ? "," : "",
                 s.id, s.mode == DroneMode::ACTIVE_MISSION ? "true" : "false", s.sensors, s.links, s.endurance_s, s.bingo_s);
  }
  std::fprintf(f, "],\"frames\":[");
  for (std::size_t i = 0; i < trace_.size(); ++i) {
    const TraceFrame& fr = trace_[i];
    std::fprintf(f, "%s{\"t\":%.1f,\"d\":[", i ? ",\n" : "\n", fr.t);
    for (std::size_t k = 0; k < fr.drones.size(); ++k) {
      const auto& d = fr.drones[k];
      std::fprintf(f, "%s[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]", k ? "," : "", d[0], d[1], d[2], d[3], d[4], d[5]);
    }
    std::fprintf(f, "],\"ev\":[");
    for (std::size_t k = 0; k < fr.events.size(); ++k) {
      const auto& e = fr.events[k];
      std::fprintf(f, "%s[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]", k ? "," : "", e[0], e[1], e[2], e[3],
                   e[4], e[5], e[6], e[7], e[8], e[9], e[10]);
    }
    std::fprintf(f, "],\"b\":[");
    for (std::size_t k = 0; k < fr.bearings.size(); ++k) {
      const auto& b = fr.bearings[k];
      std::fprintf(f, "%s[%.0f,%.0f,%.0f,%.1f,%.0f]", k ? "," : "", b[0], b[1], b[2], b[3], b[4]);
    }
    std::fprintf(f, "],\"lk\":[");
    for (std::size_t k = 0; k < fr.links.size(); ++k) std::fprintf(f, "%s%d,%d", k ? "," : "", fr.links[k][0], fr.links[k][1]);
    std::fprintf(f, "]}");
  }
  std::fprintf(f, "],\n\"alerts\":[");
  for (std::size_t i = 0; i < r.alerts.size(); ++i) {
    const AlertRecord& a = r.alerts[i];
    const Vec2 p = frame_.toEnu(a.alert.position);
    std::fprintf(f, "%s{\"t_issue\":%.2f,\"t_rx\":%.2f,\"level\":%d,\"id\":%u,\"e\":%.0f,\"n\":%.0f,\"cep\":%.0f,\"conf\":%.2f,"
                    "\"issuer\":%u,\"link\":%d,\"emitter\":%d,\"err\":%.0f,\"modalities\":%u,\"observers\":%u}",
                 i ? "," : "", a.t_issue, a.t_rx, static_cast<int>(a.alert.level), a.alert.event_id, p.e, p.n,
                 static_cast<double>(a.alert.cep_m), static_cast<double>(a.alert.confidence), a.alert.issuer,
                 linkIndex(a.link), a.emitter, a.err_m, mesh::popCount(a.alert.modality_mask),
                 static_cast<unsigned>(a.alert.observer_count));
  }
  std::fprintf(f, "],\n\"tasks\":[");
  for (std::size_t i = 0; i < r.tasks.size(); ++i) {
    const TaskRecord& tr = r.tasks[i];
    std::fprintf(f, "%s{\"t\":%.1f,\"drone\":%d,\"event\":%u,\"role\":%u,\"cue\":%u,\"emitter\":%d,\"se\":%.0f,\"sn\":%.0f}",
                 i ? "," : "", tr.t, tr.drone, tr.event, static_cast<unsigned>(tr.role), static_cast<unsigned>(tr.cue),
                 tr.emitter, tr.se, tr.sn);
  }
  std::fprintf(f, "],\n\"log\":[");
  for (std::size_t i = 0; i < log_.size(); ++i) {
    std::fprintf(f, "%s[%.1f,", i ? ",\n" : "\n", log_[i].first);
    str(log_[i].second);
    std::fputc(']', f);
  }
  std::fprintf(f, "],\n\"checks\":[");
  for (std::size_t i = 0; i < r.checks.size(); ++i) {
    std::fprintf(f, "%s{\"text\":", i ? "," : "");
    str(r.checks[i].text);
    std::fprintf(f, ",\"ok\":%s}", r.checks[i].ok ? "true" : "false");
  }
  std::fprintf(f, "]}\n");
  std::fclose(f);
}

}  // namespace

TrialResult runScenario(const Scenario& sc, const RunOptions& opt) {
  World w(sc, opt);
  return w.run();
}

}  // namespace sim
