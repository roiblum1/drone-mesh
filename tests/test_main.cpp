// Unit tests for the Model 8270 pack mesh. No external framework.
// The global operator new is replaced so the test can prove that the
// steady-state mesh loop never touches the heap.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>

#include "amc8270/mesh/crypto.hpp"
#include "amc8270/mesh/geo.hpp"
#include "amc8270/mesh/mesh_security.hpp"
#include "amc8270/mesh/pack_mesh_manager.hpp"
#include "amc8270/mesh/validation.hpp"

using namespace amc8270::mesh;

// ---------------------------------------------------------------- heap probe
static std::size_t g_allocs = 0;
void* operator new(std::size_t n) {
  ++g_allocs;
  void* p = std::malloc(n != 0 ? n : 1);
  if (p == nullptr) std::abort();
  return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

// ------------------------------------------------------------------- harness
static int g_checks = 0;
static int g_failures = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    ++g_checks;                                                           \
    if (!(cond)) {                                                        \
      ++g_failures;                                                       \
      std::printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
    }                                                                     \
  } while (0)

struct TestCase {
  const char* name;
  void (*fn)();
};
static std::array<TestCase, 128> g_tests{};
static std::size_t g_n_tests = 0;
struct Registrar {
  Registrar(const char* n, void (*f)()) { g_tests[g_n_tests++] = {n, f}; }
};
#define TEST(name)                      \
  static void name();                   \
  static Registrar reg_##name(#name, name); \
  static void name()

// ------------------------------------------------------------------ fixtures
static const GeoPoint kOrigin{36.0, -12.0, 0.0f};
static GeoPoint enu(double e, double n, float alt = 3000.0f) {
  static const LocalFrame frame(kOrigin);
  return frame.fromEnu(Vec2{e, n}, alt);
}

// Test keys only. Byte pattern makes two keys with different seeds differ everywhere.
static crypto::Key testKey(std::uint8_t seed) {
  crypto::Key k{};
  for (std::size_t i = 0; i < k.size(); ++i) k[i] = static_cast<std::uint8_t>(seed * 31U + i * 7U + 1U);
  return k;
}
static const crypto::Key kPackKey = testKey(1);
constexpr std::uint16_t kPackEpoch = 1;

struct FakeClock : IClock {
  TimeMs now = 1'700'000'000'000ULL;
  TimeMs nowMs() const override { return now; }
};

struct Datagram {
  std::array<std::uint8_t, kMaxPacketBytes> data{};
  std::size_t len = 0;
};

constexpr int kMaxNodes = 8;

class Bus {
 public:
  struct Node : IMeshTransceiver {
    Bus* bus = nullptr;
    int idx = 0;
    std::array<Datagram, 256> q{};
    std::size_t head = 0;
    std::size_t tail = 0;
    bool send(const std::uint8_t* d, std::size_t n) override { return bus->broadcastFrom(idx, d, n); }
    std::size_t receive(std::uint8_t* buf, std::size_t cap) override {
      if (head == tail) return 0;
      const Datagram& g = q[head % q.size()];
      ++head;
      const std::size_t n = g.len < cap ? g.len : cap;
      std::memcpy(buf, g.data.data(), n);
      return n;
    }
  };

  Bus() {
    for (int i = 0; i < kMaxNodes; ++i) {
      nodes[static_cast<std::size_t>(i)].bus = this;
      nodes[static_cast<std::size_t>(i)].idx = i;
    }
  }

  bool broadcastFrom(int src, const std::uint8_t* d, std::size_t n) {
    if (src == record_src && tape_n < tape.size()) {
      std::memcpy(tape[tape_n].data.data(), d, n);
      tape[tape_n].len = n;
      ++tape_n;
    }
    if (isolated[static_cast<std::size_t>(src)]) return true;
    for (int dst = 0; dst < kMaxNodes; ++dst) {
      if (dst == src || isolated[static_cast<std::size_t>(dst)]) continue;
      if (group[static_cast<std::size_t>(dst)] != group[static_cast<std::size_t>(src)]) continue;
      if (drop_permille != 0) {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        if (rng % 1000u < drop_permille) continue;
      }
      Node& nd = nodes[static_cast<std::size_t>(dst)];
      if (nd.tail - nd.head >= nd.q.size()) continue;  // inbox full
      Datagram& g = nd.q[nd.tail % nd.q.size()];
      std::memcpy(g.data.data(), d, n);
      g.len = n;
      ++nd.tail;
    }
    return true;
  }

  std::array<Node, kMaxNodes> nodes{};
  std::array<bool, kMaxNodes> isolated{};
  // Nodes in different groups cannot hear each other (a mesh partition).
  std::array<int, kMaxNodes> group{};
  unsigned drop_permille = 0;
  // Every datagram node `record_src` sends is also copied onto the tape.
  int record_src = -1;
  std::array<Datagram, 128> tape{};
  std::size_t tape_n = 0;
  std::uint32_t rng = 2463534242u;
};

struct FakeGmcs : IGmcsLink {
  std::array<GmcsAlert, 32> alerts{};
  std::array<LinkType, 32> links{};
  std::size_t n = 0;
  bool fail = false;
  bool sendAlert(const GmcsAlert& a, LinkType l) override {
    if (fail || n >= alerts.size()) return false;
    alerts[n] = a;
    links[n] = l;
    ++n;
    return true;
  }
  std::size_t count(AlertLevel lv) const {
    std::size_t c = 0;
    for (std::size_t i = 0; i < n; ++i) c += alerts[i].level == lv ? 1 : 0;
    return c;
  }
};

struct FakeGuidance : IGuidanceSink {
  VerificationTask last{};
  int requests = 0;
  int cancels = 0;
  void requestVerification(const VerificationTask& t) override {
    last = t;
    ++requests;
  }
  void cancelVerification() override { ++cancels; }
};

struct Pack {
  FakeClock clock;
  Bus bus;
  std::array<FakeGmcs, kMaxNodes> gmcs{};
  std::array<FakeGuidance, kMaxNodes> guidance{};
  std::array<std::unique_ptr<PackMeshManager>, kMaxNodes> mgr{};
  std::size_t n = 0;

  explicit Pack(std::size_t count, const MeshConfig& cfg = MeshConfig{}) : n(count) {
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t k = i;
      mgr[i] = std::make_unique<PackMeshManager>(static_cast<DroneId>(i + 1), cfg, 1U, clock, bus.nodes[k], gmcs[k],
                                                 guidance[k]);
      if (!mgr[i]->installEpochKey(kPackEpoch, kPackKey, 0, kNoExpiry)) std::abort();
    }
  }

  // Power-cycle drone i+1 with a new boot session (its tables start empty).
  void reboot(std::size_t i, std::uint32_t session, const MeshConfig& cfg = MeshConfig{}) {
    mgr[i] = std::make_unique<PackMeshManager>(static_cast<DroneId>(i + 1), cfg, session, clock, bus.nodes[i], gmcs[i],
                                               guidance[i]);
    if (!mgr[i]->installEpochKey(kPackEpoch, kPackKey, 0, kNoExpiry)) std::abort();
  }

  // Drone i+1 at (e, n); default full fuel, RF_DF + IR.
  void place(std::size_t i, double e, double nn, DroneMode mode = DroneMode::PACK_ROUTING,
             SensorMask sensors = maskOf(SensorType::RF_DF) | maskOf(SensorType::IR), float endurance = 4.0f * 3600.0f,
             float bingo = 1800.0f) {
    DroneState s;
    s.id = static_cast<DroneId>(i + 1);
    s.mode = mode;
    s.installed_sensors = sensors;
    s.available_links = kLinkMaskAll;
    s.active_sensor = SensorType::RF_DF;
    s.position = enu(e, nn);
    s.speed_mps = 40.0f;
    s.heading_deg = 90.0f;
    s.endurance_remaining_s = endurance;
    s.bingo_endurance_s = bingo;
    mgr[i]->updateOwnState(s);
  }

  void run(TimeMs ms) {
    for (TimeMs t = 0; t < ms; t += 100) {
      clock.now += 100;
      for (std::size_t i = 0; i < n; ++i) mgr[i]->tick();
    }
  }

  std::size_t totalAlerts(AlertLevel lv) const {
    std::size_t c = 0;
    for (std::size_t i = 0; i < n; ++i) c += gmcs[i].count(lv);
    return c;
  }
};

static SensorObservation bearingObs(const GeoPoint& from, const GeoPoint& truth, double noise_deg = 0.0,
                                    std::uint32_t sig = 406025) {
  SensorObservation o;
  o.sensor = SensorType::RF_DF;
  o.class_hint = EventClass::DISTRESS_BEACON;
  o.has_fix = false;
  o.bearing_deg = static_cast<float>(bearingDeg(from, truth) + noise_deg);
  o.bearing_sigma_deg = 2.5f;
  o.confidence = 0.5f;
  o.signature = sig;
  return o;
}

static SensorObservation fixObs(SensorType s, const GeoPoint& at, float cep, float conf = 0.8f) {
  SensorObservation o;
  o.sensor = s;
  o.class_hint = EventClass::UNKNOWN;
  o.has_fix = true;
  o.fix = at;
  o.fix_cep_m = cep;
  o.confidence = conf;
  return o;
}

// 5-drone line abreast, truth beacon east of the line (good crossing geometry).
static const double kTruthE = 10000.0;
static const double kTruthN = 2000.0;

static void placeLine(Pack& p) {
  p.place(0, 0, -12000, DroneMode::ACTIVE_MISSION, maskOf(SensorType::RF_DF) | maskOf(SensorType::EO) | maskOf(SensorType::IR));
  p.place(1, 0, -6000);
  p.place(2, 0, 0, DroneMode::PACK_ROUTING, maskOf(SensorType::RF_DF) | maskOf(SensorType::EO));
  p.place(3, 0, 6000);
  p.place(4, 0, 12000, DroneMode::PACK_ROUTING, maskOf(SensorType::RF_DF) | maskOf(SensorType::IR), 1500.0f, 1300.0f);  // near bingo
}

static void reportBeacon(Pack& p, std::initializer_list<std::size_t> who, double noise = 0.0) {
  const GeoPoint truth = enu(kTruthE, kTruthN);
  for (std::size_t i : who) {
    const SensorObservation o = bearingObs(p.mgr[i]->exportedState().position, truth, noise);
    CHECK(p.mgr[i]->reportLocalDetection(o));
  }
}

static const DetectedEvent* soleEvent(const PackMeshManager& m) {
  return m.table().events().size() == 1 ? &m.table().events()[0] : nullptr;
}

// --------------------------------------------------------------------- tests

TEST(static_vector_basics) {
  StaticVector<int, 3> v;
  CHECK(v.empty());
  CHECK(v.push_back(1));
  CHECK(v.push_back(2));
  CHECK(v.push_back(3));
  CHECK(v.full());
  CHECK(!v.push_back(4));
  v.erase_at(1);
  CHECK(v.size() == 2 && v[0] == 1 && v[1] == 3);
  v.clear();
  CHECK(v.empty());
}

TEST(geo_round_trip) {
  const GeoPoint a = enu(0, 0);
  const GeoPoint b = destinationPoint(a, 63.0, 12345.0);
  CHECK(std::fabs(distanceM(a, b) - 12345.0) < 0.5);
  CHECK(std::fabs(bearingDeg(a, b) - 63.0) < 0.01);
  CHECK(angleDiffDeg(359.0, 1.0) == 2.0);
  const LocalFrame f(kOrigin);
  const Vec2 v = f.toEnu(f.fromEnu(Vec2{1234.5, -987.6}, 0.0f));
  CHECK(std::fabs(v.e - 1234.5) < 1e-6 && std::fabs(v.n + 987.6) < 1e-6);
}

TEST(triangulation_exact_and_weighted) {
  const LocalFrame f(kOrigin);
  const Vec2 truth{10000.0, 2000.0};
  const Vec2 obs[3] = {{0.0, -6000.0}, {0.0, 0.0}, {0.0, 6000.0}};
  std::array<BearingLine, 3> lines{};
  for (std::size_t i = 0; i < 3; ++i) {
    lines[i].origin = obs[i];
    lines[i].bearing_rad = std::atan2(truth.e - obs[i].e, truth.n - obs[i].n);
    lines[i].sigma_rad = 0.0436;
  }
  TriangulationResult r;
  CHECK(triangulate(lines.data(), 3, 0.03, r));
  CHECK(std::fabs(r.position.e - truth.e) < 0.5 && std::fabs(r.position.n - truth.n) < 0.5);
  CHECK(r.cep_m > 100.0 && r.cep_m < 1500.0);
  (void)f;

  // Noisy bearings: error stays within a few CEPs.
  lines[0].bearing_rad += 0.03;
  lines[2].bearing_rad -= 0.03;
  CHECK(triangulate(lines.data(), 3, 0.03, r));
  const double err = std::hypot(r.position.e - truth.e, r.position.n - truth.n);
  CHECK(err < 3.0 * r.cep_m);
}

TEST(triangulation_rejects_bad_geometry) {
  TriangulationResult r;
  std::array<BearingLine, 2> l{};
  // Near-parallel (1 degree apart).
  l[0].origin = Vec2{0.0, 0.0};
  l[1].origin = Vec2{0.0, 5000.0};
  l[0].bearing_rad = 1.0;
  l[1].bearing_rad = 1.0 + 0.017;
  CHECK(!triangulate(l.data(), 2, 0.03, r));
  // Single line.
  CHECK(!triangulate(l.data(), 1, 0.03, r));
  CHECK(!triangulate(nullptr, 2, 0.03, r));
  // Crossing point behind the observers (lines diverge).
  l[0].origin = Vec2{0.0, 0.0};
  l[1].origin = Vec2{0.0, 4000.0};
  l[0].bearing_rad = 3.14159265358979 * 0.75;   // SE
  l[1].bearing_rad = 3.14159265358979 * 0.25;   // NE ... lines cross at x<0 only if extended backwards
  CHECK(!triangulate(l.data(), 2, 0.03, r));
}

TEST(peer_discovery_and_timeout) {
  auto p = std::make_unique<Pack>(3);
  for (std::size_t i = 0; i < 3; ++i) p->place(i, 0, static_cast<double>(i) * 5000.0);
  p->run(3000);
  for (std::size_t i = 0; i < 3; ++i) {
    CHECK(p->mgr[i]->table().peerCount() == 2);
    CHECK(p->mgr[i]->stats().peers_discovered == 2);
  }
  const PeerRecord* r = p->mgr[0]->table().findPeer(2);
  CHECK(r != nullptr && r->state.position.alt_m == 3000.0f);

  // Drone 3 goes silent: the others drop it after the timeout.
  p->bus.isolated[2] = true;
  p->run(8000);
  CHECK(p->mgr[0]->table().peerCount() == 1);
  CHECK(p->mgr[0]->table().findPeer(3) == nullptr);
  CHECK(p->mgr[0]->stats().peers_lost >= 1);
  CHECK(p->mgr[1]->table().peerCount() == 1);

  // It comes back and is rediscovered.
  p->bus.isolated[2] = false;
  p->run(3000);
  CHECK(p->mgr[0]->table().peerCount() == 2);
}

TEST(leave_removes_peer) {
  auto p = std::make_unique<Pack>(2);
  p->place(0, 0, 0);
  p->place(1, 0, 5000);
  p->run(2000);
  CHECK(p->mgr[0]->table().peerCount() == 1);
  p->mgr[1]->shutdown();
  for (int i = 0; i < 3; ++i) {  // only the remaining drone keeps ticking
    p->clock.now += 100;
    p->mgr[0]->tick();
  }
  CHECK(p->mgr[0]->table().peerCount() == 0);
  CHECK(p->mgr[0]->stats().peers_lost == 1);
}

TEST(cooperative_triangulation_over_mesh) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  reportBeacon(*p, {1, 2, 3, 4});
  p->run(1000);
  const GeoPoint truth = enu(kTruthE, kTruthN);
  for (std::size_t i = 0; i < 5; ++i) {
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr);
    if (ev == nullptr) continue;
    CHECK(ev->localized);
    CHECK(ev->observations.size() == 4);
    CHECK(distanceM(ev->position, truth) < 30.0);  // noise-free bearings
    CHECK(ev->cep_m > 50.0f && ev->cep_m < 2000.0f);
    CHECK(ev->cls == EventClass::DISTRESS_BEACON);
    CHECK(ev->confidence > 0.8f && ev->confidence <= 0.85f + 1e-6f);  // single modality is capped
  }
}

TEST(tables_converge_on_one_event_id) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  // Two drones file the same source before hearing each other: two local ids.
  p->bus.isolated[1] = true;
  p->bus.isolated[3] = true;
  reportBeacon(*p, {1, 3});
  CHECK(p->mgr[1]->table().events().size() == 1);
  CHECK(p->mgr[3]->table().events().size() == 1);
  const EventId id_a = p->mgr[1]->table().events()[0].id;
  const EventId id_b = p->mgr[3]->table().events()[0].id;
  CHECK(id_a != id_b);

  p->bus.isolated[1] = false;
  p->bus.isolated[3] = false;
  p->run(6000);  // anti-entropy re-sends both observations
  const EventId lowest = id_a < id_b ? id_a : id_b;
  for (std::size_t i = 0; i < 5; ++i) {
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr);
    if (ev != nullptr) CHECK(ev->id == lowest);
    CHECK(p->mgr[i]->table().resolve(lowest) == lowest);
  }
  // The drone that filed the source under the higher id renamed it and keeps
  // an alias, so a late message that still quotes the old id resolves to the survivor.
  const EventId highest = id_a < id_b ? id_b : id_a;
  const std::size_t renamer = id_a < id_b ? 3 : 1;
  CHECK(p->mgr[renamer]->table().resolve(highest) == lowest);
}

TEST(claim_arbitration) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  SensorObservation o = bearingObs(enu(0, 0), enu(5000, 0));
  o.observer = 2;
  o.observer_position = enu(0, 0);
  o.timestamp_ms = 1000;
  o.event_hint = 0x00020001u;
  const EventId id = t.ingestObservation(o, 1000);
  CHECK(id == 0x00020001u);

  CHECK(t.applyClaim(id, 3, 0, 0.60f, 1000) == ClaimResult::ACCEPTED);
  CHECK(t.findEvent(id)->verifier[0].id == 3);
  CHECK(t.applyClaim(id, 4, 0, 0.50f, 1001) == ClaimResult::REJECTED);  // lower score loses
  CHECK(t.findEvent(id)->verifier[0].id == 3);
  CHECK(t.applyClaim(id, 4, 0, 0.80f, 1002) == ClaimResult::ACCEPTED);  // higher wins
  CHECK(t.findEvent(id)->verifier[0].id == 4);
  CHECK(t.applyClaim(id, 2, 0, 0.80f, 1003) == ClaimResult::ACCEPTED);  // tie -> lower id
  CHECK(t.findEvent(id)->verifier[0].id == 2);
  CHECK(t.applyClaim(id, 9, 0, 0.80f, 1004) == ClaimResult::REJECTED);
  // A claimant moving to the other role vacates the first.
  CHECK(t.applyClaim(id, 2, 1, 0.70f, 1005) == ClaimResult::ACCEPTED);
  CHECK(t.findEvent(id)->verifier[0].id == kNoDrone && t.findEvent(id)->verifier[1].id == 2);
  CHECK(t.applyClaim(0xDEAD, 2, 0, 0.5f, 0) == ClaimResult::UNKNOWN_EVENT);
}

TEST(verifier_selection_excludes_active_mission_and_low_fuel) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  reportBeacon(*p, {1, 2, 3, 4});
  p->run(3000);

  // Drone 1 (ACTIVE_MISSION) and drone 5 (fuel at bingo) are never tasked.
  CHECK(p->guidance[0].requests == 0);
  CHECK(p->guidance[4].requests == 0);
  CHECK(!p->mgr[0]->isVerifying() && !p->mgr[4]->isVerifying());
  int tasked = 0;
  for (std::size_t i = 1; i < 4; ++i) tasked += p->mgr[i]->isVerifying() ? 1 : 0;
  CHECK(tasked >= 1);

  // Every drone agrees on who holds the slots.
  const DetectedEvent* ref = soleEvent(*p->mgr[0]);
  CHECK(ref != nullptr);
  for (std::size_t i = 1; i < 5 && ref != nullptr; ++i) {
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr);
    if (ev == nullptr) continue;
    CHECK(ev->verifier[0].id == ref->verifier[0].id);
    CHECK(ev->verifier[1].id == ref->verifier[1].id);
    CHECK(ev->verifier[0].id != 1 && ev->verifier[0].id != 5);
    CHECK(ev->verifier[1].id != 1 && ev->verifier[1].id != 5);
  }
  // The assigned drone received a task with a sensible standoff and a cue.
  for (std::size_t i = 1; i < 4; ++i) {
    if (!p->mgr[i]->isVerifying()) continue;
    const VerificationTask& t = p->guidance[i].last;
    CHECK(t.sensor_to_cue == SensorType::IR || t.sensor_to_cue == SensorType::EO);
    const GeoPoint truth = enu(kTruthE, kTruthN);
    CHECK(std::fabs(distanceM(t.standoff, t.target) - 6000.0) < 5.0);
    CHECK(distanceM(t.target, truth) < 50.0);
    CHECK(p->mgr[i]->exportedState().mode == DroneMode::VERIFYING);
  }
}

TEST(fcs_is_not_re_tasked_by_bearing_noise) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  // 20 rounds of noisy DF bearings (+-2 deg) keep nudging the fused position by a few hundred metres.
  for (int round = 0; round < 20; ++round) {
    const GeoPoint truth = enu(kTruthE, kTruthN);
    for (std::size_t i = 1; i < 5; ++i) {
      const double noise = static_cast<double>((round * 7 + static_cast<int>(i) * 3) % 5 - 2);
      CHECK(p->mgr[i]->reportLocalDetection(bearingObs(p->mgr[i]->exportedState().position, truth, noise)));
    }
    p->run(2000);
  }
  int total_requests = 0;
  for (std::size_t i = 0; i < 5; ++i) {
    total_requests += p->guidance[i].requests;
    CHECK(p->guidance[i].requests <= 2);
    CHECK(p->guidance[i].cancels == 0);
  }
  CHECK(total_requests >= 1);  // somebody was tasked
}

TEST(same_table_gives_same_plan) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  reportBeacon(*p, {1, 2, 3, 4});
  p->run(2000);
  std::array<AssignmentList, 5> plans{};
  for (std::size_t i = 0; i < 5; ++i) {
    DroneList dl;
    p->mgr[i]->table().snapshotDrones(dl);
    planAssignments(p->mgr[i]->table(), dl, p->clock.now, MeshConfig{}, plans[i]);
  }
  CHECK(plans[0].size() >= 1);
  for (std::size_t i = 1; i < 5; ++i) {
    CHECK(plans[i].size() == plans[0].size());
    for (std::size_t k = 0; k < plans[0].size() && k < plans[i].size(); ++k) {
      CHECK(plans[i][k].event_id == plans[0][k].event_id);
      CHECK(plans[i][k].drone == plans[0][k].drone);
      CHECK(plans[i][k].role == plans[0][k].role);
      CHECK(std::fabs(plans[i][k].score - plans[0][k].score) < 1e-4f);
    }
  }
}

TEST(diversion_budget_keeps_pack_scanning) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  // Six independent, urgent, well-localised sources.
  for (std::uint32_t k = 0; k < 6; ++k) {
    SensorObservation o = fixObs(SensorType::IR, enu(2000.0 * k, 20000.0), 60.0f, 0.9f);
    o.class_hint = EventClass::THERMAL_HAZARD;
    o.observer = 2;
    o.observer_position = enu(0, 0);
    o.timestamp_ms = now;
    o.event_hint = 0x00020000u + k + 1u;
    CHECK(t.ingestObservation(o, now) != kNoEvent);
  }
  t.refresh(now);
  updateUrgencies(t, now);

  DroneList drones;
  for (DroneId id = 1; id <= 6; ++id) {
    DroneState d;
    d.id = id;
    d.mode = DroneMode::PACK_ROUTING;
    d.installed_sensors = maskOf(SensorType::RF_DF) | maskOf(SensorType::EO);
    d.position = enu(0, -3000.0 * id);
    d.speed_mps = 40.0f;
    d.endurance_remaining_s = 4.0f * 3600.0f;
    d.bingo_endurance_s = 1800.0f;
    CHECK(drones.push_back(d));
  }
  AssignmentList plan;
  planAssignments(t, drones, now, cfg, plan);
  // floor(0.4 * 6) = 2 drones diverted, the other four keep scanning.
  CHECK(plan.size() == 2);
  for (std::size_t i = 0; i < plan.size(); ++i) {
    for (std::size_t j = i + 1; j < plan.size(); ++j) CHECK(plan[i].drone != plan[j].drone);
  }
}

TEST(fuel_and_geofence_feasibility) {
  MeshConfig cfg;
  DetectedEvent ev;
  ev.id = 1;
  ev.position = enu(0, 30000);
  ev.localized = true;
  ev.cep_m = 100.0f;
  DroneState d;
  d.id = 2;
  d.mode = DroneMode::PACK_ROUTING;
  d.installed_sensors = maskOf(SensorType::IR);
  d.position = enu(0, 0);
  d.speed_mps = 40.0f;
  d.bingo_endurance_s = 1800.0f;
  Suitability s;

  d.endurance_remaining_s = 4.0f * 3600.0f;
  CHECK(computeSuitability(d, ev, nullptr, 0, false, cfg, s));
  CHECK(s.fuel_margin_s > 0.0f && s.score > 0.0f && s.sensor_to_cue == SensorType::IR);

  d.endurance_remaining_s = 1800.0f + 100.0f;  // below reserve + detour + dwell
  CHECK(!computeSuitability(d, ev, nullptr, 0, false, cfg, s));

  d.endurance_remaining_s = 4.0f * 3600.0f;
  cfg.geofence_center = enu(0, 0);
  cfg.geofence_radius_m = 20000.0;  // standoff ~24-36 km away is outside the approved corridor
  CHECK(!computeSuitability(d, ev, nullptr, 0, false, cfg, s));
  cfg.geofence_radius_m = 0.0;
  cfg.max_detour_m = 5000.0;
  CHECK(!computeSuitability(d, ev, nullptr, 0, false, cfg, s));
}

TEST(approach_bearing_spreads_verifiers) {
  MeshConfig cfg;
  DetectedEvent ev;
  ev.id = 1;
  ev.position = enu(0, 0);
  ev.localized = true;
  DroneState d;
  d.id = 2;
  d.mode = DroneMode::PACK_ROUTING;
  d.installed_sensors = maskOf(SensorType::IR);
  d.position = enu(-8000, 0);  // west of the target, azimuth 270
  d.speed_mps = 40.0f;
  d.endurance_remaining_s = 4.0f * 3600.0f;
  d.bingo_endurance_s = 1800.0f;
  Suitability s;
  const double covered[1] = {270.0};  // someone already looks from the west
  CHECK(computeSuitability(d, ev, covered, 1, false, cfg, s));
  CHECK(angleDiffDeg(static_cast<double>(s.approach_bearing_deg), 270.0) >= 90.0);
}

TEST(advisory_then_priority_exactly_once) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  reportBeacon(*p, {1, 2, 3, 4});
  p->run(5000);
  // RF-only picture: early-warning ADVISORY, once, across the whole pack.
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) == 1);
  CHECK(p->totalAlerts(AlertLevel::PRIORITY) == 0);

  // Two verifiers close in and add geolocated imaging fixes.
  const GeoPoint truth = enu(kTruthE + 80.0, kTruthN - 50.0);
  CHECK(p->mgr[1]->reportLocalDetection(fixObs(SensorType::IR, truth, 60.0f, 0.8f)));
  CHECK(p->mgr[3]->reportLocalDetection(fixObs(SensorType::IR, enu(kTruthE - 40.0, kTruthN + 30.0), 60.0f, 0.8f)));
  p->run(8000);

  CHECK(p->totalAlerts(AlertLevel::PRIORITY) == 1);
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) == 1);  // no second advisory after PRIORITY

  // Alert content.
  for (std::size_t i = 0; i < 5; ++i) {
    for (std::size_t k = 0; k < p->gmcs[i].n; ++k) {
      const GmcsAlert& a = p->gmcs[i].alerts[k];
      CHECK(a.issuer == static_cast<DroneId>(i + 1));
      CHECK(a.issuer != 1);  // the ACTIVE_MISSION drone never speaks for the mesh
      CHECK(a.cls == EventClass::DISTRESS_BEACON);
      if (a.level == AlertLevel::PRIORITY) {
        CHECK(a.cep_m <= 250.0f && a.confidence >= 0.9f && popCount(a.modality_mask) >= 2);
        CHECK(distanceM(a.position, enu(kTruthE, kTruthN)) < 150.0);
      } else {
        CHECK(a.cep_m <= 2000.0f);
      }
      CHECK(p->gmcs[i].links[k] == LinkType::LOS);  // lowest-latency link available
    }
  }

  // Every table now shows the event as confirmed and the verifiers are released.
  p->run(2000);
  for (std::size_t i = 0; i < 5; ++i) {
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr);
    if (ev != nullptr) {
      CHECK(ev->status == EventStatus::CONFIRMED);
      CHECK(ev->verifier[0].id == kNoDrone && ev->verifier[1].id == kNoDrone);
    }
    CHECK(!p->mgr[i]->isVerifying());
  }
}

TEST(alert_owner_failover) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  // The would-be owner cannot reach the GMCS: the next drone in line takes over.
  reportBeacon(*p, {1, 2, 3, 4});
  for (auto& g : p->gmcs) g.fail = true;
  p->run(3000);
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) == 0);
  std::uint32_t failures = 0;
  for (std::size_t i = 0; i < 5; ++i) failures += p->mgr[i]->stats().alert_send_failures;
  CHECK(failures > 0);
  p->gmcs[3].fail = false;
  p->run(9000);
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) == 1);
  CHECK(p->gmcs[3].count(AlertLevel::ADVISORY) == 1);
}

TEST(active_mission_drone_is_released_when_gmcs_takes_over) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  reportBeacon(*p, {1, 2, 3, 4});
  p->run(3000);
  std::size_t verifier = 99;
  for (std::size_t i = 1; i < 4; ++i) {
    if (p->mgr[i]->isVerifying()) {
      verifier = i;
      break;
    }
  }
  CHECK(verifier != 99);
  if (verifier == 99) return;

  // GMCS commands that drone into ACTIVE_MISSION: the mesh task is dropped
  // at once and the others re-plan around it.
  p->place(verifier, 0, 0, DroneMode::ACTIVE_MISSION);
  CHECK(!p->mgr[verifier]->isVerifying());
  CHECK(p->guidance[verifier].cancels >= 1);
  p->run(3000);
  const DetectedEvent* ev = soleEvent(*p->mgr[0]);
  CHECK(ev != nullptr);
  if (ev != nullptr) {
    CHECK(ev->verifier[0].id != static_cast<DroneId>(verifier + 1));
    CHECK(ev->verifier[1].id != static_cast<DroneId>(verifier + 1));
  }
}

TEST(verifier_loss_triggers_reassignment) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  reportBeacon(*p, {1, 2, 3, 4});
  p->run(3000);
  const DetectedEvent* ev0 = soleEvent(*p->mgr[0]);
  CHECK(ev0 != nullptr);
  if (ev0 == nullptr) return;
  const DroneId lost = ev0->verifier[0].id;
  CHECK(lost != kNoDrone);
  if (lost == kNoDrone) return;

  p->bus.isolated[static_cast<std::size_t>(lost - 1)] = true;  // drone drops off the mesh
  p->run(9000);
  for (std::size_t i = 0; i < 5; ++i) {
    if (static_cast<DroneId>(i + 1) == lost) continue;
    CHECK(p->mgr[i]->table().findPeer(lost) == nullptr);
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr);
    if (ev == nullptr) continue;
    CHECK(ev->verifier[0].id != lost && ev->verifier[1].id != lost);
    CHECK(ev->verifier[0].id != kNoDrone);  // someone else took the primary slot
  }
}

TEST(learned_benign_signature_spreads_and_releases_verifiers) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(2000);
  const std::uint32_t kIsm = 433920;
  const GeoPoint truth = enu(kTruthE, kTruthN);
  for (std::size_t i : {1, 2, 3, 4}) {
    SensorObservation o = bearingObs(p->mgr[i]->exportedState().position, truth, 0.0, kIsm);
    o.class_hint = EventClass::RF_EMITTER;
    CHECK(p->mgr[i]->reportLocalDetection(o));
  }
  p->run(3000);
  int verifying = 0;
  for (std::size_t i = 1; i < 4; ++i) verifying += p->mgr[i]->isVerifying() ? 1 : 0;
  CHECK(verifying >= 1);  // novel emitter attracts a verifier
  for (std::size_t i = 0; i < 5; ++i) CHECK(!p->mgr[i]->table().isBenign(kIsm));

  // Operator dismisses it through drone 1's GMCS link.
  p->mgr[0]->applyGmcsDismissal(kIsm);
  p->run(2000);
  for (std::size_t i = 0; i < 5; ++i) {
    CHECK(p->mgr[i]->table().isBenign(kIsm));
    CHECK(p->mgr[i]->table().learnedCount() == 1);
    CHECK(!p->mgr[i]->isVerifying());
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr && ev->status == EventStatus::DISMISSED);
  }
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) == 0 && p->totalAlerts(AlertLevel::PRIORITY) == 0);
}

TEST(preloaded_benign_is_low_priority_and_not_tasked) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  const std::uint32_t kKnown = 162550;
  for (std::size_t i = 0; i < 5; ++i) p->mgr[i]->addKnownBenignSignature(kKnown);
  p->run(2000);
  const GeoPoint truth = enu(kTruthE, kTruthN);
  for (std::size_t i : {1, 2, 3, 4}) {
    SensorObservation o = bearingObs(p->mgr[i]->exportedState().position, truth, 0.0, kKnown);
    o.class_hint = EventClass::RF_EMITTER;
    CHECK(p->mgr[i]->reportLocalDetection(o));
  }
  p->run(4000);
  const DetectedEvent* ev = soleEvent(*p->mgr[0]);
  CHECK(ev != nullptr);
  if (ev != nullptr) CHECK(ev->urgency < 0.25f);
  for (std::size_t i = 0; i < 5; ++i) CHECK(!p->mgr[i]->isVerifying() && p->guidance[i].requests == 0);
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) + p->totalAlerts(AlertLevel::PRIORITY) == 0);
}

TEST(anti_entropy_converges_under_packet_loss) {
  auto p = std::make_unique<Pack>(5);
  p->bus.drop_permille = 300;  // 30% of datagrams lost
  placeLine(*p);
  p->run(2000);
  reportBeacon(*p, {1, 2, 3, 4});
  p->run(25000);
  for (std::size_t i = 0; i < 5; ++i) {
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr);
    if (ev != nullptr) CHECK(ev->observations.size() == 4);
    CHECK(p->mgr[i]->table().peerCount() == 4);
  }
  // Loss shows up in the per-peer link quality estimate.
  const PeerRecord* r = p->mgr[0]->table().findPeer(2);
  CHECK(r != nullptr && r->linkQuality() < 0.95f && r->linkQuality() > 0.4f);
  // Still exactly one advisory despite the loss.
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) <= 2);  // a duplicate is tolerated (GMCS de-duplicates by event id)
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) >= 1);
}

TEST(garbage_on_the_radio_is_counted_not_trusted) {
  auto p = std::make_unique<Pack>(2);
  p->place(0, 0, 0);
  p->place(1, 0, 5000);
  std::array<std::uint8_t, 40> junk{};
  for (std::size_t i = 0; i < junk.size(); ++i) junk[i] = static_cast<std::uint8_t>(i * 37u + 11u);
  for (int k = 0; k < 5; ++k) p->bus.broadcastFrom(1, junk.data(), junk.size());
  p->run(500);
  CHECK(p->mgr[0]->stats().rx_rejected >= 5);
  CHECK(p->mgr[0]->table().events().empty());
}

TEST(no_heap_allocation_in_steady_state) {
  auto p = std::make_unique<Pack>(5);
  placeLine(*p);
  p->run(1000);  // warm-up outside the measured window is not needed, but harmless
  const std::size_t before = g_allocs;
  p->run(1000);
  reportBeacon(*p, {1, 2, 3, 4}, 1.0);
  p->run(20000);
  p->mgr[0]->applyGmcsDismissal(433920);
  p->run(5000);
  const std::size_t after = g_allocs;
  CHECK(after == before);
  if (after != before) std::printf("    %zu heap allocations in the mesh loop\n", after - before);
  // Sanity: the loop did real work.
  CHECK(p->mgr[0]->stats().evaluations > 40 && p->mgr[0]->stats().tx_packets > 20);
}

TEST(event_table_overflow_evicts_without_crashing) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  std::size_t stored = 0;
  for (std::uint32_t k = 0; k < kMaxEvents + 20; ++k) {
    SensorObservation o = fixObs(SensorType::EO, enu(3000.0 * k, 50000.0), 50.0f, 0.7f);
    o.observer = 2;
    o.observer_position = enu(0, 0);
    o.timestamp_ms = now;
    o.event_hint = 0x00020000u + k + 1u;
    stored += t.ingestObservation(o, now) != kNoEvent ? 1 : 0;
  }
  CHECK(stored == kMaxEvents + 20);
  CHECK(t.events().size() == kMaxEvents);
}

TEST(observation_slot_replacement_and_expiry) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs t0 = 1'700'000'000'000ULL;
  SensorObservation o = bearingObs(enu(0, 0), enu(8000, 3000));
  o.observer = 2;
  o.observer_position = enu(0, 0);
  o.timestamp_ms = t0;
  o.event_hint = 0x00020001u;
  const EventId id = t.ingestObservation(o, t0);
  CHECK(t.findEvent(id)->observations.size() == 1);

  SensorObservation newer = o;
  newer.timestamp_ms = t0 + 1000;
  newer.bearing_deg += 1.0f;
  CHECK(t.ingestObservation(newer, t0 + 1000));
  CHECK(t.findEvent(id)->observations.size() == 1);  // same (observer, sensor): replaced
  CHECK(std::fabs(t.findEvent(id)->observations[0].bearing_deg - newer.bearing_deg) < 1e-4f);

  CHECK(t.ingestObservation(o, t0 + 1100));  // stale duplicate is ignored
  CHECK(std::fabs(t.findEvent(id)->observations[0].bearing_deg - newer.bearing_deg) < 1e-4f);

  t.refresh(t0 + 1000 + cfg.observation_max_age_ms + 1);
  CHECK(t.events().empty());
}

// ------------------------------------------------------- crypto and frames

static std::size_t fromHex(const char* hex, std::uint8_t* out, std::size_t cap) {
  std::size_t n = 0;
  for (std::size_t i = 0; hex[i] != '\0' && hex[i + 1] != '\0' && n < cap; i += 2) {
    const auto nib = [](char c) -> unsigned {
      return c <= '9' ? static_cast<unsigned>(c - '0') : static_cast<unsigned>(c - 'a' + 10);
    };
    out[n++] = static_cast<std::uint8_t>((nib(hex[i]) << 4U) | nib(hex[i + 1]));
  }
  return n;
}

static bool equalsHex(const std::uint8_t* data, std::size_t n, const char* hex) {
  std::array<std::uint8_t, 512> want{};
  return fromHex(hex, want.data(), want.size()) == n && std::memcmp(data, want.data(), n) == 0;
}

static const char kSunscreen[] =
    "Ladies and Gentlemen of the class of '99: If I could offer you only one tip for the future, sunscreen would be it.";

TEST(crypto_rfc8439_vectors) {
  using namespace crypto;
  // 2.3.2: block function.
  Key k{};
  for (std::size_t i = 0; i < k.size(); ++i) k[i] = static_cast<std::uint8_t>(i);
  Nonce n{};
  fromHex("000000090000004a00000000", n.data(), n.size());
  std::array<std::uint8_t, 64> block{};
  chacha20Block(k, 1, n, block.data());
  CHECK(equalsHex(block.data(), 64,
                  "10f1e7e4d13b5915500fdd1fa32071c4c7d1f4c733c068030422aa9ac3d46c4e"
                  "d2826446079faa0914c2d705d98b02a2b5129cd1de164eb9cbd083e8a2503c4e"));

  // 2.4.2: encryption.
  const std::size_t len = sizeof kSunscreen - 1;
  const auto* pt = reinterpret_cast<const std::uint8_t*>(kSunscreen);
  std::array<std::uint8_t, 128> ct{};
  fromHex("000000000000004a00000000", n.data(), n.size());
  chacha20Xor(k, 1, n, pt, ct.data(), len);
  CHECK(equalsHex(ct.data(), len,
                  "6e2e359a2568f98041ba0728dd0d6981e97e7aec1d4360c20a27afccfd9fae0bf91b65c5524733ab8f593dabcd62b3571639"
                  "d624e65152ab8f530c359f0861d807ca0dbf500d6a6156a38e088a22b65e52bc514d16ccf806818ce91ab77937365af90bbf"
                  "74a35be6b40b8eedf2785e42874d"));

  // 2.5.2: Poly1305, one-shot and fed in odd-sized pieces.
  std::array<std::uint8_t, 32> pk{};
  fromHex("85d6be7857556d337f4452fe42d506a80103808afb0db2fd4abff6af4149f51b", pk.data(), pk.size());
  const char* msg = "Cryptographic Forum Research Group";
  const auto* m = reinterpret_cast<const std::uint8_t*>(msg);
  Tag t1{};
  {
    Poly1305 mac(pk.data());
    mac.update(m, 34);
    mac.finish(t1);
  }
  CHECK(equalsHex(t1.data(), 16, "a8061dc1305136c6c22b8baf0c0127a9"));
  Tag t2{};
  {
    Poly1305 mac(pk.data());
    mac.update(m, 3);
    mac.update(m + 3, 0);
    mac.update(m + 3, 17);
    mac.update(m + 20, 14);
    mac.finish(t2);
  }
  CHECK(t1 == t2);

  // Appendix A.3 #5-#11: inputs chosen to hit the carries and the final reduction mod 2^130 - 5.
  struct PolyEdge {
    const char* key;
    const char* msg;
    const char* tag;
  };
  static const PolyEdge kEdges[] = {
      {"0200000000000000000000000000000000000000000000000000000000000000", "ffffffffffffffffffffffffffffffff",
       "03000000000000000000000000000000"},
      {"02000000000000000000000000000000ffffffffffffffffffffffffffffffff", "02000000000000000000000000000000",
       "03000000000000000000000000000000"},
      {"0100000000000000000000000000000000000000000000000000000000000000",
       "fffffffffffffffffffffffffffffffff0ffffffffffffffffffffffffffffff11000000000000000000000000000000",
       "05000000000000000000000000000000"},
      {"0100000000000000000000000000000000000000000000000000000000000000",
       "fffffffffffffffffffffffffffffffffbfefefefefefefefefefefefefefefe01010101010101010101010101010101",
       "00000000000000000000000000000000"},
      {"0200000000000000000000000000000000000000000000000000000000000000", "fdffffffffffffffffffffffffffffff",
       "faffffffffffffffffffffffffffffff"},
      {"0100000000000000040000000000000000000000000000000000000000000000",
       "e33594d7505e43b900000000000000003394d7505e4379cd0100000000000000000000000000000000000000000000000100000000"
       "0000000000000000000000",
       "14000000000000005500000000000000"},
      {"0100000000000000040000000000000000000000000000000000000000000000",
       "e33594d7505e43b900000000000000003394d7505e4379cd010000000000000000000000000000000000000000000000",
       "13000000000000000000000000000000"},
  };
  for (const PolyEdge& e : kEdges) {
    std::array<std::uint8_t, 32> ek{};
    std::array<std::uint8_t, 64> em{};
    fromHex(e.key, ek.data(), ek.size());
    const std::size_t ml = fromHex(e.msg, em.data(), em.size());
    Tag et{};
    Poly1305 mac(ek.data());
    mac.update(em.data(), ml);
    mac.finish(et);
    CHECK(equalsHex(et.data(), 16, e.tag));
  }

  // 2.8.2: AEAD.
  Key ak{};
  for (std::size_t i = 0; i < ak.size(); ++i) ak[i] = static_cast<std::uint8_t>(0x80 + i);
  fromHex("070000004041424344454647", n.data(), n.size());
  std::array<std::uint8_t, 12> aad{};
  fromHex("50515253c0c1c2c3c4c5c6c7", aad.data(), aad.size());
  Tag tag{};
  aeadSeal(ak, n, aad.data(), aad.size(), pt, len, ct.data(), tag);
  CHECK(equalsHex(ct.data(), len,
                  "d31a8d34648e60db7b86afbc53ef7ec2a4aded51296e08fea9e2b5a736ee62d63dbea45e8ca9671282fafb69da92728b1a71"
                  "de0a9e060b2905d6a5b67ecd3b3692ddbd7f2d778b8c9803aee328091b58fab324e4fad675945585808b4831d7bc3ff4def0"
                  "8e4b7a9de576d26586cec64b6116"));
  CHECK(equalsHex(tag.data(), 16, "1ae10b594f09e26a7e902ecbd0600691"));
  std::array<std::uint8_t, 128> back{};
  CHECK(aeadOpen(ak, n, aad.data(), aad.size(), ct.data(), len, tag, back.data()));
  CHECK(std::memcmp(back.data(), pt, len) == 0);

  // Any changed bit in the ciphertext, the AAD or the tag is refused, and no plaintext leaks.
  int forged = 0;
  int leaked = 0;
  for (std::size_t i = 0; i < len + aad.size() + tag.size(); ++i) {
    std::array<std::uint8_t, 128> c2 = ct;
    std::array<std::uint8_t, 12> a2 = aad;
    Tag t3 = tag;
    if (i < len) {
      c2[i] ^= 0x01;
    } else if (i < len + aad.size()) {
      a2[i - len] ^= 0x80;
    } else {
      t3[i - len - aad.size()] ^= 0x10;
    }
    back.fill(0xEE);
    if (aeadOpen(ak, n, a2.data(), a2.size(), c2.data(), len, t3, back.data())) ++forged;
    for (std::size_t j = 0; j < len; ++j) leaked += back[j] != 0 ? 1 : 0;
  }
  CHECK(forged == 0);
  CHECK(leaked == 0);

  const std::uint8_t a[3] = {1, 2, 3};
  const std::uint8_t b[3] = {1, 2, 4};
  CHECK(constantTimeEqual(a, a, 3) && !constantTimeEqual(a, b, 3) && constantTimeEqual(a, b, 2));
}

#include "aead_vectors.inc"

TEST(crypto_matches_an_independent_implementation) {
  // Edge lengths around the 16- and 64-byte boundaries, cross-checked against OpenSSL.
  int checked = 0;
  for (const AeadVector& v : kAeadVectors) {
    crypto::Key k{};
    crypto::Nonce n{};
    std::array<std::uint8_t, 64> aad{};
    std::array<std::uint8_t, 160> pt{};
    std::array<std::uint8_t, 192> want{};
    fromHex(v.key, k.data(), k.size());
    fromHex(v.nonce, n.data(), n.size());
    const std::size_t aad_len = fromHex(v.aad, aad.data(), aad.size());
    const std::size_t len = fromHex(v.pt, pt.data(), pt.size());
    const std::size_t want_len = fromHex(v.ct_tag, want.data(), want.size());
    CHECK(want_len == len + crypto::kTagBytes);
    std::array<std::uint8_t, 160> ct{};
    crypto::Tag tag{};
    crypto::aeadSeal(k, n, aad.data(), aad_len, pt.data(), len, ct.data(), tag);
    CHECK(std::memcmp(ct.data(), want.data(), len) == 0);
    CHECK(std::memcmp(tag.data(), want.data() + len, crypto::kTagBytes) == 0);
    std::array<std::uint8_t, 160> back{};
    CHECK(crypto::aeadOpen(k, n, aad.data(), aad_len, ct.data(), len, tag, back.data()));
    CHECK(std::memcmp(back.data(), pt.data(), len) == 0);
    ++checked;
  }
  CHECK(checked == 11);
}

// Two drones' security layers sharing the pack key.
struct SecPair {
  MeshConfig cfg{};
  MeshSecurity tx{7, 1, cfg};
  MeshSecurity rx{3, 1, cfg};
  TimeMs now = 1'700'000'000'000ULL;
  SecPair() {
    if (!tx.installEpochKey(kPackEpoch, kPackKey, 0, kNoExpiry) || !rx.installEpochKey(kPackEpoch, kPackKey, 0, kNoExpiry)) {
      std::abort();
    }
  }
  template <typename Payload>
  std::size_t seal(const Payload& payload, std::array<std::uint8_t, kMaxPacketBytes>& buf, MeshSecurity* from = nullptr) {
    MeshPacket p;
    p.payload = payload;
    return (from != nullptr ? *from : tx).seal(p, now, buf.data(), buf.size());
  }
};

static DroneState sampleState(DroneId id) {
  DroneState s;
  s.id = id;
  s.mode = DroneMode::VERIFYING;
  s.active_sensor = SensorType::IR;
  s.installed_sensors = maskOf(SensorType::IR) | maskOf(SensorType::RF_DF);
  s.available_links = kLinkMaskAll;
  s.position = GeoPoint{36.1234567, -11.7654321, 3123.5f};
  s.speed_mps = 41.5f;
  s.heading_deg = 77.0f;
  s.endurance_remaining_s = 20000.0f;
  s.bingo_endurance_s = 1800.0f;
  s.tasked_event = 0x00070001u;
  s.timestamp_ms = 1'700'000'123'400ULL;
  return s;
}

TEST(frame_round_trip_all_types) {
  auto sp = std::make_unique<SecPair>();
  std::array<std::uint8_t, kMaxPacketBytes> buf{};
  MeshPacket out;

  const DroneState s = sampleState(7);
  std::size_t n = sp->seal(HeartbeatPayload{s}, buf);
  CHECK(n == kFrameHeaderBytes + kBodyHeaderBytes + 46 + kFrameTagBytes);
  CHECK(sp->rx.open(buf.data(), n, sp->now, out) == OpenStatus::OK);
  CHECK(out.type() == MsgType::HEARTBEAT);
  CHECK(out.header.src == 7 && out.header.seq == 1 && out.header.session == 1 && out.header.epoch == kPackEpoch);
  CHECK(out.header.time_ms == sp->now);
  const auto* hb = std::get_if<HeartbeatPayload>(&out.payload);
  CHECK(hb != nullptr);
  if (hb != nullptr) {
    CHECK(hb->state.mode == DroneMode::VERIFYING && hb->state.active_sensor == SensorType::IR);
    CHECK(std::fabs(hb->state.position.lat_deg - 36.1234567) < 1e-7);
    CHECK(std::fabs(hb->state.position.lon_deg + 11.7654321) < 1e-7);
    CHECK(hb->state.endurance_remaining_s == 20000.0f && hb->state.tasked_event == 0x00070001u);
    CHECK(hb->state.installed_sensors == s.installed_sensors && hb->state.available_links == kLinkMaskAll);
  }
  // The payload is not readable on the wire.
  bool clear = false;
  for (std::size_t i = kFrameHeaderBytes; i + 4 <= n; ++i) {
    std::uint32_t v;
    std::memcpy(&v, buf.data() + i, 4);
    if (v == 0x00070001u) clear = true;
  }
  CHECK(!clear);

  SensorObservation o = bearingObs(enu(0, 0), enu(5000, 5000), 0.0);
  o.observer = 7;
  o.event_hint = 0x00070001u;
  o.observer_position = enu(0, 0);
  o.timestamp_ms = 1'700'000'000'500ULL;
  n = sp->seal(ObservationPayload{o}, buf);
  CHECK(n == 104);  // the largest frame
  CHECK(sp->rx.open(buf.data(), n, sp->now, out) == OpenStatus::OK);
  const auto* ob = std::get_if<ObservationPayload>(&out.payload);
  CHECK(ob != nullptr);
  if (ob != nullptr) {
    CHECK(ob->obs.observer == 7 && ob->obs.sensor == SensorType::RF_DF && !ob->obs.has_fix);
    CHECK(ob->obs.signature == 406025u && ob->obs.event_hint == 0x00070001u);
    CHECK(std::fabs(ob->obs.bearing_deg - o.bearing_deg) < 1e-3f);
  }

  n = sp->seal(TaskClaimPayload{0x00030001u, 7, 1, 0.77f}, buf);
  CHECK(n > 0 && sp->rx.open(buf.data(), n, sp->now, out) == OpenStatus::OK);
  const auto* tc = std::get_if<TaskClaimPayload>(&out.payload);
  CHECK(tc != nullptr && tc->claimant == 7 && tc->role == 1 && tc->score == 0.77f);

  n = sp->seal(AlertIssuedPayload{0x00030001u, AlertLevel::PRIORITY, 7}, buf);
  CHECK(n > 0 && sp->rx.open(buf.data(), n, sp->now, out) == OpenStatus::OK);
  const auto* ai = std::get_if<AlertIssuedPayload>(&out.payload);
  CHECK(ai != nullptr && ai->level == AlertLevel::PRIORITY && ai->issuer == 7);

  n = sp->seal(SignatureUpdatePayload{433920u}, buf);
  CHECK(n > 0 && sp->rx.open(buf.data(), n, sp->now, out) == OpenStatus::OK);
  const auto* su = std::get_if<SignatureUpdatePayload>(&out.payload);
  CHECK(su != nullptr && su->signature == 433920u);

  n = sp->seal(LeavePayload{}, buf);
  CHECK(n == kFrameHeaderBytes + kBodyHeaderBytes + kFrameTagBytes);
  CHECK(sp->rx.open(buf.data(), n, sp->now, out) == OpenStatus::OK && out.type() == MsgType::LEAVE);
  CHECK(sp->tx.lastSeq() == 6);

  // Our own frames come back on multicast loopback and are skipped, not counted as errors.
  CHECK(sp->tx.open(buf.data(), n, sp->now, out) == OpenStatus::LOOPBACK);

  // Output buffer too small: nothing sealed, no sequence number spent.
  std::array<std::uint8_t, 40> tiny{};
  MeshPacket p;
  p.payload = HeartbeatPayload{s};
  CHECK(sp->tx.seal(p, sp->now, tiny.data(), tiny.size()) == 0);
  CHECK(sp->tx.lastSeq() == 6);
}

TEST(frame_rejects_corruption_and_bad_content) {
  auto sp = std::make_unique<SecPair>();
  std::array<std::uint8_t, kMaxPacketBytes> buf{};
  MeshPacket out;
  const std::size_t n = sp->seal(HeartbeatPayload{sampleState(7)}, buf);
  CHECK(n > 0);

  // Any single flipped byte, header or ciphertext or tag, is refused.
  int accepted = 0;
  for (std::size_t i = 0; i < n; ++i) {
    std::array<std::uint8_t, kMaxPacketBytes> bad = buf;
    bad[i] = static_cast<std::uint8_t>(bad[i] ^ 0x5A);
    if (sp->rx.open(bad.data(), n, sp->now, out) == OpenStatus::OK) ++accepted;
  }
  CHECK(accepted == 0);
  // Truncation, oversize, null.
  for (std::size_t len = 0; len < n; ++len) CHECK(sp->rx.open(buf.data(), len, sp->now, out) == OpenStatus::MALFORMED);
  CHECK(sp->rx.open(buf.data(), kMaxPacketBytes + 1, sp->now, out) == OpenStatus::MALFORMED);
  CHECK(sp->rx.open(nullptr, 60, sp->now, out) == OpenStatus::MALFORMED);
  // The untouched frame still opens after all that.
  CHECK(sp->rx.open(buf.data(), n, sp->now, out) == OpenStatus::OK);

  // A key holder can seal anything, so authentic frames are still validated:
  // NaN, an out-of-range latitude, a heartbeat for another drone, an absurd claim score.
  DroneState s = sampleState(7);
  s.speed_mps = std::nanf("");
  std::size_t k = sp->seal(HeartbeatPayload{s}, buf);
  CHECK(k > 0 && sp->rx.open(buf.data(), k, sp->now, out) == OpenStatus::MALFORMED);
  s = sampleState(7);
  s.position.lat_deg = 91.0;
  k = sp->seal(HeartbeatPayload{s}, buf);
  CHECK(k > 0 && sp->rx.open(buf.data(), k, sp->now, out) == OpenStatus::MALFORMED);
  k = sp->seal(HeartbeatPayload{sampleState(9)}, buf);
  CHECK(k > 0 && sp->rx.open(buf.data(), k, sp->now, out) == OpenStatus::MALFORMED);
  k = sp->seal(TaskClaimPayload{0x00070001u, 7, 0, 1.0e38f}, buf);
  CHECK(k > 0 && sp->rx.open(buf.data(), k, sp->now, out) == OpenStatus::MALFORMED);
}

TEST(per_drone_keys_bind_the_sender) {
  auto sp = std::make_unique<SecPair>();
  std::array<std::uint8_t, kMaxPacketBytes> buf{};
  MeshPacket out;
  crypto::Key k7{};
  crypto::Key k8{};
  deriveDroneKey(kPackKey, kPackEpoch, 7, k7);
  deriveDroneKey(kPackKey, kPackEpoch, 8, k8);
  CHECK(k7 != k8 && k7 != kPackKey);

  // Relabelling a frame as coming from another drone breaks it: the sender id
  // picks the key and is authenticated as part of the header.
  const std::size_t n = sp->seal(SignatureUpdatePayload{433920u}, buf);
  std::array<std::uint8_t, kMaxPacketBytes> relabelled = buf;
  relabelled[4] = 8;
  CHECK(sp->rx.open(relabelled.data(), n, sp->now, out) == OpenStatus::AUTH_FAILED);

  // An outsider with a different key for the same epoch number cannot speak.
  MeshSecurity outsider(9, 1, sp->cfg);
  CHECK(outsider.installEpochKey(kPackEpoch, testKey(77), 0, kNoExpiry));
  const std::size_t m = sp->seal(SignatureUpdatePayload{433920u}, buf, &outsider);
  CHECK(m > 0 && sp->rx.open(buf.data(), m, sp->now, out) == OpenStatus::AUTH_FAILED);
  // A frame under an epoch the receiver does not hold.
  MeshSecurity other_epoch(9, 1, sp->cfg);
  CHECK(other_epoch.installEpochKey(42, kPackKey, 0, kNoExpiry));
  const std::size_t e = sp->seal(SignatureUpdatePayload{433920u}, buf, &other_epoch);
  CHECK(e > 0 && sp->rx.open(buf.data(), e, sp->now, out) == OpenStatus::NO_KEY);
}

TEST(replay_window_and_boot_sessions) {
  auto sp = std::make_unique<SecPair>();
  std::array<std::array<std::uint8_t, kMaxPacketBytes>, 72> f{};
  std::array<std::size_t, 72> len{};
  for (std::size_t i = 1; i < f.size(); ++i) len[i] = sp->seal(LeavePayload{}, f[i]);  // seq = i
  MeshPacket out;
  const auto open = [&](std::size_t i) { return sp->rx.open(f[i].data(), len[i], sp->now, out); };

  CHECK(open(1) == OpenStatus::OK);
  CHECK(open(1) == OpenStatus::REPLAYED);
  CHECK(open(3) == OpenStatus::OK);
  CHECK(open(2) == OpenStatus::OK);  // reordered within the window
  CHECK(open(2) == OpenStatus::REPLAYED);
  CHECK(open(70) == OpenStatus::OK);
  CHECK(open(6) == OpenStatus::REPLAYED);  // 64 behind the newest: outside the window
  CHECK(open(7) == OpenStatus::OK);        // 63 behind: still inside
  CHECK(open(7) == OpenStatus::REPLAYED);

  // The sender reboots with a higher session: accepted, and its old session is closed.
  MeshSecurity rebooted(7, 2, sp->cfg);
  CHECK(rebooted.installEpochKey(kPackEpoch, kPackKey, 0, kNoExpiry));
  std::array<std::uint8_t, kMaxPacketBytes> r{};
  const std::size_t rn = sp->seal(LeavePayload{}, r, &rebooted);
  CHECK(sp->rx.open(r.data(), rn, sp->now, out) == OpenStatus::OK && out.header.session == 2 && out.header.seq == 1);
  CHECK(open(71) == OpenStatus::REPLAYED);  // never seen, but from the older session

  // Why the session must grow: a reboot that reuses session 2 reuses nonces,
  // and its early frames are taken for replays.
  MeshSecurity reused(7, 2, sp->cfg);
  CHECK(reused.installEpochKey(kPackEpoch, kPackKey, 0, kNoExpiry));
  const std::size_t un = sp->seal(LeavePayload{}, r, &reused);
  CHECK(un > 0 && sp->rx.open(r.data(), un, sp->now, out) == OpenStatus::REPLAYED);
}

TEST(stale_and_future_frames_are_refused) {
  auto sp = std::make_unique<SecPair>();
  std::array<std::uint8_t, kMaxPacketBytes> buf{};
  MeshPacket out;
  const std::size_t n = sp->seal(LeavePayload{}, buf);
  const TimeMs sent = sp->now;
  CHECK(sp->rx.open(buf.data(), n, sent + sp->cfg.frame_max_age_ms + 1, out) == OpenStatus::STALE);
  CHECK(sp->rx.open(buf.data(), n, sent - sp->cfg.max_clock_skew_ms - 1, out) == OpenStatus::STALE);
  // A refused frame does not use up its place in the replay window.
  CHECK(sp->rx.open(buf.data(), n, sent + 40, out) == OpenStatus::OK);
}

TEST(key_epoch_rollover_and_erase) {
  MeshConfig cfg;
  const TimeMs t = 1'700'000'000'000ULL;
  const TimeMs rekey = t + 60'000;
  MeshSecurity a(7, 1, cfg);
  MeshSecurity b(3, 1, cfg);
  for (MeshSecurity* s : {&a, &b}) {
    CHECK(s->installEpochKey(1, kPackKey, 0, rekey + 30'000));  // old epoch overlaps the new by 30 s
    CHECK(s->installEpochKey(2, testKey(2), rekey, kNoExpiry));
  }
  CHECK(!a.installEpochKey(3, testKey(3), t, t));  // empty window
  std::uint16_t e = 0;
  CHECK(a.sendEpoch(t, e) && e == 1);
  CHECK(a.sendEpoch(rekey, e) && e == 2);

  std::array<std::uint8_t, kMaxPacketBytes> old_frame{};
  MeshPacket p;
  p.payload = LeavePayload{};
  const std::size_t on = a.seal(p, t, old_frame.data(), old_frame.size());
  CHECK(on > 0 && p.header.epoch == 1);
  MeshPacket out;
  CHECK(b.open(old_frame.data(), on, t + 100, out) == OpenStatus::OK);

  std::array<std::uint8_t, kMaxPacketBytes> new_frame{};
  MeshPacket q;
  q.payload = LeavePayload{};
  const std::size_t nn = a.seal(q, rekey + 10, new_frame.data(), new_frame.size());
  CHECK(nn > 0 && q.header.epoch == 2);
  CHECK(b.open(new_frame.data(), nn, rekey + 20, out) == OpenStatus::OK);

  // A frame under epoch 1, after epoch 1 and its grace have ended.
  MeshSecurity late(7, 5, cfg);
  CHECK(late.installEpochKey(1, kPackKey, 0, kNoExpiry));
  const TimeMs after = rekey + 30'000 + cfg.key_epoch_grace_ms + 1;
  std::array<std::uint8_t, kMaxPacketBytes> late_frame{};
  MeshPacket r;
  r.payload = LeavePayload{};
  const std::size_t ln = late.seal(r, after, late_frame.data(), late_frame.size());
  CHECK(ln > 0 && b.open(late_frame.data(), ln, after, out) == OpenStatus::NO_KEY);

  // Erasing keys silences the sender; zeroizing empties every slot.
  CHECK(a.eraseEpochKey(2) && !a.eraseEpochKey(2));
  CHECK(!a.sendEpoch(rekey + 40'000, e));
  CHECK(a.seal(q, rekey + 40'000, new_frame.data(), new_frame.size()) == 0);
  b.zeroizeAll();
  CHECK(b.epochCount() == 0);
  CHECK(b.open(new_frame.data(), nn, rekey + 20, out) == OpenStatus::NO_KEY);
}

TEST(mesh_ignores_outsiders_replays_and_revoked_drones) {
  auto p = std::make_unique<Pack>(4);
  for (std::size_t i = 0; i < 3; ++i) p->place(i, 0, static_cast<double>(i) * 5000.0);
  // Node 3 is an outsider: right frame format, wrong key.
  p->mgr[3] = std::make_unique<PackMeshManager>(9, MeshConfig{}, 1U, p->clock, p->bus.nodes[3], p->gmcs[3], p->guidance[3]);
  CHECK(p->mgr[3]->installEpochKey(kPackEpoch, testKey(66), 0, kNoExpiry));
  p->place(3, 0, 15000);
  p->bus.record_src = 1;  // record drone 2's traffic for a replay later
  p->run(3000);
  for (std::size_t i = 0; i < 3; ++i) {
    CHECK(p->mgr[i]->table().peerCount() == 2);  // the outsider is never discovered
    CHECK(p->mgr[i]->table().findPeer(9) == nullptr);
    CHECK(p->mgr[i]->stats().rx_auth_failed > 0);
  }
  // The outsider's dismissal and alert notice change nothing.
  p->mgr[3]->applyGmcsDismissal(433920u);
  p->run(500);
  for (std::size_t i = 0; i < 3; ++i) CHECK(!p->mgr[i]->table().isBenign(433920u));

  // Replaying drone 2's recorded frames: refused at once as replays, later as stale.
  p->bus.record_src = -1;
  const std::size_t taped = p->bus.tape_n;
  CHECK(taped >= 3);
  const std::uint32_t replayed_before = p->mgr[0]->stats().rx_replayed;
  for (std::size_t k = 0; k < taped; ++k) (void)p->bus.broadcastFrom(3, p->bus.tape[k].data.data(), p->bus.tape[k].len);
  p->run(100);
  CHECK(p->mgr[0]->stats().rx_replayed >= replayed_before + static_cast<std::uint32_t>(taped));
  p->clock.now += 10'000;
  const std::uint32_t stale_before = p->mgr[0]->stats().rx_stale;
  for (std::size_t k = 0; k < taped; ++k) (void)p->bus.broadcastFrom(3, p->bus.tape[k].data.data(), p->bus.tape[k].len);
  p->mgr[0]->tick();
  CHECK(p->mgr[0]->stats().rx_stale >= stale_before + static_cast<std::uint32_t>(taped));

  // Drone 3 reports a fix, then the GMCS revokes it: its readings are forgotten
  // and its frames refused.
  p->run(2000);
  CHECK(p->mgr[2]->reportLocalDetection(fixObs(SensorType::IR, enu(9000, 0), 50.0f)));
  p->run(1000);
  CHECK(p->mgr[0]->table().events().size() == 1);
  CHECK(p->mgr[0]->revokeDrone(3) && p->mgr[1]->revokeDrone(3));
  CHECK(!p->mgr[0]->revokeDrone(1));  // not itself
  p->run(2000);
  for (std::size_t i = 0; i < 2; ++i) {
    CHECK(p->mgr[i]->table().events().empty());
    CHECK(p->mgr[i]->table().findPeer(3) == nullptr);
    CHECK(p->mgr[i]->stats().rx_revoked > 0);
  }
}

TEST(no_key_means_silence_and_reboot_is_accepted) {
  auto p = std::make_unique<Pack>(2);
  p->place(0, 0, 0);
  p->place(1, 0, 5000);
  p->run(2000);
  CHECK(p->mgr[0]->table().peerCount() == 1);
  // Drone 2 reboots with a new session and comes straight back.
  p->reboot(1, 2);
  p->place(1, 0, 5000);
  p->run(2000);
  CHECK(p->mgr[0]->table().peerCount() == 1);
  CHECK(p->mgr[0]->stats().rx_replayed == 0);
  const PeerRecord* r = p->mgr[0]->table().findPeer(2);
  CHECK(r != nullptr && r->last_session == 2);
  // Keys zeroized: nothing goes out, and the peer times the drone out.
  p->mgr[1]->zeroizeKeys();
  const std::uint32_t tx = p->mgr[1]->stats().tx_packets;
  p->run(7000);
  CHECK(p->mgr[1]->stats().tx_packets == tx);
  CHECK(p->mgr[1]->stats().tx_no_key > 0);
  CHECK(p->mgr[0]->table().peerCount() == 0);
}

// ------------------------------------------- regressions from code review

static SensorObservation tableObs(SensorObservation o, DroneId observer, TimeMs ts, EventId hint = kNoEvent) {
  o.observer = observer;
  if (o.observer_position.lat_deg == 0.0) o.observer_position = enu(0, 0);
  o.timestamp_ms = ts;
  o.event_hint = hint;
  return o;
}

TEST(same_frequency_sources_far_apart_stay_separate) {
  // Every 406 MHz distress beacon shares its frequency: a shared signature
  // must not fuse two sources 80 km apart into one ghost in the middle.
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  SensorObservation a = fixObs(SensorType::EO, enu(-40000, 0), 50.0f);
  a.signature = 406025;
  SensorObservation b = fixObs(SensorType::EO, enu(40000, 0), 50.0f);
  b.signature = 406025;
  const EventId ia = t.ingestObservation(tableObs(a, 2, now, 0x00020001u), now);
  const EventId ib = t.ingestObservation(tableObs(b, 3, now, 0x00030001u), now);
  CHECK(ia != kNoEvent && ib != kNoEvent && ia != ib);
  t.refresh(now);
  CHECK(t.events().size() == 2);
  for (const auto& ev : t.events()) CHECK(ev.cep_m < 200.0f);

  // A bearing towards the second beacon must not attach to the first.
  SensorObservation brg = bearingObs(enu(0, -30000), enu(40000, 0));
  brg.observer_position = enu(0, -30000);
  const EventId ic = t.ingestObservation(tableObs(brg, 4, now), now);
  CHECK(ic != kNoEvent && ic != ia);
}

TEST(readings_beyond_pack_scale_never_fuse) {
  // Found by the fuzzer: two bearings on one frequency from observers 16,000 km
  // apart joined one event, and the flat-frame fusion put it off the earth.
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  SensorObservation far_brg = bearingObs(GeoPoint{35.92, 162.48, 3000.0f}, GeoPoint{36.2, 162.48, 0.0f});
  far_brg.observer_position = GeoPoint{35.92, 162.48, 3000.0f};
  SensorObservation near_brg = bearingObs(enu(0, 0), enu(10000, 9000));
  near_brg.observer_position = enu(0, 0);
  const EventId a = t.ingestObservation(tableObs(far_brg, 2, now), now);
  const EventId b = t.ingestObservation(tableObs(near_brg, 4, now), now);
  CHECK(a != kNoEvent && b != kNoEvent && a != b);
  t.refresh(now);
  CHECK(t.events().size() == 2);  // not merged either
  for (const auto& ev : t.events()) CHECK(isPlausible(ev.position));

  // fuseEvent on its own never reports a fused position off the earth.
  DetectedEvent ev;
  ev.id = 1;
  CHECK(ev.observations.push_back(tableObs(far_brg, 2, now)));
  CHECK(ev.observations.push_back(tableObs(near_brg, 4, now)));
  fuseEvent(ev, cfg);
  CHECK(isPlausible(ev.position));
  CHECK(!ev.localized);
}

TEST(rebooted_drone_never_reuses_an_event_id_in_use) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  // A peer re-sends a reading of X filed under 0x10001, an id drone 1 used before rebooting.
  const EventId x = t.ingestObservation(tableObs(fixObs(SensorType::IR, enu(20000, 0), 50.0f), 2, now, 0x00010001u), now);
  CHECK(x == 0x00010001u);
  t.seedEventCounter(0);  // worst case: the counter restarts exactly where it was
  const EventId fresh = t.allocateEventId();
  CHECK(fresh != kNoEvent && fresh != x);
  // An unrelated source far away, quoting the stale id, gets its own event.
  const EventId y = t.ingestObservation(tableObs(fixObs(SensorType::IR, enu(-20000, 0), 50.0f), 3, now, 0x00010001u), now);
  CHECK(y != kNoEvent && y != x);
  CHECK(t.events().size() == 2);
  CHECK(distanceM(t.findEvent(x)->position, enu(20000, 0)) < 100.0);
}

TEST(fusion_is_correct_across_the_antimeridian) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  const GeoPoint east{10.0, 179.999, 0.0f};
  const GeoPoint west{10.0, -179.999, 0.0f};
  SensorObservation a = fixObs(SensorType::IR, east, 50.0f);
  a.observer_position = GeoPoint{10.05, 179.99, 3000.0f};
  SensorObservation b = fixObs(SensorType::EO, west, 50.0f);
  b.observer_position = GeoPoint{9.95, -179.99, 3000.0f};
  const EventId ia = t.ingestObservation(tableObs(a, 2, now), now);
  const EventId ib = t.ingestObservation(tableObs(b, 3, now), now);
  CHECK(ia == ib);  // 219 m apart: one source
  const DetectedEvent* ev = t.findEvent(ia);
  CHECK(ev != nullptr);
  if (ev != nullptr) {
    CHECK(std::fabs(ev->position.lon_deg) > 179.99);  // not 0.0, the naive average
    CHECK(distanceM(ev->position, east) < 300.0);
  }
  CHECK(wrap360(-1e-15) < 360.0);
  CHECK(wrap180(190.0) == -170.0);
}

TEST(claim_arbitration_ignores_arrival_order) {
  struct Claim {
    DroneId id;
    float score;
  };
  const std::array<Claim, 3> claims = {{{1, 0.50000f}, {2, 0.50008f}, {3, 0.50016f}}};
  std::array<int, 3> idx = {0, 1, 2};
  DroneId first_holder = kNoDrone;
  int permutations = 0;
  do {
    MeshConfig cfg;
    SituationalAwarenessTable t(9, cfg);
    const TimeMs now = 1'700'000'000'000ULL;
    const EventId id = t.ingestObservation(tableObs(fixObs(SensorType::IR, enu(5000, 0), 50.0f), 2, now, 0x00020001u), now);
    for (int k : idx) (void)t.applyClaim(id, claims[k].id, 0, claims[k].score, now);
    const DroneId holder = t.findEvent(id)->verifier[0].id;
    if (permutations == 0) first_holder = holder;
    CHECK(holder == first_holder);
    ++permutations;
  } while (std::next_permutation(idx.begin(), idx.end()));
  CHECK(permutations == 6);
  CHECK(first_holder == 3);  // highest quantised score wins
}

TEST(clock_step_backwards_does_not_silence_the_drone) {
  auto p = std::make_unique<Pack>(2);
  p->place(0, 0, 0);
  p->place(1, 0, 5000);
  p->run(3000);
  const std::uint32_t tx_before = p->mgr[0]->stats().tx_packets;
  p->clock.now -= 10000;  // GNSS time correction steps the clock back 10 s
  p->run(8000);
  CHECK(p->mgr[0]->stats().tx_packets >= tx_before + 6);  // heartbeats kept flowing
  CHECK(p->mgr[1]->table().peerCount() == 1);             // so the peer never timed it out
  CHECK(p->mgr[0]->table().peerCount() == 1);
}

TEST(anti_entropy_eventually_resends_every_reading) {
  // 24 own readings with a re-send budget of 16 per cycle: a peer that missed
  // all the originals must still receive every one.
  auto p = std::make_unique<Pack>(2);
  p->place(0, 0, 0);
  p->place(1, 0, 5000);
  p->run(2000);
  p->bus.isolated[1] = true;
  for (int k = 0; k < 24; ++k) {
    CHECK(p->mgr[0]->reportLocalDetection(fixObs(SensorType::EO, enu(3000.0 * k, 40000.0), 50.0f, 0.7f)));
  }
  p->bus.isolated[1] = false;
  p->run(8000);  // four anti-entropy cycles
  CHECK(p->mgr[0]->table().events().size() == 24);
  CHECK(p->mgr[1]->table().events().size() == 24);
}

TEST(packets_that_speak_for_someone_else_are_rejected) {
  auto p = std::make_unique<Pack>(2);
  p->place(0, 0, 0);
  p->place(1, 0, 5000);
  p->run(2000);
  // Drone 7 holds the pack key (an insider) but has never been heard ...
  MeshSecurity insider(7, 1, MeshConfig{});
  CHECK(insider.installEpochKey(kPackEpoch, kPackKey, 0, kNoExpiry));
  std::array<std::uint8_t, kMaxPacketBytes> buf{};
  const auto send = [&](const MeshPacket& in) {
    MeshPacket pkt = in;
    const std::size_t n = insider.seal(pkt, p->clock.now, buf.data(), buf.size());
    CHECK(n > 0);
    CHECK(p->bus.broadcastFrom(1, buf.data(), n));
  };
  MeshPacket pkt;
  SensorObservation o = fixObs(SensorType::IR, enu(9000, 0), 50.0f);
  o.observer = 1;  // ... and claims to relay drone 1's own reading,
  o.observer_position = enu(0, 0);
  o.timestamp_ms = p->clock.now;
  pkt.payload = ObservationPayload{o};
  send(pkt);
  pkt.payload = TaskClaimPayload{0x00020001u, 2, 0, 0.9f};  // a claim in drone 2's name,
  send(pkt);
  pkt.payload = AlertIssuedPayload{0x00020001u, AlertLevel::PRIORITY, 2};  // and an alert notice in it.
  send(pkt);
  pkt.payload = TaskClaimPayload{0x00070001u, 7, 0, 1.0e38f};  // An absurd claim score.
  send(pkt);
  p->run(300);
  CHECK(p->mgr[0]->table().events().empty());
  CHECK(p->mgr[0]->stats().rx_rejected >= 4);
  CHECK(p->mgr[0]->stats().rx_auth_failed == 0);  // they were authentic; the content was refused
}

TEST(future_and_expired_timestamps_are_rejected) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  const SensorObservation f = fixObs(SensorType::IR, enu(5000, 0), 50.0f);
  CHECK(t.ingestObservation(tableObs(f, 2, now + 3'600'000), now) == kNoEvent);  // would never age out
  CHECK(t.ingestObservation(tableObs(f, 2, now - cfg.observation_max_age_ms - 1), now) == kNoEvent);
  CHECK(t.ingestObservation(tableObs(f, 2, now + 500), now) != kNoEvent);  // within clock skew
  DroneState s;
  s.id = 2;
  s.installed_sensors = maskOf(SensorType::IR);
  s.position = enu(0, 0);
  s.timestamp_ms = now + 3'600'000;
  CHECK(!t.updatePeer(s, 1, 1, now));
  CHECK(t.peerCount() == 0);
}

TEST(implausible_local_inputs_are_refused) {
  auto p = std::make_unique<Pack>(2);
  p->place(0, 0, 0);
  const GeoPoint before = p->mgr[0]->exportedState().position;
  DroneState bad = p->mgr[0]->exportedState();
  bad.mode = DroneMode::PACK_ROUTING;
  bad.speed_mps = std::nanf("");
  bad.position = enu(99999, 99999);
  p->mgr[0]->updateOwnState(bad);
  CHECK(p->mgr[0]->stats().local_rejected == 1);
  CHECK(distanceM(p->mgr[0]->exportedState().position, before) < 1.0);  // old state kept

  SensorObservation zero_sigma = bearingObs(before, enu(5000, 5000));
  zero_sigma.bearing_sigma_deg = 0.0f;  // would give the line infinite weight
  CHECK(!p->mgr[0]->reportLocalDetection(zero_sigma));
  CHECK(p->mgr[0]->stats().local_rejected == 2);
}

TEST(full_pack_fits_the_snapshot) {
  MeshConfig cfg;
  SituationalAwarenessTable t(1, cfg);
  const TimeMs now = 1'700'000'000'000ULL;
  for (DroneId id = 2; id <= kMaxPackSize + 1; ++id) {
    DroneState s;
    s.id = id;
    s.installed_sensors = maskOf(SensorType::IR);
    s.position = enu(0, 1000.0 * id);
    s.timestamp_ms = now;
    const bool added = t.updatePeer(s, 1, 1, now);
    CHECK(added == (id <= kMaxPackSize));  // 15 peers + self = the full pack
  }
  DroneList dl;
  t.snapshotDrones(dl);
  CHECK(dl.size() == kMaxPackSize);
}

TEST(config_validation) {
  CHECK(isValid(MeshConfig{}));
  MeshConfig c;
  c.urgency_decay_ms = 0.0;  // would divide by zero
  CHECK(!isValid(c));
  c = MeshConfig{};
  c.max_diversion_fraction = 1.5;
  CHECK(!isValid(c));
  c = MeshConfig{};
  c.peer_timeout_ms = c.heartbeat_period_ms;  // peers would time out between heartbeats
  CHECK(!isValid(c));
}

// ------------------------------------ deterministic re-clustering, partitions

// A table's partition as text: one line per event, "id: observer/sensor@t ...", events by id.
static void describePartition(const SituationalAwarenessTable& t, char* out, std::size_t cap) {
  std::array<std::size_t, kMaxEvents> idx{};
  const auto& events = t.events();
  for (std::size_t i = 0; i < events.size(); ++i) idx[i] = i;
  std::sort(idx.begin(), idx.begin() + static_cast<std::ptrdiff_t>(events.size()),
            [&](std::size_t a, std::size_t b) { return events[a].id < events[b].id; });
  std::size_t n = 0;
  out[0] = '\0';
  for (std::size_t k = 0; k < events.size(); ++k) {
    const DetectedEvent& ev = events[idx[k]];
    n += static_cast<std::size_t>(std::snprintf(out + n, cap - n, "%08X:", ev.id));
    for (const auto& o : ev.observations) {
      n += static_cast<std::size_t>(std::snprintf(out + n, cap - n, " %u/%u@%llu", o.observer,
                                                  static_cast<unsigned>(o.sensor),
                                                  static_cast<unsigned long long>(o.timestamp_ms % 100000)));
    }
    n += static_cast<std::size_t>(std::snprintf(out + n, cap - n, "\n"));
    if (n >= cap) break;
  }
}

TEST(reclustering_does_not_depend_on_arrival_order) {
  // The corner case the README listed: three sources in a row 440 m apart, so
  // that A-B and B-C pass the association gate but A-C does not. Greedy
  // association grouped them three different ways depending only on which
  // reading came first (found by searching spacings with re-clustering off).
  // Each source is seen by two imagers; every reading carries its observer's id.
  const TimeMs now = 1'700'000'000'000ULL;
  std::array<SensorObservation, 6> obs{};
  const double xs[3] = {0.0, 440.0, 880.0};
  for (std::size_t s = 0; s < 3; ++s) {
    for (std::size_t k = 0; k < 2; ++k) {
      SensorObservation o =
          fixObs(k == 0 ? SensorType::IR : SensorType::EO, enu(20000.0 + xs[s] + (k == 0 ? -30.0 : 30.0), 5000.0), 60.0f);
      const auto observer = static_cast<DroneId>(2 + s * 2 + k);
      obs[s * 2 + k] = tableObs(o, observer, now - 100 * (s * 2 + k), (static_cast<EventId>(observer) << 16U) | 1U);
    }
  }
  std::array<std::size_t, 6> order = {0, 1, 2, 3, 4, 5};
  std::array<char, 2048> first{};
  std::array<char, 2048> desc{};
  int permutations = 0;
  int differ = 0;
  do {
    MeshConfig cfg;
    auto t = std::make_unique<SituationalAwarenessTable>(1, cfg);
    for (std::size_t i : order) (void)t->ingestObservation(obs[i], now);
    t->refresh(now);
    describePartition(*t, desc.data(), desc.size());
    if (permutations == 0) first = desc;
    if (std::strcmp(first.data(), desc.data()) != 0) ++differ;
    ++permutations;
  } while (std::next_permutation(order.begin(), order.end()));
  CHECK(permutations == 720);
  CHECK(differ == 0);
  if (differ != 0) std::printf("    first:\n%s    last:\n%s", first.data(), desc.data());
}

TEST(reclustering_ids_follow_the_readings_not_the_drone) {
  // Two drones that file the same readings in different orders, under
  // different local ids, end with the same events under the same ids.
  const TimeMs now = 1'700'000'000'000ULL;
  MeshConfig cfg;
  auto a = std::make_unique<SituationalAwarenessTable>(1, cfg);
  auto b = std::make_unique<SituationalAwarenessTable>(9, cfg);
  std::array<SensorObservation, 5> obs{};
  for (std::size_t k = 0; k < 4; ++k) {
    SensorObservation o = bearingObs(enu(0, -9000.0 + 6000.0 * k), enu(12000, 3000));
    o.observer_position = enu(0, -9000.0 + 6000.0 * k);
    const auto observer = static_cast<DroneId>(2 + k);
    obs[k] = tableObs(o, observer, now - 50 * k, (static_cast<EventId>(observer) << 16U) | 3U);
  }
  obs[4] = tableObs(fixObs(SensorType::IR, enu(12040, 2980), 60.0f), 6, now, 0x00060003u);
  for (std::size_t k = 0; k < obs.size(); ++k) (void)a->ingestObservation(obs[k], now);
  for (std::size_t k = obs.size(); k-- > 0;) (void)b->ingestObservation(obs[k], now);
  a->refresh(now);
  b->refresh(now);
  std::array<char, 2048> da{};
  std::array<char, 2048> db{};
  describePartition(*a, da.data(), da.size());
  describePartition(*b, db.data(), db.size());
  CHECK(std::strcmp(da.data(), db.data()) == 0);
  CHECK(a->events().size() == 1 && b->events().size() == 1);
  if (a->events().size() == 1) CHECK(a->events()[0].id == 0x00020003u);  // the lowest id its readings carry
}

TEST(a_split_event_keeps_its_alerts_with_one_heir) {
  // A table that grouped two separate sources into one event (as arrival order
  // once could) is split by the refresh; the event's history goes to one heir.
  const TimeMs now = 1'700'000'000'000ULL;
  MeshConfig cfg;
  auto t = std::make_unique<SituationalAwarenessTable>(1, cfg);
  const EventId id = t->ingestObservation(tableObs(fixObs(SensorType::IR, enu(30000, 0), 60.0f), 2, now, 0x00020001u), now);
  CHECK(id == 0x00020001u);
  DetectedEvent* ev = t->findEvent(id);
  CHECK(ev != nullptr);
  if (ev == nullptr) return;
  // Force a second, far-away reading into the same event, as a stale greedy grouping would have.
  CHECK(ev->observations.push_back(tableObs(fixObs(SensorType::EO, enu(30000, 9000), 60.0f), 3, now, 0x00030001u)));
  ev->advisory_sent = true;
  t->refresh(now);
  CHECK(t->events().size() == 2);
  int with_flag = 0;
  for (const auto& e : t->events()) with_flag += e.advisory_sent ? 1 : 0;
  CHECK(with_flag == 1);
  const DetectedEvent* heir = t->findEvent(0x00020001u);
  CHECK(heir != nullptr && heir->advisory_sent);
  CHECK(t->findEvent(0x00030001u) != nullptr);
}

TEST(one_drone_hearing_two_sources_on_one_frequency_keeps_both) {
  // One DF hears two beacons on 406.025 MHz at very different bearings before
  // either is localised. Readings are keyed by (observer, sensor), so the
  // second must not overwrite the first.
  const TimeMs now = 1'700'000'000'000ULL;
  MeshConfig cfg;
  auto t = std::make_unique<SituationalAwarenessTable>(1, cfg);
  SensorObservation north = bearingObs(enu(0, 0), enu(2000, 20000));
  north.observer_position = enu(0, 0);
  SensorObservation south = bearingObs(enu(0, 0), enu(3000, -20000));
  south.observer_position = enu(0, 0);
  const EventId a = t->ingestObservation(tableObs(north, 2, now, 0x00020001u), now);
  const EventId b = t->ingestObservation(tableObs(south, 2, now + 10, 0x00020002u), now + 10);
  CHECK(a != kNoEvent && b != kNoEvent && a != b);
  t->refresh(now + 10);
  CHECK(t->events().size() == 2);
  // A later reading of the northern beacon still updates its own event.
  SensorObservation north2 = north;
  north2.bearing_deg += 0.5f;
  CHECK(t->ingestObservation(tableObs(north2, 2, now + 2000, 0x00020001u), now + 2000) == a);
  t->refresh(now + 2000);
  CHECK(t->events().size() == 2);
}

// Six drones in two groups of three, 6 km apart in each group; both groups can hear the beacon.
static void placeTwoGroups(Pack& p) {
  for (std::size_t i = 0; i < 6; ++i) p.place(i, 0, -15000.0 + 6000.0 * static_cast<double>(i));
}

static std::size_t verifyingCount(const Pack& p) {
  std::size_t n = 0;
  for (std::size_t i = 0; i < p.n; ++i) n += p.mgr[i]->isVerifying() ? 1 : 0;
  return n;
}

TEST(partition_then_heal_converges_to_one_event) {
  auto p = std::make_unique<Pack>(6);
  placeTwoGroups(*p);
  p->run(2000);
  for (std::size_t i = 3; i < 6; ++i) p->bus.group[i] = 1;  // split the mesh: {1,2,3} | {4,5,6}
  p->run(6000);                                            // each side times the other out
  CHECK(p->mgr[0]->table().peerCount() == 2 && p->mgr[3]->table().peerCount() == 2);
  reportBeacon(*p, {0, 1, 2, 3, 4, 5});
  p->run(5000);
  // Each side localised the beacon under its own id and warned the GMCS.
  const EventId id_a = soleEvent(*p->mgr[0]) != nullptr ? soleEvent(*p->mgr[0])->id : kNoEvent;
  const EventId id_b = soleEvent(*p->mgr[3]) != nullptr ? soleEvent(*p->mgr[3])->id : kNoEvent;
  CHECK(id_a != kNoEvent && id_b != kNoEvent && id_a != id_b);
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) == 2);  // one per side: the GMCS must de-duplicate
  CHECK(verifyingCount(*p) <= 4);

  for (auto& g : p->bus.group) g = 0;  // heal
  p->run(8000);
  const DetectedEvent* ref = soleEvent(*p->mgr[0]);
  CHECK(ref != nullptr);
  if (ref == nullptr) return;
  CHECK(ref->id == (id_a < id_b ? id_a : id_b));
  CHECK(ref->observations.size() == 6);
  for (std::size_t i = 0; i < 6; ++i) {
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr);
    if (ev == nullptr) continue;
    CHECK(ev->id == ref->id);
    CHECK(ev->verifier[0].id == ref->verifier[0].id && ev->verifier[1].id == ref->verifier[1].id);
    CHECK(p->mgr[i]->table().peerCount() == 5);
    // Each side's old id still resolves there, so a late claim or alert notice quoting it lands.
    CHECK(p->mgr[i]->table().resolve(i < 3 ? id_a : id_b) == ref->id);
  }
  CHECK(verifyingCount(*p) <= 2);  // the merged plan sends at most two
  CHECK(p->totalAlerts(AlertLevel::ADVISORY) == 2);  // and nobody warns again

  // Confirmation after the heal produces exactly one PRIORITY.
  const GeoPoint truth = enu(kTruthE, kTruthN);
  CHECK(p->mgr[1]->reportLocalDetection(fixObs(SensorType::IR, truth, 60.0f)));
  CHECK(p->mgr[4]->reportLocalDetection(fixObs(SensorType::EO, enu(kTruthE - 30.0, kTruthN + 20.0), 60.0f)));
  p->run(10000);
  CHECK(p->totalAlerts(AlertLevel::PRIORITY) == 1);
}

TEST(partition_during_verification_reconciles_tasks) {
  auto p = std::make_unique<Pack>(6);
  placeTwoGroups(*p);
  p->run(2000);
  reportBeacon(*p, {0, 1, 2, 3, 4, 5});
  p->run(4000);
  const std::size_t before = verifyingCount(*p);
  CHECK(before >= 1 && before <= 2);

  for (std::size_t i = 3; i < 6; ++i) p->bus.group[i] = 1;
  p->run(9000);  // each side loses sight of the other's verifiers and plans on its own
  CHECK(verifyingCount(*p) <= 4);

  for (auto& g : p->bus.group) g = 0;
  p->run(10000);
  CHECK(verifyingCount(*p) >= 1 && verifyingCount(*p) <= 2);
  const DetectedEvent* ref = soleEvent(*p->mgr[0]);
  CHECK(ref != nullptr);
  if (ref == nullptr) return;
  for (std::size_t i = 0; i < 6; ++i) {
    const DetectedEvent* ev = soleEvent(*p->mgr[i]);
    CHECK(ev != nullptr && ev->id == ref->id && ev->verifier[0].id == ref->verifier[0].id &&
          ev->verifier[1].id == ref->verifier[1].id);
    // Every drone that is still verifying holds one of the agreed slots.
    if (p->mgr[i]->isVerifying()) {
      const auto self = static_cast<DroneId>(i + 1);
      CHECK(ref->verifier[0].id == self || ref->verifier[1].id == self);
    }
  }
}

TEST(dismissal_during_partition_reaches_the_other_side_on_heal) {
  auto p = std::make_unique<Pack>(6);
  placeTwoGroups(*p);
  p->run(2000);
  const std::uint32_t kIsm = 433920;
  const GeoPoint truth = enu(kTruthE, kTruthN);
  for (std::size_t i = 0; i < 6; ++i) {
    SensorObservation o = bearingObs(p->mgr[i]->exportedState().position, truth, 0.0, kIsm);
    o.class_hint = EventClass::RF_EMITTER;
    CHECK(p->mgr[i]->reportLocalDetection(o));
  }
  p->run(3000);
  for (std::size_t i = 3; i < 6; ++i) p->bus.group[i] = 1;
  p->run(1000);
  p->mgr[0]->applyGmcsDismissal(kIsm);  // the operator's uplink reaches side A only
  p->run(3000);
  for (std::size_t i = 0; i < 3; ++i) CHECK(p->mgr[i]->table().isBenign(kIsm));
  for (std::size_t i = 3; i < 6; ++i) CHECK(!p->mgr[i]->table().isBenign(kIsm));
  for (auto& g : p->bus.group) g = 0;
  p->run(5000);
  for (std::size_t i = 0; i < 6; ++i) {
    CHECK(p->mgr[i]->table().isBenign(kIsm));
    CHECK(!p->mgr[i]->isVerifying());
  }
}

// ----------------------------------------------------------------------- main
int main() {
  int failed_tests = 0;
  for (std::size_t i = 0; i < g_n_tests; ++i) {
    const int before = g_failures;
    std::printf("[ RUN  ] %s\n", g_tests[i].name);
    g_tests[i].fn();
    const bool ok = g_failures == before;
    failed_tests += ok ? 0 : 1;
    std::printf("[ %s ] %s\n", ok ? " OK " : "FAIL", g_tests[i].name);
  }
  std::printf("\n%zu tests, %d checks, %d failed checks\n", g_n_tests, g_checks, g_failures);
  return failed_tests == 0 ? 0 : 1;
}
