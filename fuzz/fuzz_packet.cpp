// libFuzzer harness for the mesh decoder and everything behind it.
//
// Random bytes almost never get past the Poly1305 tag, so the harness takes
// its input as a list of records and seals each one itself, as a pack member
// (an insider with a valid key) would. That drives the body decoder, the
// security layer's replay logic and the manager's receive path with arbitrary
// authentic content.
//
// Input: the whole buffer is first offered to the frame decoder as a raw
// datagram. It is then read as records:
//   [ctl u8][len u8][len bytes of plaintext body]...
//   ctl bits 0-2  sender: drone 2 + (ctl & 7)
//   ctl bit  3    replay this sender's previous frame instead of sealing a new one
//   ctl bit  4    reboot this sender first (next boot session)
//   ctl bits 5-7  advance the clock by that many 100 ms ticks before delivery
// A body is the plaintext codec's format: type u8, time_ms u64, payload.
//
// Properties checked (any failure aborts, which libFuzzer reports):
//   - no raw datagram opens without a key, and nothing crashes or leaks;
//   - a body that decodes re-encodes to exactly the same bytes;
//   - a fresh sealed frame opens at an independent receiver with the same
//     content, and a replayed one is refused as REPLAYED;
//   - after every step the manager's table is internally consistent.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include "amc8270/mesh/mesh_packet.hpp"
#include "amc8270/mesh/mesh_security.hpp"
#include "amc8270/mesh/pack_mesh_manager.hpp"

using namespace amc8270::mesh;

namespace {

constexpr TimeMs kT0 = 1'700'000'000'000ULL;
constexpr std::size_t kSenders = 8;

[[noreturn]] void fail(const char* why) {
  // Printed by libFuzzer along with the crashing input.
  std::fputs(why, stderr);
  std::fputc('\n', stderr);
  std::abort();
}

crypto::Key fuzzKey() {
  crypto::Key k{};
  for (std::size_t i = 0; i < k.size(); ++i) k[i] = static_cast<std::uint8_t>(0x3C ^ (i * 13U));
  return k;
}

// Held by the raw-datagram receiver only. No seed and no sender ever seals with it,
// so a raw input that opens under it would be a forgery.
crypto::Key rawKey() {
  crypto::Key k{};
  for (std::size_t i = 0; i < k.size(); ++i) k[i] = static_cast<std::uint8_t>(0xC3 ^ (i * 41U));
  return k;
}

struct Clock : IClock {
  TimeMs now = kT0;
  TimeMs nowMs() const override { return now; }
};

struct Radio : IMeshTransceiver {
  std::array<std::array<std::uint8_t, kMaxPacketBytes>, 8> q{};
  std::array<std::size_t, 8> len{};
  std::size_t n = 0;
  std::size_t next = 0;
  bool send(const std::uint8_t*, std::size_t) override { return true; }
  std::size_t receive(std::uint8_t* buf, std::size_t cap) override {
    if (next >= n) {
      n = next = 0;
      return 0;
    }
    const std::size_t k = len[next] < cap ? len[next] : cap;
    std::memcpy(buf, q[next].data(), k);
    ++next;
    return k;
  }
  void push(const std::uint8_t* d, std::size_t k) {
    if (n >= q.size() || k > kMaxPacketBytes) return;
    std::memcpy(q[n].data(), d, k);
    len[n++] = k;
  }
};

struct Gmcs : IGmcsLink {
  bool sendAlert(const GmcsAlert& a, LinkType) override {
    if (a.event_id == kNoEvent || a.level == AlertLevel::NONE) fail("alert with no event or level");
    return true;
  }
};

struct Guidance : IGuidanceSink {
  void requestVerification(const VerificationTask& t) override {
    if (t.event_id == kNoEvent || t.role > 1) fail("task with no event or a bad role");
  }
  void cancelVerification() override {}
};

void checkTable(const SituationalAwarenessTable& t) {
  const auto& events = t.events();
  if (events.size() > kMaxEvents) fail("event table over capacity");
  for (std::size_t i = 0; i < events.size(); ++i) {
    const DetectedEvent& ev = events[i];
    if (ev.id == kNoEvent) fail("event with id 0");
    for (std::size_t j = i + 1; j < events.size(); ++j) {
      if (events[j].id == ev.id) fail("two events share an id");
    }
    if (ev.observations.size() > kMaxObsPerEvent) fail("observations over capacity");
    for (std::size_t k = 1; k < ev.observations.size(); ++k) {
      const SensorObservation& a = ev.observations[k - 1];
      const SensorObservation& b = ev.observations[k];
      const bool sorted = a.observer < b.observer || (a.observer == b.observer && a.sensor < b.sensor);
      if (!sorted) fail("observations not in (observer, sensor) order");
    }
    if (ev.verifier[0].id != kNoDrone && ev.verifier[0].id == ev.verifier[1].id) fail("one drone holds both slots");
    if (!(ev.cep_m >= 0.0f) || !(ev.confidence >= 0.0f && ev.confidence <= 1.0f)) fail("fused value out of range");
    if (ev.localized && !(ev.position.lat_deg >= -90.0 && ev.position.lat_deg <= 90.0)) fail("fused position off the earth");
  }
}

// Any valid config is fair game; a short backup delay lets every alert-owner
// rank fire within the second or two an input runs for.
MeshConfig harnessConfig() {
  MeshConfig c;
  c.alert_backup_delay_ms = 200;
  return c;
}

struct World {
  Clock clock;
  Radio radio;
  Gmcs gmcs;
  Guidance guidance;
  MeshConfig cfg = harnessConfig();
  PackMeshManager mgr{1, cfg, 1U, clock, radio, gmcs, guidance};
  MeshSecurity witness{1, 1U, cfg};  // independent receiver: the same checks the manager makes
  std::array<MeshSecurity*, kSenders> senders{};
  std::array<std::uint32_t, kSenders> session{};
  std::array<std::array<std::uint8_t, kMaxPacketBytes>, kSenders> last{};
  std::array<std::size_t, kSenders> last_len{};

  // MeshSecurity is not movable, so senders live in fixed storage, rebuilt in place.
  alignas(MeshSecurity) std::array<std::array<unsigned char, sizeof(MeshSecurity)>, kSenders> storage{};

  World() {
    if (!mgr.installEpochKey(1, fuzzKey(), 0, kNoExpiry) || !witness.installEpochKey(1, fuzzKey(), 0, kNoExpiry)) {
      fail("key install");
    }
    for (std::size_t i = 0; i < kSenders; ++i) {
      session[i] = 1;
      senders[i] = makeSender(i);
    }
    // The harness's own drone: routing with sensors and a C2 link, 6 km east of the
    // seed scenarios' beacon (a standoff point, from an uncovered side), so it is
    // the natural verifier and alert owner.
    DroneState self;
    self.mode = DroneMode::PACK_ROUTING;
    self.installed_sensors = maskOf(SensorType::RF_DF) | maskOf(SensorType::IR) | maskOf(SensorType::EO);
    self.available_links = kLinkMaskAll;
    self.position = GeoPoint{36.018, -11.822, 3000.0f};
    self.speed_mps = 40.0f;
    self.endurance_remaining_s = 5.0f * 3600.0f;
    self.bingo_endurance_s = 1800.0f;
    mgr.updateOwnState(self);
  }
  World(const World&) = delete;
  World& operator=(const World&) = delete;
  World(World&&) = delete;
  World& operator=(World&&) = delete;
  ~World() {
    for (MeshSecurity* s : senders) s->~MeshSecurity();
  }

  MeshSecurity* makeSender(std::size_t i) {
    auto* s = new (storage[i].data()) MeshSecurity(static_cast<DroneId>(2 + i), session[i], cfg);
    if (!s->installEpochKey(1, fuzzKey(), 0, kNoExpiry)) fail("key install");
    return s;
  }

  void reboot(std::size_t i) {
    senders[i]->~MeshSecurity();
    ++session[i];
    senders[i] = makeSender(i);
    last_len[i] = 0;
  }

  void advance(unsigned ticks) {
    for (unsigned k = 0; k < ticks; ++k) {
      clock.now += 100;
      mgr.tick();
      checkTable(mgr.table());
    }
  }

  void deliver(const std::uint8_t* frame, std::size_t len) {
    radio.push(frame, len);
    mgr.tick();
    checkTable(mgr.table());
  }
};

void roundTrip(const std::uint8_t* body, std::size_t len, DroneId src, MeshPacket& pkt) {
  std::array<std::uint8_t, kMaxBodyBytes> again{};
  const std::size_t n = encodeBody(pkt, again.data(), again.size());
  if (n != len || std::memcmp(again.data(), body, len) != 0) fail("decoded body does not re-encode to the same bytes");
  MeshPacket twice;
  if (!decodeBody(again.data(), n, src, twice)) fail("re-encoded body does not decode");
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  // 1. The raw bytes as a datagram, under a key nobody seals with.
  {
    MeshConfig cfg;
    MeshSecurity rx(1, 1U, cfg);
    if (!rx.installEpochKey(1, rawKey(), 0, kNoExpiry)) fail("key install");
    MeshPacket out;
    const OpenStatus st = rx.open(data, size, kT0, out);
    if (st == OpenStatus::OK) fail("an unsealed datagram opened");
    FrameHeader h;
    (void)parseFrameHeader(data, size, h);
  }

  // 2. Records, sealed by insiders and delivered to a live manager.
  // Rebuilt for every input, so no state leaks from one input to the next.
  alignas(World) static std::array<unsigned char, sizeof(World)> world_storage;
  World& w = *new (world_storage.data()) World();

  std::size_t pos = 0;
  while (pos + 2 <= size) {
    const std::uint8_t ctl = data[pos];
    const std::size_t len = data[pos + 1];
    pos += 2;
    if (len > size - pos) break;
    const std::uint8_t* body = data + pos;
    pos += len;

    const std::size_t si = ctl & 7U;
    const auto src = static_cast<DroneId>(2 + si);
    w.advance(static_cast<unsigned>(ctl >> 5U));
    if ((ctl & 0x10U) != 0) w.reboot(si);

    if ((ctl & 0x08U) != 0) {  // replay
      if (w.last_len[si] == 0) continue;
      MeshPacket out;
      const OpenStatus st = w.witness.open(w.last[si].data(), w.last_len[si], w.clock.now, out);
      if (st == OpenStatus::OK) fail("a replayed frame was accepted");
      w.deliver(w.last[si].data(), w.last_len[si]);
      continue;
    }

    MeshPacket pkt;
    if (!decodeBody(body, len, src, pkt)) continue;
    roundTrip(body, len, src, pkt);

    std::array<std::uint8_t, kMaxPacketBytes> frame{};
    const std::size_t n = w.senders[si]->seal(pkt, w.clock.now, frame.data(), frame.size());
    if (n == 0) fail("a decodable body could not be sealed");
    MeshPacket opened;
    if (w.witness.open(frame.data(), n, w.clock.now, opened) != OpenStatus::OK) fail("a fresh sealed frame was refused");
    std::array<std::uint8_t, kMaxBodyBytes> b1{};
    std::array<std::uint8_t, kMaxBodyBytes> b2{};
    const std::size_t n1 = encodeBody(pkt, b1.data(), b1.size());
    const std::size_t n2 = encodeBody(opened, b2.data(), b2.size());
    if (n1 != n2 || std::memcmp(b1.data(), b2.data(), n1) != 0) fail("frame content changed in transit");
    if (opened.header.src != src || opened.header.session != w.session[si]) fail("frame header changed in transit");

    std::memcpy(w.last[si].data(), frame.data(), n);
    w.last_len[si] = n;
    w.deliver(frame.data(), n);
  }
  w.advance(20);  // let refresh, planning and alerting run on whatever arrived
#ifdef FUZZ_VERBOSE  // corpus maintenance: shows how deep an input reaches
  const MeshStats& st = w.mgr.stats();
  std::size_t localized = 0;
  for (const auto& ev : w.mgr.table().events()) localized += ev.localized ? 1U : 0U;
  std::fprintf(stderr, "rx=%u rejected=%u replayed=%u peers=%zu events=%zu localized=%zu tasks=%u adv=%u pri=%u learned=%u\n",
               st.rx_packets, st.rx_rejected, st.rx_replayed, w.mgr.table().peerCount(), w.mgr.table().events().size(),
               localized, st.tasks_issued, st.advisories_sent, st.priority_alerts_sent, st.signatures_learned);
#endif
  w.~World();
  return 0;
}
