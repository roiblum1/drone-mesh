#include "amc8270/mesh/mesh_packet.hpp"

#include <array>
#include <cmath>
#include <cstring>

namespace amc8270::mesh {

namespace {

constexpr std::array<std::uint32_t, 256> makeCrcTable() {
  std::array<std::uint32_t, 256> t{};
  for (std::uint32_t i = 0; i < 256; ++i) {
    std::uint32_t c = i;
    for (int k = 0; k < 8; ++k) c = ((c & 1U) != 0U) ? (0xEDB88320U ^ (c >> 1U)) : (c >> 1U);
    t[i] = c;
  }
  return t;
}
constexpr std::array<std::uint32_t, 256> kCrcTable = makeCrcTable();

constexpr double kGeoScale = 1.0e7;  // 1e-7 deg ~ 1.1 cm

class Writer {
 public:
  Writer(std::uint8_t* buf, std::size_t cap) : buf_(buf), cap_(cap) {}
  bool ok() const { return ok_; }
  std::size_t size() const { return pos_; }

  void u8(std::uint8_t v) { put(&v, 1); }
  void u16(std::uint16_t v) {
    const std::uint8_t b[2] = {static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8U)};
    put(b, 2);
  }
  void u32(std::uint32_t v) {
    const std::uint8_t b[4] = {static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8U),
                               static_cast<std::uint8_t>(v >> 16U), static_cast<std::uint8_t>(v >> 24U)};
    put(b, 4);
  }
  void u64(std::uint64_t v) {
    std::uint8_t b[8];
    for (unsigned i = 0; i < 8U; ++i) b[i] = static_cast<std::uint8_t>(v >> (8U * i));
    put(b, 8);
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void f32(float v) {
    std::uint32_t bits;
    std::memcpy(&bits, &v, sizeof bits);
    u32(bits);
  }
  void geo(const GeoPoint& g) {
    i32(static_cast<std::int32_t>(std::llround(g.lat_deg * kGeoScale)));
    i32(static_cast<std::int32_t>(std::llround(g.lon_deg * kGeoScale)));
    f32(g.alt_m);
  }

 private:
  void put(const std::uint8_t* p, std::size_t n) {
    if (!ok_ || pos_ + n > cap_) {
      ok_ = false;
      return;
    }
    std::memcpy(buf_ + pos_, p, n);
    pos_ += n;
  }
  std::uint8_t* buf_;
  std::size_t cap_;
  std::size_t pos_ = 0;
  bool ok_ = true;
};

class Reader {
 public:
  Reader(const std::uint8_t* buf, std::size_t len) : buf_(buf), len_(len) {}
  bool ok() const { return ok_; }
  std::size_t remaining() const { return len_ - pos_; }
  void fail() { ok_ = false; }

  std::uint8_t u8() {
    std::uint8_t v = 0;
    take(&v, 1);
    return v;
  }
  std::uint16_t u16() {
    std::uint8_t b[2] = {};
    take(b, 2);
    return static_cast<std::uint16_t>(static_cast<unsigned>(b[0]) | (static_cast<unsigned>(b[1]) << 8U));
  }
  std::uint32_t u32() {
    std::uint8_t b[4] = {};
    take(b, 4);
    return static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8U) |
           (static_cast<std::uint32_t>(b[2]) << 16U) | (static_cast<std::uint32_t>(b[3]) << 24U);
  }
  std::uint64_t u64() {
    std::uint8_t b[8] = {};
    take(b, 8);
    std::uint64_t v = 0;
    for (unsigned i = 0; i < 8U; ++i) v |= static_cast<std::uint64_t>(b[i]) << (8U * i);
    return v;
  }
  std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
  float f32() {
    const std::uint32_t bits = u32();
    float v;
    std::memcpy(&v, &bits, sizeof v);
    if (!std::isfinite(v)) ok_ = false;
    return v;
  }
  GeoPoint geo() {
    GeoPoint g;
    g.lat_deg = static_cast<double>(i32()) / kGeoScale;
    g.lon_deg = static_cast<double>(i32()) / kGeoScale;
    g.alt_m = f32();
    if (g.lat_deg < -90.0 || g.lat_deg > 90.0 || g.lon_deg < -180.0 || g.lon_deg > 180.0) ok_ = false;
    return g;
  }

 private:
  void take(std::uint8_t* out, std::size_t n) {
    if (!ok_ || pos_ + n > len_) {
      ok_ = false;
      return;
    }
    std::memcpy(out, buf_ + pos_, n);
    pos_ += n;
  }
  const std::uint8_t* buf_;
  std::size_t len_;
  std::size_t pos_ = 0;
  bool ok_ = true;
};

// ---- payload encoders

void writeState(Writer& w, const DroneState& s) {
  w.u16(s.id);
  w.u8(static_cast<std::uint8_t>(s.mode));
  w.u8(static_cast<std::uint8_t>(s.active_sensor));
  w.u8(s.installed_sensors);
  w.u8(s.available_links);
  w.geo(s.position);
  w.f32(s.speed_mps);
  w.f32(s.heading_deg);
  w.f32(s.endurance_remaining_s);
  w.f32(s.bingo_endurance_s);
  w.u32(s.tasked_event);
  w.u64(s.timestamp_ms);
}

void readState(Reader& r, DroneState& s) {
  s.id = r.u16();
  const std::uint8_t mode = r.u8();
  const std::uint8_t sensor = r.u8();
  s.installed_sensors = r.u8();
  s.available_links = r.u8();
  s.position = r.geo();
  s.speed_mps = r.f32();
  s.heading_deg = r.f32();
  s.endurance_remaining_s = r.f32();
  s.bingo_endurance_s = r.f32();
  s.tasked_event = r.u32();
  s.timestamp_ms = r.u64();
  if (mode > kDroneModeMax || !isValidSensor(sensor) || !onlyBits(s.installed_sensors, kSensorMaskAll) ||
      !onlyBits(s.available_links, kLinkMaskAll) || s.id == kNoDrone) {
    r.fail();
    return;  // never cast an out-of-range value into an enum
  }
  s.mode = static_cast<DroneMode>(mode);
  s.active_sensor = static_cast<SensorType>(sensor);
}

void writeObservation(Writer& w, const SensorObservation& o) {
  w.u16(o.observer);
  w.u8(static_cast<std::uint8_t>(o.sensor));
  w.u8(static_cast<std::uint8_t>(o.class_hint));
  w.u8(o.has_fix ? 1 : 0);
  w.u32(o.event_hint);
  w.geo(o.observer_position);
  w.f32(o.bearing_deg);
  w.f32(o.bearing_sigma_deg);
  w.geo(o.fix);
  w.f32(o.fix_cep_m);
  w.f32(o.confidence);
  w.u32(o.signature);
  w.u64(o.timestamp_ms);
}

void readObservation(Reader& r, SensorObservation& o) {
  o.observer = r.u16();
  const std::uint8_t sensor = r.u8();
  const std::uint8_t cls = r.u8();
  const std::uint8_t has_fix = r.u8();
  o.event_hint = r.u32();
  o.observer_position = r.geo();
  o.bearing_deg = r.f32();
  o.bearing_sigma_deg = r.f32();
  o.fix = r.geo();
  o.fix_cep_m = r.f32();
  o.confidence = r.f32();
  o.signature = r.u32();
  o.timestamp_ms = r.u64();
  if (o.observer == kNoDrone || !isValidSensor(sensor) || sensor == 0 || cls > kEventClassMax ||
      has_fix > 1 || o.bearing_sigma_deg < 0.0f || o.fix_cep_m < 0.0f || o.confidence < 0.0f ||
      o.confidence > 1.0f) {
    r.fail();
    return;
  }
  o.sensor = static_cast<SensorType>(sensor);
  o.class_hint = static_cast<EventClass>(cls);
  o.has_fix = has_fix == 1;
}

}  // namespace

std::uint32_t crc32(const std::uint8_t* data, std::size_t len) noexcept {
  std::uint32_t c = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < len; ++i) c = kCrcTable[(c ^ data[i]) & 0xFFU] ^ (c >> 8U);
  return c ^ 0xFFFFFFFFu;
}

std::size_t encodePacket(const MeshPacket& pkt, std::uint8_t* buf, std::size_t cap) noexcept {
  if (buf == nullptr || pkt.payload.index() == 0 || cap < kHeaderBytes + kCrcBytes) return 0;

  // Payload first (after the header), then back-fill length.
  Writer body(buf + kHeaderBytes, cap - kHeaderBytes - kCrcBytes);
  switch (pkt.type()) {
    case MsgType::HEARTBEAT:
      writeState(body, std::get_if<HeartbeatPayload>(&pkt.payload)->state);
      break;
    case MsgType::OBSERVATION:
      writeObservation(body, std::get_if<ObservationPayload>(&pkt.payload)->obs);
      break;
    case MsgType::TASK_CLAIM: {
      const auto* p = std::get_if<TaskClaimPayload>(&pkt.payload);
      body.u32(p->event_id);
      body.u16(p->claimant);
      body.u8(p->role);
      body.f32(p->score);
      break;
    }
    case MsgType::ALERT_ISSUED: {
      const auto* p = std::get_if<AlertIssuedPayload>(&pkt.payload);
      body.u32(p->event_id);
      body.u8(static_cast<std::uint8_t>(p->level));
      body.u16(p->issuer);
      break;
    }
    case MsgType::SIGNATURE_UPDATE:
      body.u32(std::get_if<SignatureUpdatePayload>(&pkt.payload)->signature);
      break;
    case MsgType::LEAVE:
      break;
  }
  if (!body.ok()) return 0;
  const std::size_t payload_len = body.size();

  Writer head(buf, kHeaderBytes);
  head.u16(kPacketMagic);
  head.u8(kPacketVersion);
  head.u8(static_cast<std::uint8_t>(pkt.type()));
  head.u16(pkt.header.src);
  head.u16(static_cast<std::uint16_t>(payload_len));
  head.u32(pkt.header.seq);
  head.u64(pkt.header.time_ms);
  if (!head.ok()) return 0;

  const std::uint32_t crc = crc32(buf, kHeaderBytes + payload_len);
  Writer tail(buf + kHeaderBytes + payload_len, kCrcBytes);
  tail.u32(crc);
  return kHeaderBytes + payload_len + kCrcBytes;
}

bool decodePacket(const std::uint8_t* buf, std::size_t len, MeshPacket& out) noexcept {
  if (buf == nullptr || len < kHeaderBytes + kCrcBytes || len > kMaxPacketBytes) return false;

  Reader head(buf, kHeaderBytes);
  const std::uint16_t magic = head.u16();
  const std::uint8_t version = head.u8();
  const std::uint8_t type = head.u8();
  PacketHeader h;
  h.src = head.u16();
  const std::uint16_t payload_len = head.u16();
  h.seq = head.u32();
  h.time_ms = head.u64();
  if (!head.ok() || magic != kPacketMagic || version != kPacketVersion || h.src == kNoDrone) return false;
  if (type < 1 || type > 6) return false;
  if (static_cast<std::size_t>(payload_len) + kHeaderBytes + kCrcBytes != len) return false;

  Reader tail(buf + kHeaderBytes + payload_len, kCrcBytes);
  if (tail.u32() != crc32(buf, kHeaderBytes + payload_len)) return false;

  Reader r(buf + kHeaderBytes, payload_len);
  out.header = h;
  switch (static_cast<MsgType>(type)) {
    case MsgType::HEARTBEAT: {
      auto& p = out.payload.emplace<HeartbeatPayload>();
      readState(r, p.state);
      if (p.state.id != h.src) return false;
      break;
    }
    case MsgType::OBSERVATION: {
      auto& p = out.payload.emplace<ObservationPayload>();
      readObservation(r, p.obs);
      break;
    }
    case MsgType::TASK_CLAIM: {
      auto& p = out.payload.emplace<TaskClaimPayload>();
      p.event_id = r.u32();
      p.claimant = r.u16();
      p.role = r.u8();
      p.score = r.f32();
      if (p.role > 1 || p.claimant == kNoDrone || p.event_id == kNoEvent) return false;
      break;
    }
    case MsgType::ALERT_ISSUED: {
      auto& p = out.payload.emplace<AlertIssuedPayload>();
      p.event_id = r.u32();
      const std::uint8_t level = r.u8();
      p.issuer = r.u16();
      if (level == 0 || level > kAlertLevelMax || p.event_id == kNoEvent) return false;
      p.level = static_cast<AlertLevel>(level);
      break;
    }
    case MsgType::SIGNATURE_UPDATE: {
      auto& p = out.payload.emplace<SignatureUpdatePayload>();
      p.signature = r.u32();
      if (p.signature == 0) return false;
      break;
    }
    case MsgType::LEAVE:
      out.payload.emplace<LeavePayload>();
      break;
  }
  return r.ok() && r.remaining() == 0;
}

}  // namespace amc8270::mesh
