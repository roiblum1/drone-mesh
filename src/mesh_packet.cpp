#include "amc8270/mesh/mesh_packet.hpp"

#include <array>
#include <cmath>
#include <cstring>

#include "amc8270/mesh/validation.hpp"

namespace amc8270::mesh {

namespace {

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

// ---- payload codecs

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
  if (!isPlausible(s)) r.fail();
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
  if (!isPlausible(o)) r.fail();
}

}  // namespace

void writeFrameHeader(const FrameHeader& h, std::uint8_t* out18) noexcept {
  Writer w(out18, kFrameHeaderBytes);
  w.u16(kPacketMagic);
  w.u8(kPacketVersion);
  w.u8(0);  // flags, reserved
  w.u16(h.src);
  w.u16(h.epoch);
  w.u32(h.session);
  w.u32(h.seq);
  w.u16(h.body_len);
}

bool parseFrameHeader(const std::uint8_t* buf, std::size_t len, FrameHeader& out) noexcept {
  if (buf == nullptr || len < kFrameHeaderBytes + kBodyHeaderBytes + kFrameTagBytes || len > kMaxPacketBytes) {
    return false;
  }
  Reader r(buf, kFrameHeaderBytes);
  const std::uint16_t magic = r.u16();
  const std::uint8_t version = r.u8();
  const std::uint8_t flags = r.u8();
  out.src = r.u16();
  out.epoch = r.u16();
  out.session = r.u32();
  out.seq = r.u32();
  out.body_len = r.u16();
  if (!r.ok() || magic != kPacketMagic || version != kPacketVersion || flags != 0) return false;
  if (out.src == kNoDrone || out.session == 0 || out.seq == 0) return false;
  return static_cast<std::size_t>(out.body_len) + kFrameHeaderBytes + kFrameTagBytes == len;
}

std::size_t encodeBody(const MeshPacket& pkt, std::uint8_t* buf, std::size_t cap) noexcept {
  if (buf == nullptr || pkt.payload.index() == 0) return 0;
  Writer w(buf, cap);
  w.u8(static_cast<std::uint8_t>(pkt.type()));
  w.u64(pkt.header.time_ms);
  switch (pkt.type()) {
    case MsgType::HEARTBEAT:
      writeState(w, std::get_if<HeartbeatPayload>(&pkt.payload)->state);
      break;
    case MsgType::OBSERVATION:
      writeObservation(w, std::get_if<ObservationPayload>(&pkt.payload)->obs);
      break;
    case MsgType::TASK_CLAIM: {
      const auto* p = std::get_if<TaskClaimPayload>(&pkt.payload);
      w.u32(p->event_id);
      w.u16(p->claimant);
      w.u8(p->role);
      w.f32(p->score);
      break;
    }
    case MsgType::ALERT_ISSUED: {
      const auto* p = std::get_if<AlertIssuedPayload>(&pkt.payload);
      w.u32(p->event_id);
      w.u8(static_cast<std::uint8_t>(p->level));
      w.u16(p->issuer);
      break;
    }
    case MsgType::SIGNATURE_UPDATE:
      w.u32(std::get_if<SignatureUpdatePayload>(&pkt.payload)->signature);
      break;
    case MsgType::LEAVE:
      break;
  }
  return w.ok() ? w.size() : 0;
}

bool decodeBody(const std::uint8_t* buf, std::size_t len, DroneId src, MeshPacket& out) noexcept {
  if (buf == nullptr || len < kBodyHeaderBytes || len > kMaxBodyBytes || src == kNoDrone) return false;
  Reader r(buf, len);
  const std::uint8_t type = r.u8();
  out.header.src = src;
  out.header.time_ms = r.u64();
  if (!r.ok() || type < 1 || type > kMsgTypeMax) return false;

  switch (static_cast<MsgType>(type)) {
    case MsgType::HEARTBEAT: {
      auto& p = out.payload.emplace<HeartbeatPayload>();
      readState(r, p.state);
      if (p.state.id != src) return false;  // a drone only describes itself
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
      if (p.role > 1 || p.claimant == kNoDrone || p.event_id == kNoEvent || !isPlausibleClaimScore(p.score)) {
        return false;
      }
      break;
    }
    case MsgType::ALERT_ISSUED: {
      auto& p = out.payload.emplace<AlertIssuedPayload>();
      p.event_id = r.u32();
      const std::uint8_t level = r.u8();
      p.issuer = r.u16();
      if (p.issuer == kNoDrone || level == 0 || level > kAlertLevelMax || p.event_id == kNoEvent) return false;
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
