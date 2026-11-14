#include "amc8270/mesh/situational_awareness.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "amc8270/mesh/geo.hpp"
#include "amc8270/mesh/validation.hpp"

namespace amc8270::mesh {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kCepPerSigma = 1.1774;  // CEP = 1.1774 sigma for an isotropic 2-D Gaussian

bool obsKeyLess(const SensorObservation& a, const SensorObservation& b) noexcept {
  if (a.observer != b.observer) return a.observer < b.observer;
  return static_cast<std::uint8_t>(a.sensor) < static_cast<std::uint8_t>(b.sensor);
}

bool obsOlder(const SensorObservation& a, const SensorObservation& b) noexcept {
  if (a.timestamp_ms != b.timestamp_ms) return a.timestamp_ms < b.timestamp_ms;
  return obsKeyLess(a, b);
}

// Observations are kept sorted by (observer, sensor) so every drone folds
// them in the same order. Returns true if the stored set changed.
bool insertObservation(DetectedEvent& ev, const SensorObservation& o) noexcept {
  for (auto& e : ev.observations) {
    if (e.observer == o.observer && e.sensor == o.sensor) {
      if (o.timestamp_ms > e.timestamp_ms) {
        e = o;
        return true;
      }
      return false;
    }
  }
  if (ev.observations.full()) {
    std::size_t oldest = 0;
    for (std::size_t i = 1; i < ev.observations.size(); ++i) {
      if (obsOlder(ev.observations[i], ev.observations[oldest])) oldest = i;
    }
    if (!obsOlder(ev.observations[oldest], o)) return false;  // newcomer is the oldest
    ev.observations.erase_at(oldest);
  }
  if (!ev.observations.push_back(o)) return false;  // cannot fail: a full list had its oldest erased above
  std::size_t i = ev.observations.size() - 1;
  while (i > 0 && obsKeyLess(ev.observations[i], ev.observations[i - 1])) {
    const SensorObservation tmp = ev.observations[i];
    ev.observations[i] = ev.observations[i - 1];
    ev.observations[i - 1] = tmp;
    --i;
  }
  return true;
}

// Fold `d`'s history (alerts, timings, slots, dismissal) into `k`.
void absorbMetadata(DetectedEvent& k, const DetectedEvent& d) noexcept {
  if (k.signature == 0) k.signature = d.signature;
  k.advisory_sent = k.advisory_sent || d.advisory_sent;
  k.priority_sent = k.priority_sent || d.priority_sent;
  if (k.advisory_ready_ms == 0 || (d.advisory_ready_ms != 0 && d.advisory_ready_ms < k.advisory_ready_ms)) {
    k.advisory_ready_ms = d.advisory_ready_ms;
  }
  if (k.priority_ready_ms == 0 || (d.priority_ready_ms != 0 && d.priority_ready_ms < k.priority_ready_ms)) {
    k.priority_ready_ms = d.priority_ready_ms;
  }
  if (k.first_seen_ms == 0 || (d.first_seen_ms != 0 && d.first_seen_ms < k.first_seen_ms)) {
    k.first_seen_ms = d.first_seen_ms;
  }
  k.last_update_ms = std::max(k.last_update_ms, d.last_update_ms);
  if (k.verify_started_ms == 0 || (d.verify_started_ms != 0 && d.verify_started_ms < k.verify_started_ms)) {
    k.verify_started_ms = d.verify_started_ms;
  }
  for (std::size_t s = 0; s < 2; ++s) {
    if (k.verifier[s].id == kNoDrone && d.verifier[s].id != kNoDrone &&
        d.verifier[s].id != k.verifier[1 - s].id) {
      k.verifier[s] = d.verifier[s];
    }
  }
  if (d.status == EventStatus::DISMISSED) k.status = EventStatus::DISMISSED;
}

// Fold `d` into `k` (k keeps its id).
void absorb(DetectedEvent& k, const DetectedEvent& d) noexcept {
  for (const auto& o : d.observations) (void)insertObservation(k, o);
  absorbMetadata(k, d);
}

// A total order on readings that depends only on their content. Fixes come
// first, tightest first, because they anchor clusters; then bearings, grouped
// by signature. Ties are broken by every remaining field, so two different
// readings never compare equal.
bool canonicalLess(const SensorObservation& a, const SensorObservation& b) noexcept {
  if (a.has_fix != b.has_fix) return a.has_fix;
  if (a.has_fix && a.fix_cep_m != b.fix_cep_m) return a.fix_cep_m < b.fix_cep_m;
  if (!a.has_fix && a.signature != b.signature) return a.signature < b.signature;  // bearings: one run per frequency
  if (a.observer != b.observer) return a.observer < b.observer;
  if (a.sensor != b.sensor) return static_cast<std::uint8_t>(a.sensor) < static_cast<std::uint8_t>(b.sensor);
  if (a.timestamp_ms != b.timestamp_ms) return a.timestamp_ms < b.timestamp_ms;
  if (a.signature != b.signature) return a.signature < b.signature;
  if (a.event_hint != b.event_hint) return a.event_hint < b.event_hint;
  if (a.bearing_deg != b.bearing_deg) return a.bearing_deg < b.bearing_deg;
  if (a.bearing_sigma_deg != b.bearing_sigma_deg) return a.bearing_sigma_deg < b.bearing_sigma_deg;
  if (a.fix.lat_deg != b.fix.lat_deg) return a.fix.lat_deg < b.fix.lat_deg;
  if (a.fix.lon_deg != b.fix.lon_deg) return a.fix.lon_deg < b.fix.lon_deg;
  if (a.observer_position.lat_deg != b.observer_position.lat_deg) {
    return a.observer_position.lat_deg < b.observer_position.lat_deg;
  }
  if (a.observer_position.lon_deg != b.observer_position.lon_deg) {
    return a.observer_position.lon_deg < b.observer_position.lon_deg;
  }
  if (a.confidence != b.confidence) return a.confidence < b.confidence;
  return static_cast<std::uint8_t>(a.class_hint) < static_cast<std::uint8_t>(b.class_hint);
}

bool sameReading(const SensorObservation& a, const SensorObservation& b) noexcept {
  return !canonicalLess(a, b) && !canonicalLess(b, a);
}

bool holdsKey(const DetectedEvent& ev, DroneId observer, SensorType sensor) noexcept {
  for (const auto& e : ev.observations) {
    if (e.observer == observer && e.sensor == sensor) return true;
  }
  return false;
}

bool sharesKey(const DetectedEvent& a, const DetectedEvent& b) noexcept {
  for (const auto& o : a.observations) {
    if (holdsKey(b, o.observer, o.sensor)) return true;
  }
  return false;
}

// Event ids are (origin << 16) | counter. Counters 1..0x7FFF are handed out by
// allocateEventId(); 0x8000..0xFFFF are derived by the re-clustering for a
// source that no reading names, so the two never collide.
constexpr std::uint16_t kAllocatedCounterMax = 0x7FFFU;
constexpr std::uint16_t kDerivedCounterBit = 0x8000U;

constexpr double kUngatedMetric = 1.0e12;  // ranks signature-only matches after geometric ones

std::int64_t quantiseScore(float s) noexcept { return std::llround(static_cast<double>(s) * 1.0e4); }

constexpr std::uint32_t kMaxCountedGap = 1000U;  // a larger gap would wreck the loss statistic

// Sequence numbers restart with each boot session (the security layer has
// already rejected duplicates, replays and older sessions).
bool trackSeq(PeerRecord& p, std::uint32_t session, std::uint32_t seq) noexcept {
  if (session != p.last_session) {
    p.last_session = session;
    p.last_seq = seq;
    ++p.rx_count;
    return true;
  }
  if (seq <= p.last_seq) return false;  // reordered
  const std::uint32_t gap = seq - p.last_seq - 1U;
  if (gap <= kMaxCountedGap) p.missed_count += gap;
  p.last_seq = seq;
  ++p.rx_count;
  return true;
}

}  // namespace

// ---------------------------------------------------------------- fusion

void fuseEvent(DetectedEvent& ev, const MeshConfig& cfg) noexcept {
  ev.modality_mask = 0;
  ev.observer_count = 0;
  if (ev.observations.empty()) {
    ev.localized = false;
    ev.confidence = 0.0f;
    return;
  }

  EventClass cls = EventClass::UNKNOWN;
  double miss = 1.0;  // product of (1 - c): noisy-OR across independent readings
  double lat_sum = 0.0;
  double dlon_sum = 0.0;  // longitudes relative to ref_lon, unwrapped across the antimeridian
  double pos_n = 0.0;
  const double ref_lon = ev.observations[0].observer_position.lon_deg;
  std::uint32_t signature = 0;  // re-derived each time so every drone agrees
  DroneId last_observer = kNoDrone;
  for (const auto& o : ev.observations) {
    ev.modality_mask = static_cast<SensorMask>(ev.modality_mask | maskOf(o.sensor));
    if (o.observer != last_observer) {
      ++ev.observer_count;
      last_observer = o.observer;
    }
    const float sev_o = eventSeverity(o.class_hint);
    const float sev_c = eventSeverity(cls);
    if (sev_o > sev_c || (sev_o == sev_c && o.class_hint > cls)) cls = o.class_hint;
    miss *= 1.0 - std::min(0.99, std::max(0.0, static_cast<double>(o.confidence)));
    lat_sum += o.observer_position.lat_deg;
    dlon_sum += wrap180(o.observer_position.lon_deg - ref_lon);
    pos_n += 1.0;
    if (o.has_fix) {
      lat_sum += o.fix.lat_deg;
      dlon_sum += wrap180(o.fix.lon_deg - ref_lon);
      pos_n += 1.0;
    }
    if (signature == 0 && o.signature != 0) signature = o.signature;
  }
  ev.cls = cls;
  ev.signature = signature;

  GeoPoint origin;
  origin.lat_deg = lat_sum / pos_n;
  origin.lon_deg = wrap180(ref_lon + dlon_sum / pos_n);
  const LocalFrame frame(origin);

  struct Estimate {
    Vec2 p{};
    double sigma_m = 0.0;
  };
  std::array<Estimate, kMaxObsPerEvent + 1> est{};
  std::size_t n_est = 0;
  std::array<BearingLine, kMaxObsPerEvent> lines{};
  std::size_t n_lines = 0;
  const SensorObservation* newest_bearing = nullptr;
  double alt_sum = 0.0;
  double alt_n = 0.0;

  for (const auto& o : ev.observations) {
    if (o.has_fix) {
      est[n_est++] = {frame.toEnu(o.fix), std::max(static_cast<double>(o.fix_cep_m), 10.0) / kCepPerSigma};
      alt_sum += static_cast<double>(o.fix.alt_m);
      alt_n += 1.0;
    } else {
      BearingLine bl;
      bl.origin = frame.toEnu(o.observer_position);
      bl.bearing_rad = static_cast<double>(o.bearing_deg) * kDeg2Rad;
      bl.sigma_rad = std::max(static_cast<double>(o.bearing_sigma_deg), cfg.min_bearing_sigma_deg) * kDeg2Rad;
      lines[n_lines++] = bl;
      if (newest_bearing == nullptr || obsOlder(*newest_bearing, o)) newest_bearing = &o;
    }
  }

  TriangulationResult tri;
  if (n_lines >= 2 && triangulate(lines.data(), n_lines, cfg.min_geometry_eigen, tri) &&
      std::hypot(tri.position.e, tri.position.n) <= cfg.max_association_range_m) {  // inside the flat frame's range
    est[n_est++] = {tri.position, std::max(tri.cep_m, 10.0) / kCepPerSigma};
  }

  const float alt = alt_n > 0.0 ? static_cast<float>(alt_sum / alt_n) : 0.0f;
  bool fused = false;
  if (n_est > 0) {
    double sw = 0.0;
    double se = 0.0;
    double sn = 0.0;
    for (std::size_t i = 0; i < n_est; ++i) {
      const double w = 1.0 / (est[i].sigma_m * est[i].sigma_m);
      sw += w;
      se += w * est[i].p.e;
      sn += w * est[i].p.n;
    }
    const GeoPoint p = frame.fromEnu(Vec2{se / sw, sn / sw}, alt);
    if (isPlausible(p)) {  // readings too far apart for the flat frame give nothing on the earth
      ev.position = p;
      ev.cep_m = static_cast<float>(kCepPerSigma / std::sqrt(sw));
      fused = true;
    }
  }
  ev.localized = fused;
  if (!fused) {
    if (newest_bearing != nullptr) {
      ev.position = destinationPoint(newest_bearing->observer_position,
                                     static_cast<double>(newest_bearing->bearing_deg), cfg.provisional_range_m);
    }
    ev.cep_m = static_cast<float>(cfg.provisional_cep_m);
  }

  double cap = popCount(ev.modality_mask) >= 2 ? cfg.multi_modality_conf_cap : cfg.single_modality_conf_cap;
  if (!ev.localized) cap = std::min(cap, cfg.unlocalized_conf_cap);
  ev.confidence = static_cast<float>(std::min(1.0 - miss, cap));
}

// ------------------------------------------------------------ membership

SituationalAwarenessTable::SituationalAwarenessTable(DroneId self, const MeshConfig& cfg) noexcept
    : self_id_(self), cfg_(cfg) {
  self_.id = self;
}

void SituationalAwarenessTable::setSelf(const DroneState& st) noexcept {
  self_ = st;
  self_.id = self_id_;
}

bool SituationalAwarenessTable::updatePeer(const DroneState& st, std::uint32_t session, std::uint32_t seq,
                                           TimeMs now) noexcept {
  if (st.id == kNoDrone || st.id == self_id_ || !isPlausible(st)) return false;
  if (st.timestamp_ms > now + cfg_.max_clock_skew_ms) return false;
  for (auto& p : peers_) {
    if (p.state.id != st.id) continue;
    p.last_heard_ms = now;
    if (trackSeq(p, session, seq) || st.timestamp_ms > p.state.timestamp_ms) p.state = st;
    return false;
  }
  if (peers_.full()) return false;
  PeerRecord r;
  r.state = st;
  r.last_heard_ms = now;
  r.last_session = session;
  r.last_seq = seq;
  r.rx_count = 1;
  return peers_.push_back(r);
}

void SituationalAwarenessTable::notePacket(DroneId src, std::uint32_t session, std::uint32_t seq, TimeMs now) noexcept {
  for (auto& p : peers_) {
    if (p.state.id != src) continue;
    p.last_heard_ms = now;
    (void)trackSeq(p, session, seq);
    return;
  }
}

std::size_t SituationalAwarenessTable::expirePeers(TimeMs now) noexcept {
  std::size_t dropped = 0;
  for (std::size_t i = peers_.size(); i-- > 0;) {
    if (elapsedMs(now, peers_[i].last_heard_ms) > cfg_.peer_timeout_ms) {
      peers_.erase_at(i);
      ++dropped;
    }
  }
  return dropped;
}

bool SituationalAwarenessTable::removePeer(DroneId id) noexcept {
  for (std::size_t i = 0; i < peers_.size(); ++i) {
    if (peers_[i].state.id == id) {
      peers_.erase_at(i);
      return true;
    }
  }
  return false;
}

const PeerRecord* SituationalAwarenessTable::findPeer(DroneId id) const noexcept {
  for (const auto& p : peers_) {
    if (p.state.id == id) return &p;
  }
  return nullptr;
}

const DroneState* SituationalAwarenessTable::findDrone(DroneId id) const noexcept {
  if (id == self_id_) return &self_;
  const PeerRecord* p = findPeer(id);
  return p != nullptr ? &p->state : nullptr;
}

void SituationalAwarenessTable::snapshotDrones(DroneList& out) const noexcept {
  out.clear();
  // Cannot fail: peers_ holds kMaxPackSize - 1, so self + peers fit DroneList exactly.
  (void)out.push_back(self_);
  for (const auto& p : peers_) (void)out.push_back(p.state);
  // Sort by id so every drone iterates the pack in the same order.
  for (std::size_t i = 1; i < out.size(); ++i) {
    std::size_t j = i;
    while (j > 0 && out[j].id < out[j - 1].id) {
      const DroneState tmp = out[j];
      out[j] = out[j - 1];
      out[j - 1] = tmp;
      --j;
    }
  }
}

// ---------------------------------------------------------------- events

void SituationalAwarenessTable::seedEventCounter(std::uint16_t seed) noexcept {
  event_counter_ = static_cast<std::uint16_t>(seed & kAllocatedCounterMax);
}

bool SituationalAwarenessTable::idInUse(EventId id) const noexcept {
  if (findEvent(id) != nullptr) return true;
  for (const auto& a : aliases_) {
    if (a.from == id || a.to == id) return true;
  }
  return false;
}

EventId SituationalAwarenessTable::allocateEventId() noexcept {
  // At most kMaxEvents + 2 * kMaxAliases ids can be in use, so this always ends.
  for (std::size_t attempt = 0; attempt <= kMaxEvents + 2 * kMaxAliases; ++attempt) {
    ++event_counter_;
    if (event_counter_ == 0 || event_counter_ > kAllocatedCounterMax) event_counter_ = 1;
    const EventId id = (static_cast<EventId>(self_id_) << 16U) | static_cast<EventId>(event_counter_);
    if (!idInUse(id)) return id;
  }
  return kNoEvent;  // unreachable while the capacities above hold
}

EventId SituationalAwarenessTable::resolve(EventId id) const noexcept {
  for (int hop = 0; hop < 4; ++hop) {
    bool moved = false;
    for (const auto& a : aliases_) {
      if (a.from == id) {
        id = a.to;
        moved = true;
        break;
      }
    }
    if (!moved) break;
  }
  return id;
}

const DetectedEvent* SituationalAwarenessTable::findEvent(EventId id) const noexcept {
  if (id == kNoEvent) return nullptr;
  for (const auto& ev : events_) {
    if (ev.id == id) return &ev;
  }
  return nullptr;
}

DetectedEvent* SituationalAwarenessTable::findEvent(EventId id) noexcept {
  if (id == kNoEvent) return nullptr;
  for (auto& ev : events_) {
    if (ev.id == id) return &ev;
  }
  return nullptr;
}

void SituationalAwarenessTable::addAlias(EventId from, EventId to) noexcept {
  if (from == to || from == kNoEvent || to == kNoEvent) return;
  for (std::size_t i = aliases_.size(); i-- > 0;) {
    if (aliases_[i].from == from) aliases_.erase_at(i);
  }
  for (auto& a : aliases_) {
    if (a.to == from) a.to = to;  // keep chains one hop long
  }
  if (aliases_.full()) aliases_.erase_at(0);
  (void)aliases_.push_back(Alias{from, to});  // cannot fail: the oldest was dropped if full
}

void SituationalAwarenessTable::rename(EventId from, EventId to) noexcept {
  DetectedEvent* ev = findEvent(from);
  if (ev == nullptr) return;
  for (std::size_t i = aliases_.size(); i-- > 0;) {
    if (aliases_[i].from == to) aliases_.erase_at(i);
  }
  ev->id = to;
  addAlias(from, to);
}

// Can reading `o` come from the source behind `ev`? Different non-zero
// signatures never match. A localised event must also pass a geometric gate:
// many sources share a frequency (every 406 MHz distress beacon does).
// An unlocalised event has no geometry yet, so a shared signature is all
// there is to go on; such matches rank below any geometric match.
bool SituationalAwarenessTable::gatePasses(const SensorObservation& o, const DetectedEvent& ev,
                                           double& metric) const noexcept {
  if (ev.signature != 0 && o.signature != 0 && ev.signature != o.signature) return false;
  // Beyond pack scale nothing can be the same source, and the flat fusion frame would not hold.
  if (distanceM(o.has_fix ? o.fix : o.observer_position, ev.position) > cfg_.max_association_range_m) return false;
  if (!ev.localized) {
    metric = kUngatedMetric;
    return ev.signature != 0 && ev.signature == o.signature;
  }
  if (o.has_fix) {
    const double d = distanceM(o.fix, ev.position);
    const double gate = cfg_.association_gate_m +
                        cfg_.association_cep_factor * (static_cast<double>(ev.cep_m) + static_cast<double>(o.fix_cep_m));
    metric = d;
    return d <= gate;
  }
  const double range = std::max(distanceM(o.observer_position, ev.position), 1.0);
  const double diff = angleDiffDeg(static_cast<double>(o.bearing_deg), bearingDeg(o.observer_position, ev.position));
  const double ang_unc = std::atan2(static_cast<double>(ev.cep_m), range) * kRad2Deg;
  const double sigma = std::max(static_cast<double>(o.bearing_sigma_deg), cfg_.min_bearing_sigma_deg);
  metric = diff;
  return diff <= 3.0 * sigma + cfg_.bearing_gate_deg + ang_unc;
}

// One sensor sees one source in one place at a time. Before an event is
// localised there is no geometry to gate on, so a reading may join an
// unlocalised event that holds this observer's reading from this sensor only
// if it agrees with that reading: the same source seen again, not a second
// source on the same frequency.
bool SituationalAwarenessTable::consistentWithOwnReading(const SensorObservation& o,
                                                         const DetectedEvent& ev) const noexcept {
  if (ev.localized) return true;  // the geometric gate decides
  for (const auto& e : ev.observations) {
    if (e.observer != o.observer || e.sensor != o.sensor) continue;
    if (e.has_fix != o.has_fix) return false;
    if (o.has_fix) {
      const double gate = cfg_.association_gate_m +
                          cfg_.association_cep_factor * (static_cast<double>(e.fix_cep_m) + static_cast<double>(o.fix_cep_m));
      return distanceM(e.fix, o.fix) <= gate;
    }
    const double sigma = std::max({static_cast<double>(e.bearing_sigma_deg), static_cast<double>(o.bearing_sigma_deg),
                                   cfg_.min_bearing_sigma_deg});
    return angleDiffDeg(static_cast<double>(e.bearing_deg), static_cast<double>(o.bearing_deg)) <=
           3.0 * sigma + cfg_.bearing_gate_deg;
  }
  return true;
}

EventId SituationalAwarenessTable::associate(const SensorObservation& o) const noexcept {
  double metric = 0.0;
  if (o.event_hint != kNoEvent) {  // a hint is a preference, not a bypass of the gate
    const DetectedEvent* h = findEvent(resolve(o.event_hint));
    if (h != nullptr && gatePasses(o, *h, metric) && consistentWithOwnReading(o, *h)) return h->id;
  }
  EventId best = kNoEvent;
  double best_metric = 0.0;
  for (const auto& ev : events_) {
    if (!gatePasses(o, ev, metric) || !consistentWithOwnReading(o, ev)) continue;
    if (best == kNoEvent || metric < best_metric || (metric == best_metric && ev.id < best)) {
      best_metric = metric;
      best = ev.id;
    }
  }
  return best;
}

bool SituationalAwarenessTable::evictOne() noexcept {
  std::size_t victim = events_.size();
  for (std::size_t i = 0; i < events_.size(); ++i) {
    const DetectedEvent& ev = events_[i];
    if (ev.verifier[0].id != kNoDrone || ev.verifier[1].id != kNoDrone) continue;
    if (victim == events_.size() || ev.urgency < events_[victim].urgency ||
        (ev.urgency == events_[victim].urgency && ev.id < events_[victim].id)) {
      victim = i;
    }
  }
  if (victim == events_.size()) return false;
  events_.erase_at(victim);
  return true;
}

EventId SituationalAwarenessTable::ingestObservation(const SensorObservation& o, TimeMs now) noexcept {
  if (!isPlausible(o)) return kNoEvent;
  if (o.timestamp_ms > now + cfg_.max_clock_skew_ms) return kNoEvent;              // from the future
  if (elapsedMs(now, o.timestamp_ms) > cfg_.observation_max_age_ms) return kNoEvent;  // already expired

  // The hint names an event we already hold under that id (possibly a
  // different source that failed the gate); such an id can't be reused.
  const bool hint_taken = o.event_hint != kNoEvent && idInUse(o.event_hint);

  DetectedEvent* ev = nullptr;
  const EventId a = associate(o);
  if (a != kNoEvent) {
    ev = findEvent(a);
    if (ev != nullptr && o.event_hint != kNoEvent && o.event_hint != ev->id && !hint_taken) {
      // Same source known under two ids: converge on the lower one.
      if (o.event_hint < ev->id) {
        rename(ev->id, o.event_hint);
      } else {
        addAlias(o.event_hint, ev->id);
      }
    }
  }

  if (ev == nullptr) {
    if (events_.full() && !evictOne()) return kNoEvent;
    DetectedEvent n;
    n.id = (o.event_hint != kNoEvent && !hint_taken) ? o.event_hint : allocateEventId();
    if (n.id == kNoEvent) return kNoEvent;
    for (std::size_t i = aliases_.size(); i-- > 0;) {
      if (aliases_[i].from == n.id) aliases_.erase_at(i);
    }
    n.first_seen_ms = o.timestamp_ms;
    if (!events_.push_back(n)) return kNoEvent;  // cannot fail: checked full/evicted above
    ev = &events_[events_.size() - 1];
  }
  if (ev == nullptr) return kNoEvent;  // unreachable; keeps the non-null path explicit

  if (!insertObservation(*ev, o)) return ev->id;  // stale or duplicate reading
  ev->last_update_ms = std::max(ev->last_update_ms, o.timestamp_ms);
  if (ev->first_seen_ms == 0 || o.timestamp_ms < ev->first_seen_ms) ev->first_seen_ms = o.timestamp_ms;
  fuseEvent(*ev, cfg_);
  return ev->id;
}

bool SituationalAwarenessTable::shouldMerge(const DetectedEvent& a, const DetectedEvent& b) const noexcept {
  if (a.signature != 0 && b.signature != 0 && a.signature != b.signature) return false;
  if (a.localized && b.localized) {
    const double gate = cfg_.association_gate_m +
                        cfg_.association_cep_factor * (static_cast<double>(a.cep_m) + static_cast<double>(b.cep_m));
    return distanceM(a.position, b.position) <= gate;
  }
  // At least one side has no position: only a shared signature can link them,
  // and only at pack scale.
  if (a.signature == 0 || a.signature != b.signature) return false;
  if (distanceM(a.position, b.position) > cfg_.max_association_range_m) return false;
  const DetectedEvent* loc = a.localized ? &a : (b.localized ? &b : nullptr);
  if (loc == nullptr) return true;  // two bearing-only events on one frequency: nothing separates them yet
  const DetectedEvent& other = loc == &a ? b : a;
  double metric = 0.0;
  for (const auto& o : other.observations) {
    if (!gatePasses(o, *loc, metric)) return false;  // its bearings must point at the fix
  }
  return true;
}

void SituationalAwarenessTable::mergeDuplicates() noexcept {
  bool again = true;
  for (std::size_t guard = 0; again && guard < kMaxEvents; ++guard) {
    again = false;
    for (std::size_t i = 0; i < events_.size() && !again; ++i) {
      for (std::size_t j = i + 1; j < events_.size(); ++j) {
        if (!shouldMerge(events_[i], events_[j])) continue;
        const std::size_t keep = events_[i].id < events_[j].id ? i : j;
        const std::size_t drop = keep == i ? j : i;
        absorb(events_[keep], events_[drop]);
        addAlias(events_[drop].id, events_[keep].id);
        events_.erase_at(drop);
        const std::size_t k = keep > drop ? keep - 1 : keep;
        fuseEvent(events_[k], cfg_);
        again = true;
        break;
      }
    }
  }
}

bool SituationalAwarenessTable::slotValid(const VerifierSlot& s, const DetectedEvent& ev, TimeMs now) const noexcept {
  const DroneState* d = findDrone(s.id);
  if (d == nullptr) return false;
  if (d->mode != DroneMode::PACK_ROUTING && d->mode != DroneMode::VERIFYING) return false;
  const TimeMs age = elapsedMs(now, s.since_ms);
  if (d->mode == DroneMode::VERIFYING) {
    return resolve(d->tasked_event) == ev.id || age <= cfg_.slot_adopt_grace_ms;
  }
  return age <= cfg_.slot_adopt_grace_ms;  // claimed but not adopted
}

void SituationalAwarenessTable::validateSlots(DetectedEvent& ev, TimeMs now) const noexcept {
  for (auto& s : ev.verifier) {
    if (s.id != kNoDrone && !slotValid(s, ev, now)) s = VerifierSlot{};
  }
}

// ------------------------------------------------------- re-clustering
//
// Every reading in the table is grouped again from scratch on each refresh,
// in stages that each depend only on the readings:
//   1. Fixes, tightest first: each joins the nearest cluster whose gate it
//      passes, or starts one.
//   2. Bearings that pass the gate of a cluster from stage 1 join the best one.
//   3. Remaining bearings, one frequency at a time: all of them form one group
//      (a second reading from the same sensor starts a second group), then any
//      bearing that disagrees with the fix from the rest of its group moves to
//      the next group. Building from the whole group, not reading by reading,
//      keeps a noisy early pair of bearings from splitting a source.
//   4. Clusters that are plainly one source are merged, lowest first.
// A cluster holds at most one reading per (observer, sensor) and at most
// kMaxObsPerEvent readings (the newest). Ids and history are carried over
// afterwards. Nothing here depends on arrival order or on the current
// grouping, so every drone holding the same readings gets the same events.

namespace {

constexpr int kRefinePasses = 3;

// The newest kMaxObsPerEvent of `members` (indices into refs), others marked dropped.
template <typename ObsAt>
void keepNewest(std::uint16_t* members, std::size_t& count, const ObsAt& obsAt) noexcept {
  while (count > kMaxObsPerEvent) {
    std::size_t oldest = 0;
    for (std::size_t i = 1; i < count; ++i) {
      if (obsOlder(obsAt(members[i]), obsAt(members[oldest]))) oldest = i;
    }
    for (std::size_t i = oldest + 1; i < count; ++i) members[i - 1] = members[i];
    --count;
  }
}

}  // namespace

bool SituationalAwarenessTable::recluster() noexcept {
  std::size_t n = 0;
  for (std::size_t e = 0; e < events_.size(); ++e) {
    for (std::size_t j = 0; j < events_[e].observations.size(); ++j) {
      refs_[n++] = ObsRef{static_cast<std::uint8_t>(e), static_cast<std::uint8_t>(j)};
    }
  }
  const auto obsAt = [this](std::size_t k) -> const SensorObservation& {
    return events_[refs_[k].ev].observations[refs_[k].ob];
  };
  for (std::size_t i = 1; i < n; ++i) {  // insertion sort: the pool is at most kMaxReadings
    const ObsRef cur = refs_[i];
    const SensorObservation& co = events_[cur.ev].observations[cur.ob];
    std::size_t j = i;
    while (j > 0 && canonicalLess(co, obsAt(j - 1))) {
      refs_[j] = refs_[j - 1];
      --j;
    }
    refs_[j] = cur;
  }
  for (std::size_t k = 0; k < n; ++k) cluster_of_[k] = kNoCluster;
  const auto duplicate = [&](std::size_t k) { return k > 0 && sameReading(obsAt(k), obsAt(k - 1)); };
  const auto add = [&](std::size_t k, std::size_t c) {
    (void)insertObservation(scratch_[c], obsAt(k));  // false only drops the oldest of a full cluster
    fuseEvent(scratch_[c], cfg_);
    cluster_of_[k] = static_cast<std::uint8_t>(c);
  };
  const auto bestCluster = [&](std::size_t k, std::size_t limit) {
    const SensorObservation& o = obsAt(k);
    std::size_t best = limit;
    double best_metric = 0.0;
    for (std::size_t c = 0; c < limit; ++c) {
      double metric = 0.0;
      if (holdsKey(scratch_[c], o.observer, o.sensor) || !gatePasses(o, scratch_[c], metric)) continue;
      if (best == limit || metric < best_metric) {  // ties keep the earlier cluster
        best = c;
        best_metric = metric;
      }
    }
    return best;
  };

  scratch_.clear();
  std::size_t first_bearing = 0;
  while (first_bearing < n && obsAt(first_bearing).has_fix) ++first_bearing;

  // 1. Fixes.
  for (std::size_t k = 0; k < first_bearing; ++k) {
    if (duplicate(k)) {
      cluster_of_[k] = cluster_of_[k - 1];
      continue;
    }
    std::size_t c = bestCluster(k, scratch_.size());
    if (c == scratch_.size() && !scratch_.push_back(DetectedEvent{})) return false;
    add(k, c);
  }

  // 2. Bearings that fit an anchored cluster.
  const std::size_t anchored = scratch_.size();
  for (std::size_t k = first_bearing; k < n; ++k) {
    if (duplicate(k)) {
      cluster_of_[k] = cluster_of_[k - 1];
      continue;
    }
    const std::size_t c = bestCluster(k, anchored);
    if (c < anchored) add(k, c);
  }

  // 3. The rest, one frequency at a time.
  std::array<std::uint16_t, kMaxReadings> run{};
  std::array<std::uint8_t, kMaxReadings> group{};
  std::array<std::uint16_t, kMaxReadings> members{};
  for (std::size_t r0 = first_bearing; r0 < n;) {
    const std::uint32_t sig = obsAt(r0).signature;
    std::size_t r1 = r0;
    std::size_t m = 0;
    while (r1 < n && obsAt(r1).signature == sig) {
      if (cluster_of_[r1] == kNoCluster && !duplicate(r1)) run[m++] = static_cast<std::uint16_t>(r1);
      ++r1;
    }
    // Duplicates follow their twin once it is placed (below).
    // A bearing joins the first group that has no reading from its sensor and
    // whose first bearing was taken within pack range of it.
    std::size_t n_groups = 0;
    std::array<std::uint16_t, 0xFF> anchor{};  // first member of each group
    for (std::size_t i = 0; i < m; ++i) {
      const SensorObservation& o = obsAt(run[i]);
      std::size_t g = 0;
      if (sig == 0) {
        g = n_groups;  // without a signature only geometry could link bearings, and a lone bearing has none
      } else {
        for (; g < n_groups; ++g) {
          bool key_taken = false;
          for (std::size_t j = 0; j < i; ++j) {
            if (group[j] == g && obsAt(run[j]).observer == o.observer && obsAt(run[j]).sensor == o.sensor) key_taken = true;
          }
          const bool in_range = distanceM(obsAt(run[anchor[g]]).observer_position, o.observer_position) <=
                                cfg_.max_association_range_m;
          if (!key_taken && in_range) break;
        }
      }
      g = std::min<std::size_t>(g, 0xFE);
      group[i] = static_cast<std::uint8_t>(g);
      if (g >= n_groups) {
        anchor[g] = static_cast<std::uint16_t>(i);
        n_groups = g + 1;
      }
    }

    const auto collect = [&](std::size_t g, std::size_t& count) {
      count = 0;
      for (std::size_t i = 0; i < m; ++i) {
        if (group[i] == g) members[count++] = static_cast<std::uint16_t>(i);
      }
    };
    const auto groupHoldsKey = [&](std::size_t g, const SensorObservation& o) {
      for (std::size_t i = 0; i < m; ++i) {
        if (group[i] == g && obsAt(run[i]).observer == o.observer && obsAt(run[i]).sensor == o.sensor) return true;
      }
      return false;
    };
    for (int pass = 0; sig != 0 && pass < kRefinePasses; ++pass) {
      bool moved = false;
      for (std::size_t g = 0; g < n_groups && g < 0xFE; ++g) {
        std::size_t count = 0;
        collect(g, count);
        if (count < 3) continue;  // two bearings alone cannot disagree with each other
        for (std::size_t a = 0; a < count; ++a) {
          DetectedEvent rest;
          for (std::size_t b = 0; b < count; ++b) {
            if (b != a) (void)insertObservation(rest, obsAt(run[members[b]]));
          }
          fuseEvent(rest, cfg_);
          double metric = 0.0;
          const SensorObservation& o = obsAt(run[members[a]]);
          if (!rest.localized || gatePasses(o, rest, metric)) continue;
          std::size_t to = g + 1;
          while (to < 0xFE && to < n_groups &&
                 (groupHoldsKey(to, o) || distanceM(obsAt(run[anchor[to]]).observer_position, o.observer_position) >
                                              cfg_.max_association_range_m)) {
            ++to;
          }
          to = std::min<std::size_t>(to, 0xFE);
          group[members[a]] = static_cast<std::uint8_t>(to);
          if (to >= n_groups) {
            anchor[to] = members[a];
            n_groups = to + 1;
          }
          moved = true;
        }
      }
      if (!moved) break;
    }

    for (std::size_t g = 0; g < n_groups; ++g) {
      std::size_t count = 0;
      collect(g, count);
      if (count == 0) continue;
      std::array<std::uint16_t, kMaxReadings> idx{};
      for (std::size_t i = 0; i < count; ++i) idx[i] = run[members[i]];
      keepNewest(idx.data(), count, obsAt);
      if (!scratch_.push_back(DetectedEvent{})) return false;
      for (std::size_t i = 0; i < count; ++i) add(idx[i], scratch_.size() - 1);
    }
    for (std::size_t k = r0; k < r1; ++k) {
      if (duplicate(k)) cluster_of_[k] = cluster_of_[k - 1];
    }
    r0 = r1;
  }

  // 4. Merge clusters that are plainly one source, lower index first.
  bool again = true;
  for (std::size_t guard = 0; again && guard < kMaxEvents; ++guard) {
    again = false;
    for (std::size_t i = 0; i < scratch_.size() && !again; ++i) {
      for (std::size_t j = i + 1; j < scratch_.size(); ++j) {
        if (sharesKey(scratch_[i], scratch_[j]) || !shouldMerge(scratch_[i], scratch_[j])) continue;
        for (const auto& o : scratch_[j].observations) (void)insertObservation(scratch_[i], o);
        fuseEvent(scratch_[i], cfg_);
        scratch_.erase_at(j);
        for (std::size_t k = 0; k < n; ++k) {
          if (cluster_of_[k] == j) {
            cluster_of_[k] = static_cast<std::uint8_t>(i);
          } else if (cluster_of_[k] != kNoCluster && cluster_of_[k] > j) {
            --cluster_of_[k];
          }
        }
        again = true;
        break;
      }
    }
  }

  assignClusterIds();
  inheritMetadata(n);
  events_ = scratch_;
  return true;
}

// A cluster is named by the lowest event id that its readings carry and that
// it holds the most readings of (ties to the lower cluster). Every reading
// carries the id its observer filed it under, so this converges on the same
// lowest-id-wins rule that live association uses.
void SituationalAwarenessTable::assignClusterIds() noexcept {
  votes_.clear();
  for (std::size_t c = 0; c < scratch_.size(); ++c) {
    for (const auto& o : scratch_[c].observations) {
      if (o.event_hint == kNoEvent) continue;
      bool counted = false;
      for (auto& v : votes_) {
        if (v.hint == o.event_hint && v.cluster == c) {
          ++v.count;
          counted = true;
          break;
        }
      }
      if (!counted) (void)votes_.push_back(HintVote{o.event_hint, static_cast<std::uint8_t>(c), 1});  // fits: one per reading
    }
  }
  for (std::size_t i = 1; i < votes_.size(); ++i) {  // by (hint, cluster)
    const HintVote cur = votes_[i];
    std::size_t j = i;
    while (j > 0 && (cur.hint < votes_[j - 1].hint || (cur.hint == votes_[j - 1].hint && cur.cluster < votes_[j - 1].cluster))) {
      votes_[j] = votes_[j - 1];
      --j;
    }
    votes_[j] = cur;
  }

  new_ids_.fill(kNoEvent);
  for (std::size_t i = 0; i < votes_.size();) {
    std::size_t end = i;
    std::size_t win = i;
    while (end < votes_.size() && votes_[end].hint == votes_[i].hint) {
      if (votes_[end].count > votes_[win].count) win = end;
      ++end;
    }
    const std::uint8_t c = votes_[win].cluster;
    if (new_ids_[c] == kNoEvent) new_ids_[c] = votes_[i].hint;  // hints ascend: the first won is the lowest
    i = end;
  }

  const auto taken = [this](EventId id) {
    for (std::size_t c = 0; c < scratch_.size(); ++c) {
      if (new_ids_[c] == id) return true;
    }
    for (const auto& v : votes_) {
      if (v.hint == id) return true;
    }
    return false;
  };
  for (std::size_t c = 0; c < scratch_.size(); ++c) {
    if (new_ids_[c] != kNoEvent) continue;
    // No reading names this source: derive an id from its first reading, the same on every drone.
    const SensorObservation& o = scratch_[c].observations[0];
    std::uint32_t h = static_cast<std::uint32_t>(o.timestamp_ms ^ (o.timestamp_ms >> 32U)) * 0x9E3779B1U;
    h ^= static_cast<std::uint32_t>(o.sensor) * 0x85EBCA6BU;
    for (std::size_t probe = 0; probe <= kMaxReadings + kMaxEvents; ++probe) {
      const auto counter = static_cast<std::uint16_t>(kDerivedCounterBit | ((h + probe) & 0x7FFFU));
      const EventId id = (static_cast<EventId>(o.observer) << 16U) | counter;
      if (!taken(id)) {
        new_ids_[c] = id;
        break;
      }
    }
  }
}

// Each old event's history goes to one heir: the cluster that kept its id, or
// else the one that took most of its readings. Every id that moved gets an
// alias, so claims, tasks and alert notices that quote it still resolve.
void SituationalAwarenessTable::inheritMetadata(std::size_t n_refs) noexcept {
  for (std::size_t e = 0; e < events_.size(); ++e) {
    std::array<std::uint8_t, kMaxEvents> count{};
    for (std::size_t k = 0; k < n_refs; ++k) {
      if (refs_[k].ev == e && cluster_of_[k] != kNoCluster) ++count[cluster_of_[k]];
    }
    std::uint8_t heir = kNoCluster;
    for (std::size_t c = 0; c < scratch_.size(); ++c) {
      if (count[c] == 0) continue;
      if (new_ids_[c] == events_[e].id) {
        heir = static_cast<std::uint8_t>(c);
        break;
      }
      if (heir == kNoCluster || count[c] > count[heir]) heir = static_cast<std::uint8_t>(c);
    }
    heir_[e] = heir;
  }

  for (std::size_t c = 0; c < scratch_.size(); ++c) {
    DetectedEvent& ev = scratch_[c];
    ev.id = new_ids_[c];
    TimeMs first = 0;
    TimeMs last = 0;
    for (const auto& o : ev.observations) {
      if (first == 0 || o.timestamp_ms < first) first = o.timestamp_ms;
      last = std::max(last, o.timestamp_ms);
    }
    ev.first_seen_ms = first;
    ev.last_update_ms = last;
    for (std::size_t e = 0; e < events_.size(); ++e) {
      if (heir_[e] == c) absorbMetadata(ev, events_[e]);
    }
    ev.last_update_ms = last;  // freshness is this cluster's own, not that of readings now elsewhere
    ev.signature = 0;
    fuseEvent(ev, cfg_);  // fusion re-derives the signature and fused fields from the readings
  }

  // Aliases: none may shadow a live id; every id that moved points at its new home.
  for (std::size_t i = aliases_.size(); i-- > 0;) {
    for (std::size_t c = 0; c < scratch_.size(); ++c) {
      if (aliases_[i].from == new_ids_[c]) {
        aliases_.erase_at(i);
        break;
      }
    }
  }
  const auto live = [this](EventId id) {
    for (std::size_t c = 0; c < scratch_.size(); ++c) {
      if (new_ids_[c] == id) return true;
    }
    return false;
  };
  const auto point = [this, &live](EventId from, EventId to) {
    if (from == kNoEvent || to == kNoEvent || from == to || live(from) || resolve(from) == to) return;
    addAlias(from, to);
  };
  for (std::size_t e = 0; e < events_.size(); ++e) {
    if (heir_[e] != kNoCluster) point(events_[e].id, new_ids_[heir_[e]]);
  }
  for (std::size_t i = 0; i < votes_.size();) {  // ids the readings quote, to the cluster that won them
    std::size_t end = i;
    std::size_t win = i;
    while (end < votes_.size() && votes_[end].hint == votes_[i].hint) {
      if (votes_[end].count > votes_[win].count) win = end;
      ++end;
    }
    point(votes_[i].hint, new_ids_[votes_[win].cluster]);
    i = end;
  }
}

void SituationalAwarenessTable::refresh(TimeMs now) noexcept {
  for (std::size_t i = events_.size(); i-- > 0;) {
    DetectedEvent& ev = events_[i];
    for (std::size_t j = ev.observations.size(); j-- > 0;) {
      if (elapsedMs(now, ev.observations[j].timestamp_ms) > cfg_.observation_max_age_ms) {
        ev.observations.erase_at(j);
      }
    }
    if (ev.observations.empty()) events_.erase_at(i);
  }
  if (!recluster()) {  // more sources than the table holds: keep the grouping, merge what is plainly one
    ++recluster_fallbacks_;
    for (auto& ev : events_) fuseEvent(ev, cfg_);
    mergeDuplicates();
  }

  for (auto& ev : events_) {
    validateSlots(ev, now);
    bool learned_benign = false;
    for (const auto& b : benign_) {
      if (b.learned && ev.signature != 0 && b.signature == ev.signature) learned_benign = true;
    }
    if (learned_benign) {
      ev.status = EventStatus::DISMISSED;
      ev.verifier = {};
    } else if (ev.priority_sent) {
      ev.status = EventStatus::CONFIRMED;
    } else if (ev.verifier[0].id != kNoDrone || ev.verifier[1].id != kNoDrone) {
      ev.status = EventStatus::VERIFYING;
    } else {
      ev.status = ev.localized ? EventStatus::LOCALIZED : EventStatus::CANDIDATE;
    }
  }
}

std::size_t SituationalAwarenessTable::purgeObserver(DroneId observer) noexcept {
  std::size_t removed = 0;
  for (auto& ev : events_) {
    for (std::size_t j = ev.observations.size(); j-- > 0;) {
      if (ev.observations[j].observer == observer) {
        ev.observations.erase_at(j);
        ++removed;
      }
    }
    for (auto& slot : ev.verifier) {
      if (slot.id == observer) slot = VerifierSlot{};
    }
    fuseEvent(ev, cfg_);
  }
  return removed;
}

ClaimResult SituationalAwarenessTable::applyClaim(EventId event, DroneId claimant, std::uint8_t role, float score,
                                                  TimeMs now) noexcept {
  DetectedEvent* ev = findEvent(resolve(event));
  if (ev == nullptr || role > 1 || claimant == kNoDrone) return ClaimResult::UNKNOWN_EVENT;

  VerifierSlot& slot = ev->verifier[role];
  if (slot.id == claimant) {
    slot.score = score;
    return ClaimResult::ACCEPTED;
  }
  const bool free_slot = slot.id == kNoDrone;
  // Compare scores quantised to 1e-4 so the order is total: every drone ends
  // with the same holder whatever order the claims arrive in.
  const std::int64_t q_new = quantiseScore(score);
  const std::int64_t q_old = quantiseScore(slot.score);
  const bool better = q_new > q_old || (q_new == q_old && claimant < slot.id);
  if (!free_slot && !better) return ClaimResult::REJECTED;

  VerifierSlot& other = ev->verifier[1 - role];
  if (other.id == claimant) other = VerifierSlot{};  // claimant changes role
  slot = VerifierSlot{claimant, score, now};
  return ClaimResult::ACCEPTED;
}

void SituationalAwarenessTable::markAlertSent(EventId event, AlertLevel level) noexcept {
  DetectedEvent* ev = findEvent(resolve(event));
  if (ev == nullptr) return;
  if (level == AlertLevel::PRIORITY) {
    ev->priority_sent = true;
    ev->advisory_sent = true;
  } else if (level == AlertLevel::ADVISORY) {
    ev->advisory_sent = true;
  }
}

// ------------------------------------------------------ learned benign

bool SituationalAwarenessTable::markBenign(std::uint32_t signature, bool learned) noexcept {
  if (signature == 0) return false;
  for (auto& b : benign_) {
    if (b.signature == signature) {
      b.learned = b.learned || learned;
      return false;
    }
  }
  if (benign_.full()) {
    std::size_t oldest_learned = benign_.size();
    for (std::size_t i = 0; i < benign_.size(); ++i) {
      if (benign_[i].learned) {
        oldest_learned = i;
        break;
      }
    }
    if (oldest_learned == benign_.size()) return false;
    benign_.erase_at(oldest_learned);
  }
  if (!benign_.push_back(BenignSig{signature, learned})) return false;  // cannot fail: made room above
  if (learned) {
    for (auto& ev : events_) {
      if (ev.signature == signature) ev.verifier = {};
    }
  }
  return true;
}

bool SituationalAwarenessTable::isBenign(std::uint32_t signature) const noexcept {
  if (signature == 0) return false;
  for (const auto& b : benign_) {
    if (b.signature == signature) return true;
  }
  return false;
}

std::size_t SituationalAwarenessTable::learnedCount() const noexcept {
  std::size_t n = 0;
  for (const auto& b : benign_) n += b.learned ? 1 : 0;
  return n;
}

std::uint32_t SituationalAwarenessTable::learnedAt(std::size_t i) const noexcept {
  std::size_t n = 0;
  for (const auto& b : benign_) {
    if (!b.learned) continue;
    if (n == i) return b.signature;
    ++n;
  }
  return 0;
}

}  // namespace amc8270::mesh
