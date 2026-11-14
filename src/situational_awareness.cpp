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

// Fold `d` into `k` (k keeps its id).
void absorb(DetectedEvent& k, const DetectedEvent& d) noexcept {
  for (const auto& o : d.observations) insertObservation(k, o);
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

constexpr double kUngatedMetric = 1.0e12;  // ranks signature-only matches after geometric ones

std::int64_t quantiseScore(float s) noexcept { return std::llround(static_cast<double>(s) * 1.0e4); }

constexpr std::uint32_t kSeqRestartWindow = 1000U;  // a jump larger than this is a restart, not loss

bool trackSeq(PeerRecord& p, std::uint32_t seq) noexcept {
  if (seq > p.last_seq) {
    const std::uint32_t gap = seq - p.last_seq - 1U;
    if (gap <= kSeqRestartWindow) p.missed_count += gap;  // huge jumps would wreck the statistic
    p.last_seq = seq;
    ++p.rx_count;
    return true;
  }
  if (p.last_seq - seq > kSeqRestartWindow) {  // sender restarted
    p.last_seq = seq;
    ++p.rx_count;
    return true;
  }
  return false;  // duplicate or reordered
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
  if (n_lines >= 2 && triangulate(lines.data(), n_lines, cfg.min_geometry_eigen, tri)) {
    est[n_est++] = {tri.position, std::max(tri.cep_m, 10.0) / kCepPerSigma};
  }

  const float alt = alt_n > 0.0 ? static_cast<float>(alt_sum / alt_n) : 0.0f;
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
    ev.position = frame.fromEnu(Vec2{se / sw, sn / sw}, alt);
    ev.cep_m = static_cast<float>(kCepPerSigma / std::sqrt(sw));
    ev.localized = true;
  } else if (newest_bearing != nullptr) {
    ev.position = destinationPoint(newest_bearing->observer_position, static_cast<double>(newest_bearing->bearing_deg),
                                   cfg.provisional_range_m);
    ev.cep_m = static_cast<float>(cfg.provisional_cep_m);
    ev.localized = false;
  } else {
    ev.localized = false;
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

bool SituationalAwarenessTable::updatePeer(const DroneState& st, std::uint32_t seq, TimeMs now) noexcept {
  if (st.id == kNoDrone || st.id == self_id_ || !isPlausible(st)) return false;
  if (st.timestamp_ms > now + cfg_.max_clock_skew_ms) return false;
  for (auto& p : peers_) {
    if (p.state.id != st.id) continue;
    p.last_heard_ms = now;
    if (trackSeq(p, seq) || st.timestamp_ms > p.state.timestamp_ms) p.state = st;
    return false;
  }
  if (peers_.full()) return false;
  PeerRecord r;
  r.state = st;
  r.last_heard_ms = now;
  r.last_seq = seq;
  r.rx_count = 1;
  return peers_.push_back(r);
}

void SituationalAwarenessTable::notePacket(DroneId src, std::uint32_t seq, TimeMs now) noexcept {
  for (auto& p : peers_) {
    if (p.state.id != src) continue;
    p.last_heard_ms = now;
    trackSeq(p, seq);
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

void SituationalAwarenessTable::seedEventCounter(std::uint16_t seed) noexcept { event_counter_ = seed; }

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
    if (event_counter_ == 0) event_counter_ = 1;
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

EventId SituationalAwarenessTable::associate(const SensorObservation& o) const noexcept {
  double metric = 0.0;
  if (o.event_hint != kNoEvent) {  // a hint is a preference, not a bypass of the gate
    const DetectedEvent* h = findEvent(resolve(o.event_hint));
    if (h != nullptr && gatePasses(o, *h, metric)) return h->id;
  }
  EventId best = kNoEvent;
  double best_metric = 0.0;
  for (const auto& ev : events_) {
    if (!gatePasses(o, ev, metric)) continue;
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
  // At least one side has no position: only a shared signature can link them.
  if (a.signature == 0 || a.signature != b.signature) return false;
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

void SituationalAwarenessTable::refresh(TimeMs now) noexcept {
  for (std::size_t i = events_.size(); i-- > 0;) {
    DetectedEvent& ev = events_[i];
    for (std::size_t j = ev.observations.size(); j-- > 0;) {
      if (elapsedMs(now, ev.observations[j].timestamp_ms) > cfg_.observation_max_age_ms) {
        ev.observations.erase_at(j);
      }
    }
    if (ev.observations.empty()) {
      events_.erase_at(i);
      continue;
    }
    fuseEvent(ev, cfg_);
  }
  mergeDuplicates();

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
