#include "amc8270/gmcs/alert_consumer.hpp"

#include <algorithm>
#include <cmath>

#include "amc8270/mesh/geo.hpp"

namespace amc8270::gmcs {

namespace {

std::size_t linkIndex(LinkType l) noexcept { return l == LinkType::LOS ? 0U : (l == LinkType::LEO ? 1U : 2U); }

bool signaturesCompatible(std::uint32_t a, std::uint32_t b) noexcept { return a == 0 || b == 0 || a == b; }

TimeMs deadlineFor(AlertLevel level, const ConsumerConfig& cfg) noexcept {
  return level == AlertLevel::PRIORITY ? cfg.ack_deadline_priority_ms : cfg.ack_deadline_advisory_ms;
}

void takeEstimate(Record& r, const GmcsAlert& a) noexcept {
  r.position = a.position;
  r.cep_m = a.cep_m;
  r.confidence = a.confidence;
  r.modality_mask = static_cast<mesh::SensorMask>(r.modality_mask | a.modality_mask);
  if (mesh::eventSeverity(a.cls) > mesh::eventSeverity(r.cls)) r.cls = a.cls;
  if (r.signature == 0) r.signature = a.signature;
}

}  // namespace

// ------------------------------------------------------------- LatencyStats

void LatencyStats::add(double ms) noexcept {
  if (!std::isfinite(ms)) return;
  ms = std::max(0.0, ms);
  std::size_t b = 0;
  while (b < kEdges.size() && ms > kEdges[b]) ++b;
  ++bins_[b];
  if (count_ == 0 || ms < min_) min_ = ms;
  if (count_ == 0 || ms > max_) max_ = ms;
  sum_ += ms;
  ++count_;
}

double LatencyStats::percentile(double p) const noexcept {
  if (count_ == 0) return 0.0;
  if (p <= 0.0) return min_;
  if (p >= 100.0) return max_;
  const double target = p / 100.0 * static_cast<double>(count_);
  double cum = 0.0;
  for (std::size_t b = 0; b < kBins; ++b) {
    const double next = cum + bins_[b];
    if (next >= target && bins_[b] > 0) {
      const double lo = std::max(min_, b == 0 ? 0.0 : kEdges[b - 1]);
      const double hi = std::min(max_, b < kEdges.size() ? kEdges[b] : max_);
      const double frac = (target - cum) / static_cast<double>(bins_[b]);
      return lo + (hi - lo) * frac;
    }
    cum = next;
  }
  return max_;
}

// ------------------------------------------------------------ AlertConsumer

AlertConsumer::AlertConsumer(const ConsumerConfig& cfg, IUplink& uplink) noexcept : cfg_(cfg), uplink_(uplink) {}

Record* AlertConsumer::findMutable(std::uint32_t record_id) noexcept {
  for (auto& r : records_) {
    if (r.record_id == record_id) return &r;
  }
  return nullptr;
}

const Record* AlertConsumer::find(std::uint32_t record_id) const noexcept {
  for (const auto& r : records_) {
    if (r.record_id == record_id) return &r;
  }
  return nullptr;
}

const Record* AlertConsumer::findByEvent(EventId id) const noexcept {
  for (const auto& r : records_) {
    for (const EventId e : r.event_ids) {
      if (e == id) return &r;
    }
  }
  return nullptr;
}

std::size_t AlertConsumer::openCount() const noexcept {
  std::size_t n = 0;
  for (const auto& r : records_) n += r.state == RecordState::NEW ? 1U : 0U;
  return n;
}

bool AlertConsumer::uplinkPending() const noexcept {
  for (const auto& s : suppressions_) {
    if (s.signature != 0 && !s.uplink_done) return true;
  }
  return false;
}

// The nearest record that is plainly the same source: compatible frequency,
// close enough for both estimates' errors, and heard of recently.
Record* AlertConsumer::matchByPlace(const GmcsAlert& a, TimeMs rx_ms) noexcept {
  Record* best = nullptr;
  double best_d = 0.0;
  for (auto& r : records_) {
    if (!signaturesCompatible(r.signature, a.signature)) continue;
    if (mesh::elapsedMs(rx_ms, r.last_rx_ms) > cfg_.merge_window_ms) continue;
    const double d = mesh::distanceM(r.position, a.position);
    const double gate = std::max(cfg_.merge_gate_m, cfg_.merge_cep_factor * std::hypot(static_cast<double>(r.cep_m),
                                                                                         static_cast<double>(a.cep_m)));
    if (d > gate) continue;
    if (best == nullptr || d < best_d) {
      best = &r;
      best_d = d;
    }
  }
  return best;
}

const AlertConsumer::Suppression* AlertConsumer::suppressionFor(const GmcsAlert& a) const noexcept {
  for (const auto& s : suppressions_) {
    if (s.signature != 0) {
      if (a.signature == s.signature) return &s;
      continue;
    }
    const double gate = std::max(cfg_.merge_gate_m, cfg_.merge_cep_factor * std::hypot(static_cast<double>(s.cep_m),
                                                                                        static_cast<double>(a.cep_m)));
    if (mesh::distanceM(s.position, a.position) <= gate) return &s;
  }
  return nullptr;
}

bool AlertConsumer::makeRoom() noexcept {
  // Oldest closed record first, then the oldest dismissed one (its suppression
  // is kept separately), then the oldest acknowledged ADVISORY.
  for (const RecordState st : {RecordState::CLOSED, RecordState::DISMISSED, RecordState::ACKNOWLEDGED}) {
    std::size_t victim = records_.size();
    for (std::size_t i = 0; i < records_.size(); ++i) {
      const Record& r = records_[i];
      if (r.state != st || (st == RecordState::ACKNOWLEDGED && r.level == AlertLevel::PRIORITY)) continue;
      if (victim == records_.size() || r.last_rx_ms < records_[victim].last_rx_ms) victim = i;
    }
    if (victim < records_.size()) {
      records_.erase_at(victim);
      return true;
    }
  }
  return false;
}

IngestResult AlertConsumer::onAlert(const GmcsAlert& a, LinkType link, TimeMs rx_ms) noexcept {
  ++stats_.alerts;
  stats_.transport[linkIndex(link)].add(static_cast<double>(mesh::elapsedMs(rx_ms, a.timestamp_ms)));
  const auto lvl = static_cast<std::size_t>(a.level);
  if (a.first_seen_ms != 0 && lvl < stats_.detection_to_gmcs.size()) {
    stats_.detection_to_gmcs[lvl].add(static_cast<double>(mesh::elapsedMs(rx_ms, a.first_seen_ms)));
  }

  if (const Suppression* s = suppressionFor(a)) {
    ++stats_.suppressed;
    if (Record* r = findMutable(s->record_id)) {
      ++r->reports;
      r->last_rx_ms = rx_ms;
    }
    // The pack is still alerting a dismissed frequency: it may not have learned
    // it (an uplink can be lost after the C2 link accepted it). Send it again.
    for (auto& own : suppressions_) {
      if (&own == s && own.signature != 0 && own.uplink_done && mesh::isDue(rx_ms, own.last_attempt_ms, cfg_.uplink_retry_ms)) {
        own.uplink_done = false;
      }
    }
    return IngestResult::SUPPRESSED;
  }

  Record* r = nullptr;
  for (auto& rec : records_) {
    for (const EventId e : rec.event_ids) {
      if (e == a.event_id) r = &rec;
    }
  }
  if (r == nullptr) {
    r = matchByPlace(a, rx_ms);
    if (r != nullptr) ++stats_.merged_by_place;
  }

  if (r != nullptr) {
    ++r->reports;
    r->last_rx_ms = rx_ms;
    r->last_issuer = a.issuer;
    if (a.first_seen_ms != 0 && (r->first_seen_ms == 0 || a.first_seen_ms < r->first_seen_ms)) r->first_seen_ms = a.first_seen_ms;
    bool known = false;
    for (const EventId e : r->event_ids) known = known || e == a.event_id;
    if (!known) {
      if (r->event_ids.full()) r->event_ids.erase_at(0);
      (void)r->event_ids.push_back(a.event_id);  // cannot fail: made room above
    }
    if (a.level > r->level) {
      r->level = a.level;
      r->state = RecordState::NEW;  // a PRIORITY needs the operator again
      r->level_rx_ms = rx_ms;
      r->acked_ms = 0;
      r->escalated = false;
      takeEstimate(*r, a);
      ++stats_.upgrades;
      return IngestResult::UPGRADED;
    }
    if (a.level == r->level && a.cep_m < 0.8f * r->cep_m) {
      takeEstimate(*r, a);
      ++stats_.updates;
      return IngestResult::UPDATED;
    }
    ++r->duplicates;
    ++stats_.duplicates;
    return IngestResult::DUPLICATE;
  }

  if (records_.full() && !makeRoom()) {
    ++stats_.rejected;
    return IngestResult::REJECTED;
  }
  Record n;
  n.record_id = next_id_++;
  n.state = RecordState::NEW;
  n.level = a.level;
  n.cls = a.cls;
  n.signature = a.signature;
  takeEstimate(n, a);
  (void)n.event_ids.push_back(a.event_id);
  n.last_issuer = a.issuer;
  n.first_seen_ms = a.first_seen_ms;
  n.first_rx_ms = n.level_rx_ms = n.last_rx_ms = rx_ms;
  n.reports = 1;
  (void)records_.push_back(n);  // cannot fail: made room above
  ++stats_.records;
  return IngestResult::NEW_RECORD;
}

bool AlertConsumer::acknowledge(std::uint32_t record_id, TimeMs now) noexcept {
  Record* r = findMutable(record_id);
  if (r == nullptr || r->state == RecordState::DISMISSED || r->state == RecordState::CLOSED) return false;
  if (r->state == RecordState::NEW) {
    r->state = RecordState::ACKNOWLEDGED;
    r->acked_ms = now;
    stats_.operator_ack.add(static_cast<double>(mesh::elapsedMs(now, r->level_rx_ms)));
  }
  return true;
}

bool AlertConsumer::addSuppression(const Suppression& s) noexcept {
  for (auto& e : suppressions_) {
    if (s.signature != 0 && e.signature == s.signature) return true;  // already dismissed
  }
  if (suppressions_.full()) {
    // Forget the oldest one the pack has already been told about.
    for (std::size_t i = 0; i < suppressions_.size(); ++i) {
      if (suppressions_[i].signature == 0 || suppressions_[i].uplink_done) {
        suppressions_.erase_at(i);
        break;
      }
    }
  }
  return suppressions_.push_back(s);
}

void AlertConsumer::trySend(Suppression& s, TimeMs now) noexcept {
  s.last_attempt_ms = now;
  if (uplink_.sendDismissal(s.signature)) {
    s.uplink_done = true;
    ++stats_.uplinks_sent;
  } else {
    ++stats_.uplink_failures;
  }
}

bool AlertConsumer::dismiss(std::uint32_t record_id, TimeMs now) noexcept {
  Record* r = findMutable(record_id);
  if (r == nullptr) return false;
  r->state = RecordState::DISMISSED;
  Suppression s;
  s.signature = r->signature;
  s.position = r->position;
  s.cep_m = r->cep_m;
  s.record_id = record_id;
  if (!addSuppression(s)) return false;
  if (s.signature != 0) {
    for (auto& e : suppressions_) {
      if (e.signature == s.signature && !e.uplink_done) trySend(e, now);
    }
  }
  return true;
}

bool AlertConsumer::dismissSignature(std::uint32_t signature, TimeMs now) noexcept {
  if (signature == 0) return false;
  Suppression s;
  s.signature = signature;
  if (!addSuppression(s)) return false;
  for (auto& r : records_) {
    if (r.signature == signature && r.state != RecordState::CLOSED) r.state = RecordState::DISMISSED;
  }
  for (auto& e : suppressions_) {
    if (e.signature == signature && !e.uplink_done) trySend(e, now);
  }
  return true;
}

bool AlertConsumer::close(std::uint32_t record_id) noexcept {
  Record* r = findMutable(record_id);
  if (r == nullptr) return false;
  r->state = RecordState::CLOSED;
  return true;
}

void AlertConsumer::tick(TimeMs now) noexcept {
  for (auto& s : suppressions_) {
    if (s.signature != 0 && !s.uplink_done && mesh::isDue(now, s.last_attempt_ms, cfg_.uplink_retry_ms)) trySend(s, now);
  }
  for (auto& r : records_) {
    if (r.state != RecordState::NEW || r.escalated) continue;
    if (mesh::elapsedMs(now, r.level_rx_ms) > deadlineFor(r.level, cfg_)) {
      r.escalated = true;
      ++stats_.escalations;
    }
  }
}

}  // namespace amc8270::gmcs
