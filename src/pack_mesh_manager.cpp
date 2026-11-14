#include "amc8270/mesh/pack_mesh_manager.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "amc8270/mesh/geo.hpp"
#include "amc8270/mesh/validation.hpp"

namespace amc8270::mesh {

namespace {

LinkType pickLink(LinkMask m) noexcept {
  if ((m & static_cast<LinkMask>(LinkType::LOS)) != 0) return LinkType::LOS;
  if ((m & static_cast<LinkMask>(LinkType::LEO)) != 0) return LinkType::LEO;
  return LinkType::GEO;
}

}  // namespace

PackMeshManager::PackMeshManager(DroneId self, const MeshConfig& cfg, std::uint32_t boot_session, IClock& clock,
                                 IMeshTransceiver& radio, IGmcsLink& gmcs, IGuidanceSink& guidance) noexcept
    : self_(self),
      cfg_(cfg),
      clock_(clock),
      radio_(radio),
      gmcs_(gmcs),
      guidance_(guidance),
      security_(self, boot_session, cfg),
      table_(self, cfg) {
  own_.id = self;
  // Seed event ids from boot time so ids handed out before a restart are not reused.
  table_.seedEventCounter(static_cast<std::uint16_t>((clock_.nowMs() / 1000U) & 0xFFFFU));
  table_.setSelf(exportedState());
}

// ------------------------------------------------------------- own state

DroneState PackMeshManager::exportedState() const noexcept {
  DroneState s = own_;
  s.id = self_;
  s.timestamp_ms = clock_.nowMs();
  if (task_active_) {
    s.mode = DroneMode::VERIFYING;
    s.tasked_event = task_.event_id;
    s.active_sensor = task_.sensor_to_cue;
  } else {
    s.tasked_event = kNoEvent;
  }
  return s;
}

void PackMeshManager::updateOwnState(const DroneState& state) noexcept {
  DroneState candidate = state;
  candidate.id = self_;
  if (!isPlausible(candidate)) {  // a NaN here would make every peer reject our heartbeats
    ++stats_.local_rejected;
    return;
  }
  own_ = candidate;
  if (own_.mode == DroneMode::VERIFYING) own_.mode = DroneMode::PACK_ROUTING;
  if (task_active_ && own_.mode != DroneMode::PACK_ROUTING) {
    cancelTask(true);  // GMCS took the drone (or it is recovering): drop the mesh task
  }
  table_.setSelf(exportedState());
}

bool PackMeshManager::reportLocalDetection(SensorObservation obs) noexcept {
  const TimeMs now = clock_.nowMs();
  obs.observer = self_;
  obs.observer_position = own_.position;
  if (obs.timestamp_ms == 0) obs.timestamp_ms = now;
  if (!isPlausible(obs)) {
    ++stats_.local_rejected;
    return false;
  }
  if (obs.event_hint == kNoEvent) {
    obs.event_hint = table_.associate(obs);
    if (obs.event_hint == kNoEvent) obs.event_hint = table_.allocateEventId();
  }
  const EventId id = table_.ingestObservation(obs, now);
  if (id == kNoEvent) return false;
  obs.event_hint = id;
  if (broadcast(ObservationPayload{obs}, now)) ++stats_.observations_sent;
  return true;
}

void PackMeshManager::applyGmcsDismissal(std::uint32_t signature) noexcept {
  if (signature == 0) return;
  if (table_.markBenign(signature, true)) ++stats_.signatures_learned;
  (void)broadcast(SignatureUpdatePayload{signature}, clock_.nowMs());
}

void PackMeshManager::addKnownBenignSignature(std::uint32_t signature) noexcept {
  (void)table_.markBenign(signature, false);  // false just means it was already known
}

bool PackMeshManager::installEpochKey(std::uint16_t epoch, const crypto::Key& key, TimeMs valid_from_ms,
                                      TimeMs valid_until_ms) noexcept {
  return security_.installEpochKey(epoch, key, valid_from_ms, valid_until_ms);
}

bool PackMeshManager::eraseEpochKey(std::uint16_t epoch) noexcept { return security_.eraseEpochKey(epoch); }

void PackMeshManager::zeroizeKeys() noexcept { security_.zeroizeAll(); }

bool PackMeshManager::revokeDrone(DroneId id) noexcept {
  if (!security_.revokeDrone(id)) return false;
  if (table_.removePeer(id)) ++stats_.peers_lost;
  (void)table_.purgeObserver(id);
  return true;
}

void PackMeshManager::shutdown() noexcept {
  cancelTask(true);
  (void)broadcast(LeavePayload{}, clock_.nowMs());
}

// ---------------------------------------------------------------- radio

template <typename Payload>
bool PackMeshManager::broadcast(const Payload& payload, TimeMs now) noexcept {
  MeshPacket p;
  p.payload = payload;
  std::array<std::uint8_t, kMaxPacketBytes> buf;
  const std::size_t n = security_.seal(p, now, buf.data(), buf.size());
  if (n == 0) {
    ++stats_.tx_no_key;  // the only way seal() fails for a well-formed payload
    return false;
  }
  if (!radio_.send(buf.data(), n)) return false;
  ++stats_.tx_packets;
  return true;
}

void PackMeshManager::receiveAll(TimeMs now) noexcept {
  std::array<std::uint8_t, kMaxPacketBytes> buf;
  for (std::size_t i = 0; i < kMaxRxPerTick; ++i) {
    const std::size_t n = radio_.receive(buf.data(), buf.size());
    if (n == 0) break;
    MeshPacket pkt;
    switch (security_.open(buf.data(), n, now, pkt)) {
      case OpenStatus::OK:
        ++stats_.rx_packets;
        handlePacket(pkt, now);
        break;
      case OpenStatus::LOOPBACK:
        break;
      case OpenStatus::MALFORMED:
        ++stats_.rx_rejected;
        break;
      case OpenStatus::NO_KEY:
        ++stats_.rx_no_key;
        break;
      case OpenStatus::AUTH_FAILED:
        ++stats_.rx_auth_failed;
        break;
      case OpenStatus::REVOKED:
        ++stats_.rx_revoked;
        break;
      case OpenStatus::STALE:
        ++stats_.rx_stale;
        break;
      case OpenStatus::REPLAYED:
        ++stats_.rx_replayed;
        break;
    }
  }
}

void PackMeshManager::handlePacket(const MeshPacket& pkt, TimeMs now) noexcept {
  if (pkt.header.src == self_) return;  // multicast loopback (already filtered by the security layer)

  if (const auto* hb = std::get_if<HeartbeatPayload>(&pkt.payload)) {
    if (table_.updatePeer(hb->state, pkt.header.session, pkt.header.seq, now)) ++stats_.peers_discovered;
    return;
  }
  table_.notePacket(pkt.header.src, pkt.header.session, pkt.header.seq, now);

  // A drone only speaks for itself: it relays nothing, so the named observer,
  // claimant or issuer must be the sender. The sender itself is authenticated:
  // the frame verified under that drone's own key for the epoch.
  if (const auto* ob = std::get_if<ObservationPayload>(&pkt.payload)) {
    if (ob->obs.observer != pkt.header.src) {
      ++stats_.rx_rejected;
      return;
    }
    (void)table_.ingestObservation(ob->obs, now);
  } else if (const auto* c = std::get_if<TaskClaimPayload>(&pkt.payload)) {
    if (c->claimant != pkt.header.src) {
      ++stats_.rx_rejected;
      return;
    }
    (void)table_.applyClaim(c->event_id, c->claimant, c->role, c->score, now);
  } else if (const auto* a = std::get_if<AlertIssuedPayload>(&pkt.payload)) {
    if (a->issuer != pkt.header.src) {
      ++stats_.rx_rejected;
      return;
    }
    table_.markAlertSent(a->event_id, a->level);
  } else if (const auto* s = std::get_if<SignatureUpdatePayload>(&pkt.payload)) {
    if (table_.markBenign(s->signature, true)) ++stats_.signatures_learned;
  } else if (std::holds_alternative<LeavePayload>(pkt.payload)) {
    if (table_.removePeer(pkt.header.src)) ++stats_.peers_lost;
  }
}

void PackMeshManager::sendHeartbeat(TimeMs now) noexcept {
  last_heartbeat_ms_ = now;
  heartbeat_requested_ = false;
  table_.setSelf(exportedState());
  (void)broadcast(HeartbeatPayload{exportedState()}, now);
}

// Re-send our own readings and learned signatures so a lost packet only
// delays convergence instead of leaving drones with different pictures.
void PackMeshManager::sendAntiEntropy(TimeMs now) noexcept {
  last_anti_entropy_ms_ = now;

  // Own readings, at most max_antientropy_per_cycle per cycle. The start point
  // rotates so every reading is re-sent eventually, however many there are.
  std::uint32_t total = 0;
  for (const auto& ev : table_.events()) {
    for (const auto& o : ev.observations) total += o.observer == self_ ? 1U : 0U;
  }
  if (total > 0) {
    const std::uint32_t start = ae_cursor_ % total;
    std::uint32_t sent = 0;
    for (std::uint32_t pass = 0; pass < 2; ++pass) {  // [start, total) then [0, start)
      std::uint32_t idx = 0;
      for (const auto& ev : table_.events()) {
        for (const auto& o : ev.observations) {
          if (o.observer != self_) continue;
          const bool in_pass = pass == 0 ? idx >= start : idx < start;
          ++idx;
          if (!in_pass || sent >= cfg_.max_antientropy_per_cycle) continue;
          SensorObservation copy = o;
          copy.event_hint = ev.id;
          if (broadcast(ObservationPayload{copy}, now)) ++stats_.observations_sent;
          ++sent;  // count attempts so a failing radio can't stall the rotation
        }
      }
    }
    ae_cursor_ = (start + sent) % total;
  }

  for (std::size_t i = 0; i < table_.learnedCount(); ++i) {
    (void)broadcast(SignatureUpdatePayload{table_.learnedAt(i)}, now);
  }

  for (std::size_t i = issued_.size(); i-- > 0;) {
    if (elapsedMs(now, issued_[i].at_ms) > cfg_.alert_notice_repeat_ms) {
      issued_.erase_at(i);
      continue;
    }
    (void)broadcast(AlertIssuedPayload{table_.resolve(issued_[i].event), issued_[i].level, self_}, now);
  }
}

// ----------------------------------------------------------------- tick

void PackMeshManager::tick() noexcept {
  const TimeMs now = clock_.nowMs();
  receiveAll(now);
  stats_.peers_lost += static_cast<std::uint32_t>(table_.expirePeers(now));

  if (heartbeat_requested_ || isDue(now, last_heartbeat_ms_, cfg_.heartbeat_period_ms)) sendHeartbeat(now);
  if (isDue(now, last_anti_entropy_ms_, cfg_.anti_entropy_period_ms)) sendAntiEntropy(now);
  if (isDue(now, last_eval_ms_, cfg_.coordination_period_ms)) evaluateAndCoordinateAlerts(now);
}

// ---------------------------------------------- distributed coordination

void PackMeshManager::evaluateAndCoordinateAlerts(TimeMs now) noexcept {
  last_eval_ms_ = now;
  ++stats_.evaluations;

  // 1. Refresh the shared picture: prune, fuse, merge duplicates, validate slots.
  table_.setSelf(exportedState());
  table_.refresh(now);
  updateUrgencies(table_, now);

  // 2. Every drone with the same table computes the same plan.
  DroneList drones;
  table_.snapshotDrones(drones);
  AssignmentList plan;
  planAssignments(table_, drones, now, cfg_, plan);
  applyAssignments(table_, plan, now);

  // 3. Act on this drone's part of the plan.
  const Assignment* mine = nullptr;
  for (const auto& a : plan) {
    if (a.drone == self_) mine = &a;
  }
  syncOwnTask(mine, now);

  // 4. Early-warning and verified alerts to the GMCS (one owner per event).
  dispatchAlerts(now);
}

void PackMeshManager::cancelTask(bool notify_guidance) noexcept {
  if (!task_active_) return;
  if (notify_guidance) guidance_.cancelVerification();
  task_active_ = false;
  task_ = VerificationTask{};
  ++stats_.tasks_cancelled;
  heartbeat_requested_ = true;
  table_.setSelf(exportedState());
}

void PackMeshManager::syncOwnTask(const Assignment* mine, TimeMs now) noexcept {
  if (mine == nullptr) {
    cancelTask(true);
    pending_event_ = kNoEvent;
    return;
  }

  VerificationTask t;
  t.event_id = mine->event_id;
  t.role = mine->role;
  t.target = mine->target;
  t.target_cep_m = mine->target_cep_m;
  t.standoff = mine->standoff;
  t.approach_bearing_deg = mine->approach_bearing_deg;
  t.sensor_to_cue = mine->sensor_to_cue;
  t.urgency = mine->urgency;

  // A new assignment (or a different event or role) is claimed at once but flown
  // only once it has held for claim_confirm_ms.
  bool claim_now = false;
  const auto confirmNew = [&]() {
    const bool same = pending_event_ != kNoEvent && table_.resolve(pending_event_) == mine->event_id &&
                      pending_role_ == mine->role;
    if (!same) {
      pending_role_ = mine->role;
      pending_since_ms_ = now;
      claim_now = true;
    }
    pending_event_ = mine->event_id;  // adopt a renamed id
    return isDue(now, pending_since_ms_, cfg_.claim_confirm_ms);
  };

  bool issue = false;
  if (!task_active_) {
    issue = confirmNew();
  } else {
    if (table_.resolve(task_.event_id) != mine->event_id || task_.role != mine->role) {
      issue = confirmNew();
    } else {
      pending_event_ = kNoEvent;
      task_.event_id = mine->event_id;  // adopt a renamed id
      const double cep_then = static_cast<double>(task_.target_cep_m);
      const double cep_now = static_cast<double>(t.target_cep_m);
      const double moved = distanceM(task_.target, t.target);
      const bool beyond_noise = moved > std::max(cfg_.retask_move_m, cfg_.retask_cep_factor * std::hypot(cep_then, cep_now));
      const bool refined = cep_now <= 0.5 * cep_then && moved > std::max(cfg_.retask_move_m, cfg_.retask_cep_factor * cep_now);
      // A momentarily lost fix (a provisional point down one bearing) never re-points a verifier
      // already flying to a real position estimate.
      const bool lost_fix = cep_now >= cfg_.provisional_cep_m && cep_then < cfg_.provisional_cep_m;
      if ((beyond_noise || refined) && !lost_fix) {
        if (retask_pending_since_ms_ == 0) retask_pending_since_ms_ = now;
        issue = isDue(now, retask_pending_since_ms_, cfg_.retask_confirm_ms);  // only once the move persists
      } else {
        retask_pending_since_ms_ = 0;
      }
    }
  }

  if (issue) {
    retask_pending_since_ms_ = 0;
    pending_event_ = kNoEvent;
    guidance_.requestVerification(t);
    task_ = t;
    task_active_ = true;
    ++stats_.tasks_issued;
    heartbeat_requested_ = true;
    table_.setSelf(exportedState());
  } else if (!claim_now && !isDue(now, last_claim_ms_, 2ULL * static_cast<TimeMs>(cfg_.anti_entropy_period_ms))) {
    return;
  }

  if (broadcast(TaskClaimPayload{mine->event_id, self_, mine->role, mine->score}, now)) {
    ++stats_.claims_sent;
    last_claim_ms_ = now;
  }
}

void PackMeshManager::dispatchAlerts(TimeMs now) noexcept {
  std::array<std::uint8_t, kMaxEvents> order{};
  const std::size_t n = rankEvents(table_, order.data(), order.size());
  auto& events = table_.mutableEvents();

  for (std::size_t k = 0; k < n; ++k) {
    DetectedEvent& ev = events[order[k]];
    const AlertLevel level = evaluateAlertLevel(ev, table_.isBenign(ev.signature), cfg_, &table_);
    if (level != AlertLevel::PRIORITY) ev.priority_ready_ms = 0;
    if (level == AlertLevel::NONE) {
      ev.advisory_ready_ms = 0;
      continue;
    }
    const bool already = level == AlertLevel::PRIORITY ? ev.priority_sent : (ev.advisory_sent || ev.priority_sent);
    if (already) continue;

    TimeMs& ready = level == AlertLevel::PRIORITY ? ev.priority_ready_ms : ev.advisory_ready_ms;
    if (ready == 0) ready = now;

    // A position from bearings alone must hold before it warns anyone; the
    // backup ranks then count from the end of that hold, as they would otherwise.
    TimeMs hold = 0;
    if (level == AlertLevel::ADVISORY) {
      bool fix = false;
      for (const auto& o : ev.observations) fix = fix || o.has_fix;
      if (!fix) hold = cfg_.advisory_bearing_hold_ms;
    }
    const int rank = alertOwnerRank(table_, ev, self_);
    if (rank < 0) continue;
    if (elapsedMs(now, ready) < hold + static_cast<TimeMs>(rank) * cfg_.alert_backup_delay_ms) continue;

    GmcsAlert alert;
    alert.event_id = ev.id;
    alert.level = level;
    alert.cls = ev.cls;
    alert.position = ev.position;
    alert.cep_m = ev.cep_m;
    alert.confidence = ev.confidence;
    alert.modality_mask = ev.modality_mask;
    alert.observer_count = ev.observer_count;
    alert.issuer = self_;
    alert.primary_verifier = ev.verifier[0].id;
    alert.signature = ev.signature;
    alert.first_seen_ms = ev.first_seen_ms;
    alert.timestamp_ms = now;

    if (!gmcs_.sendAlert(alert, pickLink(own_.available_links))) {
      ++stats_.alert_send_failures;  // retry next cycle
      continue;
    }
    table_.markAlertSent(ev.id, level);
    (void)broadcast(AlertIssuedPayload{ev.id, level, self_}, now);
    if (issued_.full()) issued_.erase_at(0);
    (void)issued_.push_back(IssuedAlert{ev.id, level, now});  // repeated by anti-entropy (cannot fail: made room)
    if (level == AlertLevel::PRIORITY) {
      ++stats_.priority_alerts_sent;
    } else {
      ++stats_.advisories_sent;
    }
  }
}

}  // namespace amc8270::mesh
