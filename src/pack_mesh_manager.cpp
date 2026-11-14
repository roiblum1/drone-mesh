#include "amc8270/mesh/pack_mesh_manager.hpp"

#include <algorithm>
#include <array>

#include "amc8270/mesh/geo.hpp"

namespace amc8270::mesh {

namespace {

TimeMs elapsed(TimeMs now, TimeMs then) noexcept { return now > then ? now - then : 0; }

LinkType pickLink(LinkMask m) noexcept {
  if ((m & static_cast<LinkMask>(LinkType::LOS)) != 0) return LinkType::LOS;
  if ((m & static_cast<LinkMask>(LinkType::LEO)) != 0) return LinkType::LEO;
  return LinkType::GEO;
}

}  // namespace

PackMeshManager::PackMeshManager(DroneId self, const MeshConfig& cfg, IClock& clock, IMeshTransceiver& radio,
                                 IGmcsLink& gmcs, IGuidanceSink& guidance) noexcept
    : self_(self), cfg_(cfg), clock_(clock), radio_(radio), gmcs_(gmcs), guidance_(guidance), table_(self, cfg) {
  own_.id = self;
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
  own_ = state;
  own_.id = self_;
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
  broadcast(SignatureUpdatePayload{signature}, clock_.nowMs());
}

void PackMeshManager::addKnownBenignSignature(std::uint32_t signature) noexcept {
  table_.markBenign(signature, false);
}

void PackMeshManager::shutdown() noexcept {
  cancelTask(true);
  broadcast(LeavePayload{}, clock_.nowMs());
}

// ---------------------------------------------------------------- radio

template <typename Payload>
bool PackMeshManager::broadcast(const Payload& payload, TimeMs now) noexcept {
  MeshPacket p;
  p.header.src = self_;
  p.header.seq = ++seq_;
  p.header.time_ms = now;
  p.payload = payload;
  std::array<std::uint8_t, kMaxPacketBytes> buf;
  const std::size_t n = encodePacket(p, buf.data(), buf.size());
  if (n == 0 || !radio_.send(buf.data(), n)) return false;
  ++stats_.tx_packets;
  return true;
}

void PackMeshManager::receiveAll(TimeMs now) noexcept {
  std::array<std::uint8_t, kMaxPacketBytes> buf;
  for (std::size_t i = 0; i < kMaxRxPerTick; ++i) {
    const std::size_t n = radio_.receive(buf.data(), buf.size());
    if (n == 0) break;
    MeshPacket pkt;
    if (!decodePacket(buf.data(), n, pkt)) {
      ++stats_.rx_rejected;
      continue;
    }
    ++stats_.rx_packets;
    handlePacket(pkt, now);
  }
}

void PackMeshManager::handlePacket(const MeshPacket& pkt, TimeMs now) noexcept {
  if (pkt.header.src == self_) return;  // multicast loopback

  if (const auto* hb = std::get_if<HeartbeatPayload>(&pkt.payload)) {
    if (table_.updatePeer(hb->state, pkt.header.seq, now)) ++stats_.peers_discovered;
    return;
  }
  table_.notePacket(pkt.header.src, pkt.header.seq, now);

  if (const auto* ob = std::get_if<ObservationPayload>(&pkt.payload)) {
    table_.ingestObservation(ob->obs, now);
  } else if (const auto* c = std::get_if<TaskClaimPayload>(&pkt.payload)) {
    table_.applyClaim(c->event_id, c->claimant, c->role, c->score, now);
  } else if (const auto* a = std::get_if<AlertIssuedPayload>(&pkt.payload)) {
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
  broadcast(HeartbeatPayload{exportedState()}, now);
}

// Re-send our own readings and learned signatures so a lost packet only
// delays convergence instead of leaving drones with different pictures.
void PackMeshManager::sendAntiEntropy(TimeMs now) noexcept {
  last_anti_entropy_ms_ = now;
  std::uint32_t sent = 0;
  for (const auto& ev : table_.events()) {
    for (const auto& o : ev.observations) {
      if (o.observer != self_ || sent >= cfg_.max_antientropy_per_cycle) continue;
      SensorObservation copy = o;
      copy.event_hint = ev.id;
      if (broadcast(ObservationPayload{copy}, now)) {
        ++sent;
        ++stats_.observations_sent;
      }
    }
  }
  for (std::size_t i = 0; i < table_.learnedCount(); ++i) {
    broadcast(SignatureUpdatePayload{table_.learnedAt(i)}, now);
  }
}

// ----------------------------------------------------------------- tick

void PackMeshManager::tick() noexcept {
  const TimeMs now = clock_.nowMs();
  receiveAll(now);
  stats_.peers_lost += static_cast<std::uint32_t>(table_.expirePeers(now));

  if (heartbeat_requested_ || elapsed(now, last_heartbeat_ms_) >= cfg_.heartbeat_period_ms) sendHeartbeat(now);
  if (elapsed(now, last_anti_entropy_ms_) >= cfg_.anti_entropy_period_ms) sendAntiEntropy(now);
  if (elapsed(now, last_eval_ms_) >= cfg_.coordination_period_ms) evaluateAndCoordinateAlerts(now);
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

  bool issue = !task_active_;
  if (task_active_) {
    if (table_.resolve(task_.event_id) != mine->event_id || task_.role != mine->role) {
      issue = true;
    } else {
      task_.event_id = mine->event_id;  // adopt a renamed id
      const double threshold =
          std::max(cfg_.retask_move_m, cfg_.retask_cep_factor * static_cast<double>(task_.target_cep_m));
      if (distanceM(task_.target, t.target) > threshold) issue = true;
    }
  }

  if (issue) {
    guidance_.requestVerification(t);
    task_ = t;
    task_active_ = true;
    ++stats_.tasks_issued;
    heartbeat_requested_ = true;
    table_.setSelf(exportedState());
  } else if (elapsed(now, last_claim_ms_) < 2ULL * static_cast<TimeMs>(cfg_.anti_entropy_period_ms)) {
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
    const AlertLevel level = evaluateAlertLevel(ev, table_.isBenign(ev.signature), cfg_);
    if (level != AlertLevel::PRIORITY) ev.priority_ready_ms = 0;
    if (level == AlertLevel::NONE) {
      ev.advisory_ready_ms = 0;
      continue;
    }
    const bool already = level == AlertLevel::PRIORITY ? ev.priority_sent : (ev.advisory_sent || ev.priority_sent);
    if (already) continue;

    TimeMs& ready = level == AlertLevel::PRIORITY ? ev.priority_ready_ms : ev.advisory_ready_ms;
    if (ready == 0) ready = now;

    const int rank = alertOwnerRank(table_, ev, self_);
    if (rank < 0) continue;
    if (elapsed(now, ready) < static_cast<TimeMs>(rank) * cfg_.alert_backup_delay_ms) continue;

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
    alert.timestamp_ms = now;

    if (!gmcs_.sendAlert(alert, pickLink(own_.available_links))) {
      ++stats_.alert_send_failures;  // retry next cycle
      continue;
    }
    table_.markAlertSent(ev.id, level);
    broadcast(AlertIssuedPayload{ev.id, level, self_}, now);
    if (level == AlertLevel::PRIORITY) {
      ++stats_.priority_alerts_sent;
    } else {
      ++stats_.advisories_sent;
    }
  }
}

}  // namespace amc8270::mesh
