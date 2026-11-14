// Shared situational-awareness table: pack membership, fused events,
// verification slots and the learned-benign signature list. Every drone
// holds one; anti-entropy re-broadcast makes the copies converge.
#pragma once

#include <cstddef>
#include <cstdint>

#include "amc8270/mesh/config.hpp"
#include "amc8270/mesh/detected_event.hpp"
#include "amc8270/mesh/drone_state.hpp"
#include "amc8270/mesh/static_vector.hpp"

namespace amc8270::mesh {

struct PeerRecord {
  DroneState state{};
  TimeMs last_heard_ms = 0;
  std::uint32_t last_seq = 0;
  std::uint32_t rx_count = 0;
  std::uint32_t missed_count = 0;  // sequence gaps = packets lost on the mesh

  float linkQuality() const noexcept {
    const std::uint32_t total = rx_count + missed_count;
    return total == 0 ? 1.0f : static_cast<float>(rx_count) / static_cast<float>(total);
  }
};

enum class ClaimResult : std::uint8_t { ACCEPTED, REJECTED, UNKNOWN_EVENT };

using DroneList = StaticVector<DroneState, kMaxPackSize>;

class SituationalAwarenessTable {
 public:
  SituationalAwarenessTable(DroneId self, const MeshConfig& cfg) noexcept;
  // ~60 KB of state with internal ids and aliases: never copied by accident.
  SituationalAwarenessTable(const SituationalAwarenessTable&) = delete;
  SituationalAwarenessTable& operator=(const SituationalAwarenessTable&) = delete;
  SituationalAwarenessTable(SituationalAwarenessTable&&) = delete;
  SituationalAwarenessTable& operator=(SituationalAwarenessTable&&) = delete;
  ~SituationalAwarenessTable() = default;

  // ---- pack membership -------------------------------------------------
  void setSelf(const DroneState& st) noexcept;
  const DroneState& self() const noexcept { return self_; }
  // Creates or updates a peer from a HEARTBEAT. True if newly discovered.
  [[nodiscard]] bool updatePeer(const DroneState& st, std::uint32_t seq, TimeMs now) noexcept;
  // Liveness + loss accounting for any other packet from a known peer.
  void notePacket(DroneId src, std::uint32_t seq, TimeMs now) noexcept;
  std::size_t expirePeers(TimeMs now) noexcept;  // returns how many were dropped
  bool removePeer(DroneId id) noexcept;
  const PeerRecord* findPeer(DroneId id) const noexcept;
  std::size_t peerCount() const noexcept { return peers_.size(); }
  const DroneState* findDrone(DroneId id) const noexcept;  // self or peer
  void snapshotDrones(DroneList& out) const noexcept;      // self + peers

  // ---- events ----------------------------------------------------------
  // Fresh event id, never one already in use. Seed the counter once per boot
  // (e.g. from the clock) so ids from before a restart are not handed out again.
  [[nodiscard]] EventId allocateEventId() noexcept;
  void seedEventCounter(std::uint16_t seed) noexcept;
  EventId resolve(EventId id) const noexcept;  // follows merge/rename aliases
  const DetectedEvent* findEvent(EventId id) const noexcept;
  DetectedEvent* findEvent(EventId id) noexcept;
  const StaticVector<DetectedEvent, kMaxEvents>& events() const noexcept { return events_; }
  StaticVector<DetectedEvent, kMaxEvents>& mutableEvents() noexcept { return events_; }

  // Which existing event this observation belongs to (kNoEvent if none).
  [[nodiscard]] EventId associate(const SensorObservation& obs) const noexcept;
  // Stores the observation, creating/merging events as needed. Returns the
  // canonical event id, or kNoEvent if it had to be dropped.
  [[nodiscard]] EventId ingestObservation(const SensorObservation& obs, TimeMs now) noexcept;

  // Prune, re-fuse, merge duplicates, validate verifier slots, update status.
  void refresh(TimeMs now) noexcept;

  [[nodiscard]] ClaimResult applyClaim(EventId event, DroneId claimant, std::uint8_t role, float score,
                         TimeMs now) noexcept;
  void markAlertSent(EventId event, AlertLevel level) noexcept;

  // ---- learned environment ---------------------------------------------
  // `learned` signatures are re-broadcast by anti-entropy; preloaded ones are
  // identical on every drone (mission data) and are not.
  [[nodiscard]] bool markBenign(std::uint32_t signature, bool learned) noexcept;
  bool isBenign(std::uint32_t signature) const noexcept;
  std::size_t learnedCount() const noexcept;
  std::uint32_t learnedAt(std::size_t i) const noexcept;
  std::size_t benignCount() const noexcept { return benign_.size(); }

  const MeshConfig& config() const noexcept { return cfg_; }

 private:
  struct Alias {
    EventId from = kNoEvent;
    EventId to = kNoEvent;
  };
  struct BenignSig {
    std::uint32_t signature = 0;
    bool learned = false;
  };

  void addAlias(EventId from, EventId to) noexcept;
  void rename(EventId from, EventId to) noexcept;
  bool shouldMerge(const DetectedEvent& a, const DetectedEvent& b) const noexcept;
  bool gatePasses(const SensorObservation& o, const DetectedEvent& ev, double& metric) const noexcept;
  bool idInUse(EventId id) const noexcept;
  void mergeDuplicates() noexcept;
  void validateSlots(DetectedEvent& ev, TimeMs now) const noexcept;
  bool evictOne() noexcept;
  bool slotValid(const VerifierSlot& s, const DetectedEvent& ev, TimeMs now) const noexcept;

  DroneId self_id_;
  MeshConfig cfg_;
  DroneState self_{};
  std::uint16_t event_counter_ = 0;
  StaticVector<PeerRecord, kMaxPackSize - 1> peers_{};  // + self = kMaxPackSize = DroneList capacity
  StaticVector<DetectedEvent, kMaxEvents> events_{};
  StaticVector<Alias, kMaxAliases> aliases_{};
  StaticVector<BenignSig, kMaxKnownSignatures> benign_{};
};

// Re-fuse one event from its observations (exposed for tests).
void fuseEvent(DetectedEvent& ev, const MeshConfig& cfg) noexcept;

}  // namespace amc8270::mesh
