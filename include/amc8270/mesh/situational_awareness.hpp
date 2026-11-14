// Shared situational-awareness table: pack membership, fused events,
// verification slots and the common learning of the situation (benign signatures). Every drone
// holds one; anti-entropy re-broadcast makes the copies converge.
#pragma once

#include <array>
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
  std::uint32_t last_session = 0;  // sender's boot session (a new one restarts seq)
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
  [[nodiscard]] bool updatePeer(const DroneState& st, std::uint32_t session, std::uint32_t seq, TimeMs now) noexcept;
  // Liveness + loss accounting for any other packet from a known peer.
  void notePacket(DroneId src, std::uint32_t session, std::uint32_t seq, TimeMs now) noexcept;
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

  // Prune expired readings, re-cluster every reading deterministically (the
  // grouping and the ids depend only on the set of readings, not on the order
  // they arrived in or on which drone holds them), fuse, validate verifier
  // slots, update status.
  void refresh(TimeMs now) noexcept;
  // Times refresh() could not re-cluster (more sources than kMaxEvents) and
  // fell back to merging the existing events.
  std::uint32_t reclusterFallbacks() const noexcept { return recluster_fallbacks_; }

  // Forget everything `observer` reported and any slot it holds (revoked drone).
  // Events left with no readings are dropped at the next refresh().
  std::size_t purgeObserver(DroneId observer) noexcept;

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

  struct ObsRef {
    std::uint8_t ev = 0;  // index into events_
    std::uint8_t ob = 0;  // index into its observations
  };
  struct HintVote {
    EventId hint = kNoEvent;
    std::uint8_t cluster = 0;
    std::uint8_t count = 0;
  };

  [[nodiscard]] bool recluster() noexcept;
  void assignClusterIds() noexcept;
  void inheritMetadata(std::size_t n_refs) noexcept;
  bool consistentWithOwnReading(const SensorObservation& o, const DetectedEvent& ev) const noexcept;
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

  // Scratch for recluster(): the new event list is built here, then replaces events_.
  static constexpr std::size_t kMaxReadings = kMaxEvents * kMaxObsPerEvent;
  static constexpr std::uint8_t kNoCluster = 0xFF;
  StaticVector<DetectedEvent, kMaxEvents> scratch_{};
  std::array<ObsRef, kMaxReadings> refs_{};
  std::array<std::uint8_t, kMaxReadings> cluster_of_{};  // cluster of refs_[k], by assignment
  StaticVector<HintVote, kMaxReadings> votes_{};
  std::array<EventId, kMaxEvents> new_ids_{};
  std::array<std::uint8_t, kMaxEvents> heir_{};  // old event -> cluster that takes its history
  std::uint32_t recluster_fallbacks_ = 0;
};
static_assert(kMaxEvents < 0xFF, "cluster indices are std::uint8_t with 0xFF meaning none");
static_assert(kMaxObsPerEvent <= 0xFF, "vote counts are std::uint8_t");

// Re-fuse one event from its observations (exposed for tests).
void fuseEvent(DetectedEvent& ev, const MeshConfig& cfg) noexcept;

}  // namespace amc8270::mesh
