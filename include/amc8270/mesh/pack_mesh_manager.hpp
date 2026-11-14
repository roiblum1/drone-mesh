// Per-drone mesh agent for the Airborne Mission Computer Model 8270.
//
// Call tick() at a fixed rate (10 Hz recommended). All storage is fixed at
// construction; tick() performs no heap allocation, throws nothing and
// every loop is bounded by a compile-time capacity.
#pragma once

#include <cstdint>

#include "amc8270/mesh/config.hpp"
#include "amc8270/mesh/coordination.hpp"
#include "amc8270/mesh/hal.hpp"
#include "amc8270/mesh/mesh_packet.hpp"
#include "amc8270/mesh/mesh_security.hpp"
#include "amc8270/mesh/situational_awareness.hpp"

namespace amc8270::mesh {

struct MeshStats {
  std::uint32_t tx_packets = 0;
  std::uint32_t rx_packets = 0;
  std::uint32_t rx_rejected = 0;     // bad framing, or an authentic frame with bad content or sender mismatch
  std::uint32_t rx_auth_failed = 0;  // tag did not verify: corrupted, forged, or another key
  std::uint32_t rx_no_key = 0;       // key epoch unknown or outside its validity window
  std::uint32_t rx_replayed = 0;     // authentic, but already accepted or from an older boot session
  std::uint32_t rx_stale = 0;        // authentic, but sent too long ago or stamped in the future
  std::uint32_t rx_revoked = 0;      // from a drone the GMCS revoked
  std::uint32_t tx_no_key = 0;       // nothing sent: no key epoch valid for sending now
  std::uint32_t local_rejected = 0;  // implausible input from this drone's own avionics
  std::uint32_t peers_discovered = 0;
  std::uint32_t peers_lost = 0;
  std::uint32_t observations_sent = 0;
  std::uint32_t claims_sent = 0;
  std::uint32_t tasks_issued = 0;
  std::uint32_t tasks_cancelled = 0;
  std::uint32_t advisories_sent = 0;
  std::uint32_t priority_alerts_sent = 0;
  std::uint32_t alert_send_failures = 0;
  std::uint32_t signatures_learned = 0;
  std::uint32_t evaluations = 0;
};

class PackMeshManager {
 public:
  // `cfg` must satisfy isValid(cfg) (validation.hpp); check it before constructing.
  // `boot_session` must be greater than on any earlier boot of this drone (a
  // non-volatile boot counter): it is part of every frame's nonce. See mesh_security.hpp.
  // Nothing is sent or accepted until a key epoch is installed.
  PackMeshManager(DroneId self, const MeshConfig& cfg, std::uint32_t boot_session, IClock& clock,
                  IMeshTransceiver& radio, IGmcsLink& gmcs, IGuidanceSink& guidance) noexcept;
  PackMeshManager(const PackMeshManager&) = delete;
  PackMeshManager& operator=(const PackMeshManager&) = delete;
  PackMeshManager(PackMeshManager&&) = delete;
  PackMeshManager& operator=(PackMeshManager&&) = delete;
  ~PackMeshManager() = default;

  // ---- inputs from the rest of the avionics ----------------------------
  // Own state from FCS / INS-GNSS / 7-hour combustion EMU / C2 link manager. `mode` is the
  // GMCS-commanded mode (PACK_ROUTING, ACTIVE_MISSION or RTB); the manager
  // reports VERIFYING itself while it holds a task.
  void updateOwnState(const DroneState& state) noexcept;
  // A reading from a device on the Variable Homing Devices bus. Fills observer, position
  // and event association; broadcasts it to the pack. False if dropped.
  [[nodiscard]] bool reportLocalDetection(SensorObservation obs) noexcept;
  // Operator declared a signature benign: learned locally and shared.
  void applyGmcsDismissal(std::uint32_t signature) noexcept;
  // Mission-data signature known to all drones (not re-broadcast).
  void addKnownBenignSignature(std::uint32_t signature) noexcept;

  // ---- keys: mission data load, or GMCS re-key over the C2 link ---------
  [[nodiscard]] bool installEpochKey(std::uint16_t epoch, const crypto::Key& key, TimeMs valid_from_ms,
                                     TimeMs valid_until_ms) noexcept;
  bool eraseEpochKey(std::uint16_t epoch) noexcept;
  void zeroizeKeys() noexcept;  // emergency erase: the drone falls silent on the mesh
  // GMCS order: refuse `id` from now on, drop it from the pack and forget its
  // readings. Re-keying without it (a new epoch it is not given) completes the exclusion.
  [[nodiscard]] bool revokeDrone(DroneId id) noexcept;

  // ---- periodic work ---------------------------------------------------
  void tick() noexcept;
  // Fuse, rank, nominate verifiers, issue alerts. Called from tick() every
  // coordination_period_ms; public so it can be driven directly in tests.
  void evaluateAndCoordinateAlerts(TimeMs now) noexcept;
  void shutdown() noexcept;  // announce departure (LEAVE) and drop any task

  // ---- inspection ------------------------------------------------------
  const SituationalAwarenessTable& table() const noexcept { return table_; }
  DroneState exportedState() const noexcept;
  bool isVerifying() const noexcept { return task_active_; }
  const VerificationTask& currentTask() const noexcept { return task_; }
  const MeshStats& stats() const noexcept { return stats_; }
  const MeshSecurity& security() const noexcept { return security_; }
  DroneId selfId() const noexcept { return self_; }

 private:
  template <typename Payload>
  [[nodiscard]] bool broadcast(const Payload& payload, TimeMs now) noexcept;

  void receiveAll(TimeMs now) noexcept;
  void handlePacket(const MeshPacket& pkt, TimeMs now) noexcept;
  void sendHeartbeat(TimeMs now) noexcept;
  void sendAntiEntropy(TimeMs now) noexcept;
  void syncOwnTask(const Assignment* mine, TimeMs now) noexcept;
  void cancelTask(bool notify_guidance) noexcept;
  void dispatchAlerts(TimeMs now) noexcept;

  DroneId self_;
  MeshConfig cfg_;
  IClock& clock_;
  IMeshTransceiver& radio_;
  IGmcsLink& gmcs_;
  IGuidanceSink& guidance_;

  MeshSecurity security_;
  SituationalAwarenessTable table_;
  DroneState own_{};
  VerificationTask task_{};
  bool task_active_ = false;

  TimeMs last_heartbeat_ms_ = 0;
  TimeMs last_anti_entropy_ms_ = 0;
  TimeMs last_eval_ms_ = 0;
  TimeMs last_claim_ms_ = 0;
  TimeMs retask_pending_since_ms_ = 0;  // when the target first looked moved (0 = not)
  // An assignment claimed but not yet flown (claim before flying).
  EventId pending_event_ = kNoEvent;
  std::uint8_t pending_role_ = 0;
  TimeMs pending_since_ms_ = 0;
  std::uint32_t ae_cursor_ = 0;  // rotating start of the anti-entropy re-send
  struct IssuedAlert {
    EventId event = kNoEvent;
    AlertLevel level = AlertLevel::NONE;
    TimeMs at_ms = 0;
  };
  StaticVector<IssuedAlert, 8> issued_{};  // own recent alerts, re-announced by anti-entropy
  bool heartbeat_requested_ = false;
  MeshStats stats_{};
};

}  // namespace amc8270::mesh
