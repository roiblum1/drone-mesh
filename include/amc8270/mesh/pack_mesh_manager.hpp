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
#include "amc8270/mesh/situational_awareness.hpp"

namespace amc8270::mesh {

struct MeshStats {
  std::uint32_t tx_packets = 0;
  std::uint32_t rx_packets = 0;
  std::uint32_t rx_rejected = 0;  // bad CRC / framing / range
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
  PackMeshManager(DroneId self, const MeshConfig& cfg, IClock& clock, IMeshTransceiver& radio, IGmcsLink& gmcs,
                  IGuidanceSink& guidance) noexcept;

  // ---- inputs from the rest of the avionics ----------------------------
  // Own state from FCS / INS-GNSS / EMU / C2 link manager. `mode` is the
  // GMCS-commanded mode (PACK_ROUTING, ACTIVE_MISSION or RTB); the manager
  // reports VERIFYING itself while it holds a task.
  void updateOwnState(const DroneState& state) noexcept;
  // A reading from the homing/detection sensor bus. Fills observer, position
  // and event association; broadcasts it to the pack. False if dropped.
  bool reportLocalDetection(SensorObservation obs) noexcept;
  // Operator declared a signature benign: learned locally and shared.
  void applyGmcsDismissal(std::uint32_t signature) noexcept;
  // Mission-data signature known to all drones (not re-broadcast).
  void addKnownBenignSignature(std::uint32_t signature) noexcept;

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
  DroneId selfId() const noexcept { return self_; }

 private:
  template <typename Payload>
  bool broadcast(const Payload& payload, TimeMs now) noexcept;

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

  SituationalAwarenessTable table_;
  DroneState own_{};
  VerificationTask task_{};
  bool task_active_ = false;

  std::uint32_t seq_ = 0;
  TimeMs last_heartbeat_ms_ = 0;
  TimeMs last_anti_entropy_ms_ = 0;
  TimeMs last_eval_ms_ = 0;
  TimeMs last_claim_ms_ = 0;
  bool heartbeat_requested_ = false;
  MeshStats stats_{};
};

}  // namespace amc8270::mesh
