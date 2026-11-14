// GMCS-side alert consumer: turns the alerts the pack sends into records an
// operator works through, and accounts for how long everything took.
//
//  - De-duplication. The pack sends one ADVISORY and one PRIORITY per source
//    in the common case, but a mesh partition, a split under multipath or a
//    lost ALERT_ISSUED can report one source under several event ids. An alert
//    folds into an existing record by event id, or else by place (compatible
//    frequency, within max(merge_gate_m, merge_cep_factor * combined CEP), seen
//    within merge_window_ms). A PRIORITY on an ADVISORY record upgrades it and
//    re-opens it for acknowledgement; a better estimate (smaller CEP) updates
//    the position; anything else is counted as a duplicate.
//  - Acknowledgement. Every record starts NEW. An unacknowledged PRIORITY past
//    its deadline (or ADVISORY past its longer one) is flagged as escalated.
//  - Dismissal. The operator declares a record benign. If it carries a
//    signature, the consumer sends it to the pack (IUplink, retried until the
//    C2 link accepts it) and suppresses later alerts on it; the pack then
//    learns it on every drone (SIGNATURE_UPDATE). Without a signature the
//    suppression is local, by place: the pack cannot learn it.
//  - Latency accounting. Transport (drone issue -> GMCS receipt) per link,
//    detection -> GMCS receipt per level, and operator receipt -> acknowledgement.
//
// Ground software, but written to the flight-library rules (fixed capacity,
// no heap, no exceptions) so it can run on an embedded ground terminal too.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "amc8270/mesh/hal.hpp"
#include "amc8270/mesh/static_vector.hpp"
#include "amc8270/mesh/types.hpp"

namespace amc8270::gmcs {

using mesh::AlertLevel;
using mesh::EventId;
using mesh::GmcsAlert;
using mesh::LinkType;
using mesh::TimeMs;

inline constexpr std::size_t kMaxRecords = 64;
inline constexpr std::size_t kMaxIdsPerRecord = 8;
inline constexpr std::size_t kMaxSuppressions = 32;

struct ConsumerConfig {
  double merge_gate_m = 2000.0;
  double merge_cep_factor = 3.0;
  TimeMs merge_window_ms = 30ULL * 60ULL * 1000ULL;
  TimeMs ack_deadline_priority_ms = 30000;
  TimeMs ack_deadline_advisory_ms = 120000;
  TimeMs uplink_retry_ms = 10000;
};

enum class RecordState : std::uint8_t {
  NEW = 0,           // needs the operator (new, or upgraded to PRIORITY)
  ACKNOWLEDGED = 1,  // the operator has seen it
  DISMISSED = 2,     // declared benign; later alerts on it are suppressed
  CLOSED = 3,        // handled; kept so late duplicates still fold into it
};

enum class IngestResult : std::uint8_t {
  NEW_RECORD,  // a source the GMCS had not seen
  UPGRADED,    // ADVISORY -> PRIORITY: back to NEW
  UPDATED,     // same level, better position estimate
  DUPLICATE,   // same source, nothing new
  SUPPRESSED,  // a source the operator dismissed
  REJECTED,    // no room for a new record (every record is open)
};

struct Record {
  std::uint32_t record_id = 0;
  RecordState state = RecordState::NEW;
  AlertLevel level = AlertLevel::NONE;
  mesh::EventClass cls = mesh::EventClass::UNKNOWN;
  std::uint32_t signature = 0;
  mesh::GeoPoint position{};  // the best estimate received (smallest CEP)
  float cep_m = 0.0f;
  float confidence = 0.0f;
  mesh::SensorMask modality_mask = 0;
  mesh::StaticVector<EventId, kMaxIdsPerRecord> event_ids{};  // every pack id it was reported under
  mesh::DroneId last_issuer = mesh::kNoDrone;
  TimeMs first_seen_ms = 0;  // the pack's first detection
  TimeMs first_rx_ms = 0;    // first alert received
  TimeMs level_rx_ms = 0;    // when the current level was first received (starts the ack clock)
  TimeMs last_rx_ms = 0;
  TimeMs acked_ms = 0;
  std::uint16_t reports = 0;     // raw alerts folded into this record
  std::uint16_t duplicates = 0;  // of those, ones that added nothing
  bool escalated = false;        // missed its acknowledgement deadline
};

// Fixed-bin latency histogram (ms) with exact count, sum, min and max.
class LatencyStats {
 public:
  static constexpr std::size_t kBins = 16;
  // Upper edges in ms; the last bin is open.
  static constexpr std::array<double, kBins - 1> kEdges{20, 50, 100, 200, 500, 1000, 2000, 5000,
                                                        10000, 20000, 60000, 120000, 300000, 600000, 1800000};
  void add(double ms) noexcept;
  std::uint32_t count() const noexcept { return count_; }
  double mean() const noexcept { return count_ == 0 ? 0.0 : sum_ / count_; }
  double min() const noexcept { return count_ == 0 ? 0.0 : min_; }
  double max() const noexcept { return count_ == 0 ? 0.0 : max_; }
  // Percentile (0..100) from the bins, interpolated within a bin; exact at the ends.
  double percentile(double p) const noexcept;
  const std::array<std::uint32_t, kBins>& bins() const noexcept { return bins_; }

 private:
  std::array<std::uint32_t, kBins> bins_{};
  std::uint32_t count_ = 0;
  double sum_ = 0.0, min_ = 0.0, max_ = 0.0;
};

struct ConsumerStats {
  std::uint32_t alerts = 0;      // raw alerts received
  std::uint32_t records = 0;     // records created
  std::uint32_t upgrades = 0;
  std::uint32_t updates = 0;
  std::uint32_t duplicates = 0;  // folded into an existing record, nothing new
  std::uint32_t merged_by_place = 0;  // of the folded ones, matched by place under a new event id
  std::uint32_t suppressed = 0;  // on a dismissed source
  std::uint32_t rejected = 0;
  std::uint32_t escalations = 0;
  std::uint32_t uplinks_sent = 0;
  std::uint32_t uplink_failures = 0;
  std::array<LatencyStats, 3> transport{};        // by LinkType: LOS, LEO, GEO
  std::array<LatencyStats, 3> detection_to_gmcs{};  // by level: [1] ADVISORY, [2] PRIORITY
  LatencyStats operator_ack{};                    // current level received -> acknowledged
};

// The GMCS's path to the pack for a dismissal (through the C2 link manager to
// the drone the operator flies, or any drone with a link). True if queued.
class IUplink {
 public:
  virtual ~IUplink() = default;
  virtual bool sendDismissal(std::uint32_t signature) = 0;

 protected:
  IUplink() = default;
  IUplink(const IUplink&) = default;
  IUplink& operator=(const IUplink&) = default;
  IUplink(IUplink&&) = default;
  IUplink& operator=(IUplink&&) = default;
};

class AlertConsumer {
 public:
  AlertConsumer(const ConsumerConfig& cfg, IUplink& uplink) noexcept;
  AlertConsumer(const AlertConsumer&) = delete;
  AlertConsumer& operator=(const AlertConsumer&) = delete;
  AlertConsumer(AlertConsumer&&) = delete;
  AlertConsumer& operator=(AlertConsumer&&) = delete;
  ~AlertConsumer() = default;

  IngestResult onAlert(const GmcsAlert& alert, LinkType link, TimeMs rx_ms) noexcept;

  // ---- the operator ------------------------------------------------------
  [[nodiscard]] bool acknowledge(std::uint32_t record_id, TimeMs now) noexcept;
  // Declare a record benign (see the header comment). False if unknown.
  [[nodiscard]] bool dismiss(std::uint32_t record_id, TimeMs now) noexcept;
  // Dismiss a signature the pack has not alerted on (known from other sources).
  [[nodiscard]] bool dismissSignature(std::uint32_t signature, TimeMs now) noexcept;
  [[nodiscard]] bool close(std::uint32_t record_id) noexcept;

  // Acknowledgement deadlines and uplink retries. Call about once a second.
  void tick(TimeMs now) noexcept;

  // ---- inspection --------------------------------------------------------
  const mesh::StaticVector<Record, kMaxRecords>& records() const noexcept { return records_; }
  const Record* find(std::uint32_t record_id) const noexcept;
  const Record* findByEvent(EventId id) const noexcept;
  std::size_t openCount() const noexcept;  // NEW records: what the operator still has to look at
  const ConsumerStats& stats() const noexcept { return stats_; }
  bool uplinkPending() const noexcept;

 private:
  struct Suppression {
    std::uint32_t signature = 0;     // dismissed frequency (0: by place only)
    mesh::GeoPoint position{};
    float cep_m = 0.0f;
    std::uint32_t record_id = 0;
    bool uplink_done = false;        // the C2 link has accepted it (signature only)
    TimeMs last_attempt_ms = 0;
  };

  Record* findMutable(std::uint32_t record_id) noexcept;
  Record* matchByPlace(const GmcsAlert& a, TimeMs rx_ms) noexcept;
  const Suppression* suppressionFor(const GmcsAlert& a) const noexcept;
  bool addSuppression(const Suppression& s) noexcept;
  void trySend(Suppression& s, TimeMs now) noexcept;
  bool makeRoom() noexcept;

  ConsumerConfig cfg_;
  IUplink& uplink_;
  mesh::StaticVector<Record, kMaxRecords> records_{};
  mesh::StaticVector<Suppression, kMaxSuppressions> suppressions_{};
  std::uint32_t next_id_ = 1;
  ConsumerStats stats_{};
};

}  // namespace amc8270::gmcs
