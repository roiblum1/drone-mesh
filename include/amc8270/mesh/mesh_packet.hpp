// Mesh wire format. Explicit little-endian serialisation (no struct memcpy),
// CRC-32 over header + payload, strict validation of every field on decode.
//
//  0      2     3     4      6     8       12        20          20+len   +4
//  +------+-----+-----+------+-----+-------+---------+-----------+--------+
//  |magic |ver  |type |src   |len  |seq    |time_ms  |payload    |crc32   |
//  |0x8270|1    |1..6 |u16   |u16  |u32    |u64      |len bytes  |u32     |
//  +------+-----+-----+------+-----+-------+---------+-----------+--------+
#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include "amc8270/mesh/detected_event.hpp"
#include "amc8270/mesh/drone_state.hpp"
#include "amc8270/mesh/types.hpp"

namespace amc8270::mesh {

inline constexpr std::uint16_t kPacketMagic = 0x8270;
inline constexpr std::uint8_t kPacketVersion = 1;
inline constexpr std::size_t kHeaderBytes = 20;
inline constexpr std::size_t kCrcBytes = 4;

enum class MsgType : std::uint8_t {
  HEARTBEAT = 1,         // peer discovery + state sync (position, fuel, sensor, mode)
  OBSERVATION = 2,       // one sensor reading about one source
  TASK_CLAIM = 3,        // "I take verification slot R of event E"
  ALERT_ISSUED = 4,      // suppresses duplicate GMCS alerts
  SIGNATURE_UPDATE = 5,  // GMCS-taught benign signature
  LEAVE = 6,
};
inline constexpr std::uint8_t kMsgTypeMax = 6;

struct HeartbeatPayload {
  DroneState state{};
};
struct ObservationPayload {
  SensorObservation obs{};
};
struct TaskClaimPayload {
  EventId event_id = kNoEvent;
  DroneId claimant = kNoDrone;
  std::uint8_t role = 0;
  float score = 0.0f;
};
struct AlertIssuedPayload {
  EventId event_id = kNoEvent;
  AlertLevel level = AlertLevel::NONE;
  DroneId issuer = kNoDrone;
};
struct SignatureUpdatePayload {
  std::uint32_t signature = 0;
};
struct LeavePayload {};

struct PacketHeader {
  DroneId src = kNoDrone;
  std::uint32_t seq = 0;  // per-sender, shared by all types (gap = loss)
  TimeMs time_ms = 0;
};

// Variant index == MsgType value; index 0 (monostate) means "empty".
struct MeshPacket {
  PacketHeader header{};
  std::variant<std::monostate, HeartbeatPayload, ObservationPayload, TaskClaimPayload,
               AlertIssuedPayload, SignatureUpdatePayload, LeavePayload>
      payload;  // default-constructs to std::monostate

  MsgType type() const noexcept { return static_cast<MsgType>(payload.index()); }
};

// Largest encoded frame is an OBSERVATION: 20 + 61 + 4 = 85 bytes.
static_assert(kHeaderBytes + 61 + kCrcBytes <= kMaxPacketBytes, "kMaxPacketBytes must hold an OBSERVATION frame");

[[nodiscard]] std::uint32_t crc32(const std::uint8_t* data, std::size_t len) noexcept;

// Returns the encoded size, or 0 if the packet is empty or does not fit.
[[nodiscard]] std::size_t encodePacket(const MeshPacket& pkt, std::uint8_t* buf, std::size_t cap) noexcept;

// False on any framing, CRC, range or finiteness violation; `out` is then unspecified.
[[nodiscard]] bool decodePacket(const std::uint8_t* buf, std::size_t len, MeshPacket& out) noexcept;

}  // namespace amc8270::mesh
