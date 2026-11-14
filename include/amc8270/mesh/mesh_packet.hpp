// Mesh wire format, version 2: every frame is authenticated and encrypted.
//
//  0      2    3     4     6       8         12      16     18             18+len   +16
//  +------+----+-----+-----+-------+---------+-------+------+--------------+--------+
//  |magic |ver |flags|src  |epoch  |session  |seq    |len   |ciphertext    |tag     |
//  |0x8270|2   |0    |u16  |u16    |u32      |u32    |u16   |len bytes     |16 B    |
//  +------+----+-----+-----+-------+---------+-------+------+--------------+--------+
//   \_____________ clear, authenticated as AAD ____________/ \__ChaCha20__/ Poly1305
//
// The ciphertext is the body: type u8, time_ms u64, then the payload. Fields
// are written explicitly little-endian (no struct copies onto the wire) and
// every field is validated on decode. This header holds the plaintext codec;
// mesh_security.hpp seals and opens whole frames.
#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include "amc8270/mesh/detected_event.hpp"
#include "amc8270/mesh/drone_state.hpp"
#include "amc8270/mesh/types.hpp"

namespace amc8270::mesh {

inline constexpr std::uint16_t kPacketMagic = 0x8270;
inline constexpr std::uint8_t kPacketVersion = 2;
inline constexpr std::size_t kFrameHeaderBytes = 18;
inline constexpr std::size_t kFrameTagBytes = 16;
inline constexpr std::size_t kBodyHeaderBytes = 9;  // type u8 + time_ms u64
inline constexpr std::size_t kMaxBodyBytes = kMaxPacketBytes - kFrameHeaderBytes - kFrameTagBytes;

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
  std::uint16_t epoch = 0;    // key epoch the frame was sealed under
  std::uint32_t session = 0;  // sender's boot session; never repeats for a drone
  std::uint32_t seq = 0;      // per session, shared by all types (gap = loss)
  TimeMs time_ms = 0;         // sender's clock at send time (inside the ciphertext)
};

// Variant index == MsgType value; index 0 (monostate) means "empty".
struct MeshPacket {
  PacketHeader header{};
  std::variant<std::monostate, HeartbeatPayload, ObservationPayload, TaskClaimPayload,
               AlertIssuedPayload, SignatureUpdatePayload, LeavePayload>
      payload;  // default-constructs to std::monostate

  MsgType type() const noexcept { return static_cast<MsgType>(payload.index()); }
};

// Largest body is an OBSERVATION: 9 + 61 = 70 bytes, so a frame is at most 104.
static_assert(kBodyHeaderBytes + 61 <= kMaxBodyBytes, "kMaxPacketBytes must hold a sealed OBSERVATION frame");

// The clear part of a frame.
struct FrameHeader {
  DroneId src = kNoDrone;
  std::uint16_t epoch = 0;
  std::uint32_t session = 0;
  std::uint32_t seq = 0;
  std::uint16_t body_len = 0;  // ciphertext bytes, excluding the tag
};

void writeFrameHeader(const FrameHeader& h, std::uint8_t* out18) noexcept;
// False unless magic, version, flags, src, session, seq and the length all check
// out against `len`, the size of the whole datagram.
[[nodiscard]] bool parseFrameHeader(const std::uint8_t* buf, std::size_t len, FrameHeader& out) noexcept;

// Plaintext body: type, time_ms, payload. Returns the size, or 0 if the packet
// is empty or does not fit.
[[nodiscard]] std::size_t encodeBody(const MeshPacket& pkt, std::uint8_t* buf, std::size_t cap) noexcept;

// False on any framing, range or finiteness violation, or a heartbeat that
// describes a drone other than `src`. Sets out.header.src and time_ms; the
// other header fields are left for the caller. `out` is unspecified on failure.
[[nodiscard]] bool decodeBody(const std::uint8_t* buf, std::size_t len, DroneId src, MeshPacket& out) noexcept;

}  // namespace amc8270::mesh
