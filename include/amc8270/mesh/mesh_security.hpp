// Authenticated, encrypted mesh frames: key epochs, per-drone keys, replay
// protection and revocation. Wraps the plaintext body codec in mesh_packet.hpp.
//
// Keys. The GMCS issues one 256-bit key per epoch (mission data load, or a
// re-key over a C2 link). Each drone seals with its own key for that epoch,
// derived from the epoch key and its drone id; the id and epoch are also in
// the authenticated header, so a frame verifies only under the key of the
// drone it names. A drone is excluded from the pack by issuing the next epoch
// key to every drone but it, and can be dropped at once with revokeDrone().
//
// Limit (symmetric keys): every member holds the epoch key, so a drone that is
// captured with its key store intact can forge frames in any member's name
// until the next epoch. Against that, a drone needs per-sender signatures.
//
// Nonces. The 96-bit nonce is (boot session, seq, src, epoch). `boot_session`
// must be strictly greater than on any earlier boot of this drone (a counter
// in non-volatile memory). If it ever repeats under one epoch key, keystream
// and Poly1305 keys repeat and the mesh loses both secrecy and integrity.
//
// Replay. A frame is accepted once: per sender, a 64-frame sliding window
// over (session, seq) rejects duplicates, and frames from an older session
// are refused. A frame whose authenticated send time is more than
// frame_max_age_ms old (or from the future beyond max_clock_skew_ms) is
// refused too, which bounds replay to a drone that has no window state yet.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "amc8270/mesh/config.hpp"
#include "amc8270/mesh/crypto.hpp"
#include "amc8270/mesh/mesh_packet.hpp"
#include "amc8270/mesh/static_vector.hpp"

namespace amc8270::mesh {

inline constexpr std::size_t kMaxKeyEpochs = 4;
inline constexpr std::uint32_t kReplayWindowFrames = 64;
inline constexpr TimeMs kNoExpiry = ~static_cast<TimeMs>(0);

enum class OpenStatus : std::uint8_t {
  OK = 0,
  LOOPBACK,     // our own frame (multicast loopback); not an error
  MALFORMED,    // bad framing, or an authentic body that fails validation
  NO_KEY,       // unknown key epoch, or one outside its validity window
  AUTH_FAILED,  // tag mismatch: corrupted, forged, or sealed under another key
  REVOKED,      // the sender has been revoked
  STALE,        // authentic, but too old or too far in the future
  REPLAYED,     // authentic and fresh, but already accepted, or from an older session
};

class MeshSecurity {
 public:
  MeshSecurity(DroneId self, std::uint32_t boot_session, const MeshConfig& cfg) noexcept;
  MeshSecurity(const MeshSecurity&) = delete;
  MeshSecurity& operator=(const MeshSecurity&) = delete;
  MeshSecurity(MeshSecurity&&) = delete;
  MeshSecurity& operator=(MeshSecurity&&) = delete;
  ~MeshSecurity();  // zeroizes every key

  // ---- keys --------------------------------------------------------------
  // Install (or replace) the key for `epoch`, used to send from valid_from_ms
  // until valid_until_ms and accepted key_epoch_grace_ms either side.
  // False if the window is empty or all kMaxKeyEpochs slots hold other epochs.
  [[nodiscard]] bool installEpochKey(std::uint16_t epoch, const crypto::Key& epoch_key, TimeMs valid_from_ms,
                                     TimeMs valid_until_ms) noexcept;
  bool eraseEpochKey(std::uint16_t epoch) noexcept;  // zeroizes; false if not installed
  void zeroizeAll() noexcept;                         // emergency erase of every key
  std::size_t epochCount() const noexcept;
  // The epoch seal() would use now: the newest one whose send window holds `now`.
  [[nodiscard]] bool sendEpoch(TimeMs now, std::uint16_t& epoch) const noexcept;

  // Refuse every frame from `id` from now on. False if `id` is self or the list is full.
  [[nodiscard]] bool revokeDrone(DroneId id) noexcept;
  bool isRevoked(DroneId id) const noexcept;

  // ---- frames ------------------------------------------------------------
  // Fills pkt.header (src, epoch, session, next seq, time_ms = now), then
  // encodes and seals. Returns the frame size, or 0 if there is no send key
  // for `now`, the sequence space of this session is used up, or it does not
  // fit. A failed call consumes no sequence number.
  [[nodiscard]] std::size_t seal(MeshPacket& pkt, TimeMs now, std::uint8_t* buf, std::size_t cap) noexcept;

  // Verifies, decrypts, decodes and checks freshness and replay, in that
  // order. `out` holds the packet only when the result is OK.
  [[nodiscard]] OpenStatus open(const std::uint8_t* buf, std::size_t len, TimeMs now, MeshPacket& out) noexcept;

  DroneId self() const noexcept { return self_; }
  std::uint32_t session() const noexcept { return session_; }
  std::uint32_t lastSeq() const noexcept { return seq_; }

 private:
  struct EpochSlot {
    bool used = false;
    std::uint16_t epoch = 0;
    TimeMs valid_from_ms = 0;
    TimeMs valid_until_ms = 0;
    crypto::Key key{};
    crypto::Key own_key{};  // this drone's sending key, derived once
  };
  struct ReplayState {
    DroneId src = kNoDrone;
    std::uint32_t session = 0;
    std::uint32_t top = 0;      // highest seq accepted in this session
    std::uint64_t bitmap = 0;   // bit k set: seq (top - k) accepted
    TimeMs last_ok_ms = 0;
  };

  const EpochSlot* findEpoch(std::uint16_t epoch) const noexcept;
  bool acceptsAt(const EpochSlot& slot, TimeMs now) const noexcept;
  [[nodiscard]] bool replayCheckAndCommit(const FrameHeader& h, TimeMs now) noexcept;
  void clearSlot(EpochSlot& slot) noexcept;

  DroneId self_;
  std::uint32_t session_;
  MeshConfig cfg_;
  std::uint32_t seq_ = 0;
  std::array<EpochSlot, kMaxKeyEpochs> epochs_{};
  StaticVector<ReplayState, kMaxPackSize> replay_{};
  StaticVector<DroneId, kMaxPackSize> revoked_{};
};

// Per-drone sending key for one epoch: the first 32 bytes of the ChaCha20
// block (epoch key, counter 0, nonce "AMCK" | drone | epoch | "v1" | 0 0).
// Exposed for tests.
void deriveDroneKey(const crypto::Key& epoch_key, std::uint16_t epoch, DroneId drone, crypto::Key& out) noexcept;

// session | seq | src | epoch, little-endian. Exposed for tests.
void frameNonce(const FrameHeader& h, crypto::Nonce& out) noexcept;

}  // namespace amc8270::mesh
