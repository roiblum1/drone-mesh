#include "amc8270/mesh/mesh_security.hpp"

#include <array>
#include <cstring>

namespace amc8270::mesh {

namespace {

void put16(std::uint8_t* p, std::uint16_t v) noexcept {
  p[0] = static_cast<std::uint8_t>(v);
  p[1] = static_cast<std::uint8_t>(v >> 8U);
}

void put32(std::uint8_t* p, std::uint32_t v) noexcept {
  for (unsigned i = 0; i < 4U; ++i) p[i] = static_cast<std::uint8_t>(v >> (8U * i));
}

TimeMs saturatingAdd(TimeMs a, TimeMs b) noexcept { return a > kNoExpiry - b ? kNoExpiry : a + b; }

}  // namespace

void deriveDroneKey(const crypto::Key& epoch_key, std::uint16_t epoch, DroneId drone, crypto::Key& out) noexcept {
  crypto::Nonce n{};
  n[0] = 'A';
  n[1] = 'M';
  n[2] = 'C';
  n[3] = 'K';
  put16(n.data() + 4, drone);
  put16(n.data() + 6, epoch);
  n[8] = 'v';
  n[9] = '1';
  std::array<std::uint8_t, crypto::kBlockBytes> block{};
  crypto::chacha20Block(epoch_key, 0, n, block.data());
  std::memcpy(out.data(), block.data(), out.size());
  crypto::secureZero(block.data(), block.size());
}

void frameNonce(const FrameHeader& h, crypto::Nonce& out) noexcept {
  put32(out.data(), h.session);
  put32(out.data() + 4, h.seq);
  put16(out.data() + 8, h.src);
  put16(out.data() + 10, h.epoch);
}

MeshSecurity::MeshSecurity(DroneId self, std::uint32_t boot_session, const MeshConfig& cfg) noexcept
    : self_(self), session_(boot_session), cfg_(cfg) {}

MeshSecurity::~MeshSecurity() { zeroizeAll(); }

// ------------------------------------------------------------------- keys

void MeshSecurity::clearSlot(EpochSlot& slot) noexcept {
  crypto::secureZero(slot.key.data(), slot.key.size());
  crypto::secureZero(slot.own_key.data(), slot.own_key.size());
  slot.used = false;
  slot.epoch = 0;
  slot.valid_from_ms = 0;
  slot.valid_until_ms = 0;
}

bool MeshSecurity::installEpochKey(std::uint16_t epoch, const crypto::Key& epoch_key, TimeMs valid_from_ms,
                                   TimeMs valid_until_ms) noexcept {
  if (valid_until_ms <= valid_from_ms) return false;
  EpochSlot* slot = nullptr;
  for (auto& s : epochs_) {
    if (s.used && s.epoch == epoch) slot = &s;
  }
  for (auto& s : epochs_) {
    if (slot == nullptr && !s.used) slot = &s;
  }
  if (slot == nullptr) return false;
  clearSlot(*slot);
  slot->used = true;
  slot->epoch = epoch;
  slot->valid_from_ms = valid_from_ms;
  slot->valid_until_ms = valid_until_ms;
  slot->key = epoch_key;
  deriveDroneKey(epoch_key, epoch, self_, slot->own_key);
  return true;
}

bool MeshSecurity::eraseEpochKey(std::uint16_t epoch) noexcept {
  for (auto& s : epochs_) {
    if (s.used && s.epoch == epoch) {
      clearSlot(s);
      return true;
    }
  }
  return false;
}

void MeshSecurity::zeroizeAll() noexcept {
  for (auto& s : epochs_) clearSlot(s);
}

std::size_t MeshSecurity::epochCount() const noexcept {
  std::size_t n = 0;
  for (const auto& s : epochs_) n += s.used ? 1U : 0U;
  return n;
}

const MeshSecurity::EpochSlot* MeshSecurity::findEpoch(std::uint16_t epoch) const noexcept {
  for (const auto& s : epochs_) {
    if (s.used && s.epoch == epoch) return &s;
  }
  return nullptr;
}

bool MeshSecurity::acceptsAt(const EpochSlot& slot, TimeMs now) const noexcept {
  const TimeMs grace = cfg_.key_epoch_grace_ms;
  return saturatingAdd(now, grace) >= slot.valid_from_ms && now <= saturatingAdd(slot.valid_until_ms, grace);
}

bool MeshSecurity::sendEpoch(TimeMs now, std::uint16_t& epoch) const noexcept {
  const EpochSlot* best = nullptr;
  for (const auto& s : epochs_) {
    if (!s.used || now < s.valid_from_ms || now >= s.valid_until_ms) continue;
    if (best == nullptr || s.valid_from_ms > best->valid_from_ms ||
        (s.valid_from_ms == best->valid_from_ms && s.epoch > best->epoch)) {
      best = &s;
    }
  }
  if (best == nullptr) return false;
  epoch = best->epoch;
  return true;
}

bool MeshSecurity::revokeDrone(DroneId id) noexcept {
  if (id == kNoDrone || id == self_) return false;
  if (isRevoked(id)) return true;
  if (!revoked_.push_back(id)) return false;
  for (std::size_t i = 0; i < replay_.size(); ++i) {
    if (replay_[i].src == id) {
      replay_.erase_at(i);
      break;
    }
  }
  return true;
}

bool MeshSecurity::isRevoked(DroneId id) const noexcept {
  for (const DroneId r : revoked_) {
    if (r == id) return true;
  }
  return false;
}

// ----------------------------------------------------------------- frames

std::size_t MeshSecurity::seal(MeshPacket& pkt, TimeMs now, std::uint8_t* buf, std::size_t cap) noexcept {
  std::uint16_t epoch = 0;
  if (buf == nullptr || session_ == 0 || seq_ == ~static_cast<std::uint32_t>(0) || !sendEpoch(now, epoch)) return 0;
  const EpochSlot* slot = findEpoch(epoch);
  if (slot == nullptr) return 0;  // unreachable: sendEpoch found it

  pkt.header.src = self_;
  pkt.header.epoch = epoch;
  pkt.header.session = session_;
  pkt.header.seq = seq_ + 1;
  pkt.header.time_ms = now;

  std::array<std::uint8_t, kMaxBodyBytes> body{};
  const std::size_t n = encodeBody(pkt, body.data(), body.size());
  if (n == 0 || cap < kFrameHeaderBytes + n + kFrameTagBytes) return 0;

  FrameHeader h;
  h.src = self_;
  h.epoch = epoch;
  h.session = session_;
  h.seq = pkt.header.seq;
  h.body_len = static_cast<std::uint16_t>(n);
  writeFrameHeader(h, buf);
  crypto::Nonce nonce{};
  frameNonce(h, nonce);
  crypto::Tag tag{};
  crypto::aeadSeal(slot->own_key, nonce, buf, kFrameHeaderBytes, body.data(), n, buf + kFrameHeaderBytes, tag);
  std::memcpy(buf + kFrameHeaderBytes + n, tag.data(), tag.size());
  crypto::secureZero(body.data(), body.size());
  seq_ = h.seq;  // only now is the nonce spent
  return kFrameHeaderBytes + n + kFrameTagBytes;
}

bool MeshSecurity::replayCheckAndCommit(const FrameHeader& h, TimeMs now) noexcept {
  ReplayState* st = nullptr;
  for (auto& r : replay_) {
    if (r.src == h.src) st = &r;
  }
  if (st != nullptr) {
    if (h.session < st->session) return false;  // an older boot of this sender
    if (h.session == st->session) {
      if (h.seq > st->top) {
        const std::uint32_t shift = h.seq - st->top;
        st->bitmap = shift >= kReplayWindowFrames ? 1U : ((st->bitmap << shift) | 1U);
        st->top = h.seq;
      } else {
        const std::uint32_t back = st->top - h.seq;
        if (back >= kReplayWindowFrames) return false;  // older than the window
        const std::uint64_t bit = static_cast<std::uint64_t>(1) << back;
        if ((st->bitmap & bit) != 0U) return false;  // already accepted
        st->bitmap |= bit;
      }
      st->last_ok_ms = now;
      return true;
    }
    *st = ReplayState{h.src, h.session, h.seq, 1U, now};  // the sender rebooted
    return true;
  }
  if (replay_.full()) {  // forget the sender heard from least recently
    std::size_t oldest = 0;
    for (std::size_t i = 1; i < replay_.size(); ++i) {
      if (replay_[i].last_ok_ms < replay_[oldest].last_ok_ms) oldest = i;
    }
    replay_.erase_at(oldest);
  }
  (void)replay_.push_back(ReplayState{h.src, h.session, h.seq, 1U, now});  // cannot fail: made room above
  return true;
}

OpenStatus MeshSecurity::open(const std::uint8_t* buf, std::size_t len, TimeMs now, MeshPacket& out) noexcept {
  FrameHeader h;
  if (!parseFrameHeader(buf, len, h) || h.body_len > kMaxBodyBytes) return OpenStatus::MALFORMED;
  if (h.src == self_) return OpenStatus::LOOPBACK;
  if (isRevoked(h.src)) return OpenStatus::REVOKED;
  const EpochSlot* slot = findEpoch(h.epoch);
  if (slot == nullptr || !acceptsAt(*slot, now)) return OpenStatus::NO_KEY;

  crypto::Key key{};
  deriveDroneKey(slot->key, h.epoch, h.src, key);
  crypto::Nonce nonce{};
  frameNonce(h, nonce);
  crypto::Tag tag{};
  std::memcpy(tag.data(), buf + kFrameHeaderBytes + h.body_len, tag.size());
  std::array<std::uint8_t, kMaxBodyBytes> body{};
  const bool authentic =
      crypto::aeadOpen(key, nonce, buf, kFrameHeaderBytes, buf + kFrameHeaderBytes, h.body_len, tag, body.data());
  crypto::secureZero(key.data(), key.size());
  if (!authentic) return OpenStatus::AUTH_FAILED;

  MeshPacket pkt;
  const bool decoded = decodeBody(body.data(), h.body_len, h.src, pkt);
  crypto::secureZero(body.data(), body.size());
  if (!decoded) return OpenStatus::MALFORMED;

  const TimeMs sent = pkt.header.time_ms;
  if (sent > saturatingAdd(now, cfg_.max_clock_skew_ms) || elapsedMs(now, sent) > cfg_.frame_max_age_ms) {
    return OpenStatus::STALE;
  }
  if (!replayCheckAndCommit(h, now)) return OpenStatus::REPLAYED;

  pkt.header.epoch = h.epoch;
  pkt.header.session = h.session;
  pkt.header.seq = h.seq;
  out = pkt;
  return OpenStatus::OK;
}

}  // namespace amc8270::mesh
