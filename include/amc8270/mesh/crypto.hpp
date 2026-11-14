// ChaCha20-Poly1305 AEAD (RFC 8439) for the mesh, written to the flight-library
// rules: no heap, no exceptions, bounded loops, constant-time tag check.
//
// This is a from-the-RFC implementation, checked against the RFC 8439 test
// vectors and against an independent implementation (tests/test_main.cpp).
// It has not been through a crypto-module validation. For a fielded system,
// replace the bodies of these functions with a validated module; nothing else
// in the mesh depends on how they are implemented.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace amc8270::mesh::crypto {

inline constexpr std::size_t kKeyBytes = 32;
inline constexpr std::size_t kNonceBytes = 12;
inline constexpr std::size_t kTagBytes = 16;
inline constexpr std::size_t kBlockBytes = 64;

using Key = std::array<std::uint8_t, kKeyBytes>;
using Nonce = std::array<std::uint8_t, kNonceBytes>;
using Tag = std::array<std::uint8_t, kTagBytes>;

// One 64-byte ChaCha20 keystream block (RFC 8439 2.3).
void chacha20Block(const Key& key, std::uint32_t counter, const Nonce& nonce, std::uint8_t* out64) noexcept;

// XOR `len` bytes of keystream, starting at block `counter`, into in -> out.
// in and out may be the same buffer.
void chacha20Xor(const Key& key, std::uint32_t counter, const Nonce& nonce, const std::uint8_t* in, std::uint8_t* out,
                 std::size_t len) noexcept;

// Incremental Poly1305 (RFC 8439 2.5). A one-time key must never be reused.
class Poly1305 {
 public:
  explicit Poly1305(const std::uint8_t* key32) noexcept;
  Poly1305(const Poly1305&) = delete;
  Poly1305& operator=(const Poly1305&) = delete;
  Poly1305(Poly1305&&) = delete;
  Poly1305& operator=(Poly1305&&) = delete;
  ~Poly1305();  // zeroizes the key material

  void update(const std::uint8_t* data, std::size_t len) noexcept;
  void finish(Tag& tag) noexcept;

 private:
  void blocks(const std::uint8_t* m, std::size_t bytes, bool final_block) noexcept;

  std::array<std::uint32_t, 5> r_{};
  std::array<std::uint32_t, 5> h_{};
  std::array<std::uint32_t, 4> pad_{};
  std::array<std::uint8_t, 16> buf_{};
  std::size_t leftover_ = 0;
};

// AEAD_CHACHA20_POLY1305 (RFC 8439 2.8). pt and ct may be the same buffer.
void aeadSeal(const Key& key, const Nonce& nonce, const std::uint8_t* aad, std::size_t aad_len, const std::uint8_t* pt,
              std::size_t len, std::uint8_t* ct, Tag& tag) noexcept;

// Verifies the tag first and decrypts only if it matches. On failure `pt` is
// zero-filled, so a caller that ignores the result still sees no plaintext.
[[nodiscard]] bool aeadOpen(const Key& key, const Nonce& nonce, const std::uint8_t* aad, std::size_t aad_len,
                            const std::uint8_t* ct, std::size_t len, const Tag& tag, std::uint8_t* pt) noexcept;

// Time depends only on n, not on where the buffers differ.
[[nodiscard]] bool constantTimeEqual(const std::uint8_t* a, const std::uint8_t* b, std::size_t n) noexcept;

// Overwrite key material in a way the optimiser may not remove.
void secureZero(void* p, std::size_t n) noexcept;

}  // namespace amc8270::mesh::crypto
