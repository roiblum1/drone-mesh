// Fixed-capacity vector: std::array storage, no heap, no exceptions.
// T must be default-constructible and copy-assignable.
#pragma once

#include <array>
#include <cstddef>

namespace amc8270::mesh {

template <typename T, std::size_t N>
class StaticVector {
 public:
  std::size_t size() const noexcept { return size_; }
  static constexpr std::size_t capacity() noexcept { return N; }
  bool empty() const noexcept { return size_ == 0; }
  bool full() const noexcept { return size_ == N; }
  void clear() noexcept { size_ = 0; }

  // Returns false (and stores nothing) when full.
  bool push_back(const T& v) noexcept {
    if (size_ >= N) return false;
    data_[size_++] = v;
    return true;
  }

  // Order-preserving erase.
  void erase_at(std::size_t i) noexcept {
    if (i >= size_) return;
    for (std::size_t k = i + 1; k < size_; ++k) data_[k - 1] = data_[k];
    --size_;
  }

  T& operator[](std::size_t i) noexcept { return data_[i]; }
  const T& operator[](std::size_t i) const noexcept { return data_[i]; }

  T* begin() noexcept { return data_.data(); }
  T* end() noexcept { return data_.data() + size_; }
  const T* begin() const noexcept { return data_.data(); }
  const T* end() const noexcept { return data_.data() + size_; }

 private:
  std::array<T, N> data_{};
  std::size_t size_{0};
};

}  // namespace amc8270::mesh
