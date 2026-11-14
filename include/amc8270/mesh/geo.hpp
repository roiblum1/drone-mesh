// Geodesy and bearing-only triangulation. Spherical-earth model: errors are
// well under the CEP values this system works with at pack scale (< 100 km).
#pragma once

#include <cstddef>

#include "amc8270/mesh/types.hpp"

namespace amc8270::mesh {

struct Vec2 {
  double e = 0.0;  // east, metres
  double n = 0.0;  // north, metres
};

double wrap360(double deg) noexcept;  // [0, 360)
double wrap180(double deg) noexcept;  // [-180, 180)
double angleDiffDeg(double a, double b) noexcept;  // smallest difference, 0..180
double distanceM(const GeoPoint& a, const GeoPoint& b) noexcept;
double bearingDeg(const GeoPoint& from, const GeoPoint& to) noexcept;  // [0,360)
GeoPoint destinationPoint(const GeoPoint& from, double bearing_deg, double distance_m) noexcept;

// East/north tangent plane around an origin.
class LocalFrame {
 public:
  explicit LocalFrame(const GeoPoint& origin) noexcept;
  Vec2 toEnu(const GeoPoint& p) const noexcept;
  GeoPoint fromEnu(const Vec2& v, float alt_m) const noexcept;

 private:
  GeoPoint origin_;
  double m_per_deg_lat_;
  double m_per_deg_lon_;
};

struct BearingLine {
  Vec2 origin;
  double bearing_rad = 0.0;  // clockwise from north
  double sigma_rad = 0.05;   // 1-sigma angular error
};

struct TriangulationResult {
  bool ok = false;
  Vec2 position{};
  double cep_m = 0.0;
};

// Weighted least-squares intersection of bearing lines, re-weighted by range
// (cross-range error grows with distance). Rejects fewer than two lines,
// near-parallel geometry (smallest eigenvalue of sum(n n^T) below
// min_geometry_eigen) and fixes that fall behind an observer.
[[nodiscard]] bool triangulate(const BearingLine* lines, std::size_t count, double min_geometry_eigen,
                 TriangulationResult& out) noexcept;

}  // namespace amc8270::mesh
