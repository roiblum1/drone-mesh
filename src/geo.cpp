#include "amc8270/mesh/geo.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "amc8270/mesh/config.hpp"

namespace amc8270::mesh {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kEarthRadiusM = 6371008.8;
constexpr double kMinRangeM = 200.0;  // floor on range used for weighting
}  // namespace

double wrap360(double deg) noexcept {
  double d = std::fmod(deg, 360.0);
  if (d < 0.0) d += 360.0;
  return d;
}

double angleDiffDeg(double a, double b) noexcept {
  const double d = std::fabs(wrap360(a) - wrap360(b));
  return d > 180.0 ? 360.0 - d : d;
}

double distanceM(const GeoPoint& a, const GeoPoint& b) noexcept {
  const double p1 = a.lat_deg * kDeg2Rad;
  const double p2 = b.lat_deg * kDeg2Rad;
  const double dp = p2 - p1;
  const double dl = (b.lon_deg - a.lon_deg) * kDeg2Rad;
  const double s1 = std::sin(dp / 2.0);
  const double s2 = std::sin(dl / 2.0);
  const double h = s1 * s1 + std::cos(p1) * std::cos(p2) * s2 * s2;
  return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

double bearingDeg(const GeoPoint& from, const GeoPoint& to) noexcept {
  const double p1 = from.lat_deg * kDeg2Rad;
  const double p2 = to.lat_deg * kDeg2Rad;
  const double dl = (to.lon_deg - from.lon_deg) * kDeg2Rad;
  const double y = std::sin(dl) * std::cos(p2);
  const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
  return wrap360(std::atan2(y, x) * kRad2Deg);
}

GeoPoint destinationPoint(const GeoPoint& from, double bearing_deg, double distance_m) noexcept {
  const double ang = distance_m / kEarthRadiusM;
  const double brg = bearing_deg * kDeg2Rad;
  const double p1 = from.lat_deg * kDeg2Rad;
  const double l1 = from.lon_deg * kDeg2Rad;
  const double p2 = std::asin(std::sin(p1) * std::cos(ang) + std::cos(p1) * std::sin(ang) * std::cos(brg));
  const double l2 = l1 + std::atan2(std::sin(brg) * std::sin(ang) * std::cos(p1),
                                    std::cos(ang) - std::sin(p1) * std::sin(p2));
  GeoPoint out;
  out.lat_deg = p2 * kRad2Deg;
  out.lon_deg = std::fmod(l2 * kRad2Deg + 540.0, 360.0) - 180.0;
  out.alt_m = from.alt_m;
  return out;
}

LocalFrame::LocalFrame(const GeoPoint& origin) noexcept
    : origin_(origin),
      m_per_deg_lat_(kEarthRadiusM * kDeg2Rad),
      m_per_deg_lon_(kEarthRadiusM * kDeg2Rad * std::cos(origin.lat_deg * kDeg2Rad)) {}

Vec2 LocalFrame::toEnu(const GeoPoint& p) const noexcept {
  Vec2 v;
  v.e = (p.lon_deg - origin_.lon_deg) * m_per_deg_lon_;
  v.n = (p.lat_deg - origin_.lat_deg) * m_per_deg_lat_;
  return v;
}

GeoPoint LocalFrame::fromEnu(const Vec2& v, float alt_m) const noexcept {
  GeoPoint p;
  p.lat_deg = origin_.lat_deg + v.n / m_per_deg_lat_;
  p.lon_deg = origin_.lon_deg + v.e / m_per_deg_lon_;
  p.alt_m = alt_m;
  return p;
}

bool triangulate(const BearingLine* lines, std::size_t count, double min_geometry_eigen,
                 TriangulationResult& out) noexcept {
  out = TriangulationResult{};
  if (lines == nullptr || count < 2 || count > kMaxObsPerEvent) return false;

  // Line i: points x with n_i . (x - o_i) = 0, n_i = (cos b, -sin b) (normal to the
  // direction (sin b, cos b)).
  std::array<double, kMaxObsPerEvent> nx{};
  std::array<double, kMaxObsPerEvent> ny{};
  double g11 = 0.0, g12 = 0.0, g22 = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    nx[i] = std::cos(lines[i].bearing_rad);
    ny[i] = -std::sin(lines[i].bearing_rad);
    g11 += nx[i] * nx[i];
    g12 += nx[i] * ny[i];
    g22 += ny[i] * ny[i];
  }
  const double lam_min_geom =
      0.5 * ((g11 + g22) - std::sqrt((g11 - g22) * (g11 - g22) + 4.0 * g12 * g12));
  if (!(lam_min_geom >= min_geometry_eigen)) return false;

  std::array<double, kMaxObsPerEvent> range{};
  range.fill(1.0);  // first pass: weight by angular sigma only
  Vec2 x{};
  double a11 = 0.0, a12 = 0.0, a22 = 0.0;
  for (int pass = 0; pass < 3; ++pass) {
    a11 = a12 = a22 = 0.0;
    double b1 = 0.0, b2 = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
      const double sig = std::max(lines[i].sigma_rad, 1e-4) * range[i];
      const double w = 1.0 / (sig * sig);
      const double c = nx[i] * lines[i].origin.e + ny[i] * lines[i].origin.n;
      a11 += w * nx[i] * nx[i];
      a12 += w * nx[i] * ny[i];
      a22 += w * ny[i] * ny[i];
      b1 += w * nx[i] * c;
      b2 += w * ny[i] * c;
    }
    const double det = a11 * a22 - a12 * a12;
    if (!(det > 0.0)) return false;
    x.e = (a22 * b1 - a12 * b2) / det;
    x.n = (a11 * b2 - a12 * b1) / det;
    for (std::size_t i = 0; i < count; ++i) {
      const double de = x.e - lines[i].origin.e;
      const double dn = x.n - lines[i].origin.n;
      range[i] = std::max(std::sqrt(de * de + dn * dn), kMinRangeM);
    }
  }

  // Fix must lie ahead of every observer.
  for (std::size_t i = 0; i < count; ++i) {
    const double de = x.e - lines[i].origin.e;
    const double dn = x.n - lines[i].origin.n;
    const double ahead = de * std::sin(lines[i].bearing_rad) + dn * std::cos(lines[i].bearing_rad);
    if (ahead < -50.0) return false;
  }

  // Covariance = inverse of the information matrix; CEP ~ 0.589 (s1 + s2).
  const double det = a11 * a22 - a12 * a12;
  const double c11 = a22 / det;
  const double c22 = a11 / det;
  const double c12 = -a12 / det;
  const double mean = 0.5 * (c11 + c22);
  const double disc = std::sqrt(std::max(0.0, 0.25 * (c11 - c22) * (c11 - c22) + c12 * c12));
  const double l1 = std::max(0.0, mean + disc);
  const double l2 = std::max(0.0, mean - disc);
  const double cep = 0.589 * (std::sqrt(l1) + std::sqrt(l2));
  if (!std::isfinite(x.e) || !std::isfinite(x.n) || !std::isfinite(cep)) return false;

  out.ok = true;
  out.position = x;
  out.cep_m = cep;
  return true;
}

}  // namespace amc8270::mesh
