#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>
#include <random>

using Vec2d = std::array<double, 2>;
using std::vector;

struct Rectangle {
  const double xmin, ymin, xmax, ymax;
};

/**
 * A regular polygon inscribed in a circle.
 */
class RegularPolygon {
public:
  RegularPolygon(Vec2d center, double radius, size_t sides);
  RegularPolygon(double radius, size_t sides);
  const vector<Vec2d>& vertices() const;

private:
  vector<Vec2d> vertices_;
};

constexpr Vec2d ZERO_VECTOR{0.0, 0.0};

Vec2d operator+(const Vec2d& a, const Vec2d& b);
Vec2d operator-(const Vec2d& a, const Vec2d& b);
Vec2d operator*(const double lambda, const Vec2d& a);
Vec2d operator*(const Vec2d& a, const double lambda);
void operator+=(Vec2d& a, const Vec2d& b);
void operator-=(Vec2d& a, const Vec2d& b);
void operator*=(Vec2d& a, const double lambda);

vector<Vec2d> randomVectors(const Rectangle region, const size_t numberOfVectors);

#endif