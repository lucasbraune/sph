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

Vec2d operator+(const Vec2d& a, const Vec2d& b);
Vec2d operator-(const Vec2d& a, const Vec2d& b);
Vec2d operator*(const double lambda, const Vec2d& a);
void operator+=(Vec2d& a, const Vec2d& b);
void operator-=(Vec2d& a, const Vec2d& b);
void operator*=(Vec2d& a, const double lambda);

vector<Vec2d> randomPositions(const Rectangle region, const size_t N);

#endif