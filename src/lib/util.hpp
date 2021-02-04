#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>

namespace sph {

using Vec2d = std::array<double, 2>;

inline Vec2d& operator+=(Vec2d& a, const Vec2d& b)
{
  a[0] += b[0];
  a[1] += b[1];
  return a;
}
inline Vec2d& operator-=(Vec2d& a, const Vec2d& b)
{
  a[0] -= b[0];
  a[1] -= b[1];
  return a;
}
inline Vec2d& operator*=(Vec2d& a, const double lambda)
{
  a[0] *= lambda;
  a[1] *= lambda;
  return a;
}
inline Vec2d& operator/=(Vec2d& a, double lambda) { return a *= 1.0 / lambda; }

inline Vec2d operator+(Vec2d a, const Vec2d& b) { return a += b; }
inline Vec2d operator-(Vec2d a, const Vec2d& b) { return a -= b; }
inline Vec2d operator*(double lambda, Vec2d a) { return a *= lambda; }
inline Vec2d operator*(Vec2d a, double lambda) { return a *= lambda; }
inline Vec2d operator/(Vec2d a, double lambda) { return a /= lambda; }

inline double dotProduct(const Vec2d& a, const Vec2d& b) { return a[0] * b[0] + a[1] * b[1]; }
Vec2d project(const Vec2d& x, const Vec2d& normal);
double norm(const Vec2d& x);
inline double dist(const Vec2d& x, const Vec2d& y) { return norm(x - y); }
inline Vec2d unit(const Vec2d& x) { return x / norm(x); }

struct Rectangle final {
  Rectangle(double xmin = 0.0, double ymin = 0.0, double xmax = 1.0, double ymax = 1.0);
  const double xmin, ymin, xmax, ymax;
};

inline double width(const Rectangle& rect) { return rect.xmax - rect.xmin; }
inline double height(const Rectangle& rect) { return rect.ymax - rect.ymin; }
inline double area(const Rectangle& rect) { return height(rect) * width(rect); }

Vec2d randomVec2d(const Rectangle& rect = {0.0, 0.0, 1.0, 1.0});

struct Disk final {
  Disk(const Vec2d& center = {}, double radius = 1.0);
  const Vec2d center;
  const double radius;
};

template<typename T>
class Matrix final {
public:
  Matrix(size_t rows, size_t cols) : 
    rows_{rows}, cols_{cols}, data_(rows * cols)
  {
    assert(rows > 0 && cols > 0);
  }

  T& operator()(size_t i, size_t j)
  {
    assert(i < rows_ && j < cols_);
    return data_[i * rows_ + j];
  }
  
  const T& operator()(size_t i, size_t j) const
  {
    assert(i < rows_ && j < cols_);
    return data_[i * rows_ + j];
  }

  size_t rows() const { return rows_; }
  size_t columns() const { return cols_; }

private:
  const size_t rows_, cols_;
  std::vector<T> data_;
};

} // end namespace sph

#endif