#include "pressure_force.hpp"
#include <cmath> // M_PI

using std::vector;
using std::unique_ptr;
using std::function;

using namespace sph;

sph::CubicKernel::CubicKernel(double interactionRadius) :
  smoothingLength_{interactionRadius / 2},
  C_{15.0 / (14 * M_PI * smoothingLength_ * smoothingLength_)},
  D_{-3 * C_ / smoothingLength_}
{}

double sph::CubicKernel::operator()(Vec2d x) const
{
  double q = norm(x) / smoothingLength_;
  double A = 2 - q;
  if (q < 1) {
    double B = 1 - q;
    return C_ * (A*A*A - 4 * B*B*B);
  } else if (q < 2) {
    return C_ * A*A*A;
  } else {
    return 0.0;
  }
}

Vec2d sph::CubicKernel::gradientAt(Vec2d x) const
{
  double r = norm(x);
  double q = r / smoothingLength_;
  constexpr double EPSILON = 1e-6;
  if (q < EPSILON || q >= 2) {
    return Vec2d{};
  }
  double A = 2 - q;
  if (q < 1) {
    double B = 1 - q;
    return (D_ * (A*A - 4 * B*B) / r) * x;
  } else {
    // 1 <= q < 2
    return (D_ * A*A / r) * x;
  } 
}

double sph::CubicKernel::interactionRadius() const 
{
  return smoothingLength_ * 2;
}

sph::WaterPressure::WaterPressure(double pressureConstant, double restDensity) :
  pressureConstant_(pressureConstant),
  restDensity_(restDensity)
{}

double sph::WaterPressure::operator()(double density) const
{
  return pressureConstant_ * (pow((density / restDensity_), 7) - 1);
}

sph::GasPressure::GasPressure(double pressureConstant) :
  pressureConstant_(pressureConstant)
{}

double sph::GasPressure::operator()(double density) const
{
  return pressureConstant_ * density * density;
}