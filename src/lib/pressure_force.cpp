#include "pressure_force.hpp"
#include <cmath>

using std::vector;
using std::unique_ptr;
using std::function;

namespace sph {

PressureForce::PressureForce(
    unique_ptr<ParticleFilter> filter,
    unique_ptr<SmoothingKernel> kernel,
    const function<double(double)>& pressure) :
  kernel_(move(kernel)),
  filter_(move(filter)),
  pressure_(pressure)
{}

PressureForce::PressureForce(double interactionRadius, function<double(double)> pressure) :
  PressureForce(std::make_unique<TrivialFilter>(),
                std::make_unique<CubicKernel>(interactionRadius / 2),
                pressure)
{}

PressureForce::PressureForce(double interactionRadius, function<double(double)> pressure, 
                             const Rectangle& region) :
  PressureForce{std::make_unique<GridFilter>(makeGridFilter(region, interactionRadius)),
                std::make_unique<CubicKernel>(interactionRadius / 2),
                pressure}
{}

PressureForce::PressureForce(const PressureForce& other) :
  kernel_(other.kernel_->clone()),
  filter_(other.filter_->clone()),
  pressure_(other.pressure_)
{}

PressureForce& PressureForce::operator=(const PressureForce& other)
{
  filter_ = other.filter_->clone();
  kernel_ = other.kernel_->clone();
  pressure_ = other.pressure_;
  return *this;
}

void PressureForce::apply(ParticleSystem& ps)
{
  synchronizeWith(ps);
  const auto densities = computeDensities(ps);
  const auto quotient = [&pressure = std::as_const(pressure_), &densities](auto& particle) {
    auto density = densities.at(&particle);
    return pressure(density) / (density * density);
  };
  for (auto& particle: ps.particles) {
    const auto q = quotient(particle);
    auto sum = Vec2d{};
    for (auto& neighbor : neighbors(particle)) {
      if (&neighbor == &particle) continue; // OK to skip, assuming (1/r)(d kernel /dr)->0 as r->0
      sum += (q + quotient(neighbor)) * kernel_->gradientAt(particle.pos - neighbor.pos);
    }
    particle.acc -= ps.particleMass * sum;
  }
}

std::unordered_map<const Particle*, double>
PressureForce::computeDensities(const ParticleSystem& ps) const
{
  auto densities = std::unordered_map<const Particle*, double>{};
  for (const auto& particle : ps.particles) {
    auto sum = 0.0;
    for (auto& neighbor : neighbors(particle)) {
      sum += (*kernel_)(particle.pos - neighbor.pos);
    }
    densities[&particle] = ps.particleMass * sum;
  }
  return densities;
}

CubicKernel::CubicKernel(double smoothingLength) :
  smoothingLength_(smoothingLength),
  C_(15.0 / (14 * M_PI * smoothingLength * smoothingLength)),
  D_(-3 * C_ / smoothingLength_)
{}

double CubicKernel::operator()(Vec2d x) const
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

Vec2d CubicKernel::gradientAt(Vec2d x) const
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

double CubicKernel::interactionRadius() const 
{
  return smoothingLength_ * 2;
}

unique_ptr<SmoothingKernel> CubicKernel::clone() const
{
  return std::make_unique<CubicKernel>(*this);
}

WaterPressure::WaterPressure(double pressureConstant, double restDensity) :
  pressureConstant_(pressureConstant),
  restDensity_(restDensity)
{}

double WaterPressure::operator()(double density) const
{
  return pressureConstant_ * (pow((density / restDensity_), 7) - 1);
}

GasPressure::GasPressure(double pressureConstant) :
  pressureConstant_(pressureConstant)
{}

double GasPressure::operator()(double density) const
{
  return pressureConstant_ * density * density;
}

} // end namespace sph 