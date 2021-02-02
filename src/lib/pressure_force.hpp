#ifndef PRESSURE_FORCE_H
#define PRESSURE_FORCE_H

#include <unordered_map>
#include "particle_system.hpp"
#include "physics_elements.hpp"
#include "loop_strategy.hpp"

namespace sph {

class SmoothingKernel {
public:
  virtual ~SmoothingKernel() {};
  virtual double operator()(Vec2d x) const = 0;
  virtual Vec2d gradientAt(Vec2d x) const = 0;
  virtual double interactionRadius() const = 0;
};

class CubicKernel final : public SmoothingKernel {
public:
  CubicKernel(double interactionRadius);
  double operator()(Vec2d x) const final;
  Vec2d gradientAt(Vec2d x) const final;
  double interactionRadius() const final;

private:
  const double smoothingLength_;
  const double C_, D_;
};

class WaterPressure {
public:
  WaterPressure(double pressureConstant, double restDensity);
  double operator()(double density) const;

private:
  const double pressureConstant_;
  const double restDensity_;
};

class GasPressure {
public:
  GasPressure(double pressureConstant);
  double operator()(double density) const;

private:
  const double pressureConstant_;
};

template<class PressureFn,
         class NeighborLoopStrategy = GridBasedLoopStrategy,
         class KernelFn = CubicKernel>
class PressureForce : public Force {
  static_assert(std::is_base_of_v<SmoothingKernel, KernelFn>);

public:
  PressureForce(const PressureFn& pressure, const KernelFn& kernel,
                const NeighborLoopStrategy& loopStrategy) :
    loopStategy_{loopStrategy}, kernel_{kernel}, pressure_{pressure} {}

  void apply(ParticleSystem& ps)
  {
    loopStategy_.syncWith(ps);
    const auto densities = computeDensities(ps);
    for (auto& particle : ps.particles) {
      auto density1 = densities.at(&particle);
      auto quotient1 = pressure_(density1) / (density1 * density1);
      auto summand = [&](auto& neighbor) {
        auto density2 = densities.at(&neighbor);
        auto quotient2 = pressure_(density2) / (density2 * density2);
        return (quotient1 + quotient2) * kernel_.gradientAt(particle.pos - neighbor.pos);
      };
      particle.acc -= ps.particleMass *
                      loopStategy_.accumulate(summand, ps, neighborhood(particle));
    }
  }

private:
  std::unordered_map<const Particle*, double> computeDensities(const ParticleSystem& ps) const
  {
    auto densities = std::unordered_map<const Particle*, double>{};
    for (const auto& particle : ps.particles) {
      auto summand = [&](auto& neighbor) { 
        return kernel_(particle.pos - neighbor.pos);
      };
      densities[&particle] = ps.particleMass *
                             loopStategy_.accumulate(summand, ps, neighborhood(particle));
    }
    return densities;
  }

  Disk neighborhood(const Particle& particle) const
  {
    return Disk{particle.pos, kernel_.interactionRadius()};
  }

  NeighborLoopStrategy loopStategy_;
  KernelFn kernel_;
  PressureFn pressure_;
};

template<class PressureFn>
auto makePressureForce(PressureFn&& pressure, double interactionRadius)
{
  return PressureForce<PressureFn, TrivialLoopStrategy>{
    pressure, CubicKernel{interactionRadius}, TrivialLoopStrategy{}
  };
}

template<class PressureFn>
auto makePressureForce(PressureFn&& pressure, double interactionRadius, const Rectangle& region)
{
  return PressureForce<PressureFn>{
    pressure, CubicKernel{interactionRadius},
    makeGridBasedLoopStrategy(region, interactionRadius)
  };
}

} // end namespace sph 

#endif