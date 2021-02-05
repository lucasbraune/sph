#ifndef PRESSURE_FORCE_H
#define PRESSURE_FORCE_H

#include "particle_system.hpp"
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
  double operator()(Vec2d x) const override;
  Vec2d gradientAt(Vec2d x) const override;
  double interactionRadius() const override;

private:
  const double smoothingLength_;
  const double C_, D_;
};

class WaterPressure final {
public:
  WaterPressure(double pressureConstant, double restDensity);
  double operator()(double density) const;

private:
  const double pressureConstant_;
  const double restDensity_;
};

class GasPressure final {
public:
  GasPressure(double pressureConstant);
  double operator()(double density) const;

private:
  const double pressureConstant_;
};

template<class PressureFn,
         class SummationStrategy = GridBasedSummation,
         class KernelFn = CubicKernel>
class PressureForce final : public Force {
  static_assert(std::is_convertible_v<PressureFn, std::function<double(double)>>);
  static_assert(std::is_base_of_v<SmoothingKernel, KernelFn>);

public:
  PressureForce(const PressureFn& pressure, const KernelFn& kernel,
                const SummationStrategy& summationStrategy) :
    summationStrategy_{summationStrategy}, kernel_{kernel}, pressure_{pressure} {}

  void apply(ParticleSystem& ps) override
  {
    summationStrategy_.syncWith(ps);
    updateDensities(ps);
    for (auto& particle : ps.particles) {
      auto quotient1 = pressure_(particle.density) / (particle.density * particle.density);
      auto summand = [&](auto& neighbor) {
        auto quotient2 = pressure_(neighbor.density) / (neighbor.density * neighbor.density);
        return (quotient1 + quotient2) * kernel_.gradientAt(particle.pos - neighbor.pos);
      };
      particle.acc -= ps.particleMass *
                      summationStrategy_.sumOverParticles(summand, neighborhood(particle), ps);
    }
  }

private:
  void updateDensities(ParticleSystem& ps) const
  {
    for (auto& particle : ps.particles) {
      auto summand = [&](auto& neighbor) { 
        return kernel_(particle.pos - neighbor.pos);
      };
      particle.density = ps.particleMass *
                         summationStrategy_.sumOverParticles(summand, neighborhood(particle), ps);
    }
  }

  Disk neighborhood(const Particle& particle) const
  {
    return Disk{particle.pos, kernel_.interactionRadius()};
  }

  SummationStrategy summationStrategy_;
  KernelFn kernel_;
  PressureFn pressure_;
};

template<class PressureFn>
auto makePressureForce(PressureFn&& pressure, double interactionRadius)
{
  return PressureForce<PressureFn, TrivialSummation>{
    pressure, CubicKernel{interactionRadius}, TrivialSummation{}
  };
}

template<class PressureFn>
auto makePressureForce(PressureFn&& pressure, double interactionRadius, const Rectangle& region)
{
  return PressureForce<PressureFn>{
    pressure, CubicKernel{interactionRadius},
    GridBasedSummation{region, interactionRadius}
  };
}

} // end namespace sph 

#endif