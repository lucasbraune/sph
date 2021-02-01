#ifndef PRESSURE_FORCE_H
#define PRESSURE_FORCE_H

#include <memory>
#include <functional>
#include <unordered_map>
#include "particle_system.hpp"
#include "physics_elements.hpp"
#include "particle_filter.hpp"

namespace sph {

class SmoothingKernel {
  public:
  virtual double operator()(Vec2d x) const = 0;
  virtual Vec2d gradientAt(Vec2d x) const = 0;
  virtual double interactionRadius() const = 0;
  
  virtual std::unique_ptr<SmoothingKernel> clone() const = 0;
  virtual ~SmoothingKernel() {};
};

class PressureForce : public Force {
public:
  PressureForce(std::unique_ptr<ParticleFilter> filter,
                std::unique_ptr<SmoothingKernel> kernel,
                const std::function<double(double)>& pressure);
  PressureForce(double interactionRadius, std::function<double(double)> pressure);
  PressureForce(const PressureForce& other); 
  PressureForce(PressureForce&& other) = default;
  PressureForce& operator=(const PressureForce& other);
  PressureForce& operator=(PressureForce&& other) = default;
  ~PressureForce() = default;
  void apply(ParticleSystem& ps) const;

private:
  std::unordered_map<const Particle*, double> computeDensities(const ParticleSystem& ps) const;
  // synchronizeWith should NOT be const
  void synchronizeWith(const ParticleSystem& ps) const { filter_->syncWith(ps); }
  auto neighbors(const Particle& particle) const
  {
    return filter_->particlesIn(Disk{particle.pos, kernel_->interactionRadius()});
  }

  std::unique_ptr<SmoothingKernel> kernel_;
  std::unique_ptr<ParticleFilter> filter_;
  std::function<double(double)> pressure_;
};

class CubicKernel : public SmoothingKernel {
public:
  CubicKernel(double smoothingLength);
  double operator()(Vec2d x) const override;
  Vec2d gradientAt(Vec2d x) const override;
  double interactionRadius() const override;

  std::unique_ptr<SmoothingKernel> clone() const override;

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

} // end namespace sph 

#endif