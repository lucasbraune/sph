#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include "util.hpp"
#include "range/v3/view/transform.hpp"

namespace sph {

struct Particle final {
  Vec2d pos, vel, acc;
};

struct ParticleSystem final {
  ParticleSystem(const std::vector<Particle>& particles, double totalMass);
  std::vector<Particle> particles;
  double particleMass;
  double time;
};

template<class ParticleSystemType>
auto positions(ParticleSystemType& ps)
{
  constexpr auto pos = [](auto& particle)->auto&{ return particle.pos; };
  return ps.particles | ranges::views::transform(pos);
}

template<class ParticleSystemType>
auto velocities(ParticleSystemType& ps)
{
  constexpr auto vel = [](auto& particle)->auto&{ return particle.vel; };
  return ps.particles | ranges::views::transform(vel);
}

template<class ParticleSystemType>
auto accelerations(ParticleSystemType& ps)
{
  constexpr auto acc = [](auto& particle)->auto&{ return particle.acc; };
  return ps.particles | ranges::views::transform(acc);
}

struct Physics {
  virtual ~Physics() {}
  // Adds the effect of velocity-independent forces to the accelerations of particles
  virtual void applyForces(ParticleSystem& ps) const = 0;
  // Adds the effect of damping to the accelerations of particles
  virtual void applyDamping(ParticleSystem& ps) const = 0;
  // Moves particles to resolve collisions
  virtual void resolveCollisions(ParticleSystem& ps) const = 0;
};

class TimeIntegrator {
public:
  virtual ~TimeIntegrator() {};
  void integrate(ParticleSystem& ps, Physics& physics, double duration);

private:
  virtual void step(ParticleSystem& ps, Physics& physics) = 0;
};

class Euler : public TimeIntegrator {
public:
  Euler(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

class Verlet : public TimeIntegrator {
public:
  Verlet(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

} // end namespace sph

#endif