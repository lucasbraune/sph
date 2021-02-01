#include "particle_system.hpp"
#include "range/v3/algorithm/copy.hpp"
#include "range/v3/algorithm/fill.hpp"
#include "range/v3/view/zip.hpp"

using namespace sph;

sph::ParticleSystem::ParticleSystem(const std::vector<Particle>& particles, double totalMass) :
  particles{particles},
  particleMass{totalMass / particles.size()}
{
  assert(particles.size() > 0);
}

void sph::TimeIntegrator::integrate(ParticleSystem& ps, Physics& physics, double duration)
{
  double target = ps.time + duration;
  while (ps.time < target) {
    step(ps, physics);
  }
}

void sph::Euler::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (auto& particle : ps.particles) {
    particle.pos += timeStep_ * particle.vel;
    particle.vel += timeStep_ * particle.acc;
    particle.acc = Vec2d{};
  }
  physics.applyForces(ps);
  physics.applyDamping(ps);
}

void sph::Verlet::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (auto& particle : ps.particles) {
    particle.pos += timeStep_ * particle.vel + (0.5 * timeStep_ * timeStep_) * particle.acc;
  }
  physics.resolveCollisions(ps);

  static std::vector<Vec2d> prevVels, prevAccs, currForceAccs;
  prevVels.resize(ps.particles.size());
  prevAccs.resize(ps.particles.size());
  currForceAccs.resize(ps.particles.size());

  using namespace ranges;
  copy(velocities(ps), prevVels.begin());
  copy(accelerations(ps), prevAccs.begin());
  fill(accelerations(ps), Vec2d{});
  physics.applyForces(ps);
  copy(accelerations(ps), currForceAccs.begin());
  // Approximate velocities
  for (auto [particle, prevAcc] : views::zip(ps.particles, prevAccs)) {
    particle.vel += timeStep_ * prevAcc;
  }
  physics.applyDamping(ps);
  for (size_t n=0; n<2; ++n) {
    // Improve velocity approximations
    for (auto [particle, prevVel, prevAcc] : views::zip(ps.particles, prevVels, prevAccs)) {
      particle.vel = prevVel + (0.5 * timeStep_) * (prevAcc + particle.acc);
    }
    // Recompute accelerations
    copy(currForceAccs, accelerations(ps).begin());
    physics.applyDamping(ps);
  }
}