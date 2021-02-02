#include "particle_system.hpp"

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

namespace {

void copyVelocities(const ParticleSystem& ps, std::vector<Vec2d>& out)
{
  out.clear();
  for (auto& particle : ps.particles) {
    out.emplace_back(particle.vel);
  }
}

void copyAccelerations(const ParticleSystem& ps, std::vector<Vec2d>& out)
{
  out.clear();
  for (auto& particle : ps.particles) {
    out.emplace_back(particle.acc);
  }
}

void copyIntoAccelerations(const std::vector<Vec2d>& in, ParticleSystem& ps)
{
  assert(in.size() == ps.particles.size());
  for (size_t i=0; i<ps.particles.size(); ++i) {
    ps.particles[i].acc = in[i];
  }
}

void fillAccelerations(ParticleSystem& ps, const Vec2d& val)
{
  for (auto& particle : ps.particles) {
    particle.acc = val;
  }
}

} // namespace

void sph::Verlet::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (auto& particle : ps.particles) {
    particle.pos += timeStep_ * particle.vel + (0.5 * timeStep_ * timeStep_) * particle.acc;
  }
  physics.resolveCollisions(ps);

  static std::vector<Vec2d> prevVels, prevAccs, currForceAccs;
  copyVelocities(ps, prevVels);
  copyAccelerations(ps, prevAccs);
  fillAccelerations(ps, Vec2d{});
  physics.applyForces(ps);
  copyAccelerations(ps, currForceAccs);
  // Approximate velocities
  
  for (size_t i=0; i<ps.particles.size(); ++i) {
    auto& particle = ps.particles[i];
    auto& prevAcc = prevAccs[i];
    particle.vel += timeStep_ * prevAcc;
  }
  physics.applyDamping(ps);
  for (size_t n=0; n<2; ++n) {
    // Improve velocity approximations
    for (size_t i=0; i<ps.particles.size(); ++i) {
      auto& particle = ps.particles[i];
      auto& prevVel = prevVels[i];
      auto& prevAcc = prevAccs[i];
      particle.vel = prevVel + (0.5 * timeStep_) * (prevAcc + particle.acc);
    }
    // Recompute accelerations
    copyIntoAccelerations(currForceAccs, ps);
    physics.applyDamping(ps);
  }
}