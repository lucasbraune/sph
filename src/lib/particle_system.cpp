#include "particle_system.hpp"
#include <cassert>

using std::vector;

namespace sph {

ParticleSystem::ParticleSystem(const vector<Vec2d>& pos, const vector<Vec2d>& vel,
                               const vector<Vec2d>& acc, double mass) :
  numberOfParticles{pos.size()},
  particleMass{mass},
  positions{pos},
  velocities{vel},
  accelerations{acc}
{
  // TODO: handle error where the specified vectors have different lengths.
}

ParticleSystem::ParticleSystem(const vector<Vec2d>& pos, double mass) :
  numberOfParticles{pos.size()},
  particleMass{mass},
  positions{pos},
  velocities(numberOfParticles),    // parentheses, not braces!
  accelerations(numberOfParticles)  // same!
{}

void TimeIntegrator::integrate(ParticleSystem& ps, Physics& physics, double duration)
{
  double target = ps.time + duration;
  while (ps.time < target) {
    step(ps, physics);
  }
}

static void fillWithZeros(std::vector<Vec2d>& v) {
  std::fill(v.begin(), v.end(), ZERO_VECTOR);
}

void Euler::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i];
    ps.velocities[i] += timeStep_ * ps.accelerations[i];
  }
  physics.resolveCollisions(ps);
  fillWithZeros(ps.accelerations);
  physics.applyForces(ps);
  physics.applyDamping(ps);
}

void Verlet::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i] +
                       (0.5 * timeStep_ * timeStep_) * ps.accelerations[i];
  }
  physics.resolveCollisions(ps);

  static std::vector<Vec2d> prevVel, prevAcc, currForceAcc;
  prevVel = ps.velocities;
  prevAcc = ps.accelerations;
  fillWithZeros(ps.accelerations);
  physics.applyForces(ps);
  currForceAcc = ps.accelerations;
  // Approximate velocities
  for (size_t i=0; i<ps.numberOfParticles; ++i) {
    ps.velocities[i] += timeStep_ * prevAcc[i];
  }
  physics.applyDamping(ps);
  for (size_t n=0; n<2; ++n) {
    // Improve velocity approximations
    for (size_t i=0; i<ps.numberOfParticles; ++i) {
      ps.velocities[i] = prevVel[i] + 0.5 * timeStep_ * (prevAcc[i] + ps.accelerations[i]);
    }
    // Recompute accelerations
    ps.accelerations = currForceAcc;
    physics.applyDamping(ps);
  }
}

} // end namespace sph