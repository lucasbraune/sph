#include "particle_system.hpp"
#include <cassert>

using std::vector;

namespace sph {

ParticleSystem::ParticleSystem(const vector<Vec2d>& initialPositions,
                               const vector<Vec2d>& initialVelocities,
                               const vector<Vec2d>& initialAccelerations,
                               double particleMass) :
  numberOfParticles{initialPositions.size()},
  particleMass{particleMass},
  positions{initialPositions},
  velocities{initialVelocities},
  accelerations{initialAccelerations}
{
  // TODO: handle error where the specified vectors have different lengths.
}

ParticleSystem::ParticleSystem(const vector<Vec2d>& initialPositions, double particleMass) :
  numberOfParticles{initialPositions.size()},
  particleMass{particleMass},
  positions{initialPositions},
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

void EulerIntegrator::step(ParticleSystem& ps, Physics& physics)
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

void VerletIntegrator::step(ParticleSystem& ps, Physics& physics)
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
  swap(currForceAcc, ps.accelerations); // ps.accelerations is reassigned below

  // Approximate velocity
  for (size_t i=0; i<ps.numberOfParticles; ++i) {
    ps.velocities[i] += timeStep_ * prevAcc[i];
  }
  for (size_t n=0; n<2; ++n) {
    // Improve velocity approximation
    ps.accelerations = currForceAcc;
    physics.applyDamping(ps);
    for (size_t i=0; i<ps.numberOfParticles; ++i) {
      ps.velocities[i] = prevVel[i] + 0.5 * timeStep_ * (prevAcc[i] + ps.accelerations[i]);
    }
  }
  
  ps.accelerations = currForceAcc;
  physics.applyDamping(ps);
}

} // end namespace sph