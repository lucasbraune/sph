#include "particle_system.hpp"
#include <cassert>

using std::vector;

namespace sph {

void Force::apply(ParticleSystem& ps) const {
  apply(ps, ps.accelerations);
}

void Force::apply(const ParticleSystem& ps, std::vector<Vec2d>& accelerations) const
{
  apply(ps.positions, ps.particleMass, ps.time, accelerations); 
}

void Damping::apply(ParticleSystem& ps) const {
  apply(ps, ps.accelerations);
}

void Damping::apply(const ParticleSystem& ps, std::vector<Vec2d>& accelerations) const
{
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    accelerations[i] += acceleration(ps.velocities[i], ps.particleMass);
  }
}

void Collidable::resolveCollisions(ParticleSystem& ps) const
{
  resolveCollisions(ps.positions, ps.velocities, ps.time);
}

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

template<typename Container, typename Value>
void setAll(Container& cont, const Value& val)
{
  for (auto& x : cont) {
    x = val;
  }
}

void EulerIntegrator::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i];
    ps.velocities[i] += timeStep_ * ps.accelerations[i];
  }
  for (auto collidablePtr : physics.collidablePtrs()) {
    collidablePtr->resolveCollisions(ps);
  }
  setAll(ps.accelerations, ZERO_VECTOR);
  for (auto forcePtr : physics.forcePtrs()) {
    forcePtr->apply(ps);
  }
  if (physics.dampingPtr()) {
    physics.dampingPtr()->apply(ps);
  }
}

static Vec2d nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc,
                          const Damping& damping, double mass, double timeStep)
{
  auto approxVel = currVel + timeStep * currAcc;
  approxVel += (0.5 * timeStep) * (nextForceAcc + damping.acceleration(approxVel, mass) - currAcc);
  return currVel + (0.5 * timeStep) *
      (currAcc + nextForceAcc + damping.acceleration(approxVel, mass));
}

void VerletIntegrator::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i] +
                       (0.5 * timeStep_ * timeStep_) * ps.accelerations[i];
  }
  for (auto collidablePtr : physics.collidablePtrs()) {
    collidablePtr->resolveCollisions(ps);
  }
  
  if (!physics.dampingPtr()) {
    // TODO: Give the user of this class the option to skip this test at compile time.
    for (size_t i=0; i<ps.numberOfParticles; i++) {
      ps.velocities[i] += 0.5 * timeStep_ * ps.accelerations[i];
    }
    setAll(ps.accelerations, ZERO_VECTOR);
    for (auto forcePtr : physics.forcePtrs()) {
      forcePtr->apply(ps);
    }
    for (size_t i=0; i<ps.numberOfParticles; i++) {
      ps.velocities[i] += 0.5 * timeStep_ * ps.accelerations[i];
    }
  } else {
    static auto nextForceAcc = std::vector<Vec2d>(ps.numberOfParticles);
    nextForceAcc.resize(ps.numberOfParticles); // does not decrease capacity
    setAll(nextForceAcc, ZERO_VECTOR);
    for (auto forcePtr : physics.forcePtrs()) {
      forcePtr->apply(ps, nextForceAcc);
    }
    for (size_t i=0; i<ps.numberOfParticles; i++) {
      ps.velocities[i] = nextVelocity(ps.velocities[i], ps.accelerations[i], nextForceAcc[i],
                                      *physics.dampingPtr(), ps.particleMass, timeStep_);
      ps.accelerations[i] = nextForceAcc[i] +
                            physics.dampingPtr()->acceleration(ps.velocities[i], ps.particleMass);
    }
  }
}

} // end namespace sph