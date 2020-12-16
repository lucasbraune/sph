#include "particle_system.hpp"

using std::vector;

namespace sph {

Vec2d Damping::acceleration(const vector<const Damping*>& dampings, double time, double mass,
                            const Vec2d& velocity)
{
  Vec2d result = ZERO_VECTOR;
  for (auto damping : dampings) {
    result += damping->acceleration(time, mass, velocity);
  }
  return result;
}

ParticleSystem::ParticleSystem(const vector<Vec2d>& initialPositions,
                               const vector<Vec2d>& initialVelocities,
                               double particleMass) :
  numberOfParticles(initialPositions.size()),
  particleMass(particleMass),
  positions(initialPositions),
  velocities(initialVelocities),
  accelerations(positions.size()),
  time(0.0)
{}

ParticleSystem::ParticleSystem(size_t numberOfParticles, double totalMass, Rectangle region) :
  ParticleSystem(randomVectors(region, numberOfParticles),
                 vector<Vec2d>(numberOfParticles),
                 totalMass / numberOfParticles)
{}

void TimeIntegrator::integrate(ParticleSystem& ps, Physics& physics, double duration)
{
  double target = ps.time + duration;
  while (ps.time < target) {
    step(ps, physics);
  }
}

void EulerIntegrator::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i];
  }
  for (auto collidablePtr : physics.collidablePtrs()) {
    collidablePtr->resolveCollisions(ps.positions, ps.velocities, ps.time);
  }
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.velocities[i] += timeStep_ * ps.accelerations[i];
    ps.accelerations[i] =
        Damping::acceleration(physics.dampingPtrs(), ps.time, ps.particleMass, ps.velocities[i]);
  }
  for (auto force : physics.forcePtrs()) {
    force->apply(ps.time, ps.particleMass, ps.positions, ps.accelerations);
  }
}

void VerletIntegrator::step(ParticleSystem& ps, Physics& physics)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i] +
                       (0.5 * timeStep_ * timeStep_) * ps.accelerations[i];
  }
  for (auto collidablePtr : physics.collidablePtrs()) {
    collidablePtr->resolveCollisions(ps.positions, ps.velocities, ps.time);
  }
  nextForceAcc_.resize(ps.numberOfParticles);
  for (Vec2d& acc : nextForceAcc_) {
    acc = ZERO_VECTOR;
  }
  for (auto& force : physics.forcePtrs()) {
    force->apply(ps.time, ps.particleMass, ps.positions, nextForceAcc_);
  }

  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.velocities[i] = nextVelocity(ps.velocities[i], ps.accelerations[i], nextForceAcc_[i],
                                    physics.dampingPtrs(), ps.particleMass, ps.time, timeStep_);
    Vec2d nextDampingAcc =
        Damping::acceleration(physics.dampingPtrs(), ps.time, ps.particleMass, ps.velocities[i]);
    ps.accelerations[i] = nextForceAcc_[i] + nextDampingAcc;
  }
}

inline Vec2d VerletIntegrator::nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc,
                                            const vector<const Damping*>& dampings,
                                            double mass, double time, double timeStep)
{
  Vec2d approxVel = currVel + timeStep * currAcc;
  Vec2d approxDampingAcc = Damping::acceleration(dampings, time, mass, approxVel);
  approxVel += (0.5 * timeStep) * (nextForceAcc + approxDampingAcc - currAcc);
  approxDampingAcc = Damping::acceleration(dampings, time, mass, approxVel);
  return currVel + (0.5 * timeStep) * (currAcc + nextForceAcc + approxDampingAcc);
}

} // end namespace sph