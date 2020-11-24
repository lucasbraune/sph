#include "particle_system.hpp"

ParticleSystem::ParticleSystem(const vector<Vec2d>& positions, const vector<Vec2d>& velocities,
    vector<Force*> forces, Damping& damping, const double m) :
  numberOfParticles_(positions.size()),
  particleMass_(m),
  positions_(positions),
  velocities_(velocities),
  accelerations_(positions_.size()),
  forces_(forces),
  damping_(damping),
  time_(0.0)
{}

const vector<Vec2d>& ParticleSystem::positions() const
{
  return positions_;
}

double ParticleSystem::time() const
{
  return time_;
}

void TimeIntegrator::integrate(ParticleSystem& ps, double duration)
{
  double target = ps.time() + duration;
  while (ps.time() < target) {
    step(ps);
  }
}

EulerIntegrator::EulerIntegrator(double timeStep) : timeStep_(timeStep) {}

void EulerIntegrator::step(ParticleSystem& ps)
{
  ps.time_ += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles_; i++) {
    ps.positions_[i] += timeStep_ * ps.velocities_[i];
    ps.velocities_[i] += timeStep_ * ps.accelerations_[i];
    ps.accelerations_[i] = ps.damping_.acceleration(ps.time_, ps.particleMass_, ps.velocities_[i]);
  }
  for (auto force : ps.forces_) {
    force->apply(ps.time_, ps.particleMass_, ps.positions_, ps.accelerations_);
  }
}

VerletIntegrator::VerletIntegrator(double timeStep) : timeStep_(timeStep) {}

void VerletIntegrator::step(ParticleSystem& ps)
{
  ps.time_ += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles_; i++) {
    ps.positions_[i] += timeStep_ * ps.velocities_[i] +
                        (0.5 * timeStep_ * timeStep_) * ps.accelerations_[i];
  }

  nextForceAcc_.resize(ps.numberOfParticles_);
  for (Vec2d& acc : nextForceAcc_) {
    acc = ZERO_VECTOR;
  }
  for (auto force : ps.forces_) {
    force->apply(ps.time_, ps.particleMass_, ps.positions_, nextForceAcc_);
  }

  for (size_t i=0; i<ps.numberOfParticles_; i++) {
    ps.velocities_[i] = nextVelocity(ps.velocities_[i], ps.accelerations_[i], nextForceAcc_[i],
                                  ps.damping_, ps.particleMass_, ps.time_, timeStep_);
    Vec2d nextDampingAcc = ps.damping_.acceleration(ps.time_, ps.particleMass_, ps.velocities_[i]);
    ps.accelerations_[i] = nextForceAcc_[i] + nextDampingAcc;
  }
}

inline Vec2d VerletIntegrator::nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc,
                                            const Damping& damping, double mass, double time,
                                            double timeStep)
{
  Vec2d approxVel{currVel + timeStep * currAcc};
  Vec2d approxDampingAcc{damping.acceleration(time, mass, approxVel)};
  approxVel += (0.5 * timeStep) * (nextForceAcc + approxDampingAcc - currAcc);
  approxDampingAcc = damping.acceleration(time, mass, approxVel);
  return currVel + (0.5 * timeStep) * (currAcc + nextForceAcc + approxDampingAcc);
}