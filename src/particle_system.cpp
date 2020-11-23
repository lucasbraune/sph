#include "particle_system.hpp"

ParticleSystem::ParticleSystem(const vector<Vec2d>& positions, const vector<Vec2d>& velocities,
    vector<Force*> forces, Damping* damping, const double m, const double dt) :
  numberOfParticles_(positions.size()),
  positions_(positions),
  velocities_(velocities),
  accelerations_(positions_.size()),
  forces_(forces),
  damping_(damping),
  particleMass_(m),
  time_(0.0),
  timeStep_(dt),
  nextForceAcc_(positions_.size())
{}

double ParticleSystem::time() const
{
  return time_;
}

const vector<Vec2d>& ParticleSystem::positions() const
{
  return positions_;
}

inline Vec2d nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc, const Damping& damping,
                   double mass, double time, double timeStep)
{
  Vec2d approxVel{currVel + timeStep * currAcc};
  Vec2d approxDampingAcc{damping.acceleration(time, mass, approxVel)};
  approxVel += (0.5 * timeStep) * (nextForceAcc + approxDampingAcc - currAcc);
  approxDampingAcc = damping.acceleration(time, mass, approxVel);
  return currVel + (0.5 * timeStep) * (currAcc + nextForceAcc + approxDampingAcc);
}

/**
 * Velocity verlet with damping, as described in [1].
 * 
 * [1] Anders W. Sandvik, "Numerical Solutions of Classical Equations of Motion" (2018). 
 */
void ParticleSystem::step(const double dt) {
  time_ += dt;
  for (size_t i=0; i<numberOfParticles_; i++) {
    positions_[i] += dt * velocities_[i] + (0.5 * dt * dt) * accelerations_[i];
  }

  for (Vec2d& acc : nextForceAcc_) {
    acc = ZERO_VECTOR;
  }
  for (auto force : forces_) {
    force->apply(time_, particleMass_, positions_, nextForceAcc_);
  }

  for (size_t i=0; i<numberOfParticles_; i++) {
    velocities_[i] = nextVelocity(velocities_[i], accelerations_[i], nextForceAcc_[i],
                                  *damping_, particleMass_, time_, dt);
    Vec2d nextDampingAcc = damping_->acceleration(time_, particleMass_, velocities_[i]);
    accelerations_[i] = nextForceAcc_[i] + nextDampingAcc;
  }
}

void ParticleSystem::integrate(double time)
{
  size_t steps = ceil(time / timeStep_);
  for (size_t i=0; i<steps; i++) {
    step(timeStep_);
  }
}

void ParticleSystem::addForce(Force* const force)
{
  forces_.emplace_back(force);
}

void ParticleSystem::replaceDamping(Damping* const damping)  
{
  damping_ = damping;
}