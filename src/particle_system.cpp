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
  forceAccelerations_(positions_.size())
{}

double ParticleSystem::time() const
{
  return time_;
}

const vector<Vec2d>& ParticleSystem::positions() const
{
  return positions_;
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

  for (Vec2d& acc : forceAccelerations_) {
    acc = ZERO_VECTOR;
  }
  for (auto force : forces_) {
    force->apply(time_, particleMass_, positions_, forceAccelerations_);
  }

  for (size_t i=0; i<numberOfParticles_; i++) {
    Vec2d approxVel{velocities_[i] + dt * accelerations_[i]};
    Vec2d dampingAcc{damping_->acceleration(time_, particleMass_, approxVel)};
    approxVel += (0.5 * dt) * (forceAccelerations_[i] + dampingAcc - accelerations_[i]);
    dampingAcc = damping_->acceleration(time_, particleMass_, approxVel);
    velocities_[i] += (0.5 * dt) * (accelerations_[i] + forceAccelerations_[i] + dampingAcc);

    dampingAcc = damping_->acceleration(time_, particleMass_, velocities_[i]);
    accelerations_[i] = forceAccelerations_[i] + dampingAcc;
  }
}

void ParticleSystem::updateAccelerations()
{
  for (Vec2d& acc : accelerations_) {
    acc = ZERO_VECTOR;
  }
  for (auto force : forces_) {
    force->apply(time_, particleMass_, positions_, accelerations_);
  }
  for (size_t i=0; i<numberOfParticles_; i++) {
    accelerations_[i] += damping_->acceleration(time_, particleMass_, velocities_[i]);
  }
}

void ParticleSystem::eulerStep(const double dt)
{
  updateAccelerations();
  for (size_t i=0; i<numberOfParticles_; i++) {
    positions_[i] += dt * velocities_[i];
    velocities_[i] += dt * accelerations_[i];
  }
  time_ += dt;
}

void ParticleSystem::integrate(double time)
{
  size_t steps = ceil(time / timeStep_);
  for (size_t i=0; i<steps; i++) {
    step(timeStep_);
  }
}