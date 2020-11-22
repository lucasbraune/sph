#include "particle_system.hpp"

ParticleSystem::ParticleSystem(const vector<Vec2d>& positions, const vector<Vec2d>& velocities,
    vector<Force*> forces, vector<Damping*> dampings, const double m, const double dt) :
  numberOfParticles_(positions.size()),
  positions_(positions),
  velocities_(velocities),
  accelerations_(positions_.size()),
  forces_(forces),
  dampings_(dampings),
  particleMass_(m),
  time_(0.0),
  timeStep_(dt),
  approxVelocities_(positions_.size())
{}

double ParticleSystem::time() const
{
  return time_;
}

const vector<Vec2d>& ParticleSystem::positions() const
{
  return positions_;
}

vector<Vec2d> ParticleSystem::forceAccelerations() const {
  vector<Vec2d> forceAccelerations(numberOfParticles_);
  for (auto force : forces_) {
    force->apply(time_, particleMass_, positions_, forceAccelerations);
  }
  return forceAccelerations;
}

vector<Vec2d> ParticleSystem::dampingAccelerations(const vector<Vec2d>& velocities) const {
  vector<Vec2d> dampingAccelerations(numberOfParticles_);
  for (auto damping : dampings_) {
    damping->apply(time_, particleMass_, velocities, dampingAccelerations);
  }
  return dampingAccelerations;
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

  forceAccelerations_ = forceAccelerations();
  for (size_t i=0; i<numberOfParticles_; i++) {
    approxVelocities_[i] = velocities_[i] + dt * accelerations_[i];
  }
  dampingAccelerations_ = dampingAccelerations(approxVelocities_);
  for (size_t i=0; i<numberOfParticles_; i++) {
    approxVelocities_[i] += (0.5 * dt) *
        (forceAccelerations_[i] + dampingAccelerations_[i] - accelerations_[i]);
  }
  dampingAccelerations_ = dampingAccelerations(approxVelocities_);

  for (size_t i=0; i<numberOfParticles_; i++) {
    velocities_[i] += (0.5 * dt) *
        (accelerations_[i] + forceAccelerations_[i] + dampingAccelerations_[i]);
  }
  dampingAccelerations_ = dampingAccelerations(velocities_);
  for (size_t i=0; i<numberOfParticles_; i++) {
    accelerations_[i] = forceAccelerations_[i] + dampingAccelerations_[i];
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
  for (auto damping : dampings_) {
    damping->apply(time_, particleMass_, velocities_, accelerations_);
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

PointGravity::PointGravity(const Vec2d center, const double nu) :
  center_(center), intensity_(nu)
{}

void PointGravity::apply(const double, const double, const vector<Vec2d>& positions,
                         vector<Vec2d>& accelerations) 
{
  for (size_t i=0; i<positions.size(); i++) {
    accelerations[i] -= intensity_ * (positions[i] - center_);
  }
}

LinearDamping::LinearDamping(const double intensity) :
  intensity_(intensity)
{}

void LinearDamping::apply(const double, const double, const vector<Vec2d>& velocities,
                          vector<Vec2d>& accelerations) 
{
  for (size_t i=0; i<velocities.size(); i++) {
    accelerations[i] -= intensity_ * velocities[i];
  }
}