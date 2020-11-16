#include "particle_system.hpp"

PointGravity::PointGravity(const Vec2d center, const double nu) :
  center_(center), nu_(nu)
{}

void PointGravity::apply(const vector<Vec2d>& positions, const vector<Vec2d>&,  
      vector<Vec2d>& accelerations, const double, const double) 
{
  for (size_t i=0; i<positions.size(); i++) {
    accelerations[i] -= nu_ * (positions[i] - center_);
  }
}

ParticleSystem::ParticleSystem(const vector<Vec2d>& positions, const vector<Vec2d>& velocities,
    vector<Force*> forces, const double m, const double dt) :
  N_(positions.size()),
  positions_(positions),
  velocities_(velocities),
  accelerations_(positions_.size()),
  forces_(forces),
  particleMass_(m),
  t_(0.0),
  dt_(dt)
{}

double ParticleSystem::time() const
{
  return t_;
}

const vector<Vec2d>& ParticleSystem::positions() const
{
  return positions_;
}

void ParticleSystem::updateAccelerations()
{
  constexpr Vec2d ZERO_VECTOR{};
  for (Vec2d& acc : accelerations_) {
    acc = ZERO_VECTOR;
  }
  for (auto force : forces_) {
    force->apply(positions_, velocities_, accelerations_, t_, particleMass_);
  }
}

void ParticleSystem::eulerStep(const double dt)
{
  updateAccelerations();
  for (size_t i=0; i<N_; i++) {
    positions_[i] += dt * velocities_[i];
    velocities_[i] += dt * accelerations_[i];
  }
  t_ += dt;
}

void ParticleSystem::integrate(double time)
{
  size_t steps = ceil(time / dt_);
  for (size_t i=0; i<steps; i++) {
    eulerStep(dt_);
  }
}