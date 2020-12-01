#include "particle_system.hpp"

PointGravity::PointGravity(const double gravityConstant, const Vec2d center) :
  center_(center), intensity_(gravityConstant)
{}

void PointGravity::apply(const double, const double, const vector<Vec2d>& positions,
                         vector<Vec2d>& accelerations) const
{
  for (size_t i=0; i<positions.size(); i++) {
    accelerations[i] -= intensity_ * (positions[i] - center_);
  }
}

void PointGravity::setConstant(double newValue)
{
  intensity_ = newValue;
}

void PointGravity::increase()
{
  intensity_ *= 2.0;
}

void PointGravity::decrease()
{
  intensity_ *= 0.5;
}

LinearDamping::LinearDamping(const double dampingConstant) :
  intensity_(dampingConstant)
{}

Vec2d LinearDamping::acceleration(const double, const double, const Vec2d velocity) const
{
  return -intensity_ * velocity;
}

void LinearDamping::increase()
{
  intensity_ *= 2.0;
}

void LinearDamping::decrease()
{
  intensity_ *= 0.5;
}

ParticleSystem::ParticleSystem(const vector<Vec2d>& initialPositions,
                               const vector<Vec2d>& initialVelocities,
                               const vector<reference_wrapper<const Force>>& forces,
                               const Damping& damping,
                               double particleMass) :
  numberOfParticles(initialPositions.size()),
  particleMass(particleMass),
  forces(forces),
  damping(damping),
  positions(initialPositions),
  velocities(initialVelocities),
  accelerations(positions.size()),
  time(0.0)
{}

ParticleSystem::ParticleSystem(size_t numberOfParticles, double totalMass, Rectangle region,
                 const vector<reference_wrapper<const Force>>& forces, const Damping& damping) :
  ParticleSystem(randomVectors(region, numberOfParticles),
                 vector<Vec2d>(numberOfParticles),
                 forces,
                 damping,
                 totalMass / numberOfParticles)
{}

void TimeIntegrator::integrate(ParticleSystem& ps, double duration)
{
  double target = ps.time + duration;
  while (ps.time < target) {
    step(ps);
  }
}

EulerIntegrator::EulerIntegrator(double timeStep) : timeStep_(timeStep) {}

void EulerIntegrator::step(ParticleSystem& ps)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i];
    ps.velocities[i] += timeStep_ * ps.accelerations[i];
    ps.accelerations[i] = ps.damping.acceleration(ps.time, ps.particleMass, ps.velocities[i]);
  }
  for (auto& force : ps.forces) {
    force.get().apply(ps.time, ps.particleMass, ps.positions, ps.accelerations);
  }
}

VerletIntegrator::VerletIntegrator(double timeStep) : timeStep_(timeStep) {}

void VerletIntegrator::step(ParticleSystem& ps)
{
  ps.time += timeStep_;
  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.positions[i] += timeStep_ * ps.velocities[i] +
                       (0.5 * timeStep_ * timeStep_) * ps.accelerations[i];
  }

  nextForceAcc_.resize(ps.numberOfParticles);
  for (Vec2d& acc : nextForceAcc_) {
    acc = ZERO_VECTOR;
  }
  for (auto& force : ps.forces) {
    force.get().apply(ps.time, ps.particleMass, ps.positions, nextForceAcc_);
  }

  for (size_t i=0; i<ps.numberOfParticles; i++) {
    ps.velocities[i] = nextVelocity(ps.velocities[i], ps.accelerations[i], nextForceAcc_[i],
                                    ps.damping, ps.particleMass, ps.time, timeStep_);
    Vec2d nextDampingAcc = ps.damping.acceleration(ps.time, ps.particleMass, ps.velocities[i]);
    ps.accelerations[i] = nextForceAcc_[i] + nextDampingAcc;
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