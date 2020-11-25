#include "central_gravity.hpp"

CentralPotential::CentralPotential(const size_t numberOfParticles, const double totalMass,
    const Rectangle region, const double gravityConstant, const double dampingConstant) :
  gravity_(ZERO_VECTOR, gravityConstant),
  damping_(dampingConstant),
  ps_(randomVectors(region, numberOfParticles),
      vector<Vec2d>(numberOfParticles),
      vector<Force*>{&gravity_},
      damping_,
      totalMass / numberOfParticles),
  runner_(ps_)
{}

const ParticleSystem& CentralPotential::state() const
{
  return ps_;
}

SimulationRunner& CentralPotential::runner()
{
  return runner_;
}

const SimulationRunner& CentralPotential::runner() const
{
  return runner_;
}

LinearDamping& CentralPotential::damping()
{
  return damping_;
}

PointGravity& CentralPotential::gravity()
{
  return gravity_;
}