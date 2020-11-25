#include "central_gravity.hpp"

CentralGravitySimulation::CentralGravitySimulation(const size_t numberOfParticles, const double totalMass,
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

const ParticleSystem& CentralGravitySimulation::state() const
{
  return ps_;
}

SimulationRunner& CentralGravitySimulation::runner()
{
  return runner_;
}

LinearDamping& CentralGravitySimulation::damping()
{
  return damping_;
}

PointGravity& CentralGravitySimulation::gravity()
{
  return gravity_;
}