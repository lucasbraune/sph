#include "demo.hpp"
#include <functional>

using std::reference_wrapper;

CentralPotential::CentralPotential(size_t numberOfParticles, double totalMass, Rectangle region,
                                   double gravityConstant, double dampingConstant) :
  gravity_(ZERO_VECTOR, gravityConstant),
  damping_(dampingConstant),
  ps_(randomVectors(region, numberOfParticles),
      vector<Vec2d>(numberOfParticles),
      vector<reference_wrapper<const Force>>{std::cref<Force>(gravity_)},
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