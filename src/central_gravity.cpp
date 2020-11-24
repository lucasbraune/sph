#include "central_gravity.hpp"

CentralGravity::CentralGravity(const size_t numberOfParticles, const double totalMass,
    const Rectangle region, const double gravityConstant, const double dampingConstant,
    const double timeStep) :
  gravity_(ZERO_VECTOR, gravityConstant),
  damping_(dampingConstant),
  ps_(randomVectors(region, numberOfParticles),
                    vector<Vec2d>(numberOfParticles),
                    vector<Force*>{&gravity_},
                    damping_,
                    totalMass / numberOfParticles,
                    timeStep),
  sim_(ps_)
{}

Simulation& CentralGravity::simulation()
{
  return sim_;
}

LinearDamping& CentralGravity::damping()
{
  return damping_;
}

PointGravity& CentralGravity::gravity()
{
  return gravity_;
}