#include "demo.hpp"
#include <functional>
#include <cmath>

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

ToyStar::ToyStar(size_t numberOfParticles, double totalMass, double starRadius, Rectangle region,
                 double dampingConstant, double pressureConstant) :
  gravity_(ZERO_VECTOR, 0.0),
  pressureFunction_(pressureConstant),
  pressureForce_(std::make_unique<TrivialNeighborIteratorFactory>(),
                 std::make_unique<CubicKernel>(0.04 / sqrt(numberOfParticles / 1000.0)),
                 pressureFunction_),
  damping_(dampingConstant),
  ps_(randomVectors(region, numberOfParticles),
      vector<Vec2d>(numberOfParticles),
      {std::cref<Force>(gravity_), std::cref<Force>(pressureForce_)},
      damping_,
      totalMass / numberOfParticles),
  runner_(ps_)
{
  constexpr double PI = 3.14159265359;
  gravity_.setConstant(8 * totalMass * pressureConstant / (PI * pow(starRadius, 4)));
}

const ParticleSystem& ToyStar::state() const
{
  return ps_;
}

SimulationRunner& ToyStar::runner()
{
  return runner_;
}

const SimulationRunner& ToyStar::runner() const
{
  return runner_;
}

LinearDamping& ToyStar::damping()
{
  return damping_;
}

PointGravity& ToyStar::gravity()
{
  return gravity_;
}