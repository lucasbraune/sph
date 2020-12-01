#include "demo.hpp"
#include <functional>
#include <cmath>

using std::reference_wrapper;

CentralPotential::CentralPotential(size_t numberOfParticles, double totalMass, Rectangle region,
                                   double gravityConstant, double dampingConstant) :
  gravity_(gravityConstant),
  damping_(dampingConstant),
  ps_(numberOfParticles, totalMass, region, {std::cref<Force>(gravity_)}, damping_),
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

static double gravityConstant(double totalMass, double pressureConstant, double starRadius)
{
  return 8 * totalMass * pressureConstant / (M_PI * pow(starRadius, 4));
}

static double interactionRadius(double numberOfParticles)
{
  return sqrt(10.0 / numberOfParticles);
}

ToyStar::ToyStar(size_t numberOfParticles, double totalMass, double starRadius, Rectangle region,
                 double dampingConstant, double pressureConstant) :
  gravity_(gravityConstant(totalMass, pressureConstant, starRadius)),
  pressureForce_(interactionRadius(numberOfParticles), GasPressure(pressureConstant)),
  damping_(dampingConstant),
  ps_(numberOfParticles, totalMass, region,
      {std::cref<Force>(gravity_), std::cref<Force>(pressureForce_)}, damping_),
  runner_(ps_)
{}

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