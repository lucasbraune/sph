#include "demo.hpp"
#include <cmath>

using std::reference_wrapper;

CentralPotential::CentralPotential(size_t numberOfParticles, double totalMass, Rectangle region,
                                   double gravityConstant, double dampingConstant) :
  Simulation(numberOfParticles, totalMass, region),
  gravity_(gravityConstant),
  damping_(dampingConstant)
{
  addForce(gravity_);
  addDamping(damping_);
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
  CentralPotential(numberOfParticles, totalMass, region,
                   gravityConstant(totalMass, pressureConstant, starRadius),
                   dampingConstant),
  pressureForce_(interactionRadius(numberOfParticles), GasPressure(pressureConstant))
{
  addForce(pressureForce_);
}