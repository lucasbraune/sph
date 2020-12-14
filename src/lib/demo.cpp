#include "demo.hpp"
#include <cmath>

using std::reference_wrapper;

CentralGravityPhysics::CentralGravityPhysics(double gravityConstant, double dampingConstant) :
  gravity_{gravityConstant},
  damping_{dampingConstant}
{
  forces_.emplace_back(&gravity_);
  dampings_.emplace_back(&damping_);
}

ToyStarPhysics::ToyStarPhysics(double gravityConstant,
                               double dampingConstant,
                               double pressureConstant,
                               double interactionRadius) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}}
{
  forces_.emplace_back(&gravity_);
  forces_.emplace_back(&pressure_);
  dampings_.emplace_back(&damping_);
}

CentralPotentialSimulation::CentralPotentialSimulation(size_t numberOfParticles, double totalMass,
                                                       Rectangle region, double gravityConstant,
                                                       double dampingConstant) :
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

ToyStarSimulation::ToyStarSimulation(size_t numberOfParticles, double totalMass,
                                     double starRadius, Rectangle region,
                                     double dampingConstant, double pressureConstant) :
  CentralPotentialSimulation(numberOfParticles, totalMass, region,
                             gravityConstant(totalMass, pressureConstant, starRadius),
                                             dampingConstant),
                             pressureForce_(interactionRadius(numberOfParticles),
                                            GasPressure(pressureConstant))
{
  addForce(pressureForce_);
}