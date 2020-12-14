#include "demo.hpp"
#include <cmath>

using std::reference_wrapper;

CentralGravityPhysics::CentralGravityPhysics(double gravityConstant, double dampingConstant) :
  gravity_{gravityConstant},
  damping_{dampingConstant}
{}

ToyStarPhysics::ToyStarPhysics(double gravityConstant,
                               double dampingConstant,
                               double pressureConstant,
                               double interactionRadius) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}}
{}

Simulation<CentralGravityPhysics> createCentralGravitySimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant)
{
  return {ParticleSystem{numberOfParticles, totalMass, region},
          CentralGravityPhysics{gravityConstant, dampingConstant}};
}

static double gravityConstant(double totalMass, double pressureConstant, double starRadius)
{
  return 8 * totalMass * pressureConstant / (M_PI * pow(starRadius, 4));
}

static double interactionRadius(double numberOfParticles)
{
  return sqrt(10.0 / numberOfParticles);
}

Simulation<ToyStarPhysics> createToyStarSimulation(
    size_t numberOfParticles,
    double starMass,
    double starRadius,
    Rectangle initialRegion,
    double dampingConstant,
    double pressureConstant)
{
  return {ParticleSystem{numberOfParticles, starMass, initialRegion},
          ToyStarPhysics{gravityConstant(starMass, pressureConstant, starRadius),
                         dampingConstant,
                         pressureConstant,
                         interactionRadius(numberOfParticles)}};
}