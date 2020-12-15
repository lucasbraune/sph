#include "demo.hpp"
#include <cmath>

using std::reference_wrapper;

CentralGravity::CentralGravity(double gravityConstant, double dampingConstant) :
  gravity_{gravityConstant},
  damping_{dampingConstant} {}

ToyStar::ToyStar(double gravityConstant,
                 double dampingConstant,
                 double pressureConstant,
                 double interactionRadius) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}}
{}

Simulation<PhysicsAdapter<CentralGravity>> createCentralGravitySimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant)
{
  return {ParticleSystem{numberOfParticles, totalMass, region},
          PhysicsAdapter<CentralGravity>{
              CentralGravity{gravityConstant, dampingConstant}}};
}

static double gravityConstant(double totalMass, double pressureConstant, double starRadius)
{
  return 8 * totalMass * pressureConstant / (M_PI * pow(starRadius, 4));
}

static double interactionRadius(double numberOfParticles)
{
  return sqrt(10.0 / numberOfParticles);
}

Simulation<PhysicsAdapter<ToyStar>> createToyStarSimulation(
    size_t numberOfParticles,
    double starMass,
    double starRadius,
    Rectangle initialRegion,
    double dampingConstant,
    double pressureConstant)
{
  return {ParticleSystem{numberOfParticles, starMass, initialRegion},
          PhysicsAdapter<ToyStar>{
              ToyStar{gravityConstant(starMass, pressureConstant, starRadius),
                      dampingConstant,
                      pressureConstant,
                      interactionRadius(numberOfParticles)}}};
}