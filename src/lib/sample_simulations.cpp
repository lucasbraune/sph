#include "sample_simulations.hpp"
#include <random>
#include <cmath>

using std::vector;

namespace sph {

static vector<sph::Vec2d> randomVectors(const sph::Rectangle region, const size_t N) {
  auto result = vector<sph::Vec2d>{};
  auto xDist = std::uniform_real_distribution<double>{region.xmin, region.xmax};
  auto yDist = std::uniform_real_distribution<double>{region.ymin, region.ymax};
  auto re = std::default_random_engine{};
  for (size_t i=0; i<N; i++) {
    result.push_back(sph::Vec2d{xDist(re), yDist(re)});
  }
  return result;
}

ParticleSystem particlesInRandomPositions(size_t numberOfParticles, double totalMass,
                                          const Rectangle& region)
{
  return {randomVectors(region, numberOfParticles), totalMass / numberOfParticles};
}

CentralGravityPhysics::CentralGravityPhysics(double gravityConstant, double dampingConstant) :
  gravity_{gravityConstant},
  damping_{dampingConstant} {}

Simulation<PhysicsAdapter<CentralGravityPhysics>> createCentralGravitySimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, totalMass, region),
          PhysicsAdapter<CentralGravityPhysics>{
              CentralGravityPhysics{gravityConstant, dampingConstant}},
          VerletIntegrator{timeStep}};
}

WallBouncingPhysics::WallBouncingPhysics(double gravityConstant, double dampingConstant) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  walls_{
    {Vec2d{0.0, 1.0},  0.7},
    {Vec2d{1.0, 1.0},  0.8},
    {Vec2d{-1.0, 1.0},  0.8},
  } {}

const vector<const Collidable*> WallBouncingPhysics::createCollidableVector() const
{
  vector<const Collidable*> result;
  for (const auto& wall : walls_) {
    result.emplace_back(&wall);
  }
  return result;
}

Simulation<PhysicsAdapter<WallBouncingPhysics>> createWallBouncingSimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, totalMass, region),
          PhysicsAdapter<WallBouncingPhysics>{
              WallBouncingPhysics{gravityConstant, dampingConstant}},
          VerletIntegrator{timeStep}};
}

ToyStarPhysics::ToyStarPhysics(double gravityConstant,
                               double dampingConstant,
                               double pressureConstant,
                               double interactionRadius) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}}
{}

static double gravityConstant(double totalMass, double pressureConstant, double starRadius)
{
  return 8 * totalMass * pressureConstant / (M_PI * pow(starRadius, 4));
}

static double interactionRadius(double numberOfParticles)
{
  return sqrt(10.0 / numberOfParticles);
}

Simulation<PhysicsAdapter<ToyStarPhysics>> createToyStarSimulation(
    size_t numberOfParticles,
    double starMass,
    double starRadius,
    Rectangle initialRegion,
    double dampingConstant,
    double pressureConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, starMass, initialRegion),
          PhysicsAdapter<ToyStarPhysics>{
              ToyStarPhysics{gravityConstant(starMass, pressureConstant, starRadius),
                             dampingConstant,
                             pressureConstant,
                             interactionRadius(numberOfParticles)}},
          VerletIntegrator{timeStep}};
}

WellPhysics::WellPhysics(double gravityConstant,
                         double dampingConstant,
                         double pressureConstant,
                         double interactionRadius) :
  gravity_{gravityConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}},
  damping_{dampingConstant},
  walls_{
    {Vec2d{0.0, 1.0},  0.7},
    {Vec2d{1.0, 1.0},  0.8},
    {Vec2d{-1.0, 1.0},  0.8},
  } {}

const vector<const Collidable*> WellPhysics::createCollidableVector() const
{
  vector<const Collidable*> result;
  for (const auto& wall : walls_) {
    result.emplace_back(&wall);
  }
  return result;
}

Simulation<PhysicsAdapter<WellPhysics>> createWellSimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double pressureConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, totalMass, region),
          PhysicsAdapter<WellPhysics>{
              WellPhysics{gravityConstant,
                          dampingConstant,
                          pressureConstant, 
                          interactionRadius(numberOfParticles)}},
          VerletIntegrator{timeStep}};
}

} // end namespace sph 