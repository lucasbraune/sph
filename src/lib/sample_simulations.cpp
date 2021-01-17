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

Simulation<CentralGravityPhysics> createCentralGravitySimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, totalMass, region),
          CentralGravityPhysics{gravityConstant, dampingConstant},
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

void WallBouncingPhysics::resolveCollisions(ParticleSystem& ps) const
{
  for (auto& wall : walls_) {
    wall.resolveCollisions(ps);
  }
}

Simulation<WallBouncingPhysics> createWallBouncingSimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, totalMass, region),
          WallBouncingPhysics{gravityConstant, dampingConstant},
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

Simulation<ToyStarPhysics> createToyStarSimulation(
    size_t numberOfParticles,
    double starMass,
    double starRadius,
    Rectangle initialRegion,
    double dampingConstant,
    double pressureConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, starMass, initialRegion),
          ToyStarPhysics{gravityConstant(starMass, pressureConstant, starRadius),
                         dampingConstant,
                         pressureConstant,
                         interactionRadius(numberOfParticles)},
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

void WellPhysics::resolveCollisions(ParticleSystem& ps) const
{
  for (auto& wall : walls_) {
    wall.resolveCollisions(ps);
  }
}

Simulation<WellPhysics> createWellSimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double pressureConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, totalMass, region),
          WellPhysics{gravityConstant,
                      dampingConstant,
                      pressureConstant, 
                      interactionRadius(numberOfParticles)},
          VerletIntegrator{timeStep}};
}

BreakingDamPhysics::BreakingDamPhysics(double gravityConstant,
                         double dampingConstant,
                         double pressureConstant,
                         double interactionRadius) :
  gravity_{gravityConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}},
  damping_{dampingConstant},
  leftWall_{Vec2d{1.0, 0.0},  WALL_OFFSET_},
  bottomWall_{Vec2d{0.0, 1.0},  WALL_OFFSET_},
  rightWall_{Vec2d{-1.0, 0.0},  0.0} {}

void BreakingDamPhysics::resolveCollisions(ParticleSystem& ps) const
{
  leftWall_.resolveCollisions(ps);
  bottomWall_.resolveCollisions(ps);
  rightWall_.resolveCollisions(ps);
}

BreakingDamSimulation::BreakingDamSimulation(const ParticleSystem& ps,
                                             const BreakingDamPhysics& physics,
                                             const VerletIntegrator& integrator,
                                             double simulationSpeed, int fps) :
    Simulation<BreakingDamPhysics>{ps, physics, integrator, simulationSpeed, fps} {}

void BreakingDamSimulation::increaseDamping()
{
  auto& damping = physics().damping_;
  damping.setConstant(2.0 * damping.constant());
}

void BreakingDamSimulation::decreaseDamping()
{
  auto& damping = physics().damping_;
  damping.setConstant(0.5 * damping.constant());
}

void BreakingDamSimulation::increaseGravity()
{
  auto& gravity = physics().gravity_;
  gravity.setMagnitude(2.0 * gravity.magnitude());
}

void BreakingDamSimulation::decreaseGravity()
{
  auto& gravity = physics().gravity_;
  gravity.setMagnitude(0.5 * gravity.magnitude());
}

BreakingDamSimulation createBreakingDamSimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double pressureConstant,
    double timeStep)
{
  return {particlesInRandomPositions(numberOfParticles, totalMass, region),
          BreakingDamPhysics{gravityConstant,
                             dampingConstant,
                             pressureConstant, 
                             interactionRadius(numberOfParticles)},
          VerletIntegrator{timeStep}};
}

} // end namespace sph 