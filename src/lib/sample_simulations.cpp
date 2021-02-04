#include "sample_simulations.hpp"
#include <cassert>
#include <cmath>
#include <random>

using std::vector;

using namespace sph;

namespace {

double interactionRadius(const Rectangle& region, size_t numberOfParticles)
{
  constexpr auto neighborsPerParticle = 10;
  auto areaPerParticle = neighborsPerParticle * area(region) / numberOfParticles;
  return std::sqrt(areaPerParticle / M_PI);
}

} // namespace

double sph::ToyStarParameters::interactionRadius() const
{
  return ::interactionRadius(region, numberOfParticles);
}

double sph::BreakingDamParameters::interactionRadius() const
{
  return ::interactionRadius(region, numberOfParticles);
}

double sph::ToyStarParameters::gravityConstant() const
{
  return 8.0 * mass * pressureConstant / (M_PI * std::pow(radius, 4));
}

sph::ToyStarPhysics::ToyStarPhysics(const ToyStarParameters& params) :
  gravity_{params.gravityConstant()},
  damping_{params.dampingConstant},
  pressure_{makePressureForce(GasPressure{params.pressureConstant},
                              params.interactionRadius(), params.region)}
{}

Simulation<ToyStarPhysics> sph::createSimulation(const ToyStarParameters& params)
{
  return {createParticleSystem(params.numberOfParticles, params.mass, params.region),
          ToyStarPhysics{params}, params.region, Verlet{params.timeStep},
          params.simulationSpeed, params.fps};
}

double sph::BreakingDamParameters::restDensity() const
{
  const auto fractionOfArea = (damPosition - region.xmin) / width(region);
  assert(fractionOfArea > 0);
  return fluidMass / (fractionOfArea * area(region));
}

namespace {

constexpr auto LEFT = Vec2d{-1, 0};
constexpr auto RIGHT = Vec2d{1, 0};
constexpr auto UP = Vec2d{0, 1};

} // namespace

sph::BreakingDamPhysics::BreakingDamPhysics(const BreakingDamParameters& params) :
  gravity_{params.gravityAcceleration},
  damping_{params.dampingConstant},
  pressure_{makePressureForce(WaterPressure{params.pressureConstant, params.restDensity()},
                              params.interactionRadius(), params.region)},
  leftWall_{RIGHT, {params.region.xmin, 0}},
  bottomWall_{UP, {0, params.region.ymin}},
  rightWall_{LEFT, {params.damPosition, 0}},
  rightWallAfter_{LEFT, {params.region.xmax, 0}}
{}

void sph::BreakingDamPhysics::resolveCollisions(ParticleSystem& ps) const
{
  leftWall_.resolveCollisions(ps);
  bottomWall_.resolveCollisions(ps);
  rightWall_.resolveCollisions(ps);
}

sph::BreakingDamSimulation::BreakingDamSimulation(const ParticleSystem& ps,
                                                  const BreakingDamPhysics& physics,
                                                  const Rectangle& region,
                                                  double timeStep,
                                                  double simulationSpeed,
                                                  int fps) :
  Simulation<BreakingDamPhysics>{ps, physics, region, Verlet{timeStep}, simulationSpeed, fps}
{}

void sph::BreakingDamSimulation::increaseDamping()
{
  auto& damping = physics_.damping_;
  damping.setConstant(2.0 * damping.constant());
}

void sph::BreakingDamSimulation::decreaseDamping()
{
  auto& damping = physics_.damping_;
  damping.setConstant(0.5 * damping.constant());
}

void sph::BreakingDamSimulation::increaseGravity()
{
  auto& gravity = physics_.gravity_;
  gravity.setMagnitude(2.0 * gravity.magnitude());
}

void sph::BreakingDamSimulation::decreaseGravity()
{
  auto& gravity = physics_.gravity_;
  gravity.setMagnitude(0.5 * gravity.magnitude());
}

Rectangle initialRegion(const BreakingDamParameters& params)
{
  return {params.region.xmin, params.region.ymin,
          params.damPosition, params.region.ymax};
}

BreakingDamSimulation sph::createSimulation(const BreakingDamParameters& params)
{
  return {createParticleSystem(params.numberOfParticles, params.fluidMass, initialRegion(params)),
          BreakingDamPhysics{params}, params.region, params.timeStep,
          params.simulationSpeed, params.fps};
}