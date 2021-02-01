#include "sample_simulations.hpp"
#include <random>
#include <cmath>

using std::vector;

namespace sph {

namespace {

Vec2d randomVec2d(const Rectangle &r = {0.0, 0.0, 1.0, 1.0})
{
  static auto dist = std::uniform_real_distribution<double>{};
  static auto re = std::default_random_engine{};
  constexpr auto affineFn = [](double a, double b, double x) {
    return a + (b - a) * x;
  };
  return {affineFn(r.xmin, r.xmax, dist(re)),
          affineFn(r.ymin, r.ymax, dist(re))};
}

} // namespace

std::vector<Particle> randomParticles(const Rectangle& region, size_t N)
{
  std::vector<Particle> result;
  for (size_t i = 0; i < N; ++i) {
    auto p = Particle{randomVec2d(region), Vec2d{}, Vec2d{}};
    result.emplace_back(p);
  }
  return result;
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
  return {ParticleSystem{randomParticles(region, numberOfParticles), totalMass},
          CentralGravityPhysics{gravityConstant, dampingConstant},
          Verlet{timeStep}};
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
  return {ParticleSystem{randomParticles(region, numberOfParticles), totalMass},
          WallBouncingPhysics{gravityConstant, dampingConstant},
          Verlet{timeStep}};
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
    double totalMass,
    double starRadius,
    Rectangle region,
    double dampingConstant,
    double pressureConstant,
    double timeStep)
{
  return {ParticleSystem{randomParticles(region, numberOfParticles), totalMass},
          ToyStarPhysics{gravityConstant(totalMass, pressureConstant, starRadius),
                         dampingConstant,
                         pressureConstant,
                         interactionRadius(numberOfParticles)},
          Verlet{timeStep}};
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
  return {ParticleSystem{randomParticles(region, numberOfParticles), totalMass},
          WellPhysics{gravityConstant,
                      dampingConstant,
                      pressureConstant, 
                      interactionRadius(numberOfParticles)},
          Verlet{timeStep}};
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
                                             const Verlet& integrator,
                                             double simulationSpeed, int fps) :
    Simulation<BreakingDamPhysics>{ps, physics, integrator, simulationSpeed, fps} {}

void BreakingDamSimulation::increaseDamping()
{
  auto& damping = physics_.damping_;
  damping.setConstant(2.0 * damping.constant());
}

void BreakingDamSimulation::decreaseDamping()
{
  auto& damping = physics_.damping_;
  damping.setConstant(0.5 * damping.constant());
}

void BreakingDamSimulation::increaseGravity()
{
  auto& gravity = physics_.gravity_;
  gravity.setMagnitude(2.0 * gravity.magnitude());
}

void BreakingDamSimulation::decreaseGravity()
{
  auto& gravity = physics_.gravity_;
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
  return {ParticleSystem{randomParticles(region, numberOfParticles), totalMass},
          BreakingDamPhysics{gravityConstant,
                             dampingConstant,
                             pressureConstant, 
                             interactionRadius(numberOfParticles)},
          Verlet{timeStep}};
}

} // end namespace sph 