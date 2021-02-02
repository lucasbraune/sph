#include "sample_simulations.hpp"
#include <random>
#include <cmath>

using std::vector;

using namespace sph;

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

std::vector<Particle> randomParticles(const Rectangle& region, size_t numberOfParticles)
{
  std::vector<Particle> result;
  for (size_t i = 0; i < numberOfParticles; ++i) {
    auto p = Particle{randomVec2d(region), Vec2d{}, Vec2d{}};
    result.emplace_back(p);
  }
  return result;
}

double interactionRadius(double numberOfParticles)
{
  return sqrt(10.0 / numberOfParticles);
}

} // namespace

sph::ToyStarPhysics::ToyStarPhysics(double gravityConstant,
                               double dampingConstant,
                               double pressureConstant,
                               double interactionRadius,
                               const Rectangle& region) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{makePressureForce(GasPressure{pressureConstant}, interactionRadius, region)}
{}

Simulation<ToyStarPhysics> sph::createToyStarSimulation(
    size_t numberOfParticles,
    double totalMass,
    double starRadius,
    Rectangle region,
    double dampingConstant,
    double pressureConstant,
    double timeStep)
{
  auto gravityConstant = 8 * totalMass * pressureConstant / (M_PI * pow(starRadius, 4));
  return {ParticleSystem{randomParticles(region, numberOfParticles), totalMass},
          ToyStarPhysics{gravityConstant, dampingConstant, pressureConstant,
                         interactionRadius(numberOfParticles), region},
          Verlet{timeStep}};
}

sph::BreakingDamPhysics::BreakingDamPhysics(double gravityConstant,
                         double dampingConstant,
                         double pressureConstant,
                         double interactionRadius) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{makePressureForce(GasPressure{pressureConstant}, interactionRadius)},
  leftWall_{Vec2d{1.0, 0.0},  WALL_OFFSET_},
  bottomWall_{Vec2d{0.0, 1.0},  WALL_OFFSET_},
  rightWall_{Vec2d{-1.0, 0.0},  0.0} {}

void sph::BreakingDamPhysics::resolveCollisions(ParticleSystem& ps) const
{
  leftWall_.resolveCollisions(ps);
  bottomWall_.resolveCollisions(ps);
  rightWall_.resolveCollisions(ps);
}

sph::BreakingDamSimulation::BreakingDamSimulation(const ParticleSystem& ps,
                                             const BreakingDamPhysics& physics,
                                             const Verlet& integrator,
                                             double simulationSpeed, int fps) :
    Simulation<BreakingDamPhysics>{ps, physics, integrator, simulationSpeed, fps} {}

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

BreakingDamSimulation sph::createBreakingDamSimulation(
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