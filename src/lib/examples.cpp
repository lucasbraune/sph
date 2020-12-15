#include "examples.hpp"
#include <cmath>

PointGravity::PointGravity(double gravityConstant, const Vec2d& center) :
  center_(center), intensity_(gravityConstant)
{}

void PointGravity::apply(const double, const double, const vector<Vec2d>& positions,
                         vector<Vec2d>& accelerations) const
{
  for (size_t i=0; i<positions.size(); i++) {
    accelerations[i] -= intensity_ * (positions[i] - center_);
  }
}

double PointGravity::constant() const
{
  return intensity_;
}

void PointGravity::setConstant(double newValue)
{
  intensity_ = newValue;
}

void SurfaceGravity::apply(double, double, const vector<Vec2d>&,
                           vector<Vec2d>& accelerations) const
{
  for (auto& acc : accelerations) {
    acc += acceleration_;
  }
}

LinearDamping::LinearDamping(double dampingConstant) :
  intensity_(dampingConstant)
{}

Vec2d LinearDamping::acceleration(double, double, const Vec2d& velocity) const
{
  return -intensity_ * velocity;
}

double LinearDamping::constant() const
{
  return intensity_;
}

void LinearDamping::setConstant(double newValue)
{
  intensity_ = newValue;
}

Wall::Wall(const Vec2d& normal, double distanceFromTheOrigin) :
  unitNormal_{unit(normal)},
  ptOnWall_{-distanceFromTheOrigin * unitNormal_} {}

void Wall::resolveCollisions(vector<Vec2d>& positions, vector<Vec2d>& velocities, double) const
{
  for (size_t i=0; i<positions.size(); i++) {
    resolveCollision(positions[i], velocities[i]);
  }
}

void Wall::resolveCollision(Vec2d& pos, Vec2d& vel) const
{
  auto w = pos - ptOnWall_;
  if (w * unitNormal_ > 0) return;
  pos -= 2 * project(w, unitNormal_);
  vel -= 2 * project(vel, unitNormal_);
}

CentralGravityPhysics::CentralGravityPhysics(double gravityConstant, double dampingConstant) :
  gravity_{gravityConstant},
  damping_{dampingConstant} {}

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

ToyStarPhysics::ToyStarPhysics(double gravityConstant,
                               double dampingConstant,
                               double pressureConstant,
                               double interactionRadius) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}}
{}

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

Simulation<PhysicsAdapter<CentralGravityPhysics>> createCentralGravitySimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double timeStep)
{
  return {ParticleSystem{numberOfParticles, totalMass, region},
          PhysicsAdapter<CentralGravityPhysics>{
              CentralGravityPhysics{gravityConstant, dampingConstant}},
          VerletIntegrator{timeStep}};
}

Simulation<PhysicsAdapter<WallBouncingPhysics>> createWallBouncingSimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant,
    double timeStep)
{
  return {ParticleSystem{numberOfParticles, totalMass, region},
          PhysicsAdapter<WallBouncingPhysics>{
              WallBouncingPhysics{gravityConstant, dampingConstant}},
          VerletIntegrator{timeStep}};
}

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
  return {ParticleSystem{numberOfParticles, starMass, initialRegion},
          PhysicsAdapter<ToyStarPhysics>{
              ToyStarPhysics{gravityConstant(starMass, pressureConstant, starRadius),
                             dampingConstant,
                             pressureConstant,
                             interactionRadius(numberOfParticles)}},
          VerletIntegrator{timeStep}};
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
  return {ParticleSystem{numberOfParticles, totalMass, region},
          PhysicsAdapter<WellPhysics>{
              WellPhysics{gravityConstant,
                          dampingConstant,
                          pressureConstant, 
                          interactionRadius(numberOfParticles)}},
          VerletIntegrator{timeStep}};
}