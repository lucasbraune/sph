#ifndef EXAMPLES_HPP
#define EXAMPLES_HPP

#include "physics_elements.hpp"
#include "pressure_force.hpp"
#include "pre_physics.hpp"
#include "simulation.hpp"

namespace sph {

ParticleSystem particlesInRandomPositions(size_t numberOfParticles, double totalMass,
                                          const Rectangle& region);

struct CentralGravityPhysics : public PrePhysics {
  CentralGravityPhysics(double gravityConstant, double dampingConstant);

  const std::vector<const Force*> createForceVector() const override {return {&gravity_}; }
  const std::vector<const Collidable*> createCollidableVector() const override { return {}; }
  const Damping* dampingPtr() const override {return &damping_; }

  PointGravity gravity_;
  LinearDamping damping_;
};

Simulation<PhysicsAdapter<CentralGravityPhysics>> createCentralGravitySimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 1.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityConstant = 1.0,
    double dampingConstant = 1e-4,
    double timeStep = 0.01);

struct WallBouncingPhysics : public PrePhysics {
  WallBouncingPhysics(double gravityAcceleration, double dampingConstant);

  const std::vector<const Force*> createForceVector() const override {return {&gravity_}; }
  const std::vector<const Collidable*> createCollidableVector() const override; 
  const Damping* dampingPtr() const override {return &damping_; }

  SurfaceGravity gravity_;
  LinearDamping damping_;
  std::vector<Wall> walls_;
};

Simulation<PhysicsAdapter<WallBouncingPhysics>> createWallBouncingSimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 1.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityAcceleration = 10.0,
    double dampingConstant = 1e-4,
    double timeStep = 0.01);

struct ToyStarPhysics : public PrePhysics {
  ToyStarPhysics(double gravityConstant,
                 double dampingConstant,
                 double pressureConstant,
                 double interactionRadius);

  const std::vector<const Force*> createForceVector() const override
  {
    return {&gravity_, &pressure_};
  }
  const std::vector<const Collidable*> createCollidableVector() const override { return {}; }
  const Damping* dampingPtr() const override {return &damping_; }

  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce pressure_;
};

Simulation<PhysicsAdapter<ToyStarPhysics>> createToyStarSimulation(
    size_t numberOfParticles = 250,
    double starMass = 2.0,
    double starRadius = 0.75,
    Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
    double dampingConstant = 0.01,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

struct WellPhysics : public PrePhysics {
  WellPhysics(double gravityAcceleration,
              double dampingConstant,
              double pressureConstant,
              double interactionRadius);

  const std::vector<const Force*> createForceVector() const override
  {
    return {&gravity_, &pressure_};
  }
  const std::vector<const Collidable*> createCollidableVector() const override; 
  const Damping* dampingPtr() const override {return &damping_; }

  SurfaceGravity gravity_;
  PressureForce pressure_;
  LinearDamping damping_;
  const std::vector<Wall> walls_;
};

Simulation<PhysicsAdapter<WellPhysics>> createWellSimulation(
    size_t numberOfParticles = 250,
    double totalMass = 2.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityAcceleration = 10.0,
    double dampingConstant = 0.05,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

struct BreakingDamPhysics : public PrePhysics {
  BreakingDamPhysics(double gravityAcceleration,
              double dampingConstant,
              double pressureConstant,
              double interactionRadius);

  const std::vector<const Force*> createForceVector() const override
  {
    return {&gravity_, &pressure_};
  }
  const std::vector<const Collidable*> createCollidableVector() const override; 
  const Damping* dampingPtr() const override {return &damping_; }
  void breakDam() { rightWall_.move(Vec2d{WALL_OFFSET_, 0.0}); }

  SurfaceGravity gravity_;
  PressureForce pressure_;
  LinearDamping damping_;
  Wall leftWall_, bottomWall_, rightWall_;
  static constexpr double WALL_OFFSET_ = 0.9;
};

Simulation<PhysicsAdapter<BreakingDamPhysics>> createBreakingDamSimulation(
    size_t numberOfParticles = 250,
    double totalMass = 2.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityAcceleration = 10.0,
    double dampingConstant = 0.05,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

} // end namespace sph 

#endif