#ifndef EXAMPLES_HPP
#define EXAMPLES_HPP

#include "physics_elements.hpp"
#include "pressure_force.hpp"
#include "simulation.hpp"

namespace sph {

ParticleSystem particlesInRandomPositions(size_t numberOfParticles, double totalMass,
                                          const Rectangle& region);

struct CentralGravityPhysics final : public Physics {
  CentralGravityPhysics(double gravityConstant, double dampingConstant);

  void applyForces(ParticleSystem& ps) const final { gravity_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const final { damping_.apply(ps); }
  void resolveCollisions(ParticleSystem&) const final {}

  PointGravity gravity_;
  LinearDamping damping_;
};

Simulation<CentralGravityPhysics> createCentralGravitySimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 1.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityConstant = 1.0,
    double dampingConstant = 1e-4,
    double timeStep = 0.01);

struct WallBouncingPhysics : public Physics {
  WallBouncingPhysics(double gravityAcceleration, double dampingConstant);

  void applyForces(ParticleSystem& ps) const final { gravity_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const final { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem& ps) const final;

  SurfaceGravity gravity_;
  LinearDamping damping_;
  std::vector<Wall> walls_;
};

Simulation<WallBouncingPhysics> createWallBouncingSimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 1.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityAcceleration = 10.0,
    double dampingConstant = 1e-4,
    double timeStep = 0.01);

struct ToyStarPhysics : public Physics {
  ToyStarPhysics(double gravityConstant,
                 double dampingConstant,
                 double pressureConstant,
                 double interactionRadius);

  void applyForces(ParticleSystem& ps) const final { gravity_.apply(ps); pressure_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const final { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem&) const final {};

  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce pressure_;
};

Simulation<ToyStarPhysics> createToyStarSimulation(
    size_t numberOfParticles = 250,
    double starMass = 2.0,
    double starRadius = 0.75,
    Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
    double dampingConstant = 0.01,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

struct WellPhysics : public Physics {
  WellPhysics(double gravityAcceleration,
              double dampingConstant,
              double pressureConstant,
              double interactionRadius);

  void applyForces(ParticleSystem& ps) const final { gravity_.apply(ps); pressure_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const final { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem& ps) const final;

  SurfaceGravity gravity_;
  PressureForce pressure_;
  LinearDamping damping_;
  const std::vector<Wall> walls_;
};

Simulation<WellPhysics> createWellSimulation(
    size_t numberOfParticles = 250,
    double totalMass = 2.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityAcceleration = 10.0,
    double dampingConstant = 0.05,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

struct BreakingDamPhysics : public Physics {
  BreakingDamPhysics(double gravityAcceleration,
                     double dampingConstant,
                     double pressureConstant,
                     double interactionRadius);

  void applyForces(ParticleSystem& ps) const final { gravity_.apply(ps); pressure_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const final { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem& ps) const final;
  void breakDam() { rightWall_.move(Vec2d{WALL_OFFSET_, 0.0}); }

  SurfaceGravity gravity_;
  PressureForce pressure_;
  LinearDamping damping_;
  Wall leftWall_, bottomWall_, rightWall_;
  static constexpr double WALL_OFFSET_ = 0.9;
};

class BreakingDamSimulation : public Simulation<BreakingDamPhysics> {
public:
  BreakingDamSimulation(const ParticleSystem& ps,
                        const BreakingDamPhysics& prePhysics,
                        const VerletIntegrator& integrator,
                        double simulationSpeed = 1.0, int fps = 60);

  void breakDam() { physics().breakDam(); }
  void increaseDamping();
  void decreaseDamping();
  void increaseGravity();
  void decreaseGravity();
};

BreakingDamSimulation createBreakingDamSimulation(
    size_t numberOfParticles = 250,
    double totalMass = 2.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityAcceleration = 10.0,
    double dampingConstant = 0.05,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

} // end namespace sph 

#endif