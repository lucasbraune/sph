#ifndef EXAMPLES_HPP
#define EXAMPLES_HPP

#include "physics_elements.hpp"
#include "pressure_force.hpp"
#include "simulation.hpp"

namespace sph {

struct ToyStarPhysics : public Physics {
  ToyStarPhysics(double gravityConstant,
                 double dampingConstant,
                 double pressureConstant,
                 double interactionRadius,
                 const Rectangle& region);

  void applyForces(ParticleSystem& ps) final { gravity_.apply(ps); pressure_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const final { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem&) const final {};

  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce<GasPressure> pressure_;
};

Simulation<ToyStarPhysics> createToyStarSimulation(
    size_t numberOfParticles = 5000,
    double starMass = 2.0,
    double starRadius = 0.75,
    Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
    double dampingConstant = 0.01,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

struct BreakingDamPhysics : public Physics {
  BreakingDamPhysics(double gravityAcceleration,
                     double dampingConstant,
                     double pressureConstant,
                     double interactionRadius);

  void applyForces(ParticleSystem& ps) final { gravity_.apply(ps); pressure_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const final { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem& ps) const final;

  void breakDam() { rightWall_.move(Vec2d{WALL_OFFSET_, 0.0}); }

  SurfaceGravity gravity_;
  LinearDamping damping_;
  PressureForce<GasPressure> pressure_;
  Wall leftWall_, bottomWall_, rightWall_;
  static constexpr double WALL_OFFSET_ = 0.9;
};

class BreakingDamSimulation final : public Simulation<BreakingDamPhysics> {
public:
  BreakingDamSimulation(const ParticleSystem& ps,
                        const BreakingDamPhysics& prePhysics,
                        const Verlet& integrator,
                        double simulationSpeed = 1.0, int fps = 60);

  void breakDam() { physics_.breakDam(); }
  void increaseDamping();
  void decreaseDamping();
  void increaseGravity();
  void decreaseGravity();
};

BreakingDamSimulation createBreakingDamSimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 2.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityAcceleration = 10.0,
    double dampingConstant = 0.05,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

} // end namespace sph 

#endif