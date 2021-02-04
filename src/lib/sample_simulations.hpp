#ifndef SAMPLE_SIMULATIONS_HPP
#define SAMPLE_SIMULATIONS_HPP

#include "particle_system.hpp"
#include "pressure_force.hpp"
#include "simulation.hpp"

namespace sph {

struct ToyStarParameters {
  // Simulation
  size_t numberOfParticles = 1000;
  Rectangle region = {-1, -1, 1, 1};
  double timeStep = 0.01;
  double simulationSpeed = 1.0;
  int fps = 60;
  double interactionRadius() const;

  // Physics
  double mass = 2.0;
  double radius = 0.8;
  double pressureConstant = 1.0;
  double dampingConstant = 0.01;
  double gravityConstant() const;
};

struct ToyStarPhysics final : public Physics {
  ToyStarPhysics(const ToyStarParameters& params);

  void applyForces(ParticleSystem& ps) override { gravity_.apply(ps); pressure_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const override { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem&) const override {};

  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce<GasPressure> pressure_;
};

Simulation<ToyStarPhysics> createSimulation(const ToyStarParameters& params = {});

struct BreakingDamParameters {
  // Simulation
  size_t numberOfParticles = 1000;
  double timeStep = 0.01;
  double simulationSpeed = 1.0;
  int fps = 60;
  double interactionRadius() const;

  // Physics
  double fluidMass = 2.0;
  Rectangle region = {-1, -1, 1, 1};
  double damPosition = region.xmin + 0.5 * width(region);
  double pressureConstant = 1.0;
  double dampingConstant = 0.01;
  double gravityAcceleration = 9.8;
  double restDensity() const;
};

struct BreakingDamPhysics final : public Physics {
  BreakingDamPhysics(const BreakingDamParameters& params);
  void applyForces(ParticleSystem& ps) override { gravity_.apply(ps); pressure_.apply(ps); }
  void applyDamping(ParticleSystem& ps) const override { damping_.apply(ps); };
  void resolveCollisions(ParticleSystem& ps) const override;
  void breakDam() { rightWall_ = rightWallAfter_; }

  SurfaceGravity gravity_;
  LinearDamping damping_;
  PressureForce<WaterPressure> pressure_;
  Wall leftWall_, bottomWall_, rightWall_, rightWallAfter_;
};

class BreakingDamSimulation final : public Simulation<BreakingDamPhysics> {
public:
  BreakingDamSimulation(const ParticleSystem& ps, const BreakingDamPhysics& physics,
                        const Rectangle& region, double timeStep,
                        double simulationSpeed = 1.0, int fps = 60);
  void breakDam() { physics_.breakDam(); }
  void increaseDamping();
  void decreaseDamping();
  void increaseGravity();
  void decreaseGravity();
};

BreakingDamSimulation createSimulation(const BreakingDamParameters& params);

} // end namespace sph 

#endif