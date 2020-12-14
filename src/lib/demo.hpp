#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pressure_force.hpp"

struct PhysicsInterface {
  virtual ~PhysicsInterface() {}
  virtual const vector<const Force*> createForceVector() const = 0;
  virtual const vector<const Damping*> createDampingVector() const = 0;
};

struct CachedPhysicsInterface {
  virtual ~CachedPhysicsInterface() {}
  virtual const vector<const Force*>& forces() const = 0;
  virtual const vector<const Damping*>& dampings() const = 0;
};

template<class Physics /* models implementation of PhysicsInterface */> 
class CachedPhysics : public CachedPhysicsInterface {
public:
  CachedPhysics(const Physics& physics) : 
    physics_{physics},
    forces_{physics_.createForceVector()},
    dampings_{physics_.createDampingVector()} {}

  CachedPhysics(const CachedPhysics& other) :
    physics_{other.physics_},
    forces_{physics_.createForceVector()},
    dampings_{physics_.createDampingVector()} {}

  CachedPhysics(CachedPhysics&& other) :
    physics_{other.physics_},
    forces_{physics_.createForceVector()},
    dampings_{physics_.createDampingVector()}
  {
    other.forces_ = other.physics_.createForceVector();
    other.dampings_ = other.physics_.createDampingVector();
  }

  CachedPhysics& operator=(const CachedPhysics& other)
  {
    physics_ = other.physics_;
    forces_ = physics_.createForceVector();
    dampings_ = physics_.createDampingVector();
    return *this;
  }

  CachedPhysics& operator=(CachedPhysics&& other)
  {
    physics_ = other.physics_;
    forces_ = physics_.createForceVector();
    dampings_ = physics_.createDampingVector();
    other.forces_ = other.physics_.createForceVector();
    other.dampings_ = other.physics_.createDampingVector();
    return *this;
  }
  
  ~CachedPhysics() {};

  const vector<const Force*>& forces() const { return forces_; }
  const vector<const Damping*>& dampings() const { return dampings_; }

private:
  Physics physics_;
  vector<const Force*> forces_;
  vector<const Damping*> dampings_;
};

struct CentralGravity : public PhysicsInterface {
  CentralGravity(double gravityConstant,
                 double dampingConstant);

  const vector<const Force*> createForceVector() const {return {&gravity_}; }
  const vector<const Damping*> createDampingVector() const {return {&damping_}; }

  PointGravity gravity_;
  LinearDamping damping_;
};

struct ToyStar : public PhysicsInterface {
  ToyStar(double gravityConstant,
          double dampingConstant,
          double pressureConstant,
          double interactionRadius);

  const vector<const Force*> createForceVector() const {return {&gravity_, &pressure_}; }
  const vector<const Damping*> createDampingVector() const {return {&damping_}; }

  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce pressure_;
};

Simulation<CachedPhysics<CentralGravity>> createCentralGravitySimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 1.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityConstant = 1.0,
    double dampingConstant = 0.01);

Simulation<CachedPhysics<ToyStar>> createToyStarSimulation(
    size_t numberOfParticles = 250,
    double starMass = 2.0,
    double starRadius = 0.75,
    Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
    double dampingConstant = 1.0,
    double pressureConstant = 1.0);

#endif