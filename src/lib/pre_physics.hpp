/* 
 * File: physics_adapter.hpp
 * Author: Lucas Braune
 * 
 * This header defines the PrePhysics interface, which is similar to Physics interface, but easier 
 * to implement. Also provided is an adapter that converts PrePhysics implementations into Physics
 * implementations.
*/

#ifndef PRE_PHYSICS_HPP
#define PRE_PHYSICS_HPP

#include "particle_system.hpp"

struct PrePhysics {
  virtual ~PrePhysics() {}
  virtual const vector<const Force*> createForceVector() const = 0;
  virtual const vector<const Damping*> createDampingVector() const = 0;
};

template<class PrePhysicsType /* models implementation of PrePhysics */> 
class PhysicsAdapter : public Physics {
public:
  PhysicsAdapter(const PrePhysicsType& prePhysics) : 
    prePhysics_{prePhysics},
    forcePtrs_{prePhysics_.createForceVector()},
    dampingPtrs_{prePhysics_.createDampingVector()} {}

  PhysicsAdapter(const PhysicsAdapter& other) :
    prePhysics_{other.prePhysics_},
    forcePtrs_{prePhysics_.createForceVector()},
    dampingPtrs_{prePhysics_.createDampingVector()} {}

  PhysicsAdapter(PhysicsAdapter&& other) :
    prePhysics_{other.prePhysics_},
    forcePtrs_{prePhysics_.createForceVector()},
    dampingPtrs_{prePhysics_.createDampingVector()}
  {
    other.forcePtrs_ = other.prePhysics_.createForceVector();
    other.dampingPtrs_ = other.prePhysics_.createDampingVector();
  }

  PhysicsAdapter& operator=(const PhysicsAdapter& other)
  {
    prePhysics_ = other.prePhysics_;
    forcePtrs_ = prePhysics_.createForceVector();
    dampingPtrs_ = prePhysics_.createDampingVector();
    return *this;
  }

  PhysicsAdapter& operator=(PhysicsAdapter&& other)
  {
    prePhysics_ = other.prePhysics_;
    forcePtrs_ = prePhysics_.createForceVector();
    dampingPtrs_ = prePhysics_.createDampingVector();
    other.forcePtrs_ = other.prePhysics_.createForceVector();
    other.dampingPtrs_ = other.prePhysics_.createDampingVector();
    return *this;
  }
  
  ~PhysicsAdapter() {};

  const vector<const Force*>& forcePtrs() const { return forcePtrs_; }
  const vector<const Damping*>& dampingPtrs() const { return dampingPtrs_; }

protected:
  PrePhysicsType prePhysics_;
  vector<const Force*> forcePtrs_;
  vector<const Damping*> dampingPtrs_;
};


#endif