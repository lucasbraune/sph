/* 
 * File: physics_adapter.hpp
 * Author: Lucas Braune
 * 
 * This header defines the PrePhysics interface, which is similar to the Physics interface, but
 * is easier to implement. This header also provides an adapter that converts implementations of
 * PrePhysics into implementations of Physics.
*/

#ifndef PRE_PHYSICS_HPP
#define PRE_PHYSICS_HPP

#include "particle_system.hpp"

namespace sph {

struct PrePhysics {
  virtual ~PrePhysics() {}
  // Returns a vector of non-null pointers
  virtual const std::vector<const Force*> createForceVector() const = 0;
  // Returns a vector of non-null pointers
  virtual const std::vector<const Collidable*> createCollidableVector() const = 0;
  // Warning: may return a null pointer
  virtual const Damping* dampingPtr() const = 0;
};

template<class PrePhysicsType /* models implementation of PrePhysics */> 
class PhysicsAdapter : public Physics {
public:
  PhysicsAdapter(const PrePhysicsType& prePhysics) : 
    prePhysics_{prePhysics}
  {
    updatePtrs();
  }

  PhysicsAdapter(const PhysicsAdapter& other) :
    prePhysics_{other.prePhysics_}
  {
    updatePtrs();
  }

  PhysicsAdapter(PhysicsAdapter&& other) :
    prePhysics_{other.prePhysics_}
  {
    updatePtrs();
    other.updatePtrs();
  }

  PhysicsAdapter& operator=(const PhysicsAdapter& other)
  {
    prePhysics_ = other.prePhysics_;
    updatePtrs();
    return *this;
  }

  PhysicsAdapter& operator=(PhysicsAdapter&& other)
  {
    prePhysics_ = other.prePhysics_;
    updatePtrs();
    other.updatePtrs();
    return *this;
  }
  
  ~PhysicsAdapter() {};

  const std::vector<const Force*>& forcePtrs() const override { return forcePtrs_; }
  const std::vector<const Collidable*>& collidablePtrs() const override { return collidablePtrs_; }
  const Damping* dampingPtr() const override { return prePhysics_.dampingPtr(); }

  /**
   * Returns a non-const reference to the adaptee.
   * 
   * Warning: The user of this class is responsible for calling the updatePtrs() function each time 
   * the adaptee is modifed in a way that changes the pointers returned by its createForceVector()
   * and createCollidableVector() functions. 
   */
  PrePhysicsType& prePhysics() { return prePhysics_; }
  void updatePtrs()
  {
    forcePtrs_ = prePhysics_.createForceVector();
    collidablePtrs_ = prePhysics_.createCollidableVector();
  }

private:
  
  PrePhysicsType prePhysics_;
  std::vector<const Force*> forcePtrs_;
  std::vector<const Collidable*> collidablePtrs_;
};

} // end namespace sph 

#endif