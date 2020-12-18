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
    updatePtrs(*this);
  }

  PhysicsAdapter(const PhysicsAdapter& other) :
    prePhysics_{other.prePhysics_}
  {
    updatePtrs(*this);
  }

  PhysicsAdapter(PhysicsAdapter&& other) :
    prePhysics_{other.prePhysics_}
  {
    updatePtrs(*this);
    updatePtrs(other);
  }

  PhysicsAdapter& operator=(const PhysicsAdapter& other)
  {
    prePhysics_ = other.prePhysics_;
    updatePtrs(*this);
    return *this;
  }

  PhysicsAdapter& operator=(PhysicsAdapter&& other)
  {
    prePhysics_ = other.prePhysics_;
    updatePtrs(*this);
    updatePtrs(other);
    return *this;
  }
  
  ~PhysicsAdapter() {};

  const std::vector<const Force*>& forcePtrs() const override { return forcePtrs_; }
  const std::vector<const Collidable*>& collidablePtrs() const override { return collidablePtrs_; }
  const Damping* dampingPtr() const override { return prePhysics_.dampingPtr(); }

  /**
   * Return a non-const reference to the PrePhysics object adapted by this adaptor. Warning: this 
   * adapter should be discarded if the attending PrePhysics object is modified so that the 
   * pointers returned by its member function createForceVector() are different from those returned
   * by this adapter's forcePtrs() function.
   */
  PrePhysicsType& prePhysics() { return prePhysics_; }

private:
  static void updatePtrs(PhysicsAdapter& adapter)
  {
    adapter.forcePtrs_ = adapter.prePhysics_.createForceVector();
    adapter.collidablePtrs_ = adapter.prePhysics_.createCollidableVector();
  }
  PrePhysicsType prePhysics_;
  std::vector<const Force*> forcePtrs_;
  std::vector<const Collidable*> collidablePtrs_;
};

} // end namespace sph 

#endif