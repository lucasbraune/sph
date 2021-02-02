#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include "util.hpp"

namespace sph {

struct Particle final {
  Vec2d pos, vel = {}, acc = {};
  double density = 0.0;
};

struct ParticleSystem final {
  ParticleSystem(const std::vector<Particle>& particles, double totalMass);
  std::vector<Particle> particles;
  double particleMass;
  double time;
};

std::vector<Particle> randomParticles(const Rectangle& region, size_t numberOfParticles);

struct Physics {
  virtual ~Physics() {}
  // Adds the effect of velocity-independent forces to the accelerations of particles
  virtual void applyForces(ParticleSystem& ps) = 0;
  // Adds the effect of damping to the accelerations of particles
  virtual void applyDamping(ParticleSystem& ps) const = 0;
  // Moves particles to resolve collisions
  virtual void resolveCollisions(ParticleSystem& ps) const = 0;
};

struct Force {
  virtual ~Force() {}
  virtual void apply(ParticleSystem& ps) = 0;
};

struct Damping {
  virtual ~Damping() {}
  virtual void apply(ParticleSystem& ps) const = 0;
};

struct Collidable {
  virtual ~Collidable() {}
  virtual void resolveCollisions(ParticleSystem& ps) const = 0;
};

class PointGravity : public Force {
public:
  PointGravity(double gravityConstant, const Vec2d& center = {});
  void apply(ParticleSystem& ps) final;
  double constant() const { return intensity_; }
  void setConstant(double intensity) { intensity_ = intensity; }

private:
  Vec2d center_;
  double intensity_;
};

class SurfaceGravity : public Force {
public:
  SurfaceGravity(double magnitude);
  void apply(ParticleSystem& ps) final;
  double magnitude() const { return -acceleration_[1]; }
  void setMagnitude(double newValue) { acceleration_[1] = -newValue; }
  
private:
  Vec2d acceleration_;
};

class LinearDamping : public Damping {
public:
  LinearDamping(double dampingConstant);
  void apply(ParticleSystem& ps) const final;
  double constant() const { return intensity_; }
  void setConstant(double newValue) { intensity_ = newValue; }

private:
  double intensity_; // force per unit velocity
};

class Wall : public Collidable {
public:
  Wall(const Vec2d& normal, // must be nonzero
       double distanceFromTheOrigin);
  void resolveCollisions(ParticleSystem& ps) const final;
  void move(const Vec2d& displacement) { ptOnWall_ += displacement; }

private:
  void resolveCollision(Vec2d& pos, Vec2d& vel) const;
  Vec2d unitNormal_; // points out of the wall
  Vec2d ptOnWall_; 
};

} // namespace sph

#endif