#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include "util.hpp"

namespace sph {

struct Particle final {
  Vec2d pos, vel = {}, acc = {};
  double density = 0.0;
};

struct ParticleSystem final {
  std::vector<Particle> particles;
  double particleMass;
  double time = 0.0;
};

ParticleSystem createParticleSystem(size_t numberOfParticles, double totalMass,
                                    const Rectangle& region);

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

class PointGravity final : public Force {
public:
  PointGravity(double gravityConstant, const Vec2d& center = {});
  void apply(ParticleSystem& ps) override;
  double constant() const { return intensity_; }
  void setConstant(double intensity) { intensity_ = intensity; }

private:
  Vec2d center_;
  double intensity_;
};

class SurfaceGravity final : public Force {
public:
  SurfaceGravity(double magnitude);
  void apply(ParticleSystem& ps) override;
  double magnitude() const { return -acceleration_[1]; }
  void setMagnitude(double newValue) { acceleration_[1] = -newValue; }
  
private:
  Vec2d acceleration_;
};

class LinearDamping final : public Damping {
public:
  LinearDamping(double dampingConstant);
  void apply(ParticleSystem& ps) const override;
  double constant() const { return intensity_; }
  void setConstant(double newValue) { intensity_ = newValue; }

private:
  double intensity_; // force per unit velocity
};

class Wall final : public Collidable {
public:
  Wall(const Vec2d& normal, const Vec2d& ptOnWall);
  void resolveCollisions(ParticleSystem& ps) const override;
  void move(const Vec2d& displacement) { ptOnWall_ += displacement; }

private:
  void resolveCollision(Vec2d& pos, Vec2d& vel) const;
  Vec2d unitNormal_; // points out of the wall
  Vec2d ptOnWall_; 
};

} // namespace sph

#endif