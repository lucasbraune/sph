#ifndef FORCES_HPP
#define FORCES_HPP

#include "util.hpp"
#include "particle_system.hpp"

using std::vector;

class PointGravity : public Force {
public:
  PointGravity(const Vec2d center, const double gravityConstant);
  void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
             vector<Vec2d>& accelerations) const;
  void setConstant(double intensity);
  void increase();
  void decrease();
  
private:
  Vec2d center_;
  double intensity_;
};

class LinearDamping : public Damping {
public:
  LinearDamping(const double dampingConstant);
  Vec2d acceleration(const double time, const double mass, const Vec2d velocity) const;
  void increase();
  void decrease();

private:
  double intensity_;
};


#endif