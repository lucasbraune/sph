#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include "util.hpp"

using std::vector;

class ParticleSystem {
public:
  ParticleSystem(const vector<Vec2d>& positions);
  const vector<Vec2d>& positions() const;
private:
  vector<Vec2d> positions_;
};

#endif