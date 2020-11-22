#ifndef VIEW_HPP
#define VIEW_HPP

#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "util.hpp"
#include "particle_system.hpp"

/**
 * A regular polygon inscribed in a circle.
 */
class RegularPolygon {
public:
  RegularPolygon(Vec2d center, double radius, size_t sides);
  RegularPolygon(double radius, size_t sides);
  const vector<Vec2d>& vertices() const;

private:
  vector<Vec2d> vertices_;
};

class View {
public:
  View(const Rectangle region, const vector<Vec2d> particlePolygon);
  View(const Rectangle region, const double radius, const size_t sides);
  void draw(const ParticleSystem& positions) const;
  Rectangle region() const;

private:
  const Rectangle region_;
  const vector<Vec2d> particlePolygon_;
};

#endif