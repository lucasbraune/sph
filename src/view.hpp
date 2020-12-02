#ifndef VIEW_HPP
#define VIEW_HPP

#define GL_SILENCE_DEPRECATION

#include <string>
#include <GL/freeglut.h>
#include "util.hpp"
#include "simulation.hpp"

using std::string;

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
  View(const Rectangle region, const int windowHeight, const string title,
       const vector<Vec2d> particlePolygon);
  View(const Rectangle region, const double particleRadius, const size_t sides = 10,
       const int windowHeight = 750, const string title = "Fluid simulation");
  void draw(const SimulationRunner& sim) const;
  Rectangle region() const;
  int windowHeight() const;
  int windowWidth() const;
  const char* title() const;

private:
  const Rectangle region_;
  const vector<Vec2d> particlePolygon_;
  const int heightInPixels_;  // window height in pixels
  const string title_;        // window title 
};

#endif