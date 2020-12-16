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
  RegularPolygon(sph::Vec2d center, double radius, size_t sides);
  RegularPolygon(double radius, size_t sides);
  const vector<sph::Vec2d>& vertices() const;

private:
  vector<sph::Vec2d> vertices_;
};

class View {
public:
  View(const sph::SimulationInterface& simulation, const sph::Rectangle region, const int windowHeight,
       const string title, const vector<sph::Vec2d> particlePolygon);
  View(const sph::SimulationInterface& simulation,
       const sph::Rectangle region = {-1.0, -1.0, 1.0, 1.0},
       const double particleRadius = 0.02,
       const size_t sides = 10,
       const int windowHeight = 750,
       const string title = "Fluid simulation");
  void draw() const;
  sph::Rectangle region() const;
  int windowHeight() const;
  int windowWidth() const;
  const char* title() const;

private:
  const sph::SimulationInterface& simulation_;
  const sph::Rectangle region_;
  const vector<sph::Vec2d> particlePolygon_;
  const int heightInPixels_;  // window height in pixels
  const string title_;        // window title 
};

#endif