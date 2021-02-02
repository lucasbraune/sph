#ifndef VIEW_HPP
#define VIEW_HPP

#include <string>
#include "simulation.hpp"

/**
 * A regular polygon inscribed in a circle.
 */
class RegularPolygon {
public:
  RegularPolygon(sph::Vec2d center, double radius, size_t sides);
  RegularPolygon(double radius, size_t sides);
  const std::vector<sph::Vec2d>& vertices() const;

private:
  std::vector<sph::Vec2d> vertices_;
};

class View {
public:
  View(const sph::Rectangle region,
       const int windowHeight, const std::string title,
       const std::vector<sph::Vec2d> particlePolygon);
  View(const sph::Rectangle region = {-1.0, -1.0, 1.0, 1.0},
       const double particleRadius = 0.015,
       const size_t sides = 10,
       const int windowHeight = 750,
       const std::string title = "Fluid simulation");
  void draw(const sph::SimulationState& state) const;
  sph::Rectangle region() const { return region_; }
  int windowHeight() const { return heightInPixels_; }
  int windowWidth() const;
  const char* title() const { return title_.c_str(); }

private:
  const sph::Rectangle region_;
  const std::vector<sph::Vec2d> particlePolygon_;
  const int heightInPixels_;        // window height in pixels
  const std::string title_;         // window title 
};

#endif