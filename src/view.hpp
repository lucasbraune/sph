#ifndef VIEW_HPP
#define VIEW_HPP

#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "util.hpp"

class View {
public:
  View(const Rectangle region, const double particleRadius);
  void draw(const vector<Vec2d>& positions) const;
  Rectangle region() const;

private:
  void drawDisk(const Vec2d center, const double radius) const;

  const Rectangle region_;
  const double particleRadius_;
};



#endif