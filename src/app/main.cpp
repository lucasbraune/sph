#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "sample_simulations.hpp"
#include "view_controller.hpp"

using namespace sph;

void createGlutWindow(const Rectangle& region, int verticalPixels, const char* title);
void display();
void idle();
void keyboard(unsigned char key, int x, int y);

auto simulation = createSimulation(ToyStarParameters{});
// auto simulation = createSimulation(BreakingDamParameters{});

int main()
{
  createGlutWindow(simulation.region(), 750, "Fluid simulation");
  // Register callback functions
  glutDisplayFunc(display);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  // Pass control to glut
  glutMainLoop();
  return 0;
}

int horizontalPixels(int verticalPixels, const Rectangle& region)
{
  auto result = verticalPixels * (width(region) / height(region));
  return static_cast<int>(result);
}

void createGlutWindow(const Rectangle& region, int verticalPixels, const char* title)
{
  int argc = 0;
  glutInit(&argc, nullptr);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(horizontalPixels(verticalPixels, region), verticalPixels);
  glutCreateWindow(title);
  gluOrtho2D(region.xmin, region.xmax, region.ymin, region.ymax);
}

void display()
{
  static auto draw = DrawFunction{simulation.region(), simulation.numberOfParticles()};
  draw(simulation.state());
  glutSwapBuffers();
}

void idle()
{
  simulation.computeNextFrame();
  simulation.waitForNextFrame();
  glutPostRedisplay();
}

void keyboard(unsigned char c, int, int)
{
  handleKeyboardInput(simulation, c);
}