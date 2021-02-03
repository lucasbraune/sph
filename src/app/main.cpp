#define GL_SILENCE_DEPRECATION

#include <string>
#include <GL/freeglut.h>
#include "sample_simulations.hpp"
#include "view_controller.hpp"

using namespace sph;

void createGlutWindow(const Rectangle region, int verticalPixels, const std::string title);
void display();
void idle();
void keyboard(unsigned char key, int x, int y);

template<class Simulation>
size_t numberOfParticles(const Simulation& sim) { return sim.state().ps.particles.size(); }

// auto simulation = createToyStarSimulation();
auto simulation = createBreakingDamSimulation();
const auto view = View{Rectangle{-1, -1, 1, 1}, numberOfParticles(simulation)};

int main()
{
  createGlutWindow(view.region, 750, "Fluid simulation");
  // Register callback functions
  glutDisplayFunc(display);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  // Pass control to glut
  glutMainLoop();
  return 0;
}

void glutInit()
{
  int argc = 0;
  char** argv = new char*[0];
  glutInit(&argc, argv);
  delete[] argv;
}

void createGlutWindow(const Rectangle region, int verticalPixels, const std::string title)
{
  glutInit();
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  const auto horizontalPixels = static_cast<int>(verticalPixels * (width(region) / height(region)));
  glutInitWindowSize(horizontalPixels, verticalPixels);
  glutCreateWindow(title.c_str());
  gluOrtho2D(region.xmin, region.xmax, region.ymin, region.ymax);
}

void display()
{
  view.draw(simulation.state());
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