#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "sample_simulations.hpp"
#include "view.hpp"
#include "controller.hpp"

using namespace sph;

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

// auto simulation = createWallBouncingSimulation();
// auto simulation = createToyStarSimulation();
// auto simulation = createWellSimulation();
auto simulation = createBreakingDamSimulation();
const auto view = View{simulation};

int main(int argc, char** argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(view.windowWidth(), view.windowHeight());
  glutCreateWindow(view.title());

  gluOrtho2D(view.region().xmin, view.region().xmax,
             view.region().ymin, view.region().ymax);

  glutDisplayFunc(display);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);

  simulation.togglePause(); // Unpauses the simulation
  glutMainLoop();
  return 0;
}

void display()
{
  view.draw();  
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
