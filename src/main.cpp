#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "util.hpp"
#include "view.hpp"
#include "central_gravity.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

CentralGravitySimulation simulation;

const Rectangle region{-1.0, -1.0, 1.0, 1.0};
const double particleRadius = 0.02;
const View view(region, particleRadius);

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

  simulation.runner().switchPauseState(); // Unpauses the simulation
  glutMainLoop();
  return 0;
}

void display()
{
  view.draw(simulation.state(), simulation.runner());  
  glutSwapBuffers();
}

void idle()
{
  simulation.runner().computeNextFrame();
  simulation.runner().waitForNextFrame();
  glutPostRedisplay();
}

void keyboard(unsigned char c, int, int)
{
  switch (c) {
  // Central gravity controls
  case 'd':
    simulation.damping().decrease();
    break;
  case 'D':
    simulation.damping().increase();
    break;
  case 'g':
    simulation.gravity().decrease();
    break;
  case 'G':
    simulation.gravity().increase();
    break;

  // Simulation controls
  case 'p':
  case 'P':
    simulation.runner().switchPauseState();
    break;
  case 's':
    simulation.runner().speedDown();
    break;
  case 'S':
    simulation.runner().speedUp();
    break;
  }
}
