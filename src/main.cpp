#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "util.hpp"
#include "view.hpp"
#include "demo.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

ToyStar simulation;

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

  simulation.pauseOrUnpause(); // Unpauses the simulation
  glutMainLoop();
  return 0;
}

void display()
{
  view.draw(simulation);  
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
  switch (c) {
  // Central gravity controls
  case 'd':
    simulation.dampingAdjuster().decrease();
    break;
  case 'D':
    simulation.dampingAdjuster().increase();
    break;
  case 'g':
    simulation.gravityAdjuster().decrease();
    break;
  case 'G':
    simulation.gravityAdjuster().increase();
    break;

  // Runner controls
  case 'p':
  case 'P':
    simulation.pauseOrUnpause();
    break;
  case 's':
    simulation.speedAdjuster().decrease();
    break;
  case 'S':
    simulation.speedAdjuster().increase();
    break;
  }
}
