#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "util.hpp"
#include "view.hpp"
#include "central_gravity.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

CentralGravity simulation;

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

  simulation.switchPauseState(); // Unpauses the simulation
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
  case 'p':
  case 'P':
    simulation.switchPauseState();
    break;
  case 's':
    simulation.speedDown();
    break;
  case 'S':
    simulation.speedUp();
    break;
  }
}
