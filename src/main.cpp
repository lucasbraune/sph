#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "util.hpp"
#include "view.hpp"
#include "central_gravity.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

CentralGravity centralGravity;

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

  centralGravity.simulation().switchPauseState(); // Unpauses the simulation
  glutMainLoop();
  return 0;
}

void display()
{
  view.draw(centralGravity.simulation());  
  glutSwapBuffers();
}

void idle()
{
  centralGravity.simulation().computeNextFrame();
  centralGravity.simulation().waitForNextFrame();
  glutPostRedisplay();
}

void keyboard(unsigned char c, int, int)
{
  switch (c) {
  // Central gravity controls
  case 'd':
    centralGravity.damping().decrease();
    break;
  case 'D':
    centralGravity.damping().increase();
    break;
  case 'g':
    centralGravity.gravity().decrease();
    break;
  case 'G':
    centralGravity.gravity().increase();
    break;

  // Simulation controls
  case 'p':
  case 'P':
    centralGravity.simulation().switchPauseState();
    break;
  case 's':
    centralGravity.simulation().speedDown();
    break;
  case 'S':
    centralGravity.simulation().speedUp();
    break;
  }
}
