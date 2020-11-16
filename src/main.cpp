#include <GL/freeglut.h>
#include <iostream>

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

struct Rectangle {
  const double xmin, ymin, xmax, ymax;
};

constexpr int WINDOW_SIZE = 800;
constexpr char WINDOW_TITLE[] = "Fluid simulation";
const Rectangle region {0.0, 0.0, 1.0, 1.0};

int main(int argc, char** argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
  glutCreateWindow(WINDOW_TITLE);

  gluOrtho2D(region.xmin, region.xmax, region.ymin, region.ymax);

  glutDisplayFunc(display);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  glutMainLoop();
  return 0;
}

void display() {}

void idle() {}

void keyboard(unsigned char, int, int) {}
