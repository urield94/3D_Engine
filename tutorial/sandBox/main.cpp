
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  //viewer.load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj");
  viewer.load_mesh_from_file("/home/urield/study/201/3D/3D_Engine/tutorial/data/cube.obj");
  viewer.load_mesh_from_file("/home/urield/study/201/3D/3D_Engine/tutorial/data/bunny.off");
  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
