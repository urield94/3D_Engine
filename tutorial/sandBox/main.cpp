
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char *argv[])
{
  if(argc < 2){
    std::cout << "Please specify path to meshe files (as many as you wish).\nUsage - sandBox.exe full_path_to_mesh_1 full_path_to_mesh_2 .." << std::endl;
    return  0;
  }

  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;

  for(int i = 1; i < argc; i++){
    viewer.load_mesh_from_file(argv[i]);
  }

  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
