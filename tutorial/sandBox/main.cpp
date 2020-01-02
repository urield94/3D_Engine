#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <iostream>
#include <fstream>
#include <string>


using namespace std;

igl::opengl::glfw::Viewer load_meshes_from_conf() {
    igl::opengl::glfw::Viewer viewer;;
    fstream newfile;
    newfile.open("configuration.txt", ios::in);
    if (newfile.is_open()) {
        string line;
        while (getline(newfile, line)) {
            viewer.load_mesh_from_file(line);
        }
        newfile.close();
    }
    return viewer;
}

int main(int argc, char *argv[]) {

    std::ifstream input("configuration.txt");

    Display *disp = new Display(1000, 800, "Wellcome");
    Renderer renderer;

    igl::opengl::glfw::Viewer viewer = load_meshes_from_conf();

    Init(*disp);
    renderer.init(&viewer);
    disp->SetRenderer(&renderer);
    disp->launch_rendering(true);

    delete disp;
}
