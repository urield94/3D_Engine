#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdlib>

using namespace std;

igl::opengl::glfw::Viewer load_meshes_from_conf() {
    igl::opengl::glfw::Viewer viewer;;
    viewer.links_number = 10;
    viewer.snake_scale_factor = 0.3;
    viewer.snake_length = 1.6 * viewer.links_number * viewer.snake_scale_factor;
    fstream newfile;
    newfile.open("configuration.txt", ios::in);
    if (newfile.is_open()) {
        string line;
        int i = 0;
        while (getline(newfile, line)) {
            if(i==0) {
                for(int j=0; j<viewer.links_number; j++) viewer.load_mesh_from_file(line);
            } else {
                viewer.load_mesh_from_file(line);
                float angle = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                if(i < viewer.links_number + 4) {
                    viewer.data().MyRotate(Eigen::Vector3f(-(rand() % (3)),
                                                           (rand() % (3)),
                                                           -(rand() % (3))), angle);
                }else if(i < viewer.links_number + 8) {
                    viewer.data().MyRotate(Eigen::Vector3f((rand() % (3)),
                                                           -(rand() % (3)),
                                                           (rand() % (3))), -angle);
                }else if(i < viewer.links_number + 12){
                    viewer.data().MyRotate(Eigen::Vector3f((rand() % (3)),
                                                           (rand() % (3)),
                                                           -(rand() % (3))), -angle);
                }
                else{
                    viewer.data().MyRotate(Eigen::Vector3f((rand() % (3)),
                                                           (rand() % (3)),
                                                           (rand() % (3))), angle);
                }
            }
            i++;
        }
        newfile.close();
    }
    return viewer;
}

int main(int argc, char *argv[]) {
    srand (static_cast <unsigned> (time(0)));
    std::ifstream input("configuration.txt");

    Display *disp = new Display(1800, 950, "Wellcome");
    Renderer renderer;

    igl::opengl::glfw::Viewer viewer = load_meshes_from_conf();

    Init(*disp);
    renderer.init(&viewer);

    // Add another point of view screen
    renderer.core().viewport = Eigen::Vector4f(0, 0, 900, 950);
    renderer.append_core(Eigen::Vector4f(900, 0, 900, 950));

    disp->SetRenderer(&renderer);
    disp->launch_rendering(true);

    delete disp;
}