#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <functional>

#include "DB.h"


using namespace std;



igl::opengl::glfw::Viewer load_meshes_from_conf(Renderer *rndr) {
    igl::opengl::glfw::Viewer viewer;;
    viewer.links_number = 12;
    viewer.snake_scale_factor = 0.5;
    viewer.snake_length = 1.6 * viewer.links_number * viewer.snake_scale_factor;
    fstream newfile;
    newfile.open("configuration.txt", ios::in);
    if (newfile.is_open()) {
        string line;
        int i = 0;
        while (getline(newfile, line)) {
            if (i == 0) {
                for (int j = 0; j < viewer.links_number; j++) {
                    viewer.load_mesh_from_file(line);
                    Eigen::MatrixXd V(viewer.data_list[j].V.rows(), viewer.data_list[j].V.cols());
                    for (int p = 0; p < viewer.data_list[j].V.rows(); p++) {
                        V(p, 0) = viewer.data_list[j].V(p, 0);

                        V(p, 1) = viewer.data_list[j].V(p, 1) * viewer.snake_scale_factor;

                        V(p, 2) = viewer.data_list[j].V(p, 2);
                    }
                    viewer.data_list[j].set_mesh(V, viewer.data_list[j].F);
                }
                i++;
            } else {
                rndr->object_paths.push_back(line);
            }
        }
        newfile.close();
    }
    return viewer;
}



int main(int argc, char *argv[]) {
    DB db = DB();
    Player player = db.GetPlayerFromDB();
    if(!player.success)
        return 1;

    srand(static_cast <unsigned> (time(0)));
    std::ifstream input("configuration.txt");

    Display *disp = new Display(1700, 850, "Wellcome");
    Renderer renderer;

    igl::opengl::glfw::Viewer viewer = load_meshes_from_conf(&renderer);

    Init(*disp);
    renderer.Init(&viewer, player.score, player.level, player.id, db);

    // Add another point of view screen
    renderer.core().viewport = Eigen::Vector4f(0, 0, 850, 850);
    renderer.append_core(Eigen::Vector4f(850, 0, 850, 850));

    disp->SetRenderer(&renderer);

    disp->launch_rendering(true);

    delete disp;
}

