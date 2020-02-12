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
#include <sqlite3.h>
#include <stdio.h>

#define SQL_ERROR(err_msg){\
            fprintf(stderr, "Something went terribly wrong- SQL error: %s\n", err_msg);\
            sqlite3_free(err_msg);\
            return 0;\
}

using namespace std;

string player_name;
string player_id_str;
int player_id;
int player_score;
int player_level;

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

static int do_nothing_callback(void *none, int num_of_values, char **values, char **keys) {
    return 1;
}

static int set_player_id(void *none, int num_of_values, char **values, char **keys) {
    //Get last-id in "games" table as char*, convert it to string and then to int.
    stringstream last_id_str;
    unsigned int last_id;

    last_id_str << values[0];
    last_id_str >> last_id;

    // Set_player_id
    player_id = last_id + 1;
    player_id_str = std::to_string(player_id);
    return 0;
}

static int set_existing_player(void *notUsed, int num_of_values, char **values, char **keys) {
    for(int i = 0; i<num_of_values; i++) {
        if(!std::strcmp(keys[i],"username")) {
            player_name = values[i];
        }else{
            stringstream val_str;
            unsigned int val_number;
            val_str << values[i];
            val_str >> val_number;
            if (!std::strcmp(keys[i], "id")) {
                player_id = val_number;
            } else if (!std::strcmp(keys[i], "score")) {
                player_score = val_number;
            } else if (!std::strcmp(keys[i], "level")) {
                player_level = val_number;
            }
        }
    }
    return 1;
}

int add_new_player(sqlite3 *db, string name) {
    player_name = name;

    char *zErrMsg = 0;
    int rc;

    char max_id_query[] = "SELECT MAX(id) FROM games";
    rc = sqlite3_exec(db, max_id_query, set_player_id, 0, &zErrMsg);
    if (rc == SQLITE_OK) { // Player_id loaded successfully!
        string insert_query = "INSERT INTO games (id,username,score,level) VALUES (" + player_id_str + ", '" + player_name + "', 0, 0);";
        rc = sqlite3_exec(db, insert_query.c_str(), do_nothing_callback, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
            SQL_ERROR(zErrMsg); // Player was not added to the DB.
        }
    } else {
        SQL_ERROR(zErrMsg); // Player_id wasn't loaded.
    }
    return 1;
}

int get_existing_player(sqlite3 *db, string name) {
    char *zErrMsg = 0;
    int rc = 0;
    string get_user_details = "SELECT * FROM games WHERE username = '" + name + "'";
    rc = sqlite3_exec(db, get_user_details.c_str(), set_existing_player, 0, &zErrMsg);
    if (!rc) {
        return add_new_player(db, name);
    }else{
        return 1;
    }
}




int set_db() {
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;
    rc = sqlite3_open("game.db", &db);

    if (rc) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        return 0;
    } else {
        string create_games_table = "CREATE TABLE IF NOT EXISTS games("  \
                                      "id INTEGER PRIMARY KEY     NOT NULL," \
                                      "username           TEXT    NOT NULL," \
                                      "score              INTEGER NOT NULL," \
                                      "level              INTEGER NOT NULL);";

        rc = sqlite3_exec(db, create_games_table.c_str(), do_nothing_callback, 0, &zErrMsg);

        if (rc == SQLITE_OK) {
            string ans;
            cout<< "Enter Username - ";
            getline(cin, ans);
            int success =  get_existing_player(db, ans);
            if (!success) {
                return 0;
            }
        } else {
            SQL_ERROR(zErrMsg); // Table was not created.
        }
    }
    sqlite3_close(db);

    return 1;
}

int main(int argc, char *argv[]) {

    int success = set_db();
    if(!success)
        return 1;

    srand(static_cast <unsigned> (time(0)));
    std::ifstream input("configuration.txt");

    Display *disp = new Display(1700, 850, "Wellcome");
    Renderer renderer;

    igl::opengl::glfw::Viewer viewer = load_meshes_from_conf(&renderer);

    Init(*disp);
    renderer.init(&viewer);

    // Add another point of view screen
    renderer.core().viewport = Eigen::Vector4f(0, 0, 850, 850);
    renderer.append_core(Eigen::Vector4f(850, 0, 850, 850));

    disp->SetRenderer(&renderer);

    disp->launch_rendering(true);

    delete disp;
}

