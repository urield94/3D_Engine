#include "DB.h"
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream>

#define SQL_ERROR(err_msg){\
            fprintf(stderr, "Something went terribly wrong- SQL error: %s\n", err_msg);\
            sqlite3_free(err_msg);\
            return 0;\
}


int player_score;
int player_id;
int player_level;
std::string player_id_str;


DB::DB(){
}

DB::~DB() {
}

int DB::DoNothing_Callback(void *none, int num_of_values, char **values, char **keys) {
    return 1;
}


int DB::SetPlayerId_Callback(void *none, int num_of_values, char **values, char **keys) {
    //Get last-id in "games" table as char*, convert it to string and then to int.
    std::stringstream last_id_str;
    unsigned int last_id;

    last_id_str << values[0];
    last_id_str >> last_id;

    // Set_player_id
    player_id = last_id + 1;
    player_id_str = std::to_string(player_id);
    return 0;
}


int DB::SetExistingPlayer_Callback(void *notUsed, int num_of_values, char **values, char **keys) {
    for(int i = 0; i<num_of_values; i++) {
        if(!strcmp(keys[i],"username")) {
            continue;
        }else{
            std::stringstream val_str;
            unsigned int val_number;
            val_str << values[i];
            val_str >> val_number;
            if (!strcmp(keys[i], "id")) {
                player_id = val_number;
            } else if (!strcmp(keys[i], "score")) {
                player_score = val_number;
            } else if (!strcmp(keys[i], "level")) {
                player_level = val_number;
            }
        }
    }
    return 1;
}


int DB::AddNewPlayer(sqlite3 *db) {
    char max_id_query[] = "SELECT MAX(id) FROM games";
    rc = sqlite3_exec(db, max_id_query, SetPlayerId_Callback, 0, &zErrMsg);
    if (rc == SQLITE_OK) { // Player_id loaded successfully!
        player.id = player_id;
        player.id_str = player_id_str;
        std::string insert_query = "INSERT INTO games (id,username,score,level) VALUES (" + player.id_str + ", '" + player.name + "', 0, 0);";
        rc = sqlite3_exec(db, insert_query.c_str(), DoNothing_Callback, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
            SQL_ERROR(zErrMsg); // Player was not added to the DB.
        }
    } else {
        SQL_ERROR(zErrMsg); // Player_id wasn't loaded.
    }
    return 1;
}


int DB::SetPlayer(sqlite3 *db, std::string name) {
    std::string get_user_details = "SELECT * FROM games WHERE username = '" + name + "'";
    rc = sqlite3_exec(db, get_user_details.c_str(), SetExistingPlayer_Callback, 0, &zErrMsg);
    player.name = name;
    if (!rc) {
        player.score = 0;
        player.level = 0;
        return AddNewPlayer(db);
    }else{
        player.id = player_id;
        player.score = player_score;
        player.level = player_level;
        return 1;
    }
}


Player DB::GetPlayerFromDB() {

    rc = sqlite3_open(db_name.c_str(), &db);

    if (rc) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        player.success = 0;
        return player;
    } else {
        std::string create_games_table = "CREATE TABLE IF NOT EXISTS games("  \
                                      "id INTEGER PRIMARY KEY     NOT NULL," \
                                      "username           TEXT    NOT NULL," \
                                      "score              INTEGER NOT NULL," \
                                      "level              INTEGER NOT NULL);";

        rc = sqlite3_exec(db, create_games_table.c_str(), DoNothing_Callback, 0, &zErrMsg);

        if (rc == SQLITE_OK) {
            std::string ans;
            std::cout << "Enter Username - ";
            getline(std::cin, ans);
            player.success = SetPlayer(db, ans);
        } else {
            fprintf(stderr, "Something went terribly wrong- SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
            player.success = 0; // Table was not created.
        }
    }
    sqlite3_close(db);

    return player;
}


int DB::SetLevel(int player_id, int score, int level) {
    rc = sqlite3_open(db_name.c_str(), &db);
    player.id = score;
    player.level = level;
    std::string update_score_and_level = "UPDATE games " \
                                     "SET score = " + std::to_string(score) + \
                                     ", level = " + std::to_string(level) + \
                                     " WHERE id = " + std::to_string(player_id);

    rc = sqlite3_exec(db, update_score_and_level.c_str(), DoNothing_Callback, 0, &zErrMsg);

    if (rc != SQLITE_OK) {
        SQL_ERROR(zErrMsg); // Level and score didn't updated.
    }
    sqlite3_close(db);

    return 1;
}





