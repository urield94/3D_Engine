#ifndef ENGIENIGLNEW_DB_H
#define ENGIENIGLNEW_DB_H

#include <sqlite3.h>
#include <string>

struct Player{
     std::string name;
     std::string id_str;
     int id;
     int score;
     int level;
     int success;
};

class DB {
    public:
        DB();
        ~DB();
        Player GetPlayerFromDB();
        int SetLevel(int player_id, int score, int level);
        Player player;

    private:

        static int DoNothing_Callback(void *none, int num_of_values, char **values, char **keys);
        static int SetPlayerId_Callback(void *none, int num_of_values, char **values, char **keys);
        static int SetExistingPlayer_Callback(void *notUsed, int num_of_values, char **values, char **keys);

        int AddNewPlayer(sqlite3 *db);
        int SetPlayer(sqlite3 *db, std::string name);

        std::string db_name = "game.db";
        sqlite3 *db;
        char *zErrMsg = 0;
        int rc;

};


#endif //ENGIENIGLNEW_DB_H
