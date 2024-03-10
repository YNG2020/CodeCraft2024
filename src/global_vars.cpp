#include "global_vars.h"

Robot robot[robot_num];
Berth berth[berth_num];
Boat boat[boat_num];

int money, boat_capacity, id;
char map[N][N];
int goods[N][N];
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};
