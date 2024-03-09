#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include "robot.h"
#include "berth.h"
#include "boat.h"

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;

extern Robot robot[robot_num];
extern Berth berth[berth_num];
extern Boat boat[boat_num];

extern int money, boat_capacity, id;
extern char map[N][N];
extern int goods[N][N];
extern int dx[4];
extern int dy[4];

#endif // GLOBAL_VARS_H
