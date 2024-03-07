#ifndef GLOBAL_VARS_HPP
#define GLOBAL_VARS_HPP

#include "robot.hpp"
#include "berth.hpp"
#include "boat.hpp"

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;

Robot robot[robot_num];
Berth berth[berth_num];
Boat boat[boat_num];

int money, boat_capacity, id;
char map[N][N];
int goods[N][N];
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};

#endif // GLOBAL_VARS_HPP
