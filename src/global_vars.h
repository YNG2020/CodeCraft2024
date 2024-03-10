#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include "robot.h"
#include "berth.h"
#include "boat.h"
#include <vector>
#include <utility> // for std::pair

const int mapSize = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;

extern Robot robot[robot_num];
extern Berth berth[berth_num];
extern Boat boat[boat_num];

extern int money, boat_capacity, id, frame;
extern char map[N][N];
extern int goodsInMap[N][N];
extern int dx[4];
extern int dy[4];
extern vector<std::vector<std::pair<int, int>>> goodsState;	// 记录第i帧时的货物位置信息

#endif // GLOBAL_VARS_H
