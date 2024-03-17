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
const int distExRecoverBot = 25; // 将处于恢复状态的机器人排除在路径搜索范围的最小距离
const int GoodsValueMax = 200;
extern Robot robot[robot_num];
extern Berth berth[berth_num];
extern Boat boat[boat_num];

extern int money, boat_capacity, id, frame;
extern char map[N][N];
extern int goodsInMap[N][N];
extern int goodsLeftTime[N][N]; // 记录货物的剩余存在时间
extern int nearBerthDis[N][N];  // 记录每个点到最近的泊位的距离
extern int dx[4];
extern int dy[4];
extern vector<std::vector<std::pair<int, int>>> goodsState; // 记录第i帧时的货物位置信息
extern int goods_num;                                       // 分析用，地图生成的货物总量
extern int pick_goods_num;                                  // 分析用，机器人捡起的货物总量
extern int ship_goods_num;                                  // 分析用，船运走的货物总量

#endif // GLOBAL_VARS_H
